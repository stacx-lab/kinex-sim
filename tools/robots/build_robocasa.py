"""Measure the RoboCasa PandaOmron rig from MuJoCo and write catalog/robots/robocasa/robocasa-panda-omron.

Run in the RoboCasa environment after `kinex-sim prepare robocasa`:
    pixi run --frozen -e robocasa python tools/robots/build_robocasa.py
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "simulators/robocasa"), str(Path(__file__).parent)]

import rigs  # noqa: E402
from kinex_sim.core.commands import VELOCITY_TIMEOUT_S  # noqa: E402
from kinex_sim_robocasa.backend import OPTICAL, RoboCasaSim  # noqa: E402


def gripper_urdf(model, controller, fingers):
    """The integrated native command drives each finger's position actuator over its range."""
    joints = [{"name": "gripper", "type": "revolute", "parent": "base_link", "child": "drive",
               "axis": (0, 0, 1), "limits": (-1, 1)}]
    # The native command moves finger 1 by -v and finger 2 by +v within [-1, 1].
    for finger, sign, low, high in zip(fingers, (-1, 1), controller.actuator_min,
                                       controller.actuator_max):
        body = model.body(finger).id
        jid = rigs.body_joint(model, body)
        joints.append({"name": model.joint(jid).name, "type": "prismatic", "parent": "base_link",
                       "child": finger.rsplit("_", 1)[1], "origin": rigs.body_local(model, body),
                       "axis": model.jnt_axis[jid], "limits": model.jnt_range[jid],
                       "mimic": ("gripper", sign * (high - low) / 2, (high + low) / 2)})
    return rigs.urdf(
        "panda_hand",
        "RoboCasa Panda hand (robosuite panda_gripper.xml). gripper is a geometry-less virtual "
        "drive joint carrying robosuite's integrated gripper command (-1 open, +1 closed); each "
        "finger joint mimics the position its actuator is commanded to, not a measurement.",
        ["base_link", "drive", "leftfinger", "rightfinger"], joints)


def main():
    sim = RoboCasaSim()
    sim.reset(sim.default_task, 0)
    model, data, robot = sim.model, sim.data, sim.robot
    near, far = (model.vis.map.znear * model.stat.extent, model.vis.map.zfar * model.stat.extent)
    body = {name: model.body(name).id for name in (
        "mobilebase0_support", "robot0_link0", "robot0_right_hand", "gripper0_right_right_gripper",
        "gripper0_right_eef")}
    pose = {name: rigs.body_world(data, bid) for name, bid in body.items()}
    top, flange, hand = pose["mobilebase0_support"], pose["robot0_right_hand"], pose["gripper0_right_right_gripper"]

    def optical(camera):
        cid = model.camera(camera).id
        return rigs.transform(data.cam_xpos[cid], data.cam_xmat[cid].reshape(3, 3) @ OPTICAL)

    low, high = robot.part_controllers["base"].control_limits
    devices = {
        "world": rigs.world("World time and waits of a household kitchen."),
        "base": rigs.locomotion(
            "base",
            "Omron holonomic mobile base. Full-scale velocity is 1 m/s and 1.5 rad/s. Its base frame is the "
            "chassis origin, 21 cm ahead of the yaw axis.",
            dict(zip(("vx", "vy", "wz"), zip(low, high))),
            {"parent": None, "translation_m": None, "rotation_xyzw": None},
            timeout_s=VELOCITY_TIMEOUT_S),
        "torso": rigs.joint(
            "torso",
            "Prismatic lift of the Omron base.",
            ["mobilebase0_joint_torso_height"], robot.init_torso_qpos,
            rigs.frame("base:base", (0, 0, 0), (0, 0, 0, 1)), tip="top", model="torso.urdf"),
        "arm": rigs.joint(
            "arm",
            "Franka Panda seven-joint arm.",
            list(robot.robot_joints), robot.init_qpos,
            rigs.pose_frame("torso:top", rigs.relative(top, pose["robot0_link0"]))),
        "gripper": rigs.gripper(
            "gripper",
            "Panda parallel two-finger gripper. Fingers close along the base x axis. The grip joint is the "
            "applied command, not a measured opening, so fingers stalled on an object do not show in it.",
            "gripper",
            rigs.pose_frame("arm:flange", rigs.relative(flange, hand)),
            rigs.pose_frame("gripper:base", rigs.relative(hand, pose["gripper0_right_eef"])),
            quantity="a normalized command", note="The applied command moves 0.2 toward it every 0.05 s.",
            state="The applied grip command, not a measured opening, stamped at acquisition.",
            opened=-1.0, closed=1.0),
    }
    for name, camera, parent, frame in (
            ("camera_left", "robot0_agentview_left", "torso:top", top),
            ("camera_right", "robot0_agentview_right", "torso:top", top),
            ("camera_wrist", "robot0_eye_in_hand", "arm:flange", flange)):
        side = name.split("_")[1]
        devices[name] = rigs.camera(
            name,
            (f"RGB-D camera on the torso lift, behind and above the robot's {side} side, looking forward and "
             "down past the arm." if side != "wrist" else "RGB-D camera on the hand, looking along the fingers."),
            sim.intrinsics(camera),
            rigs.pose_frame(parent, rigs.relative(frame, optical(camera))), (near, far))
    files = {
        "devices/torso/torso.urdf": rigs.mujoco_chain(
            model, ["mobilebase0_fixed_support", "mobilebase0_support"], "omron_torso",
            "Omron base lift measured from MuJoCo mobilebase0_base..mobilebase0_support; top is "
            "the support body frame, where the arm is mounted.", tip="top"),
        "devices/arm/arm.urdf": rigs.mujoco_chain(
            model, [*(f"robot0_link{i}" for i in range(1, 8)), "robot0_right_hand"], "panda_arm",
            "RoboCasa Panda arm measured from MuJoCo robot0_link0..robot0_right_hand; flange is "
            "the hand body frame."),
        "devices/gripper/gripper.urdf": gripper_urdf(
            model, robot.part_controllers["right_gripper"],
            ("gripper0_right_leftfinger", "gripper0_right_rightfinger")),
    }
    order = ["world", "base", "torso", "arm", "gripper", "camera_left", "camera_right",
             "camera_wrist"]
    rigs.write_rig(ROOT / "catalog/robots/robocasa/robocasa-panda-omron",
                   rigs.workspace("robocasa-panda-omron", "RoboCasa Panda Omron", "mobile",
                                  "Franka Panda on an Omron mobile base"),
                   {name: devices[name] for name in order}, files)
    sim.close()


if __name__ == "__main__":
    main()
