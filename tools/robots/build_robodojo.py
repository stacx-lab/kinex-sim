"""Measure the RoboDojo dual ARX X5 rig and write catalog/robots/robodojo/robodojo-arx-x5.

Chains come from the X5 URDF; mounts and cameras are measured in the live Isaac
scene. Runs in the native environment after `kinex-sim prepare robodojo`:
    pixi run --frozen -e robodojo-native env PYTHONPATH=src:simulators/robodojo:.runtime/robodojo \
        python tools/robots/build_robodojo.py
"""
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "simulators/robodojo"), str(Path(__file__).parent)]

import rigs  # noqa: E402
from kinex_sim_robodojo import move  # noqa: E402
from kinex_sim_robodojo.backend import RoboDojoSim, runtime  # noqa: E402

# RoboEnv's parallel-gripper geometry: the pad center ahead of link6, in the flange frame.
TCP = (0.15007, -0.000002, -0.00144)
ARM = "ARX X5 six-joint {side} arm"
# The move configuration: what an arm's move goal does (kinex_sim_robodojo.move).
GOALS = ", moved by flange goals through its own IK"
MOVE = (f"Move the flange in straight lines through poses at the arm's own pace: multi_dof_trajectory with "
        f"joint_names [flange] and one transform per point, in {{side}}_arm/base (metres, unit quaternion xyzw). A "
        f"zero time_from_start moves at up to {move.SPEED[0]:g} m/s and {move.SPEED[1]:g} rad/s; a later one moves "
        f"slower. It succeeds within {move.TOLERANCE[0]:g} m and {move.TOLERANCE[1]:g} rad of the last pose, or "
        f"within component_goal_tolerance (joint_name flange; component 4 metres, 5 radians). It aborts where the "
        f"arm is when the flange stalls, is still short {move.SETTLE_S:g} s (or goal_time_tolerance) after the "
        f"motion should end, or has no IK solution. Result error_code: 0 reached, -4 stalled, -5 late, -1 "
        f"otherwise; error_string says why and how far from the goal. A newer goal or a stop ends it.")


def arm(side, base, goals=False):
    profile = rigs.joint(f"{side}_arm", ARM.format(side=side) + (GOALS if goals else "") + ".",
                         [f"joint{i}" for i in range(1, 7)], [0.0] * 6, base, tip_pose=True, targets=not goals)
    return rigs.action(profile, "move", move.TYPE, MOVE.format(side=side)) if goals else profile


def link(sim, side, name):
    robot = sim._actuators[side][0]
    pose = sim.env.robot_manager.get_link_pose(robot, name, [0])[0]  # x, y, z, qw, qx, qy, qz
    return rigs.transform(pose[:3], rigs.quat_matrix(pose[3:7]))


def camera(sim, name, view, parent, mount):
    """A camera's RGB profile and its RGB-D one in the depth configuration; view names the kind as {}."""
    intrinsics, base = sim.intrinsics(name), rigs.pose_frame(parent, mount)
    rgb = rigs.camera(name, view.format("RGB") + ".", intrinsics, base)
    rgbd = rigs.camera(name, view.format("RGB-D") + ".", intrinsics, base, sim._camera(name).get_clipping_range())
    return rgb, rgbd


def main():
    sim = RoboDojoSim()
    sim.reset(sim.default_task, 0)
    x5 = runtime() / "Assets/Robots/x5"
    source = rigs.urdf_joints(x5 / "X5A.urdf")
    devices = {"world": rigs.world("World time and waits of a bimanual tabletop station.")}
    files, depth, goals = {}, {}, {}
    for side in ("left", "right"):
        robot = sim._actuators[side][0]
        lower, upper = map(float, robot.gripper_scale)
        flange = link(sim, side, "link6")
        base = rigs.pose_frame("world:base", link(sim, side, "base_link"))
        devices[f"{side}_arm"], goals[f"{side}_arm"] = arm(side, base), arm(side, base, goals=True)
        devices[f"{side}_gripper"] = rigs.gripper(
            f"{side}_gripper",
            f"ARX X5 {side} parallel two-finger gripper. A closed target squeezes past the finger stop. The TCP "
            "is the pad centre and approaches along +x.",
            f"{side}_gripper",
            rigs.frame(f"{side}_arm:flange", (0, 0, 0), (0, 0, 0, 1)),
            rigs.frame(f"{side}_gripper:base", TCP, (0, 0, 0, 1)))
        files[f"devices/{side}_arm/arm.urdf"] = rigs.source_arm(
            source, [f"joint{i}" for i in range(1, 7)], f"x5_{side}_arm",
            "RoboDojo ARX X5 arm from Assets/Robots/x5/X5A.urdf: base_link..link6; flange is "
            "the link6 frame.", velocity=sim._limits[side][2] / sim.period)
        files[f"devices/{side}_gripper/gripper.urdf"] = rigs.source_gripper(
            source, f"{side}_gripper", ["joint7", "joint8"], lower, upper, f"x5_{side}_gripper",
            f"RoboDojo ARX X5 {side} parallel gripper on the link6 flange. {side}_gripper is a "
            "geometry-less virtual drive joint carrying the native normalized opening (0 closed, "
            f"1 open); both fingers mimic it as {lower} + {upper - lower} m per unit, upstream's "
            "gripper_scale. Closed targets lie below the finger stop to squeeze an object.")
        position, quaternion = sim.camera_pose(f"{side}_camera")
        optical = rigs.transform(position, rigs.quat_matrix(quaternion))
        devices[f"{side}_camera"], depth[f"{side}_camera"] = camera(
            sim, f"{side}_camera", f"D435 {{}} camera on the {side} wrist", f"{side}_arm:flange",
            rigs.relative(flange, optical))
    position, quaternion = sim.camera_pose("head_camera")
    devices["head_camera"], depth["head_camera"] = camera(
        sim, "head_camera", "Gemini 345Lg {} head camera fixed above the table", "world:base",
        rigs.transform(position, rigs.quat_matrix(quaternion)))
    order = ["world", "left_arm", "left_gripper", "right_arm", "right_gripper",
             "head_camera", "left_camera", "right_camera"]
    rigs.write_rig(ROOT / "catalog/robots/robodojo/robodojo-arx-x5",
                   rigs.workspace("robodojo-arx-x5", "RoboDojo dual ARX X5", "arm", "ARX X5 (dual)"),
                   {name: devices[name] for name in order}, files, {"depth": depth, "move": goals})
    print("rig written", flush=True)
    np.set_printoptions(precision=4)
    sim.close()


if __name__ == "__main__":
    main()
