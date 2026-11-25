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
from kinex_sim_robodojo import grip, move  # noqa: E402
from kinex_sim_robodojo.backend import JSON, RoboDojoSim, runtime  # noqa: E402

# RoboEnv's parallel-gripper geometry: the pad center ahead of link6, in the flange frame.
TCP = (0.15007, -0.000002, -0.00144)
ARM = "ARX X5 six-joint {side} arm"
# What the devices' goals do (kinex_sim_robodojo.move and grip), each a Json action's objects. A gear is named with
# its step in cm and its tolerances in m and rad.
GOALS = ", moved by flange goals through its own IK"
GEARS = ", ".join(f"{name} {100 * step:g} cm within {reach:g} m and {turn:g} rad"
                  for name, (step, reach, turn) in move.PROFILES.items())
ENDS = "A newer goal or a stop ends it."
# A step that moves an arm too far ends the goals it concerns (move.unsafe).
UNSAFE = (f"a step moves a flange more than {move.MOVED[0]:g} m or {move.MOVED[1]:g} rad, or that of an arm holding "
          f"still more than {move.HELD[0]:g} m or {move.HELD[1]:g} rad (unsafe_motion)")
MOVE = ("Move the flange in a straight line to a pose: {{pose: [x, y, z, qx, qy, qz, qw] in {side}_arm/base, "
        "profile}}. profile is the gear, the most a {period:g} s step moves the flange and how near the pose it "
        f"has reached it: {GEARS}; standard by default. The goal aborts when the flange stalls, moves away from "
        f"the pose on two steps running (diverging), when {UNSAFE}, when a step has no IK solution (unreachable) or "
        f"{move.SETTLE} steps after the gear would have arrived (late). Before it is unreachable the flange goes, "
        "once, for a pose the arm can reach within the gear's tolerance of the goal's, if there is one. Result: "
        "{{stop_reason: reached | stalled | diverging | unsafe_motion | unreachable | late | ended, pose, "
        "position_error_m, orientation_error_rad, steps, recovered: whether it went for such a pose}}; feedback: "
        f"the errors and steps. {ENDS}")
GRASP = (f"Close the fingers and keep squeezing: {{}}, no fields. The goal succeeds once the fingers have closed "
         f"(reached: nothing is between them) or have stopped short (stalled, as on an object), and aborts if they "
         f"are still moving after {grip.MAX_STEPS} steps (late) or {UNSAFE}. Result: {{stop_reason: reached | "
         f"stalled | late | unsafe_motion | ended, opening, steps}}, opening as joint_state gives it; feedback: "
         f"opening and steps. {ENDS}")
RELEASE = (f"Open the fingers: {{opening}}, a grip joint position, fully open by default. The goal succeeds once "
           f"the opening is reached and aborts when the fingers stall, are still moving after {grip.MAX_STEPS} "
           f"steps (late) or a step is unsafe as for grasp. Result and feedback: as grasp's. {ENDS}")


def arm(side, base, period, goals=True):
    """An arm, moved by goals, or in the basic configuration by joint targets."""
    profile = rigs.joint(f"{side}_arm", ARM.format(side=side) + (GOALS if goals else "") + ".",
                         [f"joint{i}" for i in range(1, 7)], [0.0] * 6, base, tip_pose=True, targets=not goals)
    return rigs.action(profile, "move", JSON, MOVE.format(side=side, period=period)) if goals else profile


def gripper(side, goals=True):
    """A gripper, commanded by its grasp and release goals, or in the basic configuration by grip targets."""
    profile = rigs.gripper(
        f"{side}_gripper",
        f"ARX X5 {side} parallel two-finger gripper. A closed grip squeezes past the finger stop. The TCP is the "
        "pad centre and approaches along +x.",
        f"{side}_gripper",
        rigs.frame(f"{side}_arm:flange", (0, 0, 0), (0, 0, 0, 1)),
        rigs.frame(f"{side}_gripper:base", TCP, (0, 0, 0, 1)), targets=not goals)
    for name, text in (("grasp", GRASP), ("release", RELEASE)) if goals else ():
        rigs.action(profile, name, JSON, text)
    return profile


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
    files, depth, basic = {}, {}, {}
    for side in ("left", "right"):
        robot = sim._actuators[side][0]
        lower, upper = map(float, robot.gripper_scale)
        flange = link(sim, side, "link6")
        base = rigs.pose_frame("world:base", link(sim, side, "base_link"))
        devices[f"{side}_arm"], basic[f"{side}_arm"] = arm(side, base, sim.period), arm(side, base, sim.period, False)
        devices[f"{side}_gripper"], basic[f"{side}_gripper"] = gripper(side), gripper(side, False)
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
                   {name: devices[name] for name in order}, files, {"depth": depth, "basic": basic})
    print("rig written", flush=True)
    np.set_printoptions(precision=4)
    sim.close()


if __name__ == "__main__":
    main()
