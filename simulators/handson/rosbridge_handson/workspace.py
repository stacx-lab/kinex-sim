"""The Kinex workspace of a HandsOn task and seed (rosbridge setup): each camera as that episode renders it, and
only the arms and hands upstream drives.

The catalog rig holds the default camera profile at its nominal mounts. A task and seed change each camera's
intrinsics to the task's profile and its mount by the episode's drawn offset (rosbridge_handson.cameras), at the
native seed the launch's seed names (seeds.json). In the HELD tasks the left arm and hand declare no commands.
"""
from pathlib import Path
import sys

from .cameras import CAMERAS, camera_seed, contract, lens, matrix_xyzw, mounts, offsets
from .seeds import screened_seed

PARENTS = {"head_camera": "world:base", "left_camera": "left_arm:palm", "right_camera": "right_arm:palm"}
# Single-arm tasks whose control path holds the left arm and hand at home and never asks for their targets
# (bimanual_assembly_env.py:_apply_joint_targets). Single-arm tasks of the frozen Core bases, and t11, t29, t33,
# t34 and t47, drive both arms. The native suite checks this against upstream.
HELD = ("eval_comb01_cover_pitcher_and_box", "eval_gen01_cover_lunchbox", "eval_gen03_orient_bottle_upright",
        "eval_gen06_spray_shoe", "eval_gen07_stack_boxes")


def clean(value, digits=12):
    value = round(float(value), digits)
    return 0.0 if value == 0 else value


def profiles(runtime, task, seed, catalog=None):
    """Profile changes by device for a task and launch seed: each camera's color geometry and base frame, and in a
    HELD task no command operations on the left arm and hand."""
    source = Path(runtime) / "handson"
    if not (source / "handson/evaluation/camera_contracts.py").is_file():
        raise ValueError("HandsOn's cameras need its upstream source; run rosbridge prepare handson")
    if str(source) not in sys.path:  # pure-Python upstream modules; nothing imports Isaac
        sys.path.append(str(source))
    native = screened_seed(task, seed)
    value = contract(task)
    mount = mounts(value, offsets(task, camera_seed(task, native)))
    result = {}
    for device, role in CAMERAS.items():
        width, height, fx, fy, cx, cy = lens(value["cameras"][role])
        result[device] = {
            "color": {"width": width, "height": height,
                      "projection": {"model": "pinhole", "fx": clean(fx), "fy": clean(fy), "cx": clean(cx),
                                     "cy": clean(cy), "skew": 0}},
            "frames": {"base": {"translation_m": [clean(v) for v in mount[device][:3, 3]],
                                "rotation_xyzw": [clean(v) for v in matrix_xyzw(mount[device][:3, :3])],
                                "parent": PARENTS[device]}}}
    if task in HELD:
        result["left_arm"] = {"operations": {"pose_target": None, "stop": None}}
        result["left_hand"] = {"operations": {"joint_target": None, "stop": None}}
    return result
