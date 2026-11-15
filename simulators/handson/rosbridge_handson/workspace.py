"""The Kinex workspace of a HandsOn task and seed (rosbridge setup): each camera as that episode renders it.

The catalog rig holds the default camera profile at its nominal mounts. A task and seed change each camera's
intrinsics to the task's profile and its mount by the episode's drawn offset (rosbridge_handson.cameras), at the
native seed the launch's seed names (seeds.json).
"""
from pathlib import Path
import sys

from .cameras import CAMERAS, camera_seed, contract, lens, matrix_xyzw, mounts, offsets
from .seeds import screened_seed

PARENTS = {"head_camera": "world:base", "left_camera": "left_arm:palm", "right_camera": "right_arm:palm"}


def clean(value, digits=12):
    value = round(float(value), digits)
    return 0.0 if value == 0 else value


def profiles(runtime, task, seed):
    """Profile changes by device for a task and launch seed: each camera's color geometry and base frame."""
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
    return result
