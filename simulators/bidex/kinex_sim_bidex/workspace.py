"""The Kinex workspace of a BiDex-100 task and seed (kinex-sim setup): each camera at the mount that episode renders it.

An out-of-domain episode moves some of its cameras by a delta that upstream's evaluator draws from a committed pose
bank (bidex/eval/ood.py:camera_randomization_plan) and BiDexEnv applies in each camera's attachment-body frame: the
position shifts in that body's axes, and the rotation turns about the camera's own MuJoCo axes. With the camera's
orientation q on its body (camera_orientations.json, written by tools/robots/build_bidex.py), a profile mount M (an
optical frame) becomes M O (R(q)^T dp, R(d)) O, where O turns MuJoCo camera axes into optical ones. In-domain episodes
and unmoved cameras keep the catalog's mounts.
"""
import importlib.util
import json
from pathlib import Path

import numpy as np

from .episodes import CAMERAS, episode

ORIENTATIONS = Path(__file__).resolve().parents[1] / "camera_orientations.json"
OPTICAL = np.diag([1., -1., -1.])  # MuJoCo cameras look along -z with y up; optical frames along +z with y down


def clean(value, digits=12):
    value = round(float(value), digits)
    return 0.0 if value == 0 else value


def upstream_ood(runtime):
    """Upstream's ood.py, loaded alone: the bidex package itself imports MuJoCo."""
    path = Path(runtime) / "bidex/eval/ood.py"
    if not path.is_file():
        raise ValueError("BiDex-100's out-of-domain cameras need its upstream source; run kinex-sim prepare bidex")
    spec = importlib.util.spec_from_file_location("kinex_sim_bidex_upstream_ood", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def robot_type(runtime, suite, name):
    import yaml
    path = Path(runtime) / "bidex/tasks_data/suites" / suite / f"{name}.yaml"
    return yaml.safe_load(path.read_text())["robot_type"]


def moved(mount, orientation_wxyz, delta):
    """A profile mount {translation_m, rotation_xyzw} moved by an OOD camera delta."""
    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_quat(mount["rotation_xyzw"]).as_matrix()
    w, x, y, z = orientation_wxyz
    camera = Rotation.from_quat([x, y, z, w]).as_matrix()
    turn = Rotation.from_euler("XYZ", delta["delta_euler_deg"], degrees=True).as_matrix()
    translation = np.asarray(mount["translation_m"]) + rotation @ OPTICAL @ camera.T @ np.asarray(delta["delta_pos"])
    rotation = rotation @ OPTICAL @ turn @ OPTICAL
    return {"translation_m": [clean(v) for v in translation],
            "rotation_xyzw": [clean(v) for v in Rotation.from_matrix(rotation).as_quat()]}


def profiles(runtime, task, seed, catalog):
    """Profile changes by device for a task and launch seed: an out-of-domain episode's moved camera mounts."""
    suite, name = task.split("/")
    if not name.endswith("__ood"):
        return {}
    eval_seed, index = episode(seed, True)
    kind = robot_type(runtime, suite, name)
    plan = upstream_ood(runtime).camera_randomization_plan(kind, suite, name.removesuffix("__ood"), eval_seed, index)
    orientations = json.loads(ORIENTATIONS.read_text())[kind]
    result = {}
    for device, camera in CAMERAS.items():
        delta = plan["cameras"][camera]
        if delta["pose_id"] != "nominal":
            result[device] = {"frames": {"base": moved(catalog[device]["frames"]["base"], orientations[camera], delta)}}
    return result
