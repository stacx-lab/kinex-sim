"""Measure the HandsOn ROKAE AR5 / Wuji rig and write catalog/robots/handson/handson-rokae-wuji.

Chains come from upstream's robot MJCF (sim/envs/xmls/rokae_wuji1_<side>.xml), which Isaac imports
at the same base poses for every task. Cameras are the default profile of the camera contract at
their nominal mounts: the head camera fixed in the world, each wrist camera on its palm. rosbridge
setup changes them to the task's profile and the episode's drawn mounts (rosbridge_handson.workspace);
the native test checks that workspace against the live Isaac scene. Runs in the native environment after
`rosbridge prepare handson`:
    pixi run -e handson-native env PYTHONPATH=src:simulators/handson:.runtime/handson \
        python tools/robots/build_handson.py
"""
import ast
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "simulators/handson"), str(Path(__file__).parent)]

import mujoco  # noqa: E402
import rigs  # noqa: E402
from handson.evaluation.camera_contracts import current  # noqa: E402
from rosbridge_handson.backend import SIDES  # noqa: E402
from rosbridge_handson.cameras import CAMERAS, OPENGL_TO_OPTICAL, lens  # noqa: E402

PACKAGE = ROOT / ".runtime/handson/handson/handson"


def home(side):
    """The scene's arm home (bimanual_assembly_scene._ARM_HOME); the left arm mirrors the right."""
    scene = ast.parse((PACKAGE / "sim/isaaclab/bimanual_assembly_scene.py").read_text())
    constants = {node.targets[0].id: node.value for node in scene.body
                 if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)}
    right = ast.literal_eval(constants["_RIGHT_ARM_HOME"])
    signs = ast.literal_eval(constants["_LEFT_ARM_MIRROR_SIGNS"]) if side == "left" else (1,) * 7
    return [sign * q for sign, q in zip(signs, right)]


def main():
    cameras = current()["cameras"]  # the default profile
    devices = {"world": rigs.world("World clock of the HandsOn station.")}
    files = {}
    for side in SIDES:
        model = mujoco.MjModel.from_xml_path(str(PACKAGE / f"sim/envs/xmls/rokae_wuji1_{side}.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        palm = f"{side}_palm_link"
        devices[f"{side}_arm"] = rigs.joint(
            f"{side}_arm",
            f"Seven-joint ROKAE AR5 {side} arm, commanded by the pose of its tip link palm, where the "
            f"{side} hand is mounted.",
            [f"joint{i}_{side}" for i in range(1, 8)], home(side),
            rigs.pose_frame("world:base", rigs.body_world(data, model.body(f"link0_{side}").id)),
            tip="palm", pose="HandsOn's own IK tracks it and may stop short of a pose.")
        files[f"devices/{side}_arm/arm.urdf"] = rigs.mujoco_chain(
            model, [f"link{i}_{side}" for i in range(1, 8)] + [f"attachment_{side}", f"{side}_hand_mount", palm],
            f"rokae_ar5_{side}_arm",
            f"HandsOn ROKAE AR5 {side} arm from rokae_wuji1_{side}.xml link0_{side}..{palm}; palm is "
            f"the {palm} body frame.", tip="palm")
        joints = [f"{side}_finger{finger}_joint{joint}" for finger in range(1, 6) for joint in range(1, 5)]
        devices[f"{side}_hand"] = rigs.joint(
            f"{side}_hand",
            f"Wuji 20-joint {side} hand on the {side}_arm palm; finger1 is the thumb and finger5 the "
            "little finger.",
            joints, [float(np.clip(0., *model.joint(joint).range)) for joint in joints],
            rigs.frame(f"{side}_arm:palm", (0, 0, 0), (0, 0, 0, 1)), part="hand", tip=None, model="hand.urdf")
        files[f"devices/{side}_hand/hand.urdf"] = rigs.mujoco_tree(
            model, palm, f"wuji_{side}_hand",
            f"HandsOn Wuji {side} hand from rokae_wuji1_{side}.xml: base_link is the {palm} body frame.")
        camera = f"{side}_camera"
        spec = cameras[CAMERAS[camera]]
        devices[camera] = rigs.camera(
            camera,
            f"RGB camera on the {side} wrist, looking along the hand.", lens(spec),
            rigs.pose_frame(f"{side}_arm:palm", np.asarray(spec["nominal_camera_to_palm_opengl"]) @ OPENGL_TO_OPTICAL))
    spec = cameras["main"]
    devices["head_camera"] = rigs.camera(
        "head_camera",
        "Fixed RGB head camera above the station.", lens(spec),
        rigs.pose_frame("world:base", np.asarray(spec["nominal_world_pose_opengl"]) @ OPENGL_TO_OPTICAL))
    order = ["world", "right_arm", "right_hand", "left_arm", "left_hand",
             "head_camera", "left_camera", "right_camera"]
    rigs.write_rig(ROOT / "catalog/robots/handson/handson-rokae-wuji",
                   rigs.workspace("handson-rokae-wuji", "HandsOn ROKAE AR5 with Wuji hands", "arm",
                                  "ROKAE AR5 (dual) with Wuji hands"),
                   {name: devices[name] for name in order}, files)
    print("rig written", flush=True)


if __name__ == "__main__":
    main()
