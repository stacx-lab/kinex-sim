"""Write the configurations of the LIBERO rigs: catalog/robots/libero/panda/configurations, which every rig links.

LIBERO's rigs predate the builders and need no measurement for their configurations, so this runs anywhere:
    pixi run --frozen python tools/robots/build_libero.py
"""
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))

import rigs  # noqa: E402

CATALOG = ROOT / "catalog/robots/libero"
OSC = ("An operational-space controller tracks it, by at most 5 cm and 0.5 rad per axis every 0.05 s, and may "
       "stop short of a pose.")


def osc(arm):
    """The arm in the osc configuration: flange pose targets that upstream's OSC_POSE executes."""
    pose = rigs.joint("arm", "", list(arm["home"]), list(arm["home"].values()), arm["frames"]["base"],
                      tip="flange", pose=OSC)["operations"]
    operations = {name: value for name, value in arm["operations"].items() if name != "joint_target"}
    return {**arm, "operations": {**operations, **{name: pose[name] for name in ("stop", "pose_target", "pose_state")}},
            "description": "Franka Panda seven-joint arm, commanded by the pose of its flange."}


def main():
    arms = {rig.parent.parent.parent.name: yaml.safe_load(rig.read_text())
            for rig in sorted(CATALOG.glob("libero-panda-*/devices/arm/profile.yaml"))}
    patches = {name: rigs.patch(arm, osc(arm)) for name, arm in arms.items()}
    first = next(iter(patches.values()))
    if any(patch != first for patch in patches.values()):
        raise ValueError("the LIBERO rigs' arms differ beyond their mounts; give each rig its own configuration")
    arm = next(iter(arms.values()))
    rigs.write_configuration(CATALOG / "panda", "osc", {"arm": arm}, {"arm": osc(arm)})
    print(f"osc written for {len(arms)} rigs", flush=True)


if __name__ == "__main__":
    main()
