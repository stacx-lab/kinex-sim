"""Export the Kinex workspace of a task and seed from the robot catalog, without importing ROS or simulators.

Each robot is a complete workspace skeleton whose profiles carry its calibration.
Files shared by several robots are symlinks; export copies their content. A simulator
whose devices differ by task or seed, such as HandsOn's cameras, names a `workspace`
hook in its manifest: "module:function" under its python_path, called with its runtime
directory, the native task and the seed, which returns profile changes by device as
JSON merge patches (RFC 7386).
"""
import importlib
from pathlib import Path
import json
import shutil
import sys
import tempfile

import yaml

from rosbridge.catalog import resolve
from rosbridge.project import project_root
from rosbridge.runs.domains import check_domain


def list_robots(project=None):
    root = project_root(project) / "catalog/robots"
    return sorted(str(file.parent.relative_to(root)) for file in root.glob("*/*/workspace.yaml"))


def merge(profile, changes):
    """Change a profile in place by a JSON merge patch: mappings merge key by key, None removes, other values
    replace."""
    for key, value in changes.items():
        if value is None:
            profile.pop(key, None)
        elif isinstance(value, dict) and isinstance(profile.get(key), dict):
            merge(profile[key], value)
        else:
            profile[key] = value


def changes(root, config, task, seed):
    """The simulator's profile changes for a task and seed, by device; none without a workspace hook."""
    if not config.get("workspace"):
        return {}
    module, function = config["workspace"].split(":")
    path = str(root / config["python_path"])
    added = path not in sys.path
    if added:
        sys.path.insert(0, path)
    try:
        hook = getattr(importlib.import_module(module), function)
    finally:
        if added:
            sys.path.remove(path)
    return hook(root / ".runtime" / config["name"], task["native_task"], seed)


def setup(selection, directory=None, *, seed=None, project=None, domain_id=0):
    """The unloaded workspace a launch of the same task and seed serves: the task's robot, with its simulator's
    profile changes for that task and seed. The seed defaults as a launch's does."""
    check_domain(domain_id)
    root, config, task = resolve(selection, project)
    seed = seed if seed is not None else task.get("seed", 0)
    identifier = task["robot"]
    if identifier not in list_robots(root):
        raise ValueError(f"task {task['id']} names an unknown robot {identifier!r}")
    by_device = changes(root, config, task, seed)
    target = Path(directory or Path.cwd()).resolve() / ".kinex"
    if any((target / name).exists() for name in ("workspace.yaml", "devices")):
        raise ValueError("workspace already contains device configuration; export into a new directory")
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".kinex-export-", dir=target.parent) as temporary:
        staging = Path(temporary) / "kinex"
        shutil.copytree(root / "catalog/robots" / identifier, staging,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        unknown = set(by_device) - {profile.parent.name for profile in staging.glob("devices/*/profile.yaml")}
        if unknown:
            raise ValueError(f"{config['name']}'s workspace hook changes unknown devices {sorted(unknown)}")
        for profile in staging.glob("devices/*/profile.yaml"):
            value = yaml.safe_load(profile.read_text())
            merge(value, by_device.get(profile.parent.name, {}))
            value["loaded"] = False
            value["runtime"] = {**value["runtime"], "domain_id": domain_id}
            profile.write_text(yaml.safe_dump(value, sort_keys=False))
        target.mkdir(exist_ok=True)
        for name in ("workspace.yaml", "devices"):
            (staging / name).rename(target / name)
    config_file = target / "kinex.json"
    config_value = json.loads(config_file.read_text()) if config_file.exists() else {}
    config_value.setdefault("python", {"provider": "ros-jazzy"})
    config_file.write_text(json.dumps(config_value, indent=2) + "\n")
    return {"task": task["id"], "seed": seed, "robot": identifier, "workspace": str(target), "loaded": False}
