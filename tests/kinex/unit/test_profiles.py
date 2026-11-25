"""Every catalog rig against Kinex's profile schema."""
from pathlib import Path
from unittest.mock import patch

import pytest

from kinex_sim.catalog import simulator, tasks
from kinex_sim.catalog.robots import list_robots, setup

PROJECT = Path(__file__).resolve().parents[3]


def test_kinex_schema_accepts_every_rig(tmp_path):
    """Kinex's own profile schema parses every export, in every configuration.

    Kinex resolves URDFs in its own Python environment, a multi-gigabyte Pixi
    project it would otherwise provision in every workspace. Set
    KINEX_PYTHON_PROJECT to one prepared `.kinex/python` directory to share it.
    """
    import json
    import os
    import shutil
    import subprocess
    dist = PROJECT.parent / "kinex/packages/core/dist"
    python = os.environ.get("KINEX_PYTHON_PROJECT")
    if not python or not shutil.which("node") or not (dist / "device/profile/store.js").is_file():
        pytest.skip("needs Node, a built Kinex core next to this checkout and KINEX_PYTHON_PROJECT")
    # Every rig in its standard devices and in each configuration its simulator declares.
    exports = [(robot, configuration) for robot in list_robots(PROJECT)
               for configuration in (None, *simulator(robot.split("/")[0], PROJECT)[2].get("configurations", {}))]
    workspaces = [tmp_path / f"{robot.replace('/', '-')}-{configuration}" for robot, configuration in exports]
    for (robot, configuration), workspace in zip(exports, workspaces):
        with patch("kinex_sim.catalog.robots.changes", lambda *args: {}):  # the rig, without a task's changes
            setup(next(task["id"] for task in tasks(PROJECT).values() if task["robot"] == robot), workspace,
                  project=PROJECT, configuration=configuration)
        (workspace / ".kinex/python").symlink_to(Path(python).resolve(), target_is_directory=True)
    result = subprocess.run(["node", str(Path(__file__).with_name("profiles.mjs")), str(dist), *map(str, workspaces)],
                            stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr
    parsed = json.loads(result.stdout)
    assert len(parsed) == len(exports)
    for (robot, configuration), directory, workspace in zip(exports, workspaces, parsed):
        exported = {path.parent.name for path in (directory / ".kinex/devices").glob("*/profile.yaml")}
        assert workspace["robot"] == robot.split("/")[1] and set(workspace["devices"]) == exported, (robot, configuration)
