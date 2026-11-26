"""CLI discovery and lifecycle plumbing without ROS or simulators."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from kinex_sim.storage import write_json
from kinex_sim.catalog import resolve, tasks
from kinex_sim.cli.main import main
from kinex_sim.project import project_root
from kinex_sim.provisioning.environment import overlay_command
from kinex_sim.runs.domains import DomainLease
from kinex_sim.runs import api
from kinex_sim.runs.records import SCHEMA, alive, identity, select

ROOT = Path(__file__).resolve().parents[2]


def test_catalog_aliases_and_cli_json(capsys):
    _, _, default = resolve("abc", ROOT)
    assert default["id"] == "abc/put-plastic-bottles-in-bin"
    assert resolve(default["id"], ROOT)[2] == default
    assert resolve("abc", ROOT, "put_plastic_bottles_in_bin")[2] == default
    assert main(["tasks", "list", "--project", str(ROOT), "--simulator", "abc", "--json"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data["tasks"] == len(tasks(ROOT, "abc")) and default["id"] in data["ids"]
    assert main(["prepare", "missing", "--json"]) == 1
    assert "Unknown simulator" in json.loads(capsys.readouterr().out)["error"]


def test_cli_imports_no_native_dependencies():
    code = '''import sys
from kinex_sim.cli.main import main
assert main(["tasks", "list", "--simulator", "abc"]) == 0
assert not any(x in sys.modules for x in ("rclpy", "mujoco", "numpy", "abc_sim"))
'''
    env = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
    subprocess.run([sys.executable, "-c", code], env=env, cwd=ROOT, check=True, capture_output=True)


def test_project_resolution_outside_checkout(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("KINEX_SIM_PROJECT", str(ROOT))
    assert project_root() == ROOT
    with pytest.raises(ValueError, match="checkout"):
        project_root(tmp_path)


def test_domain_leases_are_exclusive_and_released():
    first = DomainLease()
    second = DomainLease()
    try:
        assert first.domain != second.domain
        with pytest.raises(ValueError, match="already managed"):
            DomainLease(first.domain)
        chosen = first.domain
    finally:
        first.close()
        second.close()
    again = DomainLease(chosen)
    again.close()
    with pytest.raises(ValueError, match="between"):
        DomainLease(102)


def test_cache_and_log_environment_is_checkout_local(tmp_path):
    from kinex_sim.storage import local_environment
    inherited = {key: "/nethome/small" for key in
                 ("XDG_CACHE_HOME", "UV_CACHE_DIR", "UV_PYTHON_INSTALL_DIR",
                  "PIXI_CACHE_DIR", "ROS_HOME", "ROS_LOG_DIR", "CUDA_CACHE_PATH")}
    inherited["HOME"] = "/nethome/user"
    env = local_environment(tmp_path, inherited)
    assert env["HOME"] == inherited["HOME"]
    for key in inherited.keys() - {"HOME", "UV_PYTHON_INSTALL_DIR"}:
        assert Path(env[key]).is_relative_to(tmp_path / ".cache/kinex-sim")
    assert env["UV_PYTHON_INSTALL_DIR"] == str(tmp_path / ".runtime/python")
    assert env["KINEX_SIM_PROJECT"] == str(tmp_path)


def test_stale_owner_is_never_signalled(tmp_path, monkeypatch):
    owner = identity(os.getpid())
    record = {"schema": SCHEMA, "uid": os.getuid(), "state": "ready", "owner": {**owner, "start": "stale"}}
    path = tmp_path / "run.json"
    write_json(path, record)
    assert not alive(record)
    monkeypatch.setattr(api.os, "kill", lambda *a: pytest.fail("signalled a reused PID"))
    result = api.stop(str(path), project=ROOT)
    assert result["running"] is False


def test_default_selection_requires_one_live_run(tmp_path):
    for name in ("one", "two"):
        path = tmp_path / ".cache/kinex-sim/runs" / name / "run.json"
        path.parent.mkdir(parents=True)
        write_json(path, {"schema": SCHEMA, "uid": os.getuid(), "state": "ready", "owner": identity(os.getpid())})
    with pytest.raises(ValueError, match="2 live runs"):
        select(tmp_path)
    # A record of another schema in the directory, as an older version left them, is not a run to select.
    stale = tmp_path / ".cache/kinex-sim/runs/old/run.json"
    stale.parent.mkdir()
    write_json(stale, {"schema": "other/run/1", "uid": os.getuid(), "state": "ready", "owner": identity(os.getpid())})
    (tmp_path / ".cache/kinex-sim/runs/two/run.json").unlink()
    assert select(tmp_path)[0].parent.name == "one"


def test_describe_prints_the_task_description_of_the_live_run_on_a_domain(tmp_path, capsys):
    """describe reads the line the run's world node logged when its episode started; it opens nothing."""
    (tmp_path / "src/kinex_sim").mkdir(parents=True)
    (tmp_path / "pixi.toml").touch()
    description = {"task": "stack_bowls", "seed": 2, "instruction": "Stack the three bowls together.",
                   "detailed_instruction": "## Description\n\nNest all bowls.\n\n## Scoring\n\n- 100: stacked."}
    lines = {"started": "[sim_node-1] [INFO] [1.0] [world]: task description: " + json.dumps(description) + "\n",
             "starting": "[sim_node-1] booting\n", "other": "[sim_node-1] [INFO] [1.0] [world]: task description: "
             + json.dumps(dict(description, task="pour_by_language")) + "\n"}
    for name, domain in (("started", 42), ("starting", 43), ("other", 44)):
        folder = tmp_path / ".cache/kinex-sim/runs" / name
        folder.mkdir(parents=True)
        (folder / "graph.log").write_text(lines[name])
        write_json(folder / "run.json", {"schema": SCHEMA, "uid": os.getuid(), "id": name, "state": "ready",
                                         "owner": identity(os.getpid()), "domain_id": domain,
                                         "log": str(folder / "graph.log")})
    assert main(["describe", "--domain-id", "42", "--project", str(tmp_path)]) == 0
    assert capsys.readouterr().out == (
        "Task: stack_bowls\nSeed: 2\nInstruction: Stack the three bowls together.\n\nDetailed instruction:\n"
        "## Description\n\nNest all bowls.\n\n## Scoring\n\n- 100: stacked.\n")
    assert main(["describe", "--domain-id", "42", "--project", str(tmp_path), "--json"]) == 0
    assert json.loads(capsys.readouterr().out) == description
    # A run's description adds the episode its world initialized; describe adds its run, task and revision.
    episode = {"execution_mode": "stepped", "period_s": .04, "max_steps": 1875, "early_stop": False,
               "configuration": None, "scene": {}, "initialization": {"layout_id": 7, "layouts": 8},
               "recording": {"rate_hz": 10., "width": 960, "height": 720, "views": ["third_person"],
                             "sync": True}}
    folder = tmp_path / ".cache/kinex-sim/runs/full"
    folder.mkdir(parents=True)
    (folder / "graph.log").write_text("[sim_node-1] [INFO] [1.0] [world]: task description: "
                                      + json.dumps(dict(description, episode=episode)) + "\n")
    (tmp_path / "simulators/robodojo").mkdir(parents=True)
    (tmp_path / "simulators/robodojo/simulator.toml").write_text('name = "robodojo"\nsource_revision = "abc123"\n')
    write_json(folder / "run.json", {"schema": SCHEMA, "uid": os.getuid(), "id": "full", "state": "ready",
                                     "owner": identity(os.getpid()), "domain_id": 46, "log": str(folder / "graph.log"),
                                     "simulator": "robodojo", "task": "robodojo/stack-bowls"})
    assert main(["describe", "--domain-id", "46", "--project", str(tmp_path), "--json"]) == 0
    stated = json.loads(capsys.readouterr().out)
    assert stated == dict(description, episode=dict(episode, run="full", simulator="robodojo",
                                                    task_id="robodojo/stack-bowls", source_revision="abc123"))
    from kinex_sim.runtime.description import instructions
    assert instructions(stated) == {name: description[name] for name in ("instruction", "detailed_instruction")}
    assert main(["describe", "--domain-id", "46", "--project", str(tmp_path)]) == 0
    output = capsys.readouterr().out
    assert '\nEpisode:\n  execution_mode: "stepped"\n' in output and '  max_steps: 1875\n' in output
    assert '  initialization: {"layout_id": 7, "layouts": 8}\n' in output
    assert main(["describe", "--domain-id", "43", "--project", str(tmp_path)]) == 1
    assert "has not started its episode yet" in capsys.readouterr().err
    assert main(["describe", "--domain-id", "45", "--project", str(tmp_path)]) == 1
    assert "No live run on ROS domain 45" in capsys.readouterr().err


def test_overlay_preserves_command_arguments(tmp_path):
    overlay = tmp_path / ".workspaces/fake/install/local_setup.bash"
    overlay.parent.mkdir(parents=True)
    overlay.write_text("export KINEX_SIM_TEST=1\n")
    values = ["space here", "$(touch should-not-exist)", "`touch also-not`", "line1\nline2"]
    command = overlay_command(tmp_path, {"name": "fake"}, [sys.executable, "-c",
        "import json,sys; print(json.dumps(sys.argv[1:]))", *values])
    result = subprocess.check_output(command, cwd=tmp_path, text=True)
    assert json.loads(result) == values
    assert not (tmp_path / "should-not-exist").exists()


def test_invalid_flags_fail_before_launch():
    for args in (["launch", "abc", "--domain-id", "102"],
                 ["launch", "abc", "--timeout", "nan"],
                 ["launch", "robocasa", "--split", "test"],
                 ["demo", "robocasa", "--style-id", "x"]):
        with pytest.raises(SystemExit) as error:
            main(args)
        assert error.value.code == 2


def test_scene_options_are_those_the_simulator_declares(capsys):
    from kinex_sim.catalog import simulator
    robocasa, abc = (simulator(name, ROOT)[2] for name in ("robocasa", "abc"))
    assert api.scene_argument(robocasa, {}) is None
    assert api.scene_argument(robocasa, {"style_id": 7, "split": "target", "layout_id": 3}) == (
        'scene:={"layout_id":3,"split":"target","style_id":7}')
    with pytest.raises(ValueError, match=r"abc takes no --layout-id \(its scene options: none\)"):
        api.scene_argument(abc, {"layout_id": 3})
    # Refused before preparation is checked or a run is created.
    before = set((ROOT / ".cache/kinex-sim/runs").glob("*/run.json"))
    assert main(["launch", "abc", "--style-id", "2", "--project", str(ROOT), "--json"]) == 1
    assert "abc takes no --style-id" in json.loads(capsys.readouterr().out)["error"]
    assert set((ROOT / ".cache/kinex-sim/runs").glob("*/run.json")) == before


def test_configurations_are_those_the_simulator_declares(capsys):
    from kinex_sim.catalog import check_configuration, simulator
    robodojo, abc = (simulator(name, ROOT)[2] for name in ("robodojo", "abc"))
    assert check_configuration(robodojo, None) is None and check_configuration(robodojo, "depth") == "depth"
    with pytest.raises(ValueError, match=r"robodojo has no configuration 'stereo' \(its configurations: depth, basic\)"):
        check_configuration(robodojo, "stereo")
    # Refused before preparation is checked or a run is created.
    before = set((ROOT / ".cache/kinex-sim/runs").glob("*/run.json"))
    assert main(["launch", "abc", "--config", "depth", "--project", str(ROOT), "--json"]) == 1
    assert "abc has no configuration 'depth' (its configurations: none)" in json.loads(
        capsys.readouterr().out)["error"]
    assert set((ROOT / ".cache/kinex-sim/runs").glob("*/run.json")) == before


def test_preparation_cannot_modify_a_running_environment(tmp_path):
    from kinex_sim.provisioning.setup import workspace_lock
    with workspace_lock(tmp_path, "fake", shared=True):
        with workspace_lock(tmp_path, "fake", shared=True):
            with pytest.raises(ValueError, match="stop its runs"):
                with workspace_lock(tmp_path, "fake"):
                    pytest.fail("preparation entered while worlds were active")
    with workspace_lock(tmp_path, "fake"):
        with pytest.raises(ValueError, match="preparation to finish"):
            with workspace_lock(tmp_path, "fake", shared=True):
                pytest.fail("launch entered during preparation")


def test_a_catalog_step_limit_is_a_positive_integer(tmp_path):
    (tmp_path / "src/kinex_sim").mkdir(parents=True)
    (tmp_path / "pixi.toml").touch()
    (tmp_path / "simulators/toy").mkdir(parents=True)
    (tmp_path / "simulators/toy/simulator.toml").write_text('name = "toy"\ntask_catalog = "tasks.toml"\n')
    catalog = tmp_path / "simulators/toy/tasks.toml"
    catalog.write_text('tasks = [{ id = "toy/a", native_task = "a", max_steps = 30 }, { id = "toy/b", native_task = "b" }]\n')
    assert {k: v.get("max_steps") for k, v in tasks(tmp_path).items()} == {"toy/a": 30, "toy/b": None}
    for bad in ("0", "1.5", '"30"'):
        catalog.write_text(f'tasks = [{{ id = "toy/a", native_task = "a", max_steps = {bad} }}]\n')
        with pytest.raises(ValueError, match="max_steps must be a positive integer"):
            tasks(tmp_path)


@pytest.mark.parametrize("mode,limit", [("stepped", "max_steps:=900"), ("realtime", None), (None, None)])
def test_a_stepped_launch_takes_the_catalog_step_limit(mode, limit):
    """A stepped run ends at its task's catalog max_steps; a realtime run has no step limit."""
    from kinex_sim.runs.api import launch_arguments
    _, config, task = resolve("robocasa/close-blender-lid")
    arguments = launch_arguments(config, task, {"execution_mode": mode}, recording=True)
    assert [a for a in arguments if a.startswith("max_steps:=")] == ([limit] if limit else [])
    # A task without an upstream horizon has no step limit even in a stepped run.
    _, config, task = resolve("abc")
    assert not [a for a in launch_arguments(config, task, {"execution_mode": "stepped"}) if a.startswith("max_steps")]
