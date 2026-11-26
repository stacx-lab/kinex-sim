import hashlib
import json
import re

import pytest

from kinex_sim.assets import AssetStore
from kinex_sim.catalog import resolve, tasks
from kinex_sim_robocasa import assets

SLUG = re.compile(r'(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])')


def test_catalog_holds_robocasa365_and_leaves_the_scene_to_the_launch():
    entries = list(tasks(name='robocasa').values())
    assert len(entries) == 365
    # Kitchens and object instances are scene options, not task fields; the step limit is the task's.
    assert all(set(t) - {'max_steps'} == {'id', 'robot', 'native_task', 'simulator'} for t in entries)
    assert {t['robot'] for t in entries} == {'robocasa/robocasa-panda-omron'}
    for task in entries:
        assert task['id'] == 'robocasa/' + SLUG.sub('-', task['native_task']).lower()
        assert resolve(task['id'])[2] == task
    assert resolve('robocasa')[2]['id'] == 'robocasa/pick-place-counter-to-cabinet'
    assert resolve('PrepareVeggieDip')[2]['id'] == 'robocasa/prepare-veggie-dip'


def test_manifest_scene_options_are_the_backend_reset_keywords():
    import ast
    from pathlib import Path
    from kinex_sim.catalog import simulator
    # The backend imports MuJoCo, which the CLI environment lacks; read reset's signature from its source.
    source = Path(assets.__file__).with_name('backend.py').read_text()
    reset, = (node for node in ast.walk(ast.parse(source))
              if isinstance(node, ast.FunctionDef) and node.name == 'reset')
    assert simulator('robocasa')[2]['scene_options'] == [arg.arg for arg in reset.args.kwonlyargs]


def test_manifest_configurations_are_the_backend_configurations():
    """RGB cameras and the flange-pose arm by default, as upstream's evaluation runs; basic makes the cameras RGB-D
    and gives the arm joint targets."""
    import ast
    from pathlib import Path
    from kinex_sim.catalog import simulator
    manifest = simulator('robocasa')[2]
    source = Path(assets.__file__).with_name('backend.py').read_text()
    declared, = (node.value for node in ast.parse(source).body if isinstance(node, ast.Assign)
                 and isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'CONFIGURATIONS')
    assert [None, *manifest['configurations']] == list(ast.literal_eval(declared)) == [None, 'basic']
    cameras = {name: kind for name, kind in manifest['devices'].items() if name.startswith('camera_')}
    assert set(cameras.values()) == {'rgb'}
    assert manifest['configurations']['basic']['devices'] == dict.fromkeys(cameras, 'rgbd')


def test_seeded_prepare_adopts_verified_archives_and_overlays_the_upstream_tree(tmp_path,
                                                                                monkeypatch):
    seed = tmp_path / 'seed/robocasa'
    for name, (_, root, _) in assets.ARCHIVES.items():
        files = {f'{root}/a/{name}.xml': name.encode()}
        if root == 'fixtures':  # upstream tracks fixtures/cabinets and repeats one file
            files.update({f'{root}/cabinets/panel.xml': b'tracked',
                          f'{root}/cabinets/door/model.xml': b'door'})
        for path, data in files.items():
            (seed / path).parent.mkdir(parents=True, exist_ok=True)
            (seed / path).write_bytes(data)
        (seed / f'{name}.json').write_text(json.dumps({'url': assets.url(name), 'files': [
            {'path': path, 'sha256': hashlib.sha256(data).hexdigest()} for path, data in files.items()]}))
    runtime = tmp_path / 'runtime'
    tree = runtime / 'robocasa/models/assets'
    (tree / 'fixtures/cabinets').mkdir(parents=True)
    (tree / 'fixtures/cabinets/panel.xml').write_bytes(b'tracked')
    (runtime / 'robosuite/robosuite').mkdir(parents=True)
    monkeypatch.setenv('KINEX_SIM_ASSET_SEED', str(tmp_path / 'seed'))
    store = AssetStore(tmp_path / 'store')
    with pytest.raises(RuntimeError, match='not prepared'):
        assets.check_task(runtime, 'Task')
    (seed / 'textures/a/textures.xml').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='differs from its receipt'):
        assets.prepare(runtime, 'Task', store)
    assert (seed / 'textures').is_dir()  # a rejected seed stays where it was
    (seed / 'textures/a/textures.xml').write_bytes(b'textures')
    assets.prepare(runtime, 'Task', store)
    assert not (seed / 'textures').exists() and (seed / 'textures.json').exists()
    assert not (tree / 'fixtures').is_symlink() and (tree / 'fixtures/cabinets/door').is_symlink()
    assert not (tree / 'fixtures/cabinets/panel.xml').is_symlink()
    assert (tree / 'objects/lightwheel/a/objects_lightwheel.xml').read_bytes() == b'objects_lightwheel'
    assert (tree / 'textures').is_symlink() and (runtime / 'robocasa/macros_private.py').exists()
    assets.prepare(runtime, 'Other', store)  # one preparation serves every task
    assets.check_task(runtime, 'Other')


def test_catalog_step_limits_are_the_registry_horizons():
    """Each task's max_steps is its dataset horizon in the pinned registry (robocasa/utils/dataset_registry.py, which
    get_task_horizon returns); a task the registry gives none takes the longest of those horizons."""
    import ast
    from kinex_sim.catalog import simulator
    root, _, config = simulator('robocasa')
    registry = root / config['source'] / 'robocasa/utils/dataset_registry.py'
    if not registry.is_file():
        pytest.skip('the RoboCasa source is not exported')
    horizons = {}
    for node in ast.parse(registry.read_text()).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') in (
                'ATOMIC_TASK_DATASETS', 'COMPOSITE_TASK_DATASETS'):
            for task in node.value.keywords:
                for field in task.value.keywords:
                    if field.arg == 'horizon':
                        horizons[task.arg] = ast.literal_eval(field.value)
    entries = tasks(name='robocasa').values()
    longest = max(horizons.values())
    assert {t['id']: t['max_steps'] for t in entries} == {
        t['id']: horizons.get(t['native_task'], longest) for t in entries}
    assert resolve('robocasa/prepare-veggie-dip')[2]['max_steps'] == longest == 7200
    assert resolve('robocasa/close-blender-lid')[2]['max_steps'] == 900
