"""Official RoboCasa kitchens with the PandaOmron's native composite controller: upstream's own OSC_POSE arm and
delta torso, or in the basic configuration joint-position arm and torso on absolute targets; no ROS dependencies."""
import os
import random

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco as mj  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from kinex_sim.catalog import resolve
from kinex_sim.core import ActionLayout, CommandGroup
from kinex_sim.core.devices import device_inventory, spectator_views
from kinex_sim.project import project_root
from .assets import check_task

OPTICAL = np.diag([1., -1., -1.])
ROBOT = 'PandaOmron'
CAMERAS = {'robot0_agentview_left': 'camera_left', 'robot0_agentview_right': 'camera_right',
           'robot0_eye_in_hand': 'camera_wrist'}
SPECTATORS = (('third_person', 'robot0_agentview_left'), ('wrist', 'robot0_eye_in_hand'))
# The arm's device base, which is also OSC's reference frame (its robot0_right_center site), and its flange.
ARM_BASE, FLANGE = 'robot0_link0', 'robot0_right_hand'
# Upstream's composite controller by default; basic replaces the arm's with absolute joint positions.
CONFIGURATIONS = (None, 'basic')
# Upstream's create_env(split=...) presets (robocasa/utils/env_utils.py:86-104), passed to the kitchen unchanged so
# that the seed selects the kitchen, objects and the rest exactly as upstream does. No split passes nothing, as
# create_env(split=None) does: every kitchen and object.
SPLITS = {'target': {'obj_instance_split': 'target', 'layout_and_style_ids': [(i, i) for i in range(1, 11)]},
          'pretrain': {'obj_instance_split': 'pretrain', 'layout_ids': -2, 'style_ids': -2},
          'all': {'obj_instance_split': None, 'layout_ids': -3, 'style_ids': -3}}
# A layout or style id given replaces the split's; one not given ranges over the split's group, as upstream's
# collect_demos --layout/--style does (robocasa/scripts/collect_demos.py:449-466).
GROUPS = {'target': -1, 'pretrain': -2, 'all': -3, None: None}
KITCHEN_IDS = range(1, 61)  # upstream's layout and style ids (LayoutType, StyleType)


def kitchen_arguments(split=None, layout_id=None, style_id=None):
    """The kitchen's upstream constructor arguments for a split and optional layout and style ids, checked."""
    if split is not None and split not in SPLITS:
        raise ValueError(f'split must be one of {", ".join(SPLITS)}, not {split!r}')
    for name, value in (('layout_id', layout_id), ('style_id', style_id)):
        if value is not None and (type(value) is not int or value not in KITCHEN_IDS):
            raise ValueError(f'{name} must be an upstream id in 1-60, not {value!r}')
    arguments = dict(SPLITS.get(split, {}))
    if layout_id is None and style_id is None:
        return arguments
    arguments.pop('layout_and_style_ids', None)
    group = GROUPS[split]
    arguments.update(layout_ids=group if layout_id is None else layout_id,
                     style_ids=group if style_id is None else style_id)
    return arguments


def excluded(task, env, layout_id, style_id):
    """Why a task has no kitchen left to build: the layout or style it excludes."""
    reasons = [f'{kind} {value} (it excludes {kind}s {sorted(set(ids))}; pass another --{kind}-id)'
               for kind, value, ids in (('layout', layout_id, env.EXCLUDE_LAYOUTS),
                                        ('style', style_id, env.EXCLUDE_STYLES))
               if value in ids]
    return f'{task} excludes ' + (' and '.join(reasons) or 'every kitchen these scene options allow')


def controller_config(configuration=None):
    """Upstream's composite controller as its create_env loads it (default_pandaomron.json): OSC_POSE arm and
    JOINT_POSITION torso on delta input, native base velocity and gripper. The basic configuration replaces the
    arm's with JOINT_POSITION.

    Both joint parts start with native delta input, so the zero actions with which a
    kitchen settles during reset hold the initial posture; in basic, reset then makes them absolute.
    """
    from robosuite.controllers import load_composite_controller_config, load_part_controller_config
    config = load_composite_controller_config(controller=None, robot=ROBOT)
    if configuration == 'basic':
        parts = config['body_parts']
        parts['right'] = {**load_part_controller_config(default_controller='JOINT_POSITION'),
                          'gripper': parts['right']['gripper']}
    return config


def matrix(pose):
    """[x, y, z, qx, qy, qz, qw] as a float64 4x4 transform."""
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = Rotation.from_quat(pose[3:]).as_matrix(), pose[:3]
    return result


def placement(element):
    """A MuJoCo body's or site's world pose as a float64 4x4 transform."""
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = element.xmat.reshape(3, 3), element.xpos
    return result


def normalized(controller, change):
    """The input a part controller's scale_action maps to change; its clip stays upstream's per-step limit."""
    low, high, out_low, out_high = (controller.input_min, controller.input_max,
                                    controller.output_min, controller.output_max)
    return (change - (out_high + out_low) / 2) * (high - low) / (out_high - out_low) + (high + low) / 2


class RoboCasaSim:
    default_task = 'PickPlaceCounterToCabinet'
    period = 1 / 20

    def __init__(self, *, cameras=tuple(CAMERAS), width=256, height=256,
                 max_steps=None, early_stop=False, configuration=None):
        if configuration not in CONFIGURATIONS:
            raise ValueError(f'RoboCasa has no configuration {configuration!r}')
        self.configuration = configuration
        self.cameras, self.width, self.height = tuple(cameras), width, height
        self.max_steps = max_steps
        self.early_stop = early_stop
        self.env = self._depth = self._spectator = None
        self.spectators = ()

    @property
    def robot(self):
        return self.env.robots[0]

    @property
    def model(self):
        return self.env.sim.model._model

    @property
    def data(self):
        return self.env.sim.data._data

    @property
    def devices(self):
        cameras = {CAMERAS[camera]: camera for camera in self.cameras}
        # RGB, as upstream's evaluation observes; basic adds depth registered to each image.
        return device_inventory(self.layout, cameras, depth=tuple(cameras) if self.configuration == 'basic' else ())

    def configure_spectators(self, width=960, height=720, views=None):
        self.spectators = spectator_views(SPECTATORS, width, height, views)

    def reset(self, task, seed, randomize=True, *, split=None, layout_id=None, style_id=None):
        """Upstream's split and seed determine the episode as create_env(split=..., seed=...) does: the seed picks
        the kitchen from the split's, its objects, their placements, the base pose and the instruction. A layout or
        style id given overrides the split's."""
        spec = resolve('robocasa', task=task)[2]
        kitchen_arguments(split, layout_id, style_id)  # checked before anything is built
        scene = {'split': split, 'layout_id': layout_id, 'style_id': style_id}
        check_task(project_root() / '.runtime/robocasa', spec['native_task'])
        candidate = RoboCasaSim(cameras=self.cameras, width=self.width, height=self.height,
                                max_steps=self.max_steps,
                                early_stop=self.early_stop, configuration=self.configuration)
        candidate.spectators = self.spectators
        # Kitchen generation also draws from process RNGs; a failed reset keeps the
        # live episode's future randomness.
        python_rng, numpy_rng = random.getstate(), np.random.get_state()
        try:
            candidate._initialize(spec, seed, scene)
            if hasattr(self, 'layout') and candidate.layout != self.layout:
                raise ValueError('reset cannot change the action layout')
        except BaseException:
            candidate.close()
            random.setstate(python_rng)
            np.random.set_state(numpy_rng)
            raise
        self.close()
        self.__dict__.update(candidate.__dict__)

    def _initialize(self, spec, seed, scene):
        import robocasa  # noqa: F401  (registers the kitchen environments)
        import robosuite

        self.task, self.seed = spec['native_task'], seed
        random.seed(seed)
        np.random.seed(seed)
        self.env = robosuite.make(
            self.task, robots=ROBOT, controller_configs=controller_config(self.configuration), has_renderer=False,
            has_offscreen_renderer=True, use_camera_obs=False, camera_names=list(self.cameras),
            camera_widths=self.width, camera_heights=self.height, camera_depths=False,
            ignore_done=True, translucent_robot=False, seed=seed, **kitchen_arguments(**scene))
        # Upstream drops the task's excluded layouts and styles from the kitchens it may build; no kitchen is built
        # until the first reset, so an excluded one fails here at no cost.
        if not self.env.layout_and_style_ids:
            raise ValueError(excluded(self.task, self.env, scene['layout_id'], scene['style_id']))
        self.env.reset()
        # Measurements and frames describe the current state, not the last substep's start.
        self.env.sim.forward()
        for camera in (*self.cameras, *(v.source for v in self.spectators)):
            if mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_CAMERA, camera) < 0:
                raise ValueError(f'unknown camera: {camera}')
        self._index()
        # By default every controller stays as upstream's reset leaves it.
        if self.configuration == 'basic':
            self._basic()
        self._grip_rate = 0.
        self._origin = self.data.xpos[self._base].copy(), self.data.xmat[self._base].reshape(3, 3).copy()
        self.steps, self.done, self.success, self.reward = 0, False, False, 0.
        meta = self.env.get_ep_meta()
        # The kitchen the seed picked, or the one the ids pinned.
        self.scene = {'split': scene['split'], 'layout_id': meta['layout_id'], 'style_id': meta['style_id']}
        self.initialization = dict(self.scene)
        # The language instruction upstream's episode metadata gives its policy.
        self.description = {'instruction': meta['lang']}
        if self.spectators:
            from kinex_sim.core.mujoco_rendering import SpectatorRenderer
            context = self.env.sim._render_context_offscreen
            self._spectator = SpectatorRenderer(self.model, self.data, self.spectators,
                                               restore_context=context.gl_ctx, options=context.vopt)
            self._spectator.capture()

    def _basic(self):
        """The basic configuration's start, once the kitchen has settled under native zero-delta holds: absolute
        joint goals from now on, the integrated gripper command seeded from the fingers so stop holds them, and a
        depth renderer."""
        for part in ('right', 'torso'):
            self.robot.part_controllers[part].input_type = 'absolute'
        gripper = self.robot.part_controllers['right_gripper']
        fingers = self.data.qpos[gripper.qpos_index]
        self.robot.gripper['right'].current_action = np.clip(
            2 * (fingers - gripper.actuator_min) / (gripper.actuator_max - gripper.actuator_min) - 1, -1, 1)
        # Pixel-center depth from a separate single-sample context; RGB keeps native antialiasing.
        quality = self.model.vis.quality
        samples, quality.offsamples = quality.offsamples, 0
        try:
            self._depth = mj.Renderer(self.model, height=self.height, width=self.width)
        finally:
            quality.offsamples = samples

    def _index(self):
        parts, model = self.robot.part_controllers, self.model
        joints = {name: [model.joint(j).name for j in parts[name].joint_index]
                  for name in ('right', 'torso', 'base')}
        self._qpos = [int(model.jnt_qposadr[j]) for name in ('right', 'torso')
                      for j in parts[name].joint_index]
        self._qvel = [int(model.jnt_dofadr[j]) for name in ('right', 'torso')
                      for j in parts[name].joint_index]
        self._base = model.body(self.robot.robot_model.base.root_body).id
        arm, torso = (model.jnt_range[parts[name].joint_index] for name in ('right', 'torso'))
        low, high = parts['base'].control_limits
        # By default the arm takes poses of its flange, which upstream's OSC executes; basic takes joint targets.
        tip = None if self.configuration == 'basic' else 'flange'
        self.layout = ActionLayout((*joints['right'], 'gripper', *joints['torso'], *joints['base']), (
            CommandGroup('arm', tuple(range(7)), tuple(arm[:, 0]), tuple(arm[:, 1]), tip=tip),
            CommandGroup('gripper', (7,), (-1.,), (1.,), scalar=True),
            CommandGroup('torso', (8,), tuple(torso[:, 0]), tuple(torso[:, 1])),
            CommandGroup('base', (9, 10, 11), tuple(low), tuple(high), velocity=True)))
        if tip:
            # The grip site OSC tracks (the gripper's tcp), fixed in the flange's frame, and where the last target
            # put it.
            self._grip = np.linalg.inv(self._in_arm_base('body', FLANGE)) @ self._in_arm_base(
                'site', parts['right'].ref_name)
            self._site = None

    def _in_arm_base(self, kind, name):
        """A body's or site's measured pose in the arm base as a float64 4x4 transform."""
        return np.linalg.inv(placement(self.data.body(ARM_BASE))) @ placement(getattr(self.data, kind)(name))

    def tip_pose(self, group):
        """The measured flange, robosuite's robot0_right_hand, in the arm base: [x, y, z, qx, qy, qz, qw]."""
        pose = self._in_arm_base('body', FLANGE)
        return np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_quat()]

    def _arm(self, pose, desired):
        """The OSC_POSE input for a flange pose target in the arm base, OSC's frame: the delta of the grip site
        it puts, which upstream applies as a policy's. In upstream's arm mode the delta is from where the site is
        now; in its base mode (desired), which moves the previous goal, from the previous target. Zero, upstream's
        idle, while the arm has had no command."""
        if pose is None:
            return np.zeros(6)
        osc = self.robot.part_controllers['right']
        site = matrix(pose) @ self._grip
        reference = self._site if desired and self._site is not None else self._in_arm_base('site', osc.ref_name)
        self._site = site
        return normalized(osc, np.r_[site[:3, 3] - reference[:3, 3],
                                     Rotation.from_matrix(site[:3, :3] @ reference[:3, :3].T).as_rotvec()])

    def _torso(self, target):
        """The JOINT_POSITION delta input that moves the torso's goal from the measured lift to target."""
        torso = self.robot.part_controllers['torso']
        return normalized(torso, np.atleast_1d(target) - self.data.qpos[torso.qpos_index])

    @property
    def time_s(self):
        return self.steps * self.period

    def measures(self):
        """The episode's split and the kitchen it built, as its record states them."""
        return dict(self.scene)

    def _odometry(self):
        """Base pose relative to its episode-start pose, and its body-frame twist."""
        position, rotation = self._origin
        offset = rotation.T @ (self.data.xpos[self._base] - position)
        relative = rotation.T @ self.data.xmat[self._base].reshape(3, 3)
        twist = np.empty(6)
        mj.mj_objectVelocity(self.model, self.data, mj.mjtObj.mjOBJ_XBODY, self._base, twist, 1)
        return np.r_[offset[:2], np.arctan2(relative[1, 0], relative[0, 0])], twist[[3, 4, 2]]

    def state(self):
        """Arm and torso joints, the integrated gripper command (-1 open, +1 closed), and
        the base odometry pose (x, y, yaw) and twist (vx, vy, wz)."""
        q, v = self.data.qpos[self._qpos], self.data.qvel[self._qvel]
        pose, twist = self._odometry()
        grip = self.robot.gripper['right'].current_action[1]
        return np.r_[q[:7], grip, q[7:], pose], np.r_[v[:7], self._grip_rate, v[7:], twist]

    def step(self, action, poses=None):
        if self.done:
            raise RuntimeError('episode is done')
        action = np.asarray(action, dtype=float)
        gripper = self.robot.gripper['right']
        before = gripper.current_action[1]
        # The native gripper integrates a direction (+1 closes) by `speed` per step within
        # [-1, 1]; steer it toward the target while a native step gets strictly closer.
        error = action[7] - before
        after = np.clip(before + gripper.speed * np.sign(error), -1, 1)
        direction = float(np.sign(error)) if abs(action[7] - after) < abs(error) else 0.
        # In kitchens the native (legacy) base controller already takes body-frame velocity.
        base = action[9:12]
        if self.configuration == 'basic':
            parts = {'right': action[:7], 'torso': action[8:9]}
        else:
            # Upstream's base mode while the base drives, as its demonstrations switch to it: the arm's goal then
            # updates from its previous goal, which keeps the hand there in the moving arm base.
            mode = 1. if np.any(base) else -1.
            parts = {'right': self._arm((poses or {}).get('arm'), mode > 0), 'torso': self._torso(action[8]),
                     'base_mode': mode}
        self.env.step(self.robot.create_action_vector({**parts, 'right_gripper': [direction], 'base': base}))
        # Upstream's evaluation (robocasa/wrappers/gym_wrapper.py) evaluates the task again after the step's
        # state update and scores that, so predicates that count calls count twice per step, as there.
        self.success = bool(self.env._check_success())
        self.reward = float(self.success)
        self.env.sim.forward()
        self._grip_rate = (gripper.current_action[1] - before) / self.period
        self.steps += 1
        self.done = bool((self.max_steps and self.steps >= self.max_steps)
                         or (self.early_stop and self.success))

    @staticmethod
    def _quat(matrix):
        quat = np.empty(4)
        mj.mju_mat2Quat(quat, np.asarray(matrix).reshape(9))
        return quat

    def camera_pose(self, camera):
        cid = self.model.camera(camera).id
        return (self.data.cam_xpos[cid].copy(),
                self._quat(self.data.cam_xmat[cid].reshape(3, 3) @ OPTICAL))

    def intrinsics(self, camera):
        focal = .5 * self.height / np.tan(np.deg2rad(self.model.camera(camera).fovy[0]) / 2)
        w, h = self.width, self.height
        return w, h, float(focal), float(focal), (w - 1) / 2., (h - 1) / 2.

    def render(self, camera):
        if camera not in self.cameras:
            raise KeyError(camera)
        return np.ascontiguousarray(self.env.sim.render(camera_name=camera, width=self.width,
                                                        height=self.height)[::-1])

    def render_depth(self, camera):
        from kinex_sim.core.mujoco_rendering import render_depth
        if camera not in self.cameras or self._depth is None:  # only basic's cameras are RGB-D
            raise KeyError(camera)
        options = self.env.sim._render_context_offscreen.vopt
        self._depth.update_scene(self.data, camera=camera, scene_option=options)
        return render_depth(self._depth, self.model)

    def render_spectators(self):
        return self._spectator.capture()

    def spectator_intrinsics(self, name):
        return self._spectator.intrinsics(name)

    def close(self):
        for renderer in (self._spectator, self._depth):
            if renderer is not None:
                renderer.close()
        self._spectator = self._depth = None
        if self.env is not None:
            self.env.close()
            self.env = None
