# RoboDojo

RoboDojo runs its 42 simulation tasks on the dual ARX X5 rig in Isaac Sim 5.1,
with upstream's saved evaluation layouts. The integration follows RoboEnv's
RoboDojo device runtime. Upstream is pinned to
`c3af1ac6ad7302c0c09d17f4b050fd3a5c68c921`, with its Isaac Lab fork
(`yuechen0614/IsaacLab@afca7b09`) and cuRobo fork (`yuechen0614/curobo@895c6517`)
pinned as nested sources. See the [design and CLI guide](README.md) for shared
lifecycle and timing contracts.

```bash
kinex-sim prepare robodojo
kinex-sim tasks list --simulator robodojo
kinex-sim launch robodojo/stack-bowls --execution-mode stepped --timeout 1800
kinex-sim setup robodojo/stack-bowls --directory ../my-robodojo-workspace
kinex-sim setup robodojo/stack-bowls --config depth --directory ../my-robodojo-depth-workspace
kinex-sim launch robodojo/stack-bowls --config depth --execution-mode stepped --timeout 1800
kinex-sim launch robodojo/stack-bowls --config basic --execution-mode stepped --timeout 1800
```

`robodojo` selects `pack_objects_into_box`. The twelve `*_random` variants are not
cataloged. Three competition tasks (`imitate_sorting_sequence`, `make_kong`,
`play_tic_tac_toe`) add a Franka support arm that the native environment drives;
it is not a device.

## Native worker

Isaac Sim 5.1 publishes Python 3.11 wheels only, while ROS Jazzy from RoboStack is
Python 3.12. The world node therefore runs in the `robodojo` environment (the
shared ROS environment) and proxies `kinex_sim_robodojo.backend.RoboDojoSim`,
which runs on the main thread of a worker process in `robodojo-native` (Python
3.11, Isaac Sim 5.1, CUDA 12.8 torch 2.7, RoboEnv's pins). Calls are sequential
over a private socket, so the ROS clock, waits and device semantics are the same
as for in-process simulators; a dead worker stops the graph. The worker sees
only one GPU, the first in `CUDA_VISIBLE_DEVICES` (default 0), and renders on its
Vulkan device: on IOMMU hosts Kit otherwise validates peer-to-peer access between
every GPU, holding each for minutes. It imports torch before Kit starts, since
torch's bundled OpenMP needs static TLS that Kit's plugins would otherwise use up.

Isaac starts, compiles shaders and builds the scene before the episode starts: allow
several minutes (the default launch timeout is 900 s; the first launch on a host
may need `--timeout 1800`). A worker serves the one task it was launched with.

## Source and assets

Preparation exports RoboDojo with both forks into `.runtime/robodojo`, applies
RoboEnv's four patches (success and a failed constraint end an episode only with early stop; an
injected model client replaces the policy server; predicate diagnostics hooks;
Isaac Lab's livestream dependency becomes an extra), and installs the forks as
editable packages into `robodojo-native`. A fifth patch lets upstream's cuRobo planner read its
`curobo_tmp.yml` template, with `${ASSETS_PATH}` replaced. Upstream's install script writes that
same file as `curobo.yml` (`utils/update_embodiment_config_path.py`), but kinex-sim never writes
into the pinned asset store.

Assets come from the `RoboDojo-Benchmark/RoboDojo` dataset at
`1127bda94f172d757386945af3eb3a6754d1a4e6`: 15,365 files (41 GB) under
`Assets/`, each pinned by LFS SHA-256 or git blob SHA-1 in the dataset listing.
With `KINEX_SIM_ASSET_SEED=DIR`, a depot laid out as `DIR/robodojo/<revision>/Assets` is
verified file by file and moved into the store; otherwise files are downloaded
once at a polite request rate. A seed of hard links to an earlier revision's files, with the
changed files replaced, bumps the pin without copying.

## Devices

| Device | Interfaces |
|---|---|
| `world` | `world/now`, `world/wait`, `world/stop`, `/clock`, `world/spectator/*` while recording |
| `left_arm`, `right_arm` | `<arm>/joint_state`, `<arm>/pose_state`, `<arm>/move`, `<arm>/stop`; `<arm>/joint_target` in place of `<arm>/move` with `--config basic` |
| `left_gripper`, `right_gripper` | `<gripper>/joint_state`, `<gripper>/grasp`, `<gripper>/release`, `<gripper>/stop`; `<gripper>/target` in place of the two goals with `--config basic` |
| `head_camera`, `left_camera`, `right_camera` | `<camera>/image`, `<camera>/camera_info`; `<camera>/depth` with `--config depth` |

Both arms keep upstream's joint names `joint1..joint6` (radians; native limits
±10 rad for joints 1–5 and ±3.14 for joint 6); the device disambiguates them.
Goals command the devices (below); the basic configuration's targets are upstream's
`<side>_arm_joint_state` action entries and its normalized `<side>_ee_joint_state`
value. A gripper's position runs from 0 (closed) to 1 (open), mapped by `gripper_scale`
to finger targets `-0.01 + 0.054 v` metres; `joint_state` reports the same
normalization of the measured primary finger (about 0.185 when the fingers touch).
Each arm `stop` holds the measured position; a gripper `stop` keeps its commanded
grip, so a held object stays squeezed.

Cameras are RGB, as upstream's evaluation renders them, or RGB-D in the depth
configuration below. They are the head camera (Gemini 345Lg model, fx ≈ 288.13 px) and
two D435 wrist cameras (fx ≈ 397.04 px), all 640×480,
rendered through RoboDojo's tiled render products. Isaac's symmetric frustum puts
the principal point at the image center, (319.5, 239.5) in ROS pixel coordinates.
The rig `robodojo/robodojo-arx-x5` holds the X5 chains from `X5A.urdf`, mounts
at (∓0.3, −0.45, 0.765) with a 90° yaw, RoboEnv's TCP (the pad center, 0.15 m
ahead of `link6`) and the measured camera mounts.

With its standard devices, not in the `basic` or `depth` configuration, the rig ships
`kinex.lib.perception` (`lib/perception.py`, `.kinex/lib/perception.py` in an exported workspace),
uni-robot-agent's `locate(method=triangulate)` for these cameras:
`triangulate(views, frame)` takes the pixels of one point in two or more views, each `(camera, [u, v])`
or `(camera, [u, v], image or stamp)`. Each ray comes from the camera's profile geometry and its tf pose
at that stamp, so a wrist camera serves from any arm pose. The point is valid when the rays are at least
1° apart, it lies in front of every camera, and it reprojects within 3 px of every pixel.

The flange is `link6`, upstream's `ee_link` and cuRobo's tool frame, and `pose_state` reports its
measured pose in `<arm>/base`. Move goals control the TCP, `<gripper>/tcp`, as uni-robot-agent's
`move` controls its grasp site.

### Goals

Each device takes goals, the stepped, closed-loop counterparts of uni-robot-agent's `move`, `grasp` and
`release` tools. They are `custom_interfaces/action/Json` actions of this backend's own
(`kinex_sim_robodojo.move` and `grip`): a goal, its feedback and its result are JSON objects, the backend
advances each goal inside its step, and the world only carries them (see
[Actions](adding-a-simulator.md#actions)). A goal ends other goals of its device, and a stop ends it.
Nothing plans a path around objects.

`<arm>/move` takes uni-robot-agent's `move` parameters for one arm,
`{pose, profile, position_tolerance_m, orientation_tolerance_rad, max_steps}`: the pose of
`<gripper>/tcp` to reach, `[x, y, z, qx, qy, qz, qw]` in `<arm>/base`; a gear, uni-robot-agent's motion
profile; tolerances that replace the gear's, 0.001–0.1 m and 0.02–0.5 rad; and the most frames the goal
takes, 1–24 and 24 by default.

| `profile` | Most a 0.04 s frame moves the TCP | Reached within |
|---|---|---|
| `standard` (default) | 4 cm | 0.03 m, 0.1 rad |
| `approach` | 4 cm | 0.02 m, 0.05 rad |
| `precision` | 1 cm | 0.005 m, 0.05 rad |

1. Each frame commands a TCP pose at most the gear's step from the measured one, uni-robot-agent's
   waypoint: the TCP turns toward the goal's orientation by at most the angle that sweeps the flange,
   0.150 m away, through half the step (0.13 rad at 4 cm, 0.033 rad at 1 cm), and moves on the straight
   line to the goal by what the sweep leaves of the step. A blocked TCP is never driven far past where
   it is.
2. cuRobo's `InverseKinematics`, the solver upstream's `ee_pose` action uses (32 seeds, self and table
   collision, 1 mm and 0.02 rad), solves the flange of that pose from the measured joints
   (`RoboDojoSim.joint_step`); when none of its solutions can be taken, one with 128 seeds and the same
   settings solves it again, as uni-robot-agent's adapter does.
   Of its solutions, the one nearest the measured joints is taken that lies within the joint limits, a
   whole turn nearer where they allow, asks no joint for more than 1 rad, and has a joint path, sampled
   every 0.025 rad, that passes cuRobo's self and table constraints. These checks follow
   uni-robot-agent's RoboDojo adapter, with the nearest solution in place of cuRobo's first. A joint
   asked for more than a frame's 0.2 rad (the X5 actuators' 5 rad/s `velocity_limit_sim`) saturates, and
   the next frame goes on from where the arm is.
3. The goal is judged after each frame, never before the first, as uni-robot-agent's `move` is. It
   succeeds, the arm holding its last step, once the TCP is within the tolerance of the pose
   (`stop_reason` `reached`) or after `max_steps` frames (`max_steps`, message "reached max_steps"),
   whose errors say how far it still is from the pose.
4. It is aborted, the arm holding its measured joints, when the TCP moves away from the pose on two
   frames running (`diverging`: its distance in metres plus a tenth of its radians grows by more than
   0.01 each time), when it stalls (over four frames it gains less than a tenth of the tolerance), or
   when a frame has no IK solution (`unreachable`). A cancel, a newer goal or a stop ends it the same way
   (`ended`). The result's `message` says why and how far from the pose it ended; the result also gives
   the measured TCP `pose`, `position_error_m`, `orientation_error_rad`, `steps` and `recovered`.
5. Before a goal is unreachable it is recovered, once, as uni-robot-agent recovers a move
   (`kinex_sim_robodojo.region`): the TCP goes on to a pose the arm can reach within 80% of the
   goal's tolerance, and still arrives when it is within the tolerance of the goal's own pose. That
   pose is the TCP of one of the IK's own solutions for the goal, converged or not, that
   cuRobo's constraints allow; failing one, of the joints nearest the measured ones that a search
   finds, by the arm's kinematics from `X5A.urdf` and within 10 s of wall time. No world time passes
   meanwhile.

A frame that moves a TCP too far ends goals as `unsafe_motion`: more than 0.2 m or 0.8 rad for an arm
its goal moves, which ends that goal; more than 0.05 m or 0.25 rad for an arm that holds still, which
ends every goal that runs, a gripper's too.

`<gripper>/grasp` takes `{}` and closes: it commands the closed value, which lies past the finger stop
to squeeze, and keeps it. `<gripper>/release` takes `{opening}`, a grip joint position, fully open by
default. A goal ends once the opening is within 0.02 of its target, about 2 mm of pad gap (`reached`:
for a grasp, nothing is between the fingers), once the fingers gain less than 0.005 on it over eight
frames (`stalled`: for a grasp, they have stopped on an object), or after 24 frames (`late`). A grasp
succeeds reached or stalled, a release only reached. The result gives `stop_reason`, the measured
`opening` and `steps`. The arms hold while a gripper goal runs: nothing moves them.

Sending a goal advances no time. Send both arms their goals, then wait on the world: a wait ends on the
frame a goal ends and names it, so waiting for goals costs no more budget than their motion (see the
shared [wait semantics](README.md)). A client that gives up on a goal cancels it.

The joints run through `EvalEnv.take_action`, as joint targets do, so evaluation and scoring are
unchanged. To solve them, kinex-sim builds upstream's cuRobo planner for the arms, as upstream's
evaluation does by default (`need_planner`). RoboEnv's adapter, whose arms took joints only, skipped it.
`robodojo-native` carries cuRobo's `[cu12]` extra (`cuda-core`), as upstream's install does, because the
planner compiles its kernels through it.

### Basic configuration

`--config basic` commands the devices on topics instead, with upstream's own action entries: each arm
takes `<arm>/joint_target` and each gripper `<gripper>/target`, and no device serves a goal. The rig's
`configurations/basic/profiles.yaml` holds these profile changes.

### Depth configuration

`--config depth` makes all three cameras RGB-D, through upstream's own depth capture.
Every camera gets the `distance_to_image_plane` annotator, which upstream's
`env_cfg/camera/camera_config.yml` carries commented out. Depth is metres along the optical
axis, rendered with each image. Each camera serves it as `<camera>/depth`: 16UC1 millimetres
registered to the image, with the same stamp and frame. The profiles state the cameras'
clipping ranges as valid ranges:

- head camera: 0.005–10 m;
- wrist cameras: 0.001–10 m. Their 0.0001 m near clip is below the millimetre encoding.

0 marks a missing sample. The rig's `configurations/depth/profiles.yaml` holds these profile
changes, so setup and launch must name the same configuration. Upstream's policy observations
are unchanged: `vision.depth` stays off.

## Timing and episodes

Physics runs at 250 Hz and one control frame is ten substeps, 0.04 seconds. Each
frame holds the commanded targets and calls upstream's `EvalEnv.take_action` with
the joint action dictionary, which interpolates the arms and fingers within the
frame, as RoboEnv does. Success is native completion and success. The reward is
upstream's episode score (`EvalEnv.run_eval`): 1 for a success; a failed graded task
keeps upstream's 0..100 score, as a reward in [0, 1); other failures score 0. The horizon is the native
`step_lim` unless a maximum is given. `--early-stop` ends the episode where `EvalEnv`
does: at its completion, or when a task constraint fails (a `query` or `trigger_query`
that held on more steps than its task allows, such as `push_T`'s lifted T), with that
constraint as the cause. Without it a failed constraint keeps success false and the
episode runs to its horizon.

The episode loads saved layout `ids[seed % len(ids)]`, retried up to three times
when it fails upstream's settle check, so a seed keeps its layout.

**Instruction.** The episode's instructions are upstream's, drawn for its layout, so a seed
keeps them too. Upstream's policy receives two, and the
[task description](devices.md#task-description) holds both: `instruction`, a sentence in each
observation, and `detailed_instruction`, the task's description and scoring rubric in its
`extra_info`.

## Recording and checks

With `--record`, spectators `third_person`, `left_wrist` and `right_wrist` copy the
optics of the head and wrist cameras at 960×720 and follow them. One native render fills every
camera's and every spectator view's buffers, so a step renders once however many of them are read:
a frame of the three views takes about 86 ms, where a render for each took 220 ms.

```bash
pixi run --frozen -e robodojo-native smoke                       # native checks via worker processes
KINEX_SIM_ALL_TASKS=1 pixi run --frozen -e robodojo-native smoke  # every task boots Isaac (hours)
pixi run --frozen -e robodojo test-ros
kinex-sim demo robodojo --timeout 1800
```

The native checks run the backend in worker subprocesses because Isaac's shutdown
ends its process. `tests/kinex/native/test_robodojo.py` (with `KINEX_PYTHON`) checks the
exported rig's arm FK, finger links and camera mounts against Isaac.
