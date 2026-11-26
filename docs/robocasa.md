# RoboCasa

RoboCasa runs in its own Python 3.12 / ROS Jazzy environment with the official
RoboCasa365 kitchens, tasks, success predicates and PandaOmron mobile manipulator: a
Franka Panda on a lift torso on an Omron holonomic base. The catalog holds all 365
tasks (65 atomic, 300 composite), including upstream's 300 training and 50 evaluation
tasks; a run's flags select the kitchen and objects (see
[Scenes](#scenes-split-layout-and-style)). By default the robot is upstream's own: its
composite controller, an operational-space arm commanded by flange poses, and RGB cameras.
`--config basic` gives the arm and torso absolute joint targets and the cameras depth (see
[Basic configuration](#basic-configuration)):

```bash
kinex-sim prepare robocasa
kinex-sim tasks list --simulator robocasa
kinex-sim launch robocasa/pick-place-counter-to-cabinet --execution-mode stepped --record ../ros_robocasa/run
kinex-sim launch robocasa/pick-place-counter-to-cabinet --config basic --execution-mode stepped
```

`robocasa` selects `robocasa/pick-place-counter-to-cabinet`; task IDs are the
kebab-cased native class names, which also resolve as bare names. Upstream RoboCasa
is pinned to `456174f62b89b8fca99eaaf33949c29fec9cfc2a` and robosuite to
`5ce6643f3092639d08f7b0f90ed1c6a84f50552c` (1.5.2), exported into the same
generated tree. Three patches make object models load from a private temporary
file instead of writing into the shared asset cache, make counter placement
regions deterministic per seed, and accept GPU-UUID CUDA masks in robosuite's EGL
check. The environment pins NumPy 2.2.5 and MuJoCo 3.3.1, which RoboCasa asserts;
its learning and teleoperation extras (torch, LeRobot) are not installed.

## Assets

Every task samples kitchens from the fixture, texture and generative-texture
archives and objects from the objaverse, lightwheel and aigen registries, so one
preparation mounts all six upstream archives (about 23 GB) for every task. They are
cached once under `.runtime/assets/robocasa` and linked into
`.runtime/robocasa/robocasa/models/assets`, beside the files upstream tracks there;
ABC mounts the objaverse and lightwheel packs from the same cache. Upstream
publishes no checksums. With `KINEX_SIM_ASSET_SEED=DIR`, extracted archives at
`DIR/robocasa/<root>/` (`textures`, `generative_textures`, `fixtures`, `objaverse`,
`aigen_objs`, `lightwheel`) are verified file by file against their sha256
receipts `DIR/robocasa/<archive>.json` and moved into the cache:

```bash
KINEX_SIM_ASSET_SEED=DIR kinex-sim prepare robocasa
```

## Graph

| Device | Type | Interfaces |
| --- | --- | --- |
| `world` | world | `world/now`, `world/wait`, `world/stop`, `/clock`, `world/spectator/*` while recording |
| `base` | locomotion | `base/velocity`, `base/odometry`, `base/stop` |
| `torso` | joint | `torso/joint_state`, `torso/joint_target`, `torso/stop` |
| `arm` | joint | `arm/joint_state`, `arm/pose_target`, `arm/pose_state`, `arm/stop`; with `--config basic`, `arm/joint_target` instead of `arm/pose_target` and `arm/pose_state` |
| `gripper` | gripper | `gripper/joint_state`, `gripper/target`, `gripper/stop` |
| `camera_left`, `camera_right`, `camera_wrist` | rgb; rgbd with `--config basic` | `<camera>/{image,camera_info}`; with `--config basic`, also `<camera>/depth` |

The cameras are upstream's `robot0_agentview_left`, `robot0_agentview_right` (on
the torso lift) and `robot0_eye_in_hand`. The rig's frames chain the robot:
`torso` mounts on `base:base`, the arm on `torso:top` and the gripper and wrist
camera on `arm:flange`. The base is an unlocalized root; nothing relates it to
`world`.

## Base, torso, arm and gripper

All parts use upstream's controllers at its 20 Hz control rate. By default they are
upstream's composite controller as its `create_env` loads it (`default_pandaomron.json`),
unchanged: `OSC_POSE` for the arm and `JOINT_POSITION` on delta input for the torso,
with the native base and gripper commands. Each step the backend turns the arm's and
torso's targets into the deltas that reach them, which upstream's controllers then
take as a policy's. `--config basic` gives both joint parts absolute targets instead
([Basic configuration](#basic-configuration)).

- `base/velocity` (`geometry_msgs/msg/Twist`) carries upstream's normalized base
  action: `linear.x`, `linear.y` and `angular.z` in [-1, 1], in the body frame (the
  native kitchen controller already rotates them by the base's heading). Other
  fields must be zero. The base stops 0.5 s of world
  time after the last twist, so re-send to keep moving; `base/stop` commands zero.
  `base/odometry` (`nav_msgs/msg/Odometry`) reports the measured body twist in m/s
  and rad/s at the chassis origin (21 cm ahead of the yaw axis) in `base/base`, and
  the planar pose relative to the episode start in `base/odom`.
- `arm/pose_target` takes a `trajectory_msgs/msg/MultiDOFJointTrajectory` of absolute
  `flange` poses in `arm/base` (robosuite's `robot0_right_hand` in `robot0_link0`, the
  frame upstream's OSC works in), timed like joint trajectories and interpolated
  linearly and by slerp at each 0.05 s step. `arm/pose_state` reports the measured
  flange, and `arm/joint_state` the joints `robot0_joint1..7` (radians). OSC tracks the
  grip site (`gripper0_right_grip_site`, the profile's `gripper/tcp`), so the backend
  converts the flange target to that site and sends `OSC_POSE` the delta that reaches
  it. Upstream's clip still limits each step to 5 cm and 0.5 rad per axis, and a target
  the arm cannot reach is approached, not refused: the kitchen's cabinets stop the
  arm as they stop upstream's.
- `torso/joint_target` takes position-only `JointTrajectory` points for
  `mobilebase0_joint_torso_height` (metres, 0 to 0.34), as for the other simulators.
  Each step the backend sends upstream's torso controller the delta from the measured
  lift to the target, which its clip limits to 5 cm.
- `gripper/target` (`std_msgs/msg/Float64`) is robosuite's integrated gripper
  command from -1 (open) to +1 (closed). Every control step the backend moves it by
  the native 0.2 toward the target while that gets strictly closer, so -1 and +1 are
  reached exactly within 10 steps; a native policy's gripper action `a` maps to the
  target `sign(a)`. `gripper/joint_state` reports this command, not the measured
  fingers: the URDF fingers mimic the position their actuators are commanded to,
  so a closed grip may be stalled on an object. The command starts at 0, where
  upstream's reset leaves it.
- **Base mode.** Upstream's action ends with a mode flag, its policies' `control_mode`.
  In base mode the arm's goal moves from its previous goal, which keeps the hand where
  it is in the moving arm base; in arm mode it moves from where the hand is. No device
  carries the flag, so kinex-sim runs base mode exactly while the base's velocity
  command is nonzero, as upstream's demonstrations switch to it to drive. The arm's
  delta is then from its previous target, so a move the clip cut short resumes once
  the base stops. A policy step in base mode with a zero base action runs here as an
  arm-mode step.
- **Policies.** A policy trained on upstream's actions maps onto these targets, one
  per 0.05 s step. The arm's target is the grip site measured in `arm/base` (in base
  mode, the previous target) moved by the scaled delta, `p + 0.05 · d[:3]` and
  `R(0.5 · d[3:6]) · R`, and mapped to the flange. The torso's is the measured lift plus
  `0.05 · d`, the base's twist is the base action, and the gripper's target is the sign
  of its action. Sent so, upstream's controllers take the same goals as an independent
  upstream kitchen given the deltas, to 1e-9, and the robot's joint positions agree to
  1e-7 (native suite). MuJoCo solves every contact together, so rounding reaches the
  rest of the kitchen as it does upstream's own: a change of 1e-14 in upstream's torso
  input moves a distractor settling in a cabinet by 4e-6 in one step.
- **Start.** Each episode starts at upstream's stowed posture with its 0.02 rad
  initialization noise and the torso at 0. Upstream settles every new kitchen with
  zero actions before the episode starts, and the controllers stay as that leaves
  them. Until its first command the arm idles as upstream's does, on zero deltas, and
  the torso holds its starting lift.

Each `stop` holds the arm's measured flange pose or the torso's measured lift, keeps the
gripper's applied grip target (keeping any grip), or commands zero base velocity.

## Basic configuration

`--config basic` gives the arm and torso absolute joint targets, and the cameras depth:

```bash
kinex-sim setup robocasa/close-fridge --config basic --directory ../my-robocasa-basic
kinex-sim launch robocasa/close-fridge --config basic --execution-mode stepped
```

- The arm's controller is robosuite's `JOINT_POSITION` in place of `OSC_POSE`, and
  `arm/joint_target` takes position-only `JointTrajectory` points for
  `robot0_joint1..7` (radians) in place of `arm/pose_target`; there is no
  `arm/pose_state`.
- Both joint parts take absolute input once the kitchen has settled: they take native
  delta input during upstream's zero-action settling, so the posture holds. A target
  is the controller's goal at once, without upstream's 5 cm torso clip.
- The gripper's integrated command starts at the measured fingers' instead of 0.
- A `stop` holds the arm's measured joint positions.
- The cameras are RGB-D ([Cameras](#cameras-recording-and-demo)).

The base and the gripper's target are the same in both configurations.

## Episode timing and tasks

Each action interval advances 0.05 simulation seconds (25 MuJoCo substeps). The task,
seed and scene options (see [Scenes](#scenes-split-layout-and-style)) determine the
episode; building the kitchen takes about 10 s. The default horizon is upstream's dataset horizon for the task (for example 750 steps
for `PickPlaceCounterToCabinet`), or none when upstream has no dataset; the world
node's `max_steps` parameter overrides it. Success is the task's native predicate, scored
as upstream's evaluation scores it (`robocasa/wrappers/gym_wrapper.py`): evaluated again
after each step, once the kitchen has updated its fixtures' state, and that value is both
the success and the reward. As there, the predicate also runs inside the step, so tasks
that count calls in it (PreheatPot's heat timer, for example) count twice per step.
`--early-stop` ends the episode at its first success, and
RoboCasa has no failure termination.

The [task description](devices.md#task-description)'s `instruction` is the episode
metadata's language instruction (`get_ep_meta()["lang"]`), which depends on the kitchen and
objects the seed and scene options pick.

## Scenes: split, layout and style

A run selects its scene as upstream does, with launch flags for upstream's own arguments:

| Flag | Values | Default | Upstream argument | Selects |
|---|---|---|---|---|
| `--split` | `pretrain`, `target`, `all` | none | `create_env(split=...)` | the kitchens the seed picks from and the object instances (below) |
| `--layout-id` | 1–60 | the seed picks | `layout_ids` | the floor plan: walls, counters, cabinets and where each appliance stands (`kitchen_layouts/*/layoutNNN.yaml`) |
| `--style-id` | 1–60 | the seed picks | `style_ids` | the materials and appliance models: cabinet, counter, floor and wall textures, which sink, stove, fridge and so on (`kitchen_styles/*/styleNNN.yaml`) |

The split is upstream's `create_env` preset, passed to the kitchen unchanged:

| `--split` | Kitchens the seed picks from | Objects (`obj_instance_split`) |
|---|---|---|
| `target` | the 10 pairs `(i, i)`, i in 1–10 | `target`: per category, the held-out last instances (up to 5) |
| `pretrain` | layouts 11–60 × styles 11–60 (2,500) | `pretrain`: the instances before those |
| `all` | layouts 1–60 × styles 1–60 | every instance |
| none | as `all`, upstream's `create_env(split=None)` | every instance |

The seed then works exactly as upstream's: it seeds the kitchen's own generator, which picks
the kitchen from the split's, the fixtures' initial state, object categories and instances
within the split, their placements, the robot's base pose and the instruction.
`--split target --seed 3` builds the same episode as upstream's
`create_env(task, split="target", seed=3)`; kinex-sim also seeds Python's and NumPy's process
generators, which upstream leaves as they are. A layout or style id given overrides the
split's kitchens, as upstream's `collect_demos.py --layout/--style` does: an id not given
ranges over the split's group (1–10 for `target`, 11–60 for `pretrain`, 1–60 otherwise), and
the objects stay the split's. The seed picks only among the kitchens a task allows; an id the
task excludes fails to launch with an error naming the ids it excludes. Each episode record
states `split`, `layout_id` and `style_id`, the kitchen the seed picked or the ids pinned.

```bash
kinex-sim launch robocasa/close-fridge --split target --seed 2                         # a seed-picked target kitchen
kinex-sim launch robocasa/close-fridge --split target --layout-id 4 --style-id 4 --seed 2
```

### Training and evaluation splits

Upstream's benchmark trains on 300 tasks in its training kitchens and evaluates on 50 tasks
in 10 held-out kitchens, with held-out objects. kinex-sim ships no datasets; the split is the
scene a run launches:

| | Train: pretraining (`pretrain300`) | Test: target (`TARGET_TASKS`) |
|---|---|---|
| Tasks | 300: 65 atomic and 235 composite | 50: 18 atomic-seen, 16 composite-seen, 16 composite-unseen |
| Flag | `--split pretrain` | `--split target` |
| Kitchens | layouts 11–60 × styles 11–60 (2,500) | the 10 pairs `(i, i)`, i in 1–10 |
| Objects | pretraining instances | held-out target instances |
| Human data | 100 demonstrations per task (`pretrain_human300`) | 500 demonstrations per task (`target50`) |

Upstream evaluates each task on 50 randomly sampled scenarios and reports the average binary
success; launching `--split target` with 50 seeds samples kitchens the same way, though the
seeds upstream's evaluation scripts use are not published. `--early-stop` ends each at its
first success.

The 34 seen test tasks are also training tasks, and the 16 unseen ones are not. Layouts and
styles 1–10, and target objects, are the test scene: a training run should never use them.
All 350 tasks are in the catalog, which also holds 49 composite tasks in neither split. The
300 training tasks are upstream's registry, `PRETRAINING_TASKS["pretrain300"]`:

```bash
pixi run --frozen -e robocasa python -c "from robocasa.utils.dataset_registry import PRETRAINING_TASKS; print(*PRETRAINING_TASKS['pretrain300'], sep='\n')" | grep -v '^WARNING'
```

The 50 test tasks (`TARGET_TASKS`, native names, which also resolve as task IDs):

| Group | Tasks |
|---|---|
| Atomic, seen (18) | CloseBlenderLid, CloseFridge, CloseToasterOvenDoor, CoffeeSetupMug, NavigateKitchen, OpenCabinet, OpenDrawer, OpenStandMixerHead, PickPlaceCounterToCabinet, PickPlaceCounterToStove, PickPlaceDrawerToCounter, PickPlaceSinkToCounter, PickPlaceToasterToCounter, SlideDishwasherRack, TurnOffStove, TurnOnElectricKettle, TurnOnMicrowave, TurnOnSinkFaucet |
| Composite, seen (16) | DeliverStraw, GetToastedBread, KettleBoiling, LoadDishwasher, PackIdenticalLunches, PreSoakPan, PrepareCoffee, RinseSinkBasin, ScrubCuttingBoard, SearingMeat, SetUpCuttingStation, StackBowlsCabinet, SteamInMicrowave, StirVegetables, StoreLeftoversInBowl, WashLettuce |
| Composite, unseen (16) | ArrangeBreadBasket, ArrangeTea, BreadSelection, CategorizeCondiments, CuttingToolSelection, GarnishPancake, GatherTableware, HeatKebabSandwich, MakeIceLemonade, PanTransfer, PortionHotDogs, RecycleBottlesByType, SeparateFreezerRack, WaffleReheat, WashFruitColander, WeighIngredients |

Nine test tasks exclude some test kitchens, so with `--split target` the seed picks only
among the ones they allow, as upstream's does:

| Test task | Test kitchens `(i, i)` it builds |
|---|---|
| DeliverStraw, GetToastedBread, StoreLeftoversInBowl, ArrangeBreadBasket, GarnishPancake, PortionHotDogs | 2, 4, 7, 8, 9, 10 |
| RecycleBottlesByType | 2, 4, 7, 9, 10 |
| SeparateFreezerRack | 2, 7, 9 |
| WaffleReheat | all but 9 |

Likewise 116 training tasks exclude some training kitchens (RetrieveIceTray builds 450 of
the 2,500), and 30 of them exclude layout or style 11.

## Cameras, recording and demo

Observation cameras are 256×256 RGB, as upstream's evaluation observes them, in frames
`<camera>/base` (vertical FOV 60° for the agentview cameras and 75° for the wrist), with
principal point `(width-1)/2, (height-1)/2`. With `--config basic` they are RGB-D: RGB
keeps robosuite's antialiased rendering, and depth comes from a separate single-sample
renderer at pixel centres, in millimetres along the optical axis. The near clip plane
scales with each kitchen's extent (about 2–3 cm). With `--record`, spectator views render
`robot0_agentview_left` and `robot0_eye_in_hand` into `third_person.mp4` and `wrist.mp4`.

```bash
kinex-sim demo robocasa --record ../ros_robocasa/demo
kinex-sim demo robocasa --config basic
kinex-sim demo robocasa/prepare-veggie-dip --execution-mode realtime --seed 2
kinex-sim launch robocasa/close-fridge --split target --layout-id 4 --style-id 4 --seed 0
```

The finite ROS-only demo moves the flange 4 cm forward and back by pose (with
`--config basic`, the first arm joint by a joint target), lifts the torso, closes and
opens the gripper, turns the base and checks that it stops, and checks all three cameras.
It reports measurements as JSON and shuts down the graph.

Installed sources live in `.runtime/robocasa`, assets in `.runtime/assets/robocasa`,
and the ROS workspace in `.workspaces/robocasa`. The rig is rebuilt with
`pixi run --frozen -e robocasa python tools/robots/build_robocasa.py`.

## Checks

```bash
pixi run --frozen -e robocasa smoke      # representative tasks, both configurations' controls and rendering
pixi run --frozen -e robocasa test-ros
```

Native checks cover an atomic and a composite evaluation task and a task without a
dataset horizon, not all 365; that a split and seed build the same episode as upstream's
`create_env` for the target and pretraining splits; kitchen ids overriding a split;
invalid scene options and excluded kitchens; the default configuration's rollout beside an
independent upstream kitchen under its composite controller, in arm and base mode, and its
flange pose tracking; and the basic configuration's joint controls and depth. The graph
tests launch both configurations. `tests/kinex/native/test_robocasa.py` (with `KINEX_PYTHON`)
compares the exported rig's FK through base, torso and arm, and the reported flange pose,
with MuJoCo.
