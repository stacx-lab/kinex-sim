# BEHAVIOR: known environment issues

This page lists the problems known in BEHAVIOR-1K's 100 activities of the 2026 challenge as rosbridge runs them: the `behavior` simulator, BEHAVIOR's R1Pro with grippers and assisted grasping as the official evaluator configures it, with upstream pinned at `a8247a8cc1633fe1ca0cc66aa07243d46c64f155`. [BEHAVIOR](behavior.md) describes how rosbridge runs the simulator, including the patches preparation applies. Problems of a harness that runs agents, outside this repository, are not listed.

Most entries were found while rosbridge also ran the R1Pro with Sharpa hands (`behavior-dex`, since removed) and are measured on whichever R1Pro the entry names; both share the evaluator's base, torso, arms, cameras and house. Entries about the hands alone left with that simulator: handles a hand cannot grip, a toy beyond a hand's reach, and whether hands can solve each activity.

## What rosbridge fixes

rosbridge reproduces the pinned evaluator: its physics, transition rules, success, horizon and partial credit, so that results compare with upstream's. By that rule:

1. **rosbridge's own protocol** (how it drives, ends and reports an episode) is fixed wherever it departs from the evaluator's or hides what the evaluator would show.
2. **An upstream defect** is corrected at preparation only when the correction leaves the scored task bit-identical (physics, success, partial credit and simulation time), restores what upstream evidently intends or protects the host, and admits no other reading. `0003-goal-option-literals.patch`, which evaluates each distinct goal literal once, is one: the same `q_score`, without hours of repeated evaluation.
3. **Any other upstream defect** is left as the evaluator has it, documented here and worth reporting upstream, even when it makes an activity impossible.

Three changes on `main` predate this rule and change what is scored: the slicer and vacuum patches (5.1, 5.2) and the locked doors with the particles they cut off (6.1). They are kept as deliberate exceptions to rule 3 and marked *departs*: on the activities they touch, results do not compare with the evaluator's.

## How to read this

**Evidence.** Each entry says how it is known:

- *measured*: a native run in Isaac Sim through rosbridge's backend;
- *read*: source code or data files;
- *inferred*: reasoning from measured or read facts, not itself run;
- *recorded*: seen in an agent episode.

Unless stated otherwise, native runs used seed 0, which loads public test instance 301. Upstream paths are relative to `.runtime/behavior/`, the prepared BEHAVIOR-1K tree.

**Status.**

| Status | Meaning |
|---|---|
| rosbridge | Fixed in rosbridge's protocol (rule 1) |
| prepared | Corrected at preparation (rule 2) |
| departs | rosbridge changes the evaluator's rule; kept as an exception to rule 3 |
| upstream | Needs a change in BEHAVIOR-1K or OmniGibson; rosbridge leaves it as the evaluator has it (rule 3) |
| open | A protocol fix (rule 1) is due and not made, or no fix is proposed |
| unknown | The cause is not established |
| no longer applies | The cause is gone from rosbridge |
| left as is | Known and deliberately unchanged, with the reason |
| property | Not a defect: how the environment works, worth knowing |

Nothing has been reported to BEHAVIOR-1K yet. Section 9 lists the reports worth filing.

## 1. All issues at a glance

| # | Issue | Status |
|---|---|---|
| 2.1 | The base drive has no effort limit and tips the robot over | upstream |
| 2.2 | Joints thrown tens of radians out of range, or NaN | unknown |
| 2.3 | One-step velocity spikes of the base's virtual joints | property; upstream |
| 2.4 | Doors get stiffer with mass, and creep | upstream |
| 2.5 | Lids and flaps fall shut or fall open | upstream |
| 2.6 | Torso targets that stall are collisions | property |
| 2.7 | "Car windows have no collision" | not reproduced |
| 3.1 | The first removed visual particle crashed the world, and start poses were wrong | rosbridge |
| 3.2 | Particle removers followed a tool only when a camera was read | no longer applies; upstream |
| 3.3 | A broken attachment joint aborted the process | no longer applies; upstream |
| 3.4 | A physics failure ends the episode without a score | rosbridge (ends cleanly); open (no score) |
| 3.5 | Native worker segmentation fault | unknown |
| 4.1 | Wait replies did not say that or why the episode ended | rosbridge |
| 4.2 | A tilted base looks upright in odometry and TF | open |
| 4.3 | A client cannot read the episode's step limit | open |
| 4.4 | Device texts do not say how cutting, toggling and removal work | property |
| 5.1 | Any part of any slicer cuts; slicers cut again on their halves | departs (`0005`); upstream |
| 5.2 | A vacuum standing flat removes nothing | departs (`0006`); upstream |
| 5.3 | Oven racks count for `open` | upstream |
| 6.1 | Goal particles beyond a locked door | departs; upstream |
| 6.2 | The only route to the sink passes a counter flap | open |
| 7.1 | No official instruction for half the activities | upstream |
| 7.2 | Four goals disagree with their instructions | upstream |
| 7.3 | Two goals bind one of two identical fixtures | upstream |
| 7.4 | Wording errors in official instructions | upstream |
| 8.1 | Success fires at first contact | left as is |
| 8.2 | `OnTop` depends on yaw under an overhang | upstream |
| 8.3 | `nextto` is brittle | upstream |
| 8.4 | `inside` and `filled` are tight in two activities | upstream |
| 8.5 | Goal semantics worth knowing | property |

Section 10 lists what has not been investigated. The largest item is whether each activity can be solved at all through the R1Pro's commands.

## 2. Physics fidelity

### 2.1 The base drive has no effort limit and tips the robot over

- **Symptom.** The R1Pro tilts and often falls over when its base drives against furniture, a door or a wall.
- **Cause** (*measured*). OmniGibson writes 1000 N to the base's three virtual drive joints at initialization, through the articulation view. The stop and play that the evaluator uses to set the 250 kg base mass (`OmniGibson/omnigibson/eval/evaluator.py`, copied in `R1Pro.configure`) discards it: PhysX then holds 3.4e38, and `JointPrim.max_effort` hides that by answering 100. In contact the drive pushes with 100 to 540 kN.
- A second defect behind it: with the intended 1000 N restored, the base reaches 0.45 m/s of a 0.75 m/s command, because its wheels are fixed spheres that slide with friction 0.5 and need 1248 N to move. The evaluator's robot drives only because the limit is lost.
- **Evidence** (*measured*). Backing into a fridge, the gripper R1Pro falls over and comes to rest at 82.6°. On the R1Pro with hands, whose base and drive are the evaluator's, 123 drives into furniture, walls and doors in four scene and seed combinations tilted the base more than 10° in 90 and left it lying beyond 45° in 46. Open-floor driving stays within 4.1°.
- **Status.** Upstream. A drive limit of 500 N and 150 N m on frictionless wheels kept the same 123 drives at or below 3.9°, but it changes the robot's physics, so `behavior` keeps the evaluator's. The tilt is not visible to a client either (4.2).

### 2.2 Joints thrown tens of radians out of range, or NaN

- **Symptom.** In one control step a joint jumps far past its limit, or the base's orientation becomes NaN.
- **Evidence.** *Recorded* in agent episodes: `torso_joint2` at 24 rad after base bursts on a tipped robot; arm joints at −61 and −1334 rad.
- **Not reproduced.** No reachable state diverged in 1,234 native scenarios (falls from 30° to 150° in 8 directions and 7 postures, drives into furniture, 20 burst sequences on a tipped or lying robot, pushes). All 15 runaways produced started with the robot placed inside furniture, which no client can do.
- **Suspected cause**, untested: an ill-conditioned contact island (a 250 kg base on virtual links of 1e-6 kg, gravity-free arm links wedged under it, one velocity iteration). Its likeliest trigger is the unlimited base drive (2.1).
- **Status.** Unknown. A NaN that makes OmniGibson's step raise ends the episode (3.4). A runaway that stays finite leaves a broken robot running until the horizon.

### 2.3 One-step velocity spikes of the base's virtual joints

*Measured.* The base's six virtual joints are an x, y, z, rx, ry, rz chain, singular where ry = ±90°. When a fall carries the base through that pose, rx and rz spike for one step, up to 1690 rad/s. This happened in 15 reachable scenarios, with no lasting effect. Joint velocity therefore cannot serve as a divergence detector.

### 2.4 Doors get stiffer with mass, and creep

- **Cause** (*read and measured*). OmniGibson gives every revolute joint friction 0.2 and every prismatic joint 0.3 (`OmniGibson/omnigibson/objects/dataset_object.py`), which PhysX multiplies by the load the joint carries. The effort needed to start a part grows with its mass, and below it the part creeps instead of holding.
- **Evidence.** Effort at which a part has moved 0.02 rad or 5 mm, on 55 fixed models:

  | Part | Starts at |
  |---|---|
  | Cabinet doors | 0.6 to 2.1 N m |
  | Microwaves | 1.2 to 2.1 N m |
  | Fridges (`dszchb`, `petcxr`, `juwaoh`, `jtqazu`) | 2.6 to 6.6 N m |
  | Washer, dryer | 3.7, 4.5 N m |
  | Oven `ffitak` | 5.0 N m |
  | Room doors | 9.7 to 14.6 N m |
  | Sliding door | 33.7 N |
  | Drawers | 5.7 to 16 N |

- **How much it matters.** The gripper's assisted grasp is a rigid constraint (it held 200 N), so a gripped handle does not slip. Whether the arms' drives open the heaviest parts through a grasp was not measured.
- **Status.** Upstream. A load-independent friction would change every articulated object's physics.

### 2.5 Lids and flaps fall shut or fall open

- *Measured.* A car trunk set 25% or 50% open shuts within 1 s. Oven `ffitak` and both dishwasher doors fall open from 50%. The bar counter's flap (6.2) and the toilet seat and lid shut. The assets have no spring or detent.
- **Affects.** `loading_the_car`, `unloading_the_car`, `carrying_in_groceries`; `make_pizza`, `cook_a_brisket`; `putting_dirty_dishes_in_sink`; `tidying_bathroom`.
- **Status.** Upstream.

### 2.6 Torso targets that stall are collisions

*Measured* on the R1Pro with hands; the gripper R1Pro stalls at the same postures. Every stall is a collision with the 100 N m drive saturated against it, not a weak drive. From the start posture, with the arms hanging, the torso follows the evaluator's squat only halfway: the hanging arms meet the base. With both arms raised forward every posture tracks exactly with efforts below 5 N m. A client lowering the torso raises the arms first.

### 2.7 "Car windows have no collision": not reproduced

All six glass links of both car models have a collision mesh, and 34 to 42 of 54 rays through each glass link hit it. The body's hulls also lie in the window boxes, so a ray that reports only the closest hit names the body, which may explain the earlier reading. Not tested: whether a fast arm can tunnel through the thin shell.

## 3. Simulation robustness

### 3.1 The first removed visual particle crashed the world, and start poses were wrong

- **Cause** (*measured*). An earlier version of rosbridge's `0004-particle-handle-updates.patch` stopped updating the physics handles after a visual particle was removed. Deleting any prim drops the handle of every articulation view, so the next joint read raised and joint or pose writes were silently discarded.
- **Symptoms.** The first particle a cleaning tool removed ended the episode, in the 11 activities whose goals remove visual particles. Restoring an instance lost the robot's start pose: `sweeping_garage` started 1.04 m and 2.02 rad off, `clean_a_patio` 4.8 m, `putting_dirty_dishes_in_sink` 1.4 m, `scrubbing_bathroom_floor` 0.10 m.
- **Status.** rosbridge: the patch updates the handles once per batch of particles removed, visual ones included.

### 3.2 Particle removers followed a tool only when a camera was read

- **Cause** (*measured*). Particle removers and `link.aabb`, `link.visual_aabb`, `obj.aabb` read poses from Fabric, which Kit updates when it renders. While rosbridge stepped without rendering, a broom driven 1.03 m over three particles removed none; its removal volume read 1028 mm behind.
- **Status.** No longer applies: every step is the evaluator's `env.step`, which renders. The defect remains upstream for any step that does not render.

### 3.3 A broken attachment joint aborted the process

- **Cause** (*measured*). `on_joint_break` deletes the joint prim inside Kit's physics-step callback; on a step that does not render the process aborts (`Mutex.cpp(103): Recursion not allowed`). Under the evaluator's own step the same break is handled.
- **Affects.** `make_cabinet_doors`, and by the same code `hanging_pictures`, `attach_a_camera_to_a_tripod`, `installing_smoke_detectors`.
- **Status.** No longer applies, as 3.2; upstream.

### 3.4 A physics failure ends the episode without a score

- **Symptom.** PhysX reports `Illegal BroadPhaseUpdateData` and OmniGibson's own step asserts on the base's NaN orientation.
- **Status.** rosbridge: a step that raises ends the episode with reason `error`; every later wait reply says `the episode ended because the simulation failed (<kind>)`, and the world goes on answering. The record carries the error and no `q_score`, since the failure may have broken the simulation. Open: a check of the robot's state after every step, ending the episode with its score before OmniGibson fails, was proposed and not merged.

### 3.5 Native worker segmentation fault

One recorded `canning_food` episode ended with signal 11. It was never reproduced and the cause is unknown.

## 4. What the client sees

### 4.1 Wait replies did not say that or why the episode ended

rosbridge: every wait reply at or after the end states it, for example `the episode ended because the time limit was reached`, `because the task succeeded` (with `--early-stop`), or `because the simulation failed (<kind>)`. Later waits are rejected.

### 4.2 A tilted base looks upright

The base's odometry is planar (x, y and yaw of `base_link` relative to the episode start), and nothing puts `base/odom` on TF. A base tilted against furniture (2.1) reads as upright, and the cameras' own tilt is the only sign. Open (rule 1): publishing the full attitude on the odometry and on TF, and stating a large tilt in wait replies, was proposed and not merged. Measured for that proposal: a base propped at 30.9° recovers to 0.01° by driving back out the way it came, so refusing base motion on a tilt would trap it.

### 4.3 A client cannot read the episode's step limit

The limit is 1.5 times the mean human demonstration length (`task.jsonl`). The world device gives elapsed time only, and the limit appears nowhere a client can read before the episode ends. It cannot be computed from `docs/challenge/task_data.json` either: 1.5 times its `duration` matches the real limit for none of the 100 activities, is within 1% for 53 and off by more than 10% for 30 (`tidying_living_room` 14220 against 18848 steps; `cook_brussels_sprouts` 30915 against 23492). The world node's `max_steps` launch argument sets another limit. Open (rule 1).

### 4.4 Device texts do not say how cutting, toggling and removal work

Device texts say what each device does, not the rules of the house: that only a blade cuts and the slicer that cut is inactive until 2 s after it last touched food (5.1); that a washer, dryer, dishwasher or microwave refuses to switch on while open (*read*); that one remaining particle keeps a surface "covered" (8.5). This page and [BEHAVIOR](behavior.md) state them.

## 5. Transition rules

### 5.1 Any part of any slicer cuts, and slicers cut again on their halves

- *Measured* with OmniGibson's rule: the handles of an axe, a carving knife and a table knife cut a log, an egg and a cupcake at first contact. A knife that had just cut, and was inactive, cut the next cupcake while another knife was active. A knife laid on the egg halves it had cut diced them, taking `halve_an_egg` from `q_score` 0.4 to 0. In `make_pizza` the knife's handle halved the dough and a pepperoni, which the recipe takes whole.
- **Affects.** Nine activities with a slicer and cuttable food: `chopping_wood`, `halve_an_egg`, `cook_brussels_sprouts`, `chop_an_onion`, `slicing_vegetables`, `canning_food`, `cook_cabbage`, `make_pizza`, `setting_the_table`.
- **Status.** Departs: `0005-slicer-blade.patch` cuts an object with a slicer that is active, touches it and reaches it with its blade (its slicer meta link's box, 5 mm larger); the slicer that cut is inactive until 2 s after it last touched whole or cut food. There is no speed condition. Kept as an exception to rule 3. Upstream for the rule itself.
- **Left as it is under both rules.** Food set down on a blade at rest is cut. `halve_an_egg`'s knife lies on the chopping board at load.

### 5.2 A vacuum standing flat removes nothing

- **Cause** (*read and measured*). The asset places the vacuum's removal box above its meta link; `ParticleModifier._initialize` re-places it below, 1.1 mm under the floor it stands on. The rule removes floor dust only when the vacuum is lifted 1.2 to 21.5 mm, or leans by a degree or more.
- **How much it matters.** When the robot's base pushes the vacuum, the vacuum leans 5 to 10° on the robot and OmniGibson's own rule removes the dust (2 of 2 runs). Pushed flat by a block, it removes 0 of 8 particles.
- **Status.** Departs: `0006-projection-remover-margin.patch` takes a visual particle within 1 cm of the volume, which makes removal independent of lean. It is not what makes `vacuuming_floors` possible. Kept as an exception to rule 3. Upstream for the rule itself.

### 5.3 Oven racks count for `open`

*Measured* in `make_pizza`: with the door at 0 and a rack 0.021 m out, the oven reads open and its heat is inactive. Whether a rack moves that far during real manipulation was never tested. Affects `make_pizza`, `cook_a_brisket`, `cook_a_frozen_pie`, `cook_brussels_sprouts`.

## 6. Task instance data

### 6.1 Goal particles beyond a locked door

- *Measured and read.* OmniGibson loads every door of the scene but the floors of the loaded rooms only, so a door can open onto no floor; rosbridge closes and locks such doors at load. OmniGibson's ground sampler tests that a pose is traversable, not that it lies in the covered object's room. `sweeping_garage` at seed 0 has a sand and a dust particle on the slab beyond the garage door, which is locked. `covered` holds while one particle is left, so the goal could not be reached. Instance 313 (seed 12) has one such particle, and so do 24 of 300 training instances.
- **Status.** Departs: loading removes the goal particles that the room map places in a room that is not loaded, and the record names them in `particles_removed_out_of_loaded_rooms`. Among the public instances this changes `sweeping_garage` at seeds 0 and 12 only; no locked door is part of an activity. Upstream for the sampler.
- **Limit of the rule.** It is correct for every public instance, checked on all 894 public particle places, but not by construction. It holds because the sampler's 7×7 erosion empties the three map cells between the door and the room boundary, and because the 0.1 m room map takes each cell's label from its first pixel. Instances resampled for a smaller robot, or a room map at another resolution, would break it.

### 6.2 The only route to the sink passes a counter flap

`putting_dirty_dishes_in_sink`: from the start in the dining room, the route to the kitchen sink (11.7 m, passage 0.85 to 0.95 m) passes a flap that is a jointed link of the bar counter, not a door object. OmniGibson's traversability map does not know it and reads "no route". The flap needs 13 N m to lift and shuts by itself (2.5). Nobody has lifted it with the gripper or driven through. Open.

## 7. Goal definitions and instructions

### 7.1 No official instruction for half the activities

*Read.* `docs/challenge/task_data.json` has an instruction for activities 0 to 49 only. Texts for all 100 exist in the Hugging Face dataset `behavior-1k/2026-challenge-demos`, but for activities 50 to 99 they contradict the goal in 4 (`tidying_living_room`, `unloading_the_car`, `organizing_school_stuff`, `carrying_out_garden_furniture`), omit judged conditions in 15, give wrong counts in 2 (`make_rose_centerpieces`, `installing_smoke_detectors`) and name an ambiguous target in 4 (`store_honey`, `store_batteries`, `installing_a_modem`, `tidying_bathroom`).

### 7.2 Four goals disagree with their instructions

| Activity | Defect | Evidence |
|---|---|---|
| `loading_the_car` | The BDDL has a second top-level `:goal` expression, which the parser drops, so closing the trunk is not judged. It is the only one of 1016 definitions like this | measured: success with the trunk open |
| `setting_mousetraps` | `forn (2)` means exactly two; the instruction says "at least two". Only "under" the sink works in this instance | read and measured |
| `collecting_childrens_toys` | The goal also requires the train set, which the instruction omits | measured: the six instructed items give `q_score` 0.857 and no success |
| `make_microwave_popcorn` | The goal names no microwave | read |

### 7.3 Two goals bind one of two identical fixtures

- `thawing_frozen_food`: the bread's plate must end on one particular kitchen countertop of two.
- `carrying_out_garden_furniture`: the lawn chair must end on one particular garden floor of two.

No start fact points at the bound one, so no instruction derived from the BDDL can name it. That a chair on the other lawn fails is read, not measured.

### 7.4 Wording errors in official instructions

`spraying_for_bugs` and `putting_dishes_away_after_cleaning` (six cabinets are bound); `spraying_fruit_trees` ("both trees" are two specific trees of 26); `bringing_in_wood` and `putting_shoes_on_rack` (the goal names one of two same-kind fixtures).

## 8. Judging

### 8.1 Success fires at first contact

The task succeeds at the first step all literals hold, for example with a cup still moving at 0.29 m/s. This is the evaluator's semantics and rosbridge keeps it; with `--early-stop` that step ends the episode. A stricter dwell measure belongs in a harness, as an extra recorded metric.

### 8.2 `OnTop` depends on yaw under an overhang

`OnTop` tests "nothing above" with one ray. A filter on the coffee maker's plate reads false at 3 of 8 yaws, because the maker's head overhangs the plate. On the maker's top surface, 24 of 24 placements read true, so `set_up_a_coffee_station_in_your_kitchen` stays satisfiable.

### 8.3 `nextto` is brittle

It needs a box gap of at most (sum of both boxes' sides)/18 and one of 16 horizontal rays to hit. In `polishing_shoes`, 22 of 96 side-by-side placements satisfied all three literals. In `setting_mousetraps`, a trap on the floor beside the wall-hung sink reads false. Measured in these two only; `tidying_bedroom`, `putting_shoes_on_rack`, `setup_a_bar_for_a_cocktail_party`, `laying_tile_floors` and `setting_the_table` rely on the same predicate.

### 8.4 `inside` and `filled` are tight in two activities

- `inside` tests only the inner object's box centre. Roses dropped into the vase of `make_rose_centerpieces` read inside for at most 2 of 3 at once.
- `filled` is a 20% volume threshold. In `canning_food` the bowl needs 7 of the steak's 8 cubes.
- Both logs of `setting_the_fire` must rest on one newspaper barely larger than they are. Neither this nor the three roses was shown satisfiable in one configuration.

### 8.5 Goal semantics worth knowing

- `covered` is true with one attached particle: "clean" means every particle gone, and one spray hit "covers".
- `q_score` counts only literals newly true, so literals true at load give no credit but can block success.
- Any heat source satisfies a cooking goal, and any removal method a washing goal.
- Every one of the 100 goals is expressible and judged by a working mechanic. 24 were driven to success natively by setting state, 5 fell short because of the probe's own placement, and 71 rest on reading or on a sibling activity.

## 9. Reports worth filing with BEHAVIOR-1K

None has been filed.

| # | Report | Reproduction |
|---|---|---|
| U1 | The base joints' effort limit is lost after stop and play, and the getter hides it; the wheels cannot drive at the intended limit (2.1) | Write 1000 N, `og.sim.stop(); og.sim.play()`, read 3.4e38; `JointPrim.max_effort` answers 100 |
| U2 | Stepping without rendering leaves Fabric stale for particle removers and link boxes (3.2) | A broom moved by velocity over floor particles with `render_on_step(False)` removes none |
| U3 | A joint-break callback deletes a prim inside the step callback (3.3) | Attach in `make_cabinet_doors`, break the joint on a step that does not render |
| U4 | Slicing: any link cuts, any active slicer suffices, a slicer is active again on its halves (5.1) | Handle onto the egg cuts; an inactive knife cuts while another is active; active again at 2.03 s on its halves |
| U5 | The vacuum's removal box is re-placed below its meta link (5.2) | A vacuum standing flat removes nothing; it works lifted 1.2 to 21.5 mm |
| U6 | The ground sampler accepts particles outside the covered object's room (6.1) | `sweeping_garage` instances 301 and 313 |
| U7 | Joint friction is a coefficient on the joint's load, and lids have no counterbalance (2.4, 2.5) | The table in 2.4; the trunk shuts from 50%; the oven door falls open |
| U8 | Oven rack joints count for `Open` (5.3) | `make_pizza`: door at 0, rack at 0.021 m, oven reads open, heat inactive |
| U9 | `OnTop`'s single "nothing above" ray (8.2) | Filter on the coffee maker's plate: false at 3 of 8 yaws |
| U10 | BDDL and parser (7.2) | The four activities of 7.2 |
| U11 | Instruction data (7.1, 7.3, 7.4) | `task_data.json` has no instruction for activities 50 to 99 |
| U12 | The base's virtual joints spike at ±90° pitch (2.3) | A fall through that pose |

## 10. Not investigated

### 10.1 Whether each activity can be solved through the R1Pro's commands

This is the largest open question. Every goal is satisfiable by the judge (8.5), but almost no manipulation was measured with the gripper R1Pro.

| Question | Activities | What is known |
|---|---|---|
| Can the arms open the heaviest parts through an assisted grasp? | room doors, the sliding door of `sorting_household_items` and `turning_out_all_lights_before_sleep`, fridges (2.4) | The grasp holds 200 N; the arms' drives against these parts were not measured |
| Do objects attach by physical contact? | `hanging_pictures`, `attach_a_camera_to_a_tripod`, `installing_smoke_detectors` | Placing the poster at the nail produced no contact and no attachment; set by state it holds. `make_cabinet_doors` attaches as designed |
| Can small toggle markers be reached? | radio, modem, fax, scanner, table lamp, lighter, stove | Not measured natively |
| Objects on the floor | 22 activities have a goal object within reach only with a deep forward lean | Measured with the hands' fingertips, 0.07 m above the floor in ordinary postures; not with the grippers |
| Heavy or large objects to move | for example `carrying_out_garden_furniture` (barrow 11.3 kg, lawn chair 5.0 kg) | No carry or two-handed grasp was measured |

### 10.2 Geometric screening, not followed up

From a native survey of all 100 activities at seed 0. These are 2-D results; none was confirmed by manipulation.

- **A goal object starts inside a closed container (22):** `boxing_food_after_dinner`, `bringing_water`, `can_meat`, `canning_food`, `clean_your_rusty_garden_tools`, `cook_a_brisket`, `cook_a_frozen_pie`, `cook_bacon`, `cook_brussels_sprouts`, `cook_cabbage`, `cook_hot_dogs`, `freeze_fruit`, `freeze_pies`, `halve_an_egg`, `make_gift_bags_for_baby_showers`, `make_pizza`, `preparing_lunch_box`, `setting_the_table`, `slicing_vegetables`, `thawing_frozen_food`, `unloading_the_car`, `wash_dog_toys`.
- **A door or jointed part on the shortest route to a goal object (11):** `bringing_paper_to_recycling`, `carrying_in_groceries`, `dispose_of_batteries`, `hiding_Easter_eggs`, `loading_the_car`, `moving_boxes_to_storage`, `putting_dirty_dishes_in_sink`, `sorting_household_items`, `tidying_bathroom`, `turning_out_all_lights_before_sleep`, `unloading_the_car`. Every door starts closed.
- **No standing point within ordinary reach (3)**, by the hands' palm envelope: `clean_up_your_desk` (one pen), `cook_bacon` (the bacon, inside the closed fridge), `hiding_Easter_eggs` (four trees; the nearest five satisfy the goal).
- **Something goal-relevant moved during two seconds of idling (5):** `composting_waste`, `dispose_of_batteries`, `sorting_household_items`, `store_batteries`, `tidying_bathroom`. No goal literal flipped.

### 10.3 Also not examined

- Agent episodes on `main`. The 25 agent episodes run earlier, one activity each, used pull requests that were reworked before they merged; nobody has read their logs for why most failed.
- Other instances and seeds: almost everything above is seed 0, instance 301, and other public instances place objects elsewhere.
- Perception: whether the cameras show what each activity needs, objects in unlit closed containers, depth quality.
- Whether any upstream item is already fixed after the pinned BEHAVIOR-1K revision. Only the joint friction lines and the missing instructions were checked against upstream main.
