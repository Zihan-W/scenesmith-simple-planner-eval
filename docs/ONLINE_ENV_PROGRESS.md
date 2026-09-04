# Online Manipulation Environment Progress

Last updated: 2026-09-05
Current phase: Phase 7 — Manual PREGRASP Acceptance Gate

## Product Goal

构建可供 Behavior Tree、TAMP、策略替换、场景替换、任务替换和 benchmark 使用的通用在线机器人仿真环境。

红盒 Pick-and-Lift 仅作为端到端集成测试。

## Existing Commits

* `8dcf4a0` Add online Zerith left-arm control environment
* `27d017f` Fix Zerith gripper command direction
* `29b8cfb` Replace Zerith wrist and gripper collision hulls
* `c2378a1` Add reproducible Zerith pick evaluation scene
* `ebffab1` Scope Zerith pregrasp collision-constrained IK
* `9448af4` Calibrate Zerith rail and pick target coordinates
* `ab551a2` Stabilize Zerith pick target calibration
* `d5d6dc2` Validate fixed-rail Zerith pregrasp
* `7e8a210` Validate online Zerith pregrasp execution
* `d4334fe` Define online manipulation public contracts
* `a2debf7` Introduce Zerith online environment adapter
* `8cab195` Add typed online manipulation environment facade
* `30736b5` Add task and planning query interfaces
* `cee8389` Add selective DMD pose finalization
* `7bc092d` Add deterministic online episode runner
* `ab94332` Add replaceable online policy examples
* `72f9d69` Add state-synchronized Cartesian control
* `55ba5d8` Document BT and TAMP online integrations
* `0738fdc` Complete generic scenario initialization and diagnostics
* `1ad0aba` Document generic scenario runtime completion
* `9ef30dd` Guard staged Cartesian pick motions
* `2ab2722` Enforce generic environment boundaries
* `5f71dfc` Ignore deprecated pregrasp prototype
* `5e155eb` Complete online environment API audit
* `d5b8c8b` Report online action safety decisions
* `64aec13` Expand action decision contract coverage

## Current Validated State

* Drake physics: 1000 Hz
* Low-level control: 200 Hz
* Policy: 10 Hz
* Zerith base fixed
* Rail fixed at 0.4 m
* Left arm and gripper controlled
* Scene and red-box placement manually accepted
* PICK_HOME manually accepted
* PREGRASP manually accepted
* PICK_HOME → PREGRASP direct edge validated
* Online PREGRASP execution passed 3/3 resets
* No robot penetration
* No sustained torque saturation
* Gripper remained open
* Red box was not contacted or moved

## Current PREGRASP Metrics

Read from `output/zerith_pick_eval/*.json` on 2026-09-04:

* Selected candidate: `tilt_15_yaw_-20`
* Valid candidates: 8 of 9
* Gripper–box surface gap: `0.06277452173085399 m`
* IK position error: `0.001552067377860639 m`
* IK orientation error: `3.0164305947682446 deg`
* Elbow bend: `42.88274580741916 deg`
* Minimum normalized joint-limit margin: `0.26620318961696926`
* PICK_HOME: seven-joint URDF zero configuration
* Dense PICK_HOME-to-PREGRASP samples: `376`
* Edge minimum nonpenetration distance: `0.0016412699999999864 m`
* Edge minimum safety clearance: `0.020037650047172152 m`
* Edge minimum wrist/gripper height above table: `0.013640011911204009 m`
* Online episodes: 3 of 3 passed in 81 policy steps each
* Final maximum joint error: `0.000883233817970619 rad`
* Final maximum joint speed: `0.004871037207713848 rad/s`
* Maximum tracking error: `0.032234694937831876 rad`
* Final grasp-frame position error: `0.0002358868051699479 m`
* Final grasp-frame orientation error: `0.02142664873390224 deg`
* Maximum continuous torque saturation: `0.0 s`
* Dynamic minimum safety clearance: `0.020037650047172152 m`
* Red-box translation during PREGRASP episode: `1.9876752782681098e-05 m`
* Final gripper width: `0.07999975975522111 m`

## Phase 0 Audit

All validated code was split into the three checkpoint commits listed above.
At the start of Phase 1, only documentation changes and the explicitly
deprecated, untracked `scripts/simulate_zerith_safe_pregrasp.py` remain.
Requirements prohibit committing that script. It has not been deleted because
it is a 972-line uncommitted artifact and deletion requires an explicit owner
decision.

The requested repository-local `AGENTS.md` is absent. The only file found
under `/root/workspace` is `/root/workspace/scenesmith/AGENTS.md`, whose scope
does not cover this repository.

## Deprecated or WIP

* `scripts/simulate_zerith_safe_pregrasp.py` is obsolete and must not be included in active entry points.
* Earlier rail-zero PICK_HOME/PREGRASP results are obsolete.
* APPROACH, CLOSE, VERIFY_GRASP and LIFT have not been validated.

## Phase 1 Public Contract

The experimental `src.online_manipulation` package now defines versioned,
Drake-independent public contracts for:

* typed joint, Cartesian, gripper, hold and composite actions;
* generic robot, object, contact and task observations;
* scenario, timing, joint, gripper and robot specifications;
* RobotAdapter, Task, Policy, ContactPolicy and OnlineEnvironment protocols.

Version `0.1` is additive. The legacy `ZerithOnlineEnv` remains the validated
runtime and must be wrapped by the real Adapter before compatibility code is
removed. The initial Adapter test uses a structural mock so public contract
tests run without constructing Drake.

Reproduce the Phase 1 tests from the repository root:

```bash
.venv/bin/python -B -m unittest discover -s tests -v
```

Result on 2026-09-05: 8 tests passed.

## Phase 2 Scenario and Zerith Adapter

`ZerithRobotAdapter` now owns the generated model path and package mapping,
base weld, nine controlled joint specifications, locked-joint state, gripper
width mapping, and public robot-observation conversion. The fixed rail is an
explicit locked joint at 0.4 m; it is not presented as a controllable joint.

The existing PREGRASP entry point now constructs the unchanged legacy runtime
through `ScenarioSpec`, `TimingConfig`, and the compatibility Adapter. Target
identity remains an explicit compatibility argument and has not entered the
generic environment contracts.

Reproduce the contract and real-model integration tests:

```bash
.venv/bin/python -B -m unittest discover -s tests -v
```

Result on 2026-09-05: 11 tests passed, including real Drake model loading,
base welding, actuator creation, locked-joint initialization, gripper mapping,
and public observation conversion.

Reproduce the full compatibility regression with the Phase 0 command under
`Phase 0 Reproducible Commands`. Result on 2026-09-05: 3/3 episodes reached
PREGRASP in 81 policy steps. All previously recorded metrics were reproduced
exactly, including zero continuous torque saturation, 20.04 mm minimum safety
clearance, 0.884 mrad final maximum joint error, and an open 80 mm gripper.

## Phase 3 Generic Control, Action and Observation

The new `OnlineManipulationEnv` facade now exposes Gym-style five-value steps,
typed actions, generic object observations, contact penetration observations,
and explicit time-limit truncation. The environment core contains no Zerith,
red-box, table, or current-scene names.

The Zerith compatibility backend translates named joint commands and physical
gripper width to the legacy runtime. `CartesianDeltaAction` raises an explicit
`NotImplementedError` until Phase 4 connects it to planning/IK; no fallback or
offline trajectory is used.

`CoupledInverseDynamicsServo` is independent of joint count and names. The
Adapter's ordered `JointSpec` values supply limits, gains, and actuator
mapping. Direct legacy construction still derives the identical defaults.

Reproduce unit and real-model tests:

```bash
.venv/bin/python -B -m unittest discover -s tests -v
```

Result on 2026-09-05: 18 tests passed. The suite includes typed facade,
truncation, action translation, and a real Drake torque-limiting test.

The full PREGRASP command was also rerun through typed actions and normalized
observations. Result: 3/3 episodes reached PREGRASP in 81 policy steps. All
eight tracked dynamics and safety metrics were bit-for-bit identical to the
Phase 0 baseline. The regression now also rejects any step that does not
perform exactly 20 controller updates with 5 physics steps per update; this
new assertion passed in a separate 1/1 episode run.

## Phase 4 Task, Contact Policy and Planning Query

The environment now delegates reset metadata, task observation, reward,
termination, success metrics, and allowed-contact selection to a replaceable
Task. `NullTask` preserves control regressions. `PickLiftTask` evaluates a
generic observed object's physical vertical displacement and required hold
duration; it contains no approach/grasp/lift motion state machine.

`PairContactPolicy` denies task contacts by default and matches explicit
qualified-body pairs independent of order. Whitelisting relaxes only the
safety-clearance layer, never strict nonpenetration or the Plant's real
collision filter.

`PlanningQuery` builds a separate RobotDiagram and context from ScenarioSpec,
RobotAdapter, and TimingConfig. It exposes FK, joint limits, collision pairs,
configuration validation, dense edge validation, dual clearance, and pose IK.
World-frame `CartesianDeltaAction` can use this query for one online pose-IK
solve per action; a missing query or unsupported reference frame fails loudly.

Reproduce all tests:

```bash
.venv/bin/python -B -m unittest discover -s tests -v
```

Result on 2026-09-05: 25 tests passed. PlanningQuery is tested with a separate
non-Zerith one-joint robot and obstacle, covering FK, IK, collision,
penetration, allowed-contact safety semantics, and a colliding edge.

On the calibrated Zerith scene, the generic query reproduced 1.64127 mm
minimum nonpenetration distance and validated the 376-sample
PICK_HOME-to-PREGRASP edge with 26.28 mm minimum safety clearance. Its context
time remained 0.0 s. A real NullTask PREGRASP episode also passed in 81 policy
steps with the gripper open.

## Phase 5A Selective DMD Finalizer

`ObservedBodySpec.write_back` is an explicit, deny-by-default selection for
scenario finalization. The finalizer updates only the selected direct
`add_model.default_free_body_pose` entry, preserves every other DMD line, and
expresses the final world pose in the entry's original `base_frame`. It refuses
to overwrite the source DMD or proceed without an explicit selection.

The public environment now delegates `write_updated_scenario()` to its runtime
backend. A one-episode real Zerith PREGRASP regression wrote only
`living_room_box_0`; its 19.88 micrometer settling displacement was preserved.
Drake successfully reloaded the generated DMD and recovered world translation
`[1.98312044694348, 2.7700146302787867, 0.5107344413880195]` meters. A textual
diff confirmed that no furniture, room, robot, or other manipuland definition
changed.

Reproduce the 28-test suite and real round-trip:

```bash
.venv/bin/python -B -m unittest discover -s tests -v

SCENE_ROOT=/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000
.venv/bin/python -B scripts/simulate_zerith_pick_home_to_pregrasp.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --pick-home-json output/zerith_pick_eval/pick_home.json \
  --rail-position 0.4 --episodes 1 \
  --write-updated-scenario \
    output/zerith_pick_eval/roundtrip_pregrasp.dmd.yaml
```

## Phase 5B Episode Runner

`run_episode()` and `run_episodes()` now execute external Policy objects over
the public environment contract. They support deterministic seeds, a strict
runner step limit, complete reset between episodes, and non-overwriting output
directories. The runner owns no robot, object, task, or motion-state names.

Each episode summary records success, termination reason, simulated duration,
policy-step count, live robot-related minimum signed distance, maximum joint
tracking error, torque-saturation statistics, contact events, task finalization
metadata, and initial/final poses of every observed object. Artifacts are
`summary.json`, `trace.csv`, optional `simulation.html`, and optional
`final.dmd.yaml`.

The 31-test suite covers the runner metrics, both artifact formats, optional
HTML/final-DMD hooks, multi-seed reset, distinct directory naming, and refusal
to overwrite a nonempty result directory. A real headless Zerith hold episode
also passed through the runner and reported the expected 1.64127 mm minimum
robot-related signed distance with zero tracking error.

## Phase 6 External Policy Examples

The external `HoldPolicy`, `JointStepPolicy`, and `PickLiftPolicy` consume only
public observations and produce only typed public actions. The staged pick
policy owns PREGRASP, APPROACH, CLOSE, LIFT, and HOLD transitions; the
environment and task contain no motion state machine. Its transitions depend
on measured joint state, end-effector motion, bilateral target contact, and
target height rather than elapsed waypoint playback.

The Zerith Adapter now exposes the calibrated `left_grasp_frame` as its public
end effector. The fixed offset remains robot-specific, while Cartesian policy
actions and observations operate at the physical center between the fingers.
The legacy runtime adds this frame without changing URDF geometry, dynamics,
or the validated joint-space PREGRASP trajectory.

Run the first two real examples with the unified CLI:

```bash
SCENE_ROOT=/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000

.venv/bin/python -B scripts/run_zerith_online_example.py hold \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --pick-home-json output/zerith_pick_eval/pick_home.json \
  --output-root output/online_examples/hold

.venv/bin/python -B scripts/run_zerith_online_example.py joint-step \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --pick-home-json output/zerith_pick_eval/pick_home.json \
  --output-root output/online_examples/joint_step
```

Both real five-step smoke runs completed headlessly. Hold reported zero
tracking error; JointStep reported 0.02293 rad maximum transient error. Both
had zero saturation steps and retained 1.64127 mm minimum robot-related signed
distance. The 33-test suite covers all staged policy transitions. Physical
PickLift is intentionally not claimed until Phase 7 passes real bilateral
contact and stable-lift criteria.

### Phase 6B State-Synchronized Cartesian Steps

The planning context now synchronizes the latest poses of observed free bodies
before translating each online action. This keeps collision checks consistent
with objects that moved in the dynamics simulation without mutating or
advancing the planning context. Fixed observed bodies remain usable as query
objects, while an articulated observed body fails explicitly because a single
body pose is insufficient to recover its joint state.

`CartesianDeltaAction` now uses a damped differential-IK step instead of a
loose endpoint pose solve. Arm-joint increments are scaled as one vector to
preserve Cartesian direction, the gripper configuration remains fixed, and
the full command edge is checked against joint limits, strict
nonpenetration, and policy-filtered safety clearance. Episode CSV traces now
include the end-effector pose and every observed object pose at each policy
boundary.

The 37-test suite covers free-body synchronization, fixed observed bodies,
bounded differential IK, and rejection of a colliding Cartesian edge. A real
Zerith smoke test commanded a 1 mm world-Z increment from PICK_HOME with the
gripper open. The command produced exactly 20 servo updates with 5 physics
steps each, retained the 1.64127 mm minimum robot-related signed distance, and
did not terminate or truncate. This is a control/query integration check, not
a claim that APPROACH or grasp execution is validated.

### Phase 6C Behavior Tree and TAMP Handoffs

Two importable examples now document the external integration boundary. The
Behavior Tree leaf performs exactly one typed action and one `env.step()` per
tick. The TAMP helper synchronizes observed object poses, rejects an invalid
start or direct edge before execution, and then sends absolute joint targets
online until the measured position and velocity settle. It deliberately does
not implement a Behavior Tree framework, path search, or time
parameterization.

README now shows a user-defined Policy, deterministic batch episodes,
JSON/CSV/HTML/final-DMD artifacts, BT ticking, TAMP query-to-execution handoff,
and task/scene replacement. Three contract tests verify tick semantics,
query-before-step ordering, and rejection without environment mutation.

Both handoffs were also run against the real Zerith dynamics model with an
open gripper and a 0.02 rad left-shoulder target. The BT leaf and TAMP helper
each settled successfully in 5 policy steps at measured position
`0.0180566 rad` and speed `0.0133001 rad/s`. The TAMP edge retained
`0.00164127 m` strict nonpenetration distance and `0.02628 m` safety
clearance. The full suite contains 40 passing tests.

### Phase 6D Pick Policy Safety Guards

The external staged policy now waits for the measured joint command to settle
before issuing another Cartesian increment. APPROACH recomputes longitudinal
distance from the live target pose but remains on the calibrated approach
axis. Excessive lateral error, overshoot, or an unexpectedly distant target
enters a failed hold state instead of closing the gripper. Contract tests cover
settling, calibrated-axis motion, and rejection without a close command.

This checkpoint only validates policy logic. It has not been executed past
PREGRASP in the physical Drake scene and makes no PickLift success claim.

## Phase 8 API Audit

Scenario initialization is now effective rather than declarative. Initial
free-body poses are keyed by public observation name and applied consistently
to both the real simulation context and independent planning context. Planning
pose synchronization accepts partial updates and rejects unknown names. A
`NullTask` scene no longer needs a legacy target model.

The Zerith runtime applies the two documented positive Drake contact
parameters, `penetration_allowance_m` and `stiction_tolerance_m_s`, and rejects
unknown names. EpisodeRunner now preserves policy/environment exceptions as
failures: it writes `failure.json`, the completed portion of `trace.csv`, and
the current optional Meshcat HTML before re-raising the original exception.

The full suite contains 48 passing tests, including a real Drake target-free
scene with an overridden free-body pose and contact parameters. The calibrated
open-gripper PREGRASP regression was rerun for one episode after these changes:
it reached PREGRASP in 81 policy steps with 0.884 mrad final joint error,
20.04 mm minimum safety clearance, zero continuous torque saturation, an
80 mm open gripper, and 0.020 mm target settling motion. No APPROACH or CLOSE
command was issued.

Two architecture tests now enforce that the generic core neither contains the
current robot/scene identifiers nor imports the Zerith adapter. The deprecated
untracked PREGRASP prototype is preserved locally through one exact ignore
rule. After the milestone commits, `git status --short` is empty.

Completed policy steps now return `info.action_decision` with one of
`accepted`, `adjusted`, or `rejected`, explicit reason codes, and requested and
applied values. Joint limits and per-period deltas are reported as adjustments.
A collision-invalid Cartesian edge or invalid gripper width is atomically
rejected as a hold; API/configuration errors still raise. The same decision is
stored in each EpisodeRunner CSV row.

The 52-test suite passed after this addition. A real 0.03 rad JointStep request
with a 0.01 rad policy limit recorded `adjusted/maximum_joint_delta` and applied
exactly 0.01 rad; its next Hold step recorded `accepted`. The open-gripper
PREGRASP regression then reproduced the 81-step baseline exactly.

Real EpisodeRunner artifact coverage was also rerun with a two-step headless
Hold episode and produced `summary.json`, `trace.csv`, `simulation.html`, and
`final.dmd.yaml`. The final DMD reloaded through Drake and completed a 3 s
free-body settling run; the target remained free with 0.0127 mm maximum XY
drift and negligible final Z displacement.

Reproduce the artifact and reload checks:

```bash
SCENE_ROOT=/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000

.venv/bin/python -B scripts/run_zerith_online_example.py hold \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --pick-home-json output/zerith_pick_eval/pick_home.json \
  --output-root output/online_examples/phase8_artifacts \
  --episodes 1 --seed 41 --max-steps 2 \
  --record-html --write-final-dmd --realtime-rate 0

.venv/bin/python -B scripts/validate_zerith_target_settle.py \
  output/online_examples/phase8_artifacts/episode_000_seed_41/final.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --duration 3
```

## Phase 0 Reproducible Commands

Run from `/root/workspace/scenesmith-simple-planner-eval` after activating the
repository environment:

```bash
SCENE_ROOT=/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000

.venv/bin/python -B scripts/convert_zerith_for_drake.py --check
.venv/bin/python -B scripts/validate_zerith_collision_proxies.py

.venv/bin/python -B scripts/prepare_zerith_pick_eval_scene.py \
  "$SCENE_ROOT/combined_house/house_furniture_welded.dmd.yaml" \
  --output-dir output/zerith_pick_eval

.venv/bin/python -B scripts/validate_zerith_target_settle.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml"

.venv/bin/python -B scripts/validate_zerith_pregrasp_collision_scope.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml"

.venv/bin/python -B scripts/search_zerith_safe_home.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --rail-position 0.4 --num-random-seeds 0

.venv/bin/python -B scripts/validate_zerith_pregrasp_ik.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --rail-position 0.4 \
  --approach-tilt-deg 15 30 60 \
  --yaw-offset-deg 0 -20 20 \
  --num-random-seeds 0 --exhaustive

.venv/bin/python -B scripts/search_zerith_pick_home.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --pregrasp-json output/zerith_pick_eval/pregrasp_ik.json \
  --rail-position 0.4

.venv/bin/python -B scripts/validate_zerith_safe_home_dynamics.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --home-json output/zerith_pick_eval/pick_home.json \
  --home-key q_pick_home --rail-position 0.4

.venv/bin/python -B scripts/simulate_zerith_pick_home_to_pregrasp.py \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --pick-home-json output/zerith_pick_eval/pick_home.json \
  --rail-position 0.4 --episodes 3
```

Legacy entry-point compatibility was checked with shortened hold/step timing:

```bash
.venv/bin/python -B scripts/simulate_zerith_left_arm.py \
  "$SCENE_ROOT/combined_house/house_furniture_welded.dmd.yaml" \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --hold-duration 0.1 --step-delta 0.02 \
  --step-settle-duration 0.1 --realtime-rate 0
```

It loaded with zero initial robot penetration, zero hold error and zero hold
torque saturation. All seven positive/return joint steps completed; the
largest reported step error was `0.013955 rad`.

## Milestones

* [x] Phase 0: audit, regression and checkpoint current work
* [x] Phase 1: define public dataclasses, protocols and contract tests
* [x] Phase 2: introduce ScenarioSpec and ZerithRobotAdapter
* [x] Phase 3: generalize controller, action and observation
* [x] Phase 4: implement Task, ContactPolicy and PlanningQuery
* [x] Phase 5: implement EpisodeRunner and DMD finalizer
* [x] Phase 6: add external policy, BT and TAMP examples
* [ ] Phase 7: run PickLift integration test
* [x] Phase 8: documentation, API audit and clean worktree

## Definition of Done Audit

1. **Pass:** architecture tests reject current robot/scene identifiers in the
   generic core.
2. **Pass:** Zerith model, joint, gripper, and legacy translation logic lives
   in `ZerithRobotAdapter` and its adapter module.
3. **Pass:** Policy and Task have independent public protocols, implementations,
   and replacement tests.
4. **Pass:** deterministic `reset()` / one-policy-period `step()` semantics are
   documented and tested.
5. **Pass:** the 1000/200/10 Hz integer schedule is unit-tested and checked in
   real PREGRASP execution.
6. **Pass:** EpisodeRunner covers seeded multi-episode headless execution and
   non-overlapping outputs.
7. **Pass:** PlanningQuery exposes FK, IK, collision, clearance, configuration,
   and dense-edge checks to TAMP without advancing simulation.
8. **Pass:** selective final-DMD write/reload has unit and real Drake evidence.
9. **Pass:** Hold, JointStep, and PickLift policies remain outside environment
   core; BT and TAMP handoffs use the same typed action boundary.
10. **Pending:** physical PickLift has not passed bilateral contact, 8 cm lift,
    3 s hold, and three consecutive episodes.
11. **Pass:** tests, README, JSON, CSV, real Meshcat HTML, and final-DMD example
    commands and artifacts exist.
12. **Pass:** milestone commits are separated and `git status --short` is empty
    after documentation commit.

## Current Blockers

* No trusted rail velocity or force parameters.
* Robot portability has structural mock coverage but no second real adapter.
* Physical PickLift has not yet been completed.
* PREGRASP needs the requested manual front, side, and top-view acceptance
  before any physical APPROACH or CLOSE command may run.

## Next Action

Present the open-gripper PREGRASP execution in Meshcat for manual front, side,
and top-view acceptance. Do not issue APPROACH or CLOSE commands until that
review is complete. After approval, resume Phase 7 with the final-centimeters
online approach and bilateral-contact grasp test.
