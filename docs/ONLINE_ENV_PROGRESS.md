# Online Manipulation Environment Progress

Last updated: 2026-09-05
Current phase: Phase 3 — Generic Control, Action and Observation

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
* [ ] Phase 3: generalize controller, action and observation
* [ ] Phase 4: implement Task, ContactPolicy and PlanningQuery
* [ ] Phase 5: implement EpisodeRunner and DMD finalizer
* [ ] Phase 6: add external policy, BT and TAMP examples
* [ ] Phase 7: run PickLift integration test
* [ ] Phase 8: documentation, API audit and clean worktree

## Current Blockers

* No trusted rail velocity or force parameters.
* Robot portability has not yet been verified with a second adapter or mock.
* Physical PickLift has not yet been completed.
* Removing the deprecated untracked script requires an explicit owner choice.

## Next Action

Begin Phase 3 by implementing the generic `OnlineManipulationEnv` facade,
typed-action translation, and normalized observation conversion around the
legacy runtime. Preserve exact 1000/200/10 Hz scheduling and keep the 3/3
PREGRASP regression green.
