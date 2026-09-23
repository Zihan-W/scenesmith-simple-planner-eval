# TAMP refactor report — first-phase implementation verified

This is the requested A–G report for the completed first-phase refactor.
`live_010` is a successful real-VLM online simulation, not recorded replay.
The original 0–32 requirements are reconciled in `TAMP_REQUIREMENT_AUDIT.md`.
This does not claim full paper reproduction or multi-scene grasp robustness.

## A. Architecture

```text
User task + measured world + synchronized annotated cameras
  → SemanticSubgoalPlanner: strict goals.v3, semantic predicates only
  → registry-backed symbolic search: skill_program.v2 with open variables
  → SamplingSolver / CuTAMPSolver interface: geometric assignments
  → ParameterizedSkillAction
  → existing JsonBtPolicy skill leaf + existing robot runtime
  → fresh observation + expected-effect verification
  → next skill/subgoal, or bounded four-level recovery
```

The runner may geometrically refine a skeleton, but executes only its first
skill. The remainder is re-solved from the new measured state, never executed
as a stale full-tree plan. Failure order is skill retry → reparameterization →
alternative skeleton → semantic revision. Deterministic path rejections skip
an identical skill retry. Verified intermediate horizons do not consume the
semantic failure budget.

The BT baseline still uses its original generation and whole-tree execution
path. The legacy TAMP route retains model-authored numeric sampling programs
and whole-plan rollouts for ablation. All physical commands use the same robot
runtime and safety checks.

## B. Files and responsibilities

These are the files relevant to the refactor, not a claim of ownership over
every pre-existing dirty file in the checkout. No user changes were discarded
and no commit or push was made.

| File | Responsibility |
| --- | --- |
| `planner/src/tamp/hierarchy.py` | N-ary world facts/goals, registry, open-variable programs, generic search, strict contracts |
| `planner/src/tamp/semantic.py` | Semantic-only VLM payload, model configuration, retries, request/response provenance |
| `planner/src/tamp/model.py` | Shared provider-format builders and bounded legacy model validation |
| `planner/src/tamp/geometry.py` | Solver protocol, batching/ranking, assignment exclusions, abstract constraints, cuTAMP adapter |
| `planner/src/tamp/scenesmith.py` | Automatic base/grasp/approach domains, measured-state queries, IK and full joint-edge checks |
| `planner/src/tamp/online.py` | Incremental execution, observation/verification, bounded recovery, metrics and JSONL decisions |
| `planner/src/tamp/scenesmith_online.py` | Adapter into existing BT leaves, semantic observations, fresh image annotations, failure details |
| `planner/src/skills/picklift.py` | Shared parameterized PickLift skill; approach/lift waypoints, contact gates, uniformly bounded joint commands |
| `planner/src/tamp/cli.py` | BT/hierarchical/legacy routing, backend selection, explicit common seed and result artifacts |
| `planner/src/tamp/planner.py` | Retained legacy proposals/refinement; versioned prompts and validated retries |
| `planner/src/tamp/program.py` | Retained legacy numeric-domain DSL and bounded model retries; no generated Python execution |
| `planner/src/tamp/pipeline.py` | Retained ablation entrypoint; model config, whole-plan artifacts and shared waypoint binding |
| `planner/src/bt/runtime.py` | Existing runtime retained; current navigation query and model-bound waypoint step limits |
| `planner/src/bt/generation.py` | Existing generation retained; optional provider response format, absent from default BT requests |
| `planner/resources/prompts/tamp_subgoal_v3.txt` | Versioned semantic-only rules and examples |
| `planner/resources/prompts/tamp_legacy_goals_v2.txt` | Versioned legacy state-goal prompt |
| `planner/resources/prompts/tamp_legacy_subgoals_v1.txt` | Versioned older skill-proposal prompt |
| `planner/resources/prompts/tamp_legacy_program_v1.txt` | Versioned legacy numeric-domain prompt |
| `simulation/src/geometry/planning.py` | Measured-joint query update and correctly composed support policies |
| `simulation/src/geometry/contact.py` | Nested support-policy permission and penetration-limit delegation |
| `simulation/src/tasks/tasks.py` | Separate optional support bound while preserving original defaults/policy name |
| `simulation/src/io/experiment.py` | Load explicit task contact/support settings without changing defaults |
| `simulation/src/sensors/sensors.py` | Explicit fresh camera evaluation without advancing time/changing periodic latch |
| `simulation/src/runtime/runtime.py` | Public fresh capture and carrying/squeeze checks through the existing runtime |
| `experiments/tamp_hierarchical_config.json` | Model/temperature/format/retry settings, geometry and recovery budgets |
| `experiments/navigation_picklift_tamp_12cm.json` | Existing explicit simulation fixture used for online validation; same scene/control settings available to the BT baseline |
| `experiments/inputs/tamp/recorded_navigation_picklift_goals_v3.json` | Deterministic v3 fixture, clearly distinct from live calls |
| `pyproject.toml` | Unified CLI entrypoint and packaged prompt assets; old entrypoints retained |
| `README.md` | Baseline/ablation routing and honest paper mapping |
| `docs/TAMP_ARCHITECTURE_AUDIT.md` | Initial architecture review |
| `docs/TAMP_HIERARCHICAL.md` | Running, contracts, limitations and external cuTAMP integration steps |
| `docs/TAMP_PIPELINE.md` | Legacy/historical route, clearly distinguished from the online path |
| `docs/TAMP_REQUIREMENT_AUDIT.md` | Original numbered requirements and evidence/gaps |
| Historical simulation audit (removed during artifact cleanup) | Read-only asset checks and verified clipping-path failure diagnosis |
| Historical continuation log (removed during artifact cleanup) | Test history, process handles and intermediate outcomes |

Test files: `test_tamp_hierarchy.py` (schemas/search/geometry/ranking),
`test_tamp_online.py` (A–E and recovery contracts), `test_tamp_program_contracts.py`
(n-ary/shared variables/backend immutability), `test_tamp_measured_geometry.py`
(actual measured-state queries), `test_tamp_scenesmith_contacts.py` (support
composition), `test_tamp_execution.py` (waypoint/contact/step limits),
`test_tamp_runtime_binding.py` (actual adapter parameters),
`test_tamp_runtime_completion.py` (observed TAMP completion and unchanged BT behavior),
`test_tamp_observation_metadata.py` (IDs/metadata/frames/timestamps),
`test_tamp_model.py` (legacy retries), `test_tamp_structured_output.py`
(provider formats plus local validation), `test_tamp_cli.py` (seed controls).
The original `test_tamp_planner.py` and `test_tamp_program.py` remain as legacy
regressions; `test_mobile_cameras.py` verifies fresh rendering with real Drake.

## C. Paper mapping

- **VLM-TAMP responsibility:** `tamp_semantic.py` chooses intermediate semantic
  world states from images, identities, facts and abstract physical feedback.
  It does not output geometric parameters or feasibility claims.
- **PRoC3S-inspired:** `tamp_hierarchy.py` creates open-parameter skill programs.
  This implementation uses bounded generic symbolic search, not the paper's
  complete learned program-generation procedure.
- **cuTAMP-inspired:** `tamp_geometry.py` separates variables/constraints and
  candidate search from semantics/runtime. Current feasibility evaluation uses
  SceneSmith/Drake sampling, not an installed GPU differentiable optimizer.
- **Repository-specific:** online verifier/recovery, observation annotations,
  and adapters to the existing navigation and joint-waypoint skills.

## D. Simplifications and guarantees

The physical domain currently supports only NavigateToPick and PickLift. Its
calibrated target-relative grasp family and constant-heading base candidates
are limited domains, not arbitrary-object grasp synthesis or global navigation.
Batch evaluation is sequential CPU work; the API permits replacement. Simulator
labels provide observed object identities; there is no new detector.

IK/joint-edge checks cannot certify friction, deformation or successful contact.
Current geometric lift checks carry the target using planned open-finger
geometry; the real runtime checks measured fingers, carried geometry and actual
contact. This limitation is explicit, not a claim of complete dynamic
feasibility. No DP/VLA policy was added or claimed to consume a pose condition.
External cuTAMP is not installed; its typed adapter and integration instructions
are the allowed first-phase seam, not optimization results.

## E. Current skill interface

NavigateToPick's selected base pose is transformed to the existing navigation
frame and passed to the same `NavigateTo` leaf. Geometry checking uses the
measured robot/base/object state. Both base-link and navigation-frame poses are
logged distinctly.

PickLift's staging, optional segmented approach, grasp and lift joint positions
are passed to the existing `ExecutePickLift` binding and
`JointWaypointPickLiftSkill`. The skill follows the selected waypoints and waits
for actual bilateral contact before lifting. It uniformly bounds increments
using runtime/URDF limits so per-axis clipping cannot distort the certified
joint-space segment. Ordinary BT experts and underlying safety limits remain
unchanged. Registered conditioning capability is checked and logged; unsupported
policies must not claim geometric guarantees.

## F. Tests and physical results

| Required case | Evidence |
| --- | --- |
| A Basic | `test_a_basic_pick_is_incremental`: automatic navigation prerequisite and per-skill verification |
| B Geometry failure | `test_b_geometric_failure_tries_alternative_automatically`: no manual candidate change |
| C Skill failure | `test_c_execution_failure_retries_then_verifies`, plus failed-sample exclusion and measured-state refresh |
| D Skeleton failure | `test_d_unsat_skeleton_replans_to_other_skill`, plus Clear→same-Pick recovery |
| E Semantic failure | `test_e_unreachable_semantic_goal_prompts_revised_goal`, plus abstract blocker/no joint-number leakage |

All five required cases passed. These are deterministic integration tests,
not five physical robot demos. Full regression 005 passed **248 tests,
663.989 s, OK (skipped=1)**. The subsequent observed-completion fix passed
TAMP regression 012: **61 tests, 37.960 s, OK**, plus two existing BT core tests.
`git diff --check` passed. The collision-proxy validator separately passed
162 configurations; its bounded coverage is documented in the simulation audit.

### Successful real online demo

Authoritative result: `runs/tamp-hierarchical-20260919/live_010/result.json`.
The actual provider returned `gpt-4.1-mini-2025-04-14`, with image tokens in the
recorded usage. Seed 500; no recorded proposals or manual base/grasp overrides.

| Observation / metric | Result |
| --- | --- |
| Final result | `success: true`, `task_goal_verified` |
| Actual target lift | 0.0801343889 m, exceeding the unchanged 0.08 m requirement |
| Continuous stable hold | 3.1 s, exceeding the unchanged 3.0 s requirement |
| Contacts | Bilateral grip, no support contact, no unexpected target contacts |
| Verified subgoals / skills | 2 / 2: navigation, then holding |
| VLM / skill-model calls | 1 / 0 (symbolic search creates the skeleton) |
| Recovery / retries | All zero |
| Candidates / constraint failures | 16 / 2 |
| Semantic / symbolic / geometry / execution time | 6.908 / 0.000186 / 171.052 / 216.805 s |
| Physical trace | 239 steps, 6 synchronized annotated images |

Artifact audit checked JSONL event coverage, both verifier results, final
result/metric agreement, all image hashes and timestamps, symbolic annotation
IDs, and the final task's actual contact/height/hold conditions. The pick runtime
exits as `episode_finished` when the environment's task succeeds; the separate
post-execution verifier establishes the holding effect and counts skill success.

Earlier runs exposed and preserved genuine failures: per-axis command clipping,
loaded-contact tracking gates, nominal versus solved lift height, and premature
predicted completion. The final implementation uniformly bounds joint steps,
uses the existing expert's contact tracking/velocity config, plans from solved
grasp geometry using the shared nominal 10 cm lift with an IK error budget,
checks predicted carried-object height, and requires observed completion for
TAMP. Ordinary BT completion behavior is unchanged. No model mass, friction,
servo gains or safety limits were relaxed. The obsolete `live_009` diagnostic
was intentionally interrupted after `live_010` succeeded; its logs are retained
and it is not represented as a completed successful episode.

## G. Remaining work

For paper-level breadth (not falsely claimed here): integrate/evaluate the real
cuTAMP optimizer, add larger skill/predicate domains and physical obstacle
removal/placement, connect and test genuinely conditioned DP/VLA policies,
replace calibrated grasp families with general scene/robot geometry domains,
and collect controlled multi-scene/multi-seed BT/legacy/hierarchical comparisons.
