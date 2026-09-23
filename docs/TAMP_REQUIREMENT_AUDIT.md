# TAMP requirement audit — first-phase completion

This maps the original numbered request, not a reduced definition of success.
The allowed first-phase implementation is complete: successful real online
episode `live_010`, full regression 005, latest TAMP regression 012, and the
artifact integrity audit. Remaining limitations below are explicitly allowed
first-phase simplifications or future paper-level scope, not hidden completion
claims. One scene/seed does not establish general physical robustness.

| Request | Current implementation / evidence | Limitations / clarification |
| --- | --- | --- |
| 0 audit before refactor | `TAMP_ARCHITECTURE_AUDIT.md`: data, predicates, operators, runtime, geometry, feedback and KEEP/REFACTOR/REPLACE/ADD decisions | Historical starting-state audit, not current-state proof |
| 1 four responsibilities | `tamp_semantic`, `tamp_hierarchy`, `tamp_geometry`, shared BT runtime | Successful `live_010` traverses all layers |
| 2 n-ary goals.v3 | `PredicateGoal.arguments`, strict semantic parser and numeric-pose rejection tests | Expanded physical predicate domains not implemented |
| 3 incremental subgoals | `IncrementalTampRunner`; test A and successful horizon budget test | `live_010` verifies navigation, then re-solves and verifies holding |
| 4 open skeleton layer | `SkillProgram`, `refine_goals`, basic prerequisite test | No model-generated skeleton needed: deterministic search |
| 5 unified registry | Same registry instance in CLI planner/solver/executor; registry trace and alias-binding test | More physical skills are future work |
| 6 no model numeric domains | Hierarchical semantic payload and program contract tests | Numeric domains remain only in requested legacy mode |
| 7 generic prerequisites | Registry STRIPS search; added-skill and Clear→same-Pick tests | Clear is a synthetic test, not a physical skill |
| 8 geometric abstraction | `GeometricParameterSolver`, `SamplingSolver`, `ParameterizedSkillPlan` | Successful single-fixture grasp; broad robustness not claimed |
| 9 replaceable backend | Sampling works through existing checks; `CuTAMPSolver(backend)` seam | No external optimizer installed |
| 10 automatic domains | Target-relative base candidates, gripper-scaled approaches, offsets, solved-grasp lift and carried-height check | `live_010` succeeds without manual overrides; domains remain calibrated |
| 11 batch/check/rank | `evaluate_batch`, domain ranking, candidate trace test | Sequential CPU batch, not vectorized GPU |
| 12 preserve semantic task | N-ary argument/shared-variable tests; cuTAMP result and mutation guard | External backend must run the documented physical rechecks |
| 13 four-level recovery | `RecoveryLimits`, A–E and deterministic rejection/stable exclusion tests | Real obstacle-clearing semantic recovery not demonstrated |
| 14 abstract failure feedback | `ConstraintResult`, collision body extraction, reason counts; runtime rejection details plus symbolic blocker IDs | Test proves numeric details stay in execution trace, not semantic feedback |
| 15 annotated observation | Fresh RGB/labels, exact object IDs, metadata, timestamps; real capture and metadata tests | Uses registered simulator observations, no detector |
| 16 prompt registry/files | `tamp_subgoal_v3.txt`, `tamp_legacy_*.txt`, name/version/schema | No skill-model prompt in hierarchical path because no such call |
| 17 validated retries | Semantic + legacy schema/transport retries; strict provider schema and JSON-mode config, four format tests, successful live strict-schema request | Local validation retained even when a compatible service ignores the schema |
| 18 model config | `tamp_hierarchical_config.json`; both live routes consume configured settings | Hierarchical `skill_model` intentionally unused |
| 19 shared runtime | `SceneSmithSkillExecutor` binds one existing `JsonBtPolicy` leaf per action | Not a separate physical executor |
| 20 consume geometry | Navigation target, pick joint plan; binding, waypoint-order, command-limit and contact-gate tests | Actual dynamics verified separately in `live_010` |
| 21 observe/verify each skill | Fresh observation, factual effects, terminal-episode test | `live_010`: both effects true; 8.013 cm lift and 3.1 s stable hold |
| 22 no whole-tree TAMP run | Only first selected skill executes, rest is re-solved after observation | Legacy keeps full-tree behavior for ablation |
| 23 separation | Typed modules listed above with shared runtime adapters | Existing file layout retained |
| 24 legacy ablation | `--tamp-mode legacy-vlm-domain`; old recorded inputs/tests retained | Physical comparison dataset not yet collected |
| 25 CLI | BT/hierarchical/legacy plus sampling/cutamp selection; help checked | cutamp explicitly errors until installed |
| 26 trace | JSONL model, world, skeleton, candidates, selection, execution, verification, recovery, final result | Successful and failed-run artifact audits passed; some fields are embedded events |
| 27 metrics | Task/subgoal/skill success, latencies, retries/replans, constraint counts, model calls | Baseline statistical comparison not claimed |
| 28 A–E tests | `test_tamp_online.py::test_a` through `test_e`, plus additional regressions | These are deterministic integration tests, not five physical demos |
| 29 paper mapping | README and `TAMP_HIERARCHICAL.md` explicitly distinguish inspired/simplified components | No full paper reproduction claim |
| 30 phased implementation | Prior continuation history, typed adapter and documented integration steps | Latest geometry/completion changes covered by tests and successful live run |
| 31 DoD | Semantic-only model, independent open skeleton, automatic solver, no manual base, per-skill observe/verify, four recovery levels, retained BT/shared runtime, A–E, live demo, honest README | All 12 first-phase conditions have evidence; no full optimizer/general skill claim |
| 32 final A–G report | `docs/TAMP_IMPLEMENTATION_REPORT.md` | Complete architecture/files/mapping/simplifications/interfaces/tests/future report |

Validation evidence: full suite `regression_005.log` passed **248 tests with one
skip**. The subsequent observed-completion fix is covered by
`tamp_regression_012.log`: **61 tests, OK**, plus the existing BT core tests.
Physical outcomes reside in
`runs/tamp-hierarchical-20260919/live_*/result.json`; navigation or a model call
alone does not establish `holding`. `live_010` reports `task_goal_verified`;
its 239-step trace, two true verifications, final contact/height/hold state,
metrics agreement and six annotated image hashes/timestamps were checked.
`live_009` was intentionally interrupted after the corrected run succeeded;
its retained partial trace is not a successful or fully completed episode.
