# Paper alignment: pre-implementation audit

Date: 2026-09-19. This report records the repository **before** the paper-aligned implementation. A successful sampling rollout is not evidence of PRoC3S or cuTAMP integration. Status describes the mechanism in this repository, not the quality of the paper or upstream repository.

Allowed statuses: FAITHFUL, SIMPLIFIED, INSPIRED, MISSING. FAITHFUL is scoped to the individual row only.

## Evidence and provenance

Official source was downloaded and read, including function bodies, not just READMEs:

- [PRoC3S source, revision 5641773](https://github.com/Learning-and-Intelligent-Systems/proc3s/tree/56417730421e53eb2346929e44ef9c1464d526c6), [paper](https://arxiv.org/abs/2406.05572), especially LMP and constraint satisfaction sections. `config/` is actually `vtamp/config/` in this revision.
- [cuTAMP source, revision 7932e6c](https://github.com/NVlabs/cuTAMP/tree/7932e6cf0ee216331e37e06b60f18bb8b3ec1fbd), [paper](https://arxiv.org/abs/2411.11833), Algorithm 1 and Sections V–VII. The arXiv PDF was read; a roboticsproceedings download returned HTML and was not used as PDF evidence.
- [VLM-TAMP paper](https://arxiv.org/html/2410.02193v1), Algorithm 1 and Sections III-A–E. This table uses paper evidence, not an unverified third-party implementation.

In tables, `proc3s/` and `cuTAMP/` mean these official revisions. Local implementation paths are relative to `planner/src/` unless prefixed `simulation/`. Line numbers are pre-change references; named functions remain the stable identifiers.

## VLM-TAMP

| Paper mechanism | Official implementation / paper evidence | Current implementation | Status | Gap | Action required |
|---|---|---|---|---|---|
| Image and symbolic observation | III-A, Figure 3 | `tamp_scenesmith_online.py:183` SceneSmithWorldObserver; `tamp_semantic.py:46` propose | FAITHFUL | Row concerns observed scene context, not perception generality | Preserve annotated cameras and refresh state |
| Semantic subgoals rather than numeric actions | III-B/C, Algorithm 1 | `tamp_semantic.py:46`; `tamp_hierarchy.py:163` parse_semantic_goals | FAITHFUL | Restricted predicate vocabulary | Preserve strict semantic boundary |
| English proposal then formal translation | III-C, Figure 4 | `tamp_semantic.py:46` single JSON request | SIMPLIFIED | One-stage constrained output | Declare modification in final fidelity report |
| Semantic feasibility validation | III-C | `tamp_hierarchy.py:163` shape/object checks; `:200` refine_goals | SIMPLIFIED | No paper PDDL semantic checker | Keep explicit registry/precondition validation |
| Reduced planning objects, collision-driven expansion | III-D | BFS grounds over all world objects | MISSING | No object-universe reduction | Do not claim this optimization; retain all physical obstacles |
| TAMP refinement of each goal | III-D | `tamp_online.py:76`; `tamp_geometry.py:84` SamplingSolver | INSPIRED | Custom STRIPS/sampling, not paper planner | Preserve as baseline; add interchangeable requested planners |
| Execute, observe, then replan with failure context | Algorithm 1, III-E | `tamp_online.py:76` run; observer geometry_state/capture_images | SIMPLIFIED | Per-skill execution; shared abstraction for geometry/semantics | Retain fresh observations and separate three feedback layers |

## PRoC3S

| Paper mechanism | Official implementation / paper evidence | Current implementation | Status | Gap | Action required |
|---|---|---|---|---|---|
| LLM prompt contains state, skills, program/domain instructions | `proc3s/vtamp/policies/ours/policy.py:115`, `:146`; `vtamp/policies/prompt_elements/ours_role.txt:1` | `tamp_online.py:76` calls BFS, no skill LLM | MISSING | Configured skill_model is unused here | Independent PRoC3SProgramGenerator using real model client |
| Model outputs executable program and domains | `proc3s/vtamp/policies/ours/policy.py:175`, `:187`; `vtamp/policies/ours/prompt_RavenEnv.txt:18` | `tamp_hierarchy.py:87` SkillProgram is authored by BFS | INSPIRED | No model-produced program | Generate validated open-parameter JSON DSL; disclose restriction versus Python exec |
| Open continuous function arguments | `proc3s/vtamp/policies/prompt_elements/ours_role.txt:1`; `vtamp/policies/ours/prompt_RavenEnv.txt:18` | `tamp_hierarchy.py:80` SkillStep.continuous_variables | INSPIRED | Open names exist but not from LLM | Enforce unassigned variables at generation boundary; reject solved numeric values |
| Domain generation | `proc3s/vtamp/policies/ours/policy.py:59`; `vtamp/policies/ours/prompt_RavenEnv.txt:26`; `vtamp/policies/utils.py:118` | SceneSmith domain.samples supplies candidates | SIMPLIFIED | Upstream LLM writes gen_domain, including numeric bounds | Expose typed domain declarations referencing measured scene domains; disclose difference, no numeric solution in LLM output |
| Environment-owned constraints | `proc3s/vtamp/environments/raven/env.py:620`, `:629`, `:759` | `tamp_scenesmith.py` SceneSmith checks; `tamp_geometry.py:93` | INSPIRED | Different robot and constraint implementation | Reuse real reachability, IK, limits, collision, corridor, grasp checks |
| CCSP formation from program and environment | `proc3s/vtamp/policies/ours/policy.py:44` rejection_sample_csp | `tamp_geometry.py:112` recursive per-step search | INSPIRED | Not full-assignment rejection loop | Independent Proc3sCCSPSolver: sample complete assignment, instantiate full program, sequentially validate |
| Sampling, reset and full program rollout | `proc3s/vtamp/policies/ours/policy.py:59`–`:78` | SamplingSolver uses domain.predict after feasible step | SIMPLIFIED | No official twin reset/step; search ranks batches | Keep baseline; explicitly document SceneSmith validator versus physical twin rollout |
| Constraint feedback statistics | `proc3s/vtamp/policies/ours/policy.py:67`, `:215` | `tamp_geometry.py:36` abstract_constraint_feedback | INSPIRED | Lacks program-level typed unsat and real recipient | PRoC3SProgramUnsat with failed categories, skill/index and involved objects |
| Unsat causes actual LLM reprompt | `proc3s/vtamp/policies/ours/policy.py:153`, `:175`, `:219`; `prompt_RavenEnv.txt:35`–`:61` | Excludes BFS paths; only semantic VLM is called | MISSING | No program LLM loop | Feed failed program and structured failures to generator; prohibit BFS fallback |
| Simulator validation and skill execution | `proc3s/eval_policy.py:79`, `:95`; `vtamp/policies/ours/policy.py:126`; `vtamp/environments/raven/env.py:620` | `tamp_scenesmith_online.py:40` existing shared skill executor | SIMPLIFIED | Incremental measured-state replanning differs from upstream cached plan | Keep user-required shared runtime and verify every skill; disclose modification |

Upstream rejection sampling stops at a configured budget; failure is not a mathematical proof of continuous infeasibility. Our requested `PRoC3SProgramUnsat` must retain this distinction in diagnostics. Upstream also assumes the generated family reaches the goal; its rejection sampler checks environment violations and does not independently prove the final semantic goal. Our symbolic/final-observation checks are additional checks, not upstream claims.

## cuTAMP

| Paper mechanism | Official implementation / paper evidence | Current implementation | Status | Gap | Action required |
|---|---|---|---|---|---|
| Ground operator skeleton | `cuTAMP/cutamp/task_planning/tamp_structs.py:17`, `:35`, `:40` | SkillProgram; external adapter only | INSPIRED | No official GroundTAMPOperator conversion | Independent ContinuousProblemBuilder |
| Continuous variable typing | `cuTAMP/cutamp/optimize_plan.py:71`; `particle_initialization.py:61` | String variable names | MISSING | No cuTAMP Pose/Conf/Grasp parameters | Map supported skill variables to real official problem representation |
| GPU particle representation | `cuTAMP/cutamp/utils/common.py:31` | Python candidate dictionaries | MISSING | No batched torch tensors | Use official Particles with actual GPU data |
| Conditional particle initialization | `cuTAMP/cutamp/particle_initialization.py:42`, `:125`, `:149` | domain.samples | MISSING | No official sampler/IK initialization | Call official ParticleInitializer; log counts |
| Differentiable constraint costs | `cuTAMP/cutamp/cost_function.py:205`, `:215`, `:347`, `:428` | Drake boolean checks | MISSING | No differentiable costs | Reuse official CostFunction for supported operators; validate remaining domain constraints |
| Differentiable kinematics and rollout | `cuTAMP/cutamp/rollout.py:84`, `:94` | CPU domain.check/predict | MISSING | No cuRobo batched FK | Validate robot model and frame correspondence before conversion |
| Gradient optimization | `cuTAMP/cutamp/optimize_plan.py:99`, `:133`, `:157`, `:192` | `tamp_geometry.py:220` CuTAMPSolver delegates arbitrary backend | MISSING | Placeholder has paper name | Rename placeholder ExternalGeometrySolverAdapter; only real optimizer can use CuTAMPSolver |
| Per-constraint feasibility thresholds | `cuTAMP/cutamp/constraint_checker.py:22`, `:69` | Boolean GeometryDomain checks | MISSING | No upstream tolerance mask | Use ConstraintChecker and report satisfying particles plus independent execution validity |
| Skeleton prioritization/backtracking | `cuTAMP/cutamp/algorithm.py:303`, `:333`, `:357`, `:510`; paper Algorithm 1 | BFS exclusions | MISSING | Upstream itself leaves dynamic queue additions/resorting/revisits as TODO at :510 | Per user clarification, do not implement or attribute general skeleton search/backtracking to cuTAMP; generator owns all skeleton revisions |
| cuRobo IK, FK, collision and optional motion planning | `cuTAMP/cutamp/particle_initialization.py:149`; `rollout.py:94`; `tamp_world.py:75`, `:185` | Drake IK and trajectory checks | MISSING | No compatible cuRobo installation verified | Install official supported cuRobo v0.7.8 and run independent official demo first |
| Robot embodiment | `cuTAMP/cutamp/tamp_world.py:75`; `robots/__init__.py:70`, `:102` | Zerith mobile manipulator | MISSING | Stock upstream dispatch supports Panda/UR5, not Zerith | Validate cuRobo robot model, collision spheres, tool transform and mobile base mapping; stop at genuine missing-model blocker |
| Full trajectory versus waypoint optimization | `cuTAMP/cutamp/particle_initialization.py:44`; `rollout.py:136`; `algorithm.py:490` | Shared runtime consumes staged waypoints | MISSING | Upstream enable_traj raises; MoveFree/MoveHolding skipped in rollout | Do not claim optimized continuous trajectories; retain physical edge/collision validation |
| Grasp optimization scope | `cuTAMP/cutamp/optimize_plan.py:51`, `:83` | Samples grasp candidates | MISSING | Official optimizer selects Pose/Conf, not Grasp | Report sampled grasp and optimized configurations separately; do not claim gradient updates to fixed grasp |

## Implementation and acceptance gates

1. This document precedes implementation edits. Preserve BFS as STRIPS baseline and remove the placeholder paper name.
2. Generate open programs with the real LLM, declared domains and explicit PRoC3SGenerationFailure. No hidden STRIPS fallback. JSON DSL is a disclosed representation restriction.
3. Add a complete-assignment CCSP backend and typed program-unsat feedback. PRoC3S CCSP and cuTAMP are alternative solvers, never sequential optimization passes.
   User clarification: cuTAMP receives a fixed skill skeleton and optimizes continuous poses/configurations only. PRoC3S or STRIPS owns skeleton generation and revision. Upstream's incomplete general skeleton search is a reported gap, not an adapter feature to recreate.
4. Install official cuTAMP in an isolated environment, run its official demo, preserve command/configuration/version/logs and verify actual optimization before starting the repository adapter. Hardware inventory: RTX 4090, driver 580.95.05, toolkit 12.1, base PyTorch 2.4.1+cu121/Python 3.11.9; this is not yet an installation success claim.
5. Validate the Zerith conversion. Preserve one ParameterizedSkillPlan schema, shared BT/TAMP executor, consumption checks, and observed-state recovery. A failed execution first re-solves geometry for the same program; program-unsat reprompts PRoC3S; exhausted programs reprompt semantic VLM.
6. Exercise A/B/C/D CLI modes and all five requested acceptance tests: real program generation; all-grasps-fail reprompt with changed program; real GPU cost reduction; same-input solver/schema equivalence; failed-execution world refresh and replanning. Tests must not silently substitute mocks for the live/GPU gates.
7. Create `docs/PAPER_FIDELITY_REPORT.md` only with inspected implementation evidence, statuses FAITHFUL/MODIFIED/MISSING and explicit remaining gaps. Do not mark the goal complete without all 15 Definition-of-Done items.
8. User addition: preserve a playable recording for each subsequent successful simulation together with configuration, seed, planner/backend labels and result logs. Do not relabel an old sampling success as a PRoC3S/cuTAMP result.

If dependency, robot-model or missing-interface limitations prevent the requested integration after safe checks, stop at that actual boundary and report evidence; do not create a fake optimizer or claim completion.
