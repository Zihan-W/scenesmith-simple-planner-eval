# Online manipulation TAMP architecture audit

The current path is reset observation (head and wrist RGB plus `world_snapshot.json`) → `goals.v2` semantic goal → target-specific BFS refinement → `program.v1` model-authored numeric sampling domains → SceneSmith corridor/IK/joint-edge checks → one BT for the whole task → one `scene-eval` rollout. The BT generator in `bt_generation.py` is a separate baseline with strict input hashes, compiler validation, retry, MDSL and HTML output.

| Concern | Current implementation | Decision |
| --- | --- | --- |
| Data | `GoalPredicate(name,target)`, `Subgoal(skill,arguments)`, `PlanSketch(variables,steps)`, `GroundedSkill` | Refactor into n-ary semantic predicates, open-variable skill skeletons, and parameterized actions. |
| Predicates | `observed`, `at_pick_pose`, `holding`, scoped to one target | Refactor as typed n-ary facts with object validation. |
| Operators | Two target-specific `SymbolicOperator`s in `tamp_pipeline.py`; BFS assumes every predicate argument is the same target | Replace with a registry-backed generic STRIPS search. |
| Skills | `NavigateToPick`, `PickLift`; BT leaves `NavigateTo`, `ExecutePickLift` | Keep existing runtime and public robot actions; register skill contracts in one place. |
| Geometry | SceneSmith corridor check, staging/grasp/lift IK, joint limits and joint-edge collision checks; physical rollout distinguishes geometric from dynamic feasibility | Keep checkers; add system-owned domain generation, structured constraint results, batching and a replaceable backend. |
| Feedback | Detailed failures and re-prompt at whole-plan level | Refactor into skill retry, geometric reparameterization, skeleton replan and semantic replan. |
| Execution | Ground whole plan, compile BT, execute from one reset; one final task result | Keep for BT and `legacy-vlm-domain` ablation; add incremental execute/observe/verify for hierarchical TAMP. |
| Logging | `tamp_plan.json`, candidate BT HTML, `trace.csv`, `summary.json` | Keep artifacts; add JSONL decision trace and layer-specific metrics. |

The primary architectural mismatch is that the current second model request authors numeric sampler bounds, while the hierarchical path requires the model to choose only semantic subgoals. The current BT `ExecutePickLift` leaf can consume the grounded lateral offset and joint waypoints through `JointWaypointPickLiftSkill`; the normal BT expert remains available, so geometric conditioning must be reported per skill rather than assumed. The environment has not yet been shown to satisfy the physical PickLift task, and a recorded proposal is not evidence of a live VLM call.

Implementation order: keep both existing baseline CLIs intact; add shared schema and registry, then hierarchical semantic/symbolic/geometry modules, then a separate online executor and verifier, finally compare both paths through the same runtime and report live results. A cuTAMP adapter must state explicitly whether it uses the original differentiable optimizer or only the SceneSmith sampling backend.
