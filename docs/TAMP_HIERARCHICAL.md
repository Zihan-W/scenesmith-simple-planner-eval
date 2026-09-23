# 分层 TAMP 在线路径

`planner/src/tamp/cli.py` 是通用入口。正常运行不需要手工 `--base-candidate`：系统根据目标位置、机器人朝向和技能参数范围生成候选，再以 SceneSmith 的走廊、IK、关节边和碰撞检查筛选。`--base-candidate` 仅是 legacy/debug 覆盖。模型只能提交 `goals.v3` 谓词和对象名；`SkillRegistry` 根据前置条件/效果生成 `skill_program.v2` 开放变量骨架；采样求解器才填写连续几何参数。

每个已参数化技能由原 `JsonBtPolicy` 导航或抓取叶节点执行。技能完成后重新读取环境观测并核对预期效果；失败依次尝试有限次技能重试、几何重参数化、技能骨架重规划、语义重规划。`experiments/tamp_hierarchical_config.json` 控制次数上限、模型名、温度和采样预算。每次运行写 `tamp_trace.jsonl`、`skill_steps.jsonl`、逐技能 BT 与 `result.json`，其中 `model_source` 区分录制输入和真实模型调用。

```bash
python -B planner/src/tamp/cli.py \
  --planner tamp --tamp-mode hierarchical --geometry-backend sampling \
  --experiment experiments/navigation_picklift_tamp_12cm.json \
  --repository-root "$PWD" --scene-root "$SCENE_ROOT" \
  --output-root runs/tamp-hierarchical-demo \
  --task "导航到红色物体并抓起保持"
```

做公平对照时，两条路径使用同一实验、场景和显式 `--seed`；该参数会传给 BT 执行、
hierarchical reset 和 legacy rollout。未指定时保留原默认：BT 使用实验配置的 seeds，
TAMP 使用 500。共享运行时不等于两种策略自动具有相同控制参数或相同成功率。

在线模式需要 `OPENAI_BASE_URL`、`OPENAI_API_KEY`。添加 `--recorded-subgoals experiments/inputs/tamp/recorded_navigation_picklift_goals_v3.json` 仅用于确定性回放，不能证明 VLM 在线工作。默认 `--skill-planner strips` 使用确定性符号搜索；`--skill-planner proc3s` 实际调用 `skill_model` 生成开放变量和命名采样域，失败抛出 `PRoC3SGenerationFailure`，不会回退 BFS。`--geometry-backend proc3s` 使用完整赋值的拒绝采样，失败反馈给程序生成层；`sampling` 保留旧批次排序 baseline。当前尚无 `Place` 或多物体重排；Zerith cuTAMP 接入及验证边界见 [第三层报告](CUTAMP_INTEGRATION_VALIDATION.md)。

模型配置还包含 `response_format`：语义模型默认使用 `json_schema`，从当前谓词、元数和
对象名生成严格 schema；legacy 自由键参数程序使用 `json_object`，再经原 DSL parser 校验。
如服务不支持这些格式，可在显式配置中选 `text`（仍要求 JSON-only 提示、本地校验和重试）；
程序不会收到错误后悄悄降级。共享 BT transport 的新参数是可选的，原 BT 请求不附加该字段。
格式依据 [OpenAI Structured Outputs 文档](https://developers.openai.com/api/docs/guides/structured-outputs?api-mode=chat)。
当前配置服务的 `gpt-4.1-mini` 已通过一次严格 goals.v3 在线请求；这不证明任意兼容服务都支持它。

`--geometry-backend cutamp --cutamp-config <file>` 接入独立 CUDA 环境的真实优化器，并执行 SceneSmith 精确后验；配置与实测状态见 [第三层报告](CUTAMP_INTEGRATION_VALIDATION.md)。`GeometricParameterSolver` / `ExternalGeometrySolverAdapter` 只是通用求解契约及结果校验器，不是 cuTAMP。现有 SamplingSolver 是独立的自研 baseline；真实 cuTAMP 只能作为固定骨架的连续姿态/配置优化后端，骨架生成和修订仍归 PRoC3S/STRIPS。详见 [论文对齐报告](PAPER_ALIGNMENT.md)。

## 接口与验证边界

- `tamp_semantic.py` 只传语义属性、谓词与带时间戳的标注图像；测量关节值保留在世界状态日志，不进入模型请求。模型格式/传输失败使用配置中的有界重试。
- `tamp_hierarchy.py` 的同一 Skill Registry 供符号搜索、几何求解、执行绑定和日志使用。`skill_program.v2` 校验所有对象参数、开放变量与共享变量类型。
- `tamp_geometry.py` 批量检查后按 domain 提供的分数排序，并记录每个候选；相同共享变量的取值必须一致。通用 ExternalGeometrySolverAdapter 拒绝更换骨架中的技能或对象。
- `tamp_scenesmith.py` 从实测关节与物体状态检查进场路径，包含夹爪张开命令；抬升路点逐段检查，并附带假设已抓住的目标。抓取排序先比较两指间隙平衡，再比较关节裕度。抬升检查的夹爪几何仍使用计划张开宽度，闭合后的接触动力学必须由运行时检查和物理 verifier 判定。
- `tamp_scenesmith_online.py` 将参数交给原 BT 叶节点；导航走廊检查使用当前状态。技能边界即时渲染相机而不推进物理时间，保存 `observations/*/manifest.json`，标注名称与符号对象保持一致。只有停稳且释放底盘控制权，才验证 `at_pick_pose`。
- `tamp_online.py` 在成功、失败之后都刷新世界状态；成功完成中间目标后继续选择下个目标，不消耗失败重规划预算。几何重参数化会排除已经失败的采样参数，即使重新求解得到的 IK 数值略有变化。

运行时失败的最后一次动作拒绝及技能诊断保留在 `skill_execution.failure_details` 和逐技能结果中；
反馈给语义层的仅有原因和涉及物体。已观测模型名映射回符号对象名，机器人模型不作为场景障碍物列入。

接近路径也属于 solver 的变量：域生成器自动尝试标定单段路径及逐段 IK 的接近路径，
每一段都通过同一碰撞检查。可选的 `approach_waypoints` 由共享抓取状态机按顺序消费，
到达最终抓取路点之前不会闭爪；旧计划不含此字段时仍使用原单段行为。

参数化关节技能还会根据 Runtime 步长和 URDF 速度限制统一缩放关节增量，避免底层逐轴
截断把已验证的关节插值线段扭曲成另一条末端路径。缩放以观测中的 `q_commanded` 为基准，
实际状态仍由运行时进行完整安全检查；未改动普通 BT 专家、碰撞阈值或底层限速行为。

带载抬升复用普通专家的接触跟踪容差及速度门槛。抬升从实际求解的抓取位姿出发，
使用共享专家的 `lift_distance_m`（默认 0.1 m，尊重实验 override），计入 IK 误差预算，
并检查预测携带物体的最终高度。接触转动/滑移仍须实际验证。TAMP 的 BT 叶节点等待
观测到任务成功，不使用下一时刻完成预测；普通 BT baseline 的完成行为不变。

当前实现包含 VLM-TAMP 风格的语义中间状态、自研 STRIPS 开放参数骨架和自研采样求解器。确定性 BFS 不称为 PRoC3S，采样过滤不称为 cuTAMP。当前执行技能是导航控制器和关节路点抓取状态机；尚未证明 DP/VLA 几何条件化或论文 GPU 优化算法。

物理 demo 的成功与否只认各运行目录的 `result.json` 和 verifier 日志。真实模型调用、几何可行、导航成功均不能单独证明抓取成功。后续成功仿真还须留存可播放录像、配置、seed 和 planner/backend 标签。

追加 `--record-html` 会使用现有 Meshcat 录制功能保存 `simulation.html`，失败或异常时也保留。
`planner_config.json` / `resolved_experiment.json` 保存标签与配置。逐技能
`parameter_consumption.json` 核对开放变量绑定是否真正进入运行时；缺失或不一致时
`geometry_guarantee=false` 并警告。这个标志仅说明参数消费，不等于物理成功保证。
失败反馈采用 `LowLevelFailure → ProgramFailure → SemanticFailure`，上层不接收低层数值。

PRoC3S 程序生成独立验收（不是物理成功）：加载授权 API 环境后运行
`PYTHONPATH=. .venv/bin/python scripts/check_proc3s_generation.py --output-root runs/proc3s-generation-check`。
`runs/paper-alignment-20260919/proc3s_generation_001` 已保存一次真实调用成功的开放程序和请求/响应日志。

PRoC3S 的 JSON 程序和命名采样域是相对官方 Python exec / gen_domain 的明确限制；
CCSP 使用 SceneSmith 几何检查和预测，而非官方 Raven twin 的物理动作仿真。
导航在现有候选包络内连续采样且保持已测朝向，抓取偏移在现有允许范围内连续采样。
接近路径仍来自标定支持的离散模式。有限预算耗尽不等于数学证明无解。

已验证 `runs/tamp-hierarchical-20260919/live_010/result.json`：真实 VLM、seed 500，
无人工底盘/抓取参数，导航及抓取均通过验证；实际抬高 0.080134 m、稳定保持 3.1 s。
本轮 1 次 VLM 调用、2 次技能执行、0 次重试/重规划。全仓回归 248 项通过（1 skip）；
随后完成判定修正的 61 项 TAMP 测试及 BT 核心测试通过。仅证明这一已选场景/种子，
不代表多场景成功率。完整 A–G 报告见 [TAMP_IMPLEMENTATION_REPORT.md](TAMP_IMPLEMENTATION_REPORT.md)。

## 通用外部求解器契约（不是 cuTAMP 实现）

官方 cuTAMP 已在独立环境安装，Zerith 固定骨架后端通过 CLI 的显式 `--cutamp-config` 加载，不会回退采样。下面描述通用外部求解契约；具体实现和验证见 [第三层报告](CUTAMP_INTEGRATION_VALIDATION.md)。
独立 Python 集成可使用 `ExternalGeometrySolverAdapter(backend)`；后端需要实现：

```python
solve(world, program, initial_state, *, excluded_assignments) -> ParameterizedSkillPlan
```

1. 将 `program.steps[*].continuous_variables` 映射为优化变量；同名变量必须共享，
   不得改动技能名称、对象参数或目标语义。适配器给后端独立快照，拒绝语义更改。
2. 从 `world` 的观测对象、机器人状态及 `initial_state` 建立与仿真一致的场景。
   外部优化器的坐标系、单位、碰撞模型需显式核对；当前仓库公共 pose 使用米及 wxyz 四元数。
3. 将优化结果转换成 `ParameterizedSkillAction` 和完整的 `assignments`。
   对当前两个技能必须生成运行时消费的导航目标或抓取关节路点，不能只返回不被执行的 TCP。
4. 用同一 SceneSmith IK/关节限位/整段碰撞检查复核外部候选。
   不可行时抛出带 `ConstraintResult` 的 `GeometricUnsat`；遵守失败参数排除集。
5. 在 `IncrementalTampRunner.solver_factory` 注入该适配器，再接通 CLI 后端构造。
   执行器和语义/符号规划器不需要替换。运行程序语义契约、A–E 恢复测试以及真实物理 demo；
   具体接入结果以第三层报告为准，不以契约测试替代真实执行。

该协议本身不证明外部优化器已安装、候选已复核或 GPU 求解有效。
