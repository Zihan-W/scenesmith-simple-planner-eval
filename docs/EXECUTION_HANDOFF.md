# 两条路径最终交给执行层的东西

更新日期: 2026-09-21。

对比对象：

- **VLM → BT**：`scenesmith-generate-bt`（`planner/src/bt/generation.py`），
  模型一次生成整棵树，`--planner bt`。
- **VLM-TAMP + PRoC3S**：`scenesmith-plan --planner tamp`（`tamp_cli.py`），语义层出
  目标、骨架层出结构、几何层出数值，逐技能执行。

## 1. 结论

两条路**最终都落到同一个执行接口**：一棵 BT JSON 树 + policy options，由
`bt_runtime.make_policy()` 构造 `JsonBtPolicy` 来 tick。BT 是两条路共同的执行 IR。

| 维度 | VLM → BT | VLM-TAMP + PRoC3S |
| --- | --- | --- |
| 交给执行层的东西 | 整棵树 `generated_bt.json` | 单技能 `skill_bt.json` + `policy_options` 里的连续参数 |
| 树规模 | 多个 action/condition（`root → selector → …`） | 固定三层：`root → sequence → 1 个 leaf` |
| 结构由谁决定 | 模型（MAIN_SEQUENCE） | 符号搜索 / PRoC3S 生成器 |
| 连续数值由谁决定 | 只能写成节点 `args` 字面量 | 几何求解器，经 `policy_options` 注入 |
| 生命周期 | 一次 reset 跑到底 | 每个技能新建 policy、执行、暂停、重观测、重新求解 |
| 失败语义 | 叶子返回 FAILURE，树继续或结束 | 四级恢复（skill_retry → geometry_retry → skill_replan → semantic_replan） |

## 2. VLM → BT：产物与执行入口

### 2.1 输入与输出

请求 schema `scenesmith.bt_generation.request.v1`，字段为
`schema`、`request_id`、`environment{path, sha256}`、`task{description, plan_path,
plan_sha256}`、`observations`、`model`；环境、任务计划与相机图片都由 SHA-256 绑定
(`bt_generation.py:130`)。

`write_result` (`bt_generation.py:433`) 一次写出五个文件：

| 文件 | 内容 |
| --- | --- |
| `generated_plan.json` | 完整结果：`raw_response`、`parsed_response`、`generation`、`mdsl`、`tree`、`mdsl_sha256` |
| `generated_bt.json` | **只有 `tree`**，即执行层真正读的文件 |
| `generated_bt.mdsl` | MDSL 文本 |
| `generated_bt.mmd` | Mermaid 可视化 |
| `generated_bt.html` | 离线交互树图 |

### 2.2 树的 schema

节点结构固定为 `{"kind", "name", "args", "children"}`，`kind` 只允许
`root` / `selector` / `sequence` / `action` / `condition`
(`bt_core.py:151` 的 `to_dict`，`bt_runtime.py:40` 的 `load_tree`)。`action`/`condition`
的 `name` 必须命中 `SKILLS` 注册表，且 `args` 个数必须等于签名长度、`children` 必须为空。

真实树（`experiments/inputs/bt_picklift/generated_plan.json`）：

```json
{"kind": "root", "name": "", "args": [], "children": [
  {"kind": "selector", "name": "", "args": [], "children": [
    {"kind": "condition", "name": "PickLiftSucceeded", "args": [], "children": []},
    {"kind": "sequence", "name": "", "args": [], "children": [
      {"kind": "action", "name": "Wait", "args": ["1.0"], "children": []},
      {"kind": "action", "name": "ExecutePickLift", "args": [], "children": []}]}]}]}
```

导航 profile 的同构例子把叶子换成 `NavigateTo["2.8","0.0","0.0","world"]` 与
`SetDualJointTargets["-0.08","0.06","0.055","0.4"]`，条件换成
`ParkedDualTargetsReached`。

### 2.3 执行入口

`--planner bt --execute` 时 `tamp_cli.py` 把
`options["bt_json_input"] = <output>/generated_bt.json`，然后复用运行的
`experiment.run()`。`make_policy` (`bt_runtime.py:504`) 读树、校验叶子、按树里出现的
技能构造 `Navigator` 与 `PickLift` 专家组，返回一个 `JsonBtPolicy`。

也就是说：**BT 路径交给执行层的唯一东西就是 `generated_bt.json`**，其余四个文件是给人和
审计看的；连续参数没有任何旁路，只能写死在 `args` 里。

## 3. VLM-TAMP + PRoC3S：产物与执行入口

### 3.1 几何层的输出：`ParameterizedSkillAction`

几何求解器返回 `ParameterizedSkillPlan`，其中的
`ParameterizedSkillAction` (`tamp_hierarchy.py:103`) 是：

| 字段 | 内容 |
| --- | --- |
| `skill_name` | 注册表里的技能名，如 `NavigateToPick` |
| `symbolic_args` | 对象绑定，如 `{"object": "pick_target"}` |
| `geometric_parameters` | 候选采样值 + 计算出的赋值（`base_pose`/`grasp_pose`/`approach_pose`）+ `checks`（IK、关节边、走廊、抬升路点）+ `candidate_identity` |
| `expected_effects` | 由 `add_effects` 绑定得到的谓词，供 `verify_expected_effects` 使用 |
| `supports_geometric_conditioning` | 是否声明几何条件化 |
| `parameter_bindings` | 参数名 → 开放变量名（如 `{"grasp_pose": "g0"}`） |

### 3.2 执行器做的三步转换

`SceneSmithSkillExecutor.execute()` (`tamp_scenesmith_online.py:143`) **不把上述 dataclass
直接交给运行时**，而是：

1. **选 binding**：按注册表的 `runtime_action` 取出叶子构造函数
   （`NavigateTo` 或 `ExecutePickLift`），并校验它同时存在于 `bindings` 与
   `bt_core.SKILLS`。
2. **把连续参数塞进 policy options**：
   `expert_grasp_lateral_offset_m` 取横向偏移；
   `tamp_joint_skill_plan` 取 staging/grasp 关节与抬升、接近路点。
3. **编译成一棵单技能 BT**，写 `skills/skill_NNN/skill_bt.json`，把
   `options["bt_json_input"]` 指向它，并打上
   `options["tamp_generation"] = {"mode": "tamp", "model_called": …, "candidate": …}`，
   然后调用与 BT 路径同一个 `make_policy`。

导航技能的真实产物（`live_seed500/runtime/skills/skill_001/skill_bt.json`）：

```json
{"kind": "root", "name": "", "args": [], "children": [
  {"kind": "sequence", "name": "", "args": [], "children": [
    {"kind": "action", "name": "NavigateTo",
     "args": ["2.6853058166724724", "2.881963489761692", "-3.141592649145102", "world"],
     "children": []}]}]}
```

抓取技能是同一个形状，叶子换成 `ExecutePickLift` 且 **`args` 为空**——它的连续参数全部
通过 `policy_options` 传入，而不是写进节点。

### 3.3 空 args 的叶子怎么拿到参数

`bt_runtime._picklift` (`bt_runtime.py:378`) 在构造 policy 时检查
`options["tamp_joint_skill_plan"]`；存在时用
`JointWaypointPickLiftSkill` 替换标定的 PickLift 专家组，状态机为
`pregrasp → align → close → verify(逐路点抬升) → hold`。导航叶子则直接消费写进
`args` 的 `(x, y, yaw, frame)`。

### 3.4 每个技能的产物

| 文件 | 内容 |
| --- | --- |
| `skills/skill_NNN/skill_bt.json` | 交给执行层的单技能树 |
| `skills/skill_NNN/result.json` | 该技能的成功/原因/耗时/`episode_finished` |
| `skills/skill_NNN/parameter_consumption.json` | 参数消费审计 |

审计文件把「哪些几何参数真的被运行时消费」记成：

```json
{"supports_geometric_conditioning": {"grasp_pose": true, "approach_pose": true},
 "geometry_guarantee": true,
 "parameter_bindings": {"grasp_pose": "g0", "approach_pose": "a0"},
 "scope": "runtime_parameter_consumption_not_physical_success"}
```

`geometry_guarantee=false` 时执行器只发 `RuntimeWarning` 并把 `false` 写进产物，不会阻止
执行。`scope` 字段明确声明：这只表示参数被消费，**不代表物理成功**。

## 4. 交汇点

两条路共享三件事，这也是它们能被公平对比的前提：

1. **同一个解释器**：`JsonBtPolicy`，动作表固定为 `Wait` / `NavigateTo` /
   `ExecutePickLift`，条件表为 `PickLiftSucceeded` / `BaseParkedAtPickPose`。
2. **同一个校验器**：`load_tree` 对深度（≤16）、节点数（≤64）、节点字段、技能签名做
   严格校验，未注册技能直接报错。
3. **同一个入口 option**：`options["bt_json_input"]` 指向任意一棵合法 BT JSON。

差别只在树的来源、树的大小、以及连续参数走 `args` 还是走 `options`。`metadata["generation"]["mode"]`
与 `diagnostics()["tamp_used"]` 会把这一区别写进产物，便于事后区分。

## 5. 容易误判的三点

**BT 路径没有"求解器复核"环节。** 数值只能以字面量写进 `args`，树一旦生成就直接执行；
TAMP 路径的数值先经过走廊/IK/关节边/抬升路点检查，再以 options 注入。

**TAMP 的叶子名字与 BT 路径完全相同。** 单看 `skill_bt.json` 无法区分两条路，必须看
`tamp_generation` 或 `parameter_consumption.json`。

**几何保证不等于任务成功。** TAMP 的 `expected_effects` 校验与任务 evaluator 共用
`holding` 判据；实际是否抓起仍由接触与抬升判定，`parameter_consumption.json` 的
`scope` 字段已显式声明这一点。

## 6. 复现

```bash
# BT 路径：只生成，不执行
scenesmith-generate-bt --request <request.json> --output-dir <new-dir>

# BT 路径：生成并执行
scenesmith-plan --planner bt --request <request.json> --output-root <new-dir> \
  --execute --experiment <experiment.json> --repository-root "$PWD" --scene-root <scene>

# TAMP 路径：逐技能执行
scenesmith-plan --planner tamp --tamp-mode hierarchical \
  --skill-planner proc3s --geometry-backend proc3s \
  --experiment <experiment.json> --repository-root "$PWD" --scene-root <scene> \
  --task "..." --output-root <new-dir>
```

产物核对：BT 路径看 `generated_bt.json` 的 `tree`；TAMP 路径看
`skills/*/skill_bt.json`、`skills/*/parameter_consumption.json` 与
`tamp_trace.jsonl` 的 `skill_execution` 事件。

## 7. 代码位置

| 关注点 | 位置 |
| --- | --- |
| BT 生成与产物落盘 | `bt_generation.py:130`（请求）、`:433`（写出） |
| BT 语法与序列化 | `bt_core.py`（`Node`、`to_dict`、`to_mdsl`、`SKILLS`） |
| BT 载入校验与解释器 | `bt_runtime.py:40`（`load_tree`）、`:151`（`JsonBtPolicy`）、`:504`（`make_policy`） |
| TAMP 动作契约 | `tamp_hierarchy.py:103`（`ParameterizedSkillAction`） |
| TAMP 执行器与单技能 BT | `tamp_scenesmith_online.py:143`（`execute`）、`:137`（`bindings`） |
| 抓取技能参数消费 | `bt_runtime.py:378`（`_picklift`）→ `tamp_execution.py`（`JointWaypointPickLiftSkill`） |
