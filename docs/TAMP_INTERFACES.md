# TAMP 层接口格式：VLM 输入与 Skill 注册表

更新日期: 2026-09-21。

本文只描述当前主路径在**模型边界**与**注册表边界**上的数据格式，不评估算法
质量，也不覆盖 legacy 消融路径的内部结构。所有结论都能用第 5 节的命令复现。

## 1. 总览

模型调用只有一个 transport：`OpenAICompatibleChatClient`
(`bt_generation.py:505`)，发送 OpenAI 兼容的 Chat Completions 请求。调用方有
两个，payload 互不相同：

| 调用方 | 代码位置 | system | user | 图像 |
| --- | --- | --- | --- | --- |
| 语义子目标 | `tamp_semantic.py:44` | `tamp_subgoal_v3.txt` 全文 | content 数组：文本 JSON + 图像 | 有 |
| 骨架程序 | `tamp_proc3s.py:143` | `proc3s_program_v1.txt` 全文 | 纯文本 JSON 字符串 | 无 |

Skill 注册表不是配置文件，而是内存中的冻结 dataclass 元组
(`tamp_hierarchy.py:40`)，对外有三个不同的序列化视图（第 3.3 节）。

## 2. VLM 输入格式

### 2.1 公共部分

`system` 消息是仓库内版本化 prompt 文件的全文。`user` 消息承载任务数据。
图像的传输形态不是文件路径：`_wire_messages` (`bt_generation.py:483`) 在发送前
把 `{"path", "sha256", "mime_type"}` 读成 base64 data URL，并重算 sha256 与记录
值比对，不一致时抛 `GenerationError("Observation changed after request validation")`。
因此 trace 里记录的请求体带路径，真实 wire 上是内联图片。

### 2.2 语义层 payload

由 `tamp_semantic.py:51` 组装，字段如下：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `task` | string | 任务自然语言 |
| `world.objects` | object | 对象名 → 元数据；**只保留 `category`、`movable`、`articulated`、`surface` 四个键** |
| `world.facts` | array | 当前事实，保持运行时顺序 |
| `known_objects` | array | 对象名，已排序 |
| `predicates` | object | 谓词名 → arity |
| `abstract_physical_feedback` | array | 抽象失败反馈，可为空 |
| `observations` | array | 每台相机只保留 `camera`、`timestamp_s`、`annotations` |

实测请求（`runs/seed500-501-fix-20260920/live_seed500/runtime/tamp_trace.jsonl`
的 `semantic_model_request` 事件）：

```json
{
  "task": "Pick up the red object and hold it.",
  "world": {
    "objects": {"pick_target": {"category": "manipuland", "movable": true,
                                "articulated": false, "surface": null}},
    "facts": [{"predicate": "gripper_empty", "arguments": []},
              {"predicate": "observed", "arguments": ["pick_target"]}]
  },
  "known_objects": ["pick_target"],
  "predicates": {"observed": 1, "at_pick_pose": 1, "holding": 1, "gripper_empty": 0},
  "abstract_physical_feedback": [],
  "observations": [
    {"camera": "head_camera", "timestamp_s": 0.0,
     "annotations": [{"object": "pick_target", "body": "living_room_box_0::base_link",
       "category": "manipuland", "movable": true, "articulated": false, "surface": null,
       "bbox_xyxy": [281, 337, 291, 351], "source": "simulator_label_mask"}]},
    {"camera": "left_wrist_camera", "timestamp_s": 0.0,
     "annotations": [{"object": "pick_target", "body": "living_room_box_0::base_link",
       "category": "manipuland", "movable": true, "articulated": false, "surface": null,
       "bbox_xyxy": [315, 219, 330, 236], "source": "simulator_label_mask"}]}
  ]
}
```

三条剥离规则值得记住：

1. `world.objects` 只留四个元数据字段，关节角、位姿、质量、`static_in_sdf`
   等全部不进入请求。
2. runner 传入的 `robot` 字段在这一层被丢弃；模型看不到任何机器人状态。
3. 语义层 payload **不含任何技能信息**（没有技能名、没有注册表、没有约束）。

`observations[i].annotations` 由 `SceneSmithWorldObserver.capture_images` 生成，
来源标注为 `simulator_label_mask`，即用仿真的 label mask 计算 bbox，不涉及
检测模型。

### 2.3 骨架层 payload

由 `tamp_proc3s.py:163` 组装，字段如下：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `goals` | array | 待实现的语义目标（来自语义层输出） |
| `world` | object | 与语义层同构：objects 只留四字段 + facts |
| `skills` | array | **技能注册表的模型视图**，白名单 6 字段 |
| `domain_samplers` | object | 参数角色 → 采样器名，固定为 `DOMAIN_SAMPLERS` |
| `constraint_feedback` | array | 上一轮失败反馈，经白名单过滤 |
| `excluded_skeletons` | array | 已被判定失败的骨架标识 |
| `previous_program` | object/null | 仅在有反馈时给出上一轮程序原文 |

与语义层相比有三处差异：技能注册表进入 payload；`facts` 改为
`sorted(world.facts)`；没有图像。

`constraint_feedback` 在进入 payload 前经过一次接收边界过滤
(`tamp_proc3s.py:150`)：`ProgramFailure.from_feedback` 规范化后，技能名必须属于
当前注册表，涉及对象必须存在于当前世界，否则替换为空串；粒子、IK、碰撞的
数值细节不会被传给模型。

### 2.4 结构化输出约束

| 层 | 配置项 | 配置值 | 实际 response_format |
| --- | --- | --- | --- |
| 语义 | `subgoal_model.response_format` | `json_schema` | 严格 schema |
| 骨架 | `skill_model.response_format` | `json_object` | `{"type": "json_object"}` |

语义 schema 由 `goal_response_format` (`tamp_model.py:29`) 依据当前谓词表生成：
每个谓词一个 `anyOf` 分支，`predicate` 用单元素 enum 锁定，`arguments.items.enum`
为已知对象名，`minItems == maxItems == arity`，对象与数组均
`additionalProperties: false`。因此 `gripper_empty` 只能有 0 个参数、`holding`
只能填当前场景中存在的对象名，格式错误在 provider 侧即被拒绝。

骨架层也实现了同类的严格 schema：`_response_format` (`tamp_proc3s.py:100`) 会按
注册表逐技能生成 `skill`/`arguments`/`continuous_variables` 变体，并用
`DOMAIN_SAMPLERS` 的值锁定 `domains.sampler`。但仓库配置选择 `json_object`，
在线运行实际走的是「模型自由输出 + `parse_proc3s_program` 本地严格解析」。
两者都不是静默降级：解析失败会带错误信息重问，耗尽预算抛
`PRoC3SGenerationFailure`。

## 3. Skill 注册表格式

### 3.1 数据结构

```python
SkillSpec(
    name: str,
    symbolic_parameters: tuple[str, ...],       # 离散对象槽位
    geometric_parameters: tuple[str, ...],      # 连续参数槽位
    preconditions: tuple[PredicateGoal, ...],   # 用 $name 占位
    add_effects: tuple[PredicateGoal, ...],
    delete_effects: tuple[PredicateGoal, ...] = (),
    constraints: tuple[str, ...] = (),
    supports_geometric_conditioning: bool = True,
    runtime_action: str | None = None,
)
```

`SkillRegistry` (`tamp_hierarchy.py:62`) 是 `dict[name -> SkillSpec]`，构造时拒绝
重名；对外只提供迭代、按名索引与 `.names`。
`PredicateGoal` (`tamp_hierarchy.py:22`) 是 `{predicate: str, arguments: tuple[str]}`。

构造期不变量（`SkillSpec.__post_init__`）：名字与两套参数名不得重复；
preconditions、add_effects、delete_effects 中任何以 `$` 开头的参数都必须出现在
`symbolic_parameters` 里，否则注册即失败。

### 3.2 当前注册内容

`picklift_registry()` (`tamp_hierarchy.py:280`) 的实测 dump：

```json
[
  {"name": "NavigateToPick", "symbolic_parameters": ["object"],
   "geometric_parameters": ["base_pose"],
   "preconditions": [{"predicate": "observed", "arguments": ["$object"]}],
   "add_effects": [{"predicate": "at_pick_pose", "arguments": ["$object"]}],
   "delete_effects": [],
   "constraints": ["corridor", "base_collision", "reachability"],
   "supports_geometric_conditioning": true, "runtime_action": "NavigateTo"},

  {"name": "PickLift", "symbolic_parameters": ["object"],
   "geometric_parameters": ["grasp_pose", "approach_pose"],
   "preconditions": [{"predicate": "observed", "arguments": ["$object"]},
                     {"predicate": "at_pick_pose", "arguments": ["$object"]},
                     {"predicate": "gripper_empty", "arguments": []}],
   "add_effects": [{"predicate": "holding", "arguments": ["$object"]}],
   "delete_effects": [{"predicate": "gripper_empty", "arguments": []}],
   "constraints": ["ik", "joint_limits", "joint_edge", "contact"],
   "supports_geometric_conditioning": true, "runtime_action": "ExecutePickLift"}
]
```

### 3.3 三种序列化视图

| 视图 | 字段集合 | 生成位置 | 消费方 |
| --- | --- | --- | --- |
| trace 视图 | 全部 9 个字段（`dataclasses.asdict`） | `tamp_online.py:121` | `tamp_trace.jsonl` 的 `skill_registry` 事件 |
| 模型视图 | 6 个：`name`、`symbolic_parameters`、`geometric_parameters`、`preconditions`、`add_effects`、`delete_effects` | `tamp_proc3s.py:159` | 骨架层 payload 的 `skills` |
| 运行时视图 | `runtime_action` 字符串 | `tamp_scenesmith_online.py:137` | 执行器 `bindings` → BT 叶子 |

模型视图刻意去掉 `constraints`、`supports_geometric_conditioning` 与
`runtime_action`：模型只应看到符号契约，不应看到运行时的物理约束标签与叶子名。
运行时视图另外会校验 `runtime_action` 同时存在于 `bindings` 与
`bt_core.SKILLS` (`tamp_scenesmith_online.py:149`)。

### 3.4 各字段的实际消费方

| 字段 | 消费者 | 用途 |
| --- | --- | --- |
| `name` | 全部层 | 索引键、校验、trace |
| `symbolic_parameters` | `validate_skill_program`、STRIPS 搜索、CCSP/cuTAMP、执行器 | 参数集合校验与对象绑定 |
| `geometric_parameters` | `validate_skill_program`、`refine_goals` | 决定性开放变量名（形如 `base_pose_0`）与角色映射 |
| `preconditions` | `refine_goals` (`tamp_hierarchy.py:201`) | STRIPS 前置条件 |
| `add_effects` | `refine_goals`、`_bind_effect` | 状态推进与 `expected_effects` |
| `delete_effects` | `refine_goals` | 状态推进 |
| `constraints` | **无** | 仅进入 trace |
| `supports_geometric_conditioning` | `tamp_geometry.py:210`、`tamp_ccsp.py:125`、`tamp_cutamp.py:168`、执行器 | 复制到 action 并在执行前核对一致性 |
| `runtime_action` | 执行器 | BT 叶子绑定 |

`expected_effects` 由 `add_effects` 绑定而来，是 `verify_expected_effects`
(`tamp_online.py:78`) 判定技能是否达成预期效果的依据。

## 4. 已知边界

**`constraints` 目前没有任何代码消费。** 全仓检索 `SkillSpec.constraints` 的读取
点为零；`corridor`、`ik`、`joint_edge`、`contact` 这些标签是描述性的，真正的
检查逻辑按技能名硬编码在 `SceneSmithPickDomain.check()` 中。把该字段当作
「约束由注册表驱动」的证据会误导。

**`facts` 顺序在两层不一致。** 语义层传运行时顺序，骨架层传
`sorted(world.facts)`。对模型无影响，但对比两次请求的 trace 时需要注意。

**模型看不到物理约束标签。** 骨架层 payload 的 `skills` 不含 `constraints`，
模型只能从符号前置条件和反馈推断结构。这是刻意的边界，但也意味着模型无法
通过注册表得知某技能需要走廊检查或关节边检查。

**语义层没有任何技能信息。** VLM 在语义层只输出谓词与对象，不能命名技能，
这是「模型权限收在语义层」这条设计的落点。

## 5. 复现命令

导出注册表：

```bash
PYTHONPATH=. .venv/bin/python -c "
import dataclasses, json
from planner.src.tamp.hierarchy import picklift_registry
print(json.dumps([dataclasses.asdict(s) for s in picklift_registry()], indent=2))
"
```

导出语义层严格 schema：

```bash
PYTHONPATH=. .venv/bin/python -c "
import json
from planner.src.tamp.model import goal_response_format
fmt = goal_response_format('json_schema', schema_name='scenesmith.tamp.goals.v3',
                           predicate_arity={'observed': 1, 'at_pick_pose': 1,
                                            'holding': 1, 'gripper_empty': 0},
                           known_objects=frozenset({'pick_target'}))
print(json.dumps(fmt, indent=2))
"
```

从一次真实运行中提取请求：`tamp_trace.jsonl` 里的 `semantic_model_request`、
`proc3s_model_request` 与 `skill_registry` 事件分别保留两次 payload 与注册表快照。
