# Planner：任务规划与共享技能执行

本目录由原 `examples/` 中的规划实现整理而来。它依赖 `simulation.src` 的公共能力，
不包含另一套仿真 Runtime。完整 Bash 命令见[项目 README](../README.md)。

```text
planner/
├── src/
│   ├── bt/          # 请求校验、模型生成、严格 BT 编译、可视化与 tick 执行
│   ├── tamp/        # 语义子目标、技能程序、CCSP、几何绑定与闭环调度
│   └── skills/      # BT/TAMP 共用的关节轨迹 PickLift
├── resources/
│   ├── prompts/    # 原提示词，内容未改变
│   └── bt_viewer.html
└── examples/       # 纯 BT tick 演示
```

## BT 是否已模块化

**BT 生成已经是完整的模块接口；生成与物理执行是分开的。**
`src/bt/generation.py` 提供输入校验、可注入模型客户端、编译、结果写入和 CLI。
本次保留这些实现，新增包级命令入口，不复制生成器或改变提示词。
它不是“任意一句话自动完成任意机器人任务”的通用规划系统。

### 生成模块输入

入口：`python -m planner.src.bt --request <request.json> --output-dir <新输出目录>`。
也可继续使用 `python -m planner.src.bt.generation`。

| 输入 | 必需内容 |
|---|---|
| `schema` | `scenesmith.bt_generation.request.v1`；兼容原 PickLift v1 请求 |
| `request_id` | 非空的请求标识 |
| `environment` | 环境 JSON 的相对路径及 SHA-256 |
| `task` | 自然语言描述、具体任务计划相对路径、计划 SHA-256 |
| `observations` | PickLift：头部和左腕 PNG，按顺序携带路径、哈希和媒体类型；导航：空列表 |
| `model` | 模型 ID、1–5 次最大生成尝试 |
| 传输设置 | `OPENAI_BASE_URL` 与 `OPENAI_API_KEY`，或对应 CLI 参数；不写入请求文件 |

路径必须留在请求所在目录内，文件哈希必须匹配。
PickLift 的图片还要与环境快照中的相机记录、目标可见性对应。
环境/计划 schema 决定使用导航还是 PickLift 编译器。
模型必须严格保留输入计划的技能顺序和参数，不能自行追加技能。

现成输入为 `experiments/inputs/bt_picklift_first_person/generation_request.json`。
它是固定模型的已有快照，不是当前移动场景的重新观测结果。

### 生成模块输出

Python `generate()` 返回结果字典，包含：

`schema`、`request_id`、`request_sha256`、`raw_response`、`parsed_response`、
`generation`、`mdsl`、`mdsl_sha256`、`tree`。

其中 `generation` 记录模型、每次尝试、用量、编译错误、输入哈希与成功尝试；
`tree` 是严格编译后的树。`write_result()` 写入：

- `generated_plan.json`：完整结果字典。
- `generated_bt.json`：执行器可加载的树。
- `generated_bt.mdsl`、`generated_bt.mmd`：文本与 Mermaid。
- `generated_bt.html`：离线树查看器，不是仿真录像。

已有同名输出会被拒绝覆盖。输入无效会明确报错；模型调用或有界编译修订失败
抛出 `GenerationError`，不会用预设答案伪造成功。

Python 调用接口：

```python
from planner.src.bt.generation import load_request, generate, write_result

request = load_request("/path/to/request.json")
result = generate(request, client)  # client 实现 complete(model=..., messages=...)
artifact = write_result(result, "/path/to/new-output")
```

### 执行模块输入与输出

`src/bt/runtime.py: make_policy()` 接收 `FactoryContext`：实验组装后的环境配置、
机器人规格、策略选项和仓库根目录。策略选项中的 `bt_json_input` 指向已编译树，
抓取仍需匹配当前机器人的专家输入、标定与策略配置；TAMP 分支额外提供
`tamp_joint_skill_plan`，复用专家配置并以已绑定关节轨迹执行。

策略通过 `reset(observation, info)` 初始化，`act(observation)` 返回动作请求；
Runtime 完成检查和提交后，通过 `record_action_result(observation, info)` 反馈最终接受结果。
`diagnostics()` 返回 `tree_status`（RUNNING/SUCCESS/FAILURE）、`active_path`、
`reason`、导航/专家阶段及闭合状态；最终任务成功由实际任务观测判定。

统一入口 `python -m planner.src.tamp.cli --planner bt ... --execute` 可串联生成与执行，
需要额外提供匹配的 `--experiment`、`--repository-root`、`--scene-root`。
树生成成功不是物理抓取成功。当前生成器也未提供导航+抓取联合请求 profile；
不能将手写联合树可执行，描述为联合 VLM 输入协议已完成。

## TAMP 入口

`python -m planner.src.tamp.cli` 是通用入口。
`--skill-planner proc3s --geometry-backend proc3s` 选择模型生成开放程序与完整赋值拒绝采样。
默认 STRIPS + SamplingSolver 是不同基线。分层入口直接进入规划/仿真闭环，没有纯规划开关。
本次 +100 mm、无固定 seed 的场景入口为 `scripts/run_current_tamp.py`；
参数与边界见根目录 README 和[架构报告](../docs/CURRENT_ARCHITECTURE.md)。
