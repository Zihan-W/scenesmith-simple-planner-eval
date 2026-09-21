# SceneSmith Online Eval

消费已有 SceneSmith 场景的机器人仿真与任务评测仓库。
[SceneSmith](https://github.com/nepfaff/scenesmith) 负责生成场景；本项目加载资产、
装配机器人、规划任务并在 Drake 仿真中执行。仿真本身不需要模型 API；
下面的在线 VLM/LLM 规划命令需要已配置的模型服务。

## 开始之前

安装与模型准备见[中文 Quickstart](docs/QUICKSTART_ONLINE_ENV.md)。
以下命令针对当前开发机，均在独立仿真中工作，不连接真实机器人。

按当前仓库管理约定，`runs/`、`tests/`、`AGENTS.md` 和 `pyproject.toml`
仅保留在开发机，不随 Git 分发。以下命令使用已有 `.venv` 和 Python 模块入口。
新克隆缺少打包元数据，不能直接执行旧 Quickstart 中的 `pip install -e .`；
恢复本地 `pyproject.toml` 后才能使用该安装方式。

```bash
source ~/.bashrc
cd /root/workspace/scenesmith-simple-planner-eval
export REPO_ROOT="$PWD"
export SCENE_ROOT="/root/scenesmith-simple-planner-eval/scene/scene_000"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export RUN_ROOT="$(mktemp -d "$REPO_ROOT/runs/manual-XXXXXX")"
test -x "$REPO_ROOT/.venv/bin/python"
test -d "$SCENE_ROOT"
: "${OPENAI_BASE_URL:?请在 bashrc 中配置模型服务的 /v1 地址}"
: "${OPENAI_API_KEY:?请在 bashrc 中配置模型服务密钥}"
```

`SCENE_ROOT` 是外部 SceneSmith 场景目录，不是仓库中的模型目录。
每次新任务重新创建 `RUN_ROOT`；以下命令不会覆盖已有结果。
不要把密钥写入请求 JSON 或 Git。

## 方案 1：VLM → BT

### 直接开始生成任务树

```bash
"$REPO_ROOT/.venv/bin/python" -m planner.src.bt \
  --request "$REPO_ROOT/experiments/inputs/bt_picklift_first_person/generation_request.json" \
  --output-dir "$RUN_ROOT/bt"
```

这条命令调用真实模型，生成并严格校验 BT，**不启动物理执行**。
BT 生成已封装为独立模块；输入字段、Python API、输出文件与执行接口见
[Planner 模块说明](planner/README.md)。
它使用仓库保留的固定 PickLift 示例请求：环境快照、任务计划、头部/左腕图片，
以及模型 `gpt-4.1-mini-2025-04-14`。图片和环境是请求中绑定的既有快照，
不是本次 `+100 mm` 移动物体后新采集的观测；不能用它证明当前移动场景已完成规划或抓取。

直接生成器的等价接口是：

```bash
"$REPO_ROOT/.venv/bin/python" -m planner.src.bt.generation \
  --request "$REPO_ROOT/experiments/inputs/bt_picklift_first_person/generation_request.json" \
  --output-dir "$RUN_ROOT/bt-direct"
```

主要输出：`generated_plan.json`（模型响应、调用记录和编译结果）、
`generated_bt.json`、`generated_bt.mdsl`、`generated_bt.mmd`、`generated_bt.html`。
最后一个是树结构查看器，不是仿真回放。

### 如何换任务

输入接口是 `--request <request.json>`，不是单独的 `--task` 字符串。
请求包含 `environment`、`task.description`、`task.plan_path`、`observations` 和 `model`。
环境、计划和图片均通过相对路径与 SHA-256 绑定；改变计划/图片后必须更新对应哈希。
模型必须保留输入计划的技能顺序和参数，不能自由新增技能。

当前生成器只支持导航和固定 PickLift 两类已实现的 schema。
本次请求属于固定 PickLift（`Wait → ExecutePickLift`），没有 CCSP 自动选站位。
统一入口的 `--execute` 可在生成后调用共享执行器，且必须同时提供
`--experiment`、`--repository-root`、`--scene-root`；执行配置必须与请求场景相符。
不要把固定模型的图片请求直接当成移动机器人当前场景的输入。

## 方案 2：VLM → TAMP

### 当前 +100 mm 场景：从头进行在线规划和仿真执行

下面复用本次成功运行的脚本。它读取实际初始物体变换，**仅在初始化时**沿世界
`+X` 移动 `0.100 m`，不移动机器人初始站位，不随机化场景；随后真实调用模型、
进行 CCSP 求解并执行导航/抓取。

```bash
"$REPO_ROOT/.venv/bin/python" \
  "$REPO_ROOT/scripts/run_current_tamp.py" \
  --repository-root "$REPO_ROOT" \
  --scene-root "$SCENE_ROOT" \
  --experiment "$REPO_ROOT/experiments/navigation_picklift_tamp.json" \
  --output-root "$RUN_ROOT/tamp-current" \
  --shift-world-x-m 0.10 \
  --max-samples 16 \
  --repeats 0
```

这是有界实验入口，任务固定为 `Pick up the red object and hold it.`。
不固定 seed；`--repeats 0` 关闭额外的符号反事实实验，仍完整执行在线任务。
16 是每次 CCSP 求解的完整赋值预算，不是整轮 IK 调用数。
该脚本不修改生产配置，使用的恢复上限会写入 `audit_config.json`。
每轮仍重新选站位、重新求 grasp，不复用上次成功参数。

输出位于 `$RUN_ROOT/tamp-current/`：

- `live_result.json`：完整任务结果、真实接触/抬升/保持指标。
- `raw_model_io.jsonl`：实际模型请求、响应、用量和调用耗时，不含认证头。
- `runtime/tamp_trace.jsonl`、`runtime/skill_steps.jsonl`：规划与执行过程。
- `simulation_01_success.html` 或 `simulation_01_failure.html`：本轮仿真回放。

### 通用任务接口：正式 CLI

如果要传入任务文本、选择规划器或使用其他实验配置，使用 `planner.src.tamp.cli`。
先生成本轮独立的 16 次 CCSP 配置，避免触发求解器默认的 250 次预算：

```bash
"$REPO_ROOT/.venv/bin/python" - <<'PY'
import json
import os
from pathlib import Path

repo = Path(os.environ["REPO_ROOT"])
out = Path(os.environ["RUN_ROOT"]) / "planner_config.json"
settings = json.loads((repo / "experiments/tamp_hierarchical_config.json").read_text())
settings["proc3s_ccsp"] = {"max_samples": 16}
with out.open("x") as f:
    json.dump(settings, f, indent=2)
    f.write("\n")
PY

# 每次生成新的算法随机种子，不使用固定 500–503 编号；场景是否随机由实验配置决定。
TAMP_SEED="$("$REPO_ROOT/.venv/bin/python" -c 'import secrets; print(secrets.randbits(32))')"
"$REPO_ROOT/.venv/bin/python" -m planner.src.tamp.cli \
  --planner tamp \
  --tamp-mode hierarchical \
  --skill-planner proc3s \
  --geometry-backend proc3s \
  --experiment "$REPO_ROOT/experiments/navigation_picklift_tamp.json" \
  --repository-root "$REPO_ROOT" \
  --scene-root "$SCENE_ROOT" \
  --config "$RUN_ROOT/planner_config.json" \
  --task "Pick up the red object and hold it." \
  --seed "$TAMP_SEED" \
  --record-html \
  --output-root "$RUN_ROOT/tamp-cli"
```

**接口边界：**分层 TAMP CLI 会自动执行仿真，不需要 `--execute`，
目前没有该路径的纯规划开关。省略 `--seed` 会默认使用 500，
省略两个 planner/backend 参数会选择 STRIPS + SamplingSolver。
该 CLI 不自动添加 `+100 mm` 物体平移，因此其默认实验初态与上面的当前场景复现不同；
要运行本次场景，请使用上一条命令，不能将两者视为同条件复测。
当前任务目标仍固定为已注册目标的 `holding`，不是任意自然语言任务执行器。

正式 CLI 输出 `result.json`、`tamp_trace.jsonl`、`skill_steps.jsonl`、
`skills/`、`observations/` 和 `simulation.html`。后者的名称不表示成功；
必须检查 `result.json`。`--recorded-*` 参数属于离线对照，不是真实模型调用。

## 两条路线的关系与当前验收

| 项目 | VLM → BT | VLM → TAMP（PRoC3S） |
|---|---|---|
| 任务输入 | 哈希绑定的环境、图片和具体任务计划 | 任务文本、实际观测、技能注册表 |
| 模型输出 | 严格匹配任务计划的 BT 序列 | 语义子目标、含开放连续参数的技能程序 |
| 几何参数 | 已有计划/专家输入；无 CCSP 选站位 | 完整赋值采样、IK、几何与关节边检查 |
| 调度 | 编译树后 tick | 每次执行一个技能，重新观测、求解并验证 |
| 共同执行基础 | `JsonBtPolicy`、RobotCommand、Drake Runtime | 同一共享入口，抓取可使用 CCSP 关节轨迹 |

详细调用链、检查边界及已知差异见[当前项目架构报告](docs/CURRENT_ARCHITECTURE.md)。

保留的本次成功结果为
开发机本地 `runs/object-plusx100mm-online-unseeded-20260921/run_001/live_result.json`：
`task_goal_verified`，实际抬升约 **86.01 mm**、保持 **3.1 s**、双侧接触、脱离支撑，
满足原速度与接触判据。1 次语义模型调用、2 次技能模型调用，整轮墙钟约 412.69 s。
这是当前场景的一次成功，不是新运行必然成功或多场景可靠性的证明。
运行结果、观测、日志和 HTML 回放均保留在开发机；整个 `runs/` 不纳入 Git。

## 分层目录与迁移后的入口

```text
planner/                     # 原 examples 中的上层规划实现
  src/bt/                    # BT 生成、编译、可视化与执行
  src/tamp/                  # 语义规划、技能程序、CCSP 和闭环调度
  src/skills/                # BT/TAMP 共用的 PickLift 轨迹技能
  resources/                 # 提示词与 HTML 模板
  examples/                  # BT tick 演示
simulation/                  # 原 src 中的仿真实现
  src/core/                  # 公共数据、动作、观测与协议
  src/runtime/               # 组装、环境、状态、执行与 Runner
  src/control/               # 控制、导航、专家策略
  src/geometry/              # IK、碰撞、接触与几何查询
  src/scene/                 # 场景资产输入和输出
  src/sensors/               # 相机和同步
  src/robots/                # 机器人适配器与配置
  src/tasks/                 # 任务与成功判据
  src/io/                    # 实验配置 CLI、记录工具
  src/recipes/               # 装配配方
  examples/                  # 公共 API、相机、导航、抓取演示
```

模块入口为 `python -m planner.src.bt`、`python -m planner.src.tamp.cli` 和
`python -m simulation.src`。旧 `examples.*` / `src.online_manipulation.*`
导入路径已迁移，不再保留同名重复实现。外部调用方需要同步新路径。
配置文件中的 `module:function` 入口和本地安装的 console scripts 均随本次迁移更新。
运行时和模型/物理配置不变。子目录说明见 [Planner](planner/README.md) 与
[Simulation](simulation/README.md)。

## 目录与维护

| 目录 | 职责 |
|---|---|
| `simulation/` | 按 core/runtime/control/geometry 等职责分组的仿真实现和客户端 |
| `planner/` | BT、TAMP、共享技能实现及提示词/页面资源 |
| `experiments/` | 实验配置与运行所需的小型专家输入 |
| `scripts/` | 场景/模型工具、输入提取、诊断与验证入口 |
| `models/` | 机器人模型、目标资产及上游子模块 |
| `runs/` | 所有运行产物，仅本地保留；整个目录忽略且不跟踪 |
| `docs/` | 当前使用、接口、架构与模型说明；旧实验报告已清理 |

公共接口见[API 与配置契约](docs/GENERIC_ONLINE_EXAMPLE.md)。
历史维护记录可用 `git show 708edf0:docs/EVAL_STRUCTURE_PROGRESS.md` 查看。
旧发布说明只代表当时版本。生产模型、控制律、任务阈值未因本次文档更新改变。