# Simulation：机器人仿真与公共接口

本目录由原 `src/` 实现及纯仿真示例整理而来。公共 Python API 为
`simulation.src`；它不导入 `planner`，不自动调用模型服务。

```text
simulation/
├── src/
│   ├── core/       # 动作、观测、规格、协议和底盘配置
│   ├── runtime/    # 环境、状态模型、组装、执行与 Episode Runner
│   ├── control/    # 控制器、导航器、专家策略
│   ├── geometry/   # IK、碰撞/接触、导航几何及 Drake 几何工具
│   ├── scene/      # 场景输入、依赖解析与最终 DMD 写回
│   ├── sensors/    # 相机与时间同步
│   ├── robots/     # 机器人适配器及 Zerith 配置
│   ├── tasks/      # 任务定义、成功判定与评估
│   ├── io/         # 实验配置 CLI 与记录覆盖层
│   └── recipes/    # 既有场景/机器人装配配方
└── examples/       # 相机、Cartesian、导航、PickLift 与公共 API 客户端
```

## 公共接口

```python
from simulation.src import load_experiment, make_env, HoldAction

experiment = load_experiment(
    config_path,
    repository_root=repository_root,
    cache_root=cache_root,
)
env = make_env(experiment.environment_config)
observation, info = env.reset()
observation, reward, terminated, truncated, info = env.step(HoldAction())
```

配置入口：`python -m simulation.src`，等价于本地重新安装后的 `scene-eval`。

```bash
RUN_ROOT="$(mktemp -d /tmp/scene-eval-XXXXXX)"
"$REPO_ROOT/.venv/bin/python" -m simulation.src \
  "$REPO_ROOT/experiments/minimal.json" \
  --repository-root "$REPO_ROOT" \
  --cache-root "$RUN_ROOT/cache" \
  --output-root "$RUN_ROOT/output"
```

最小例子为 Hold/NullTask，执行 10 步后达到步数上限是预期结果。
完整规划任务请使用[根目录 README](../README.md) 中的 `planner` 入口。

## 示例与依赖

`examples/` 包含 `public_api_client.py`、`camera_public_api_client.py`、
`mobile_public_api_client.py`、`cartesian_client.py`、`navigate_demo.py` 和 `pick_lift.py` 等。
它们使用同一公共 API，不另建 Runtime；可用对应脚本的 `--help` 查看参数。

外部 SceneSmith 场景、机器人模型和实验配置仍放在原来的资产/配置位置。
本次只改模块归属和入口路径，没有改变坐标、质量、摩擦、控制参数或成功判据。
本地 `pyproject.toml` 已同步新包路径，但按用户要求不随 Git 分发。
