# 仿真评测 Quickstart

本文针对当前本地提交后的代码，不是旧 online-env-v0.3 tag 的命令手册。
本轮未推送、未打新 tag；同事需要取得包含本轮提交的 checkout 后再执行。
旧发布的文档可用 `git show online-env-v0.3:docs/QUICKSTART_ONLINE_ENV.md` 查看。

## 1. 安装与模型准备

支持并验证的运行栈为 Linux x86_64、Python 3.11、Drake 1.49。
仿真控制主要使用 CPU；相机用渲染器，不要求 A100 或 SceneSmith 生成服务。
没有验证 Windows/macOS 或任意 GPU/驱动组合。

在当前 checkout 根目录执行（已有 .venv 可直接复用，不必重建）：

```bash
export REPO_ROOT="$(pwd)"
git submodule update --init --recursive
python3.11 -m venv "$REPO_ROOT/.venv"
export PYTHON="$REPO_ROOT/.venv/bin/python"
"$PYTHON" -m pip install --index-url https://pypi.org/simple -e "$REPO_ROOT[model-tools]"
"$PYTHON" -B "$REPO_ROOT/scripts/convert_zerith_for_drake.py"
"$PYTHON" -B "$REPO_ROOT/scripts/convert_zerith_for_drake.py" --check
```

上游 Zerith STL 由 submodule 提供，转换器生成本地 OBJ 与派生模型；
OBJ 不进 Git。已有模型时先执行 --check。转换失败不能靠跳过碰撞模型解决。
安装核心可用 `pip install -e "$REPO_ROOT"`；model-tools 仅模型生成需要，
iiwa 为离线基线，dev 为测试/审计。依赖以 pyproject.toml 为准，
requirements-online/model-tools.txt 是无打包安装时的版本列表，不是另一套环境。

可编辑安装保留现有 `src.online_manipulation` 公共命名，不为改包名重写项目。
Python 包不携带大型场景、OBJ 或 submodule；运行时明确传入 repository-root。
不再需要设置 PYTHONPATH，不需要激活虚拟环境；下文使用绝对可执行文件路径。

本轮实测：在已有完整 .venv 中安装 editable 包、构建wheel、pip check，
在全新临时模型根完整生成OBJ并 --check，
以及仓库外运行。**没有宣称本轮重新联网安装了一整套全新虚拟环境**。
首次无隔离构建发现缺 wheel，补齐构建依赖后安装成功；默认 pip 隔离构建会安装它。

## 2. 最快运行：空 output 与独立 cache

```bash
export RUN_ROOT="$(mktemp -d /tmp/scene-eval-XXXXXX)"
cd /tmp
"$REPO_ROOT/.venv/bin/scene-eval" "$REPO_ROOT/experiments/minimal.json" --repository-root "$REPO_ROOT" --cache-root "$RUN_ROOT/cache" --output-root "$RUN_ROOT/output"
```

这是唯一正式配置 CLI。等价的模块入口是 `"$PYTHON" -m src.online_manipulation`，
参数完全相同，不是另一套组装逻辑。

预期：`policy_steps: 10`、约1秒仿真时间、`termination_reason: "max_steps"`。
NullTask 没有成功条件，`success: false` 不代表初始化失败。
既有 minimal fixture 的 floor/box 初始重叠只适合 API smoke，
不把它作为稳定接触质量示例。未因目录整理改动物理参数。

输出根包含 resolved_config.json（完整参数、来源、hash）；
每个 episode 独立子目录包含 summary.json、trace.csv，
benchmark_summary.json 汇总。已有非空输出目录会报错，换一个新目录运行。
cache 是可重建派生资产，不是日志；final.dmd 可能继续引用它，不能提前删除。

## 3. 公共 Python API 与外部扩展

```bash
"$PYTHON" -B "$REPO_ROOT/examples/online_manipulation/public_api_client.py" --repository-root "$REPO_ROOT"
```

预期返回 `simulation_time_s: 0.1` 和动作状态。它只导入公共 API，
从任何目录执行，不读专家文件。完整自定义循环：

```python
import os
import tempfile
from pathlib import Path
from src.online_manipulation import load_experiment, make_env

root = Path(os.environ["REPO_ROOT"])
with tempfile.TemporaryDirectory() as cache:
    experiment = load_experiment(root / "experiments/minimal.json",
                                 repository_root=root, cache_root=cache)
    env = make_env(experiment.environment_config)
    obs, info = env.reset(seed=0)
    policy = experiment.policy
    policy.reset(obs, info)
    for _ in range(10):
        obs, reward, terminated, truncated, info = env.step(policy.act(obs))
        if terminated or truncated:
            break
    print(obs.time_s, info["action_decision"])
```

更换策略不改 Runtime。Policy.reset/act、Evaluator.reset/evaluate 的完整契约与
Task/机器人 factory 签名见 [API与配置契约](GENERIC_ONLINE_EXAMPLE.md)。
外部模块必须是显式信任的本地代码；`--trust-factories` 不是安全沙箱。

外部 Policy/evaluator 的可复制例子：

```bash
export CLIENT_ROOT="$(mktemp -d /tmp/eval-client-XXXXXX)"
cp "$REPO_ROOT/examples/online_manipulation/external_evaluator.py" "$CLIENT_ROOT/my_components.py"
cd "$CLIENT_ROOT"
"$PYTHON" - <<'PY'
import json
import os
from pathlib import Path
config = json.loads((Path(os.environ["REPO_ROOT"]) / "experiments/minimal.json").read_text())
config.update(policy="my_components:make_policy", evaluator="my_components:make_evaluator")
config["run"]["seeds"] = [7, 8]
Path("experiment.json").write_text(json.dumps(config))
PY
"$PYTHON" -B -m src.online_manipulation "$CLIENT_ROOT/experiment.json" --repository-root "$REPO_ROOT" --cache-root "$CLIENT_ROOT/cache" --output-root "$CLIENT_ROOT/output" --trust-factories
```

此处 -m 让 Python 按正常规则导入当前目录的 my_components；
包本身已经安装，不靠临时 sys.path 注入。安装了自己的 factory 包时可直接用 scene-eval。
每个 seed 应在3步后得到 `custom_hold_confirmed`。

## 4. 相机与组合动作

```bash
"$PYTHON" -B "$REPO_ROOT/examples/online_manipulation/camera_public_api_client.py" --repository-root "$REPO_ROOT" --output-dir "$RUN_ROOT/camera"
"$PYTHON" -B "$REPO_ROOT/examples/online_manipulation/mobile_public_api_client.py" --repo-root "$REPO_ROOT" --mode wheel_dynamic --output "$RUN_ROOT/mobile-client"
```

相机例子保存 RGB PNG、米制深度 NPY并打印 shape/dtype/时间戳。
组合例子展示一次 step 同时发送左右臂、双夹爪和底盘动作，以及当前规划状态同步。
相机安装定义属于 RobotAdapter，不随场景复制。
标定/真实内容图与几何依据见 [相机清单](ZERITH_CAMERA_INVENTORY.md)；
示例的空场景图只用于 API，不代替几何和可见性验收。
没有目标检测或视觉抓取模型。

## 5. 两种底盘与静态导航停车后双臂操作

```bash
"$REPO_ROOT/.venv/bin/scene-eval" "$REPO_ROOT/experiments/navigation_wheel_dynamic.json" --repository-root "$REPO_ROOT" --cache-root "$RUN_ROOT/nav-dynamic-cache" --output-root "$RUN_ROOT/nav-dynamic"
"$REPO_ROOT/.venv/bin/scene-eval" "$REPO_ROOT/experiments/navigation_planar_kinematic.json" --repository-root "$REPO_ROOT" --cache-root "$RUN_ROOT/nav-planar-cache" --output-root "$RUN_ROOT/nav-planar"
```

需要可视化时给同一命令加 `--meshcat --record-html`，并使用新的 output。
Meshcat URL 由终端打印；远程机器需 SSH 转发对应端口。
HTML 位于 episode 子目录的 simulation.html，可下载离线查看。

两份配置复用同一个运行链，分别选择 wheel_dynamic/planar_kinematic，
匹配的初始高度及导航收拢姿态；控制/停车阈值沿用原演示。
预期成功原因 `parked_dual_targets_held`。
这是导航后空手双臂目标，不是双臂协同抓物。

导航接受 world/map/odom 或接收时的 link pose；local目标解析后固定在世界系。
取消后仍需 step 执行减速，满足停车窗口才 cancelled；到达/取消后显式 release
再交还底盘控制权。到达要求位置3cm、yaw3°、实际速度0.01m/s、
角速度0.02rad/s同时保持0.5s，不宣称完全静止。细节见API契约和 navigate_demo.py。
运动学不等价轮地动力学，差异证据见 [底盘审计](BASE_MODE_IMPLEMENTATION_AUDIT.md)。

## 6. 固定 PickLift：明确依赖外部 SceneSmith 场景

本例不是完全自包含。需要与已验收专家匹配的完整 scene_000 目录，
包括 combined_house、package.xml、所有 SDF/网格/纹理。
不要只复制一个 DMD；缺资产会列出实际路径，不自动替换纹理。

选择源场景（下面通过交互读取真实路径，不写死开发者目录）：

```bash
read -r -p "完整 SceneSmith scene_000 路径: " SCENE_ROOT
export SCENE_ROOT
export PICK_RUN="$(mktemp -d /tmp/eval-picklift-XXXXXX)"
"$REPO_ROOT/.venv/bin/scene-eval" "$REPO_ROOT/experiments/picklift.json" --repository-root "$REPO_ROOT" --scene-root "$SCENE_ROOT" --cache-root "$PICK_RUN/cache" --output-root "$PICK_RUN/output" --write-final-dmd
```

缓存准备、显式小红盒替换、位姿覆盖和 metadata 更新由这条命令完成，
不需要从旧 output 手动复制 pick_home.json。已验收 IK与标定在
experiments/inputs/pick_lift，模型/源语义 hash 不匹配时明确拒绝。
free/furniture_welded 必须显式选择；固定基准使用 furniture_welded。
源目录只读；最终 DMD 写到新输出，源metadata不是仿真后的状态。

需要HTML同样加 `--meshcat --record-html`，使用全新 output。
成功要求真实双指接触、脱离桌面托举、抬升至少8cm、稳定保持3秒。
固定成功不代表随机扰动鲁棒性；任意新场景需重新绑定和校准自己的策略，
不会自动将本专家迁移到所有桌子。

## 7. 可选 IIWA 基线

原离线 IK → RRT → TOPPRA → 仿真只保留一个编排入口，与在线CLI分开：

```bash
"$PYTHON" -m pip install -e "$REPO_ROOT[iiwa]"
export IIWA_RUN="$(mktemp -d /tmp/iiwa-run-XXXXXX)"
"$REPO_ROOT/.venv/bin/iiwa-baseline" --repository-root "$REPO_ROOT" --scene-root "$REPO_ROOT/models/21-20-10_cleaned/scene_000" --output-root "$IIWA_RUN/episode"
```

步骤日志 stage_1.log…stage_4.log；失败停止，不自动吞掉失败重试，
不删除已有文件。不强制在线用户安装 manipulation/networkx。
保留原sticky夹爪/携物近似与研究基准的摩擦倍率10；
这些仅属于原IIWA基线，不是在线Zerith的真实接触实现。
本轮实际验证范围见维护记录；不以 --help 或资产加载冒充完整pick-place成功。

已验证四阶段启动以及真实场景/机器人加载（46模型、2742碰撞几何）；
本轮没有完整重跑IIWA离线规划和抓放。

## 8. 迁移、开发与限制

| 旧内容 | 当前去向 |
|---|---|
| run_zerith_online_example、examples/run_online、pick_lift_demo组装 | experiments配置 + scene-eval；旧CLI参数不再兼容 |
| models/zerith_pick_eval/environment.json | profiles的initial_state/control/task及独立专家输入 |
| 旧Zerith字典环境/Facade、safe-home/导轨/阶段搜索链 | 删除；历史可从检查点cb79ba8取回，不另存legacy |
| 原IIWA脚本和批处理shell | iiwa-baseline；算法仅在tools/iiwa |
| 机制审计/相机标定/碰撞代理检查 | tools/audit、calibration、validation |
| examples中的导航完整实验和运动记录器 | navigation两份配置；组合控制见mobile_public_api_client |
| TAMP执行helper | 公共execute_validated_joint_goal；例子不持有正式实现 |

单臂动作/相机公共契约不变。私有旧导入与脚本路径不兼容。
TAMP只同步并检查直接边后执行，不是完整TAMP或动态重规划。

```bash
"$PYTHON" -m pip install -e "$REPO_ROOT[dev]"
cd "$REPO_ROOT"
"$PYTHON" -B -m unittest discover -s tests -v
```

有真实A场景时设置 SCENE_ROOT，相关迁根/墙体测试会实际运行。
测试中的缺资产 skip 不算视觉验收；CI/同事无外部A资产时应分别报告。
本轮已用真实A验证，B缺纹理仍未视觉验收。

不支持任意URDF免适配、任意策略免输入输出适配、SLAM、移动携物、
双手协同抓物、PLACE、导轨动力学或新策略模型。
当前代码与实验结果见唯一[维护记录](EVAL_STRUCTURE_PROGRESS.md)。
