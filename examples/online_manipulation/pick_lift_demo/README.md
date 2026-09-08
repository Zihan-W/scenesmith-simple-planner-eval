# 一个 PickLift 环境，三个独立策略

环境配置只在 `minimal_setup.py`；脚本专家只在 `policy.py`。两者通过统一
`examples.online_manipulation.run_online` 运行，不再复制执行器。

## 1. 所需文件

环境所需：

- 完整 SceneSmith `scene_000` 目录及其 `package.xml`；
- 已生成的 `zerith_pick_eval.dmd.yaml`；
- 仓库的 Zerith Drake 模型和 `models/zerith_pick_eval/environment.json`。

只有脚本专家额外需要：

- `pick_home.json`、`pregrasp_ik.json`；
- `models/zerith_pick_eval/pick_lift_calibration.json`。

Hold、关节增量和未来其他策略不需要这些专家文件。外部场景资产仍须存在，
这并未把 PickLift 变成只靠仓库即可重建的自包含场景。

## 2. 命令

在当前机器的 eval 仓库根目录执行；其他机器修改两个资产变量：

```bash
export REPO_ROOT="$(pwd)"
export PYTHON="$REPO_ROOT/.venv/bin/python"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export SCENE_ROOT="/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000"
export PICK_ARTIFACT_ROOT="$REPO_ROOT/output/zerith_pick_eval"
export RUN_OUTPUT="$(mktemp -d "$REPO_ROOT/output/policy-swap.XXXXXX")"
cd /tmp
```

同一个环境先运行 Hold：

```bash
"$PYTHON" -B -m examples.online_manipulation.run_online \
  --env-factory examples.online_manipulation.pick_lift_demo.minimal_setup:make_env_config \
  --policy-factory examples.online_manipulation.example_policies:make_hold_policy \
  --output-root "$RUN_OUTPUT/hold" --seeds 500 --max-steps 10
```

同一个环境运行一次小关节增量：

```bash
"$PYTHON" -B -m examples.online_manipulation.run_online \
  --env-factory examples.online_manipulation.pick_lift_demo.minimal_setup:make_env_config \
  --policy-factory examples.online_manipulation.example_policies:make_joint_step_policy \
  --output-root "$RUN_OUTPUT/joint-step" --seeds 500 --max-steps 10
```

运行脚本抓取并录制。这里仅额外开启环境可视化，不改变物理初态和任务：

```bash
"$PYTHON" -B -u -m examples.online_manipulation.run_online \
  --env-factory examples.online_manipulation.pick_lift_demo.minimal_setup:make_visual_env_config \
  --policy-factory examples.online_manipulation.pick_lift_demo.policy:make_policy \
  --output-root "$RUN_OUTPUT/picklift" --seeds 500 --max-steps 1200 \
  --record-html --write-final-dmd
```

不开可视化时，三个命令的 `--env-factory` 完全相同；换 Policy 不会换 Task，
Hold 和关节增量仍接受 PickLift 的评分，只是在 10 步预算结束时尚未完成任务。

## 3. 自己写循环

设置好上面的变量后，这段 Python 可直接运行：

```python
from examples.online_manipulation.pick_lift_demo.minimal_setup import make_env_config
from examples.online_manipulation.example_policies import MyPolicy
from src.online_manipulation import make_env

env = make_env(make_env_config())
policy = MyPolicy()
obs, info = env.reset(seed=500)
policy.reset(obs, info)
for _ in range(10):
    obs, reward, terminated, truncated, info = env.step(policy.act(obs))
    if terminated or truncated:
        break
print(env.finalize_episode())
```

需要完整记录时使用公共 runner，不要让策略自己保存环境状态或调用 reset。

## 4. 行为与验证范围

专家继续使用原来的在线 PickLiftPolicy：PREGRASP → APPROACH → CLOSE →
VERIFY → LIFT → HOLD。任务要求真实双指接触、脱离桌面、抬升至少 8 cm 并
稳定保持 3 秒；没有 weld、attach 或全局摩擦倍增。

当前工作树已统一 Runner 的 `StoppablePolicy.stop_reason` 与 Task 评价，
并验证单物体具名携带关系在旧动作和 RobotCommand 的 joint/Cartesian
分支中一致。详情见[通用接口契约](../../../docs/GENERIC_ONLINE_EXAMPLE.md)。
没有实现 RL wrapper，不把几何测试或关节增量冒烟测试当作新的真实抓取验证。

所有生成结果仍在 Git 忽略的 output 或临时目录中；没有创建新 tag。

历史分离工厂工作树 seed 500 输出：`success=True reason=lift_held steps=277`。
红盒抬升 10.29 cm、保持 3.1 秒。结果目录为
`output/decoupled_picklift_seed500/episode_000_seed_500/`。
另外验证了同一环境实例切换 Hold/关节增量并 reset；构造环境时不需要专家文件。

当前收尾回归见 `output/closure_audit/fixed_picklift`：旧CLI经相同工厂/runner，
seed500仍277步、`lift_held`。两次均为固定场景，不是随机鲁棒性验证。
