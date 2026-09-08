# 公共 API 示例

安装仓库后，这些示例可从仓库外执行；正式运行用 scene-eval，不由示例组装工厂。

| 文件（online_manipulation/） | 用法 |
|---|---|
| public_api_client.py | 最小构造/reset/step，只导入公共API |
| pick_lift.py | 固定 PickLift 复现：复用短配置与 Runner，保存 JSON/CSV/最终 DMD，可选 Meshcat/HTML |
| external_evaluator.py | 可复制到仓库外的Policy/evaluator factory与独立reset状态 |
| example_policies.py | Hold、具名关节增量、自定义Policy |
| camera_public_api_client.py | 图像读取、RGB/米制depth保存与同帧元数据 |
| mobile_public_api_client.py | 双臂/夹爪/底盘组合动作、三相机与规划同步 |
| cartesian_client.py + cartesian_policy.py | 世界系delta/abs与持续发送绝对目标 |
| navigate_demo.py | 公共Navigator的实际观测循环与静态避障 |
| behavior_tree_tick.py | 一个BT叶节点的一tick一步，不是完整BT |
| tamp_execution.py | 调用公共execute_validated_joint_goal，不私有修改Context |

完整命令和参数见[Quickstart](../docs/QUICKSTART_ONLINE_ENV.md)。
完整导航停车双臂基准用experiments/navigation_*.json；PickLift用experiments/picklift.json。
不再为每个策略复制minimal_setup或run_online。

## 固定 PickLift 的易读脚本

`pick_lift.py` 取代旧 `pick_lift_demo/` 的重复组装示例，不是另一套 Runtime。
按 Quickstart 完成安装与模型准备后，设置 `REPO_ROOT` 为本 checkout，
`SCENE_ROOT` 为已验收场景的 `scene_000` 目录（不是 `combined_house`）。
它依赖完整的外部场景资产，不会自动下载；不支持任意场景直接套用专家抓取。

```bash
RUN_DIR="$(mktemp -d /tmp/pick-lift-XXXXXX)"
"$REPO_ROOT/.venv/bin/python" -B "$REPO_ROOT/examples/online_manipulation/pick_lift.py" --repository-root "$REPO_ROOT" --scene-root "$SCENE_ROOT" --run-dir "$RUN_DIR" --meshcat
```

不需要激活 `.venv`，可从仓库外执行。去掉 `--meshcat` 即为 headless；
加上它同时保存 HTML。终端打印 Meshcat URL、结果及输出位置。
成功时打印 `PickLift SUCCESS`，失败返回非零退出码并保留诊断。
`output/` 保存 resolved_config.json、benchmark_summary.json、benchmark_episodes.csv；
`output/episode_000_seed_500/` 保存 summary.json、trace.csv、final.dmd.yaml，
启用可视化时还有 simulation.html。缓存位于 `cache/`，最终 DMD 可能引用它。
固定基准为一次 seed=500、最多1200个策略步，不代表随机场景鲁棒性。
