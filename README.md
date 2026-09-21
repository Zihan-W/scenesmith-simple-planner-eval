# SceneSmith Online Eval

消费已有 SceneSmith 场景的机器人仿真与任务评测仓库。
[SceneSmith](https://github.com/nepfaff/scenesmith) 负责生成场景；
这里负责只读加载资产、装配机器人、在线控制、策略执行和任务评估，
不需要安装场景生成服务或提供 API key。

## 从哪里开始

先读[中文 Quickstart](docs/QUICKSTART_ONLINE_ENV.md)：安装、模型准备、
唯一配置CLI、公共Python API、相机、导航、固定PickLift和可选IIWA。
当前整理是本地提交，尚未推送或发布新tag；旧v0.3 tag不包含这些路径迁移。

已有模型与.venv时，在仓库根运行：

```bash
export REPO_ROOT="$(pwd)"
export RUN_ROOT="$(mktemp -d /tmp/scene-eval-XXXXXX)"
"$REPO_ROOT/.venv/bin/python" -m pip install -e "$REPO_ROOT"
"$REPO_ROOT/.venv/bin/scene-eval" "$REPO_ROOT/experiments/minimal.json" --repository-root "$REPO_ROOT" --cache-root "$RUN_ROOT/cache" --output-root "$RUN_ROOT/output"
```

最小例子为NullTask+Hold，10步后max_steps是预期结果。首次使用先按Quickstart
初始化Zerith submodule、安装model-tools并生成OBJ，不依赖开发者历史output。

## 保留的能力

- 只读场景依赖检查、迁根、显式覆盖、派生metadata和独立缓存。
- RobotAdapter、TCP、夹爪、机载RGB/depth/label相机与同帧时间戳。
- reset/step；单/双臂关节和Cartesian控制；双夹爪组合动作、碰撞/携物检查。
- 平面运动学与轮驱动力学底盘；静态pose导航、取消、停车和控制权交接。
- 独立Policy/Task/evaluator、显式可信factory；当前规划快照与TAMP执行衔接。
- episode记录、最终DMD写回，以及固定Zerith真实接触PickLift基准。
- 可选原IIWA离线pick-place研究基线，非在线环境必选依赖。

通用接口不代表任意URDF/策略零适配。未验证双臂协同抓物、移动携物、
完整TAMP、SLAM、PLACE或导轨动力学；固定PickLift不证明随机鲁棒性。

## BT 与分层 TAMP 对照

BT baseline 仍由 `bt_generation.py` 生成、由 `bt_runtime.py` 的 `JsonBtPolicy` 执行。通用入口 `examples/online_manipulation/tamp_cli.py` 的 `--planner bt` 走原 BT 生成器；`--planner tamp --tamp-mode legacy-vlm-domain` 保留模型给数值采样域的消融路线；`--planner tamp --tamp-mode hierarchical --geometry-backend sampling` 走新分层路线。两条执行路线调用同一 `JsonBtPolicy` 导航和抓取叶节点，分层 TAMP 每次只执行一个技能并重新观测。

论文映射：`tamp_semantic.py` 提供 VLM-TAMP 风格的语义子目标；`tamp_skill_planning.py` 的 `StripsProgramGenerator` 调用 `tamp_hierarchy.py` 的确定性 BFS，是本仓库自研 STRIPS baseline，不是 PRoC3S。`tamp_geometry.py` 的候选生成、IK/碰撞过滤与排序是自研 SamplingSolver，不是 cuTAMP。外部结果校验器已明确命名为 `ExternalGeometrySolverAdapter`，其本身不包含 GPU 优化。当前技能域只有 `NavigateToPick` 和 `PickLift`。新增 `--skill-planner proc3s --geometry-backend proc3s` 接入真实 LLM 开放程序与完整赋值拒绝采样，但尚未完成端到端验收。官方 cuTAMP 独立 demo 已运行，Zerith 适配器仍未实现。官方源码逐项差距见 [论文对齐报告](docs/PAPER_ALIGNMENT.md)，当前证据与未完成项见 [进展报告](docs/PAPER_ALIGNMENT_PROGRESS.md)。

已验证真实在线仿真 `runs/tamp-hierarchical-20260919/live_010/result.json`：seed 500，自动导航/抓取参数，实际抬高 8.013 cm 并稳定保持 3.1 s，最终 `task_goal_verified`。这是单场景验证，不是多场景成功率；完整文件职责、A–E 测试、边界与后续工作见 [A–G 实现报告](docs/TAMP_IMPLEMENTATION_REPORT.md) 和 [0–32 需求审计](docs/TAMP_REQUIREMENT_AUDIT.md)。

## 目录

| 目录 | 职责 |
|---|---|
| src/online_manipulation | 唯一Runtime、动作/观测、控制/传感器、Adapter、规划、任务/策略、组装 |
| experiments | 短实验配置、有限profiles和已验收的小型专家输入 |
| examples/online_manipulation | 公共API的独立使用方式，无正式实现依赖 |
| scripts | 模型转换及两个模型/场景查看工具 |
| tools | 必要相机标定、碰撞检查、独立审计及可选IIWA基线 |
| models | 本地模型/小场景；OBJ可重建；上游submodule及现有示例资产不删除 |
| output | 运行产物，不作为正式运行必需输入 |
| docs | 用户契约、几何依据和一份维护记录 |

扩展从[API与配置契约](docs/GENERIC_ONLINE_EXAMPLE.md)开始，
工具用途见[tools](tools/README.md)，示例索引见[examples](examples/README.md)。
实际测试、已知限制和版本历史见[维护记录](docs/EVAL_STRUCTURE_PROGRESS.md)；
旧发布说明保留为历史，不当成当前实测。
