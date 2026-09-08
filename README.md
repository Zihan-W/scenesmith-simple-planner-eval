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
