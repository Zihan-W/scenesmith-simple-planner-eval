# PickLift 示例：正式组装与专家输入

当前工作树的日常入口是 `python -m src.online_manipulation`，
完整、实际验证过的命令见[中文 Quickstart 第0、4节](../../../docs/QUICKSTART_ONLINE_ENV.md)。
这次结构迁移尚未发布，不在旧 `online-env-v0.3` tag 中。

## 文件职责

- `experiments/picklift.json`：短实验配置，引用有限的机器人、控制、初态、场景、Task、Policy、evaluator profiles。
- `experiments/profiles.json`：完整配置来源，不要求用户每次复制全部参数。
- `experiments/inputs/pick_lift/{pick_home.json,pregrasp_ik.json,pick_lift_calibration.json}`：已验收专家输入；只有专家 Policy 构造需要读取。
- `experiments/inputs/pick_lift/scene_overrides.yaml`：显式目标替换与物体初态，不在启动时重新标定或移动目标。
- 完整外部 `SCENE_ROOT`：原始 SceneSmith 场景，只读。
- 用户指定的新 cache 根：依赖闭包、重定位后的 DMD、与替换物体一致的 metadata、源哈希。
- 用户指定的 output 根：HTML、JSON、CSV、最终 DMD，不是下一次运行的必要输入。

固定专家明确要求原 `furniture_welded` 基准及匹配的源场景/模型指纹。
普通实验可以选 `free`，但不能冒充固定专家基准。

## 换策略而不换环境

复制 `experiments/picklift.json` 为自己的小配置：

- 保持原 `robot/control/initial_state/scene/task`，将 `policy` 改成 `hold`：不读专家 IK。
- 用自己的 `my_module:make_policy`：明确加 `--trust-factories`，实现公共 Policy 协议。
- 把 `task` 改为 `null` 时不需要复制机器人参数。
- 使用仓库外 `my_module:make_evaluator` 可替换评分，不修改 Task 的接触许可或 Runtime。

工厂是可信本地 Python 代码，不是可执行任意第三方代码的安全沙箱。
接口、生命周期与完整仓库外示例见[通用文档第9节](../../../docs/GENERIC_ONLINE_EXAMPLE.md)。

## 旧示例文件为什么还在

本目录的 `minimal_setup.py`、`policy.py` 只转发正式
`src/online_manipulation/recipes/pick_environment.py` 和 `pick_policy.py`；
`examples/online_manipulation/run_online.py` 转发正式 assembly 和既有 runner。
正式模块不反向依赖这些示例。

旧环境工厂仍接受显式 `SCENE_ROOT/PICK_ARTIFACT_ROOT`，并按旧约定找
`zerith_pick_eval.dmd.yaml`；它不会猜测新 cache 的路径，也不会自动回读旧 output。
迁移后的日常用法选择短实验入口，不再照抄旧的历史输出路径。

## 验证边界

固定专家仍在线执行 PREGRASP → APPROACH → CLOSE → VERIFY → LIFT → HOLD，
10 Hz策略、200 Hz伺服、1000 Hz物理。成功需要真实双指接触、脱离桌面、
至少8 cm抬升并保持3秒。不使用 weld/attach/瞬移/全局摩擦倍增。

本轮独立缓存运行 seed500：277步、27.7秒、抬升10.29cm、保持3.1秒，
`lift_held`。这是固定基准，不是随机鲁棒性或双臂共同抓物验收。
完整记录及最终 DMD 格式衔接修复见[结构迁移进度](../../../docs/EVAL_STRUCTURE_PROGRESS.md)。
历史 `output/decoupled_picklift_seed500` 和 `output/closure_audit/fixed_picklift`
只保留其对应旧代码状态的证据，不作为当前运行输入。
