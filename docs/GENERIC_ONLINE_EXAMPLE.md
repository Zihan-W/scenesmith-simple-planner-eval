# 公共接口、配置和执行契约

日常命令只看[Quickstart](QUICKSTART_ONLINE_ENV.md)，本页解释接口行为。
当前代码整理在本地完成，公共API版本沿用0.3，未发布新tag。
旧脚本、字典环境和环境JSON兼容层已经退役；历史源码在cb79ba8检查点。

## 实验组装与替换接口

```text
短实验 JSON → 有限 profiles + 显式 options → load_experiment
  ├─ SceneSmith 依赖适配 → 只读输入 / 派生 cache / 对象映射
  ├─ RobotAdapter + 初态 + control → 既有 RuntimeConfig / ZerithEnvironmentConfig
  ├─ 独立 Policy factory（仅专家策略读取 IK/权重）
  └─ Task（绑定/接触）+ Evaluator（决策结果）
                ↓
        make_env → reset → policy.act(obs) → step → 原 Runner 记录
```

正式工厂位于 `src/online_manipulation/recipes/`，不能反向依赖 examples/scripts。
`assembly.run` 是公共API和配置CLI共用的装配调用，`runtime.py` 仍是唯一实际积分/控制循环。
旧 CLI 及其重复配置来源已删除。普通策略不读取专家文件。

### 参数归属

| 数据 | 单一来源 / 使用位置 |
| --- | --- |
| 机器人模型、关节、执行器/TCP/夹爪/相机安装 | Adapter + 模型包；TCP 位于 src/zerith_tcp.py，不再导入盒子/桌子参数 |
| 运行初态：站位、yaw、导轨、手臂 q | profiles.initial_state |
| 时钟、动作上限、servo 设置 | profiles.control；controller.configure_joint_servos 仅允许 kp/kd/effort cap，不改位置/速度硬件限位 |
| 场景资产、显式 free/welded、观察对象 | profiles.scene + scene_options；物体替换/位姿覆盖写入 cache |
| 目标、支撑、双指绑定和 Task 条件 | profiles.task + task_options；不写入机器人定义 |
| 已验收 IK、专家相对位姿/权重 | profiles.policy；experiments/inputs/pick_lift；不属于初态构造的必要输入 |
| episode 时序、结果和记录 | 原 runner；实验根下追加 resolved_config.json，不向 trace 内联图像 |

`control_options.joint_servo_settings` 示例：
`{"left_wrist_pitch_joint":{"kp":800,"kd":60,"effort_limit":8}}`。
这是可选运行设置，不是新的硬件标定。默认仍用 src/zerith_servo_config.py 的既有
验证值；所有最终 JointSpec 数值进入 resolved_config。未知关节或超出 Adapter 上限明确报错。

### 最小公共 API

以下代码在安装包并设好 REPO_ROOT 后可从仓库外运行；不需要专家文件：

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
    experiment.policy.reset(obs, info)
    for _ in range(10):
        obs, reward, terminated, truncated, info = env.step(experiment.policy.act(obs))
        if terminated or truncated:
            break
    print(obs.time_s, info["task"])
```

换场景主要改 `scene`、`scene_options`、必要 `task_options.bindings` 和
`initial_state_options`。直接 DMD 可用 `scene_options.kind="dmd"`、`dmd`、
`package_xmls`、`observed_bodies`；路径可为显式绝对路径或仓库相对路径。
SceneSmith 的 `variant` 必填 free/furniture_welded。地面和墙若共用 body，
必须指定 `ground_geometries=[["model::body","collision_name"]]`。
独立地面的旧 `ground_body_names` 只在该 body 恰有一个 collision 时兼容；混合 body 会报错。

`prepare_scene` 也接受显式 `dmd_relative` 与 `metadata_path` 处理单房间产物。
不根据 JSON 再加房间偏移：DMD 的 base_frame 决定世界变换；派生索引记录初态而非动态真值。
源场景已有与 Adapter 同名的机器人会报错；未知名字的机器人不能靠模型名可靠识别，
用户必须选明确的 robot-free 输入或另做显式派生，不支持自动拆掉任意机器人。

通过profile启用机载相机：在实验中加
`"robot_options":{"camera_options":{"enabled_names":["head_camera","left_wrist_camera","right_wrist_camera"]}}`。
width/height/fov_y_rad/update_period_s等仍是显式仿真配置，安装外参来自同一Adapter，
不随场景复制。缺省enabled_names为空，不创建渲染系统。本轮新入口已在fixed/minimal和
wheel_dynamic/mobile双臂配置中实际reset/step并采样三帧；空场景测试仅证明接线，
真实可见性另由A场景PREGRASP验证。

### 仓库外 Policy / evaluator

把 `examples/online_manipulation/external_evaluator.py` 复制到你自己的工作目录，
命名为 `my_components.py`。在自己的短 JSON 里选择：

```json
{
  "robot": "zerith_left", "control": "smoke", "initial_state": "minimal",
  "scene": "minimal", "task": "null",
  "policy": "my_components:make_policy",
  "evaluator": "my_components:make_evaluator",
  "run": {"seeds": [7, 8], "max_steps": 10}
}
```

在该目录用已安装的 Python 执行 `-m src.online_manipulation`，加 `--trust-factories`；
具体完整命令见 Quickstart。第三方已安装模块也可直接用 scene-eval。
未显式信任时 module:function 会被拒绝；启用后相当于执行本地 Python，**不是安全沙箱**，
不能加载来源不明的配置/工厂。不需要修改中央分支或 Runtime。

公开协议：

- `Policy.reset(observation, info)` / `act(observation) -> RobotAction`。
- Policy factory 接收 FactoryContext：environment_config、robot_spec、options、repository_root；没有真实 Context/内部索引。
- `Evaluator.reset(observation, task_reset_info)` 每个 episode 调用一次，负责清空自己的状态。
- `Evaluator.evaluate(observation, baseline: TaskEvaluation) -> TaskEvaluation` 每个 step 调用一次。
- 结果包含 reward、terminated、truncated、success、reason、metrics。分类器状态归每个 evaluator 实例；不放 Runtime/global。
- 默认 `TaskResultEvaluator` 原样返回 Task 的判定，保持 PickLift 已有双指/脱桌/抬升/保持时长逻辑。
- `EvaluatedTask` 委托原 Task 的 observe/contact/finalize；外部 evaluator 只替换评估结果，不获得物理修改权限。
- evaluator factory 接收 options；Task factory 接收 `(options, robot_spec)`，返回公开 Task；Robot factory 接收 `(options, initial_state, control, base, repository_root)`，返回 RobotAdapter。
- Policy 可声明 `required_capabilities`（joints/arms/grippers/sensors 名称集合）；实验的 `requires` 合并检查。不兼容时报错，不假定任意权重/动作维度天然兼容。

VLM 离线 success classifier 仍是 episode artifacts 的外部消费者；本轮不实现 VLM/VLA、
新导航器或完整 TAMP。原 TAMP helper 继续使用 env.get_planning_query 的实时同步，
核心动作检查不受可选 planner 是否启用影响。已验证双臂基本动作/两种底盘/相机继承，
没有因此宣称双手共同搬物、任意工作空间抓取或完整 TAMP 已实现。

## 单臂与双臂动作、坐标及执行语义

- TCP 从 `RobotSpec.end_effector_frames`（具名臂）或显式单臂默认
  `end_effector_frame_name` 获取。Zerith 是校准的 grasp frame，不是机械腕
  link 原点；当前双臂相对 wrist_pitch 的平移为 `[0.176,0,-0.002] m`，
  grasp +X 为接近轴、+Y 为闭合轴。机器人安装细节只属于适配器。
- `obs.robot.end_effectors[name]` 与单臂 `end_effector_pose` 是**实际世界位姿**。
  `Pose.translation_m` 单位米，四元数 `quaternion_wxyz` 为 w,x,y,z。
- `CartesianPoseAction(frame, 'world', pose)` 的位置、姿态均相对世界。
  每个 tick 只做一次局部 IK，目标残差不在后台持续求解；要继续趋近，必须
  每次 step 重发 abs。`accepted` 不等于末端已到达或目标在全局可达域。
- `CartesianDeltaAction` 的平移用世界米制向量；旋转用世界轴角向量 rad，
  `R_goal = Exp(rotvec_world) @ R_commanded_FK`（左乘）。
  **表达系是 world，累加基准是上一指令关节目标的 FK**；它们不是同一概念。
  当前实际基座/其他状态同步进规划；夹爪几何使用实际位置，不以空闭合目标
  冒充已穿过物体的手指位置。没有增加局部 Cartesian action 坐标系。
- joint delta 累加到旧指令目标，joint abs 相对旧目标限幅；revolute 用 rad，
  prismatic 用米。仅发一次之后 Hold 只跟踪已算出的关节目标，不补发残差。
  不调用 step 则仿真暂停；不是让真实控制器继续独立运行。
- 默认一个 step=0.1仿真秒，200Hz伺服、1000Hz物理，不保证实时墙钟速度。
  控制器按受限力矩跟踪目标；没有运行中 SetPositions 瞬移机械臂。
- `GripperAction.width_m` 是双指内侧开口宽度，Zerith 最大0.07628m，
  不是单指位移。新组合动作不支持每动作 `maximum_effort_n`，明确返回
  `per_action_effort_unsupported`；力矩限制仍来自配置。
- 新组合动作一次处理 `arms/grippers/base`；任何指定分量拒绝则所有新目标
  均不提交，仍推进仿真并跟踪旧臂/夹爪目标。返回
  `info['action_decision']`（accepted/reason/components/edge）。
  省略 base 请求零速度；省略臂/夹爪保持旧目标。
- frame/组名错误、碰撞或关节约束不通过不可当作成功；查看上述 decision。
  局部 IK 不提供任意目标可达性证明。违反类型/模型契约则直接异常，不吞错。

左右臂可达目标的持续 abs 跟踪，世界 delta 平移/左旋转方向及 commanded-FK
基准已在 `tests/test_cartesian_handoff.py` 做真实执行测试；目标来自合法关节
配置的 FK，整条目标关节边已检查。只验收该小邻域最后1秒位置<1mm、姿态<1°，
不宣称任意工作空间精度。结果入口见 progress 的整体验收表。

## Task 携物与 RobotAdapter 契约

`RobotAdapter.gripper_position_targets(width_m, name=None)` 必须支持具名夹爪。
None 只表示显式 `spec.gripper`，不能随意选 `spec.grippers` 第一项；无夹爪/
未知名称应 ValueError。每个 controlled 单自由度关节必须有唯一
`<joint_name>_actuator` 驱动它，不能映射别的关节。`model.populate_model`
在 Finalize 前验证，规划与仿真共用。额外轮执行器由底盘适配器负责。
真实两轴无夹爪 fixture 和错误映射适配器的构建测试不等于第二个完整机器人项目。

PickLift 的单物体携带关系属于 Task：双指实际接触后绑定 carrier arm、gripper
和末端 frame。多臂模型必须显式给 `carrier_arm_name`；可配置
`carrier_gripper_name` 并核对 contact bodies，身份不一致直接拒绝。
旧单臂动作、CompositeAction 和具名 RobotCommand 的 joint/Cartesian
动作使用相同 CarriedBody 与允许接触。携带物体也检查与其他场景几何的边碰撞。

抓住后的进一步受限闭合是夹持力目标，不是穿过物体的几何位置目标：仅对
Task 已证实携带、指—目标关系吻合且继续闭合的手指，几何边使用实际手指位置；
放开和机械臂动作仍检查候选几何。真实控制目标/力矩上限、物理状态、碰撞过滤
不因此改变。v0.3 在**携物状态**增加共同边检查；本轮 M0—M4 进一步让固定单臂
空手关节动作也执行候选边检查。关闭可选规划查询不再绕过核心碰撞检查。
这可能拒绝过去漏检而接受的关节动作，是明确的安全修正，不改动作坐标/累加语义。

不支持双手共同搬一个物体，也不以本单物体 Task 支持移动中搬运；携物时非零
base 命令明确报错。新接触等价测试是几何/解析验证，物理基准仍为固定单臂
PickLift。新的任意关节抓取策略、移动携物及双臂实际抓取未验收。

TAMP 的 `execute_validated_joint_goal(env=..., goal_positions=...)` 自动调用
`env.get_planning_query()` 并读取当前 observation，不信任旧 query/obs 参数；
当前底座、关节及全部自由物体在执行前同步。它发送计算后的绝对 goal，而非
写死0.001。移动后同步已有针对性实际运行证据。它是静态直接边执行示例，
不是动态重规划或完整 TAMP；单独持有 `build_planning_query()` 不会自动刷新。

## 相机 hardening 契约（从已删除文档迁入）

- RobotSpec.cameras 定义安装与内参，ScenarioSpec.renderer 定义渲染器。
  `X_parent_camera_optical = X_parent_camera_mount @ X_mount_camera_optical`。
  optical +X右、+Y下、+Z前；安装依据见
  [Zerith 相机清单](ZERITH_CAMERA_INVENTORY.md)。没有真实硬件内参，当前为
  simulation camera intrinsics，不宣称硬件一致。
- `obs.sensors[name]`：RGB H×W×3 uint8；depth H×W float32，米制Z-forward
  深度（不是径向距离；超近/超远按 Drake 用0/inf等标记）；label H×W int16。
  label_names 提供物体对应。RGB/depth/label、timestamp及光学pose同一采样帧。
- 相机周期独立于policy。采样事件之间保持整帧，包括旧pose；reset确定返回
  t=0帧。不能用当前robot FK代替旧帧pose，也不能floor当前时间推算timestamp。
- 优先读取 `RgbdSensorDiscrete.image_time_output_port()`；仅Drake1.49.0
  的已知端口导出缺陷使用 `_drake_camera_time.sampled_image_time` 内封装的
  唯一一维 held-time ZOH。其他版本端口异常、来源歧义或类型异常明确报错。
- `Observation.sensors`、相机label_names为只读MappingProxy，图像复制且只读。
  `objects/task` 是复制的普通dict，**不承诺深层不可变**；勿从 frozen dataclass
  推导所有嵌套内容不可变。
- 两级 `as_dict()` 默认只导出图像元数据，不内联像素list；显式
  `include_images=True` 才导出像素。benchmark默认不内联图像；常规图片另存
  PNG/NPY。所有相机关闭不创建renderer/sensor，无渲染开销。
- hardening/慢相机移动腕部、版本fallback测试仍在测试集；移动底座继承同一
  CameraSystems。采样事件更新前图像与物理更新后同时间robot.q可差一个物理步，
  不能拿这个差异伪称外参错误或用最新pose覆盖缓存。

## 版本化历史，不当成当前实测

| 版本/阶段 | 当时验收范围 | 当前应如何使用 |
| --- | --- | --- |
| v0.1，25eb9a4 | 固定底座、导轨0.4m、单臂真实PickLift；固定3/3、扰动2/3，seed402是策略鲁棒性失败 | 历史记录，不能推导当前随机成功率；当时未验证第二真实机器人、PLACE、移动/导轨动力学 |
| v0.2代码，23d656c | 可迁移相机、三相机几何/可见性人工验收及hardening | 后续相机改造应保留接口/采样契约，不代表完成视觉策略 |
| v0.2本地/远端tag，682ba7c | 上述代码加中文全新clone Quickstart；23d656c之后两次仅文档提交 | 只读核验记录见progress；本轮不改tag |
| b433ee6（v0.2之后的模型修复） | 修复指部contact几何、开口标定0.07628m、转换器v5 | v0.3继承该修复，首次运行需要生成本版本OBJ |
| v0.3，API 0.3 | 配置/核心迁移、具名双臂、两底盘、静态导航、TAMP执行前同步 | 最终实现150项通过；固定PickLift一次回归不是扰动鲁棒性验证，详见[发布说明](RELEASE_ONLINE_ENV_V0.3.md) |

历史回归目录：`output/online_env_final_audit/{fixed_pick_lift,randomized_pick_lift}`；
相机图像/指标见 `docs/assets` 与相机清单。大型output是本地证据，不随Git交付。
文档记录与仍存在的历史日志可引用，未重跑的实验不得改称本轮执行。
