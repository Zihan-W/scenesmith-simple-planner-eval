# 通用在线环境：配置、控制与接入契约

本说明适用于 `online-env-v0.3`（公共 API `0.3`），不修改已发布的 v0.1/v0.2
tag。旧 `scripts/run_zerith_online_example.py` 已转发到相同工厂和 runner；
旧参数差异和专家文件的职责见第7节。

## 1. 先看清楚边界

配置字段的逐项归属，以及单臂 joint/delta pose/abs pose 控制接口，见
本文第7—8节。`environment.json` 是一个组合
episode 配置，并不是与场景、任务无关的机器人默认配置。

```text
minimal_setup.make_env_config() ──→ 环境配置 ──→ make_env(config)
                                      │                │
外部 make_policy(config) ←──────────────┘                │
         │                                             │
         └──────────── policy.act(obs) → env.step(action)
```

真正的环境实现仍在 `src/online_manipulation/environment.py`；示例中的
`minimal_setup.py` 只构造配置，不定义或创建 Policy。

| 文件 | 职责 |
|---|---|
| `examples/online_manipulation/minimal_setup.py` | 最小场景 + Zerith + NullTask 环境配置 |
| `examples/online_manipulation/example_policies.py` | 独立 MyPolicy、Hold 和关节增量工厂 |
| `examples/online_manipulation/pick_lift_demo/minimal_setup.py` | PickLift 场景及 Task 配置，无 IK 文件依赖 |
| `models/zerith_pick_eval/environment.json` | 机器人初始状态、控制周期、任务绑定与阈值 |
| `examples/online_manipulation/pick_lift_demo/policy.py` | 专家策略配置；只有它读取 PREGRASP 等标定 |
| `examples/online_manipulation/run_online.py` | 唯一示例入口，调用公共 make_env/run_episodes |

策略工厂可以读取环境的配置，用于匹配机器人名称和任务目标；它不接收 Drake
Context 或 runtime，不应修改配置。环境工厂既不导入策略工厂，也不创建策略。
换成 RL policy 时，可以完全忽略原专家策略及其标定文件。

## 2. 最小的 Python 使用方法

先在仓库根目录设置（已安装 `.venv` 并生成 Zerith URDF/OBJ）：

```bash
export REPO_ROOT="$(pwd)"
export PYTHON="$REPO_ROOT/.venv/bin/python"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd /tmp
```

下面是完整的最小循环，不需要通用 runner，也没有策略和环境的混合工厂：

```bash
"$PYTHON" -B - <<'PY'
from examples.online_manipulation.minimal_setup import make_env_config
from examples.online_manipulation.example_policies import MyPolicy
from src.online_manipulation import make_env

config = make_env_config()
env = make_env(config)
policy = MyPolicy()

obs, info = env.reset(seed=0)
policy.reset(obs, info)
for _ in range(10):
    action = policy.act(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    print(f't={obs.time_s:.1f}s reward={reward} terminated={terminated} truncated={truncated}')
    if terminated or truncated:
        break
print(env.finalize_episode())
PY
```

`MyPolicy.act()` 在独立文件中，目前返回 `HoldAction()`。只改这个方法即可
接入自己的动作生成逻辑，不需要改 `minimal_setup.py`。
这个最小环境 1 秒后截断；NullTask 没有成功目标，所以 success=False 是预期。

## 3. 用统一入口保存完整记录

沿用上述变量，从任意目录执行：

```bash
export RUN_OUTPUT="$(mktemp -d /tmp/online-example.XXXXXX)"
"$PYTHON" -B -m examples.online_manipulation.run_online \
  --env-factory examples.online_manipulation.minimal_setup:make_env_config \
  --policy-factory examples.online_manipulation.example_policies:make_hold_policy \
  --output-root "$RUN_OUTPUT/hold" \
  --seeds 0 1 \
  --max-steps 10 \
  --write-final-dmd
```

换策略只改 `--policy-factory`，例如
`examples.online_manipulation.example_policies:make_joint_step_policy`。
这个具体策略对 `left_shoulder_pitch_joint` 发送一次 0.003 rad 增量，然后保持；
它是 Zerith 策略示例，不声称该关节名适用于所有机器人。

两个工厂协议：

```python
def make_env_config():  # 无参数
    return config      # 实现公共 EnvironmentConfig

def make_policy(config):
    return policy      # 实现 reset(obs, info) 和 act(obs)
```

工厂是可信的本地 Python 代码，模块必须能从 PYTHONPATH 导入。
也可直接 `run(config, policy, ...)`；不必通过 CLI，也不必把自己的网络模型
塞进现有 Policy 类。环境负责应用动作，策略负责计算动作。

可视化只需将环境工厂换成 `:make_visual_env_config` 并添加 `--record-html`。
策略无需知道是否启用 Meshcat。输出目录中有批次 JSON/CSV，每个 episode
独立保存 `summary.json`、`trace.csv`、可选 `simulation.html`、`final.dmd.yaml`。
未成功还会生成 `failure.json`。输出目录必须为空或不存在，禁止覆盖旧结果。

## 4. 换场景和任务，不动策略循环

场景路径、package.xml、观测对象、机器人初始位姿由环境配置决定；Task
负责奖励、成功和任务规则，不属于 Policy。场景变化后需要重新确认物理初态，
不能假设旧场景的机器人高度或任务绑定自动适用于新场景。

对于 PickLift：环境只需要已生成的 DMD、完整外部场景资产、仓库机器人模型
和 `environment.json`。不再要求它读取 `pick_home.json`、`pregrasp_ik.json`
或专家抓取标定。三个文件仅被 `pick_lift_demo/policy.py` 使用。

同一 PickLift 环境切换策略的完整命令见
[PickLift 独立环境与策略示例](../examples/online_manipulation/pick_lift_demo/README.md)。

`environment.json` 的初始状态来源于之前人工确认的场景：固定底座
`(2.65, 2.95, 0.1815)`、yaw 180°、导轨 0.4 m、左臂全零。它现在是该示例
环境的初态来源；专家构造时核对旧标定，不匹配会报错，不会覆盖环境位置。

## 5. 本版边界

- 不改物理、控制器、安全层、Task 成功条件或原专家状态机。
- 默认仍是策略 10 Hz、控制 200 Hz、物理 1000 Hz；一个 step 是 0.1 秒
  仿真时间，不保证耗时等于 0.1 秒墙钟时间。
- 环境时长以秒配置，CLI `max_steps` 是执行预算；两者仍独立，任一先到
  都会结束。Hold 冒烟测试的 max_steps 截断不是 PickLift 成功。
- 旧未发布的 `--factory` 混合工厂接口已替换为两个独立工厂；实验性的
  `pick_lift_demo/run_online.py` 重复文件已移除。reset/step形式及合理旧CLI参数
  保留；具名RobotAdapter契约、配置迁移和携物安全检查的兼容变化见
  [v0.3发布说明](RELEASE_ONLINE_ENV_V0.3.md)，并非所有扩展接口完全不变。
- 本版不提供 Gymnasium wrapper、action_space/observation_space、close、
  向量化训练或学习算法。
- Runner 通过公共 `StoppablePolicy.stop_reason` 处理策略主动失败，不解析
  PickLift diagnostics。携物关节/Cartesian 的新旧动作等价性见第9节；
  几何等价测试不代表新的双臂物理抓取验收。

## 6. 历史验证（2026-09-07，分离工厂工作树，非当前重测）

- 39 项相关单元测试通过：环境/策略边界、独立工厂 CLI、公共 API、runner
  以及已有 BT/TAMP 示例；没有宣称运行全量测试。
- 仓库外最小 CLI 两个 episode 和原公共 API 客户端通过。
- 将 PickLift DMD 单独复制到不含专家标定文件的目录，成功构建环境；在同一
  实例运行 Hold 和关节增量各 10 步，reset 后机器人和物体观测均恢复一致。
- 分离工厂后的固定 PickLift seed 500：success=True、lift_held、277 步，
  抬升 0.102855 m，保持 3.1 秒。双指接触成立、无桌面支撑，记录中无力矩饱和。
- 抓取结果位于 `output/decoupled_picklift_seed500/episode_000_seed_500/`，
  包含 HTML、JSON、CSV、最终 DMD。HTML 约 450 MB，尚待本次人工视觉复验。

此次验证的是一个固定 PickLift episode，不是三次重复性或随机扰动测试。

## 7. 配置归属与旧入口迁移

`models/zerith_pick_eval/environment.json` 由
`pick_lift_demo/minimal_setup.py::make_config` 唯一读取。它是一个特定 episode
配方，不是机器人默认定义；结构分组已经进入代码，不只是改了说明。

| 字段 | 实际用途/读取后去向 | 归属 |
| --- | --- | --- |
| `description` | 说明文字，不参与仿真 | 配方说明 |
| `initial_state.robot_xyz/robot_yaw_deg` | 解析到机器人初始世界 pose；分别米/度 | 本次初态 |
| `initial_state.rail_position/q_home_left` | 导轨米、左臂7个关节rad；reset 初始化 | 本次初态，不改 URDF 零位 |
| `control.timing` | `TimingConfig` 的 physics/controller/policy 周期，秒 | 运行控制 |
| `control.episode_duration` | 环境时间截断，秒 | 运行预算 |
| `control.max_joint_delta` | 每步关节目标限幅 | 运行控制，不替代硬件限位 |
| `control.maximum_cartesian_joint_delta` | 每次局部 IK 关节增量上限 | 运行控制 |
| `scene_bindings.target_model_name/target_body_name` | `ObservedBodySpec` 及 Task 目标完整名称 | 场景对象绑定 |
| `scene_bindings.support_contact_bodies` | Task 支撑接触判定 | 场景与 Task 的绑定 |
| `task.target_observation_name` | 观测别名及 Task 查找键 | Task |
| `task.required_lift_m/required_hold_s` | 抬升/保持成功门槛 | Task |

关节/link/TCP/手指 contact body、相机安装由 RobotAdapter 提供，不在 JSON
重复。硬件位置/速度限位来自模型；仿真伺服增益在 `src/zerith_servo_config.py`
及适配器内组装，不能宣称是已标定硬件控制器。RobotSpec 是解析完成的描述，
可带本次 home/增益；没有为此再创建一套平行配置类。

真实调用链：

```text
新 CLI → env_factory + policy_factory → run(config, policy)
旧 CLI → build_run → 同一个 make_config + build_policy（仅 PickLift）
                                    ↓
run → make_env → OnlineManipulationEnv(Task, DrakeRuntime)
    → run_episodes / EpisodeRunner → reset → policy.act → env.step
DrakeRuntime / PlanningQuery → model.populate_model → RobotAdapter
```

旧 CLI 的 Hold/JointStep 使用 `NullTask`，不读取 pick_home/pregrasp/calibration。
初态来自同一 environment.json。`--environment-json` 可显式选择另一配方。
仍接受旧专家参数，但非专家策略不使用；PickLift 专家只在 `build_policy` 读取，
并核对其 base/rail/home/目标与环境一致，**不允许专家文件反向覆盖初态**。
此前依赖专家文件隐式修改 home 的自定义调用，需明确提供一致的 environment.json；
不一致现在报错。这是可见的迁移行为，不是兼容猜测。

保留旧 CLI 默认 joint step=0.01、closed width=0.03、时长=(max_steps+1)×policy_dt；
新配方默认分别为0.1、专家closed width=0、120.1秒。它们是显式入口覆盖，
不是第二份场景/Task 来源。固定 PickLift 命令显式使用0.1/0/1200后，解析后的
初态、Task、Timing和控制上限与新工厂一致，针对性测试逐项比较。

## 8. 单臂与双臂动作、坐标及执行语义

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

## 9. Task 携物与 RobotAdapter 契约

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
不因此改变。旧关节动作在**携物状态**新增这项共同边检查；空手旧分支保留原语义。
这项安全修复可能拒绝过去遗漏检查的携物关节动作，是明确的行为修正。

不支持双手共同搬一个物体，也不以本单物体 Task 支持移动中搬运；携物时非零
base 命令明确报错。新接触等价测试是几何/解析验证，物理基准仍为固定单臂
PickLift。新的任意关节抓取策略、移动携物及双臂实际抓取未验收。

TAMP 的 `execute_validated_joint_goal(env=..., goal_positions=...)` 自动调用
`env.get_planning_query()` 并读取当前 observation，不信任旧 query/obs 参数；
当前底座、关节及全部自由物体在执行前同步。它发送计算后的绝对 goal，而非
写死0.001。移动后同步已有针对性实际运行证据。它是静态直接边执行示例，
不是动态重规划或完整 TAMP；单独持有 `build_planning_query()` 不会自动刷新。

## 10. 相机 hardening 契约（从已删除文档迁入）

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

## 11. 版本化历史，不当成当前实测

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
