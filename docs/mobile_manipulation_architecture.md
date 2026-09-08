# 在线移动操作环境架构

实现边界：SceneSmith产物 → 只读适配/派生cache → 实验profile组装 → 单一Runtime。
机器人定义、控制参数、初态、Task、Policy、evaluator分别管理；不引入第二套Runtime。
用户最新确认优先于历史任务书；行为详细契约见[GENERIC](GENERIC_ONLINE_EXAMPLE.md)，
运行命令见[Quickstart](QUICKSTART_ONLINE_ENV.md)，实际证据见[维护记录](EVAL_STRUCTURE_PROGRESS.md)。

## 实际实现映射

| 职责 | 实际路径/入口 |
| --- | --- |
| 通用执行 | `src/online_manipulation/runtime.py::RuntimeConfig/DrakeRuntime`，外层仍为 `make_env → OnlineManipulationEnv` |
| 统一模型 | `model.py::populate_model`，仿真与规划同一定义，默认同一Plant、独立Context |
| 可替换机构 | `adapters/description.py::DescriptionRobotAdapter`，真实两轴无夹爪fixture已运行 |
| 双臂定义 | `adapters/zerith_dual.py::make_zerith_dual_spec/ZerithDualRobotAdapter` |
| 移动机器人 | `adapters/zerith_mobile.py::ZerithMobileRobotAdapter`，拥有轮组、轴符号、安装frame及移动collision |
| 底盘执行 | `base.py::PlanarKinematicBase/WheelDrivenDynamicBase`，同一Runtime时钟 |
| 相机 | `sensors.py::CameraSystems`，RobotSpec.cameras来自机器人，renderer来自ScenarioSpec |
| 静态导航 | `navigation.py::Navigator`，`navigation_geometry.py::build_navigation_map` |
| 任务编排 | `recipes/navigation.py::NavigateExercisePolicy/ParkedExerciseTask`，由experiments/navigation_*.json选择 |
| 配置入口 | experiment.load_experiment → assembly.run → make_env/Shared Runtime；旧字典环境和重复CLI已删除 |

RobotSpec是解析后的机器人描述，硬件/安装归Adapter，初态和控制来自profiles。
Task/Policy/evaluator有各自实例；专家输入只在策略构造时读取。
正式代码不反向依赖examples/scripts/tools；IIWA基线独立可选。

### 动作与观测契约

- `RobotCommand(arms={名称: ArmAction}, grippers={名称: GripperAction}, base=BaseVelocityAction(...))`一次step统一提交。同步接受不代表同步到达。
- 各臂支持关节绝对/增量和Cartesian绝对/增量。名字从`spec.arm_groups/end_effector_frames`获取，不靠切片。夹爪宽度单位米，Zerith范围`[0,.07628]`；每侧映射两个手指，非单指行程。
- Cartesian保持world表达、commanded臂FK基准、实际手指几何、左乘world rotvec；每tick一次局部IK。abs必须持续提交，限幅残差不会在后台追踪。未新增局部Cartesian模式。
- 新组合动作对共同配置edge进行校验；任何指定分量失败，所有旧目标不变，仍推进仿真。`info['action_decision']`包含整体accepted、reason、components、edge证据。固定单臂关节/Cartesian和组合动作都执行对应核心碰撞检查，不因可选planner关闭而绕过。
- 默认step=.1s，20个.005s控制更新，每控制5个.001s物理更新。base速度命令仅本tick有效；省略base或Hold均按限幅制动。
- `BaseVelocityAction(v,omega)`：m/s、rad/s；navigation_frame的+X向前、世界+Z逆时针为正。
- `obs.robot`含具名q/v/q_commanded/torque、`end_effectors/gripper_widths_m/frame_poses_world`。`obs.base.pose`是导航参考点，另有物理`base_link_pose`和`odom_from_base_link/odom_from_navigation`，不能混用原点。
- 相机图像、pose、timestamp同帧采样保持。离散事件采样更新前状态，而同时间戳robot.q为更新后状态；移动测试用物理步速度误差界核对，不用最新FK覆盖缓存pose。reset返回t=0真实帧。
- `env.get_planning_query()`同步完整实际底座/关节/自由物体到独立规划Context，`state_time_s`记录时刻；每step/reset后重新获取。单独`build_planning_query`是初态查询，不会自动跟随环境移动。

### 两模式与导航假设

- 模式在创建时选定，不支持热切换。运动学为锁定PlanarJoint的规定小步运动，每1ms检查起点/中点/终点整机proximity，受阻返回`base.blocked/blocking_pairs`；不伪称自由体动力学或牵引响应。TCP公开速度补上规定底盘运动分量。
- 轮驱root保留6DOF，仅真实轮执行器受力矩；不覆盖root pose/velocity。左右轮轴+Y/−Y，r=.0835m、轮距=.379m；速度分别`(v−omega*b/2)/r`与`−(v+omega*b/2)/r`。模型限制2.3rad/s、60Nm与运行限制取较小者。
- 默认运行限速v=.15m/s、omega=.6rad/s、加速度=.25m/s²、角加速度=.8rad/s²、轮速servo gain=30。它们不是硬件标定。
- 仅四辅助轮collision摩擦置0并调平，两个驱动轮改r=.0835m圆柱。visual/惯量及地面/夹爪/物体摩擦不改。六轮实际承重，但辅助滑动支撑不等同真实脚轮。
- 规划压缩容差2mm只属于六具名轮—ScenarioSpec显式`ground_geometries`配对，不扩展Task指—目标容差，不修改动力学filter。
- 地图来自实际proximity高度范围+AABB障碍、整机保守圆盘；本示例双肘1.4rad、半径约.41566m。格点.1m、边采样.025m，包含旋转与倒车占地，但可能保守拒绝窄道。
- 本轮导航平面world Z=0。真实动态nav frame可能因沉降/倾角略偏离平面，观测不隐藏该事实；local/link目标须完整变换后成为平面pose。不能把有高度的物理base_link局部Z=0当作地面。
- `NavigationGoal(Pose(...), frame_id=...)`始终包含最终yaw；原始目标、解析时刻和world目标保留。目标不会跟随后续link运动。未知frame、非平面目标明确拒绝。
- Navigator只输出动作，不调用step。其速度带`control_owner='navigation'`，环境拒绝其他owner竞争。取消后继续`env.step(navigator.act(obs))`直到实测停车窗满足且状态cancelled。arrived/cancelled后`navigator.release()`，再发送一次`BaseVelocityAction(0,0,control_owner='navigation',release_control=True)`显式交接。
- 成功条件保持3cm、3°、.01m/s、.02rad/s且连续.5s。当前完整示例是导航停车后双臂关节运动和两空夹爪到宽度，**不是双臂协同抓取/搬运**。

操作阶段首次组合动作发送具名owner的零速度请求并释放导航控制权；之后`base=None`在每个step显式过期为零速度请求。轮驱通过零轮速目标的受限轮速伺服制动，不进行世界位置/yaw闭环保持，也不重焊；运动学限幅速度降至0后不再积分位移。操作阶段的相对起点偏差和实际速度统计见progress第13节，只有已有10Hz日志样本，不代替物理子步峰值。
