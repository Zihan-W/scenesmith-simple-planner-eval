# 两种底盘的实现核验与对照实验

2026-09-08。本次仅增加验证脚本和文档，没有修改Environment、底盘实现、模型、摩擦、控制增益、默认演示参数；未commit/push/tag。用户已查看两种HTML，确认均有运动，暂未发现明显视觉异常；视觉相似不作为实现错误证据。

## 1. 实际调用链

公共入口均为`OnlineManipulationEnv.step()`（`src/online_manipulation/environment.py`），调用`DrakeRuntime.step()`（`runtime.py`），完成动作校验后调用`base_backend.command()`保存本tick速度请求。速度命令不直接成为导航位置目标。

### 平面运动学

```text
BaseVelocityAction → Runtime.step → BaseBackend.command
  → 每1ms PlanarKinematicBase.before_physics
  → update_limit → integrate_planar
  → check_base_edge（规划Context的起点/中点/终点）
  → joint.set_translation / joint.set_rotation（真实仿真Context）
  → Simulator.AdvanceTo
```

- 积分及状态设置：`src/online_manipulation/base.py::integrate_planar`、`PlanarKinematicBase.before_physics`（约164—181行）。没有隐藏这一点：它就是规定运动，不是轮地牵引。
- `adapters/zerith_mobile.py::configure_model`仅在本模式添加`PlanarJoint`；`initialize_state`锁定该关节。运行中小步改变其平移/旋转；因此Drake求解器本身的锁定基座twist为0。
- 公共速度是规定运动速度。本实验额外从连续FK位姿做1ms差分，独立核对它；CSV保留`plant_*`、`public_*`、`pose_fd_*`三套字段，避免把锁定关节速度或请求值冒充动力学测量。

### 轮驱动力学

```text
BaseVelocityAction → Runtime.step → BaseBackend.command
  → 每5ms Runtime._servo_update
  → WheelDrivenDynamicBase.wheel_actuation
  → 加速度限幅 → 左右轮目标角速度 → 轮速误差力矩/限幅
  → 与机械臂关节力矩相加 → Plant actuation input FixValue
  → Simulator.AdvanceTo（内部物理步1ms）
  → 轮地接触和浮动整机动力学产生运动
```

- `base.py::wheel_actuation`（约187—223行）：半径0.0835m、轮距0.379m；左右轮轴符号(+1,−1)。目标为`[(v−ωb/2)/r, −(v+ωb/2)/r]`，轮速限制2.3rad/s，力矩`clip(30*(目标−实测轮速), ±60Nm)`。
- `runtime.py::_servo_update`（约231—236行）将两轮力矩和18个臂/指关节力矩送入实际执行器输入。`controller.py::CoupledInverseDynamicsServo.compute`虽然计算全模型逆动力学，但只提取具名臂/指关节广义力，不向未驱动底座施加“保持力”。
- 实际位姿和速度：`base.py::BaseBackend.observe`（约117—134行）从`navigation_frame.CalcPoseInWorld/CalcSpatialVelocityInWorld`读取；物理`dipan_link`位姿来自`EvalPoseInWorld`。本实验monitor另外直接读取同一真实Plant状态，不用轮速积分代替位姿。

### 排查状态覆盖和隐藏约束

- `ZerithMobileRobotAdapter.configure_model`的动力学分支只添加两轮执行器，不创建平面关节、不焊接world。
- `SetFreeBodyPose`只在`initialize_state`中设置reset初态。`DescriptionRobotAdapter.initialize_state`的`SetPositions/Lock`也只用于初始关节状态和具名被动关节。
- 实验实际模型：root floating=true、20执行器；锁定关节只有导轨、body pitch/yaw、neck pitch/yaw，未锁底座或驱动轮；`num_constraints=0`表示未添加额外constraint对象，不表示没有普通关节和内部锁定。
- `PlanningQuery.synchronize_state`的`SetPositions/SetVelocities`写入独立规划Context，不写实际Simulator Context。`Runtime.check_base_edge`的位姿设置同样只写规划Context，并且仅供运动学扫掠。
- 轮驱step分支只有力矩输入和`AdvanceTo`，未找到运行中覆盖底座pose/velocity、外加基座空间力输入或停车焊接路径。
- 这些静态代码核验与下述真实零驱动力矩反例共同构成证据；不是只看类名、`num_constraints`或测试数量。

## 2. 同指令实验与隔离设置

复用自包含空地面场景及现有默认参数，双臂均保持原导航收拢姿态（肘1.4rad），无导航器、无相机渲染。三个独立环境、相同seed=0：

| 仿真时间 | 请求v / omega |
| --- | --- |
| 0—2s | 0 / 0：静置 |
| 2—6s | 0.1m/s / 0：前进 |
| 6—9s | 0 / 0：请求停止 |
| 9—13s | 0 / 0.3rad/s：原地旋转 |
| 13—16s | 0 / 0：请求停止 |

保留原限速/限加速度：0.15m/s、0.6rad/s、0.25m/s²、0.8rad/s²。初始高度按各模式既有设置：运动学0.1816m；动力学0.1808m并自然沉降。两模式的既有支撑/轮几何差异未修改，不宣称模型完全相同。

隔离组`wheel_disabled`使用独立`AuditSettings(disable_drive_torque=True)`，仅在该测试进程把`wheel_actuation`返回向量中两个驱动执行器位置置零，安装在reset前。原servo仍计算目标和力矩，机械臂力矩、摩擦、轮模型、关节、初态、时间步均不变。不锁轮、不关闭重力、不改底座状态。monitor读取Plant实际actuation input确认两轮输入确实为0；CSV同时保留computed和applied两种力矩。

实际数据在`output/mobile_manipulation/base_mode_audit_01/`：

| 指标 | 运动学 | 正常轮驱 | 零驱动力矩 |
| --- | --- | --- | --- |
| 前进末1s平均v | 0.100000m/s | 0.099707m/s | −0.00000463m/s |
| 前进4s实际XY位移 | 0.380050m | 0.378018m | 0.00001412m |
| 旋转末1s平均omega | 0.300000rad/s | 0.299184rad/s | −0.00001504rad/s |
| 全程峰值实际轮力矩 | 不适用 | 4.4902Nm | 0Nm |

停止请求到降到`|v|≤.01m/s、|omega|≤.02rad/s`并保持的时间：前进停止运动学约0.360s、轮驱0.370s；旋转停止运动学约0.349s、轮驱0.360s。这不是到达任务的0.5s稳定窗，而是该指令响应实验的减速时间。

零力矩组前进期间峰值速度约`2.88e−5 m/s`，只出现约0.014mm微小漂移，未跟踪0.1m/s请求。动力学两组从reset至末帧均沉降约1.191mm，允许的沉降没有被伪装成前進运动。配置和指令一致性由分析脚本核对并写入JSON。

曲线接近的原因：原请求很慢、两模式使用相同加速度限制；轮驱在平地上有非零轮地摩擦，四辅助支撑无切向摩擦，轮速闭环有足够控制余量。轮驱仍存在起停滞后、微小滑移/偏差与沉降；运动学直接实现限幅后的积分。没有为放大视觉区别提高速度或削弱控制器。

### 原始曲线尖点的附加核对

`base_comparison.png`保留运动学monitor差分在4.986s的0m/s和9.816s的0.6rad/s两个孤立尖点，不能把它们误解释为实际积分骤停/倍速。独立复跑`base_sampling_audit_02/planar_kinematic`记录了全部原始monitor回调，以及每次`before_physics`实际写入前/后状态：

- 4.984999999999999s和4.985s的monitor分别处于规定状态写入前/后；下一整毫秒回调的采样侧发生变化。
- 9.815999999999999s和9.816s同样存在两次回调。记录器把距离小于1e−10s的时间合并，选取后者，导致名义1ms差分偶尔跨两个写入或零个写入。
- **16,000次实际写入**的时间步都约1ms。平移增量与`v*dt`最大误差`2.77e−17 m`，角增量与`omega*dt`最大误差`1.11e−16 rad`。本实验只含直行和原地转，因此该平移核对不需要曲线弦长修正。
- 原始CSV、尖点不删除或平滑；另给`base_sampling_audit_02/kinematic_sampling_diagnostic.png`比较monitor差分与真实写入增量。需要严谨比较运动学的瞬时速度时，应区分这两种采样侧。导航验收窗口中未发生这两个尖点，窗口检查使用保存的原始差分数据且通过。

这属于本次验证记录器与规定运动事件时序的边界问题，不是轮驱力矩通路证据失效，也未因此改动Runtime积分代码。

## 3. 导航完整到达窗口

两模式使用原障碍场景、相同Navigator默认阈值和world目标(2.8,0,yaw=0)，重新运行。记录Simulator每个约1ms物理状态，不只保存10Hz策略观测。

分析独立使用FK计算XY误差、最短yaw误差；动力学速度用实际twist，运动学额外用位姿差分核验。取Navigator宣布到达时刻之前完整0.5s，同时检查3cm、3°、0.01m/s、0.02rad/s，要求数据覆盖完整窗口且相邻采样不超过1ms（浮点误差除外）。逐行证据写入两目录的`arrival_window.csv`；四指标窗口最大值、采样数量和时间区间写入`comparison_metrics.json`。

这提供离散仿真1ms分辨率上的持续满足证据，不是采样之间连续数学安全证明。到达时刻取`Navigator.act(obs)`判断使用的obs时间，不混用执行下一step后的时间或缓存errors。

两模式实际窗口均为40.6—41.1s、501个样本、最大采样间隔约0.001s。窗口内各指标最大值如下，**不是只取最终一帧**：

| 模式 | XY误差 | yaw误差 | 实际平面速度 | 实际角速度 | 共同满足0.5s |
| --- | --- | --- | --- | --- | --- |
| planar_kinematic | 0.0185734m | 0.0105794rad（约0.606°） | 0m/s（位姿差分） | 0.0179104rad/s | PASS |
| wheel_dynamic | 0.0203996m | 0.0104091rad（约0.596°） | 0.000278766m/s | 0.0181680rad/s | PASS |
| 约定上限 | 0.03m | 0.0523599rad（3°） | 0.01m/s | 0.02rad/s | ≥0.5s |

初始瞬间角速度仍可非零，只要已经低于约定停车阈值；随后继续制动到接近0。未把“请求速度0”当作“实际已停”。

## 4. 可复制命令和图表

从已有安装完成的仓库执行，无需activate；所有输出放新目录，不覆盖历史证据：

```bash
cd /root/workspace/scenesmith-simple-planner-eval
AUDIT_ROOT="$(mktemp -d "$PWD/output/base_mode_review_XXXXXX")"
for CASE in planar_kinematic wheel_dynamic wheel_disabled navigation_planar_kinematic navigation_wheel_dynamic; do
  .venv/bin/python -m examples.online_manipulation.audit_base_modes --case "$CASE" --output "$AUDIT_ROOT" || break
done
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case analyze --output "$AUDIT_ROOT"
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case analyze_sampling --output "$AUDIT_ROOT"
```

脚本仅用已有NumPy、Matplotlib和CSV，不安装新依赖。第一次试跑因未安装pandas而失败，已改用现有依赖；失败日志保留。Drake1.49会对`is_floating()`给弃用提醒，本轮可正常执行，不代表模型错误。

图表位于上述输出根：

- `base_comparison.png`：指令/真实运动及位姿，对比三组。
- `base_transients.png`：正常两模式起停局部放大，参数未变。
- `wheel_torque_isolation.png`：轮目标/实际转速、计算/实际力矩。
- `navigation_arrival_window.png`：两模式四指标与0.5s核验窗。

每个case有`physics.csv`、`commands.json`、`manifest.json`；后者含模型拓扑、锁定关节、配置和原始CSV SHA256。总表还保存相关源文件SHA256。
