# Online Environment v0.3 发布说明

日期：2026-09-08。公共版本声明：`PUBLIC_API_VERSION = "0.3"`。
发布分支：用户fork的`dev/wzh`；annotated tag：`online-env-v0.3`。
v0.1、v0.2保留原tag对象及commit，不重建、不移动。运行入口见
[中文Quickstart](QUICKSTART_ONLINE_ENV.md)，接口细则见
[通用环境契约](GENERIC_ONLINE_EXAMPLE.md)。

## 本次发布范围

| 范围 | 交付内容 |
| --- | --- |
| 配置与运行链 | episode初态、control、scene_bindings、Task分组；专家IK/预抓取参数只由Policy消费。旧CLI薄转发到同一工厂和Runner；无专家文件也能构造/reset/step并运行Hold、JointStep。 |
| 共享Runtime | Environment负责公共生命周期，RobotAdapter拥有机器人定义，统一模型构建供仿真及独立规划Context使用。Task、Policy、Runner分离，每环境独立Task。 |
| 单臂/双臂 | 具名关节abs/delta、末端Cartesian abs/delta；RobotCommand在一次step同时提交双臂、双夹爪及底盘，统一校验共同动作边。 |
| 夹爪与相机 | 继承指部碰撞修复、0.07628m开口标定和机器人相机安装定义；三相机沿用同帧图像/pose/timestamp缓存和默认不内联像素的API hardening。 |
| 两底盘 | 创建时选择planar_kinematic或wheel_dynamic；前者为碰撞检查下规定运动，后者由驱动轮力矩及接触产生自由底座动力学。 |
| 静态pose导航 | 独立Navigator支持世界及接收时刻link表达的pose、绕静态障碍、倒车/原地旋转、取消及控制权交接；到达后进行双臂空手操作。 |
| TAMP交接 | 执行前自动读取完整当前规划快照，核验configuration/直接edge后发送绝对关节目标，已在实际移动底座后验证同步。不是完整TAMP规划器。 |

## 兼容性与迁移

- `models/zerith_pick_eval/environment.json`是任务episode配方，不是通用机器人
  硬件默认值。当前字段按`initial_state/control/scene_bindings/task`读取；旧实验
  扁平JSON须按此结构迁移。不要把专家文件重新塞回环境创建链。
- 示例工厂使用`--env-factory`和`--policy-factory`，不再使用旧未发布实验的
  混合`--factory`接口。旧`run_zerith_online_example.py`仍保留合理CLI参数，
  其joint-step默认0.01、closed-width默认0.03不静默改成专家演示值。
- 单臂旧7+1数组wrapper仅为Zerith兼容入口；新客户端优先具名typed动作。
  `RobotAdapter.gripper_position_targets(width_m, name=None)`须支持具名夹爪；
  None仅表示明确默认夹爪，不猜任意末端。受控单自由度关节必须对应唯一的
  `<joint_name>_actuator`执行器，构建时验证。没有夹爪/未知名字明确报错。
- 新组合动作检查共同配置及完整采样边。任一分量拒绝，组合新目标均不提交，
  仿真仍推进并可能跟踪旧臂目标；不是事务回滚或急停。携物关节动作补齐了
  与Cartesian等价的携物碰撞检查，因此旧版误放行的动作现在可能被拒绝。
- 单物体PickLift的carrier由Task绑定具名arm/gripper，身份歧义明确拒绝；
  不支持双手共同携物或携物时发送非零底盘动作，不偷偷选默认末端。
- Cartesian没有改成新坐标语义：位姿/平移增量用world表达，四元数wxyz，
  旋转增量为world rotvec左乘；从上一命令关节目标的FK作局部更新，而非最新
  实测位姿。每step一次局部IK；abs需持续发送，限幅残差不在后台补齐。
- 默认policy/servo/physics为10/200/1000Hz；臂和夹爪省略时保持旧目标，
  底盘非零速度只对当前step有效，省略base或Hold在下一step请求受控减速。
  accepted不等于到达，zero command不等于完全静止。
- 相机由RobotAdapter携带安装外参，renderer由ScenarioSpec提供；关闭相机
  不创建渲染系统。RGB/depth/label/pose/timestamp同帧保持，depth为米制
  Z-forward。内参仍为仿真内参。Drake 1.49.0时间端口fallback独立版本受控，
  不把未知Drake端口故障静默替换成推算时间。

## 模型和依赖准备

使用Python 3.11；依赖版本沿用`requirements.txt`，包括Drake 1.49.0。
安装、submodule和相机环境命令见Quickstart第1节。首次checkout后必须执行：

```bash
export REPO_ROOT="$(pwd)"
export PYTHON="$REPO_ROOT/.venv/bin/python"
git submodule update --init --recursive
"$PYTHON" "$REPO_ROOT/scripts/convert_zerith_for_drake.py"
"$PYTHON" "$REPO_ROOT/scripts/convert_zerith_for_drake.py" --check
```

源Zerith commit为`ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`。转换器v5
产出43个OBJ（35 visual + 8 finger collision）、65处派生URDF网格引用、
53个collision。OBJ不入Git，由STL重建；派生URDF和manifest已跟踪。
转换器5、SDF/URDF/XML格式版本及文档任务书版本不是公共API版本，不修改为0.3。
本仓库没有另一个pyproject/setup包版本源。

最小场景、相机标定场景、移动导航场景自包含；固定PickLift依赖完整外部
SceneSmith场景及四份ignored派生文件，详见Quickstart第4节。发布没有附带
大网格、HTML、CSV、临时日志、虚拟环境或专家输出，也未宣称全链资产可从
本tag独立重建。

## 验证证据

- 最终实现全量：**150项、581.234s、OK、exit 0**。日志路径
  `output/closure_audit/full_tests_submission_ready.log`，时序和实现指纹见
  [维护记录中的历史读取方式](EVAL_STRUCTURE_PROGRESS.md)；原progress §16保存在cb79ba8检查点。
  较早150项和最后24项重叠，不相加；版本收口仅改声明及对应版本断言，
  无控制、动力学、抓取或导航实现变化，因此复用最终全量。
- 版本针对性检查：`tests.test_online_api_contracts`的14项通过；公开API
  版本确认为0.3。转换器`--check`通过（43/70/65/53），仓库外最小公共
  API客户端检查reset/step。实际输出与执行命令记录在progress §17。
- 现有物理证据：两模式导航及停车后双臂动作、底盘同命令曲线及零轮矩
  隔离、具名联合碰撞、双臂可达小邻域Cartesian误差、移动相机缓存和TAMP
  当前状态同步。发布时不重复这些充分验证的实验。
- 固定PickLift本次迁移回归1次：277步、抬升0.102855139m、保持3.1s、双指
  接触、无桌面托举。是固定回归，不是新的随机鲁棒性结果；v0.1的固定3/3、
  扰动2/3仍仅为历史版本记录。
- 本轮没有重新安装全新虚拟环境或重建完整外部PickLift资产链，不把v0.2
  全新clone记录称为v0.3重新实测。具体实施/验证/范围外映射见progress §15。

## 能力边界

- 导航到达表示位置≤3cm、最短yaw误差≤3°、实际平面线速度≤0.01m/s、
  实际|omega|≤0.02rad/s同时持续0.5s（均可配置），不是完全静止。
- 导轨固定0.4m；动力学底盘停车是限幅零轮速伺服，不是世界位置锁定。
  四辅助轮为零摩擦滑动支撑，轮速增益及限制不是硬件标定。
- 静态保守地图和采样edge不能证明任意连续动态安全；目前是真值里程计，
  非SLAM。没有验证动态障碍、坡地、任意场景/工作空间鲁棒性或真实部署。
- 双臂验收是同一步控制、关节/Cartesian响应、独立夹爪及停车后空手动作；
  **没有验证双臂协同抓物、闭链搬运、移动中精细操作或PLACE**。
- TAMP仅执行交接；无完整任务/运动搜索。真实两轴无夹爪fixture验证通用
  Runtime，不等于第二个完整机器人项目已适配完成。

## 检出与最小运行

全新安装按Quickstart第1节执行；已有本版本依赖和OBJ时，从仓库根运行：

```bash
export REPO_ROOT="$(pwd)"
export PYTHON="$REPO_ROOT/.venv/bin/python"
export PYTHONPATH="$REPO_ROOT"
cd /tmp
"$PYTHON" -B "$REPO_ROOT/examples/online_manipulation/public_api_client.py" --repository-root "$REPO_ROOT" --seed 0
```

应该看到`action_status: accepted`、时间约0.1s；NullTask没有抓取成功目标。
两底盘可视化命令见Quickstart第6节，更换策略和任务见第7节。
