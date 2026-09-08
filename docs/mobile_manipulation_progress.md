# 移动双臂仿真环境：执行进度与交接

初始化：2026-09-07
当前状态：此前P0—P7证据及双模式人工验收保留；整套迁移收尾的逐项实现/验证见§15，不能仅以P0—P7摘要代替整体结论。“停车”均指连续满足配置的到达与停车阈值，不宣称完全静止。没有暂存、提交、推送或修改tag。
目标仓库：`/root/workspace/scenesmith-simple-planner-eval`
先读：`mobile_manipulation_requirements.md` → `mobile_manipulation_architecture.md` → 本文件。

## 1. 使用规则

- 每阶段开始/完成、出现阻塞或准备交接时更新本文件；阶段通过后自主推进，不重复请求用户确认已定事项。
- 只把有本轮证据的验收标为 PASS。历史日志、测试代码存在、Mock 结果、计划都不能替代实测。
- 状态使用 TODO / RUNNING / PASS / FAIL / BLOCKED / NOT_RUN。禁止以“基本完成”混淆未验证项。
- 记录本地实际时间及 UTC 偏移，不虚构工作小时数。记录命令、退出码、配置、日志路径和结果来源。
- 保存关键结果到仓库约定输出目录，不仅存 /tmp；不要提交大体积运行产物。本轮不 commit/push/tag。
- 不覆盖用户未提交工作，不 reset/clean。不删除失败结果来只保留成功运行。
- 遇到问题先定位修复；阻塞无法解决时记录证据，继续独立任务，不用 Mock/瞬移替代真实 backend。

## 2. 启动基线（由执行 agent 填写）

| 项目 | 值 |
| --- | --- |
| 检查时间/时区 | 2026-09-08 00:24 +08:00（shell `date -Is`） |
| 实际仓库路径 | `/root/workspace/scenesmith-simple-planner-eval` |
| 分支、完整 HEAD | `dev/wzh` / `b433ee6dd5b20017fd141adbbb5910347526c3a2` |
| 工作树改动与归属 | 旧 ONLINE_ENV 三文档删除；Quickstart、公共 API、Cartesian、通用示例、PickLift 示例及 environment.json 等既有未提交工作全部保留；本任务三文档也未跟踪。开始状态保存到 `output/mobile_manipulation/p0/status_before.txt`。 |
| Python/Drake/运行依赖 | `.venv/bin/python` 3.11.13，Drake 1.49.0 |
| 当前可运行入口 | `examples.online_manipulation.run_online`、`mobile_smoke`、`navigate_demo`、`navigation_manipulation`；原单臂脚本已转共享 Runtime |
| 固定 PickLift 基线配置及产物 | `p0/fixed_picklift`、`shared_fixed_picklift_01`、`final_fixed_picklift` 均 seed 500 / 277 步成功 |
| 模型/submodule 版本 | `models/Zerith_Model` = `ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be` |
| 关键未决模型事实 | 轮组模型事实已核实；硬件真实摩擦/电机响应/脚轮机制仍无标定，本实现明确使用仿真假设 |

历史报告不作为证据。本轮已重跑固定 PickLift 与全量测试，详见下方带日志的记录。

## 3. 阶段计划与门禁

| 阶段 | 工作 | 进入下一阶段的证据 | 状态 |
| --- | --- | --- | --- |
| P0 | 检查代码/AGENTS、记录工作树与基线、确认模型事实 | 基线与事实表；明确实际旧语义及差异 | PASS |
| P1 | 配置单一来源、专家依赖移出、独立 Task、Runner 协议 | A01；固定基线无未解释回归 | PASS |
| P2 | 通用真实模型构建/执行、去固定维度 | A02；规划与仿真模型定义一致 | PASS |
| P3 | 双臂/夹爪组合动作与共同校验 | A03/A04；两臂真实运行，同一步时间证据 | PASS |
| P4 | 平面运动学 backend、碰撞与坐标更新 | A05 运动学部分；无穿障、odom/TCP/相机一致 | PASS |
| P5 | 轮驱动力学 backend、辅助支撑、停车控制 | A05 动力学部分、A06；轮驱与接触证据 | PASS |
| P6 | 静态地图、pose 导航、目标固定、倒车/旋转与状态 | A07/A08/A09 两种模式均验证 | PASS |
| P7 | 导航停车后双臂操作、公共示例、文档与回归 | A10/A11/A12；全量相关回归和交接清单 | PASS |

依赖：P6 可在 P4 后开发并先验证运动学，但最终必须回到 P5 模式验证；不得因此把动力学省略。P3 或 P5 阻塞时可以推进不依赖它的文档/地图等工作，清楚标注未通过门禁，不宣称整体完成。

## 4. 模型事实与假设记录

| 字段 | 实测/代码值 | 来源文件/符号或测量 | 类型：事实/仿真假设 | 影响 |
| --- | --- | --- | --- | --- |
| 左右驱动轮及转轴符号 | left/right_middle_wheel_joint；轴分别 +Y/-Y；URDF effort=60 Nm、velocity=2.3 rad/s | 派生 URDF 与上游轮组定义；P0 XML 检查 | 模型事实（非硬件标定） | 前进为左正/右负 |
| 轮半径与轮距 | 网格 X 半径 0.0835 m，Z 半径 0.08335873 m；轮距 0.379 m | trimesh bounds；joint origin y=±0.1895 | 模型事实 | 低面数圆周；轮驱阶段需验证接触 |
| 四个辅助轮碰撞几何 | left/right_front/rear_wheel_link：仅 wheel_dynamic 置静/动摩擦0，保留法向接触；前+1.5mm、后−0.08539mm调平collision | `ZerithMobileRobotAdapter.add_model` | 仿真假设；视觉/惯量不变 | 不是有转向自由度的真实脚轮 |
| 接地高度与载荷 | 两驱动轮圆柱 r=.0835m、长.0415m；初始root Z=.1808m，实际沉降约.17961m；收拢停车各轮约195N | `navigation_wheel_02/result.json`；质量119.39526432kg | 几何/质量为模型值，调平为假设，载荷为实测 | 六轮均承重，驱动轮有牵引 |
| base_link 与导航参考系 | dipan_link；navigation_frame 相对root为[.0544,0,−base_height_m] | 移动适配器；公开base_link_pose和odom_from_base_link | 编制参考系 | 运动学root=.1816m；动力学六自由度保持实际沉降/倾角 |
| odom 是否实体 link/连接 | 派生 URDF 根为 dipan_link，没有 odom link | XML 检查 | 事实 | 应公开固定世界真值 odom，不新增随车刚性 odom |
| 左右臂/TCP/夹爪映射 | 每侧7轴+2手指关节；grasp_frame相对wrist_pitch=[.176,0,−.002]，+X接近、+Y闭合；开口0.07628m | `make_zerith_dual_spec` / `add_tcp_frames` / GripperSpec | 左侧已标定，右侧使用对应机械结构与镜像伺服增益 | 不宣称真实硬件标定或闭链控制 |
| 移动基座控制器处理 |18个位置执行器+轮驱模式2轮力矩执行器；全模型ID，基座期望加速度0近似，只取实际驱动关节广义力 | Runtime + CoupledInverseDynamicsServo + WheelDrivenDynamicBase | 控制近似 | 未驱动基座不施加虚构保持力；操作后实测漂移 |

未知数值不得伪称硬件数据。对必要仿真参数可自主采用可审查初值并验证，记录理由；无法支持合理模型的缺失事实才作为阻塞。

## 5. 验收台账

| ID | 状态 | 命令/配置 | 证据路径 | 结论与限制 |
| --- | --- | --- | --- | --- |
| A01 | PASS | `tests.test_mobile_composition`、`test_picklift_demo_example` | `p0/p1_composition.log`、`p0/shared_fixed_01.log` | 两真实环境隔离；无专家环境构造；任务/策略独立 |
| A02 | PASS | `tests.test_description_runtime` | `p0/p2_two_joint.log` | 真实2轴、无夹爪，共享执行和模型，不是Mock |
| A03 | PASS | `tests.test_dual_runtime` | `p0/p3_dual.log`、`p0/dual_cartesian.log` | 双臂同一步响应、宽度独立、坏分量拒绝整体；两种Cartesian接口 |
| A04 | PASS | 真实独立臂合法/组合冲突；双目标允许接触几何 | `p0/independent_arm_collision_search.log`、`p0/dual_task_contact.log` | 新强反例指腕−25.5mm；Task仅放行明确指/目标对；不是双臂物理抓取证明 |
| A05-K | PASS | `mobile_smoke --mode planar_kinematic`；移动相机测试 | `final_kinematic_smoke`、`p0/mobile_cameras_03.log` | 修复后高度.1816m；前后转曲线/停止；同帧采样验证 |
| A05-D | PASS | `mobile_smoke --mode wheel_dynamic`；移动相机测试 | `final_wheel_smoke`、`p0/mobile_cameras_03.log` | 轮驱实际速度、根位姿、相机/TCP更新 |
| A06 | PASS | 两轮力矩伺服、六轮接触载荷 | `base_model_audit.json`、`navigation_wheel_02/result.json`、`final_wheel_smoke/states.json` | 约195N/轮；无root运行中覆盖、无重焊 |
| A07-K | PASS | `navigate_demo --mode planar_kinematic --meshcat` | `navigation_kinematic_02` | 绕障到pose；.01857m/.595° |
| A07-D | PASS | `navigate_demo --mode wheel_dynamic --meshcat` | `navigation_wheel_02` | 绕障到pose；.02040m/.576° |
| A08-K | PASS | `validate_mobile_navigation` | `navigation_cases_01/planar_kinematic_local_result.json` | 初始yaw90°、完整腕部表达、目标固定、实际到达；.01802m/.617° |
| A08-D | PASS | `validate_mobile_navigation` | `navigation_cases_01/wheel_dynamic_local_result.json` | 初始yaw90°、完整腕部表达、目标固定、实际到达；.01809m/.638° |
| A09-K | PASS | 无路径/窄道/取消测试；实际阻挡倒车恢复 | `p0/navigation_contract_01.log`、`navigation_cases_01/kinematic_blocked_trace.json` | 受控停车/控制权；整机扫掠阻挡并可倒退脱离；圆盘保守窄道拒绝 |
| A09-D | PASS | 无路径/窄道/取消测试；实际直倒转 | `p0/navigation_contract_01.log`、`final_wheel_smoke` | 实际取消停车/控制权；同一静态地图拒绝窄道/无路径；直接轮速命令不自带运动学扫掠防撞 |
| A10-K | PASS | `navigation_manipulation --mode planar_kinematic --meshcat` | `parked_dual_kinematic_02`、`final_parked_kinematic`、`acceptance_metrics.json` | 停车≥.5s，双臂+双夹爪到位；理想基座漂移0 |
| A10-D | PASS | `navigation_manipulation --mode wheel_dynamic --meshcat` | `parked_dual_wheel_02`、`final_parked_wheel`、`acceptance_metrics.json` | 停车≥.5s，平移漂移.172mm，yaw漂移.00113°；无障碍接触/关节饱和 |
| A11 | PASS | 原固定PickLift命令/旧关节回归脚本 | `final_fixed_picklift`、`p0/legacy_command_02.log` | 277步成功；旧CLI hold/7关节阶跃/CSV/HTML可用 |
| A12 | PASS | 全量测试、外部客户端、移动相机 | `p0/full_tests_public_snapshot.log`、`external_wheel_final`、`external_kinematic_final` | 最终141项/469.837s OK；两模式仓库外实际运行、移动规划快照和相机回归通过 |

K/D 合并行实际执行时应拆行，避免一种模式通过掩盖另一种未运行。

## 6. 每次关键运行的最小记录

- run_id、时间与时区、分支/HEAD、当前工作树 diff 标识、依赖版本。
- 场景、机器人、底盘模式、初态、seed、控制限制、导航容差及地图参数。
- 精确启动命令、退出码、运行状态和异常堆栈（如果存在）。
- 双臂前后实际关节/TCP、命令提交时刻、step 起止仿真时间。
- 原始导航 pose/frame、接受时刻、固定后的世界目标。
- 时间序列：实际底盘 pose、v/omega、位置误差、最短 yaw 误差、稳定计时、导航状态。
- 碰撞/受阻/动作拒绝记录；动力学轮速、力矩、接触与停车后漂移；运动学受限运动记录。
- JSON/CSV 与可视化路径。相同 demo 的重复运行不是多个随机场景验证。

## 7. 阶段更新模板

### P1—P6 开发及实测：2026-09-08 00:58 +08:00

- 阶段门禁仍逐项验收：P2 的新真实 runtime 已跑两轴与双臂，但旧固定入口尚未全部转为薄转发，因此 P2 迁移尚未完成；P3 的联合碰撞反例与 P6 的取消/局部目标测试尚待补齐。以下运行不等于这些阶段全部 PASS。
- `PYTHONPATH=tests:. .venv/bin/python -m unittest tests.test_episode_runner -v`：9 tests PASS。新增显式 `StoppablePolicy.stop_reason`，Runner 不再解析 PickLift diagnostics 控制终止。此生命周期接口调整明确记录，原 diagnostics 继续作为日志。
- `.venv/bin/python -m unittest tests.test_description_runtime -v`：真实两轴无夹爪，1 test PASS，0.103 s；同一 plant/diagram 的独立规划 context，q/reset/20次伺服验证。
- `.venv/bin/python -m unittest tests.test_dual_runtime tests.test_description_runtime tests.test_cartesian_pose_action -v`：前两项 PASS，最后一项因旧测试文件采用顶层 test 模块导入而 ERROR（完整日志 `p0/p3_dual.log`）。随后 `PYTHONPATH=tests:. .venv/bin/python -m unittest tests.test_cartesian_pose_action -v`：5 tests PASS，`p0/cartesian_original_semantics.log`。
- `PYTHONPATH=tests:. .venv/bin/python -m unittest discover -s tests -v`：133 tests / 354.402 s，1 FAIL（旧 state-only JSON 顶层多了 base 字段）。已修复为仅移动观测输出 base；未改测试期望，等待全量重跑。其余相机、固定接口、抓取测试均通过。日志 `p0/full_tests_01.log` 保留。
- 两模式基础脚本：`.venv/bin/python -m examples.online_manipulation.mobile_smoke --mode wheel_dynamic --output output/mobile_manipulation/wheel_smoke_01`；相同命令改为 `planar_kinematic` 和 `kinematic_smoke_01`。均退出 0，直/倒/转/曲线/停止 JSON、CSV 已保存。运动学初版发现辅助前轮初态低于地面0.7mm，配置高度已由0.1808改为0.1816，仅影响理想运动学；须以修复后重跑为验收依据。
- 轮驱只改派生移动模型的2个驱动轮圆柱 collision（r=0.0835、长0.0415），4个辅助 collision 设零摩擦，前辅助碰撞局部 z+0.0015、后辅助 z−0.00008539 使底部共面；视觉、惯量和上游文件不改。该支撑调平是仿真假设，不是硬件标定。未修改地面/夹爪/场景对象摩擦。
- 实测轮驱前进0.0997 m/s、倒车−0.0997 m/s、旋转0.30087 rad/s；状态读取物理 link，非命令积分。收拢姿态终点驱动轮各约195N，4辅助各约195N，六轮均实际承重（`navigation_wheel_02/result.json`）。底盘为6DOF浮动，不重焊、无运行中pose覆盖。伺服仅向实际关节/轮执行器分配力矩，未驱动基座无隐形执行器；机械臂ID采用零基座期望加速度近似，动态耦合的更强补偿未实现。
- 导航命令：`.venv/bin/python -m examples.online_manipulation.navigate_demo --mode wheel_dynamic --output output/mobile_manipulation/navigation_wheel_02 --meshcat`；实际 arrived，41.2s，位置误差约0.0204m，yaw约0.0101rad，停车窗0.6s；结果保存实际速度及逐轮载荷。地图来自整机 proximity、高度筛选和保守0.41566m圆盘；左右肘初态1.4rad，起始几何验证通过。
- 第一轮 `navigation_wheel_01` 也到达，但最终对齐阶段存在位置边界切换低效，失败/低效记录未删。第二轮加入最终yaw阶段迟滞，D07/D08门槛不变。相同配置运动学 `navigation_kinematic_02` 正在运行，尚不写PASS。
- 正常轮地支撑允许有物理接触压缩。规划新增按支撑body/指定ground配对的2mm上限，仅作用该模式6轮；Task夹爪/物体阈值不改变，所有真实动力学过滤不变。须增加配对范围回归，不能将该限值泛化。

### P0 / P1 实测更新：2026-09-08 00:38 +08:00

- 固定基线命令：`SCENE_ROOT=/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000 PICK_ARTIFACT_ROOT=/root/workspace/scenesmith-simple-planner-eval/output/zerith_pick_eval .venv/bin/python -m examples.online_manipulation.run_online --env-factory examples.online_manipulation.pick_lift_demo.minimal_setup:make_env_config --policy-factory examples.online_manipulation.pick_lift_demo.policy:make_policy --output-root output/mobile_manipulation/p0/fixed_picklift --seeds 500 --max-steps 1200`；退出 0，277 步，`success=True reason=lift_held`。JSON/CSV 在该 output 目录。本轮未为基线重新录制 HTML。
- `.venv/bin/python -m unittest tests.test_mobile_composition tests.test_picklift_demo_example -v`：退出 0，5 tests / 19.299 s。两个真实 Drake 环境独立计数、Hold/JointDelta 单步 0.1 s；构造不读专家文件。
- P1：environment.json 分为 initial_state/control/scene_bindings/task，数值不变；夹爪接触 link 归 GripperSpec；ZerithEnvironmentConfig 每次构造复制独立 Task。Runner 生命周期协议和通用构建迁移尚未完成，不标 P1 PASS。
- 旧 Cartesian 语义：world 表达、commanded FK 为基准、单次局部 IK、旋转 world 左增量、abs 需每 tick 重发；本轮保留。旧 JointDelta 分支原本未做 edge 检查，移动双臂分支将按新需求增加共同边检查并明确区分。
- 底盘质量 92.40985888 kg，整机质量 119.39526432 kg。中轮中心 z=-0.0965，辅助前轮最低 z=-0.1815，驱动轮最低约 -0.17986；四辅助固定轮可能卸载驱动轮，P5 必须实测，不假定已有牵引。
- 安全审核曾阻止通用 runtime 补丁，误将 A11 固定回归与移动实现混同；补丁未落盘。已核对 architecture §5.3 和 requirements A11：允许 fixed 基准，不计移动模式。描述适配器增加 fixed-only 显式保护；移动实现须另有非焊接构建，无模式 fallback。

### P? 更新：<实际时间及偏移>

- 状态：
- 本阶段目标：
- 实际修改（文件/关键符号及原因）：
- 与原计划差异：
- 验证命令及退出码：
- 通过证据：
- 失败/未覆盖限制：
- 对既有行为的影响：
- 下一步最小动作：

## 8. 阻塞台账

| ID | 现象与复现 | 已尝试及结果 | 根因/仍不确定 | 影响阶段 | 可继续的工作 | 需要用户决策吗 |
| --- | --- | --- | --- | --- | --- | --- |
| — | 无阻止本轮门禁完成的未解决阻塞 | 失败及修复记录见§7、§10、§11 | 未覆盖范围见§10.1，不等同已验证 | — | — | 否 |

停止修改已确认语义前，明确写出具体冲突及备选方案。普通代码组织、参数初值、搜索算法选择自行处理；不要因为用户睡觉或缺少阶段性确认就停止已授权工作。

## 9. 最终交接必须回答

1. 哪些 A 项 PASS、FAIL、BLOCKED、NOT_RUN？两种模式分别列出。
2. 如何创建环境并选择两种 backend？给出真实可复制命令。
3. 如何一次 step 控制双臂、如何提交世界/local pose？示例使用实际公共 API。
4. 如何运行导航→停车→双臂操作？两模式各自误差、稳定时间、漂移如何？
5. 改了哪些物理参数？辅助轮零摩擦是否仅影响目标几何？有哪些仿真假设？
6. 旧入口、固定 PickLift、相机、Cartesian 语义是否回归？未解释差异是什么？
7. 当前工作树、未提交修改、产物位置、剩余问题和下一步是什么？

完成定义：需求要求的真实能力和证据齐全；不是增加 Action 类、Mock 测试通过或写好文档。若未全完成，诚实交接实际状态，不调整验收阈值来制造成功。

## 10. 最终回归前状态：2026-09-08 01:52 +08:00

- 实际已跑 `full_tests_02.log`：140项 / 458.011s / OK。固定 PickLift 最终目录 `final_fixed_picklift`：seed500、277步、lift_held。它是同一初态固定回归，**不是扰动鲁棒性验证**。
- 新增两臂Cartesian真实测试已通过；旧世界系/commanded FK/左乘rotvec/每tick一次IK保留。新组合关节分支做公共共同edge检查，旧单臂JointDelta为兼容原语义不强加新拒绝条件，二者明确区别。
- `src/zerith_online_env.py` 已改为旧8维/字典接口到 `DrakeRuntime` 的兼容转发，不再持有第二套模型/物理步循环。旧typed归一化桥仍保留，最终底层同一Runtime。自动审核曾拒绝一次大范围删除归一化桥的尝试，该删除未执行；采取保留桥、验证实际共享执行的方案，不影响功能门禁。
- 旧CLI首跑暴露循环导入（`p0/legacy_command.log`）。机器人伺服常量移到 `src/zerith_servo_config.py`，原模块重导出；复跑 `p0/legacy_command_02.log` 通过，初态有效穿透0，hold最终误差0、饱和0、7轴阶跃完成。没有修改控制增益。
- 相机首轮误拒根因：相机t=0事件同时触发Plant更新，指关节出现−2.54e−11m止挡数值误差。只给关节位置**核验**加1e−9数值容差，命令限位、碰撞门槛不变；1e−6越界仍拒绝。失败证据 `mobile_camera_collision.log`、`external_wheel_01.log` 保留。
- 相机第二轮是测试比较了离散采样前和物理更新后状态：同一时间戳差约一个1ms物理更新；已以速度给出误差界并保留精确图像/pose/时间缓存断言。未改外参/渲染/频率。两模式三相机 moving-base + moving-arm、.25s慢相机、.05s策略、reset图像确定性通过：`p0/mobile_cameras_03.log`。
- 外部命令在 **/tmp** 执行，只导入公共API，输出0.5s、双夹爪实际约.060/.055m、三相机时间.5s：`external_wheel_02/client.json`、`external_kinematic_01/client.json`。RGB/NPY同目录。公共规划快照的后续补测需以新外部结果为准。
- 独立CSV审计结果 `acceptance_metrics.json` 含原CSV SHA256，重算停车窗、位置/yaw、漂移、关节饱和和障碍接触，不用命令判断成功。两种模式均0障碍接触、0关节饱和；HTML为实际仿真录制，尚未经本轮用户人工视觉验收。
- 轮尺寸/轴符号/模型速度力矩上限归适配器 `drive_spec`；运行速度/加速度/servo增益归 `BaseConfig`。支持观测同时区分 `navigation_frame` 与 `dipan_link`；world/map/odom对齐，公开实际 `odom_from_base_link`。运动学TCP速度显式补上规定基座运动分量，不能误说它有牵引动力学。
- 新 `env.get_planning_query()` 刷新独立规划Context中的**完整实际状态**，含基座、关节、自由物体和采样时刻；调用者不需要Drake Context。每次step/reset后重新获取。轮地可压缩接触范围同时用于共享规划，仍严格局限具名轮—指定地面。

### 10.1 保留限制

1. 静态地图是整机proximity高度筛选、AABB障碍及半径约.41566m的保守圆盘，不是精确SE(2)网格；窄通道可能正确地保守报无路。动态障碍、坡地不在范围。
2. 运动学模式是锁定平面关节的规定运动、每1ms三点扫掠；不模拟底盘反作用力。轮驱模式是实际浮动6DOF，四辅助轮为无摩擦滑块，不是真实脚轮。
3. 动作edge是采样几何检查，不是连续时间动力学安全证明；基座扫掠用当时实测臂姿，移动中精确操作未验收。本轮编排固定为停车后再双臂操作。
4. 双臂闭链、双臂真实协同抓取未实现。端到端验收是两臂到关节目标、两空夹爪独立到宽度，不应叫“双臂搬运成功”。
5. 新组合动作对候选夹爪位置执行几何检查；允许目标接触以Task阈值为准，不能把“穿过物体的完全闭合位置目标”当作可行位姿。任意形状物体的双手力控抓取需要独立策略/接触验收。旧固定PickLift物理接触路径已保留通过。
6. 硬件真实摩擦、电机动态、右臂增益、载荷鲁棒性未标定；右侧使用显式镜像仿真参数。导轨锁定.4m，不是导轨动力学。
7. 移动示例场景自包含（但需要上游submodule和本地生成OBJ）；固定PickLift仍依赖外部SceneSmith原始资产及派生DMD/专家Policy文件。
8. 原有用户工作和三份旧文档删除均保留。未commit/push/tag，不要求工作树干净；不可用git clean/reset清除本轮或用户改动。

## 11. 最终交接核对：2026-09-08 02:05:49 +08:00

本轮实际时间跨度约1小时42分钟（00:24起），非仿真时间。以下最终命令均退出0；路径基于仓库根，证据目录统一为`output/mobile_manipulation/`。

```bash
PYTHONPATH=tests:. .venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m examples.online_manipulation.navigation_manipulation --mode wheel_dynamic --output output/mobile_manipulation/final_parked_wheel
.venv/bin/python -m examples.online_manipulation.navigation_manipulation --mode planar_kinematic --output output/mobile_manipulation/final_parked_kinematic
```

- 最终全量日志`p0/full_tests_public_snapshot.log`：**141 tests / 469.837s / OK**，包含真实2轴无夹爪、双臂共同碰撞、两臂Cartesian、移动相机和原有测试。
- 最终轮驱421步/42.1s、运动学420步/42.0s，均`success=True`、`parked_dual_targets_held`；和此前录制HTML的运行误差一致。日志分别`p0/final_parked_wheel.log`、`p0/final_parked_kinematic.log`。真实双臂/双空夹爪响应，不代表双手抓物。
- 公共外部客户端按architecture §12命令在`/tmp`执行，实际输出目录改为`external_wheel_final`、`external_kinematic_final`。各5步/0.5s，三相机同帧时间0.5s；每步独立规划快照时间和左右TCP与实际观测一致。对应日志`p0/external_wheel_final.log`、`p0/external_kinematic_final.log`，无私有Context读取。
- `base_model_audit.json`：运动学root非浮动、18执行器；动力学root浮动、20执行器、32位置/31速度。动力学2驱动圆柱摩擦1、4辅助Mesh摩擦0；轮地配对之外的摩擦未修改。两模式默认PlanningQuery有效。
- 固定PickLift最终277步成功，独立于移动演示。此项以及重复导航都只说明固定配置回归，不说明随机鲁棒性。
- `git diff --check`通过；HEAD仍`b433ee6dd5b20017fd141adbbb5910347526c3a2`，submodule仍`ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`。工作树保留本轮及用户既有改动和三份文档删除；无commit/push/tag操作。

### 可视化及证据索引

- 轮驱：`parked_dual_wheel_02/episode_000_seed_0/simulation.html`。
- 运动学：`parked_dual_kinematic_02/episode_000_seed_0/simulation.html`。
- 同目录`summary.json`、`trace.csv`为真实状态/任务/控制证据；顶层有benchmark汇总。
- `acceptance_metrics.json`独立从上述CSV重算停车窗、误差、漂移、接触与饱和，含源文件SHA256。
- `external_wheel_final`、`external_kinematic_final`各有三相机RGB、深度NPY和公共观测JSON。
- HTML可下载后本地打开，每份约225MiB；本轮尚无用户人工视觉验收。完整命令、公共组合动作、世界/局部导航及控制权交接见architecture §11—12。

至此停止扩大功能范围。移动中的精细操作、真实脚轮/硬件标定、随机场景鲁棒性、双臂物理协同抓取和导轨动力学均未宣称完成。

## 12. 两模式实现对照与零轮力矩隔离：2026-09-08 10:39 +08:00

用户视觉反馈已记录：两模式均确实运动，暂未发现明显异常，肉眼难区分不视为实现错误。本轮只新增`examples/online_manipulation/audit_base_modes.py`和`docs/BASE_MODE_IMPLEMENTATION_AUDIT.md`、更新本进度；没有修改生产核心、导航/控制参数、URDF、摩擦、默认演示或环境依赖。

### 实际命令与证据

在仓库根运行以下case，均退出0（独立进程，导航未参与前三组指令实验）：

```bash
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case wheel_disabled --output output/mobile_manipulation/base_mode_audit_01
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case wheel_dynamic --output output/mobile_manipulation/base_mode_audit_01
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case planar_kinematic --output output/mobile_manipulation/base_mode_audit_01
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case navigation_wheel_dynamic --output output/mobile_manipulation/base_mode_audit_01
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case navigation_planar_kinematic --output output/mobile_manipulation/base_mode_audit_01
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case analyze --output output/mobile_manipulation/base_mode_audit_01
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case planar_kinematic --output output/mobile_manipulation/base_sampling_audit_02
.venv/bin/python -m examples.online_manipulation.audit_base_modes --case analyze_sampling --output output/mobile_manipulation/base_sampling_audit_02
```

以上为已执行目录，不应再次覆盖。新的可复制命令使用`mktemp -d`，见核验报告§4。每个case有`physics.csv`、`commands.json`、`manifest.json`；运行日志保存于`base_mode_audit_01/logs/`和`base_sampling_audit_02/run.log`，分析日志及JSON保存在相应根目录。

- 同指令：0—2s静置，2—6s前进0.1m/s，6—9s停止，9—13s原地转0.3rad/s，13—16s停止。三组指令序列相同，默认限加速度保持不变。
- 正常运动学/轮驱前进4s位移分别0.380050/0.378018m；稳态v分别0.100000/0.099707m/s；旋转稳态omega分别0.300000/0.299184rad/s。正常轮驱峰值实际轮力矩4.4902Nm。
- 仅验证进程中切断两个驱动执行器输出，控制器计算仍保留，其他参数/模型/初态一致。实际actuation input验证两轮力矩全程0；前进阶段漂移约0.01412mm、峰值|v|约2.88e−5m/s，是正常行程的0.003734%。没有跟踪请求速度。沉降约1.191mm如实保留。
- 静态代码+实际拓扑核对：动力学root浮动、20执行器、无附加constraint，锁定的仅导轨/body/neck关节；无运行中底座pose/velocity覆盖、无world焊接。规划Context状态复制不等同修改Simulator。详见报告调用链。
- 导航两模式均在40.6—41.1s有501个约1ms样本共同满足四阈值。运动学窗口最大位置0.0185734m、yaw0.0105794rad、速度0、omega0.0179104rad/s；轮驱分别0.0203996m、0.0104091rad、0.000278766m/s、0.0181680rad/s。`arrival_window.csv`逐行保留，不能用最终pose替代。
- 两模式前进停车耗时约0.360/0.370s，旋转停车约0.349/0.360s（降到停车速度阈值后保持）；接近的曲线来自低速、相同限加速度及轮速闭环，未人为放大差异。

### 失败与采样问题记录

- 首次脚本因未安装pandas而失败（`logs/base_audit_disabled.log`），未安装新依赖，改用已有NumPy/CSV后通过。没有掩盖运行失败。
- 查看原始曲线发现两个运动学1ms差分尖点。进一步记录原始monitor回调和16,000次实际状态写入，证实浮点近同时刻的写入前/后回调被记录器合并，造成差分采样侧错配；实际每次积分增量误差≤1.11e−16。原始尖点保留，不平滑；`base_sampling_audit_02/sampling_metrics.json`及诊断PNG提供证据。导航验收窗口没有该尖点，原始窗口仍满足阈值。
- 本轮没有修改生产代码，因此没有重新运行上一轮141项全量测试，不把历史测试冒称本轮新结果。本轮五组实际实验、额外采样定位复跑、两个分析命令和语法检查已运行；`git diff --check`通过。
- HEAD仍`b433ee6dd5b20017fd141adbbb5910347526c3a2`。保留所有既有工作；未commit/push/tag。

关键图：`base_comparison.png`、`base_transients.png`、`wheel_torque_isolation.png`、`navigation_arrival_window.png`位于`base_mode_audit_01`；`kinematic_sampling_diagnostic.png`位于`base_sampling_audit_02`。四阈值持续证据以1ms离散采样为界，不宣称连续数学证明或硬件标定完成。

## 13. 最终交接：操作阶段已有日志核算（2026-09-08 +08:00）

本次只读已有CSV、核对代码依赖并整理文档，未运行新仿真或重跑全量测试。已更新Quickstart第6节：模式选择、公共组合动作、global/local pose、取消/失败、范围和未提交工作树说明。requirements/architecture明确到达与停车阈值，未改变任何数值或控制语义。

### 13.1 相对操作开始状态的最大偏差

数据源是用户已查看HTML对应的`parked_dual_wheel_02`和`parked_dual_kinematic_02`，不是后续独立导航核验的日志。取首次`policy_diagnostics_json.stage=manipulate`动作执行**前一行**实际状态作为操作开始基准；该XYZ与策略记录的`parking_pose`逐项相等。包含基准帧及所有操作后的帧，不只比较首尾。

- 平移：`max ||p(t)-p(t_start)||`；分别计算XY和XYZ。
- yaw：对`yaw(t)-yaw(t_start)`做最短角归一化后取绝对值最大。
- 线/角速度：取实际世界速度范数/实际yaw角速度绝对值，不减去开始时的速度。偏差基准与速度表达系不是同一概念。

| 已记录样本最大值 | wheel_dynamic | planar_kinematic |
| --- | --- | --- |
| 操作时间区间 | 41.3—42.1s | 41.2—42.0s |
| 样本数（含操作开始基准） | 9，间隔0.1s | 9，间隔0.1s |
| XY平移偏差 | 0.000172382336m（0.172382mm） | 0 |
| XYZ平移偏差 | 0.000172384465m | 0 |
| yaw偏差 | 0.000019693879rad（0.001128376°） | 0 |
| 实际平面线速度 | 0.000975738795m/s | 0（规定运动速度） |
| 实际3D线速度 | 0.000982161152m/s | 0（规定运动速度） |
| 实际yaw角速度绝对值 | 0.000649651747rad/s | 0（规定运动速度） |

**缺失项：该双臂操作阶段没有保存200Hz/1000Hz完整状态序列，无法给出物理子步的真实峰值。** 上表只能称为已有10Hz样本最大值。不能用独立导航阶段的1ms记录补齐另一段操作的缺失数据，也不以0.8s短动作结果宣称长时间保持或扰动鲁棒性。

核算产物`output/mobile_manipulation/manipulation_handoff_metrics.json`保存源CSV SHA256、参考pose/时刻、全部9个样本、各最大值及发生时刻、操作阶段实际动作和缺失声明。计算使用Python标准库csv/json/math读取旧文件，不修改原始日志。

### 13.2 操作阶段底盘命令与保持方式

`NavigateExercisePolicy.act`首次从navigate进入manipulate时，发送`BaseVelocityAction(0, 0, control_owner='navigation', release_control=True)`；之后组合动作`base=None`。CSV中的首个action和后续所有action与此一致，所有操作后`requested_velocity`为`[0,0]`。

- Runtime每tick将省略base解释为0速度请求，而不是维持最后的非零速度。
- wheel_dynamic：轮速目标0，经现有受限轮速伺服输出驱动力矩。**没有世界位置/yaw闭环保持、没有底盘锁定或重新焊接**，允许实测微小偏移。
- planar_kinematic：规定速度限幅至0后不再积分位移。零漂移是该理想模式性质，不应称为真实动力学抗扰动性能。
- Task在该示例中继续检查到达位置/yaw阈值与停车速度阈值，并结合双臂/夹爪误差持续0.5s判断完成；不要求底座完全静止。

### 13.3 采样尖点影响范围确认

引用已完成的`base_sampling_audit_02`诊断，不重新跑大规模测试：16,000次实际写入增量误差≤1.11e−16，两个尖点来自审计monitor近同时刻前/后样本的合并和差分。

依赖检查：`Recorder.save()`在episode运行结束后才生成`pose_fd_*`；生产`src/`、导航和操作示例没有读取这些字段或导入审计脚本。公共Observation速度来自`BaseBackend.observe`（规定运动速度或实际twist）；Navigator读取公开pose/速度；控制器读取实际关节速度及限幅命令，不读取审计差分。因此**这两个审计尖点没有反馈到公共observation、导航成功判定或控制输入**。

本轮只读对比首轮与附加采样复跑：均16,001帧；公开v/omega、实际x/y/yaw逐帧最大差为0，完整公共动作序列相等。独立导航原始1ms窗口也已通过，不把差分尖点当作控制速度尖峰。这里确认的是该特定审计问题的影响范围，不宣称所有传感器事件时序都无需进一步检查。

## 14. 待提交改动分类（仅清单，未暂存/提交）

归属依据：P0的`output/mobile_manipulation/p0/status_before.txt`与`tracked_before.patch`，加当前工作树实际状态。**启动时已有的未跟踪文件不属于本轮新建。** 同一个文件可能叠加原有工作与本轮实现，不能仅按文件名把整个文件归给某一轮。

| 类别 | 文件/范围 | 归属说明 |
| --- | --- | --- |
| 原有删除，继续保留 | `docs/ONLINE_ENV_ARCHITECTURE.md`、`docs/ONLINE_ENV_PROGRESS.md`、`docs/ONLINE_ENV_REQUIREMENTS.md` | 用户在P0前已删除；未恢复 |
| P0前已有的修改 | `docs/QUICKSTART_ONLINE_ENV.md`、`examples/online_manipulation/public_api_client.py`、`src/online_manipulation/{__init__,actions,adapters/zerith,planning,tasks}.py` | baseline已标M；其中Quickstart及公共核心又叠加本轮内容，需要按差异审查 |
| P0前已有的未跟踪工作 | `docs/GENERIC_ONLINE_EXAMPLE.md`；`examples/online_manipulation/{cartesian_client,cartesian_policy,example_policies,minimal_setup,run_online}.py`、`pick_lift_demo/`；`models/zerith_pick_eval/environment.json`；`tests/test_{cartesian_pose_action,generic_online_example,picklift_demo_example}.py` | 不是本轮新建；PickLift组装、environment.json等有本轮职责整理叠加，不覆盖原工作 |
| 用户提供的任务文档 | `docs/mobile_manipulation_{requirements,architecture,progress}.md` | P0已存在未跟踪；本轮追加实际实现、证据和最终交接，不作为全新需求发明 |
| 本轮新实现模块 | `src/online_manipulation/{runtime,model,sensors,base,navigation,navigation_geometry}.py`；`adapters/{description,zerith_dual,zerith_mobile}.py`；`src/zerith_servo_config.py` | 共享真实runtime、模式实现、独立导航、机器人描述和常量归属 |
| 本轮修改既有模块 | `src/zerith_online_env.py`、`scripts/run_zerith_online_example.py`；`src/online_manipulation/{contact,environment,observations,policies,protocols,runner,specs}.py`，以及上方重叠文件 | 旧入口转发、组合动作/观测、共同碰撞检查、生命周期及配置整理 |
| 本轮场景/机构fixtures | `models/mobile_scene/`、`models/runtime_fixture/` | 自包含静态导航场景与真实非7轴机构/接触验证模型；未改上游submodule |
| 本轮运行/验证入口 | `examples/online_manipulation/{mobile_smoke,navigate_demo,navigation_manipulation,mobile_public_api_client,validate_mobile_navigation,audit_base_modes}.py` | 前五项用于公共示例/验证；audit脚本专门做机制隔离和采样审计，非生产功能 |
| 本轮测试 | 新增`tests/test_{description_runtime,dual_runtime,mobile_cameras,mobile_composition,mobile_navigation}.py`；修改`test_{architecture_boundaries,episode_runner,zerith_cameras}.py` | 加真实执行与边界/回归检查；原有测试工作仍保留 |
| 审计文档与本轮最终整理 | `docs/BASE_MODE_IMPLEMENTATION_AUDIT.md`、Quickstart第6节、progress第12—14节及相关措辞 | 核验报告、交接和缺失项；本次最终整理没有新增运行功能 |
| 本地产物，不建议纳入源码提交 | `output/mobile_manipulation/`下HTML、CSV、JSON、PNG、日志；本地OBJ及环境 | 保留作证据/运行资产；未删除、未新增Git/LFS跟踪 |

本次最终整理仅修改文档并生成一份从旧CSV导出的统计JSON。未改变核心、动力学、抓取参数、默认示例或tag；未暂存任何文件。最后核对`git diff --check`、HEAD/submodule及删除状态；用户确认后再决定提交分组，不在本轮执行。

## 15. 整套迁移收尾（2026-09-08）

本节是用户整体盘点后授权的收尾修复，覆盖此前发布、发布后提交、所有未提交
迁移及跨阶段衔接。不是仅做双底盘收尾，也没有新增抓取/导航能力。上面各阶段
记录保留其当时含义：例如旧CLI曾只共享Runtime、仍重复解析专家与环境配置，
本轮才补齐工厂和Runner的薄转发；不能把旧阶段PASS理解成此前没有迁移缺口。

### 15.1 版本与工作树基线（只读）

| 对象 | 本轮核实结果 |
| --- | --- |
| 当前分支/HEAD | `dev/wzh`，`b433ee6dd5b20017fd141adbbb5910347526c3a2`，没有创建提交 |
| v0.1 tag对象/commit | `2781df041d2382d8d9d3680e1edda321e3b81403` / `25eb9a40586c6234ae71b87b8ac4c6629702d0f6` |
| v0.2本地tag对象 | `d80d1b41b329b6c047b67b3b18f62f143d6f4c74`（annotated） |
| v0.2本地解引用commit | `682ba7c1168c49d2be4e71d7ba3d9fd48308d082` |
| 用户fork远端v0.2 | tag对象与解引用commit均逐字匹配本地；`git ls-remote git@github.com:Zihan-W/scenesmith-simple-planner-eval.git refs/tags/online-env-v0.2 'refs/tags/online-env-v0.2^{}'` |
| 23d656c与682ba7c | `23d656c`发布API0.2代码；其后`7fa0b9b`、`682ba7c`为Quickstart文档提交。`git diff 23d656c..682ba7c --stat`仅一个文件Quickstart，281增/85删 |
| 历史原因的证据边界 | 内容历史与annotated message支持“中文文档并入v0.2”；当前refs不能证明谁在何时用何种push/retag命令替换，未据此编造历史操作 |
| origin风险提示 | origin fetch仍为作者HTTPS地址，push为用户fork SSH地址；故远端核验显式指定用户fork，没有改remote |
| 模型与环境 | Zerith submodule仍`ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`；Python3.11.13、Drake1.49.0 |

23d656c不是当前tag落点，但其后直到682ba7c没有运行代码差异。b433ee6才是
tag之后指部碰撞/开口修复；本轮保留，没有再次修改模型、摩擦、控制增益或抓取参数。
没有fetch、移动tag、force push或其他引用写操作。初次默认网络只读查询被沙箱
阻止，随后获准针对用户fork的只读查询成功；不是把作者仓库结果当用户fork结果。

### 15.2 整体验收映射

“实现完成”只指本轮已确认范围，“验证完成”注明本轮或此前证据；范围外不写成
缺陷已修复，也不把证据不足写成实现一定缺失。代码存在不自动等于所有行为已验收。

| 原始诉求/衔接 | 当前实现与涉及文件/提交 | 实现状态 | 验证状态及证据 | 未完成/未验证/影响 |
| --- | --- | --- | --- | --- |
| 模型可加载、指部接触不过度穿模 | v0.1派生URDF/转换器；b433ee6指部几何与0.07628m开口修复；`convert_zerith_for_drake.py`、`models/zerith_drake` | 已提交完成；本轮不改 | 此前人工验收；本轮固定PickLift仍通过 | 任意CAD原始表面精确碰撞、硬件标定未验证；OBJ仍本地生成 |
| 初态/场景/Task/控制归属，不依赖专家 | `environment.json`四分组、`pick_lift_demo/minimal_setup.make_config`；机器人contact links来自RobotSpec | 迁移完成 | 本轮工厂配置逐字段比较；旧CLI无专家实际Hold/JointStep；既有双环境隔离测试 | JSON是episode配方，非通用机器人默认配置；外部PickLift资产仍需提供 |
| 通用运行核心、非7+1 | `RuntimeConfig/DrakeRuntime`、`model.populate_model`、`DescriptionRobotAdapter`、`zerith_online_env.py`薄wrapper | 共享实现完成 | 真实两轴无夹爪fixture、公共API Mock契约；本轮回归包含这些测试 | 第二个完整真实机器人未做；不是把Mock当物理实现 |
| Environment/Task/Policy/Runner解耦 | `environment.py/tasks.py/policies.py/runner.py`；两独立工厂、`StoppablePolicy` | 完成 | 独立Task、无专家工厂、Runner失败机器可读及公共外部运行证据 | 未引入Gymnasium/向量化训练；没有用PickLift成功替代架构验收 |
| 旧CLI与新入口相同配置来源 | `scripts/run_zerith_online_example.build_run` → `make_config/build_policy` → `run_online.run` | 本轮修复完成 | `test_closure_contracts`逐项比初态/Task/时钟/限幅；旧CLI固定277步；无专家2步×2策略 | 旧默认0.01/0.03与新0.1/0作为显式覆盖；自定义专家与环境初态冲突明确报错，不隐式覆盖 |
| RobotAdapter签名/执行器映射 | `protocols.py`具名夹爪；`description.py/zerith.py`实现；`model.validate_actuator_mapping` | 本轮修复完成 | 命名双夹爪、无夹爪、缺失/错关节执行器构建测试 | 保留公开`<joint>_actuator`、单DOF约定；不提供任意传动矩阵/闭链执行器 |
| 单臂joint/Cartesian与双臂组合 | `actions.py`、`runtime._combined_candidate`、固定resolver；相同时间统一提交 | 完成 | 此前双臂真实运动/共同碰撞反例；本轮reachable abs/delta真实测试 | 每tick局部IK、commanded FK、world左乘不变；无任意工作空间精度结论 |
| 单物体Task携物与新动作等价 | `PickLiftTask.allowed_contacts`明确carrier；`runtime.check_command_edge`、`ZerithFixedCommandResolver.resolve` | 本轮修复完成 | `test_picklift_command_contact`：四种arm动作×旧/Composite/RobotCommand关系一致；joint/Cartesian障碍拒绝；固定真实PickLift | 几何夹持关系不替代真实接触；双手共同搬物/移动携物范围外并明确拒绝；新双臂物理抓取未验收 |
| 规划与仿真消费同一定义/当前状态 | `populate_model`、共享Plant独立Context、`env.get_planning_query`；TAMP helper自动取快照 | 本轮衔接修复完成 | 实际轮驱移动后传旧t=0 query/旧obs，执行取t=4.5s及当前base frame；不只核对调用了方法 | 静态直接边示例；动态重规划、在线全身TAMP范围外 |
| 相机与坐标系继承 | `CameraSystems`、RobotSpec.cameras、`_drake_camera_time`、只读`sensors`；旧外参不变 | 已有实现保留 | v0.2三相机人工/数值验证；此前移动腕/底座慢相机；本轮回归含camera硬化及外部client | 仿真内参，不是硬件标定；未重新做三相机人工视觉验收，不做目标检测 |
| 两种底盘真实机制 | `base.py`、`zerith_mobile.py`及轮组配置 | 此前完成，本轮不改 | 用户接受的`base_mode_audit_01`、零轮矩隔离、`base_sampling_audit_02`、载荷日志 | 不重跑机制实验；辅助支撑不是实际脚轮；无硬件动力学校准 |
| 独立global/local pose导航与停车后双臂 | `navigation.py/navigation_geometry.py`、`navigate_demo/navigation_manipulation` | 此前范围完成 | 两模式实际到达窗、用户已看HTML；本台账§13最大漂移 | 成功是满足配置到达/停车阈值；不是完全静止。仅空夹爪双臂操作，不是双臂搬运 |
| 示例、文档、既有删除 | README/GENERIC/Quickstart、三mobile文档、PickLift README | 本轮承接完成 | 本地链接存在性检查；版本化历史、控制和camera硬化迁入现存文档 | 三份ONLINE_ENV删除保留；历史35OBJ/276步不能拿来判断当前43OBJ/277步；没有重新做全新clone |

上述实现的关键限制已列在GENERIC第7—11节及architecture第13节。本轮重点修复
迁移接缝，不扩展PLACE、双臂物理抓取、动态障碍或移动中操作。

### 15.3 本轮实际命令与结果

命令工作目录为仓库根；`output/closure_audit`是本轮本地产物目录，不纳入源码提交。
从11时段开始执行收尾，下面均为2026-09-08（+08:00）实际日志，不是历史结果重命名。

```bash
PYTHONPATH=tests:. MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B -m unittest tests.test_online_api_contracts tests.test_closure_contracts -v
PYTHONPATH=tests:. MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B -m unittest tests.test_picklift_command_contact -v
PYTHONPATH=tests:. MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B -m unittest tests.test_cartesian_handoff -v
PYTHONPATH=tests:. MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B -m unittest discover -s tests -v
```

- `contracts_final.log`：19 tests / 23.151s / OK。TAMP实际命令0.06m/s、0.08rad/s
  持续15步，之后10步Hold；按既有验证前置先收拢肘部1.4rad并静置2s。导航参考点
  X从0.054397变为0.144782m，旧query时间0，执行前query4.5s，目标执行1步。
  `tamp_pose_final.log`另外核对实际base_link的规划/观测坐标一致，不只核对时间。
- `contact_02.log`：3 tests / 68.326s / OK。Task携带关系覆盖JointDelta、
  JointPosition、CartesianDelta、CartesianPose；人为场景重叠仅用于规划反例，
  不推进该碰撞fixture动力学，不伪称物理抓取成功。
- Cartesian第一次在`contracts_cartesian_01.log`联跑已通过，后续全量再次覆盖。
  `cartesian_metrics.json`、`cartesian_trace.csv`保存60个实际abs样本及delta。
  左右目标来自合法FK、共同边有效；末1秒位置均<1mm、姿态<1°。该固定数值场景
  收敛到约3.24e−15m/8.06e−15rad浮点量级，仅为仿真同模型局部测试，不是精度宣传。
  delta请求world +Y 0.5mm、world左增量+X 0.003rad，左右实测命令FK变化分别约
  `[0.000000164,0.000500,0.000000809]`m，旋转X约0.0029996rad；通过实际伺服滞后
  区分commanded-FK和实测-FK基准，下一步Hold不追加增量。
- `full_tests_final.log`：150 tests / 578.788s / OK，进程退出0。此前底盘机制
  对照/零轮矩隔离实验未重跑；测试集内的原有短契约回归继续保留。
- 全量启动之后补充的固定适配器具名观测（`end_effectors['left']`、
  `gripper_widths_m['left']`与原单臂字段相等）单独由
  `named_fixed_observation.log`覆盖：24 tests / 85.998s / OK、退出0。
  此补充不改变旧字段或控制逻辑；明确区分全量快照与最后的针对性回归。

固定PickLift实际命令（本机外部场景路径沿用用户已提供的场景）：

```bash
MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B scripts/run_zerith_online_example.py pick-lift output/zerith_pick_eval/zerith_pick_eval.dmd.yaml --scene-package-xml /root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000/package.xml --pick-home-json output/zerith_pick_eval/pick_home.json --output-root output/closure_audit/fixed_picklift --episodes 1 --seed 500 --max-steps 1200 --maximum-joint-step 0.1 --maximum-cartesian-joint-step 0.02 --closed-width 0 --write-final-dmd
MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B scripts/run_zerith_online_example.py hold output/zerith_pick_eval/zerith_pick_eval.dmd.yaml --scene-package-xml /root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000/package.xml --output-root output/closure_audit/hold_no_expert --max-steps 2
MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B scripts/run_zerith_online_example.py joint-step output/zerith_pick_eval/zerith_pick_eval.dmd.yaml --scene-package-xml /root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000/package.xml --output-root output/closure_audit/joint_no_expert --max-steps 2
```

三条均退出0。PickLift `success=True, reason=lift_held, steps=277`：升高
0.102855139m、保持3.1s、双指接触True、support_contact=False、0个policy样本
力矩饱和。后两条不带专家参数，`success=False, reason=max_steps, steps=2`，
是NullTask预算终止，不是抓取失败。输出目录存在时重跑应换名，不覆盖旧证据。

外部最小运行采用Quickstart §7.1完全相同模块/参数，在`/tmp`执行，输出根
指定为`output/closure_audit/external_minimal`；只用自包含场景、NullTask+Hold，
无需专家文件或外部SceneSmith目录。实际退出0，打印
`seed=0 success=False reason=time_limit steps=10`，写回对象`portable_object`，
summary/trace/final DMD均保存。NullTask的success=False是预期，不伪称抓取成功。

TAMP坐标加强检查退出0：1 test / 22.652s，`tamp_pose_sync_metrics.json`记录
旧base pose `[0,0,0.1808]`与新实际/规划pose
`[0.0907696947,-0.0006049667,0.1796094824]`m逐项一致，精度断言1e−10。

### 15.4 失败/修复记录与未验证项

- `targeted_initial.log`初次20项中7项错误：1项为Drake1.49 Finalize前
  instance执行器列表未填充的误报，已改为全部执行器按归属筛选；其余Task测试
  fixture缺少本轮携带身份查询，已补规范fixture。`targeted_second.log`16项通过。
- `contracts_cartesian_01.log`：Cartesian通过，TAMP初始测试未使用此前已验证
  的收拢/静置前置，+X位移断言失败（−0.0427m）；不据此宣称底盘机制错误。
  改为复用导航收拢姿态与静置前置后实际移动和同步通过，默认演示/控制参数不改。
- `contact_tamp_01.log`：7通过、1测试API误用（collision_pairs不接收
  contact_policy）；改按其真实carried_bodies/additional_body_names接口查询，
  `contact_02.log`三项通过；不是删除携物碰撞检查来通过。
- `full_tests.log`首轮未完成，已见一个公共工厂兼容错误：本轮不应把planning
  新增为所有第三方OnlineEnvironment的必选方法。撤销该协议扩张，TAMP helper
  使用内置环境已有能力；`contracts_final.log`通用外部配置契约恢复通过。
  随后完整重跑另存`full_tests_final.log`，保留首轮原始日志。
- 固定277步是1次本轮回归，不是新的3次重复性或扰动测试。v0.1固定3/3与
  ±3mmXY/±2°yaw扰动2/3、seed402策略失败作为版本化历史迁入GENERIC。
- 物理PickLift只重跑旧动作链；具名RobotCommand携物本轮验证的是同状态允许
  接触、携带几何、joint/Cartesian拒绝分支，**没有新双臂真实抓物episode**。
- 原始SceneSmith到全部ignored专家文件的全链重建、本轮全新clone安装、任意
  工作空间Cartesian精度、第二完整机器人均未重新验收；清楚保留限制。
- 没有恢复用户删除的旧文档，没有删除旧失败日志或大产物。新增JSON/CSV证据
  保存在Git忽略目录；不新增OBJ/LFS跟踪。

### 15.5 上轮初步提交方案（已由§16.3替代，不执行）

已提交模型修复b433ee6不重复提交，不重写v0.1/v0.2。P0前未提交工厂/Cartesian
与P0—P7及本轮收尾在同一批文件叠加；原未跟踪文件缺少逐轮Git快照，不能声称
可按历史回合精确拆分。当前`__init__`公共导出、Runtime、适配器和示例互相依赖，
不按七项清单强拆不可运行的提交。推荐两份可审查的变更：

1. **`Unify online runtime and complete named mobile manipulation handoff`**：
   全部实际运行代码与配套测试/自包含fixtures：`src/online_manipulation`相关改动、
   旧`zerith_online_env`和CLI、servo配置、独立工厂/策略/通用Runner、双臂/两底盘/
   导航/组合动作/携物安全/TAMP；`environment.json`及相关模型fixtures；包含P0前
   原有Cartesian/工厂工作和本轮修复。生产示例与公共API一起提交，以免导出或入口
   指向未提交模块。已有测试和本轮针对性测试随代码，不另拖成无验证代码提交。
2. **`Document versioned contracts and migration acceptance evidence`**：README、
   GENERIC、Quickstart、mobile需求/架构/progress、PickLift README，保留三份旧文档
   删除；`BASE_MODE_IMPLEMENTATION_AUDIT`与独立`audit_base_modes.py`可同组，标注
   是验证工具而非运行功能。记录命令与结果索引，不提交ignored大日志/HTML/OBJ/.venv。

如果之后希望拆出独立导航提交，需要同时按hunk调整公共导出和依赖并在每个中间
commit验证，不能直接按目录暂存。本轮只给方案，不作暂存、提交、推送或tag操作。

上轮记录（最终工作树验证见§16.1）：迁移修复完成，150项全量快照和最后24项具名观测/携物针对性回归
均退出0。`git diff --check`通过，README/GENERIC/Quickstart/PickLift README的
本地文件链接无缺失；暂存区为空，HEAD及两个tag不变，三份既有删除仍为D。
记录以上验证范围，不把无关未验收能力算作完成。后续只按用户确认决定提交。

## 16. 提交前最终记录核对（2026-09-08，未暂存）

本节取代§15.5的初步分组；本轮只修改README和本progress，不修改实现、
测试、配置或模型。所有原有修改和三份文档删除保留，不提交、不推送、不改tag。

### 16.1 测试时序与最终代码状态

1. 上轮先启动全量测试，`full_tests_final.log`最终为150项、578.788s、OK。
   但运行期间（12:04:42 +08:00）仍修改了固定Zerith适配器的具名观测、
   缺省夹爪异常类型及对应测试断言。测试发现/导入早于该修改，因此不能仅凭日志
   最后写入时间12:05:52就认定这150项覆盖最终文件。
2. 最后24项针对性回归覆盖上述最终适配器修改：
   `named_fixed_observation.log`，85.998s、OK，模块为
   `test_zerith_robot_adapter`、`test_online_environment`、
   `test_picklift_command_contact`。它们与全量重叠，**不是新增24项，不统计为174项**。
   TAMP实际base pose断言另由`tamp_pose_final.log`的1项回归覆盖
   （22.652s、OK）；同样不相加。
3. 为覆盖最终工作树，本轮重新执行已有完整测试集，不新增测试、不重复底盘机制实验：

```bash
PYTHONPATH=tests:. MPLCONFIGDIR=/tmp/mpl .venv/bin/python -B -m unittest discover -s tests -v
```

工作目录为仓库根；stdout/stderr由`tee`保存到
`output/closure_audit/full_tests_submission_ready.log`，shell启用`pipefail`。
最终结果：**150 tests / 581.234s / OK，进程退出0**。这是最终实现工作树的
完整测试结果，替代上轮较早快照作为本次提交准备依据，不与此前150/24/1相加。

测试启动后只修改文档。174个Git可见Python/配置/模型文件（该数值是文件数，
不是测试数）的SHA256清单聚合指纹在启动及结束复核时均为：
`ba5985d5c181f57fa5efedee614b38e684c13244afbdbdc49c6491f53b23a3e5`。
范围为tracked/untracked、非ignored的.py/.json/.yaml/.yml/.xml/.urdf/.sdf/.toml
及requirements.txt；不把文档变更或ignored产物当作实现变化。

### 16.2 TAMP正文与recap统一结论

**实现完成、针对性验证完成的是TAMP执行交接与执行前自动状态同步，不是完整TAMP系统。**

- `examples/online_manipulation/tamp_execution.py::execute_validated_joint_goal`
  每次使用`env.get_planning_query()`与最新`env.observation`，即使兼容参数传入旧
  query/observation也不用于执行判定。调用方无需自行记住刷新。
- 当前实际底座、关节及自由物体状态进入独立规划context；当前configuration及
  完整直连edge通过后才发送计算后的绝对关节目标。不是只更新时间戳。
- `tests/test_closure_contracts.py::test_tamp_refreshes_after_actual_mobile_motion`
  已在实际wheel_dynamic移动后核对base坐标、拒用旧快照和执行前刷新。
  `tamp_pose_final.log`及`tamp_pose_sync_metrics.json`的数值见§15.3。
- README原正文“intended boundary / caller synchronizes”以及传旧query示例
  已改为实际自动同步行为；本文recap与GENERIC/Quickstart/架构文档保持一致。
- 全局路径搜索、完整TAMP求解器、任意第三方环境规划能力不在本轮验收范围；
  不因该handoff通过而宣称这些能力完成。

### 16.3 两组明确提交清单（仅准备）

按当前`git status --short --untracked-files=all`逐项归属：79个路径全部覆盖，
无遗漏或额外未变更路径。下列73/6为主要文件归属；两处使用文档的审计链接
hunk按第二组处理，具体见后文。不执行任何暂存命令。

**第一组：运行链、接口、配置、示例、测试及必要使用文档。**

建议提交说明：
`Unify online runtime, named robot contracts and reusable examples`

公共导出、RobotAdapter/Runtime、配置迁移、旧CLI薄转发、Task携物语义、
单/双臂Cartesian、两底盘、导航、TAMP同步及所有配套测试/fixtures是耦合整体。
包含P0前已有工厂/Cartesian工作，不只提交最近收尾。需求/架构和用户运行文档
随实现提交，不把必要使用说明全部拖到审计提交。

```text
README.md
docs/GENERIC_ONLINE_EXAMPLE.md
docs/QUICKSTART_ONLINE_ENV.md
docs/mobile_manipulation_architecture.md
docs/mobile_manipulation_requirements.md
examples/online_manipulation/cartesian_client.py
examples/online_manipulation/cartesian_policy.py
examples/online_manipulation/example_policies.py
examples/online_manipulation/minimal_setup.py
examples/online_manipulation/mobile_public_api_client.py
examples/online_manipulation/mobile_smoke.py
examples/online_manipulation/navigate_demo.py
examples/online_manipulation/navigation_manipulation.py
examples/online_manipulation/pick_lift_demo/README.md
examples/online_manipulation/pick_lift_demo/__init__.py
examples/online_manipulation/pick_lift_demo/minimal_setup.py
examples/online_manipulation/pick_lift_demo/policy.py
examples/online_manipulation/public_api_client.py
examples/online_manipulation/run_online.py
examples/online_manipulation/tamp_execution.py
examples/online_manipulation/validate_mobile_navigation.py
models/mobile_scene/empty.dmd.yaml
models/mobile_scene/floor.sdf
models/mobile_scene/obstacle.dmd.yaml
models/mobile_scene/obstacle.sdf
models/mobile_scene/package.xml
models/runtime_fixture/contact_block.sdf
models/runtime_fixture/contact_scene.dmd.yaml
models/runtime_fixture/package.xml
models/runtime_fixture/urdf/two_joint.urdf
models/zerith_pick_eval/environment.json
scripts/run_zerith_online_example.py
src/online_manipulation/__init__.py
src/online_manipulation/actions.py
src/online_manipulation/adapters/description.py
src/online_manipulation/adapters/zerith.py
src/online_manipulation/adapters/zerith_dual.py
src/online_manipulation/adapters/zerith_mobile.py
src/online_manipulation/base.py
src/online_manipulation/contact.py
src/online_manipulation/environment.py
src/online_manipulation/model.py
src/online_manipulation/navigation.py
src/online_manipulation/navigation_geometry.py
src/online_manipulation/observations.py
src/online_manipulation/planning.py
src/online_manipulation/policies.py
src/online_manipulation/protocols.py
src/online_manipulation/runner.py
src/online_manipulation/runtime.py
src/online_manipulation/sensors.py
src/online_manipulation/specs.py
src/online_manipulation/tasks.py
src/zerith_online_env.py
src/zerith_servo_config.py
tests/test_architecture_boundaries.py
tests/test_cartesian_handoff.py
tests/test_cartesian_pose_action.py
tests/test_closure_contracts.py
tests/test_description_runtime.py
tests/test_dual_runtime.py
tests/test_episode_runner.py
tests/test_generic_online_example.py
tests/test_mobile_cameras.py
tests/test_mobile_composition.py
tests/test_mobile_navigation.py
tests/test_online_api_contracts.py
tests/test_online_environment.py
tests/test_online_integration_examples.py
tests/test_picklift_command_contact.py
tests/test_picklift_demo_example.py
tests/test_zerith_cameras.py
tests/test_zerith_robot_adapter.py
```

**第二组：独立审计工具与其余交接文档，保留既有删除。**

建议提交说明：
`Record migration acceptance and base-mode audit; retire superseded docs`

```text
docs/BASE_MODE_IMPLEMENTATION_AUDIT.md
D docs/ONLINE_ENV_ARCHITECTURE.md
D docs/ONLINE_ENV_PROGRESS.md
D docs/ONLINE_ENV_REQUIREMENTS.md
docs/mobile_manipulation_progress.md
examples/online_manipulation/audit_base_modes.py
```

`audit_base_modes.py`仅供机制审计，不被运行核心/策略/普通示例或测试导入。
三份旧ONLINE_ENV文档的必要契约和版本化历史由第一组的GENERIC/Quickstart/
mobile架构文档承接后，再保留删除；不恢复旧文件。

**仅文档跨组hunk：** README、Quickstart中指向第二组新progress/audit文档的
链接随第二组加入；其余运行说明随第一组。这样第一组既可运行也没有指向尚未
提交报告的断链。无需拆分强耦合代码、临时兼容桩或复制配置来凑两个提交。

**原有工作与排除项：**

- 以`output/mobile_manipulation/p0/status_before.txt`及§14的历史/工作树映射核对，
  P0前已有未提交工厂、Cartesian、配置工作与后续迁移/收尾叠加的文件整体保留。
  未跟踪文件缺少逐轮Git快照，不虚构精确作者或逐轮hunk来源。
- 已提交夹爪/模型修复`b433ee6`在HEAD历史中，不重复作为新改动；
  既有删除仍为D。第一组与第二组覆盖的是当前未提交差异，不重写已有提交。
- 不纳入`output/`、`outputs/`、`/tmp`日志、HTML、PNG、CSV、专家IK生成物、
  OBJ、SceneSmith网格、`.venv`或缓存；审核日志继续留在ignored目录作为本机证据。
  当前清单没有无关改动或大体积输出，复核时最大文件约55KB（progress追加清单
  后略增，仍是文本），不是把目录下所有文件一并加入。
- 本轮仅准备文件/hunk范围；实际提交需后续授权，不能据此声称已经形成两个commit。

### 16.4 最终recap

最终工作树既有全量150项通过；TAMP自动当前状态同步与执行交接实现/验证完成，
完整TAMP规划系统明确范围外。没有为本轮核对新增功能或修改实现。
两组79个路径及审计链接hunk已明确归属；暂存区为空，
HEAD保持`b433ee6dd5b20017fd141adbbb5910347526c3a2`，v0.1/v0.2的tag对象及解引用
commit均未变，三份旧文档删除保留。`git diff --check`通过。
未暂存、提交、推送或修改tag；测试日志是ignored本地产物，不属于提交清单。

## 17. v0.3提交与发布收口（2026-09-08）

用户在§16核对后明确授权本地提交、推送用户fork、创建并推送annotated
`online-env-v0.3`。因此前文“不提交/不推送/tag只读”保留为各阶段历史约束，
不再代表本次发布授权；v0.1/v0.2依然不可移动。

### 17.1 两组提交与版本提交

- `a22da6c7c906c2f080efc3490f54aacfd2d00d3b`：第一组73个显式路径，运行链、
  共享Runtime、具名接口/配置、示例、测试/fixtures和必要文档一起提交。
- `7f33fc7`：第二组6个归属路径及README/Quickstart的报告链接hunk，保留
  三份旧文档删除。独立审计工具不混入生产运行链。
- 后续版本收口提交：`PUBLIC_API_VERSION`及唯一对应测试断言0.2→0.3；
  README/Quickstart/GENERIC/架构映射更新，新增中文
  `RELEASE_ONLINE_ENV_V0.3.md`。没有新增控制模式、修改抓取/物理/相机参数。
  该最终commit由`online-env-v0.3^{commit}`标识，不在自己的文件内硬编码自身hash。

每组均使用明确文件清单暂存、检查cached diff/stat及空白、核对内容归属和大小；
第一组仅延后新审计报告链接，避免不可运行代码拆分和中间断链。输出、临时文件、
OBJ、环境目录均未暂存，原先已提交的b433ee6模型修复通过历史继承。

### 17.2 验证复用与版本针对性检查

两组提交后实现/配置聚合指纹仍为§16.1的
`ba5985d5c181f57fa5efedee614b38e684c13244afbdbdc49c6491f53b23a3e5`。
版本收口的Python差异仅为公开版本字符串和测试期待值；没有实现变化或合并
冲突，因此复用最终全量**150项/581.234s/OK**，不重复大规模物理实验。

本次实际命令（仓库根起步，使用本地既有.venv）：

```bash
export REPO_ROOT="$(pwd)"
export PYTHON="$REPO_ROOT/.venv/bin/python"
PYTHONPATH=tests:. MPLCONFIGDIR=/tmp/mpl "$PYTHON" -B -m unittest tests.test_online_api_contracts -v
"$PYTHON" -B scripts/convert_zerith_for_drake.py --check
cd /tmp
PYTHONPATH="$REPO_ROOT" MPLCONFIGDIR=/tmp/mpl "$PYTHON" -B "$REPO_ROOT/examples/online_manipulation/public_api_client.py" --repository-root "$REPO_ROOT" --seed 0
```

结果分别为：

- 14项/0.002s/OK，公开版本断言0.3；这是重叠针对性测试，不加成164项。
- `Check passed`，源commit `ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`，
  OBJ=43、源引用70、派生引用65、collision=53。仅check，本次没有重新生成模型。
- 仓库外客户端退出0，实际stdout：

```json
{"seed": 0, "action_type": "HoldAction", "simulation_time_s": 0.10000000000000002, "joint_count": 9, "object_names": ["portable_object"], "reward": 0.0, "terminated": false, "truncated": false, "action_status": "accepted"}
```

本次没有重新做全新.venv安装、移动底盘机制实验或PickLift物理episode；
已有150项全量、固定277步及底盘机制证据按对应阶段归属复用。

发布文档检查：Quickstart的15个bash块、发布说明的2个bash块均通过`bash -n`；
README/Quickstart/GENERIC/发布说明的本地文件链接均存在。首次对README所有
bash块做语法检查时，旧研究脚本模板的`<x>`占位符触发语法错误；已明确标记
这些为参数模板，发布smoke以Quickstart完整命令为准，不声称模板可直接执行。

### 17.3 远端基线与发布约束

发布前只读核验：当前分支`dev/wzh`；origin fetch仍指向作者仓库，**push URL**
为`git@github.com:Zihan-W/scenesmith-simple-planner-eval.git`。
只向此用户fork推送`dev/wzh`及v0.3指定tag，禁用隐式followTags，不force push。
初次检查远端dev/wzh为682ba7c，是本地HEAD祖先，无远端独有提交需合并；
本地和远端均无v0.3。

v0.1保留对象`2781df041d2382d8d9d3680e1edda321e3b81403`，解引用
`25eb9a40586c6234ae71b87b8ac4c6629702d0f6`；v0.2保留对象
`d80d1b41b329b6c047b67b3b18f62f143d6f4c74`，解引用
`682ba7c1168c49d2be4e71d7ba3d9fd48308d082`。

发布门禁：正常推送分支后在最终版本提交创建annotated v0.3并显式推送；
若远端已出现同名tag则不覆盖。最终核对分支commit、tag对象及解引用commit、
旧tag不变和本地工作树；结果由发布回报及远端Git refs给出，不伪造提前成功记录。
