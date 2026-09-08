# Eval 结构整理执行记录

## M0：基线（2026-09-08）

- SS：dev/wzh，d1a2a2580d1003e4fa952a7bb3ecaf70fc00f413；原有未跟踪 AGENTS.md、examples/ 保留。
- EV：dev/wzh，221c7b8ae4e2d7f4e2842941051a02d9feb7f7e0；原有 README 和 noninteractive planner 文档修改、四个旧交互入口删除保留；已有方案文档未跟踪。
- 用户批准 M0—M4；版本化小型专家输入，大资产仍由 SCENE_ROOT 提供，free/furniture_welded 显式选择。M5只准备清单。
- 不提交、不推送、不改tag，不删除旧输出，不改SS产物。测试使用独立临时缓存/输出根。
- M1首先处理几何级地面：导航地图、运动学动作边和轮地接触许可共用同一选择器；真实A场景必须有墙阻挡证据。

## 阶段状态

| 阶段 | 状态 | 证据 |
| --- | --- | --- |
| M0 | 完成 | Git/AGENTS基线已核对 |
| M1 | 实现与验证完成 | A 混合 body 地面/墙、原始上游迁根、缓存再迁根/哈希、单房间实际加载；B 3 张缺失纹理 |
| M2 | 实现与固定基准验收通过 | 版本化 IK/覆盖/profile；空新 output 的固定 PickLift 277 步成功 |
| M3 | 实现与针对性验收通过 | 受信任外部 Policy/evaluator 两次 reset；新旧工厂对齐；可选规划不禁用核心检查 |
| M4 | 实现与验证完成 | camera/AABB/旧 CLI 转发；分层依赖；首次157通过后补修实际DMD写回，最终158项/637.878s通过 |
| M5 | 清理准备完成，未删除输出 | 本文末尾逐项列消费者、可删副本和建议保留证据 |

## 针对性执行记录（不是测试数量累加）

- `MPLCONFIGDIR=/tmp/ev-mpl .venv/bin/python -B -m unittest tests.test_contact_policy tests.test_mobile_navigation -v`：6 项通过（37.772s）。初次使用了 Drake 1.49 不存在的 `plant.GetSceneGraphInspector`，已改为显式传入 SceneGraph inspector；失败没有隐藏。
- `MPLCONFIGDIR=/tmp/ev-mpl PYTHONPATH=.:tests .venv/bin/python -B -m unittest test_closure_contracts test_architecture_boundaries test_picklift_demo_example test_generic_online_example -v`：18 项通过（22.937s），包括真实移动后的 TAMP 查询同步。单独以 `tests.test_closure_contracts` 导入曾因旧测试采用顶层兄弟导入失败，正确运行方式为上述 PYTHONPATH 或 discover。
- 同样方式执行 `test_mobile_navigation test_mobile_composition test_picklift_command_contact`：7 项通过（123.462s）；执行 `test_generic_online_example test_zerith_camera_calibration`：9 项通过（37.875s）。并非把这些组相加成全量数量。
- `SCENE_ROOT=.../2026-09-02/10-01-49/scene_000 ... -m unittest test_structure_migration -v`：首轮 5 项通过（47.200s）。A 房间 body 有 1 floor+5 wall collision，floor 选择 1 个，wheel/support 许可 6 对 exact geometry；导航提取1913个障碍，北墙穿越边被拒绝。B 精确报告 Wood094 Color/NormalGL/Roughness 三张缺失图片，未替换；B 视觉验收仍未完成。
- 外部 evaluator 测试把示例复制到临时仓库外目录，用公开 API/CLI，seeds 7/8 均3步 `custom_hold_confirmed`，证明 reset 清除 evaluator 状态；这是 HOLD 接口验收，不是抓取测试。

## 独立缓存和 output 的实际运行

```bash
MPLCONFIGDIR=/tmp/ev-mpl .venv/bin/python -B -m src.online_manipulation.experiment experiments/minimal.json --repository-root /root/workspace/scenesmith-simple-planner-eval --cache-root /tmp/ev-migration-minimal-cache --output-root /tmp/ev-migration-minimal-run
MPLCONFIGDIR=/tmp/ev-mpl .venv/bin/python -B -m src.online_manipulation.experiment experiments/picklift.json --repository-root /root/workspace/scenesmith-simple-planner-eval --scene-root /root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000 --cache-root /tmp/ev-migration-pick-cache --output-root /tmp/ev-migration-pick-run
```

- minimal：10步、1秒，NullTask正常以max_steps结束；未读取专家文件。该旧minimal fixture的自由floor/box初始有约5cm环境间穿透，**它只用于API smoke，不作为稳定接触或抓取物理基准**；本轮未改其动力学或模型。
- PickLift：seed500、277步、27.7s、`lift_held`，lift=0.1028551391m，hold=3.1s，双指contact=true、support=false，无力矩饱和；与既有固定基准277步一致。不是扰动鲁棒性验收。
- 产物：上述 `/tmp/ev-migration-*-run` 的 `resolved_config.json`、benchmark JSON/CSV、episode summary/trace。源 A 只读；新增缓存约262MiB。没有删除旧 output 来造空目录。
- `models/zerith_pick_eval/pick_lift_calibration.json` 已迁至 `experiments/inputs/pick_lift/`，所有默认消费者已调整；旧 Git 版本可恢复原小文件。未删除大输出或运行资产。

## 最终验收证据及执行顺序

1. 针对性测试之后，首次全量 `unittest discover -s tests -v`：**157项/635.198s/OK**，
   日志 `/tmp/ev-structure-full-tests.log`。这是当时工作树的结果，不与前述小组相加。
2. 新入口再跑固定PickLift，`/tmp/ev-structure-final-pick`，277步/成功，保存HTML和DMD。
   随后的**真实重载**发现 `yaml-cpp ... end of map not found`：缓存使用多行translation，
   旧finalizer只替换首行，留下3条旧列表行。此前测试仅覆盖行内向量，故全量通过不足以证明真实写回可用。
3. `dmd_finalizer._replace_pose_block` 改为替换整个translation字段，处理同缩进/更深缩进列表；
   `test_cached_block_translation_reloads_in_drake`覆盖两种格式，4项finalizer针对性测试通过。
   不调整物理、相机外参或抓取数值；只修文件格式衔接。
4. 最终实现再次跑全量，日志为 `/tmp/ev-structure-final-full-tests.log`，**最终状态见文末**。
   与此同时用新cache/output重新跑PickLift，不能把修复前的无效DMD当成最终交付。

### 固定PickLift与真实DMD写回

实际从 `/tmp` 执行：

```bash
MPLCONFIGDIR=/tmp/ev-mpl PYTHONPATH=/root/workspace/scenesmith-simple-planner-eval /root/workspace/scenesmith-simple-planner-eval/.venv/bin/python -B -m src.online_manipulation /root/workspace/scenesmith-simple-planner-eval/experiments/picklift.json --repository-root /root/workspace/scenesmith-simple-planner-eval --scene-root /root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000 --cache-root /tmp/ev-structure-roundtrip-pick-cache --output-root /tmp/ev-structure-roundtrip-pick --meshcat --record-html --write-final-dmd
```

- seed500：success=true，lift_held，277steps，27.7s；红盒抬升0.1028551391421132m，保持3.1s；
  双指接触、无支撑接触、无unexpected_target_contacts，无力矩饱和观测。
- 最大跟踪误差0.08268214445471378，未声称所有末端动作具有相同精度。
  summary的minimum_collision_distance为null，是有限影响范围下未给出数值，不解释成“无限安全距离”。
- 最终位置 `[2.00056152550464,2.772513642720439,0.6135990339712619] m`。
  `/tmp/ev-structure-roundtrip-check.py`从新DMD重新构造Drake场景：位置误差0，旋转矩阵最大误差
  `3.122502256758253e-17`，1804个源依赖文件哈希保持不变；结果 `/tmp/ev-structure-roundtrip-metrics.json`。
- 完整产物：`/tmp/ev-structure-roundtrip-pick/episode_000_seed_500/` 下
  `simulation.html`、`summary.json`、`trace.csv`、`final.dmd.yaml`；根目录有resolved/benchmark JSON/CSV。
  final.dmd仍需同一cache内的package闭包，不能孤立搬走DMD后宣称独立场景包。
- 这是**固定基准回归**；没有新做扰动鲁棒性实验。历史3/3、2/3属于旧版本，不移植成本轮数字。

### 场景与几何证据

- A：完整源场景与其闭包复制到新的source root，再准备独立cache；缓存再次整体迁根并重新注册package，
  目标model/body和世界位姿正确。测试逐文件校验源哈希。无API key、生成服务或原SS Python依赖。
- A房间body同含floor和5个wall；仅1个floor geometry产生6对wheel/support精确许可。
  导航地图1913个障碍，北墙边 `(3.5,4) → (3.5,6)` 不可通过。
- 更强反例：隔离规划模型初始底座 `(3.5,5.2855,0.1808)`，左驱动轮对真实
  `north_wall_collision` 距离 `-0.04575m`，接触许可False。脚本
  `/tmp/ev-structure-wall-check.py`、指标 `/tmp/ev-structure-wall-metrics.json`。
  这是故意碰墙的静态诊断，不是合法执行初态，不以瞬移冒充运行。
- A单房间 `room_living_room/scene_states/final_scene/scene.dmd.yaml`：用dmd_relative和
  sibling scene_state.json准备 `/tmp/ev-structure-room-cache`，原/缓存均实际解析，
  box_0世界位置 `[-1.7509344495019237,0.1989479692670291,0.4947917020709052]` 一致；
  不再从JSON重复叠加house房间偏移。地面为 `room_geometry::room_geometry_body_link/floor_collision`。
- A的free变体也在新cache/output中运行NullTask+Hold两步，0.2s/max_steps，产物
  `/tmp/ev-structure-free-smoke`。只是加载/推进证据，不是自由家具全场景稳定性验收。
- B精确缺3张Wood094纹理，依赖检查在模拟前报错；没有资源、没有自动替换，**B视觉验收未完成**。

### 替换接口、相机及继承能力

- 仓库外 `/tmp/ev-structure-external-client`：复制公共API示例为my_components.py，配置
  Policy/evaluator各为该模块factory；seeds7、8分别3steps/0.3s/custom_hold_confirmed。
  两次reset分别从0计数，产物 `/tmp/ev-structure-external-output`，执行日志在外部目录run.log。
  未访问Drake Context、私有索引；Task仍管理接触规则，evaluator只替换判定。
- 禁止scenesmith/openai/agents/torch/trimesh/manipulation/networkx导入的独立子进程，
  实际完成minimal reset/step，动作被接受。证明可选依赖不进入state-only路径；不是新venv安装验收。
- 两种mobile profile均reset/step且暴露左右夹爪；既有全量继续验证双臂Cartesian/组合动作、
  携物joint/Cartesian等价检查、移动后的TAMP同步、导航到达/取消/失败及相机hardening。
  本轮没有重复大规模双底盘驱动力隔离实验，没有新增导航或操控模式。
- `/tmp/ev-structure-camera-review/calibration`：三相机各3目标RGB/depth/label对齐；
  最大像素误差0.757635px，最大深度误差2.48e-7m，全部通过。
- 同目录 `picklift_pregrasp`：left_wrist_target_visible=true（4097像素），
  head_workspace_visible=true（咖啡桌6409像素）；三套RGB、米制depth及彩色depth/label保留。
  neck pitch=0，外参/内参/缓存语义未改；内参仍是simulation camera intrinsics。
- `.venv/bin/python -B scripts/convert_zerith_for_drake.py --check`通过：43OBJ、原模型70处引用、
  派生65处mesh引用/53个collision。不是重新改上游模型；未删除运行所需OBJ。
- 全量后再经新profile启用三相机，双臂fixed/minimal及wheel_dynamic/mobile均reset/step；
  三帧均240×320×3、timestamp=0.1s，记录 `/tmp/ev-structure-profile-camera-metrics.jsonl`。
  这是配置接线检查，空场景背景图不当作视觉可用性证据；可见性以真实A的前述图像为准。
  首次诊断打印误用 `obs.base.mode`（base实际是mapping）而失败，改为公共reset_info的base_mode重跑通过，
  未修改Environment或公共observation语义。

## M5：清理准备（没有执行删除）

判断依据：检查正式src/recipes、scripts/tools、examples及当前使用文档的读取位置，
不凭文件名年代判断。通用参数允许用户显式指定任意文件，因此“无默认消费者”不等于
能够证明任何人的私人命令都不使用它；旧手动标定工具仍可显式读相邻JSON。

| 输出/输入 | 已迁移与消费者结论 | 后续可选处理 |
| --- | --- | --- |
| `output/zerith_pick_eval/pick_home.json` | 与versioned输入逐字节相同，SHA256 `6d701815b4763127ecc31659292663d82f940e095cbde4f0fb186f01963cff89`；新默认专家只读experiments | 可删重复副本；使用旧PICK_ARTIFACT_ROOT的私人命令须先换路径 |
| `output/zerith_pick_eval/pregrasp_ik.json` | 同上，SHA256 `67f7884f4f0e396f3d76c991529165b1db5ed279043cdf9ef628494ec6b03286` | 可删重复副本，保留版本化输入 |
| `output/zerith_pick_eval/zerith_pick_eval.dmd.yaml`、`task_metadata.yaml` | 新流程用源包+版本化覆盖重新准备cache，旧metadata不作为运行事实；旧标定README仍显式演示此路径 | 退出日常路径；不再重做旧标定链时可删，不能一边使用旧手工命令一边删 |
| `target_settle.json`、`blue_vase_settle.json`、`rail_postures.json`、`pick_home_dynamics.json`、`online_pregrasp*.json`、`roundtrip_pregrasp.*` | 无正式运行读取；主要为标定/旧验收输出，工具生成相应结果 | 不需复查旧标定时可删；不是新策略输入 |
| 各历史episode的HTML、重复trace/summary | 无正式运行输入依赖；部分旧版本文档引用结果 | 选择少量summary/关键CSV后可清理重复回放，不要求永久归档全部 |
| `output/mobile_manipulation/base_mode_audit_01`、`base_sampling_audit_02`、`final_parked_*` | 旧机制/差分/停车证据，交接文档仍引用 | 建议保留关键JSON/CSV和少量对照图，不必保留每次HTML |
| `output/online_env_final_audit`、`closure_audit` | 历史固定/扰动/版本测试证据，不是本轮输入 | 可仅保留批次summary和最终测试日志，注明对应版本 |
| `.venv`、models/zerith_drake/meshes、SS完整源包、small_red_box.sdf | 运行或模型重建依赖 | 不列入本轮可删除输出 |
| 新cache的packages闭包 | 日常加载、相机渲染以及final.dmd仍引用 | episode不再需要且可重建时整根清理；不能只删其中纹理/mesh |

最少建议保留：本轮最终全量日志、一个固定PickLift的summary/trace/HTML、round-trip和墙体指标、
三相机标定/真实场景RGB及metrics、仓库外两episode摘要；旧结果只保留区分两种底盘及版本历史的少量证据。
本轮没有复制巨型HTML到Git，没有移除1537个已跟踪示例资产，没有删除任何旧output目录。

## 剩余限制（不把范围外当作失败，也不把缺证据说成通过）

- B缺纹理，视觉未验收。最小旧fixture的环境间重叠只标注、不借此次迁移改动力学。
- 新依赖清单使用本机已有venv验证、并阻断可选导入；**未在新venv联网重装**。
- 任意外部机器人/学习Policy仍需符合公开动作/传感器能力契约；无第二个真实机器人实测。
- 缓存采用显式新根，不提供自动复用/下载/垃圾回收；源SDF嵌套或其他未知导出格式不声称全覆盖。
- 固定专家需要匹配的源/机器人/目标资产与初态；换场景不能免除绑定和策略标定。
- 不新增VLA/VLM、完整TAMP、PLACE、移动携物、双臂协同抓同一物体或导轨动力学。

## 最终测试状态与工作树交接

最终命令（在eval仓库根目录执行；路径是本机实际执行记录）：

```bash
set -o pipefail
SCENE_ROOT=/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000 MPLCONFIGDIR=/tmp/ev-mpl .venv/bin/python -B -m unittest discover -s tests -v 2>&1 | tee /tmp/ev-structure-final-full-tests.log
```

**158项，637.878s，OK**。包含新增的缓存block-translation写回回归，真实A测试未skip。
它覆盖最后一次实现修复；之后只做文档与额外实际产物核验，没有再改控制/场景处理实现。
不把首次157和最终158相加，也不把针对性测试数量累加为验收数量。

范围映射：

- 实现且验证：有限profile/解析记录；输入迁移；只读依赖准备/迁根；geometry级地面和真实墙；
  Policy/evaluator外部factory；原Runtime及双臂/双夹爪/两种底盘/相机/携物/TAMP同步继承；真实DMD写回。
- 证据缺失并明确保留：B视觉、新venv联网安装、任意新真实机器人/新策略的物理效果。
- 范围外：资产下载系统、自动缓存复用回收、新模型策略、完整TAMP/移动协同抓物等。

变更归属（尚未暂存）：

1. 正式实现/必需输入：src场景适配、geometry选择、experiment/assembly/evaluation、recipes、TCP/AABB、
   finalizer格式修复；experiments和旧environment.json引用；新增针对性测试及既有测试导入迁移。
2. 工具/调用消费者：旧CLI、examples薄转发、tools/calibration和tools/audit、独立外部evaluator例子。
3. 文档/依赖清单：本方案/进度、Quickstart/GENERIC/README及示例README、分层requirements。
4. 既有工作：原README与noninteractive planner文档修改、run_experiment和3个旧交互入口删除均保留；
   不将其冒认为本轮新删除。此次小calibration文件是有明确目的地的迁移，不是丢弃。

SS的HEAD/工作树保持基线；eval HEAD仍为221c7b8ae4e2d7f4e2842941051a02d9feb7f7e0。
Git索引无暂存内容、diff --check通过；没有提交、推送、tag操作或批量清理。

## 检查点准备（后续用户授权，本节取代此前“本轮不提交”的操作限制）

用户批准先创建本地检查点，再继续入口精简。实现未改变，复用最终158项/637.878s/OK，
不重复跑全量。显式核对每条暂存路径，不纳入OBJ、output、venv或大场景资产。

- 本轮M0—M4：正式实现、配置/小型专家输入、测试、工具迁移与必要文档一起提交。
- 原有README与noninteractive planner说明、run_experiment.sh和三个旧交互脚本删除：
  彼此为同一旧流程退役的说明/调用依赖，且当前文档沿用这组变更，因此一并保留在检查点；
  不将它们计作本轮新实现。没有发现其余无关已修改文件。
- models中的小calibration迁移到experiments属于重构，不恢复旧重复数据来源。
- 后续可能涉及的六份未跟踪历史快照已在仓库外 /root/workspace/eval-checkpoint-CdmzSR1W 备份，
  保留相对路径和说明；当前策略IK/标定另由Git检查点保存。
- 缓存、普通日志/HTML、OBJ、虚拟环境和上游大场景不整批备份、不删除。
- 本次只授权本地检查点；不推送、不改tag。之后的入口精简与退役变更单独保留工作树，
  不悄悄并入这个已验证检查点。
