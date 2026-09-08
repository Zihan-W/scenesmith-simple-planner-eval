# SceneSmith → Online Eval：面向实际使用的结构整理方案

日期：2026-09-08。状态：**用户已批准 M0—M4 实施，执行证据见 EVAL_STRUCTURE_PROGRESS.md**。

实施补充：已验收专家 JSON/标定/显式覆盖版本化；大场景保留外部 SCENE_ROOT；free/furniture_welded显式选择；短实验配置引用有限profiles，完整来源写resolved_config.json。正式组装不依赖examples/scripts。几何级地面同时约束导航和轮地接触，真实A场景验证；B缺失纹理只验证错误。公开evaluator支持外部factory且不改Runtime。M5仅清理准备，不删除。以下调查事实保留为实施前基线，不把建议段落视为完成证据。

本轮仅新增本文；不改变实现、配置、上游产物、既有修改或删除，不运行场景生成、物理实验，不暂存、提交、推送或修改 tag。文中的目标目录、配置字段与新增入口均是建议，不代表当前已可执行。

## 0. 调查基线、证据与限制

下文 `SS` 指 `/root/workspace/scenesmith`，`EV` 指 `/root/workspace/scenesmith-simple-planner-eval`。表格中的 `SS/`、`EV/` 是本次调查的路径缩写，不是要写入新代码的绝对路径。

| 项目 | 本地事实 |
| --- | --- |
| 适用指令 | 完整读取 `SS/AGENTS.md`；检查 `/AGENTS.md`、`/root/AGENTS.md`、`/root/workspace/AGENTS.md` 未发现文件；EV 未发现额外 AGENTS.md。遵循务实、小范围、保留 API 与明确失败原则。 |
| SceneSmith | `dev/wzh`，HEAD `d1a2a2580d1003e4fa952a7bb3ecaf70fc00f413`；未跟踪 `AGENTS.md`、`examples/`。 |
| Eval | `dev/wzh`，HEAD `221c7b8ae4e2d7f4e2842941051a02d9feb7f7e0`；本地 `online-env-v0.3` 解引用到该提交，公共 API 为 `0.3`。这里只读本地 refs，不将 remote-tracking ref 当作刚核验的远端状态。 |
| EV 已有工作树 | 修改 `README.md`、`scripts/plan_robot_waypoints_rrt_noninteractive.py`；删除 `run_experiment.sh`、`scripts/compute_grasp_config.py`、`scripts/plan_robot_waypoints_rrt.py`、`scripts/simulate.py`。本方案不恢复这些删除。 |
| 远端配置 | SS origin 为用户 fork，upstream 为 nepfaff；EV origin fetch 为 cohnt，push 为 `git@github.com:Zihan-W/scenesmith-simple-planner-eval.git`。本轮未 fetch/push。 |
| 机器人来源 | 两仓库 Zerith submodule 均为 `ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`；SS 的 Hunyuan submodule 在当前 checkout 状态为未初始化，不据历史安装对话认定可重新生成场景。 |
| 环境版本 | EV `.venv`：Python 3.11.13、Drake 1.49.0、NumPy 2.4.6、SciPy 1.15.3、trimesh 4.11.0、manipulation 2025.10.20、Pillow 12.3.0、PyYAML 6.0.3。SS 的 pyproject 要求 Python 3.11、NumPy <2、bpy 4.5.4，不能混用两个依赖环境。 |

调查覆盖本地历史，不仅最近提交：SS 的依赖/Hunyuan/API relay/Zerith 可视化提交分别是 `4eca5b8`、`b351868`、`401e949`、`d1a2a25`；EV 包括 v0.1 `25eb9a4`、相机与 hardening、v0.2 代码发布 `23d656c`、文档更新后的 v0.2 tag `682ba7c`、夹爪修复 `b433ee6`、共享运行链 `a22da6c`、审计文档 `7f33fc7` 和 v0.3 `221c7b8`。本轮不重新判断或修改旧发布历史。

上游只用于确认原始职责：[SceneSmith 上游](https://github.com/nepfaff/scenesmith) 是场景生成系统；[原始 eval 上游](https://github.com/cohnt/scenesmith-simple-planner-eval) 提供示例模型、抓取配置、路径规划与执行流程。本地 eval 已增加通用在线环境，不能按原始上游的目录含义删除新增能力。

实际执行的调查包括 `git status/log/submodule status`、`rg` 消费者检索、JSON/XML/DMD 静态解析、依赖文件存在性检查、已有日志抽查，以及一次无 API key 的公共 API 导入。没有重新运行动态验收。导入时 Matplotlib 在 `/tmp` 创建了临时配置缓存，没有写入仓库输出目录。

## A. 两仓库职责与实际场景产物契约

### A1. 推荐边界

```text
SceneSmith 生成/导出 → 只读 DMD + 资产包 + 场景元数据
                              ↓
eval 场景适配/显式实验覆盖 → ScenarioSpec
机器人定义 + 初态 + 控制 → RobotAdapter / RuntimeConfig
任务绑定 → Task       Policy/导航/TAMP → 公共 action
                              ↓
                 现有 make_env / DrakeRuntime
                              ↓
                  现有 runner / EpisodeResult
                              ↓
                可选离线 evaluator / 新 DMD
```

不把 SceneSmith Python 包作为默认 eval 场景加载依赖；不新建 Runtime、资产服务、万能 PluginManager 或新的场景格式。

### A2. 实际抽查范围与入口选择

SS 本地 outputs 只有一个完整场景，另有一个未完成目录，不能将同一场景的多个 checkpoint 当成多个独立场景。为扩大产物覆盖，另抽查 EV 内已有的原始餐厅场景。

| 样本 | 代表性实际文件与事实 |
| --- | --- |
| A：本地客餐厅 | `SS/outputs/2026-09-02/10-01-49/scene_000`；一个 `living_room`，44 个 DMD（包括渲染快照），仅一个房间 `final_scene`。 |
| A 的完整房屋 | `combined_house/house.dmd.yaml`：23 个 add_model、7 个 add_weld、16 个显式自由体初态；`house_furniture_welded.dmd.yaml`：23/18/5。两个入口的约束语义不同。 |
| A 的单房间 | `room_living_room/scene_states/final_scene/scene.dmd.yaml` 与同目录 `scene_state.json`；对象没有房间名前缀，DMD 使用本机绝对 `file:///root/workspace/...`。 |
| 未完成目录 | `SS/outputs/2026-09-01/13-34-59/scene_000` 有 package.xml，但没有 DMD 和 final_scene；不能作为可运行场景。 |
| B：本地上游餐厅样例 | `EV/models/21-20-10_cleaned/scene_000/combined_house/`，一个 `dining_room`；自由版 DMD 42/16/26，家具焊接版 42/25/17；包含复合小物体展开后的多个实例。不是本次 SS 生成的第二个场景，也不是多房间实测。 |

代码依据：`SS/scenesmith/agent_utils/house.py::HouseScene.assemble`、`_generate_combined_directive`；`room.py::RoomScene.to_drake_directive`；`experiments/indoor_scene_generation.py::_generate_room` 最终记录与房屋 assemble 调用。

日常 eval 优先显式选择 `scene_000/combined_house/` 中的最终 DMD，而不是扫描“最新文件”：

- 需要家具也能受力运动：选择 `house.dmd.yaml`。
- 当前固定 PickLift 基准：选择 `house_furniture_welded.dmd.yaml` 并显式应用任务派生覆盖；小物体仍自由，不代表所有物体可交互。
- 单房间也可加载，但要显式选择 room-local 入口及对应元数据；当前绝对路径须在派生缓存中重定位，不能修改原文件。
- `combined_house_after_furniture/after_wall_objects/after_ceiling`、`final_scene_pre_projection`、`scene_renders/*/scene.dmd.yaml` 是阶段/渲染状态，不自动升级为最终输入。
- 房屋和单房间不是同时加载的两层：加载两者会重复加入同一批物体与房间几何。

### A3. 完整依赖闭包，而不是只复制 DMD

```text
scene_000/package.xml（name=scene）
  └─ combined_house/house*.dmd.yaml
       ├─ package://scene/room_geometry/*.sdf
       │    └─ ../floor_plans/.../*.gltf → .bin / 纹理
       └─ package://scene/room_<id>/generated_assets/.../*.sdf
            ├─ visual *.gltf → buffers *.bin / images *.jpg、*.png
            └─ collision *.obj（部分声明 drake:declare_convex）
```

静态递归检查了 DMD add_model、SDF URI、PBR 图、glTF buffers/images、OBJ/MTL 引用；没有运行 Drake 加载或渲染，因此“文件齐全”不等于几何或渲染验收通过。

| 检查 | A 客餐厅 | B 餐厅样例 |
| --- | --- | --- |
| DMD 到图形/碰撞的唯一依赖路径数，不含入口 DMD/package/元数据 | 1800 | 1534，其中 3 个缺失 |
| 类型 | 19 SDF、1613 OBJ、33 glTF、127 bin、2 png、6 jpg | 22 SDF、1331 OBJ、33 glTF、145 bin、3 jpg |
| 已存在文件内容合计 | 约 255.33 MiB | 约 71.03 MiB |
| 完整房屋运行引用 | 本次闭包没有字面绝对路径、跨根资产或缺失文件 | DMD/SDF 可相对解析，但 floor.glTF 逃逸场景根目录 |
| 缺失依赖 | 未发现 | `/root/workspace/materials/Wood094_1K-JPG/Wood094_1K-JPG_{Color,NormalGL,Roughness}.jpg` |

B 的证据：`floor_plans/dining_room/floors/floor.gltf` 第 84/87/90 行使用 `../../../../../../../materials/...`。不能只搜索 `/root` 或 `file://` 就声称可迁移，也不能认为没有纹理只影响 Blender；机载 RGB 同样依赖 visual 资产。第一遍仅检查 SDF 时未发现缺失，继续追踪 glTF 后才发现这三张图。

另行递归检查两个 house_state.json：A 有 5 个绝对路径字符串，包含 `layout.room_materials.living_room.*.path` 指向旧运行 `SS/outputs/2026-09-01/22-12-19/.../materials/`；B 有 32 个，包含 `/home/ubuntu/scene-agent/outputs/2026-01-13/21-20-10/...` 的房间几何与复合成员资产路径。**DMD 依赖闭包可迁移，不代表 metadata 全部可迁移。** 核心加载不需要的历史材料字段保留为 provenance，不强行用它加载模型；VLM/复合对象解析需要的路径必须在派生元数据视图中按明确旧根→新根映射修复并校验，不能对所有字符串做无差别替换。不得覆盖原 JSON。

建议适配层枚举并验证传递依赖，检查 `..`、软链接、嵌套 DMD、URI、外部资源；未知格式明确报告未覆盖。第一阶段不要求打包任意 USD/MJCF，不自动联网补资源。缺失资产由配置显式提供；若复制到缓存，保留来源和精确引用改写记录。

运行文件集合分层：

1. Drake 场景加载：选定 DMD、package.xml、其传递依赖；机器人由另一包提供。
2. Task/语义绑定：对应 `house_state.json` 或 `scene_state.json`；原始字段要保留。纯 NullTask 且不需要语义绑定时可不读。
3. 特定导出/评测：`sceneeval_state.json` 是 SceneEval 格式，不是当前在线 env 必需文件；VLM 工具可能需要元数据中的额外 geometry_path、图像、房间信息，应按模块补充依赖。
4. 生成/调试：prompt、agent memory、日志、imagegen 原图、未选资产、Blender 回放、中间 state 不属于最小运行包。但不能按文件夹粗删：例如 `floor_plans` 和 `generated_assets` 中有最终 SDF 直接引用的资产。

### A4. 对象身份、可动性、坐标与元数据

| 信息 | 真实来源与适配要求 |
| --- | --- |
| 语义对象 ID | 房间 JSON `objects` 的 key 和 `object_id`，例如 `box_0`；`name=box`、`object_type=manipuland`、description 是语义，不直接当作唯一物理身份。 |
| 完整房屋实例名 | DMD `add_model.name`，例如 `living_room_box_0`；房间对象通常前缀 `<room_id>_`，复合物体有展开后的特殊名字，不能只机械拼接。 |
| Body/link | SDF/URDF 的 link 与拓扑，加载后核验。咖啡桌、盒子为 `base_link`，sideboard 根是 `E_body_5`，不能全局默认 base_link。 |
| 复合物体 | B 包含 `dinner_plate_0_s0_*`、`glass_bowl_0_f0_c`、`apple_0_f0_*` 等；对照 JSON metadata 与 `SS/scenesmith/robot_eval/dmd_scene.py::_expand_composite_members` 的规则建立映射，保留“语义容器/组 → 多个物理成员”。 |
| 固定关系 | DMD add_weld、SDF static/fixed joint 和实际 Plant 拓扑；JSON immutable/类别是提示，不是是否自由的最终判据。 |
| 可动关节 | A sideboard SDF 有四个 revolute joint；将根焊接不等于柜门锁死。关节轴/上下限/惯量在 SDF，默认关节初值须解析实际模型，不从类别猜测。 |
| 初态 | DMD default_free_body_pose 或 add_weld 的 X_PC、模型默认关节值；JSON transform 是生成时状态。运行后位姿从 Context/observation 读取，不回读历史 JSON。 |
| 物理参数 | SDF inertial、collision、surface/proximity_properties；生成视觉网格不自动等价精确碰撞。保留已接受的碰撞代理，不在重构中更改质量/摩擦来凑回归。 |
| 单位与方向 | 位置为米、Z-up；关节转角 rad、直线关节 m；DMD 本次有 `!AngleAxis angle_deg`，也可能有 `!Rpy deg`；SDF pose 的 rpy 为 rad；JSON quaternion 为 wxyz。明确按字段解析，不统一按一个角度单位猜。 |
| 房间/房屋原点 | `HouseScene._get_room_position` 将 layout 角点加半宽深得到房间中心。A 的 `room_living_room_frame` 相对 house/world 平移 `[3.5,2.75,0]`；单房间世界在房间中心。metadata 中 box 平移 `[-1.750934...,0.198948...,0.494792...]` 是 room-local，不能直接作为完整房屋世界位姿。 |
| 模型原点 | SDF link 原点、visual/collision origin 与质心不同。原 box 质量 0.5 kg，惯性原点 z≈0.01982；EV 小红盒为中心原点 primitive、0.1 kg、6×4×3 cm。替换文件后沿用旧 bbox/中心语义不正确。 |

`SS/scenesmith/robot_eval/dmd_scene.py::_normalize_scene_state` 展示 room/house 归一化；`DMDScene._sync_poses_from_drake` 将 DMD 的真实世界位姿同步到评测视图。这是适配规则参考，不建议默认 import 该模块：它会进入 SceneSmith 依赖链。eval 的小型适配层只实现自己需要的格式子集，并用 A/B 样本校验。

两个样本原始 DMD 均没有机器人，已有 room geometry 地面/墙/天花板；A 已有任务语义盒子，不再无条件额外添加第二个盒子。eval 派生 DMD 用 `package://zerith_pick_eval/small_red_box.sdf` 替换同一实例并改变目标/另一物体的位姿；必须配套**派生元数据视图**，不能把原始 house_state 原样交给 VLM 作为新盒子几何事实。

### A5. 包冲突、地面和机器人重复加载

- 两个场景 package.xml 都声明 `scene`。每个 environment/Parser 独立注册一个场景根；同名不同根构建时明确报错。当前 `register_package_xml` 直接 Add，`populate_model` 开启 AutoRenaming；自动重命名不能当成语义 ID 冲突解决方案。
- 本轮目标不需要同一 world 拼接两份 scene 包。若未来需要，必须在派生缓存中显式命名空间化，不改上游，不静默改名。
- `populate_model` 当前无条件先加载场景、再添加 Adapter 机器人。最小新规则：默认场景必须不含待装配机器人；发现冲突/疑似已有机器人要求显式声明处理，不靠名称猜测并删除。第一阶段支持明确拒绝；复用已有 robot instance 只有在 Adapter 明确支持时才允许。
- **新发现的实际适配缺口**：SS `room_geometry_living_room.sdf` 的 floor_collision 与墙体 collision 同属 `room_geometry_body_link`。`EV/src/online_manipulation/navigation_geometry.py::build_navigation_map` 按 ground_body_names 排除整个 body；当前 mobile fixture 地面是独立 `ground::floor`，所以已有实验不覆盖 SS 这个情况。若直接把房间 body 标记为地面，会把墙也排除。
- 修复建议是给场景适配/导航投影提供精确的地面 geometry 选择器（model/body/geometry name，构建时解析 GeometryId），仅排除 floor_collision；轮地支撑许可也应使用精确几何对，不能将 wheel↔整个房间 body 都当合法支撑。沿用旧独立地面声明，但遇到混合墙地 body 必须拒绝含糊声明。不得删除墙碰撞、放宽全局阈值或改 Runtime 来适配某个房间名字。

### A6. 上游 policy_interface 与 validator：可选外部模块

| 模块 | 当前本地输入/输出 | 依赖与接入建议 |
| --- | --- | --- |
| `SS/scripts/robot_eval/policy_interface.py` | 输入 scene-state、DMD、scene-dir、自然语言 task、model/API mode；`--output-json` 输出 task、robot_start_xy、world_bounds、commands。每项 command 有 action/rank/confidence/drake_model_name/target_position/placement_bounds/reasoning。 | LLM/Agents、Drake、SceneSmith 元数据工具；默认 Blender vision，可 --no-vision。它是语义绑定/几何目标提议，不是关节控制器，不产出可直接送 env.step 的 RobotCommand，也不保证目标可达。作为离线准备/可选 planner 输入。 |
| `SS/scripts/robot_eval/validate.py` | 输入原/派生语义 state、机器人输出的新 DMD、task、scene-dir；默认终端日志展示 overall_success/overall_score/requirements/reasoning。 | `validator_agent.py::validate_task` 返回结构化 ValidationResult，但当前 CLI 没有对应 --output-json，不得编造命令。用选中时才调用的 SS 环境桥接脚本序列化结果，保留独立进程/解释器。 |
| generate_prompts | 人类任务→prompts.csv/task metadata，给 SceneSmith 生成阶段使用。 | 不在 eval 默认 reset/step 路径；换现有场景无需重新生成。 |

`--no-vision` 跳过 Blender server 启动，但脚本仍顶层导入 BlenderServer，不能宣称完全免除 Blender 安装；LLM 仍需有效服务配置。不要将这两个脚本直接 import 到 eval 公共 API。桥接使用明确的 SS Python 路径、超时、退出码和机器可读错误；不从日志中的字符串“success”猜测结果。

在线 success 与离线 VLM 结果单独记录：前者决定实时 Task termination，后者只能增加评测报告，不悄悄改变 episode 已记录的成功、控制或允许接触规则。

## B. Eval 全项目分类与实际消费者

### B1. 已有运行链：保留，不重写

当前通用入口为 `examples/online_manipulation/run_online.py::main`：`load_factory(env)` → `load_factory(policy)(config)` → `run` → `make_env(config)` → `config.build_environment()` → `run_episodes/run_episode` → `reset/step`。

`RuntimeConfig.build_environment` 组装 `OnlineManipulationEnv(DrakeRuntime, Task)`。固定 Zerith 的 `ZerithEnvironmentConfig.build_environment` 也生成 RuntimeConfig，仅保留固定单臂动作 resolver；`src/zerith_online_env.py::ZerithOnlineEnv` 是旧字典 API wrapper，不是独立物理循环。双臂/移动使用 Description/ZerithDual/ZerithMobile Adapter。

`scripts/run_zerith_online_example.py::build_run` 已复用 examples 中的 make_config/build_policy/run，Hold/JointStep 不读专家文件；但仍使用 PickLift recipe、声明 pick_target observation，因此“无专家依赖”不等于“这个旧 CLI 已对任意场景通用”。默认推荐通用入口，旧 CLI 保留约定参数，不重新复制环境组装。

### B2. 文件归属和迁移影响表

所有建议位置均相对 EV；“可选”表示退出默认导入/日常入口，不代表直接删文件。

| 当前位置 | 实际职责与消费者 | 分类 | 建议位置/处理 | 受影响导入、命令、文档 |
| --- | --- | --- | --- | --- |
| `src/online_manipulation/{environment,runtime,factory,model,controller}.py` | 通用 reset/step、单一 Simulator、模型装配、多频伺服；新旧入口共用 | 核心保留 | 原位保留；新增组装代码在外围，不再建 Runtime | 全公共 API、运行/控制/双臂测试 |
| `{actions,observations,specs,protocols,contact}.py` | 具名动作、传感器、机器人契约、Task 接触规则 | 核心保留 | 保留现有类型和语义，组装期补能力检查 | API contract、旧动作/组合动作等价检查 |
| `{sensors,_drake_camera_time}.py` | 通用 RGBD 采样、同帧时间/pose、Drake 1.49 fallback | 核心能力，按配置启用 | 原位保留，不复制到任务 | 相机测试、所有 Adapter；不调整外参 |
| `adapters/description.py` | 任意具名受控关节、无夹爪 fixture 的真实适配 | 核心保留 | 原位保留 | 第三方 Adapter 和真实非7轴测试 |
| `adapters/zerith.py` | RobotSpec、三相机、固定动作兼容、旧 backend/config 集中 | 机器人组件 + 兼容 | 优先移出任务常量；后续可拆同包 `zerith_model`/兼容部分，保留公共导出，不按行数强拆 | zerith_dual 私有 helper 导入、转换/兼容测试、旧 CLI |
| `adapters/{zerith_dual,zerith_mobile}.py` | 双臂组/双夹爪、轮轴与支持轮、可动根拓扑 | 机器人组件，必须保留 | 保持模式与模型假设；右臂不另复制相机定义 | mobile/dual/camera tests、导航配置 |
| `src/zerith_{robot_config,grasp_geometry,servo_config,gripper_config}.py` | 站位、TCP、盒尺寸、控制增益、夹爪行程混合；Adapter、转换器、标定脚本消费 | 机器人/控制/任务混合 | TCP/行程归机器人；增益归控制 profile；站位/盒/预抓取归实验或专家配置 | `prepare/search/visualize/validate`、Adapter；新旧解析结果比较 |
| `models/Zerith_Model` | 上游 URDF/STL submodule | 外部模型输入 | 保留 source commit，不改上游 | 转换器、首次 clone |
| `models/zerith_drake/{urdf,package.xml,conversion_manifest.json}` + `meshes/*.obj` | 派生机器人包；43 OBJ 未跟踪，URDF/manifest 已跟踪 | 小型定义 + 可重建缓存 | 第一阶段保留包位置；OBJ 按缓存生命周期生成，不必须物理搬目录 | convert --check、所有 Zerith 入口、包路径 |
| `scripts/convert_zerith_for_drake.py` | STL→OBJ/接触代理生成，v5，35 visual+8接触OBJ | 模型准备工具 | 可归 `scripts/models/`，先保留现有可复制命令 | 相对 REPOSITORY_ROOT、tests、Quickstart |
| `models/zerith_pick_eval/{environment.json,pick_lift_calibration.json}` | episode recipe/专家参数；示例 make_config/build_policy 消费 | 实验输入/Policy 输入 | `experiments/picklift.json` 与 `experiments/policies/picklift/`，不留第二份默默覆盖 | 旧 CLI --environment-json、校准检查、示例/tests/docs |
| `models/zerith_pick_eval/{small_red_box.sdf,package.xml}` | 任务对象；派生 DMD 用 package URI 引用 | 可选任务资产 | 保留此包，不因配置搬走而改 URI | 派生场景生成与加载 |
| `models/{online_env_minimal_scene,mobile_scene,zerith_camera_calibration,runtime_fixture}` | 自包含场景和真实测试机构 | 测试/示例资产保留 | 小且有消费者，不删除；用实验配置引用 | scene/camera/mobile/description tests |
| `models/{21-20-10_cleaned,iiwa}` | 原始离线评估任务与机器人 | 可选示例资产 | 暂留；未来可外置餐厅包并提供完整下载校验；先处理缺失纹理 | README 离线章节、run_experiment_*、离线脚本 |
| `planning.py`、`contact.py` | 当前状态查询、IK/局部IK、动作边/携物检查 | 核心安全能力 | 保留；不把安全查询随可选 TAMP 一起删除 | Runtime、Task、Cartesian、TAMP helper |
| `src/zerith_pregrasp_collision.py`、`zerith_pick_workspace.py` | 固定左臂/可选导轨静态搜索，独立规划模型；多个标定脚本消费 | 可选标定组件 | 收到 `tools/calibration/`，复用模型/TCP来源，不直接用于移动在线规划 | search/validate/pregrasp 仿真脚本 |
| `src/item_locking_monitor.py` | 旧离线自动锁物体；同时提供 AABB 辅助 | 可选离线算法 + 被复用工具 | 先抽 AABB 到小型 geometry 模块，锁定算法退出默认路径 | prepare_scene、pick_workspace、simulate_noninteractive |
| `src/{rrt,shortcut}.py` | 旧 BiRRT/shortcut；非交互离线 planner 消费 | 可选 planner | 原位或 `planning_offline/`，不挂到默认 Policy | plan_robot_waypoints_rrt_noninteractive |
| `policies.py` | Hold、JointStep、PickLift 状态机 | 可替换 Policy | 核心协议不动；可按 basic/picklift 分类，保留顶层导出；权重/输入输出适配只放 Policy 侧 | factory、CLI、策略测试 |
| `tasks.py` | NullTask/PickLiftTask，绑定/接触/成功判定 | 可替换 Task | 保留 Task 生命周期；抽取当前成功判定为可注入 evaluator 时先保持数值等价 | termination、runner summary、携物规则 |
| `base.py` | 平面运动学积分/轮驱力矩，真实观测 | 运行能力，必须保留 | 原位保留，模式构建时选，不是目录清理对象 | 两模式所有验收 |
| `{navigation,navigation_geometry}.py` | 静态 pose 导航与地图构建 | 可选运行组件，必须保留 | 原位或 navigation 子包；地面几何选择先补齐；不新建 ROS/Nav2 | 导航示例、地图/取消测试 |
| `runner.py`、`dmd_finalizer.py` | episode/benchmark/日志/选定自由体写回 | 核心保留 | 沿用输出契约；离线评估在 runner 外围调用 | 所有入口、CSV/JSON消费者、round-trip |
| `examples/.../run_online.py` | 已有可信 Python factory 入口与 run 薄包装 | 通用入口 | 将可复用组装函数放 `experiment.py`，示例继续示范；复用 runner | 旧 CLI imports、generic tests、Quickstart |
| `examples/.../minimal_setup.py`、`pick_lift_demo/*` | 用户参考配置与专家输入适配；旧 CLI imports | 示例/实验组装 | 示例保持短小；被正式入口复用的组装移出 examples，避免反向依赖 | 旧 CLI、外部客户端测试、示例 README |
| `examples/.../mobile_smoke.py::make_config` | 被 navigate_demo/navigation_manipulation/audit/验证及测试导入 | 共享组装错放 | 抽到实验工厂，不让示例以 smoke 为配置源 | mobile_navigation/mobile_cameras/closure tests |
| `examples/.../{public_api_client,camera_public_api_client,mobile_public_api_client,cartesian_client,cartesian_policy,example_policies}.py` | 外部使用示例/接口 smoke | 示例保留 | 不为消除少量重复而破坏“可复制的独立客户端” | Quickstart、外部 API 验证 |
| `examples/.../{behavior_tree_tick,tamp_execution}.py` | BT tick 与已验证边执行；TAMP 自动同步当前状态 | 可选接入示例 | 保留小接口；生产复用 helper 才移 integrations，不实现完整 BT/TAMP | examples/__init__、handoff/closure tests |
| `examples/.../{audit_base_modes,validate_mobile_navigation}.py` | 读取 backend/Plant 的机制审计与反例 | 验证工具 | `scripts/validation/`，不得作为用户 Policy 范例 | 审计文档、路径 parents、shared factory |
| `scripts/{audit_cartesian_edge_rejections,validate_zerith_collision_proxies,validate_zerith_camera_geometry}.py` | 安全/模型/相机审计；tests 有直接 import | 验证工具保留 | 分类目录，保留必要 public 验证函数 | camera calibration、finger geometry tests/docs |
| `scripts/{search_zerith_safe_home,search_zerith_pick_home,search_zerith_rail_postures,validate_zerith_pregrasp_ik,validate_zerith_pregrasp_collision_scope,validate_zerith_safe_home_dynamics,validate_zerith_target_settle,simulate_zerith_pick_home_to_pregrasp}.py` | 历史标定搜索、静置与 PREGRASP 验证 | 可选标定/验证 | 退出默认安装后的运行路径；输入参数化，保留可选再标定能力 | 专家生成说明、private helper/固定机器人配置引用 |
| `scripts/{simulate_zerith_left_arm,visualize_zerith_left_arm,visualize_dmd_scene}.py` | 控制诊断与可视化 | 工具保留 | 归类，不误删仍被使用的兼容 wrapper | 诊断文档、模型路径 |
| `scripts/prepare_zerith_pick_eval_scene.py` | 小盒替换、目标/另一物体姿态标定、派生 metadata | 任务专用准备 | 通用 DMD/包校验抽出去；保留专家任务生成逻辑，不伪装通用场景加载器 | Quickstart、PickLift input pipeline |
| `scripts/{add_robot_to_directives,compute_grasp_config_noninteractive,plan_robot_waypoints_rrt_noninteractive,simulate_noninteractive}.py`、`run_experiment_{noninteractive,folder,folder_parallel}.sh` | 原始离线流水线 | 可选流程 | 退出默认 Quickstart 主线，但保留独立章节；不混入在线 action/控制语义 | README、IIWA/餐厅资产、manipulation 依赖 |
| `output/`、根目录 `robot_task.dmd.yaml/robot_waypoints.json/robot_plan_good.json/out_good.dmd.yaml` | 运行结果与部分仍被读取的输入 | 混合，见 F | 先迁必需输入，后清输出；不因 ignored 判无用 | 环境变量、示例、审计文档 |
| `.venv`、`requirements.txt` | 运行环境/安装依赖 | 环境保留 | 不清环境；后续按 runtime/tools/offline 分 requirements，沿用现有 pip，不引入第二配置框架 | 全新 clone 安装、renderer 系统库 |
| `docs/`、`tests/`、已有删除 | 当前使用说明/历史版本证据/回归 | 保留并修链接 | 按功能入口索引，不恢复三份旧 ONLINE_ENV 文档或已退役交互脚本 | README/GENERIC/Quickstart/进度与对应测试 |

### B3. 跨阶段问题的准确结论

1. **配置已分组但未彻底单一来源**：environment.json 的 initial_state 与 zerith_robot_config.py 的咖啡桌旁站位重复；grasp_geometry 混合 TCP 与盒参数。JointSpec 又合并硬件/模型限制和 kp/kd；RobotSpec 包含 base_pose/home。无需立即破坏公开 dataclass：组装时从机器人 profile、控制 profile、初态分别读取，再生成原类型；禁止同一个值在两处定义后靠覆盖顺序碰运气。
2. **通用环境无专家强制依赖已实现**，固定专家 Policy 仍读 output 中两份 IK 文件。旧 CLI 的 task_enabled=False 仍组装特定目标 observation，是旧场景 recipe 的限制，不是 Runtime 依赖红盒。
3. **规划与执行**：通用 Runtime 用一个 Plant，PlanningQuery 独立 Context，`get_planning_query()` 同步当前底座及自由体。静态标定模型另有焊接底座分支，不能宣称该分支支持移动控制；保留为可选搜索工具。
4. **私有访问**：Task 当前用公开 observation/get_planning_query；TAMP helper 自动更新状态。audit_base_modes 直接读 env.backend/Plant 并挂审计钩子，属于授权诊断；一些旧 calibration/visualizer 直接 SetPositions 用于候选/可视化，不等于在线控制瞬移。禁止将这些模式带进新 Policy。`navigation_geometry` 读的是规划 Context，属于 Drake 几何适配，不修改真实仿真状态。公开 backend 仍是迁移诊断出口，不推荐外部策略使用。
5. **默认重依赖**：实际移除 API-key 环境变量后，EV 公共 API 导入成功，sys.modules 中没有 scenesmith/openai/agents/torch/bpy/manipulation/trimesh/networkx/scipy。不能声称当前已强依赖生成环境。公共 __init__ 仍 eager import 全部机器人/导航/参考策略；将来增加模型权重或 VLM 时必须选择后才 import。运行安全 PlanningQuery 属于 Drake 核心，不等于可选外部 planner。
6. **成功分类器**：当前 PickLiftTask 自己 evaluate，尚无独立 evaluator 注入接口；Policy 有 reset/act，但无通用权重、图像预处理、输出适配配置。应承认缺口，增加小型组装适配，不声称任意策略模型可直接运行。
7. **写回限制**：`dmd_finalizer.write_updated_dmd` 仅修改标记 write_back 的直接 add_model/default_free_body_pose，保留原 base_frame；不写机器人/关节速度/关节状态，不递归改嵌套 directives，不保存整套仿真快照。输入不能被覆盖。离线 evaluator 若需要所有移动自由体，应显式选全，不能只写红盒后声称其他物体状态也最新。
8. **已有配置能力不等于任意替换能力**：Runtime 当前直接构造 `CoupledInverseDynamicsServo`，RendererSpec 仅接受 VTK，公开 GripperSpec 表达平行夹爪宽度；本次先分离控制参数来源，不为“可替换”引入尚无消费者的控制器框架。未知控制器/renderer/夹爪映射应明确报不支持；第三方 RobotAdapter 仍须实现实际执行器与动作适配。公共 __init__ 的显式导入与 __all__ 列表也应在迁移时核对，避免遗漏已新增的组合/底盘 API 导出。

## C. 最小目标目录与依赖方向

先复用现有包和配置机制，只新增确有消费者的小模块。目录是职责目标，不要求一阶段完成全部 mv。

```text
EV/
  experiments/
    minimal.json / mobile_kinematic.json / mobile_dynamic.json / picklift.json
    robots/zerith.json                 # 模型/TCP/夹爪/机载安装选项
    controls/zerith_validated.json      # 明确的仿真伺服 profile
    policies/picklift/                  # 专家标定、已验收小型 IK 输入
  models/                              # 原模型、任务 SDF、自包含测试场景
  cache/                               # 派生 DMD/元数据视图/资产修复/搜索候选
  output/                              # 每次运行的日志、图像、CSV、HTML、新 DMD
  src/online_manipulation/
    [现有运行核心、actions、observations、Adapter、base、sensors、planning]
    experiment.py                      # 配置解析/能力校验/组装，调用已有 runner
    scenesmith_input.py                # 只读解析与依赖检查，生成 ScenarioSpec
    geometry.py                        # 仅提取确实共享的 bbox/变换工具
    evaluators.py                      # 小型在线评估结果接口，先等价拆当前判定
    integrations/scenesmith_eval.py     # 可选外部进程桥；不默认导入 SS
  examples/online_manipulation/         # 短小真实客户端，不承载核心配置来源
  scripts/{models,calibration,validation}/
  [既有旧 CLI 可继续薄转发，离线流水线单独说明]
```

JSON 沿用现有 json/dataclass/importlib factory，不把 SceneSmith 的 Hydra 搬进 eval。一个实验配置分别引用机器人、控制、Policy 参数；不是建立任意层级继承和递归 merge 系统。路径相对配置文件，外部资产根由明确变量解析；未知字段、未知变量和重复 package 映射立即报错。首次仅支持现有受信任 `module:function` factory；不执行远端配置，不自动下载权重。

模型文件生成位置先保留 `models/zerith_drake/meshes` 以降低 URI/命令迁移风险；其逻辑分类是可重建缓存。以后确有需要再整体迁派生包，不为追求目录外观改所有资产引用。

依赖规则：核心不能 import experiments/examples/scripts/SceneSmith；机器人定义不能 import 任务；Policy 可依赖动作/观测和明确规划能力；Task 可依赖公开观测/规划快照，不读私有 Context；Runner 不认识任务 ID 或成功阈值；离线 evaluator 消费 EpisodeResult/artifact，不推进 env.step。

外部场景相机归场景，不伪装为机器人 CameraSpec。当前 ScenarioSpec 没有外部相机列表，CameraSystems 的 parent lookup 在 robot instance 内；本次迁移先明确不支持并在构建时拒绝请求，不为目录整理实现新相机模式。机器人三相机及 renderer 机制完整保留。

## D. 完整实验配置与组装接口建议

以下是**建议 JSON，不是当前 API 已接受的格式**。目的：表达一套完整固定 PickLift 实验，保留既有数值，不让用户进入 Python 修改 Runtime。引用的 profile/专家输入在迁移阶段创建，不能现在复制运行。

```json
{
  "schema_version": 1,
  "name": "zerith_fixed_picklift",
  "scene": {
    "source": "scenesmith",
    "root": "${SCENE_ROOT}",
    "dmd": "combined_house/house_furniture_welded.dmd.yaml",
    "metadata": "combined_house/house_state.json",
    "package_xml": "package.xml",
    "existing_robot": "reject",
    "missing_asset_policy": "error",
    "external_assets": {},
    "overrides": {
      "factory": "eval_scene_recipes:prepare_validated_picklift",
      "target_model": "living_room_box_0",
      "replacement_sdf": "../models/zerith_pick_eval/small_red_box.sdf",
      "package_xml": "../models/zerith_pick_eval/package.xml",
      "calibration": "policies/picklift/scene_calibration.json"
    }
  },
  "robot": {
    "profile": "robots/zerith.json",
    "adapter": "zerith_fixed_left",
    "model_directory": "../models/zerith_drake",
    "base_mode": "fixed",
    "cameras": {
      "enabled_names": [],
      "width": 320,
      "height": 240,
      "fov_y_rad": 1.0471975511965976,
      "near_m": 0.05,
      "far_m": 10.0,
      "update_period_s": 0.05,
      "modalities": ["rgb", "depth", "label"]
    }
  },
  "initial_state": {
    "robot": {
      "base_xyz_m": [2.65, 2.95, 0.1815],
      "base_yaw_rad": 3.141592653589793,
      "rail_position_m": 0.4,
      "arm_positions_rad": {"left": [0, 0, 0, 0, 0, 0, 0]},
      "locked_joint_overrides": {}
    },
    "objects_world": {},
    "randomizations": []
  },
  "control": {
    "profile": "controls/zerith_validated.json",
    "controller": "coupled_inverse_dynamics",
    "physics_dt": 0.001,
    "controller_dt": 0.005,
    "policy_dt": 0.1,
    "maximum_joint_delta": 0.1,
    "maximum_cartesian_joint_delta": 0.02,
    "action_semantics": "v0.3_fixed_left"
  },
  "observations": {
    "objects": {
      "pick_target": {
        "model": "living_room_box_0",
        "body": "base_link",
        "write_back": true
      }
    }
  },
  "task": {
    "factory": "picklift",
    "target": "pick_target",
    "support_bodies": ["living_room_coffee_table_0::base_link"],
    "carrier_arm": "left",
    "carrier_gripper": "left",
    "required_lift_m": 0.08,
    "required_hold_s": 3.0,
    "online_evaluator": "picklift_physical_v0.3"
  },
  "policy": {
    "factory": "picklift_expert",
    "pick_home": "policies/picklift/pick_home.json",
    "pregrasp": "policies/picklift/pregrasp_ik.json",
    "calibration": "policies/picklift/pick_lift_calibration.json",
    "weights": null,
    "observation_adapter": "identity",
    "action_adapter": "typed_v0.3",
    "parameters": {
      "closed_width_m": 0.0,
      "lift_distance_m": 0.1,
      "cartesian_step_m": 0.003
    }
  },
  "planning": {"safety_query": true, "external_planner": null, "tamp": null},
  "navigation": {"enabled": false},
  "evaluation": {"offline_evaluators": []},
  "run": {
    "seeds": [500],
    "max_steps": 1200,
    "episode_duration_s": 120.1,
    "cache_directory": "../cache/zerith_fixed_picklift",
    "output_directory": "../output/zerith_fixed_picklift",
    "meshcat": {"enabled": true, "port": 7025},
    "renderer": {"engine": "vtk"},
    "record_html": true,
    "write_final_dmd": true
  }
}
```

`eval_scene_recipes:prepare_validated_picklift` 等简称/factory 是建议组装名，不是已存在函数。`scene_calibration.json` 应保存已验收的显式物体覆盖，不在每次启动时根据夹爪位置重新移动红盒。`objects_world={}` 表示沿用派生场景物体初态，避免重复定义两个位置来源。profile 提供固定默认值，CLI 只允许有文档的显式覆盖并记录最终解析值。

建议命令形式（**迁移完成后才可用**）：

```bash
python -m src.online_manipulation.experiment --config experiments/picklift.json --prepare-only
python -m src.online_manipulation.experiment --config experiments/picklift.json
```

通过已定义的 SCENE_ROOT 传入外部场景根，所有其他路径相对实验配置。不要将上面的命令加入“已验证 Quickstart”。当前可执行的通用方式仍是 `python -m examples.online_manipulation.run_online --env-factory module:function --policy-factory module:function ...`。

替换方式与能力校验：

- 换场景：改 scene root/DMD/metadata、必要对象绑定、初态及导航地面几何声明；不复制机器人 profile，不改 Runtime。
- 换机器人：选择 Adapter/profile；对 Policy 声明的 arm/TCP/gripper/camera names 与支持 action 类型逐项校验。fixed_left、dual_fixed、dual_mobile 不假装是任意硬件通用配置。
- 换策略模型：选择 Policy factory、weights、obs/action adapter；权重加载、resize/归一化、模型动作映射在 Policy 侧，输出仍为公共 action。权重格式不支持即报错，不创建万能推理平台。
- 换 evaluator：Task 的在线 evaluate 委托小型评估器；绑定/接触仍由 Task 控制。离线 evaluator 在 episode 后消费结果和新 DMD，VLM 服务只在选中时启动。
- 换导航：独立 `set_goal/act/cancel/status` 协作面；沿用已有控制权与停车规则。新底盘组合用 RuntimeConfig，不静默将固定 resolver 的历史语义替换掉。
- planner/TAMP：消费 `env.get_planning_query()` 的当前快照和具名目标，输出规划结果/动作；TAMP 执行 helper 继续自动同步，不允许用旧 snapshot 驱动新底盘位置。
- 初始阶段不支持“复用场景已含机器人”、新外部相机、任意物体关节状态写回时明确报错/声明限制，不临时实现或伪装支持。

## E. 分阶段迁移：最小改动、验收和回退

### M0：保护基线与输入清点

- 保存当前工作树的改动/删除清单，不以 git reset/checkout 恢复；与本轮方案分开处理。
- 明确 A 完整依赖、B 缺失纹理、PickLift 当前四份输入的消费者；不要先删 output。
- 验收：无代码行为变化；已有 v0.3/ref 不移动；关键输入内容哈希可追溯。
- 回退：仅撤销新增清单/文档，不动用户原有工作。

### M1：只读 SceneSmith 适配与依赖检查（优先）

- 增加 `scenesmith_input.py`，从明确 DMD/package/metadata 构造现有 ScenarioSpec 及只读绑定视图；严格校验缺失资源、包冲突、对象 model/body/根链接、同名机器人、自由/固定关系。
- 原始生成包只读。绝对路径重定位、外部纹理补齐、SDF 替换/释放对象/增加任务对象仅落到派生 cache，附简短 source hash+覆盖记录，不建新场景格式。
- 单房间变换与复合对象映射做针对性检查；不把 metadata 位置重复加房间偏移。
- 地面/墙混合 body 的导航选择与轮地接触许可增加几何粒度适配；保持旧独立地面配置结果不变。做“排除 floor 但保留 wall”的反例，不能以已有 mobile fixture 通过替代。
- 验收：A 依赖搬到另一根目录后加载、对象名称和世界初态一致，原始字节不变；B 精确报出三张缺失图，补显式资源后才宣称视觉包完整。机器人/地面重复加载明确拒绝。
- 回退：旧 ScenarioSpec 构造入口仍可用；新适配器未启用不影响原流程。缓存可删除重建，不覆盖源。

### M2：输入迁出 output、配置归属落地

- environment.json 分组迁为实验 recipe；删除重复任务常量的定义来源，由旧工具读取同一配置。TCP/夹爪定义与策略盒参数分离，控制 profile 组装回现有 JointSpec，不破坏其公开结构。
- 小型已验收 pick_home/pregrasp 作为专家 Policy 输入保存；未承诺搜索器能逐位重现人工选择，不能只写“运行 search 即可”代替交付。先保留已验收数值，并校验 robot/source scene/初态指纹。
- 派生 DMD/元数据按已验收覆盖可重建；复杂搜索候选进 cache，运行日志进 output。
- 验收：空 output + 现有只读场景与模型输入，执行准备后可 reset/step；NullTask/Hold 完全不读专家文件；换 Task 不复制机器人参数；旧 CLI 与新 recipe 解析的初态/control/task 等价。
- 回退：提供显式路径参数继续读取旧位置，迁移完成前不删除旧文件；不增加永久的“找不到则偷偷回读 output” fallback。

### M3：统一实验组装与小型替换接口

- 新 experiment 入口解析 JSON/受信任 factory，调用现有 make_env/run_episodes；将正式共享组装从 examples 提出。旧 CLI 只翻译旧参数，保留 .01 joint-step/.03 closed-width 等合理兼容默认值，不静默换成专家参数。
- 抽 `mobile_smoke.make_config`；抽 monitor 中已有共享 AABB；不复制仿真循环。
- 构建时检查 joint actuator 约定、arm/gripper/TCP/camera需求、动作语义与规划能力。默认实例与指定实例冲突报错；禁用权重/VLM/导航模块时不加载其专属依赖。
- 先将当前 PickLift 成功判定等价封装为在线 evaluator；Task 仍拥有双侧接触/携物/终止时间状态或明确交给 evaluator 实例，不全局共享。替换 Null evaluator/纯几何 evaluator 可用同一个 Runtime。
- 可选外部 SS evaluator 通过独立 Python 进程返回结构化结果；仅在确需该评测时实现桥，默认配置不引入服务。既有输出 schema 不悄悄变化，可追加单独 `evaluation.json`。
- 验收：外部客户端换兼容 Policy/Task/evaluator 不触碰核心；配置不兼容启动即失败；新旧同配置命令差异检查。固定 PickLift 回归只是行为继承，不是“任意策略可用”。
- 回退：旧公共 API/factory 保留；新入口取消导出即可回到旧入口，共享 Runtime 不分叉。

### M4：工具分类、最小安装与文档收口

- 移动审计/标定到明确工具位置，检查 tests 的直接 import、__file__.parents、脚本根路径、shell wrapper、README/GENERIC/Quickstart 链接。
- runtime 必需项与模型转换/离线 IIWA/外部 SS 的依赖分别说明；SS 独立虚拟环境，沿用 pip/requirements，不引入 Hydra、ROS 或万能插件框架。
- 保留仍有人使用的薄转发；已明确删除的四个交互入口不恢复。共享 Runtime、双臂、相机、底盘、导航不属于退役内容。
- 验收：从仓库外启动公共 API；默认禁用可选模块无其服务环境可运行；模型生成/--check 命令仍真实可用；既有 API 导出不意外缺项。
- 回退：按一组相互依赖的 import/CLI/docs 迁移整体回退，不制造脚本已搬走而测试尚未更新的中间状态。

### M5：只按消费者清理产物

- M2 完成前不删除四份 PickLift 输入；M1 依赖检查通过前不外置/清除大场景依赖。
- 清理 F 表中已迁移、无运行消费者的重复输出；保留少量对应发布的 summary/metrics 与必要最终日志，不建永久复杂归档系统。
- 验收：output 清空不损坏运行准备链；文件不存在时文档不再给死链，历史结果标为历史且可选重跑。
- 回退：在新链验收前保留旧输入副本；无需永久保存全部失败 trial/HTML。Git 跟踪资产若退役须另做明确授权的提交，不重写历史。

## F. 必需输入迁移与可清理范围（本轮不删除）

### F1. 先迁的输入

| 现有文件 | 当前消费者 | 建议归属/条件 |
| --- | --- | --- |
| `output/zerith_pick_eval/zerith_pick_eval.dmd.yaml` | 示例环境、旧 CLI、camera visibility、PREGRASP 工具 | cache 中派生 DMD；输入是只读 scene 包+显式覆盖+小盒 SDF。 |
| 同目录 `task_metadata.yaml` | Quickstart 完整性检查、人工标定说明；不等价每步 Task 配置 | 派生 metadata/cache；当前文件是 JSON 文本（YAML兼容），source_dmd 是绝对路径。保留 provenance，但运行引用以新 root 解析，不能依赖该旧绝对路径。 |
| 同目录 `pick_home.json` | `pick_lift_demo/policy.py::build_policy`、相机可见性、旧 PREGRASP 工具 | `experiments/policies/picklift/` 的已验收专家输入，或明确交付的外部 Policy 包；不是日志。 |
| 同目录 `pregrasp_ik.json` | build_policy/标定搜索/可视化 | 同上；与机器人、导轨、目标几何/位置保持一致，不能只搬文件并删一致性验证。 |
| `models/zerith_pick_eval/environment.json` | 示例和旧 CLI make_config | 实验 recipe，初态/control/bindings/task各自组装。 |
| `models/zerith_pick_eval/pick_lift_calibration.json` | 专家 Policy | 策略输入，不做机器人默认配置。 |
| `models/zerith_drake/meshes/*.obj` | 所有 Zerith 运行及相机 | 本地必需可重建数据；运行前 convert+check，不能删后声称开箱可跑。 |
| SS A 的 DMD/package/资产/对应元数据 | 当前 PickLift、视觉/物理查询 | 外部只读输入，不在 EV output 清理范围。 |

### F2. 输出处理清单

当前 EV output 约 170 MiB：mobile_manipulation 86 MiB、online_env_phase8 27 MiB、online_env_final_audit 19 MiB、zerith_pick_eval 16 MiB，其余为旧示例/回归。不是所有这些目录都必须永久保存。

- **迁完输入后可清**：`online_examples/pick_lift_trial_*`、重复入口的 `decoupled_picklift_seed500/generic_picklift_demo_seed500/quickstart_pick_lift_seed_500`、旧相机关闭/标定后的固定 PickLift trace，以及无当前消费者的 search/settle 中间诊断。先核对 docs 引用并更新，不按 trial 名称猜成功失败。
- **重复文件候选**：前轮对 >100KB 输出做内容哈希检查，有约 34.56 MiB 重复副本；包括 phase8/final_audit 的相同 summary、两个 planar physics.csv。`runner.py::run_episode` 对普通失败还将同一 summary 写入 failure.json。历史文件可选去重，但本次迁移不改变 summary/failure 双文件契约，不让下游突然找不到文件。
- **建议仅保留少量**：当前固定 PickLift final summary、两模式 parked summary 与 `manipulation_handoff_metrics.json`；最终全量日志 `closure_audit/full_tests_submission_ready.log`；底盘机制审计的关键 metrics/曲线；相机已跟踪 docs/assets 的标定/可见性图。无需保留每个版本全部 CSV/HTML。
- **大审计 CSV 可在确认后清理**：例如 `base_mode_audit_01/navigation_wheel_dynamic/physics.csv` 约22.08 MiB、kinematic导航约10.4 MiB；仅是机制证据，不是运行输入。保留关键汇总和对应生成命令即可；若未来需要复查具体尖点，再选择性保留那段原始数据。
- **根目录离线产物**：robot_task.dmd.yaml、robot_waypoints.json、robot_plan_good.json、out_good.dmd.yaml 在保留非交互离线生成命令后可清，不属于在线 env 必需输入；本轮不删。
- **不可算垃圾**：.venv、上游模型、被 DMD 引用的 OBJ/glTF/bin/纹理、small_red_box.sdf、测试 fixtures，以及仍被 Quickstart/Policy 读取的上述输入。
- **大跟踪场景**：`models/21-20-10_cleaned` 1537 个已跟踪文件，忽略规则不解除跟踪。外置是可选交付决策，不是磁盘缓存清理；删除当前版本文件不自动缩小 Git 历史。本轮不移、不改历史。

## G. 验收映射：复用已有证据，只补迁移风险

### G1. 已有证据与不能外推的结论

- 本轮读到 `output/closure_audit/full_tests_submission_ready.log` 的 `Ran 150 tests in 581.234s / OK`；这是此前最终实现测试，不是本轮重跑，也不与早先 24 项相加。
- 当前保留 `mobile_manipulation/final_fixed_picklift/.../summary.json`：seed500、277步、lift_held；两模式 final_parked summaries：wheel421步、kinematic420步、parked_dual_targets_held。它们证明对应运行，不证明双手共同抓物或任意上游场景导航。
- 历史 online_env_final_audit 固定3/3、扰动2/3/seed402失败须标对应版本；不能把这些旧结果当作 v0.3 新重构鲁棒性实测。
- Hold/NullTask 的 summary.success=False、max_steps/time_limit 不必是环境故障：该任务未定义 success。结构验收应检查构造、状态推进、退出语义，而不是强改 success=True。

### G2. 最小验收矩阵

| 目标 | 复用检查 | 迁移后新增/补充的具体检查 |
| --- | --- | --- |
| 空 output 可运行 | generic_online_example、picklift_demo_example、closure 无专家测试 | 在独立 checkout/临时输出根，显式准备缓存后运行；监测必需输入不再从旧 output 读取。固定专家输入不是现场重新搜索。 |
| 上游迁根不修改 | dmd_finalizer、既有 scene replacement | A 完整闭包复制到不同根并只读加载；hash原始文件；对比物体名、world pose、包映射；单房间绝对路径仅缓存重写；B缺失纹理必须先失败。 |
| 新场景无需改核心 | description_runtime、mobile_composition | A/B ID/body/复合展开检查；换场景仅配置，metadata不是直接用于覆盖DMD初态；同名包及重复机器人反例。 |
| SS 房屋导航接入 | mobile_navigation 既有fixture | floor/wall同body仍只排除floor；墙/桌沿阻挡保留；轮地支撑许可不放行轮↔墙。无需重跑全部机制隔离。 |
| 换机器人/策略/任务 | online_api_contracts、description_runtime、zerith_robot_adapter | 真实非7轴fixture和Mock契约分别说明；同一场景换Hold/JointStep/custom Policy、NullTask/另一Task不复制robot profile；未知相机/夹爪/动作明确失败。 |
| evaluator可替换 | PickLiftTask已有判定/runner tests | 在线 evaluator 等价重放同一 observation 序列；离线 evaluator 禁用不加载服务，失败记录不更改在线结论；VLM选择时才检查专属环境。 |
| 双臂与具名夹爪继承 | dual_runtime、picklift_command_contact、closure_contracts、cartesian_handoff | 保持组合动作原子提交、联合边检查、携物关节/Cartesian等价；不宣称双手协同抓同物体。 |
| 控制/底盘语义继承 | online_controller、mobile_composition、既有两模式审计 | 参数组装前后相同，模式不变；轮驱仍通过wheel torque/contact，运动学仍显式积分；配置迁移不重复大规模机制实验。 |
| 相机继承 | camera_api、drake_camera_time、mobile_cameras、camera_external_client、zerith_camera_calibration | 配置迁移后三相机名/外参/帧缓存不变；关闭时不创建renderer；新根加载后检查RGB来自场景而非仅非空。 |
| 当前规划状态 | planning_query、closure TAMP移动底盘测试 | 新入口使用相同get_planning_query；旧query不作为执行依据，候选Context变化不污染真实Context。 |
| 只写新结果 | dmd_finalizer round-trip | 在有房间base_frame的新根输入上写所选自由体；输出重载、输入hash不变；不支持嵌套写回/关节状态时明确失败。 |
| 依赖隔离 | 本轮无key公共API导入、已有外部客户端 | runtime最小安装启动state-only；选中可选外部模块才导入专属库/解释器。相机仍需要Drake/VTK系统渲染条件，不能承诺无图形依赖。 |

必须保留的接口语义：policy/servo/physics 默认10/200/1000Hz；Cartesian world表达、wxyz、rotvec左乘、以q_commanded的FK为累加基准；abs每step局部IK、持续提交，不改成隐含持久轨迹；未指定臂/夹爪保持旧目标，底盘省略则请求减速；拒绝不等于物理回滚/急停。相机RGB uint8 H×W×3、depth float32 H×W米制Z-forward、label int16 H×W，同一采样timestamp/pose保持；as_dict默认元数据、图像显式保存；Drake1.49时间fallback版本受控。

导航仍是静态 pose 导航：global用world/map/odom，local在接收时转为固定世界目标；到达默认位置≤3cm、yaw≤3°、实际v≤0.01m/s、|omega|≤0.02rad/s，连续0.5s；不宣称严格静止。双臂操作阶段不等价双臂抓取；移动携物、PLACE、导轨动力学、SLAM、动态障碍、完整TAMP、真实部署均不借结构整理扩展。

## H. 未决事项与推荐

普通文件命名和组织按本方案推进即可，不需要用户逐个决定。实施前只需确认影响交付内容的选择：

1. **专家输入交付**：推荐将两份小型已验收 IK JSON 和显式场景覆盖作为版本化 Policy 输入，附来源/适用场景，不依赖 output。若不希望入仓库，则必须提供外部资产包位置；不能仅提供不可保证重现人工选择的搜索命令。
2. **外部大资产**：推荐当前仍由用户提供 SCENE_ROOT；A 可按依赖闭包交付，B 在补齐纹理前标为不完整。是否将大样例改为下载包涉及存储位置与分发许可，应另行确认，不影响先做适配/配置迁移。
3. **默认可动性**：推荐日常通用实验显式选择 free/furniture_welded，当前固定 PickLift 保留家具焊接基准；不擅自将所有家具释放后调物理参数来维持成功。若目标是“所有家具都交互”，这是新的物理验收范围。

不需等待新功能选择即可实施的第一步是 **M1只读场景适配/依赖与地面几何检查，接着M2迁移必需输入和配置单一来源**。目录搬迁和删除放后面。

## H. 已批准方案的实施落点（2026-09-08，未提交）

A—G保留调查时的基线和建议；其中建议API、目录名和历史结果不是当前运行说明。
本节及 [EVAL_STRUCTURE_PROGRESS.md](EVAL_STRUCTURE_PROGRESS.md) 为实施后的权威映射，
实际运行命令见 Quickstart 第0、4节。没有创建第二套 Runtime，也没有新增通用 PluginManager。

| 阶段 | 实际文件/实现 | 保留行为与补充验收 |
| --- | --- | --- |
| M0 | 本地 Git/AGENTS、源A/B闭包及版本基线 | 保留用户原有README修改、旧入口删除；不修改SS产物、版本或Git索引 |
| M1 | `scene_input.py`、`scene_geometry.py`、`model.py`、`contact.py`、`planning.py`、`navigation_geometry.py`、`runtime.py` | A原始上游迁根及缓存再迁根、单房间绝对路径重写；地面/墙同body按geometry分开；B缺纹理明确失败 |
| M2 | `experiments/{profiles,minimal,mobile,picklift}.json`、`experiments/inputs/pick_lift/`、`recipes/settings.py` | 小型IK/标定/覆盖进入版本化输入；environment.json只引用profile；不重新优化标定，不读旧output补缺 |
| M3 | `experiment.py`、`assembly.py`、`evaluation.py`、`recipes/`、公共`__main__.py` | 新旧入口复用正式工厂和原runner；trusted Policy/evaluator替换；原PickLift判定透传；关闭可选规划不关闭核心动作边检查 |
| M4 | `tools/calibration`、`tools/audit`、`geometry_bounds.py`、`zerith_tcp.py`、分层requirements和现存文档 | 正式组装不导入scripts/examples/monitor；旧脚本仅转发已搬工具；不删除仍有消费者的研究工具 |
| M4衔接修复 | `dmd_finalizer.py`及针对性测试 | 真实缓存产生多行translation，旧写回只改首行导致无效YAML；现替换整个字段，保留其他模型文本并实际重载验证 |
| M5准备 | progress的消费者/清理表 | 仅列清单，不删除output、OBJ、环境目录或大跟踪资产 |

### 实际日常配置

`experiments/picklift.json` 已是可执行的小配置；策略路径由 `policy=picklift` 展开，
不要求日常用户复制巨大 RobotSpec。普通实验可将 task/policy 改为 null/hold，
选择自己的DMD及绑定，而不读取专家输入。

```json
{
  "robot": "zerith_left",
  "control": "picklift",
  "initial_state": "picklift",
  "scene": "scenesmith",
  "scene_options": {
    "variant": "furniture_welded",
    "overrides": "experiments/inputs/pick_lift/scene_overrides.yaml",
    "additional_package_xmls": ["models/zerith_pick_eval/package.xml"]
  },
  "policy": "picklift",
  "task": "picklift",
  "evaluator": "task_result",
  "run": {"seeds": [500], "max_steps": 1200}
}
```

入口为 `python -m src.online_manipulation CONFIG --repository-root REPO_ROOT
--scene-root SCENE_ROOT --cache-root CACHE_ROOT --output-root OUTPUT_ROOT`；此处大写项
只是语法说明，Quickstart提供完整变量命令。profile每组只展开一次，`*_options`
显式浅覆盖；不做复杂继承。`resolved_config.json`保存原配置、展开profile、实际
RobotSpec/相机/控制/Task/Policy/base、运行选项及来源哈希。SCENE_ROOT仍是外部资产，
没有下载系统；free与furniture_welded不是自动猜测。

`Policy`仍采用reset/act，`Evaluator`采用reset(observation, task_reset_info)及
evaluate(observation, baseline)->TaskEvaluation。评价器的连续判定计数只属于该实例，
每次reset清空；Task保留对象绑定及接触规则。默认TaskResultEvaluator返回原判定，
不更改抓取条件。外部factory需显式信任并可导入；不要求改中央分支，示例为
`examples/online_manipulation/external_evaluator.py`。离线VLM仍是episode产物消费者，
本轮没有实现新模型、完整TAMP或统一万能接口。

### 实施边界和回退

- 缓存必须是尚不存在的新目录；本轮不实现自动复用、失效回收或清理服务。
  输入移动到另一根目录后重新准备，或移动整个缓存闭包并重新指定package.xml。
- `prepare_scene`支持本地已出现的DMD/SDF/URDF/glTF/OBJ/MTL引用；不从网络补资产。
  未声明外部package、重复同名package、未知格式/标签应暴露错误。
- 替换对象保留语义ID到实际model/body映射。已明确的小盒尺寸/质量来自版本化覆盖，
  其余无法重建的bbox/表面等字段为unknown；源JSON不能被当作仿真后的真实状态。
- 已含Adapter同名机器人明确报错；不同名字是否代表另一个机器人不能可靠猜测，
  用户必须显式准备无重复机器人输入，不自动删除模型或静默加载两台。
- 相机安装、采样缓存、双臂/夹爪、底盘机制、导航、携物和TAMP同步继承原实现；
  没有把可选planner关闭等同于不检查动作碰撞。
- 原固定基准仍是固定底座、导轨0.4m、左臂PickLift。新场景绑定不是专家可迁移性保证；
  不宣称双手协同抓物、动态导航或任意场景抓取已验收。
- 回退按模块及其消费者一起回退；版本化输入需先恢复旧显式路径再退工厂，不制造
  缺输入的中间态。本轮只保留工作树变更，不执行任何回退、暂存、提交或tag操作。

确实可退出默认运行路径的旧内容：离线IIWA的IK→RRT→轨迹执行与自动锁物体monitor、历史safe/pick-home和导轨搜索、机制审计工具、生成prompt/VLM语义解析与离线validator、全部旧trial输出。退出默认路径不等于删除：有明确消费者的功能保留可选入口，已完成的双臂/夹爪/相机/双底盘/导航全部继承。
