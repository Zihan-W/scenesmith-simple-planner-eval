# Online Environment v0.3 快速开始

需要接入自己的场景、Task 和 Policy 时，请看
[通用在线运行示例](GENERIC_ONLINE_EXAMPLE.md)。已发布基线为
`online-env-v0.3`，公共 API 版本为 `0.3`；第0、4节明确描述尚未发布的结构迁移，不能在旧tag上执行。发布基线的兼容变化及能力边界见
[v0.3 发布说明](RELEASE_ONLINE_ENV_V0.3.md)。v0.1/v0.2 tag 不修改。

本文面向第一次使用仓库的同事。第1节只用于全新clone，不要覆盖有改动的工作树。
第2—3节是最小API/相机，第4节是外部资产PickLift，第6节是移动双臂，第7节是
可替换工厂入口。第5节单独标记v0.2历史安装证据；v0.3复用已验证依赖，未重新
做全新虚拟环境安装，不把历史安装记录当成新版本实测。

v0.3 的交付内容分为两类：

- **自包含功能**：最小 state-only 公共 API、机器人相机公共 API、三相机
  几何标定、两种底盘及导航停车后双臂空手操作。这些功能只依赖本仓库和 Zerith submodule，可以从全新 clone
  直接运行。
- **非便携 PickLift 演示**：还依赖一棵完整的 SceneSmith `scene_000` 场景
  目录。发布 tag v0.3 仍有旧的 ignored 输入依赖；当前工作树已迁移小型输入并支持从场景重建缓存，见第0、4节。


## 0. 当前工作树：有限 profile 实验入口（尚未发布）

本轮 M0—M4 代码仍是**未提交工作树**，不在 `online-env-v0.3` tag 中。不要为了执行
以下命令切换 tag 或覆盖现有修改。第1—3节保留发布基线的安装/相机用法；第5节明确是历史证据。
新入口复用原 Runtime，并不新增控制模式。下列命令从当前 eval 仓库根目录执行：

```bash
export REPO_ROOT="$(pwd)"
export PYTHON="$REPO_ROOT/.venv/bin/python"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export RUN_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/eval-experiment-XXXXXX")"
export MPLCONFIGDIR="$RUN_ROOT/matplotlib"

"$PYTHON" -B -m src.online_manipulation "$REPO_ROOT/experiments/minimal.json" --repository-root "$REPO_ROOT" --cache-root "$RUN_ROOT/cache-minimal" --output-root "$RUN_ROOT/minimal"
```

minimal 不读任何专家 JSON，也不需要 SCENE_ROOT/API key。10步后打印 `max_steps`；
NullTask 的 `success=false` 是没有定义任务成功，不是初始化失败。旧 minimal fixture
的自由 floor/box 存在环境间初始重叠，仅用于 API smoke，不用于稳定接触质量验收。
`resolved_config.json` 位于输出根；episode 的 JSON/CSV 使用既有输出契约。

- 日常只改 `experiments/minimal.json`、`mobile.json`、`picklift.json` 这类短配置。
- `experiments/profiles.json` 是有限、无继承的 profile 目录。每组只展开一次；
  `*_options` 是该组的显式浅覆盖，不支持隐式递归合并。
- 硬件/安装定义在 RobotAdapter/URDF；控制算法在 controller，运行时 `control_options`
  可设 `joint_servo_settings`（具名 kp/kd/effort_limit，不能超过 Adapter 上限）。
- 全部展开参数、robot spec、base配置、Task/Policy、源文件hash写入 resolved_config。
- 导航/VLM/RRT 未启用不加载其专属依赖；**核心动作碰撞检查始终保留**。

两种底盘都可用同一个入口。`experiments/mobile.json` 默认 wheel_dynamic：

```bash
"$PYTHON" -B -m src.online_manipulation "$REPO_ROOT/experiments/mobile.json" --repository-root "$REPO_ROOT" --cache-root "$RUN_ROOT/cache-mobile" --output-root "$RUN_ROOT/mobile"
```

选择 planar_kinematic 时，在自己的实验 JSON 中同时选择
`"initial_state":"mobile_planar"` 和 `"base":{"mode":"planar_kinematic","base_height_m":0.1816}`。
wheel_dynamic 对应 `mobile` / 0.1808；两者不同的是实际运动机制，不是渲染外观。
这里 HOLD 仅验证环境组装；导航和停车后双臂示例仍见第6节，不把 HOLD 当作导航验收。

可选依赖安装分层：`requirements-online.txt` 为在线、相机和场景准备；
`requirements-model-tools.txt` 另含 OBJ 转换所需 trimesh；原 `requirements.txt`
保留离线 IIWA/RRT 流程。不需要把 SceneSmith 的生成环境装进 eval。
本轮使用已有依赖完整环境，另做禁止可选模块导入的实际 reset/step 检查，
没有声称重新做了一次联网的全新虚拟环境安装。

## 1. 已发布 v0.3 的全新 clone、checkout 与安装

先进入准备存放仓库的空目录，然后在**同一个终端**中执行：

```bash
mkdir -p online-env-v0.3-work
cd online-env-v0.3-work
export WORK_ROOT="$(pwd)"

git clone https://github.com/Zihan-W/scenesmith-simple-planner-eval.git
export REPO_ROOT="$WORK_ROOT/scenesmith-simple-planner-eval"
cd "$REPO_ROOT"

git checkout online-env-v0.3
test "$(git rev-parse HEAD)" = "$(git rev-parse 'online-env-v0.3^{commit}')"
echo $?
git submodule update --init --recursive
```

`git checkout` 后处于 detached HEAD 是正常的：这里使用的是发布 tag，不是
开发分支。上述 `test` 没有输出，随后 `echo $?` 输出为 0，表示检出的 commit 正确。

创建并激活独立环境。这里显式使用官方 PyPI，因为某些镜像站没有
`drake==1.49.0`：

```bash
cd "$REPO_ROOT"
python3.11 -m venv "$REPO_ROOT/.venv"
source "$REPO_ROOT/.venv/bin/activate"
export PYTHON="$REPO_ROOT/.venv/bin/python"

"$PYTHON" -m pip install --index-url https://pypi.org/simple --upgrade pip
"$PYTHON" -m pip install --index-url https://pypi.org/simple \
  -r "$REPO_ROOT/requirements.txt"
```

需要已安装Python 3.11及venv支持。现有验证环境为 `drake 1.49.0`、`trimesh 4.11.0` 和
`manipulation 2025.10.20`。

依赖版本保持原requirements锁定值；相机时间端口fallback仅为Drake 1.49.0的
已知导出问题启用，不能随意升级Drake并假定fallback仍适用。

准备运行输出和可写缓存目录：

```bash
export QUICKSTART_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/online-env-v0.3-quickstart-XXXXXX")"
export CACHE_ROOT="$QUICKSTART_ROOT/cache"
export MPLCONFIGDIR="$CACHE_ROOT/matplotlib"
export MESA_SHADER_CACHE_DIR="$CACHE_ROOT/mesa"
export PYTHONPYCACHEPREFIX="$CACHE_ROOT/pycache"
export PYTHONPATH="$REPO_ROOT"

mkdir -p "$MPLCONFIGDIR"
mkdir -p "$MESA_SHADER_CACHE_DIR"
mkdir -p "$PYTHONPYCACHEPREFIX"
mkdir -p "$REPO_ROOT/output"
```

### 生成 Zerith OBJ

Zerith 上游模型位于 submodule 中；Drake 使用的 OBJ 是可重复生成的派生产物，
因此未纳入 Git。首次运行必须生成并检查它们：

```bash
cd "$REPO_ROOT"
"$PYTHON" "$REPO_ROOT/scripts/convert_zerith_for_drake.py"
"$PYTHON" "$REPO_ROOT/scripts/convert_zerith_for_drake.py" --check
```

成功时检查结果包含：

```text
OBJ meshes: 43
Source URDF mesh references: 70
Generated URDF mesh references: 65
Collision geometries: 53
```

v0.3继承 `b433ee6` 的指部碰撞修复，使用转换器v5：35个visual OBJ加8个
collision OBJ（共43个），53个collision。源submodule固定为
`ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`，清单位于
`models/zerith_drake/conversion_manifest.json`。转换器版本5是模型格式/生成器
版本，不是PUBLIC_API_VERSION，不应改成0.3。不要复用旧v0.2指部OBJ来省略生成。

## 2. 自包含 smoke：最小公共 API

该示例使用仓库内的最小 DMD、`ZerithRobotAdapter`、`NullTask` 和
`HoldPolicy`。为了验证真正的外部调用方式，命令从仓库外目录执行：

```bash
mkdir -p "$QUICKSTART_ROOT/public-api"
cd "$QUICKSTART_ROOT/public-api"

"$PYTHON" "$REPO_ROOT/examples/online_manipulation/public_api_client.py" \
  --repository-root "$REPO_ROOT" \
  --seed 0
```

该客户端的预期输出结构如下（浮点末位可能略有差异）：

```json
{"seed": 0, "action_type": "HoldAction", "simulation_time_s": 0.10000000000000002, "joint_count": 9, "object_names": ["portable_object"], "reward": 0.0, "terminated": false, "truncated": false, "action_status": "accepted"}
```

这条命令不需要 SceneSmith 输出目录，也不会启动相机渲染。

## 3. 自包含 smoke：机器人相机

### 外部相机公共 API 客户端

下面的外部客户端通过公共 API 读取 `head_camera`，保存一张 RGB PNG 和一份
米制 `float32` 深度 NPY：

```bash
mkdir -p "$QUICKSTART_ROOT/camera-client"
cd "$QUICKSTART_ROOT/camera-client"

"$PYTHON" "$REPO_ROOT/examples/online_manipulation/camera_public_api_client.py" \
  --repository-root "$REPO_ROOT" \
  --output-dir "$QUICKSTART_ROOT/camera-client" \
  --seed 0
```

验证结果包括：

- RGB：`(240, 320, 3)`、`uint8`；
- depth：`(240, 320)`、`float32`，单位为米；
- label：`(240, 320)`、`int16`；
- `timestamp_s=0.0`，相机更新周期为 0.05 秒；
- `action_status="accepted"`；
- 输出文件：
  `$QUICKSTART_ROOT/camera-client/head_camera_rgb.png` 和
  `$QUICKSTART_ROOT/camera-client/head_camera_depth_m.npy`。

公共访问方式如下：

```python
observation, info = env.reset(seed=0)
camera = observation.sensors["head_camera"]
image = camera.rgb
action = policy.act(observation)
observation, reward, terminated, truncated, info = env.step(action)
```

`CameraObservation.as_dict()` 和 `Observation.as_dict()` 默认只序列化元数据，
不会把图像展开成嵌套 Python list。只有显式传入
`include_images=True` 才会包含像素数组；通常应像上述示例一样把 RGB 保存为
PNG、米制深度保存为 NPY。`observation.sensors` 是只读映射。

这些内参是 **simulation camera intrinsics**；Zerith URDF 没有提供可宣称与
硬件一致的真实内参。

### 三相机几何标定

标定场景同样完全自包含，会检查头部、左腕和右腕相机的 RGB、depth、label、
光轴方向和数值投影：

```bash
mkdir -p "$QUICKSTART_ROOT/camera-calibration"

"$PYTHON" "$REPO_ROOT/scripts/validate_zerith_camera_geometry.py" \
  --repository-root "$REPO_ROOT" \
  --output-dir "$QUICKSTART_ROOT/camera-calibration"
```

成功时打印：

```json
{"calibration": true}
```

RGB、原始米制深度、深度彩色图、label 彩色图和
`calibration_metrics.json` 位于
`$QUICKSTART_ROOT/camera-calibration/calibration/`。

仓库中也保留了已经人工验收的标定图：

| 相机 | RGB | 深度彩色图 | Label 彩色图 |
| --- | --- | --- | --- |
| 头部 | [RGB](assets/zerith_camera_calibration/head_camera_rgb.png) | [depth](assets/zerith_camera_calibration/head_camera_depth_color.png) | [label](assets/zerith_camera_calibration/head_camera_label_color.png) |
| 左腕 | [RGB](assets/zerith_camera_calibration/left_wrist_camera_rgb.png) | [depth](assets/zerith_camera_calibration/left_wrist_camera_depth_color.png) | [label](assets/zerith_camera_calibration/left_wrist_camera_label_color.png) |
| 右腕 | [RGB](assets/zerith_camera_calibration/right_wrist_camera_rgb.png) | [depth](assets/zerith_camera_calibration/right_wrist_camera_depth_color.png) | [label](assets/zerith_camera_calibration/right_wrist_camera_label_color.png) |

## 4. 当前工作树的固定 PickLift：外部场景 + 版本化专家输入

大场景仍是外部依赖；不提供下载系统。必须取得原 A 场景完整依赖，不能只拿 DMD。
当前工作树已版本化 `experiments/inputs/pick_lift/` 中的两个 IK JSON、专家标定、
显式 scene_overrides 和 fingerprints；**不再需要维护者提供四个 ignored 文件**。
小红盒模型仍在 `models/zerith_pick_eval/small_red_box.sdf`，没有改变质量/摩擦/碰撞代理。

在取得场景后设置 `SCENE_ROOT`（交互输入实际目录，不把别人的绝对路径写死）：

```bash
read -r -p 'SceneSmith scene_000 完整目录: ' SCENE_ROOT
export SCENE_ROOT
test -f "$SCENE_ROOT/package.xml"
test -f "$SCENE_ROOT/combined_house/house_furniture_welded.dmd.yaml"

"$PYTHON" -B -m src.online_manipulation "$REPO_ROOT/experiments/picklift.json" --repository-root "$REPO_ROOT" --scene-root "$SCENE_ROOT" --cache-root "$RUN_ROOT/pick-cache" --output-root "$RUN_ROOT/pick" --meshcat --record-html --write-final-dmd
```

这一条命令先检查 DMD→SDF/URDF→mesh/material/image 的完整依赖，再复制到新缓存，
按已验收覆盖替换小盒及初态，生成派生 DMD、metadata、manifest，最后运行专家策略。
既不改上游，也不重新搜索专家 IK；原数值是版本化输入，不承诺搜索器逐位重现人工选择。
缓存根需是**尚不存在的新目录**；当前不提供自动缓存复用/过期清理，重复运行显式选择新根。
完整缓存可以搬家，`package://` 和内部相对引用仍有效；manifest 中绝对路径只用于源出处。

固定 profile 明确选择 `furniture_welded`。普通实验可以显式选择 `free`，但这不是
固定 PickLift 的原基准；改场景/模型/初态不能继续冒充同一专家标定。指纹或标定不匹配会报错。
B 示例场景目前缺 Wood094 的 Color/NormalGL/Roughness 三张纹理：会列出实际缺失路径，
不会静默换纹理；B 尚未完成视觉验收，不妨碍 A 场景运行。

首轮本次迁移实测：seed500、277步、27.7s、`lift_held`；抬升约10.29cm，保持约3.1s，
双指接触且无桌面支撑。这是**固定回归**，不是扰动鲁棒性验证。

产物：

- `$RUN_ROOT/pick/resolved_config.json`：展开配置、输入及参数来源；
- `$RUN_ROOT/pick/benchmark_summary.json`、`benchmark_episodes.csv`；
- `$RUN_ROOT/pick/episode_000_seed_500/{summary.json,trace.csv,simulation.html,final.dmd.yaml}`；
- `$RUN_ROOT/pick-cache/scene_metadata.json`：实际模型名/body、语义ID映射、替换后几何事实；
- `$RUN_ROOT/pick-cache/manifest.json`：源sha256及显式覆盖；
- `$RUN_ROOT/pick-cache/packages/scene/combined_house/house_furniture_welded.dmd.yaml`：派生初态。

Meshcat 在终端打印本机地址；远程使用需转发对应端口。HTML可以下载后本地浏览。
final DMD 只写 `ObservedBodySpec(write_back=True)` 指定自由物体的新位姿，不是完整机器人/关节状态快照。
原始 metadata 不是运行后的真实状态；替换模型未可靠重建的 bbox/support-surface 等字段标为未知。

可选相机审核（同样读取新缓存与版本化 IK）：

```bash
"$PYTHON" -B -m tools.calibration.validate_zerith_camera_geometry --repository-root "$REPO_ROOT" --output-dir "$RUN_ROOT/pick-cameras" --picklift-dmd "$RUN_ROOT/pick-cache/packages/scene/combined_house/house_furniture_welded.dmd.yaml" --picklift-scene-package-xml "$RUN_ROOT/pick-cache/packages/scene/package.xml" --pick-home-json "$REPO_ROOT/experiments/inputs/pick_lift/pick_home.json" --picklift-additional-package-xml "$RUN_ROOT/pick-cache/packages/zerith_pick_eval/package.xml" --neck-pitch-rad 0.0
```

旧 `scripts/run_zerith_online_example.py` 仍接受旧参数，但它转发正式 recipes/assembly；
专家路径仍须显式传入，不能找不到新输入时偷偷回读 output。

## 5. 历史v0.2全新 checkout 验证记录（不是v0.3重测）

验证基线：

- tag：`online-env-v0.2`；
- 完整运行验证所用的代码基线：`23d656c75e31391bba843e59462f50610b3969f4`；
- Zerith submodule：`ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`。

历史v0.2 tag解引用为`682ba7c1168c49d2be4e71d7ba3d9fd48308d082`，包含中文
Quickstart。上述23d656c只记录当时完整运行的代码基线；其后的两次提交仅修改
Quickstart。该历史tag本次不动。**当前v0.3发布commit**使用
`git rev-parse 'online-env-v0.3^{commit}'` 查询，第1节checkout也是v0.3。

实际结果：

| 项目 | 结果 |
| --- | --- |
| 全新 clone、checkout、submodule | 通过 |
| 全新 `.venv` 安装 | 通过 |
| 35 个 OBJ 生成与 `--check` | 通过 |
| 仓库外 state-only 公共 API | 通过，动作被接受，仿真推进到 0.1 秒 |
| 仓库外 camera 公共 API | 通过，保存 RGB PNG 与米制 depth NPY |
| 自包含三相机标定 | 通过，`{"calibration": true}` |
| 从原始 SceneSmith 场景独立生成 PickLift 派生文件 | **不通过**，旧脚本存在上述循环导入 |
| 补齐外部场景和四个派生文件后的 PickLift 相机验证 | 通过，`{"calibration": true, "picklift": true}` |
| 补齐外部资产后的固定 PickLift | 通过，`lift_held`，276 steps |

## 6. v0.3：移动底盘与双臂交接

本节随v0.3发布。安装/submodule/OBJ准备沿用上文；移动演示的场景在
`models/mobile_scene`，不依赖SceneSmith外部场景或专家IK文件。

### 6.1 最快运行两种模式

在当前仓库根、同一终端执行；直接使用`.venv`，无需activate：

```bash
cd "$REPO_ROOT"
export PYTHON="$REPO_ROOT/.venv/bin/python"
mkdir -p "$REPO_ROOT/output"
export MOBILE_OUTPUT="$(mktemp -d "$REPO_ROOT/output/mobile_handoff_XXXXXX")"

"$PYTHON" -m examples.online_manipulation.navigation_manipulation --mode wheel_dynamic --output "$MOBILE_OUTPUT/wheel" --meshcat
"$PYTHON" -m examples.online_manipulation.navigation_manipulation --mode planar_kinematic --output "$MOBILE_OUTPUT/kinematic" --meshcat
```

已完成的实际运行打印`success: True`、`parked_dual_targets_held`。各模式输出`episode_000_seed_0/{simulation.html,summary.json,trace.csv}`和benchmark汇总。实时地址看终端；远程需要转发对应端口，或下载HTML后本地打开。

| 模式 | 运动来源 | 请求0速度后的行为 |
| --- | --- | --- |
| `planar_kinematic` | 受扫掠碰撞检查的平面关节小步积分，理想规定运动 | 按加速度限幅减速，之后不再积分位移；不模拟底座反作用力 |
| `wheel_dynamic` | 浮动底座、两个驱动轮力矩及轮地接触 | 轮速伺服以0为目标制动；不是位置锁定，不保证完全静止 |

创建时选择，不能运行中热切换。程序组装用`BaseConfig(mode=...)`与`ZerithMobileRobotAdapter(spec, base_config)`，再传给`RuntimeConfig`/`make_env`。各模式原有初始高度和模型派生参数由示例工厂处理；只想切换演示时改`--mode`即可。

“到达/停车”统一指：位置误差≤3cm、最短yaw误差≤3°、实际平面速度≤0.01m/s、实际|omega|≤0.02rad/s，**同时连续满足0.5s**，均为可配置阈值；不是严格零速度。

### 6.2 一次step控制双臂、双夹爪和底盘

完整可运行的无专家文件客户端：

```bash
cd /tmp
PYTHONPATH="$REPO_ROOT" "$PYTHON" "$REPO_ROOT/examples/online_manipulation/mobile_public_api_client.py" --repo-root "$REPO_ROOT" --mode wheel_dynamic --output "$MOBILE_OUTPUT/client"
cd "$REPO_ROOT"
```

客户端已在仓库外实跑，使用公共API。核心形式如下，`spec`来自机器人配置，`env`/`obs`由构造和reset取得：

```python
from src.online_manipulation import (
    BaseVelocityAction, GripperAction, JointDeltaAction, RobotCommand,
)

action = RobotCommand(
    arms={side: JointDeltaAction((spec.arm_groups[side][0],), (-0.004,))
          for side in ("left", "right")},
    grippers={"left": GripperAction(0.060), "right": GripperAction(0.055)},
    base=BaseVelocityAction(0.06, 0.1),
)
obs, reward, terminated, truncated, info = env.step(action)
```

- 旋转关节单位rad，夹爪宽度为双指开口米制距离；底盘v单位m/s、omega单位rad/s。
- 默认step推进0.1s，伺服200Hz、物理1000Hz。组合动作共同校验，任一分量失败则新目标整体拒绝，返回`info['action_decision']`；拒绝仍推进仿真，不是状态回滚或急停。
- 未指定臂/夹爪保持旧目标；未指定base每tick都请求0速度，不能把省略理解为沿用上次非零速度。
- 各臂已有abs/delta Cartesian接口保持world表达、commanded FK增量基准、world左乘rotvec语义；abs需要每tick持续提交。导航的local frame支持不意味着新增了Cartesian局部控制模式。

### 6.3 global/local pose导航

完整世界目标示例（已有场景/收拢姿态/地图组装）：

```bash
"$PYTHON" -m examples.online_manipulation.navigate_demo --mode wheel_dynamic --output "$MOBILE_OUTPUT/world_navigation"
"$PYTHON" -m examples.online_manipulation.validate_mobile_navigation --output "$MOBILE_OUTPUT/local_navigation"
```

第二条已验证两种模式：初始yaw90°，目标用实际腕部frame表达，接收后固定世界目标并实际到达。以下为替换现有`navigator.set_goal`调用的两种写法，**二选一，不在已有控制权未释放时连续设置两次**：

```python
from src.online_manipulation import NavigationGoal, Pose

# global在本API中用world/map/odom表示，没有名为global的frame别名。
navigator.set_goal(NavigationGoal(Pose((2.8, 0, 0), (1, 0, 0, 0)), "world"), obs)
```

如需把同一个目标写成腕部局部pose，只对公开Pose做刚体数学运算，不访问Context：

```python
from pydrake.all import Quaternion, RigidTransform

name = "right_wrist_pitch_link"
reference = obs.robot.frame_poses_world[name]
X_WR = RigidTransform(Quaternion(reference.quaternion_wxyz), reference.translation_m)
X_WG = RigidTransform([2.8, 0, 0])
X_RG = X_WR.inverse() @ X_WG
local_pose = Pose(tuple(X_RG.translation()), tuple(X_RG.rotation().ToQuaternion().wxyz()))
navigator.set_goal(NavigationGoal(local_pose, name), obs)
```

四元数顺序wxyz。执行对象始终是`navigation_frame`，`frame_id`只指定目标的表达系。接收时使用完整3D变换`X_WG=X_WR(t_accept)@X_RG`并固定；后续腕部运动不会带着目标移动。动态底座有沉降/倾角，不应直接把局部z=0当作世界地面。转换后非平面目标明确拒绝，不静默投影。

### 6.4 取消、失败与控制权

Navigator只生成action，调用者仍负责`env.step(navigator.act(obs))`。

- `tracking`不是成功；`arrived`才表示连续满足配置的到达与停车阈值。
- `no_path/blocked/timeout`是不同失败，不能当作已到达。需要继续减速时仍要推进step；停止调用step只是暂停仿真，不证明已停车。
- 导航已持有控制权时调用`navigator.cancel()`，继续act/step直到`cancelled`，表示实际速度连续满足停车阈值；不是瞬间清零。
- `arrived/cancelled`后先`navigator.release()`，再通过一次`env.step(BaseVelocityAction(0, 0, control_owner="navigation", release_control=True))`交接环境中的控制权。默认owner为navigation；若配置了其他名称使用相应名称。仍被导航占用时直接发另一owner命令会拒绝。
- 无路径且尚未获得控制权时直接处理规划失败，不需要假装完成导航。已获得控制权后的受阻/超时，可走上述取消与减速流程再交接。
- 轮驱直接速度命令不自带全局避障；静态Navigator负责所用地图上的避障。运动学backend另有小步扫掠阻挡检查。这些检查不是连续动力学安全证明。

### 6.5 已验证边界与交接证据

已运行同一步双臂/夹爪、两模式直倒转、静态pose绕障、local目标固定、取消/无路径、零轮力矩隔离、移动相机和固定PickLift回归。完整端到端示例是在满足到达与停车阈值后做双臂关节动作及**空夹爪开合**，不是双臂协同抓物。

动力学四辅助轮为零摩擦滑动支撑，非真实脚轮标定；导轨固定0.4m。尚未验证动态障碍、坡地、随机场景鲁棒性、移动中精细操作、双臂闭链/协同抓取、导轨动力学。最终实现已补跑既有150项全量（581.234s，全部通过）；发布收口不重跑已充分验收的底盘机制实验，也没有重新做全新安装。

- [实现核验与曲线说明](BASE_MODE_IMPLEMENTATION_AUDIT.md)
- [进度、操作阶段最大偏差及待提交分类](mobile_manipulation_progress.md)
- `output/mobile_manipulation/manipulation_handoff_metrics.json`：已有HTML对应CSV的10Hz操作阶段统计，含参考时刻、全部样本、峰值时刻、源CSV哈希；物理子步峰值没有记录，明确缺失。

## 7. v0.3：通用入口及执行同步

配置归属、单/双臂动作语义、RobotAdapter 执行器约定、相机采样和版本历史，
统一见[通用接口契约](GENERIC_ONLINE_EXAMPLE.md)。整套变更与验收的对应表见
[progress 第15节](mobile_manipulation_progress.md#15-整套迁移收尾2026-09-08)。

### 7.1 无专家文件的最小运行

使用第6节设置的 `REPO_ROOT/PYTHON`，仓库外运行：

```bash
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export HANDOFF_OUTPUT="$(mktemp -d "$REPO_ROOT/output/closure_handoff_XXXXXX")"
cd /tmp
"$PYTHON" -B -m examples.online_manipulation.run_online --env-factory examples.online_manipulation.minimal_setup:make_env_config --policy-factory examples.online_manipulation.example_policies:make_hold_policy --output-root "$HANDOFF_OUTPUT/hold" --seeds 0 --max-steps 10 --write-final-dmd
```

使用自包含minimal DMD，NullTask没有抓取目标，`success=False`及时间截断是
预期。换策略只改 `--policy-factory`；换场景/Task只改环境工厂，不复制Runtime。

### 7.2 PickLift：相同执行链，仍依赖外部资产

先按第4节准备 `SCENE_ROOT`、`PICK_ARTIFACT_ROOT` 及完整外部资产；本轮没有
重新验收从原始场景到全部专家JSON的生成链，因此仍按非便携演示交付。
环境只需DMD；下面的专家Policy才需要专家文件。原循环导入已在共享Runtime
迁移中处理，但这不能替代整个资产生成流程的独立验收。

旧命令现在调用同一环境/策略工厂与runner，可直接运行：

```bash
cd "$REPO_ROOT"
test -f "$SCENE_ROOT/package.xml"
test -f "$PICK_ARTIFACT_ROOT/pick_home.json"
"$PYTHON" -B scripts/run_zerith_online_example.py pick-lift "$PICK_ARTIFACT_ROOT/zerith_pick_eval.dmd.yaml" --scene-package-xml "$SCENE_ROOT/package.xml" --pick-home-json "$PICK_ARTIFACT_ROOT/pick_home.json" --output-root "$HANDOFF_OUTPUT/picklift" --seed 500 --episodes 1 --max-steps 1200 --maximum-joint-step 0.1 --maximum-cartesian-joint-step 0.02 --closed-width 0 --write-final-dmd
```

本轮实际输出：`Episode 0: success=True, reason=lift_held, steps=277`。
记录在 `output/closure_audit/fixed_picklift/episode_000_seed_500/`：JSON、CSV、
final DMD；本轮不重复生成大HTML。需要另一次可视化时在命令末加
`--meshcat --record-html`，使用新的输出目录。

同一旧入口的 `hold`、`joint-step` 不再要求 `--pick-home-json`；本轮均不传
专家参数实际运行2步，输出 `success=False, reason=max_steps, steps=2`，这是
预算截断，不是抓取失败。若显式传了旧专家参数，它们在这两种策略下不读取。

旧默认 joint-step 上限0.01、closed-width 0.03继续保留；上面显式传0.1/0，
与新专家工厂一致。自定义初态用 `--environment-json`，专家标定不一致直接
报错，不再偷偷用专家home覆盖环境。旧7+1数组wrapper仍是机器人专用兼容层；
新客户端使用具名typed动作。

### 7.3 TAMP 执行前状态同步

已有 `execute_validated_joint_goal(env=env, goal_positions=...)` 自动取得
`env.get_planning_query()`；无需用户记得刷新旧查询。旧 `query/observation`
参数保留调用兼容，但不会覆盖当前执行状态。该helper要求内置环境的规划能力；
通用 `EnvironmentConfig` 不因此强制所有第三方环境必须实现Drake规划。

本轮轮驱移动后实测：旧独立查询t=0，新执行前快照t=4.5s；导航参考点从
`(0.054397, -0.000000043)`移动到`(0.144782, 0.005879)`m。传入旧查询和旧
obs也使用新快照；不是只写文档要求调用方刷新。只验证当前静态直接边执行，
没有实现完整TAMP或动态避障。
