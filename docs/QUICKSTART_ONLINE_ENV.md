# Online Environment v0.3 快速开始

需要接入自己的场景、Task 和 Policy 时，请看
[通用在线运行示例](GENERIC_ONLINE_EXAMPLE.md)。本文入口均包含在
`online-env-v0.3` 中，公共 API 版本为 `0.3`；兼容变化及能力边界见
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
  目录，以及四个未纳入 Git 的 PickLift 派生文件。只有仓库本身不能重建或
  运行该演示，详见“PickLift 非便携演示”。

## 1. 全新 clone、checkout 与安装

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

## 4. PickLift 非便携演示

### 为什么不能仅靠 v0.3 tag 运行

固定 PickLift 已经验证成功，但它目前不是自包含交付。全新 clone 还需要：

1. 一棵完整的 SceneSmith 生成场景目录，记为 `$SCENE_ROOT`。至少直接使用：
   - `$SCENE_ROOT/package.xml`；
   - `$SCENE_ROOT/combined_house/house_furniture_welded.dmd.yaml`；
   - 该 DMD 通过 package URI 引用的全部 SDF、glTF、mesh 和材质。因此只复制
     上面两个文件不够，最稳妥的做法是复制完整 `scene_000` 目录。
2. 一个由维护者提供的 PickLift 派生文件目录，记为
   `$PICK_ARTIFACT_ROOT`，必须包含：
   - `zerith_pick_eval.dmd.yaml`；
   - `task_metadata.yaml`；
   - `pregrasp_ik.json`；
   - `pick_home.json`。

仓库已跟踪小红盒模型、任务标定和 package 定义：

- `$REPO_ROOT/models/zerith_pick_eval/small_red_box.sdf`；
- `$REPO_ROOT/models/zerith_pick_eval/pick_lift_calibration.json`；
- `$REPO_ROOT/models/zerith_pick_eval/package.xml`。

派生链是“替换目标物体 → 静置验证 → 碰撞范围验证 → PREGRASP IK → PICK_HOME
搜索”。v0.2旧脚本的循环导入已随共享Runtime迁移修复，但v0.3没有重新验收
从原始SceneSmith场景到全部专家JSON的独立生成链。因此这里不提供未经验证的
全链重建命令，仍按**需要维护者提供场景与派生文件的非便携演示**交付。
环境构造本身不依赖专家文件；这些文件是固定PickLift专家Policy的前置条件。

### 检查外部资产

取得完整场景和派生文件后，在同一个终端设置两个路径。下面两个值必须替换为
你实际收到的**绝对目录**：

```bash
export SCENE_ROOT="/absolute/path/to/scene_000"
export PICK_ARTIFACT_ROOT="/absolute/path/to/zerith_pick_eval_artifacts"

test -f "$SCENE_ROOT/package.xml"
test -f "$SCENE_ROOT/combined_house/house_furniture_welded.dmd.yaml"
test -f "$PICK_ARTIFACT_ROOT/zerith_pick_eval.dmd.yaml"
test -f "$PICK_ARTIFACT_ROOT/task_metadata.yaml"
test -f "$PICK_ARTIFACT_ROOT/pregrasp_ik.json"
test -f "$PICK_ARTIFACT_ROOT/pick_home.json"
```

所有 `test` 都无输出且退出码为 0，才可以继续。

### 可选：验证真实 PickLift 场景中的相机

```bash
mkdir -p "$QUICKSTART_ROOT/picklift-camera-review"

"$PYTHON" "$REPO_ROOT/scripts/validate_zerith_camera_geometry.py" \
  --repository-root "$REPO_ROOT" \
  --output-dir "$QUICKSTART_ROOT/picklift-camera-review" \
  --picklift-dmd "$PICK_ARTIFACT_ROOT/zerith_pick_eval.dmd.yaml" \
  --picklift-scene-package-xml "$SCENE_ROOT/package.xml" \
  --pick-home-json "$PICK_ARTIFACT_ROOT/pick_home.json" \
  --picklift-additional-package-xml "$REPO_ROOT/models/zerith_pick_eval/package.xml" \
  --neck-pitch-rad 0.0
```

成功时打印：

```json
{"calibration": true, "picklift": true}
```

### 运行固定 PickLift

下面的命令启用实时 Meshcat，同时保存离线 HTML、JSON、CSV 和最终 DMD。
固定 seed 500 的验证运行约需数分钟：

```bash
export PICK_OUTPUT="$REPO_ROOT/output/quickstart_pick_lift_seed_500"

cd "$REPO_ROOT"
"$PYTHON" -B "$REPO_ROOT/scripts/run_zerith_online_example.py" \
  pick-lift \
  "$PICK_ARTIFACT_ROOT/zerith_pick_eval.dmd.yaml" \
  --scene-package-xml "$SCENE_ROOT/package.xml" \
  --pick-home-json "$PICK_ARTIFACT_ROOT/pick_home.json" \
  --output-root "$PICK_OUTPUT" \
  --episodes 1 \
  --seed 500 \
  --max-steps 1200 \
  --maximum-joint-step 0.1 \
  --maximum-cartesian-joint-step 0.02 \
  --closed-width 0 \
  --meshcat \
  --meshcat-port 7025 \
  --record-html \
  --write-final-dmd
```

终端先打印 Meshcat 地址；在普通本机或已正确转发端口的远程机器上用浏览器打开
该地址即可实时查看。若 7025 已占用，请选择一个空闲端口并同步修改
`--meshcat-port`。

v0.3最终实现的固定seed回归结束输出为（启用Meshcat时另打印地址）：

```text
Meshcat URL: http://localhost:7025
Episode 0: success=True, reason=lift_held, steps=277
```

输出位于 `$PICK_OUTPUT`：

- `benchmark_summary.json`：批次汇总；
- `benchmark_episodes.csv`：逐 episode 汇总；
- `episode_000_seed_500/simulation.html`：独立 Meshcat 录像；
- `episode_000_seed_500/summary.json`：完整 episode 指标；
- `episode_000_seed_500/trace.csv`：每个 10 Hz policy step 一行；
- `episode_000_seed_500/final.dmd.yaml`：从最终仿真状态写出的新 DMD。

输入 DMD 不会被修改。输出目录不能预先存在；重新运行时请使用新的
`PICK_OUTPUT` 或先选择另一个空目录。

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
