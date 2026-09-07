# Online Environment v0.2 快速开始

本文面向第一次使用仓库的同事。下面的命令已经在一个全新 clone、全新
`.venv` 和 `online-env-v0.2` tag 上逐条验证。

v0.2 的交付内容分为两类：

- **自包含功能**：最小 state-only 公共 API、机器人相机公共 API、三相机
  几何标定。这些功能只依赖本仓库和 Zerith submodule，可以从全新 clone
  直接运行。
- **非便携 PickLift 演示**：还依赖一棵完整的 SceneSmith `scene_000` 场景
  目录，以及四个未纳入 Git 的 PickLift 派生文件。只有仓库本身不能重建或
  运行该演示，详见“PickLift 非便携演示”。

## 1. 全新 clone、checkout 与安装

先进入准备存放仓库的空目录，然后在**同一个终端**中执行：

```bash
mkdir -p online-env-v0.2-work
cd online-env-v0.2-work
export WORK_ROOT="$(pwd)"

git clone https://github.com/Zihan-W/scenesmith-simple-planner-eval.git
export REPO_ROOT="$WORK_ROOT/scenesmith-simple-planner-eval"
cd "$REPO_ROOT"

git checkout online-env-v0.2
test "$(git rev-parse HEAD)" = "23d656c75e31391bba843e59462f50610b3969f4"
git submodule update --init --recursive
```

`git checkout` 后处于 detached HEAD 是正常的：这里使用的是发布 tag，不是
开发分支。上述 `test` 没有输出且退出码为 0，表示检出的 commit 正确。

创建并激活独立环境。这里显式使用官方 PyPI，因为某些镜像站没有
`drake==1.49.0`：

```bash
cd "$REPO_ROOT"
python3 -m venv "$REPO_ROOT/.venv"
source "$REPO_ROOT/.venv/bin/activate"
export PYTHON="$REPO_ROOT/.venv/bin/python"

"$PYTHON" -m pip install --index-url https://pypi.org/simple --upgrade pip
"$PYTHON" -m pip install --index-url https://pypi.org/simple \
  -r "$REPO_ROOT/requirements.txt"
```

本次验证安装得到 `drake 1.49.0`、`trimesh 4.11.0` 和
`manipulation 2025.10.20`。

准备运行输出和可写缓存目录：

```bash
export QUICKSTART_ROOT="${TMPDIR:-/tmp}/online-env-v0.2-quickstart"
export CACHE_ROOT="$QUICKSTART_ROOT/cache"
export MPLCONFIGDIR="$CACHE_ROOT/matplotlib"
export MESA_SHADER_CACHE_DIR="$CACHE_ROOT/mesa"
export PYTHONPYCACHEPREFIX="$CACHE_ROOT/pycache"
export PYTHONPATH="$REPO_ROOT"

mkdir -p "$MPLCONFIGDIR"
mkdir -p "$MESA_SHADER_CACHE_DIR"
mkdir -p "$PYTHONPYCACHEPREFIX"
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
OBJ meshes: 35
Source URDF mesh references: 70
Generated URDF mesh references: 57
Collision geometries: 53
```

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

本次验证的实际输出为：

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

### 为什么不能仅靠 v0.2 tag 运行

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

理论上的派生链是“替换目标物体 → 静置验证 → 碰撞范围验证 → PREGRASP IK
→ PICK_HOME 搜索”。但是在原样的 `online-env-v0.2` checkout 中，第一批旧版
生成/验证脚本直接启动会在 `src.zerith_online_env` 与
`src.online_manipulation.adapters.zerith` 之间触发循环导入：

```text
ImportError: cannot import name 'ALL_SERVO_CONFIGS' from partially initialized module 'src.zerith_online_env'
```

因此本文不提供一套声称能从原始场景独立重建上述四个文件的假命令，也不采用
临时 import hack。v0.2 的 PickLift 应明确视为**需要维护者提供场景与派生文件
的非便携演示**。修复该生成入口属于后续代码工作，不在本次文档修正范围内。

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

本次全新 checkout 验证的结束输出为：

```text
Meshcat URL: http://localhost:7025
Episode 0: success=True, reason=lift_held, steps=276
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

## 5. 本文档的全新 checkout 验证记录

验证基线：

- tag：`online-env-v0.2`；
- commit：`23d656c75e31391bba843e59462f50610b3969f4`；
- Zerith submodule：`ddd6dc76ec9ec0a8ebd597d5576e466e11aa72be`。

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
