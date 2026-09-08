# 可选工具，不是在线运行依赖

- `calibration/prepare_zerith_pick_eval_scene.py`：旧场景/目标标定器；需要主动重新标定时才运行。日常入口从已验收显式覆盖生成 cache，不调用此搜索器。
- `calibration/validate_zerith_camera_geometry.py`：三相机几何及实际可见性审核。
- `audit/audit_cartesian_edge_rejections.py`：消费 trace.csv，输出拒绝动作几何证据。
- `scripts/` 的同名命令保留为薄转发；测试导入也仍兼容。
- `scripts/convert_zerith_for_drake.py`：仍是首次安装必须运行的模型转换命令；不是可删输出。
- `scripts/*noninteractive.py` 与 `src/rrt*`、`src/shortcut.py`、`src/item_locking_monitor.py`：离线 IIWA 研究流程，保留但不进入默认在线入口。
- `examples/online_manipulation/`：公共 API 使用、导航和回归演示；正式配置工厂在 `src/online_manipulation/recipes`，不再反向导入示例。

运行工具用 `PYTHONPATH="$REPO_ROOT" "$PYTHON" -m tools.calibration.validate_zerith_camera_geometry --help`，或原 `scripts/` 薄转发。
不会为清理目录删除已追踪的示例大资产、Zerith OBJ 或上游模型。
