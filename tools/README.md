# 可选工具

日常使用 scene-eval + experiments，不经本目录组装。
工具可以依赖正式库，正式src不能反向导入工具。

| 入口 | 独立用途 |
|---|---|
| scripts/convert_zerith_for_drake.py | 首次模型准备/转换一致性--check |
| scripts/visualize_dmd_scene.py | 任意DMD的场景检查 |
| scripts/visualize_zerith_left_arm.py | 模型安装与关节零位的运动学检查，不是动力学 |
| tools.calibration.validate_zerith_camera_geometry | 三相机数值投影及真实场景可见性 |
| tools.validation.validate_zerith_collision_proxies | 腕部/手指几何行程扫描 |
| tools.audit.audit_cartesian_edge_rejections | CSV中的动作拒绝逐几何诊断 |
| tools.audit.audit_base_modes | 两种底盘机制及独立零驱动力矩对照 |
| tools.audit.validate_mobile_navigation | local目标、到达窗口和受阻专项验证 |
| iiwa-baseline | 可选原IIWA四阶段离线基线 |

安装后可从仓库外运行 `python -m tools.calibration.validate_zerith_camera_geometry --help`。
其余tools同理；scripts使用显式文件路径。审计可能访问仿真内部以记录物理证据，
不是Policy应模仿的接口。完整基线见 [Quickstart](../docs/QUICKSTART_ONLINE_ENV.md)。

旧Zerith字典兼容层、临时搜索、阶段回放和重复CLI已删除，
不再全部搬进legacy；需要历史源码时从cb79ba8检查点检出。
当前已验收策略输入仍版本化，未重新搜索替代。新目标专家的自动再标定流程
不作为此次交付能力；相机/模型的有效独立检查保留。
