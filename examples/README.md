# 公共 API 示例

安装仓库后，这些示例可从仓库外执行；正式运行用 scene-eval，不由示例组装工厂。

| 文件（online_manipulation/） | 用法 |
|---|---|
| public_api_client.py | 最小构造/reset/step，只导入公共API |
| external_evaluator.py | 可复制到仓库外的Policy/evaluator factory与独立reset状态 |
| example_policies.py | Hold、具名关节增量、自定义Policy |
| camera_public_api_client.py | 图像读取、RGB/米制depth保存与同帧元数据 |
| mobile_public_api_client.py | 双臂/夹爪/底盘组合动作、三相机与规划同步 |
| cartesian_client.py + cartesian_policy.py | 世界系delta/abs与持续发送绝对目标 |
| navigate_demo.py | 公共Navigator的实际观测循环与静态避障 |
| behavior_tree_tick.py | 一个BT叶节点的一tick一步，不是完整BT |
| tamp_execution.py | 调用公共execute_validated_joint_goal，不私有修改Context |

完整命令和参数见[Quickstart](../docs/QUICKSTART_ONLINE_ENV.md)。
完整导航停车双臂基准用experiments/navigation_*.json；PickLift用experiments/picklift.json。
不再为每个策略复制minimal_setup或run_online。
