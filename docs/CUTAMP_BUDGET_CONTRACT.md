# 可选 cuTAMP 后验预算契约

正式配置默认关闭 `adaptive_postcheck`。启用必须显式给出三个正有限秒数：

```json
{"adaptive_postcheck": {"execution_reserve_s": 240, "first_pass_remaining_s": 60, "native_call_guard_s": 5}}
```

这些数字是验证配置，不是统计标定结果。继续保留原 `max_postchecks`、
`postcheck_timeout_s`、`solve_timeout_s`、精确 Drake 后验及动态成功判据。

- 时钟：全部使用既有 `time.perf_counter()` 绝对截止时间；不读取仿真时间、CUDA event 或日历时间。
- 后验软截止：`min(solve_deadline, postcheck_start + postcheck_timeout_s, run_deadline - execution_reserve_s)`。
  此预留只约束后验阶段；上游模型请求、GPU 优化和先前执行可能已经消耗时间，不能宣称预留必然可用。
- 每次启动候选前，剩余后验时间不大于 `native_call_guard_s` 就停止，不启动新的完整检查。
  这不是原生调用最大耗时保证。已经进入的 C++ 检查可能超出软截止；返回后按原逻辑标记未完成，
  不把部分证据用于执行，记录 `soft_deadline_overrun_s`。整轮既有外部进程组 watchdog 在硬截止杀进程；
  操作系统调度/清理延迟不作为可接受的规划时间。
- 剩余后验时间不大于 `first_pass_remaining_s + native_call_guard_s` 时不可逆切到首个通过策略。
  已有完整可行候选则立即停止，仍按原 rank 从此前完整可行集合选择；否则继续到第一个完整通过者。
  既有固定质量窗口可更早结束；不会重新排序、降低精确检查要求或修改 GPU 赋值。
- `candidate_selection.json` 记录策略、切换位置、剩余时间、首次通过、最终粒子、次数耗尽及超支；
  `checks.jsonl` 逐候选记录当时模式和完整性。`postcheck_result.json` 保存可执行赋值，
  正式 `tamp_trace.jsonl`、`skill_steps.jsonl`、`result.json` 独立保存后续动态结果。
  相同 solve 目录是关联键，静态通过不推断动态成功。

球集是近似碰撞加速模型。有限球集不能精确表示所有网格边缘及薄片，浅穿透漏检和保守误拒仍属
已知精度边界。当前修复不能把确定性、拟合样本覆盖率或静态间隙排序当作安全/抓取成功证明。
精确后验及执行中的接触、抬升、保持判据必须继续保留。

受限 gen_domain 的连续失败仍用于诊断；没有新增长程位置反馈或单指接触后的动作恢复。
安全停止是既有控制契约。VLM 原始输出作为审计证据保留，已满足谓词在进入执行目标之前过滤。
