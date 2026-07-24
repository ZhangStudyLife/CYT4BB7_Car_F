你现在要重新研究并长期优化车端信标检测算法。请始终使用简体中文回复，先读代码和日志，再写离线分析脚本，再基于数据设计和修改算法。不要只围绕当前 `beacon_detection.c` 的阈值小修小补；如果数据证明当前算法方向不对，可以直接替换为一套新的、简单可解释、可嵌入式实时实现的检测算法。

## 2026-05-22 最终上车版本同步结论

本轮任务已按“停止继续扩展搜索，把当前离线回放最好的算法写入嵌入式 C 程序”的要求收口。当前车端 `beacon_detection.c` 已同步为 `c_stream_replay` 对应的流式峰簇状态机，保留低延迟早确认规则，并通过 29 份第二次 40 通道日志复测。

最终硬指标结果：

- 29 份日志已重新跑通。
- `c_stream_replay` 总计为 `64 enter / 64 exit / 0 false`。
- 无信标日志 `全程没有碰到信标灯.csv` 为 0 次误触发。
- 24 份短日志全部精确 `1 enter / 1 exit`。
- 4 份 10 信标长日志全部精确 `10 enter / 10 exit`。
- 所有 enter/exit 配对间隔都落在 `0.195s - 1.005s` 约束内，没有几秒后补报。
- 最大 enter 输出延迟约 `0.801s`。
- 平均 enter 输出延迟约 `0.412s`。
- `>0.5s` 的 enter 为 25 个，`>0.8s` 的 enter 为 2 个，`>1.0s` 的 enter 为 0 个。
- 当前方向识别正确率为 `27/28`；仅 `信标在车正右方-慢速.csv` 方向投票不匹配，事件检测本身正确。
- `BEACON_DETECTION_STARTUP_TICKS` 保持 `0U`，`BEACON_DETECTION_EVENT_HOLD_TICKS` 保持 `120U`，40 路 `wifi_justfloat` 保持不删减。

最终同步到 C 的核心规则：

- 使用 `0.195s - 1.005s` enter/exit 物理配对窗口，彻底移除旧算法几秒级 exit 兜底思路。
- 使用流式峰簇状态机：局部峰段、峰簇、候选、exit 窗口、早确认、事件队列、冷却。
- `BEACON_DETECTION_CLUSTER_GAP_TICKS` 保持 `750U`，这是当前 29 份日志中同时满足无信标 0、短日志 24/24、长日志 4/4 的安全点。
- `BEACON_DETECTION_CANDIDATE_PEAK_GAP_TICKS` 保持 `80U`，这是当前扫描中唯一完整通过点。
- 早确认规则包括：`early_strong`、`early_yaw_shock`、`peak_closed_medium`、`peak_closed_mid_strong`、`fast_exit_gate`、`weak_clean_tail`、`strong_tail_pose`、`very_strong_exit_reference`、`side_tail_pose`、`left_mid_tail`、`right_low_pose`、`fast_side_medium`、`rear_quiet_late`、`front_weak_late`、`weak_short_age_gate`。
- `right_low_pose` 只提前排队，不立即关簇；其它尾段姿态早确认规则确认后立即关簇并进入 750ms 冷却，避免同一次通过的尾振被拆成第二个信标。
- 新增 `window_max_wheel_highpass_count`，用于记录候选窗口内连续轮速高通最大值；不能用峰点 `max_wheel_highpass_count` 替代。

最终规则命中分布：

| 规则 | enter 数 | 无信标误触发 |
| --- | ---: | ---: |
| `early_strong` | 25 | 0 |
| `early_yaw_shock` | 9 | 0 |
| `peak_closed_medium` | 5 | 0 |
| `very_strong_exit_reference` | 4 | 0 |
| `peak_closed_mid_strong` | 3 | 0 |
| `side_tail_pose` | 3 | 0 |
| `strong_tail_pose` | 3 | 0 |
| `fast_exit_gate` | 2 | 0 |
| `left_mid_tail` | 2 | 0 |
| `peak_closed_strong` | 2 | 0 |
| `early_weak_gyro` | 1 | 0 |
| `front_weak_late` | 1 | 0 |
| `rear_quiet_late` | 1 | 0 |
| `right_low_pose` | 1 | 0 |
| `weak_clean_tail` | 1 | 0 |
| `weak_short_age_gate` | 1 | 0 |

最终已执行验证：

```text
python -m py_compile CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_rule_search.py CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_slow_event_diagnostics.py
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_rule_search.py
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_slow_event_diagnostics.py
git -C CYT4bb7_Car diff --check -- project/code/Estimation/Beacon_Detection
arm-none-eabi-gcc -fsyntax-only beacon_detection.c （按 `cyt4bb7_cm_7_0.ewp` 带 CYT4BB7CEE / CPU_BOARD_REVB / CY_CORE_CM7_0 等宏和 include path）
```

注意：当前只跑过 GCC 语法检查，没有真实跑 IAR 构建，不能声称 IAR 工程编译已通过；上车前仍需在 IAR 工程里完整编译。当前 29 份日志上的事件检测硬指标已经达标，但实车仍建议补采高强度无信标晃动、急转、结构振动、不同电量/速度的新 40 通道日志复核。

## 2026-05-22 历史记录：强尾段姿态早确认同步结论

注意：本节为过程记录，保留了 `0.967s / 0.436s / 9 个 >0.8s` 阶段的历史结果。当前最终上车版本以本文档顶部“2026-05-22 最终上车版本同步结论”为准。

本轮继续围绕第二次 29 份 40 通道日志优化 `c_stream_replay` 和车端 `beacon_detection.c`，目标是在不破坏“无信标 0 误触发、短日志每份 1/1、长日志每份 10/10”的前提下，降低慢 enter 输出延迟。

当前硬指标结果：

- 29 份日志已重新跑通。
- `c_stream_replay` 总计仍为 `64 enter / 64 exit / 0 false`。
- 无信标日志 `全程没有碰到信标灯.csv` 仍为 0 次误触发。
- 24 份短日志全部保持精确 `1 enter / 1 exit`。
- 4 份 10 信标长日志全部保持精确 `10 enter / 10 exit`。
- `BEACON_DETECTION_STARTUP_TICKS` 保持 `0U`，`BEACON_DETECTION_EVENT_HOLD_TICKS` 保持 `120U`，40 路 `wifi_justfloat` 保持不删减。

本轮已同步到 C 的安全优化：

- 新增 `weak_clean_tail` 低延迟窄门控。
- 只在候选年龄达到约 `700ms` 后生效。
- 约束条件为：`exit_age_ticks` 在 `620ms-720ms`、`exit_score` 在 `1.55-1.90`、`win_gyro_xy` 在 `25-35 dps`、`exit_accel <= 0.09g`、`max_wheel_highpass_count <= 30`、`first_speed` 在 `0.70-0.90m/s`、`peak_count >= 2`。
- 该规则主要把右边长日志第 5 个弱但干净的慢事件从约 `1.355s` 提前到约 `0.700s`，同时保持全量计数不退化。
- 新增 `strong_tail_pose` / `very_strong_exit_reference` 早确认，并且只采用“早确认后立即关簇 + 750ms 冷却”的安全版本。
- `strong_tail_pose` 约束为：候选年龄 `700ms-850ms`、候选启动时间 `>=4s`、`exit_age_ticks` 在 `580ms-780ms`、`exit_score >= 2.0`、`win_gyro_xy >= 60dps`、`exit_accel <= 0.22g`、`max_wheel_highpass_count >= 30`、roll/pitch 单轴姿态跨度 `>=3deg`。
- `very_strong_exit_reference` 约束为：候选年龄 `600ms-850ms`、候选启动时间 `>=3s`、`exit_score >= 3.20`、`win_gyro_xy >= 90dps`、`exit_accel <= 0.24g`。
- 早确认后必须关闭当前实时候选并进入 `BEACON_DETECTION_CLUSTER_GAP_TICKS=750ms` 冷却；直接加入强尾段规则但不关簇，会把少数短日志拆成 `2 enter / 2 exit`，不能上车。

当前延迟指标：

- 最大 enter 输出延迟从约 `1.273s` 继续降到约 `0.967s`。
- 平均 enter 输出延迟约 `0.436s`。
- `>0.5s` 的 enter 仍有 26 个。
- `>0.8s` 的 enter 从 16 个降到 9 个。
- `>1.0s` 的 enter 从 4 个降到 0 个。
- `>1.1s` 的 enter 从 2 个降到 0 个。

仍未同步到 C 的实验规则：

- “强尾段早确认 + 0.55s 尾振吸收”不安全；完整逐 tick 回放会破坏长日志计数或引入多报，不能上车。
- 单纯放开 exit 强度、缩短 cluster gap、缩短 candidate peak gap 仍会引入短日志多报、长日志漏报或无信标误触发，不能同步到 C。

当前结论：

- 事件数量、无信标误触发和 enter/exit 配对稳定性已经达到本批日志验收硬指标。
- “0.5s 体感立刻响”仍未完全达标；平均延迟已进入 `0.5s` 以内，但尾部仍有 9 个 `>0.8s` 慢 enter。
- 本轮已经消除 `>1.0s` 的 enter 输出延迟；下一步若继续压到 0.5s 以内，需要更多无信标强晃动、急转、结构振动负样本支撑，不能只靠放宽阈值。

本轮新增/更新的离线脚本：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_rule_search.py
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_slow_event_diagnostics.py
```

本轮已执行验证：

```text
python -m py_compile CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_rule_search.py
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_rule_search.py
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_slow_event_diagnostics.py
git -C CYT4bb7_Car diff --check -- project/code/Estimation/Beacon_Detection
arm-none-eabi-gcc -fsyntax-only CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.c
```

注意：当前只跑过 GCC 语法检查，没有真实跑 IAR 构建，不能声称 IAR 编译已通过；上车前仍需在 IAR 工程里完整编译并实车复采 40 通道日志。

## 2026-05-21 历史记录：当前阶段落地结论

注意：本节为早期过程记录，保留了阶段性实验、失败规则和历史延迟指标。当前最终上车版本以本文档顶部“2026-05-22 最终上车版本同步结论”为准。

本阶段已从第二次 29 份 40 通道日志出发，完成旧算法基线复现、候选算法批量比较、实时可上车版本 `candidate_g` 设计，并已把车端 `beacon_detection.c` 主流程切到新的实时峰簇确认状态机。

当前车端核心策略：

1. 不再把单个峰值立即当成上信标灯；先建立接触候选。
2. enter/exit 物理配对只允许落在 `0.195s - 1.005s` 内，不能再使用旧的 `2.8s exit` 超时兜底。
3. strong 形态一旦在配对窗口内闭合就立即确认；`yaw_shock` 在配对窗口内一满足也立即确认；`strong` / `mid_strong` / `medium` / `yaw_shock` 在 exit 窗口更新后逐 tick 尝试早确认；`weak_gyro` / `weak_short` 在候选年龄达到 `0.8s` 后才允许提前确认，避免更早的弱特征把无信标晃动误报成信标。
4. 如果候选超过 `1.005s` 仍未确认，只有遇到当前新峰分数 `>= 2.35` 时才关闭旧候选并重新建簇，避免把几秒后的强峰回报成早期弱峰。
5. 已早确认的峰簇后续不会重复上报，避免同一段连续晃动被拆成两个信标事件。
6. 蜂鸣器仍由 `g_beacon_detection.bump_detected` 触发，所以 enter 和 exit 都可能响；每次响时必须同时看 `enter_event` / `exit_event` 区分是上信标灯还是下信标灯。蜂鸣器响代表算法确认了一个事件，不等价于毫秒级真实接触瞬间。
7. `wifi_justfloat` 保持 40 路，已经包含原始 IMU、滤波 IMU、欧拉角、四轮速度、归一化物理量、tilt、检测结果、enter/exit/on_beacon、鲁棒冲击量、车体速度和轮速高通特征，足够支撑后续继续离线调参。

已验证结果：

- `analysis/beacon_second_pass.py` 已跑通 29 份日志。
- 无信标日志 `全程没有碰到信标灯.csv`：`c_stream_replay` 为 0 次误触发。
- 24 份短日志：`c_stream_replay` 全部为 1 次 `enter_event` 和 1 次 `exit_event`。
- 4 份 10 信标长日志：`c_stream_replay` 全部为 10 次 `enter_event` 和 10 次 `exit_event`。
- 本批 `c_stream_replay` 没有超过 `1.005s` 的 enter-exit 物理配对。
- 已消除本批日志中的几秒级慢确认：`c_stream_replay` 当前总计 `64 enter / 64 exit`，无信标 0 误触发；24 份短日志全部精确 `1 enter / 1 exit`，4 份 10 信标长日志全部精确 `10 enter / 10 exit`。当前最大确认/输出延迟约 `0.967s`，enter 平均输出延迟约 `0.436s`；仍有 26 个 enter 超过 `0.5s`、9 个超过 `0.8s`，但已经没有 `>1.0s` 和 `>1.1s` 的 enter。
- 已验证三类可保留的低延迟优化：`yaw_shock` 不再等到 `1.005s` 到期或峰段闭合，只要在配对窗口内满足 `early_max_gyro_z_abs_dps >= 50.0` 且 `win_gyro_xy_dps >= 45.5` 就立即确认；`strong` / `mid_strong` / `medium` / `yaw_shock` 在 exit 窗口更新后逐 tick 尝试早确认；`weak_gyro` / `weak_short` 在候选年龄达到 `0.8s` 后允许提前确认。29 份日志保持 0 误触发、短日志 24/24、长日志 4/4。
- 已尝试过“1.005s 到期强制裁决/失败即重开候选”和“0.8s 之前更激进的弱特征先报 enter”的低延迟方向：前者会导致短日志漏检、长日志计数不达标或无信标误触发；后者在当前 IMU/轮速/姿态快照特征上找不到同时命中慢样本且 0 命中无信标/被淘汰簇的安全阈值组合，不能直接上车。另测 `EVENT_HOLD_TICKS=80/60`，事件数量仍安全但 enter 延迟无收益，所以车端仍保留 `120ms`，保证 100Hz 蜂鸣器循环稳定采到事件。
- 已新增一条很窄的 `fast_exit_gate` 低延迟门控：仅在候选年龄 `250ms-320ms`、exit 间隔 `205ms-320ms`、`max_score >= 1.05`、`exit_score >= 1.20`、`win_gyro_xy >= 45.0`、`exit_accel <= 0.12`、`max_wheel <= 60`、`first_speed <= 0.80` 时提前确认。该规则保持 29 份日志 `64/64/0`，主要把右边长日志第 8 个事件从约 `1.241s` 延迟提前到约 `0.250s`。
- 已新增 `strong_tail_pose` / `very_strong_exit_reference` 强尾段早确认：利用 0.6s-0.85s 尾段的 exit 强度、gyro_xy、roll/pitch 姿态跨度和轮速高通边界提前确认；早确认后立即关簇并进入 750ms 冷却，避免同一次通过的尾振被拆成第二个信标。该组合保持 29 份日志 `64/64/0`，把剩余 4 个 `>1.0s` 慢 enter 全部压到 `0.601s-0.702s` 区间。
- 继续尝试过“强 exit 主导提前闭合”：以 exit 峰 `score >= 1.7` 且窗口 `gyro_xy >= 60` 为代表的规则，能把部分慢样本提前到 0.2s-0.7s，并且不打穿无信标日志；但完整状态机回放会在有信标短日志和长日志中把尾振/相邻弱簇多报成新信标，事件数膨胀到 76 enter / 76 exit，不能上车。后续如果继续利用强 exit，必须增加“同一通过后的尾振抑制/与前一事件最小间隔/簇归属”约束，而不能只看 exit 峰强度。
- 继续复核过两条更窄的低延迟方向，均不能上车：
  - “pending 峰段提前闭合”：在当前候选已有 exit 证据、后续 pending 峰临时满足 strong/mid/medium 形态时提前入队。该规则能把部分慢样本提前，但会在 `信标在车正前方-快速2.csv` 这类日志中把真实信标前的晃动多报成一对 enter/exit。原因是峰段未闭合时 exit 峰还不稳定，临时合法不等于最终合法。
  - “三档 exit 提前闭合”：按 strong_exit / mid_exit / quiet_exit 组合约束 gap、exit_score、accel、wheel、gyro。该规则仍会让无信标日志出现 2 次误触发，并让部分短日志/长日志多报。因此当前 29 份日志下，不能继续靠 exit 强度规则压 enter 延迟。
- 额外扫描过 `BEACON_DETECTION_CLUSTER_GAP_TICKS` 对延迟的影响：从 750ms 降到 700ms 后无信标仍为 0，但 4 份长日志只剩 3/4 达标；650ms 开始无信标出现误触发；继续降低虽能把最大 enter 延迟压到 0.832s、0.471s 等，但短日志/长日志和无信标都会被打穿。因此当前 `750ms` 是这批数据上第一个同时满足无信标 0、短日志 24/24、长日志 4/4 的安全点，不能为了体感延迟直接缩短。
- 额外扫描过 `BEACON_DETECTION_CANDIDATE_PEAK_GAP_TICKS` 对延迟的影响：80ms 是当前唯一完整通过点。增大到 90/100ms 会增加多报和延迟；缩短到 70ms 以下会让长日志漏检或无信标误触发，虽然部分延迟指标会下降，但不满足主验收标准。
- 又复核过“pending strong/mid 提前闭合”：只允许 pending 峰段的 strong/mid_strong，不允许 medium/weak。该规则无信标仍为 0，但会让若干短日志从 1/1 变成 2/2，说明真实信标外的强尾振仍可能被提前认成新信标，不能上车。
- 继续离线验证过“强 exit 提前确认 + 0.50s-0.68s 尾振吸收”组合。真实相邻事件中 `next_enter - prev_exit` 最小约 `0.70s`，所以尾振吸收窗口不能粗暴拉长；本轮扫描的多组组合没有一组能同时保持 29 份日志全量计数正确和无信标 0 误触发，因此不同步到 C。最终采用的是“强尾段早确认后关簇冷却”，不是宽尾振吸收。
- 已新增慢样本诊断脚本 `analysis/beacon_slow_event_diagnostics.py`，输出 `slow_enter_diagnostics.csv` 和 `slow_enter_nearby_peaks.csv`。诊断结果表明：多数 `>0.8s` 慢 enter 需要等 0.6s-0.9s 后的 exit/尾段强证据才与无信标晃动拉开边界；其中剩余 `>1s` 样本里，右边长日志第 5 个事件在 1.005s 配对窗口内没有足够强的 exit 早确认证据，后边长日志第 10 个、左慢速2、右边长日志第 2 个、前中速则能被强 exit 特征提前，但简单加入会引入多报风险。
- 结论：当前安全版本已经消除 `>1.0s` enter 输出延迟，但仍保留少量 `0.8s-0.97s` 弱峰确认延迟；不能为了 0.5s 体感目标把无信标 0 误触发和每个信标只报一次的主目标打穿。若继续追求 0.5s，需要补充更多无信标高强度晃动、急转、结构振动负样本，或引入更强的簇归属/事件间隔状态，而不是单纯放宽阈值。
- 方向识别仍是二级目标，目前仅 `信标在车正右方-慢速.csv` 方向投票不匹配，事件检测本身达标。

当前输出文件：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\output_second_pass\scan_and_old_baseline.csv
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\output_second_pass\candidate_events.csv
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\output_second_pass\candidate_peaks.csv
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\output_second_pass\report.md
```

已执行验证：

```text
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py
python CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_slow_event_diagnostics.py
python -m py_compile CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_second_pass.py CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis\beacon_slow_event_diagnostics.py
git diff --check -- CYT4bb7_Car\project\code\Estimation\Beacon_Detection
arm-none-eabi-gcc -fsyntax-only beacon_detection.c （按 `cyt4bb7_cm_7_0.ewp` 带 CYT4BB7CEE / CPU_BOARD_REVB / CY_CORE_CM7_0 等宏和 include path）
```

注意：当前机器没有真正跑 IAR 构建，不能声称车端 IAR 编译已通过。GCC 语法检查已能确认本轮 `beacon_detection.c` 没有再出现 `exit_peak_location` / `exit_min_speed_location` 被误写成结构字段这类错误。下一步必须在 IAR 里实编，并上车采集新 40 通道日志复核；同时继续把 `c_stream_replay` 残余的 `0.8s-0.97s` 弱峰确认延迟压到 0.5s 体感目标内。

## 项目路径

```text
D:\HDUASC-SmartCar-21st-FlyOverMinefield
```

重点代码：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.c
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\beacon_detection.h
```

第二次标定日志目录：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\第二次算法的标定数据
```

数据提交节点：

```text
6cdceaeb9484627ef0141bf7eca2c91e45e17d3f
```

该节点提交信息为：`运行了29次,记录了29份日志`。

## 信标灯物理信息

信标灯是一个整体的小圆形凸起，不是长坡道：

- 外形：圆形盘状。
- 外径：260 mm。
- 边缘厚度：1.8 mm。
- 中心最高处总厚度：15 mm。
- 通过过程很短。正常实车经过时，上信标灯到下信标灯不应该拖到几秒后；如果算法几秒后才报事件，应按失败处理。

方向语义：

- `信标在车正前方`：上信标灯时，车体应先出现抬头趋势；下信标灯时，车体应出现低头/降头趋势。
- `信标在车正后方`：上/下信标灯的俯仰变化方向与正前方相反。
- `信标在车正右方`：上信标灯时，车体右侧应上抬；下信标灯时，车体应向右侧下落/倾斜。
- `信标在车正左方`：上/下信标灯的横滚变化方向与正右方相反。
- 文件名里的方向描述的是上信标灯瞬间车体与信标灯的相对位置，不代表车只沿单一轴运动。

## 总目标和优先级

最终目标是让车端 C 代码稳定识别每一次经过信标灯的事件。优先级固定如下：

1. 无信标日志必须 0 误触发。
2. 准确、及时识别上信标灯 `enter_event`。
3. 准确、及时识别下信标灯 `exit_event`，并和对应 `enter_event` 配对。
4. 正确维护 `on_beacon` 状态，不能长期卡住，也不能几秒后才补报。
5. 在前面目标满足后，再优化方向识别：`FRONT / RIGHT / LEFT / REAR`。

方向识别是二级目标。能精确识别最好，但不能为了方向结果牺牲“是否正在信标上”和“上/下信标时刻”的稳定性。

## 日志数据集

本轮必须以 `第二次算法的标定数据` 里的 29 份 CSV 为主验收集，不再以旧 28 份日志作为主目标。

29 份日志分三类：

1. 短日志：整份日志保证只压过 1 次信标灯。文件名写明信标在车的正前方、正后方、正左方、正右方，以及快速/中速/慢速工况。
2. 长日志：4 份，每份经过 10 个信标灯，方向分别全在车前边、后边、左边、右边。
3. 无信标日志：`全程没有碰到信标灯.csv`，全程不应检测到任何信标事件。

### 已扫描到的日志概况

这 29 份 CSV 均为 40 通道日志。行数如下：

| 文件名 | 行数 | 通道数 |
| --- | ---: | ---: |
| `全程没有碰到信标灯.csv` | 469344 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车前边.csv` | 45490 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车后边.csv` | 56108 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车右边.csv` | 96172 | 40 |
| `此日志一共经过10个信标灯,信标方向都在车左边.csv` | 51097 | 40 |
| 其余 24 份短日志 | 每份约 3078 到 18984 行 | 40 |

## 40 通道顺序

日志来自车端 `wifi_justfloat`，通道顺序必须按以下定义解析：

1. `tick_1000us_cnt`
2. `ICM42688.acc_x`
3. `ICM42688.acc_y`
4. `ICM42688.acc_z`
5. `ICM42688.gyro_x`
6. `ICM42688.gyro_y`
7. `ICM42688.gyro_z`
8. `g_imufilter_1000hz.accx`
9. `g_imufilter_1000hz.accy`
10. `g_imufilter_1000hz.accz`
11. `g_imufilter_1000hz.gyrox`
12. `g_imufilter_1000hz.gyroy`
13. `g_imufilter_1000hz.gyroz`
14. `g_euler.roll`
15. `g_euler.pitch`
16. `g_euler.yaw`
17. `left_front`
18. `right_front`
19. `left_rear`
20. `right_rear`
21. `accel_x_g`
22. `accel_y_g`
23. `accel_z_g`
24. `gyro_x_dps`
25. `gyro_y_dps`
26. `gyro_z_dps`
27. `sample.tilt_deg`
28. `g_beacon_detection.bump_detected`
29. `g_beacon_detection.confidence`
30. `g_beacon_detection.location`
31. `g_beacon_detection.wheel_mask`
32. `g_beacon_detection.score`
33. `g_beacon_detection.enter_event`
34. `g_beacon_detection.exit_event`
35. `g_beacon_detection.on_beacon`
36. `g_beacon_detection.impact_robust_z`
37. `g_beacon_detection.speed_mps`
38. `g_beacon_detection.vel[0]`
39. `g_beacon_detection.vel[1]`
40. `g_beacon_detection.wheel_highpass_count`

离线脚本必须兼容旧 32 通道日志和新 40 通道日志，但本轮主评估必须使用上述 29 份 40 通道日志。

## 当前旧算法基线问题

必须先评估当前算法，不能跳过。已知当前车端输出在第二次标定数据上存在明显问题：

| 文件/类别 | 当前表现 |
| --- | --- |
| `全程没有碰到信标灯.csv` | `bump_detected` 有 12 次上升沿，`enter_event` 8 次，`exit_event` 8 次，属于严重误触发。 |
| `此日志一共经过10个信标灯,信标方向都在车前边.csv` | `enter_event` 10 次，`exit_event` 10 次，但 `bump_detected` 14 次，存在重复/额外蜂鸣器触发。 |
| `此日志一共经过10个信标灯,信标方向都在车后边.csv` | `enter_event` 10 次，`exit_event` 10 次，但 `bump_detected` 17 次，存在重复/额外蜂鸣器触发。 |
| `此日志一共经过10个信标灯,信标方向都在车右边.csv` | `enter_event` 6 次，`exit_event` 6 次，漏检严重。 |
| `此日志一共经过10个信标灯,信标方向都在车左边.csv` | `enter_event` 4 次，`exit_event` 4 次，漏检严重。 |
| 多个短日志 | 出现应为 1 次却检测 0 次、2 次、4 次的情况。 |

旧算法不能作为最终方案。后续必须重点解决：

- 平地晃动、急转、结构振动导致的无信标误触发。
- 真正上信标灯时漏检。
- 下信标灯未及时识别，几秒后才补报。
- `bump_detected` 同时代表 enter/exit，导致蜂鸣器含义混乱。
- 左右方向和横滚相关工况明显漏检。

## 验收标准

### 事件数量

- 短日志：每份期望 1 次 `enter_event` 和 1 次 `exit_event`。
- 10 信标长日志：每份期望 10 次 `enter_event` 和 10 次 `exit_event`。
- 无信标日志：期望 0 次 `enter_event`、0 次 `exit_event`、0 次等价蜂鸣器触发。

### 事件时序

- 同一次信标的 `enter_event -> exit_event` 配对间隔必须在 `0.2s - 1.0s` 内。
- 超过 `1.0s` 才出现 `exit_event`，或者几秒后才响蜂鸣器，直接判为失败。
- 检测时刻应贴近真实冲击/姿态特征峰，实车感知尺度上要求在 `0.5s` 内输出。
- 不追求毫秒级人工标注，但不能出现用户可明显感知的延迟。

### 方向识别

- 方向输出只允许：
  - `UNKNOWN`
  - `FRONT`
  - `RIGHT`
  - `LEFT`
  - `REAR`
- 短日志方向标签来自文件名。
- 方向识别为二级指标，报告中必须统计正确率，但不能为了方向牺牲事件检测和误触发控制。

### 蜂鸣器语义

- 当前实车允许上信标灯和下信标灯都响。
- 但算法评估不能只看 `bump_detected`。必须区分：
  - `enter_event`：上信标灯。
  - `exit_event`：下信标灯。
  - `on_beacon`：当前是否认为车正在信标上。
- 如果保留上下都响，报告必须说明每一次蜂鸣器对应 enter 还是 exit。不能让“响了一下”成为含糊事件。

## 离线分析要求

必须优先用 Python 做离线分析，严禁一上来直接改 C。

建议使用：

- `pandas`
- `numpy`
- `scipy`
- `matplotlib`
- 必要时可用 `numba`、`parquet/npz` 缓存提升速度。

必须建立或更新：

```text
CYT4bb7_Car\project\code\Estimation\Beacon_Detection\analysis
```

该目录用于放：

- 数据加载脚本。
- 批量评估脚本。
- 事件明细 CSV。
- 扫描摘要 CSV。
- 分析报告。
- 关键曲线图。
- 缓存文件。

注意：

- 不要破坏原始 CSV。
- 不要覆盖原始日志。
- 可以覆盖 `analysis/output` 下由脚本生成的结果文件。

## 必须分析的曲线和特征

对典型有信标、无信标、误检、漏检日志，必须画图或输出窗口统计：

- `gyro_x_dps / gyro_y_dps / gyro_z_dps`
- `gyro_xy = sqrt(gyro_x_dps^2 + gyro_y_dps^2)`
- `accel_norm_error = abs(norm(accel_x_g, accel_y_g, accel_z_g) - 1)`
- `roll / pitch / yaw`
- `tilt_deg`
- `tilt_rate_dps`
- 四轮速度：`left_front / right_front / left_rear / right_rear`
- 车体速度：`forward / strafe / speed`
- 轮速高通特征：`wheel_highpass_count`
- 当前车端输出：`bump_detected / enter_event / exit_event / on_beacon / score / location`
- 新算法候选输出：候选峰、enter、exit、配对关系、方向、置信度。

必须重点比较：

- 快速、中速、慢速下冲击峰值和持续时间。
- 前、后、左、右方向下俯仰/横滚特征差异。
- 有信标事件与无信标平地晃动、急转、结构振动的边界。
- 当前旧算法误触发点附近的 IMU 和轮速特征。
- 漏检样本附近是否存在更可靠的低幅特征。

## 真值和自分析要求

本轮不要求先人工逐帧标注所有 enter/exit 时间。要求算法先自分析，因为数据中的信标特征应当足够明显。

实现方式：

1. 根据文件名确定期望事件数、方向、速度等级。
2. 对每份日志自动寻找候选 enter/exit 峰。
3. 按物理约束配对：同一次信标的 enter/exit 间隔优先落在 `0.2s - 1.0s`。
4. 输出每个候选事件前后至少 `1.5s` 的关键特征摘要。
5. 对无法满足数量或时间约束的样本，自动列入“需要人工复核”列表。
6. 人工复核只针对争议样本，不作为前置条件。

## 新算法设计方向

不要只调几个阈值。至少设计并批量比较 2 套候选算法。

候选方向可以包括但不限于：

1. 多特征窗口峰值检测：
   - 结合 `gyro_xy`、`tilt_rate_dps`、`accel_norm_error_g`、`wheel_highpass_count`。
   - 使用短窗口峰值而不是单点阈值。
2. 自适应噪声基线：
   - 在无信标和正常跑动中估计噪声边界。
   - 使用 robust z-score 或类似抗异常指标。
3. enter/exit 分离：
   - 不允许一个 `bump_detected` 混掉上下信标。
   - enter 检测负责开始，exit 检测负责闭合。
4. 物理时间约束状态机：
   - `idle -> enter_candidate -> on_beacon -> exit_candidate -> cooldown`
   - 或者等价但更稳的状态机。
   - 必须防止几秒后才把 exit 当成当前信标的闭合事件。
5. 方向判定：
   - 正前/正后重点看 pitch/俯仰和前后速度。
   - 正左/正右重点看 roll/横滚和横向速度。
   - 方向不确定时允许 `UNKNOWN`，不要硬判导致错误传播。

## C 端实现约束

最终 C 代码必须符合嵌入式实时要求：

- 不能引入动态内存。
- 不能引入大数组。
- 不能引入高阶复杂模型。
- 不能把离线 Python 复杂逻辑无脑搬进 C。
- 可以使用少量静态状态变量。
- 可以继续保持当前 `1kHz IMU 更新 + 100Hz 轮速更新` 结构。

建议职责：

- `beacon_detection_update_1000HZ()`：
  - 更新 IMU 特征。
  - 推进窗口峰值。
  - 推进 enter/exit 状态机。
  - 锁存事件。
  - 输出 40 路调试日志。
- `beacon_detection_update_100HZ()`：
  - 更新四轮速度。
  - 更新车体 `forward / strafe / speed`。
  - 更新轮速高通特征。
  - 更新方向辅助信息。

`beacon_detection_data_t` 至少应保留：

- `bump_detected`
- `on_beacon`
- `enter_event`
- `exit_event`
- `confidence`
- `location`
- `wheel_mask`
- `hold_ticks`
- `event_count`
- `enter_count`
- `exit_count`
- `score`
- `impact_baseline`
- `impact_robust_z`
- `speed_mps`
- `vel[2]`
- `gyro_xy_dps`
- `gyro_z_abs_dps`
- `tilt_rate_dps`
- `tilt_deg`
- `accel_norm_error_g`
- `wheel_highpass_count`

## 输出成果要求

最终必须输出以下成果：

1. 日志扫描结果：
   - CSV 数量。
   - 每个文件行数。
   - 通道数。
   - NaN/Inf。
   - tick 跳变/丢帧情况。
2. 离线脚本路径和运行方法。
3. 当前旧算法基线评估：
   - 每份日志 `bump_detected / enter_event / exit_event / on_beacon` 统计。
   - 漏检、误检、重复触发、延迟闭合。
4. 新候选算法评估表：
   - 文件名。
   - 期望 enter 数。
   - 期望 exit 数。
   - 实际 enter 数。
   - 实际 exit 数。
   - 误触发数。
   - enter-exit 配对间隔。
   - 方向识别结果。
   - 备注。
5. 参数选择依据：
   - 为什么能覆盖快速/中速/慢速。
   - 为什么能压住无信标平地晃动。
   - 为什么不会几秒后才补报。
6. 最终算法说明：
   - 使用哪些特征。
   - 窗口大小。
   - 阈值。
   - 状态机逻辑。
   - enter/exit 配对逻辑。
   - 方向判定逻辑。
7. C 代码改动总结：
   - `beacon_detection.c`
   - `beacon_detection.h`
   - 如需调整蜂鸣器触发处，也必须说明原因。
8. 最终风险和后续补采建议。

## 工作顺序

严格按以下顺序做：

1. 读 `AGENTS.md` 和现有 `beacon_detection.c/h`。
2. 扫描 `第二次算法的标定数据` 下 29 份 CSV。
3. 建立或更新 Python 数据加载器，兼容 32/40 通道。
4. 复现当前车端输出，先评估旧算法。
5. 针对无信标误检日志画图，找误触发边界。
6. 针对短日志漏检/多检样本画图，找 enter/exit 共同特征。
7. 针对 4 份 10 信标长日志评估连续事件稳定性。
8. 设计至少 2 套新候选算法并批量评估。
9. 选择最稳的一套，说明参数依据。
10. 转成 C 端实时实现。
11. 用同一批 29 份日志复测。
12. 输出报告和代码改动总结。

## 禁止事项

- 禁止直接从当前算法阈值小修小补开始。
- 禁止不读日志就改 C。
- 禁止只看有信标日志，不看无信标误触发。
- 禁止只追求方向识别，忽略 enter/exit 时序。
- 禁止把几秒后的补报当成正确检测。
- 禁止覆盖原始 CSV。
- 禁止执行 `git commit`、`git push`、`git reset`、`git checkout`，除非用户明确要求。
- 没有 IAR 环境时，不要声称车端编译通过。

## 最终必须明确回答

每次阶段性完成后，必须明确说明：

- 是否跑通 29 份日志。
- 无信标日志是否 0 误触发。
- 4 份 10 信标长日志分别识别到多少次 enter/exit。
- 短日志是否每份都识别到 1 次 enter/exit。
- 是否存在超过 `1.0s` 的 enter-exit 闭合。
- 当前方向识别正确率。
- 哪些样本仍需人工复核。
- 后续还需要补采哪些实车日志。
