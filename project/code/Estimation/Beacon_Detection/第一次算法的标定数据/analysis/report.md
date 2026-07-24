# 信标检测离线分析报告

## 日志扫描

- 共发现 28 份 CSV，合计 522615 行。
- 28 份都能按 32 通道解析，NaN/Inf 均为 0。
- tick 存在 1/2/3ms 混合采样和少量 900ms 级跳变；离线脚本用 tick 计算相对时间，并对异常大跳变做窗口隔离。
- 旧车端输出在 `经过20次信标灯.csv` 只触发 7 次；`实际没有碰到信标灯.csv` 误触发 1 次。

完整扫描表：

```text
analysis/output/scan_summary.csv
```

## 脚本

入口：

```powershell
python "CYT4bb7_Car/project/code/Estimation/Beacon_Detection/analysis/beacon_analysis.py"
```

只生成 CSV、不画图：

```powershell
python "CYT4bb7_Car/project/code/Estimation/Beacon_Detection/analysis/beacon_analysis.py" --no-plots
```

输出文件：

- `analysis/output/scan_summary.csv`
- `analysis/output/evaluation_summary.csv`
- `analysis/output/events_old.csv`
- `analysis/output/events_candidate_a.csv`
- `analysis/output/events_candidate_b.csv`
- `analysis/output/events_candidate_c.csv`
- `analysis/output/events_candidate_d.csv`
- `analysis/output/events_candidate_rt.csv`
- `analysis/output/figures/*.png`

## 算法路线

候选 A 是固定多特征峰值：`gyro_xy / accel_norm_error / wheel_highpass / speed`。它能抓住明显信标，但无信标急转和急刹会打出类似 IMU 峰，误触发压不稳。

候选 B 是轻量实时基础版：

- 特征：`gyro_xy_dps`、`tilt_rate_dps`、`accel_norm_error_g`、`gyro_z_abs_dps`、车体 `forward/strafe/speed`、轮速高通。
- 冲击分数：`min(gyro_xy/45, tilt_rate/45, accel_norm_error/0.12)`。
- 峰值窗口：15 个采样点滚动最大，用于消除单点错位。
- 自适应基线：历史中位数 + IQR 计算 robust z-score，避免正常噪声边界漂移。
- 上信标规则：自适应冲击、低速落下冲击、强冲击三类之一。
- 抑制规则：高速急停噪声、`gyro_z` 过大且冲击不足的转向形态不计入信标。

候选 C 在 B 基础上追加低速弱冲击补检：

- `impact_peak>=0.70`、`robust_z>=2.0`、`gyro_z<=5dps`。
- 事件前 0.6s 速度中位数 `<=1.00m/s`，事件后 0.6s 速度中位数 `<=0.80m/s`。
- 事件后 0.20s 到 0.90s 的 `accel_norm_error_g` 最大值 `<=0.18`，用于压住无信标假峰。

候选 D 是离线质量上限验证：

- 以候选 C 为主干。
- 增加高速前向补检：`impact_peak>=1.45`、原始 `impact_score>=1.20`、`robust_z>=4.0`、`gyro_z<=45dps`、峰值帧加速度误差 `<=0.20g`、轮速高通总量 `<180`。
- 增加四轮急停压灯补检：`impact_peak>=1.20`、`robust_z>=4.0`、`gyro_z<=8dps`、轮速高通总量 `>=600`、事件前速度 `>=1.40m/s`，事件后速度均值 `<=0.10m/s`。
- 增加不稳定假峰拒绝：当 `impact_peak<1.30`、尾部加速度 `>=0.24g`、事件后最小速度 `>=0.60m/s`，且当前/前/后方向三者互相不一致时，拒绝该峰。

候选 RT 是最终同步到 C 的实时可实现版本：

- enter 检测和 exit 搜索拆成双轨，enter 峰值检测持续运行，不被 `on_beacon/exit_search/cooldown` 卡死。
- enter 候选按局部峰分段，主峰阈值 `impact_peak>=0.75`，弱峰阈值 `0.35<=impact_peak<0.75`，段间间隔 450ms。
- enter 延迟 1.1s 确认，使用事件后速度、尾部加速度和轮速高通复核。
- 弱峰如果 1.1s 内遇到强主峰则让位；0.7s 内重复候选只保留质量更高的一个。
- exit 只负责闭合当前 `on_beacon`，窗口为 enter 后 280ms 到 2800ms；若没有二次冲击，用窗口内速度谷值兜底。
- 方向统一按车体速度主轴输出 `FRONT/RIGHT/LEFT/REAR`；全轮急停压灯优先使用事件前速度方向。

## 批量评估摘要

| 指标 | 旧算法 | 候选 B | 候选 C | 候选 D | 候选 RT |
| --- | ---: | ---: | ---: | ---: | ---: |
| 无信标误触发 | 1 | 0 | 0 | 0 | 0 |
| `经过20次信标灯.csv` enter/exit | 7 / 0 | 20 / 20 | 20 / 20 | 20 / 20 | 20 / 20 |
| 25 份短信标漏检 | 多数漏检 | 7 | 2 | 0 | 0 |
| 25 份短信标多报 | - | 1 | 1 | 0 | 0 |
| 短日志方向需复核 | - | 2 | 3 | 1 | 1 |

候选 RT 的剩余方向复核样本：

- `信标在正右方,直接经过,慢速4.csv`：检测到 1 次 enter，但速度主轴全程更像 `REAR`。文件名标注为 `RIGHT`，这可能需要侧向 IMU/轮速模式或人工标注补充；当前算法不使用文件名强行修方向。

完整评估表：

```text
analysis/output/evaluation_summary.csv
```

## C 端实现

- `beacon_detection.h`
  - `beacon_bump_location_t` 简化为 `UNKNOWN/FRONT/RIGHT/LEFT/REAR`。
  - `beacon_detection_data_t` 增加 `on_beacon`、`enter_event`、`exit_event`、`enter_count`、`exit_count`、`impact_baseline`、`impact_robust_z`。
- `beacon_detection.c`
  - 保留 1kHz IMU 更新 + 100Hz 轮速更新结构。
  - 1kHz 计算 `gyro_xy`、`tilt_rate`、`accel_norm_error`、32 点过去窗口冲击峰值和自适应基线。
  - 100Hz 计算车体 `forward/strafe/speed`、速度方向、轮速高通最大值和运动方向参考。
  - 删除单线程 `idle -> enter_candidate -> on_beacon -> exit_search -> cooldown` 卡脖子状态机。
  - 新增双轨实时状态：active enter 峰段、最多 4 个 pending enter、独立 exit track、最多 4 个事件锁存队列。
  - 事件输出按队列逐个锁存，避免相邻 enter/exit 在同一轮确认时互相覆盖。

C 端实时参数：

| 参数 | 值 | 作用 |
| --- | ---: | --- |
| IMU 窗口 | 32 ms | 取过去窗口最大冲击，压单点错位 |
| 启动屏蔽 | 1000 ms | 避免姿态零点和滤波启动误触发 |
| enter 主峰阈值 | 0.75 | 常规上信标候选 |
| enter 弱峰阈值 | 0.35 到 0.75 | 慢速轻压补检候选 |
| enter 段间隔 | 450 ms | 分割连续候选峰段 |
| enter 确认延迟 | 1100 ms | 等事件后速度和尾部加速度稳定 |
| pending enter | 4 个 | 支持长日志里相邻信标连续确认 |
| 后段速度起点 | 120 ms | 避开冲击峰当帧速度抖动 |
| 尾部加速度起点 | 200 ms | 用于过滤假峰尾部乱动 |
| exit 最短延迟 | 280 ms | 上信标后延迟搜索下信标 |
| exit 搜索 | 2800 ms | 搜索下信标二次冲击，超时用速度谷值闭合 |
| 事件锁存 | 120 ms | 给上层和 wifi 日志留采样窗口 |
| 事件队列 | 4 个 | 避免相邻 enter/exit 输出互相覆盖 |
| 冲击分数 | `min(gyro_xy/45, tilt_rate/45, accel_err/0.12)` | 三个 IMU 特征同时成立才算有效冲击 |
| 自适应基线 | alpha=0.001, floor=0.05 | 输出 `impact_baseline/impact_robust_z` |

## 当前风险

- Python 候选 RT 已跑通 28 份日志：无信标 0 误触发，长日志 20 次 enter / 20 次 exit，25 份短信标 0 漏检 / 0 多报。
- C 端已按 RT 思路改成双轨状态机，但没有 IAR 环境，未做车端编译；不能把离线通过吹成实车闭环通过。
- Python RT 仍有局部峰分组和离线窗口辅助，C 端用固定状态近似实现。两者原则一致，但上车日志必须复测闭环。
- 下信标仍有一部分来自“上信标后速度谷值”兜底，这类 exit 可用于状态闭合和复测线索，不能当成人工精确标注的下信标。
- `正右慢速4` 的方向仍需复核。当前速度主轴给出 `REAR`，文件名期望 `RIGHT`，不能为了漂亮结果用文件名修方向。

建议补采：无信标高速急刹/急转日志、同一信标慢速极轻碾压日志、带人工标注上/下信标时间的长日志。
