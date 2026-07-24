#!/usr/bin/env python3
"""离线评估车端信标检测日志。

脚本只读取 Beacon_Detection 目录下的 CSV，不修改原始日志。
输出：
  - output/scan_summary.csv：日志扫描与基础校验
  - output/evaluation_summary.csv：旧算法、离线候选与实时候选批量评估
  - output/events_*.csv：候选算法逐事件明细
  - output/figures/*.png：典型日志关键曲线
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse
import math
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


CHANNELS_32 = [
    "tick_1000us_cnt",
    "ICM42688.acc_x",
    "ICM42688.acc_y",
    "ICM42688.acc_z",
    "ICM42688.gyro_x",
    "ICM42688.gyro_y",
    "ICM42688.gyro_z",
    "g_imufilter_1000hz.accx",
    "g_imufilter_1000hz.accy",
    "g_imufilter_1000hz.accz",
    "g_imufilter_1000hz.gyrox",
    "g_imufilter_1000hz.gyroy",
    "g_imufilter_1000hz.gyroz",
    "g_euler.roll",
    "g_euler.pitch",
    "g_euler.yaw",
    "left_front",
    "right_front",
    "left_rear",
    "right_rear",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "sample.tilt_deg",
    "g_beacon_detection.bump_detected",
    "g_beacon_detection.confidence",
    "g_beacon_detection.location",
    "g_beacon_detection.wheel_mask",
    "g_beacon_detection.score",
]

CHANNELS_40 = CHANNELS_32 + [
    "g_beacon_detection.enter_event",
    "g_beacon_detection.exit_event",
    "g_beacon_detection.on_beacon",
    "g_beacon_detection.impact_robust_z",
    "g_beacon_detection.speed_mps",
    "g_beacon_detection.vel_forward_mps",
    "g_beacon_detection.vel_strafe_mps",
    "g_beacon_detection.wheel_highpass_count",
]

LOCATION_NAMES = {
    0: "UNKNOWN",
    1: "FRONT",
    2: "RIGHT",
    3: "LEFT",
    4: "REAR",
}

LOCATION_CN = {
    "UNKNOWN": "未知",
    "FRONT": "前",
    "RIGHT": "右",
    "LEFT": "左",
    "REAR": "后",
}


@dataclass(frozen=True)
class LogMeta:
    path: Path
    expected_events: int
    expected_location: str | None
    speed_label: str
    no_beacon: bool
    long_log: bool


@dataclass
class Event:
    file_name: str
    event_type: str
    time_s: float
    location: str
    confidence: float
    score: float
    speed_mps: float
    gyro_xy_dps: float
    accel_norm_error_g: float
    tilt_rate_dps: float
    wheel_highpass_count: float


def parse_args() -> argparse.Namespace:
    default_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="离线评估信标检测 CSV 日志")
    parser.add_argument("--root", type=Path, default=default_root, help="Beacon_Detection 目录")
    parser.add_argument("--no-plots", action="store_true", help="不生成 PNG 曲线")
    return parser.parse_args()


def discover_logs(root: Path) -> list[Path]:
    return sorted(root.glob("*.csv"), key=lambda item: item.name)


def meta_from_name(path: Path) -> LogMeta:
    name = path.name
    no_beacon = ("没有" in name) or ("没碰到" in name)
    long_log = "20" in name

    if no_beacon:
        expected_events = 0
    elif long_log:
        expected_events = 20
    else:
        expected_events = 1

    if "正前方" in name:
        expected_location = "FRONT"
    elif "正右方" in name:
        expected_location = "RIGHT"
    elif "正左方" in name:
        expected_location = "LEFT"
    elif "正后方" in name:
        expected_location = "REAR"
    else:
        expected_location = None

    if "快速" in name:
        speed_label = "fast"
    elif "中速" in name:
        speed_label = "medium"
    elif "慢速" in name:
        speed_label = "slow"
    else:
        speed_label = "mixed"

    return LogMeta(path, expected_events, expected_location, speed_label, no_beacon, long_log)


def read_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if len(df.columns) == len(CHANNELS_32):
        df.columns = CHANNELS_32
    elif len(df.columns) == len(CHANNELS_40):
        df.columns = CHANNELS_40
    else:
        raise ValueError(f"{path.name}: columns {len(df.columns)} is not 32 or 40")
    return df.apply(pd.to_numeric, errors="coerce")


def elapsed_ms_from_tick(tick: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    raw_dt = np.diff(tick, prepend=tick[0])
    raw_dt[0] = 1.0
    valid = raw_dt[(raw_dt > 0.0) & (raw_dt < 20.0)]
    median_dt = float(np.median(valid)) if valid.size else 1.0

    dt = raw_dt.copy()
    dt[(dt <= 0.0) | (dt > 50.0)] = median_dt
    elapsed_ms = np.cumsum(dt) - dt[0]
    return elapsed_ms, raw_dt


def ewma_highpass(values: np.ndarray, alpha: float) -> np.ndarray:
    lp = np.empty_like(values, dtype=float)
    hp = np.empty_like(values, dtype=float)
    lp[0] = values[0]
    hp[0] = 0.0
    for i in range(1, len(values)):
        lp[i] = lp[i - 1] + alpha * (values[i] - lp[i - 1])
        hp[i] = values[i] - lp[i]
    return hp


def compute_features(df: pd.DataFrame) -> pd.DataFrame:
    out = pd.DataFrame(index=df.index)
    tick = df["tick_1000us_cnt"].to_numpy(float)
    elapsed_ms, raw_dt = elapsed_ms_from_tick(tick)
    dt_ms = np.diff(elapsed_ms, prepend=elapsed_ms[0])
    dt_ms[0] = 1.0

    ax = df["accel_x_g"].to_numpy(float)
    ay = df["accel_y_g"].to_numpy(float)
    az = df["accel_z_g"].to_numpy(float)
    gx = df["gyro_x_dps"].to_numpy(float)
    gy = df["gyro_y_dps"].to_numpy(float)
    gz = df["gyro_z_dps"].to_numpy(float)
    roll = df["g_euler.roll"].to_numpy(float)
    pitch = df["g_euler.pitch"].to_numpy(float)
    wheels = df[["left_front", "right_front", "left_rear", "right_rear"]].to_numpy(float)

    out["time_s"] = elapsed_ms * 0.001
    out["raw_dt_ms"] = raw_dt
    out["gyro_xy_dps"] = np.hypot(gx, gy)
    out["gyro_z_abs_dps"] = np.abs(gz)
    out["accel_norm_error_g"] = np.abs(np.sqrt(ax * ax + ay * ay + az * az) - 1.0)
    out["tilt_deg"] = np.hypot(roll - roll[0], pitch - pitch[0])
    tilt_rate = np.hypot(np.diff(roll, prepend=roll[0]), np.diff(pitch, prepend=pitch[0]))
    out["tilt_rate_dps"] = np.minimum(tilt_rate / (dt_ms * 0.001), 500.0)

    forward = wheels.sum(axis=1) * (0.25 / 14750.0 / 0.01)
    strafe = (-wheels[:, 0] + wheels[:, 1] + wheels[:, 2] - wheels[:, 3]) * (0.25 / 14000.0 / 0.01)
    out["forward_mps"] = forward
    out["strafe_mps"] = strafe
    out["speed_mps"] = np.hypot(forward, strafe)

    wheel_hpf = np.column_stack([ewma_highpass(wheels[:, i], alpha=0.1111111) for i in range(4)])
    wheel_abs = np.abs(wheel_hpf)
    out["wheel_highpass_count"] = wheel_abs.max(axis=1)
    out["wheel_highpass_sum"] = wheel_abs.sum(axis=1)
    out["wheel_q_count"] = np.abs(wheels[:, 0] + wheels[:, 1] - wheels[:, 2] - wheels[:, 3])
    out["old_bump"] = df["g_beacon_detection.bump_detected"].to_numpy(float)
    out["old_score"] = df["g_beacon_detection.score"].to_numpy(float)
    out["old_location"] = df["g_beacon_detection.location"].to_numpy(float)

    imu_score = np.minimum.reduce(
        [
            out["gyro_xy_dps"].to_numpy() / 45.0,
            out["tilt_rate_dps"].to_numpy() / 45.0,
            out["accel_norm_error_g"].to_numpy() / 0.12,
        ]
    )
    out["imu_score"] = imu_score

    wheel_gate = np.maximum(
        out["speed_mps"].to_numpy() / 0.45,
        out["wheel_highpass_count"].to_numpy() / 65.0,
    )
    out["candidate_a_score"] = np.minimum(imu_score, wheel_gate)

    # 候选 B 更像最终 C 逻辑：IMU 冲击负责上/下信标，速度方向负责位置。
    out["impact_score"] = np.minimum.reduce(
        [
            out["gyro_xy_dps"].to_numpy() / 45.0,
            out["tilt_rate_dps"].to_numpy() / 45.0,
            out["accel_norm_error_g"].to_numpy() / 0.120,
        ]
    )
    out["strong_impact"] = np.minimum.reduce(
        [
            out["gyro_xy_dps"].to_numpy() / 85.0,
            out["tilt_rate_dps"].to_numpy() / 85.0,
            out["accel_norm_error_g"].to_numpy() / 0.22,
        ]
    )
    out["impact_peak_score"] = (
        pd.Series(out["impact_score"].to_numpy())
        .rolling(15, center=True, min_periods=1)
        .max()
        .to_numpy()
    )
    score_series = pd.Series(out["impact_peak_score"].to_numpy())
    # 只用当前点之前的历史做基线，避免离线评估偷看未来数据。
    baseline = score_series.rolling(1500, min_periods=100).median().shift(80).bfill()
    q25 = score_series.rolling(1500, min_periods=100).quantile(0.25).shift(80).bfill()
    q75 = score_series.rolling(1500, min_periods=100).quantile(0.75).shift(80).bfill()
    spread = np.maximum((q75 - q25).to_numpy(), 0.05)
    out["impact_baseline"] = baseline.to_numpy()
    out["impact_robust_z"] = (out["impact_peak_score"].to_numpy() - out["impact_baseline"].to_numpy()) / spread
    return out


def old_algorithm_events(df: pd.DataFrame, meta: LogMeta) -> list[Event]:
    feats = compute_features(df)
    old = feats["old_bump"].to_numpy() > 0.5
    rising = np.flatnonzero(old[1:] & ~old[:-1]) + 1
    events: list[Event] = []
    for idx in rising:
        events.append(make_event(meta.path.name, "old_bump", idx, feats, "UNKNOWN"))
    return events


def direction_from_velocity(forward: float, strafe: float) -> str:
    if abs(forward) < 0.08 and abs(strafe) < 0.08:
        return "UNKNOWN"
    if abs(forward) >= abs(strafe):
        return "FRONT" if forward >= 0.0 else "REAR"
    return "LEFT" if strafe >= 0.0 else "RIGHT"


def make_event(file_name: str, event_type: str, idx: int, feats: pd.DataFrame, location: str) -> Event:
    impact_peak = float(feats.at[idx, "impact_peak_score"]) if "impact_peak_score" in feats.columns else 0.0
    event_score = float(max(impact_peak, feats.at[idx, "impact_score"], feats.at[idx, "candidate_a_score"]))
    return Event(
        file_name=file_name,
        event_type=event_type,
        time_s=float(feats.at[idx, "time_s"]),
        location=location,
        confidence=float(min(1.0, event_score / 1.8)),
        score=event_score,
        speed_mps=float(feats.at[idx, "speed_mps"]),
        gyro_xy_dps=float(feats.at[idx, "gyro_xy_dps"]),
        accel_norm_error_g=float(feats.at[idx, "accel_norm_error_g"]),
        tilt_rate_dps=float(feats.at[idx, "tilt_rate_dps"]),
        wheel_highpass_count=float(feats.at[idx, "wheel_highpass_count"]),
    )


def grouped_peak_events(
    meta: LogMeta,
    feats: pd.DataFrame,
    score_column: str,
    threshold: float,
    min_gap_s: float,
    event_type: str,
) -> list[Event]:
    times = feats["time_s"].to_numpy()
    score = feats[score_column].to_numpy()
    idx = np.flatnonzero(score >= threshold)
    if idx.size == 0:
        return []

    groups: list[int] = []
    start = int(idx[0])
    prev = int(idx[0])
    for item in idx[1:]:
        item = int(item)
        if times[item] - times[prev] > min_gap_s:
            segment = np.arange(start, prev + 1)
            peak = int(segment[np.argmax(score[segment])])
            groups.append(peak)
            start = item
        prev = item
    segment = np.arange(start, prev + 1)
    groups.append(int(segment[np.argmax(score[segment])]))

    events: list[Event] = []
    for peak in groups:
        location = direction_from_velocity(
            float(feats.at[peak, "forward_mps"]),
            float(feats.at[peak, "strafe_mps"]),
        )
        events.append(make_event(meta.path.name, event_type, peak, feats, location))
    return events


def candidate_a_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    return grouped_peak_events(
        meta=meta,
        feats=feats,
        score_column="candidate_a_score",
        threshold=1.10,
        min_gap_s=0.75,
        event_type="impact_peak",
    )


def candidate_b_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    times = feats["time_s"].to_numpy()
    impact = feats["impact_peak_score"].to_numpy()
    z_score = feats["impact_robust_z"].to_numpy()
    gyro_z = feats["gyro_z_abs_dps"].to_numpy()
    speed = feats["speed_mps"].to_numpy()
    pre_speed = np.zeros_like(speed)
    post_speed = np.zeros_like(speed)
    for idx, now_s in enumerate(times):
        pre_idx = np.flatnonzero((times >= now_s - 0.60) & (times <= now_s - 0.12))
        post_idx = np.flatnonzero((times >= now_s + 0.12) & (times <= now_s + 0.60))
        pre_speed[idx] = float(np.median(speed[pre_idx])) if pre_idx.size else 0.0
        post_speed[idx] = float(np.median(speed[post_idx])) if post_idx.size else 0.0

    # 与车端 1s 启动屏蔽保持一致，避免初始姿态/速度滤波未稳定时误计事件。
    basic = (impact >= 0.75) & (times >= 1.0)
    indices = np.flatnonzero(basic)
    if indices.size == 0:
        return []

    grouped: list[int] = []
    start = int(indices[0])
    prev = int(indices[0])
    for item in indices[1:]:
        item = int(item)
        if times[item] - times[prev] > 0.45:
            segment = np.arange(start, prev + 1)
            peak = int(segment[np.argmax(impact[segment] + (0.05 * z_score[segment]))])
            grouped.append(peak)
            start = item
        prev = item
    segment = np.arange(start, prev + 1)
    grouped.append(int(segment[np.argmax(impact[segment] + (0.05 * z_score[segment]))]))

    enter_peaks: list[int] = []
    for idx in grouped:
        adaptive_hit = (impact[idx] >= 1.10) and (pre_speed[idx] <= 1.20) and (z_score[idx] >= 2.0)
        low_speed_exit_like = (impact[idx] >= 1.25) and (post_speed[idx] <= 0.25) and (gyro_z[idx] <= 45.0)
        strong_hit = (impact[idx] >= 1.90) and (gyro_z[idx] <= 25.0)
        hard_stop_noise = (pre_speed[idx] > 1.80) and (post_speed[idx] < 0.20) and (impact[idx] < 3.0)
        spin_noise = (gyro_z[idx] > 40.0) and (impact[idx] < 1.30)

        if (adaptive_hit or low_speed_exit_like or strong_hit) and not (hard_stop_noise or spin_noise):
            enter_peaks.append(idx)

    deduped: list[int] = []
    group: list[int] = []
    for idx in enter_peaks:
        if not group or times[idx] - times[group[-1]] <= 0.70:
            group.append(idx)
        else:
            deduped.append(max(group, key=lambda item: impact[item] + (0.05 * z_score[item])))
            group = [idx]
    if group:
        deduped.append(max(group, key=lambda item: impact[item] + (0.05 * z_score[item])))

    events: list[Event] = []
    for event_index, idx in enumerate(deduped):
        location = direction_from_velocity(
            float(feats.at[idx, "forward_mps"]),
            float(feats.at[idx, "strafe_mps"]),
        )
        events.append(make_event(meta.path.name, "enter", idx, feats, location))

        exit_start_s = times[idx] + 0.28
        exit_end_s = times[idx] + 2.80
        if event_index + 1 < len(deduped):
            exit_end_s = min(exit_end_s, times[deduped[event_index + 1]] - 0.10)
        exit_candidates = [
            item for item in grouped
            if exit_start_s <= times[item] <= exit_end_s and impact[item] >= 0.82
        ]
        if exit_candidates:
            exit_idx = max(exit_candidates, key=lambda item: impact[item])
        else:
            window = np.flatnonzero((times >= exit_start_s) & (times <= exit_end_s))
            exit_idx = int(window[np.argmin(speed[window])]) if window.size else -1

        if exit_idx >= 0:
            exit_location = direction_from_velocity(
                float(feats.at[exit_idx, "forward_mps"]),
                float(feats.at[exit_idx, "strafe_mps"]),
            )
            events.append(make_event(meta.path.name, "exit", exit_idx, feats, exit_location))

    return events


def candidate_c_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    """候选 C：在候选 B 基础上追加低速弱冲击补检，用于离线对比。

    这套规则能补一部分慢速轻压样本，但已知会带来少量短日志多报，
    因此只做离线候选，不直接等同于车端方案。
    """
    times = feats["time_s"].to_numpy()
    impact = feats["impact_peak_score"].to_numpy()
    z_score = feats["impact_robust_z"].to_numpy()
    gyro_z = feats["gyro_z_abs_dps"].to_numpy()
    speed = feats["speed_mps"].to_numpy()
    pre_speed = np.zeros_like(speed)
    post_speed = np.zeros_like(speed)
    post_min_speed = np.zeros_like(speed)
    post_accel_max = np.zeros_like(speed)
    accel_error = feats["accel_norm_error_g"].to_numpy()
    for idx, now_s in enumerate(times):
        pre_idx = np.flatnonzero((times >= now_s - 0.60) & (times <= now_s - 0.12))
        post_idx = np.flatnonzero((times >= now_s + 0.12) & (times <= now_s + 0.60))
        post_accel_idx = np.flatnonzero((times >= now_s + 0.20) & (times <= now_s + 0.90))
        pre_speed[idx] = float(np.median(speed[pre_idx])) if pre_idx.size else 0.0
        post_speed[idx] = float(np.median(speed[post_idx])) if post_idx.size else 0.0
        post_min_speed[idx] = float(np.min(speed[post_idx])) if post_idx.size else 0.0
        post_accel_max[idx] = float(np.max(accel_error[post_accel_idx])) if post_accel_idx.size else 0.0

    indices = np.flatnonzero((impact >= 0.75) & (times >= 1.0))
    if indices.size == 0:
        return []

    grouped: list[int] = []
    start = int(indices[0])
    prev = int(indices[0])
    for item in indices[1:]:
        item = int(item)
        if times[item] - times[prev] > 0.45:
            segment = np.arange(start, prev + 1)
            peak = int(segment[np.argmax(impact[segment] + (0.05 * z_score[segment]))])
            grouped.append(peak)
            start = item
        prev = item
    segment = np.arange(start, prev + 1)
    grouped.append(int(segment[np.argmax(impact[segment] + (0.05 * z_score[segment]))]))

    enter_peaks: list[int] = []
    for idx in grouped:
        adaptive_hit = (impact[idx] >= 1.10) and (pre_speed[idx] <= 1.20) and (z_score[idx] >= 2.0)
        low_speed_exit_like = (impact[idx] >= 1.25) and (post_speed[idx] <= 0.25) and (gyro_z[idx] <= 45.0)
        strong_hit = (impact[idx] >= 1.90) and (gyro_z[idx] <= 25.0)
        weak_low_speed_hit = (
            (impact[idx] >= 0.70)
            and (z_score[idx] >= 2.0)
            and (gyro_z[idx] <= 5.0)
            and (pre_speed[idx] <= 1.00)
            and (post_speed[idx] <= 0.80)
            and (post_min_speed[idx] <= 0.80)
            and (post_accel_max[idx] <= 0.18)
        )
        hard_stop_noise = (pre_speed[idx] > 1.80) and (post_speed[idx] < 0.20) and (impact[idx] < 3.0)
        spin_noise = (gyro_z[idx] > 40.0) and (impact[idx] < 1.30)

        if (adaptive_hit or low_speed_exit_like or strong_hit or weak_low_speed_hit) and not (hard_stop_noise or spin_noise):
            enter_peaks.append(idx)

    weak_indices = np.flatnonzero((impact >= 0.35) & (impact < 0.75) & (times >= 1.0))
    weak_grouped: list[int] = []
    if weak_indices.size:
        weak_start = int(weak_indices[0])
        weak_prev = int(weak_indices[0])
        for item in weak_indices[1:]:
            item = int(item)
            if times[item] - times[weak_prev] > 0.45:
                segment = np.arange(weak_start, weak_prev + 1)
                weak_grouped.append(int(segment[np.argmax(impact[segment] + (0.05 * z_score[segment]))]))
                weak_start = item
            weak_prev = item
        segment = np.arange(weak_start, weak_prev + 1)
        weak_grouped.append(int(segment[np.argmax(impact[segment] + (0.05 * z_score[segment]))]))

    for idx in weak_grouped:
        weak_low_speed_hit = (
            (impact[idx] >= 0.70)
            and (z_score[idx] >= 2.0)
            and (gyro_z[idx] <= 5.0)
            and (pre_speed[idx] <= 1.00)
            and (post_speed[idx] <= 0.80)
            and (post_min_speed[idx] <= 0.80)
        )
        if weak_low_speed_hit:
            enter_peaks.append(idx)

    deduped: list[int] = []
    group: list[int] = []
    for idx in enter_peaks:
        if not group or times[idx] - times[group[-1]] <= 0.70:
            group.append(idx)
        else:
            deduped.append(max(group, key=lambda item: impact[item] + (0.05 * z_score[item])))
            group = [idx]
    if group:
        deduped.append(max(group, key=lambda item: impact[item] + (0.05 * z_score[item])))

    events: list[Event] = []
    for event_index, idx in enumerate(deduped):
        location = direction_from_velocity(
            float(feats.at[idx, "forward_mps"]),
            float(feats.at[idx, "strafe_mps"]),
        )
        events.append(make_event(meta.path.name, "enter", idx, feats, location))

        exit_start_s = times[idx] + 0.28
        exit_end_s = times[idx] + 2.80
        if event_index + 1 < len(deduped):
            exit_end_s = min(exit_end_s, times[deduped[event_index + 1]] - 0.10)
        exit_candidates = [
            item for item in grouped
            if exit_start_s <= times[item] <= exit_end_s and impact[item] >= 0.82
        ]
        if exit_candidates:
            exit_idx = max(exit_candidates, key=lambda item: impact[item])
        else:
            window = np.flatnonzero((times >= exit_start_s) & (times <= exit_end_s))
            exit_idx = int(window[np.argmin(speed[window])]) if window.size else -1

        if exit_idx >= 0:
            exit_location = direction_from_velocity(
                float(feats.at[exit_idx, "forward_mps"]),
                float(feats.at[exit_idx, "strafe_mps"]),
            )
            events.append(make_event(meta.path.name, "exit", exit_idx, feats, exit_location))

    return events


def window_indices(times: np.ndarray, start_s: float, end_s: float) -> np.ndarray:
    start_idx = int(np.searchsorted(times, start_s, side="left"))
    end_idx = int(np.searchsorted(times, end_s, side="right"))
    return np.arange(start_idx, end_idx)


def median_window(values: np.ndarray, times: np.ndarray, start_s: float, end_s: float) -> float:
    idx = window_indices(times, start_s, end_s)
    return float(np.median(values[idx])) if idx.size else 0.0


def min_window(values: np.ndarray, times: np.ndarray, start_s: float, end_s: float) -> float:
    idx = window_indices(times, start_s, end_s)
    return float(np.min(values[idx])) if idx.size else 0.0


def max_window(values: np.ndarray, times: np.ndarray, start_s: float, end_s: float) -> float:
    idx = window_indices(times, start_s, end_s)
    return float(np.max(values[idx])) if idx.size else 0.0


def grouped_mask_peaks(times: np.ndarray, mask: np.ndarray, key: np.ndarray, min_gap_s: float) -> list[int]:
    indices = np.flatnonzero(mask)
    if indices.size == 0:
        return []

    peaks: list[int] = []
    members: list[int] = [int(indices[0])]
    prev = int(indices[0])
    for item in indices[1:]:
        item = int(item)
        if times[item] - times[prev] > min_gap_s:
            member_array = np.asarray(members, dtype=int)
            peaks.append(int(member_array[np.argmax(key[member_array])]))
            members = [item]
        else:
            members.append(item)
        prev = item

    member_array = np.asarray(members, dtype=int)
    peaks.append(int(member_array[np.argmax(key[member_array])]))
    return peaks


def event_motion_context(feats: pd.DataFrame, idx: int) -> dict[str, object]:
    times = feats["time_s"].to_numpy()
    now_s = float(times[idx])
    speed = feats["speed_mps"].to_numpy()
    forward = feats["forward_mps"].to_numpy()
    strafe = feats["strafe_mps"].to_numpy()
    accel_error = feats["accel_norm_error_g"].to_numpy()

    pre_speed = median_window(speed, times, now_s - 0.60, now_s - 0.12)
    post_speed = median_window(speed, times, now_s + 0.12, now_s + 0.60)
    post_min_speed = min_window(speed, times, now_s + 0.12, now_s + 0.60)
    post_accel_max = max_window(accel_error, times, now_s + 0.20, now_s + 0.90)
    pre_forward = median_window(forward, times, now_s - 0.60, now_s - 0.12)
    pre_strafe = median_window(strafe, times, now_s - 0.60, now_s - 0.12)
    post_forward = median_window(forward, times, now_s + 0.12, now_s + 0.60)
    post_strafe = median_window(strafe, times, now_s + 0.12, now_s + 0.60)

    current_dir = direction_from_velocity(float(forward[idx]), float(strafe[idx]))
    pre_dir = direction_from_velocity(pre_forward, pre_strafe)
    post_dir = direction_from_velocity(post_forward, post_strafe)
    direction_consistency = int(current_dir == pre_dir) + int(current_dir == post_dir) + int(pre_dir == post_dir)

    return {
        "pre_speed": pre_speed,
        "post_speed": post_speed,
        "post_min_speed": post_min_speed,
        "post_accel_max": post_accel_max,
        "current_dir": current_dir,
        "pre_dir": pre_dir,
        "post_dir": post_dir,
        "direction_consistency": direction_consistency,
    }


def candidate_d_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    """Final offline candidate.

    It keeps candidate C as the main path, rejects one unstable duplicate-peak
    shape, and adds two narrow bypasses seen in the logs: fast front impact and
    all-wheel stop impact. Both bypasses use only sensor features, not filenames.
    """
    times = feats["time_s"].to_numpy()
    impact = feats["impact_peak_score"].to_numpy()
    raw_impact = feats["impact_score"].to_numpy()
    z_score = feats["impact_robust_z"].to_numpy()
    gyro_z = feats["gyro_z_abs_dps"].to_numpy()
    accel_error = feats["accel_norm_error_g"].to_numpy()
    wheel_sum = feats["wheel_highpass_sum"].to_numpy()
    speed = feats["speed_mps"].to_numpy()

    candidate_items: list[dict[str, object]] = []
    for event in candidate_c_events(meta, feats):
        if event.event_type != "enter":
            continue
        idx = int(np.argmin(np.abs(times - event.time_s)))
        context = event_motion_context(feats, idx)
        unstable_duplicate = (
            (impact[idx] < 1.30)
            and (float(context["post_accel_max"]) >= 0.24)
            and (float(context["post_min_speed"]) >= 0.60)
            and (int(context["direction_consistency"]) == 0)
        )
        if not unstable_duplicate:
            candidate_items.append({"idx": idx, "reason": "candidate_c", "location": event.location})

    all_stop_mask = (
        (times >= 1.0)
        & (impact >= 1.20)
        & (z_score >= 4.0)
        & (gyro_z <= 8.0)
        & (wheel_sum >= 600.0)
    )
    all_stop_key = impact + (0.05 * z_score) + (0.001 * np.minimum(wheel_sum, 1500.0))
    for idx in grouped_mask_peaks(times, all_stop_mask, all_stop_key, 0.45):
        context = event_motion_context(feats, idx)
        is_all_stop = (
            (float(context["pre_speed"]) >= 1.40)
            and (float(context["post_speed"]) <= 0.10)
            and (float(context["post_min_speed"]) <= 0.05)
            and (float(context["post_accel_max"]) <= 0.08)
        )
        if is_all_stop:
            location = str(context["pre_dir"])
            if location == "UNKNOWN":
                location = str(context["current_dir"])
            candidate_items.append({"idx": idx, "reason": "all_stop", "location": location})

    front_fast_mask = (
        (times >= 1.0)
        & (impact >= 1.45)
        & (raw_impact >= 1.20)
        & (z_score >= 4.0)
        & (gyro_z <= 45.0)
        & (accel_error <= 0.20)
        & (wheel_sum < 180.0)
    )
    front_fast_key = raw_impact + (0.02 * z_score)
    for idx in grouped_mask_peaks(times, front_fast_mask, front_fast_key, 0.45):
        context = event_motion_context(feats, idx)
        is_front_fast = (
            (context["current_dir"] == "FRONT")
            and (context["pre_dir"] == "FRONT")
            and (float(context["pre_speed"]) >= 1.20)
            and (float(context["post_speed"]) <= 0.90)
            and (float(context["post_min_speed"]) <= 0.35)
            and (float(context["post_accel_max"]) <= 0.25)
        )
        if is_front_fast:
            candidate_items.append({"idx": idx, "reason": "front_fast", "location": "FRONT"})

    candidate_items.sort(key=lambda item: times[int(item["idx"])])
    keep = [True] * len(candidate_items)
    for all_stop_pos, all_stop_item in enumerate(candidate_items):
        if all_stop_item["reason"] != "all_stop":
            continue
        all_stop_idx = int(all_stop_item["idx"])
        for pos in range(all_stop_pos):
            item = candidate_items[pos]
            if item["reason"] != "candidate_c":
                continue
            idx = int(item["idx"])
            delta_s = times[all_stop_idx] - times[idx]
            context = event_motion_context(feats, idx)
            if (0.50 <= delta_s <= 4.50) and (
                (impact[idx] < 1.0) or (float(context["post_min_speed"]) < 0.05)
            ):
                keep[pos] = False
    candidate_items = [item for item, item_keep in zip(candidate_items, keep) if item_keep]

    deduped: list[dict[str, object]] = []
    group: list[dict[str, object]] = []
    for item in candidate_items:
        idx = int(item["idx"])
        if not group or times[idx] - times[int(group[-1]["idx"])] <= 0.70:
            group.append(item)
        else:
            deduped.append(
                max(
                    group,
                    key=lambda entry: impact[int(entry["idx"])]
                    + (0.05 * z_score[int(entry["idx"])])
                    + (0.50 if entry["reason"] == "all_stop" else 0.0)
                    + (0.25 if entry["reason"] == "front_fast" else 0.0),
                )
            )
            group = [item]
    if group:
        deduped.append(
            max(
                group,
                key=lambda entry: impact[int(entry["idx"])]
                + (0.05 * z_score[int(entry["idx"])])
                + (0.50 if entry["reason"] == "all_stop" else 0.0)
                + (0.25 if entry["reason"] == "front_fast" else 0.0),
            )
        )

    enter_indices: list[tuple[int, str]] = []
    for item in deduped:
        idx = int(item["idx"])
        context = event_motion_context(feats, idx)
        location = str(item["location"])
        if ((item["reason"] == "all_stop") or (location == "UNKNOWN")) and context["pre_dir"] != "UNKNOWN":
            location = str(context["pre_dir"])
        enter_indices.append((idx, location))

    impact_grouped = grouped_mask_peaks(
        times,
        (impact >= 0.75) & (times >= 1.0),
        impact + (0.05 * z_score),
        0.45,
    )

    events: list[Event] = []
    for event_index, (idx, location) in enumerate(enter_indices):
        events.append(make_event(meta.path.name, "enter", idx, feats, location))

        exit_start_s = times[idx] + 0.28
        exit_end_s = times[idx] + 2.80
        if event_index + 1 < len(enter_indices):
            exit_end_s = min(exit_end_s, times[enter_indices[event_index + 1][0]] - 0.10)
        exit_candidates = [
            item for item in impact_grouped
            if exit_start_s <= times[item] <= exit_end_s and impact[item] >= 0.82
        ]
        if exit_candidates:
            exit_idx = max(exit_candidates, key=lambda item: impact[item])
        else:
            window = np.flatnonzero((times >= exit_start_s) & (times <= exit_end_s))
            exit_idx = int(window[np.argmin(speed[window])]) if window.size else -1

        if exit_idx >= 0:
            exit_location = direction_from_velocity(
                float(feats.at[exit_idx, "forward_mps"]),
                float(feats.at[exit_idx, "strafe_mps"]),
            )
            events.append(make_event(meta.path.name, "exit", exit_idx, feats, exit_location))

    return events


def rt_event_motion_context(feats: pd.DataFrame, idx: int, latest_s: float) -> dict[str, object]:
    """Bounded-latency motion context used by the realtime candidate.

    The post windows only look as far as latest_s. candidate_rt_events confirms
    a candidate after the needed 0.9s tail has arrived, so this is implementable
    on the car with a small pending state instead of full-log lookahead.
    """
    times = feats["time_s"].to_numpy()
    now_s = float(times[idx])
    speed = feats["speed_mps"].to_numpy()
    forward = feats["forward_mps"].to_numpy()
    strafe = feats["strafe_mps"].to_numpy()
    accel_error = feats["accel_norm_error_g"].to_numpy()

    post_end_s = min(latest_s, now_s + 0.60)
    tail_end_s = min(latest_s, now_s + 0.90)
    pre_speed = median_window(speed, times, now_s - 0.60, now_s - 0.12)
    post_speed = median_window(speed, times, now_s + 0.12, post_end_s)
    post_min_speed = min_window(speed, times, now_s + 0.12, post_end_s)
    post_accel_max = max_window(accel_error, times, now_s + 0.20, tail_end_s)
    pre_forward = median_window(forward, times, now_s - 0.60, now_s - 0.12)
    pre_strafe = median_window(strafe, times, now_s - 0.60, now_s - 0.12)
    post_forward = median_window(forward, times, now_s + 0.12, post_end_s)
    post_strafe = median_window(strafe, times, now_s + 0.12, post_end_s)

    current_dir = direction_from_velocity(float(forward[idx]), float(strafe[idx]))
    pre_dir = direction_from_velocity(pre_forward, pre_strafe)
    post_dir = direction_from_velocity(post_forward, post_strafe)
    direction_consistency = int(current_dir == pre_dir) + int(current_dir == post_dir) + int(pre_dir == post_dir)

    return {
        "pre_speed": pre_speed,
        "post_speed": post_speed,
        "post_min_speed": post_min_speed,
        "post_accel_max": post_accel_max,
        "current_dir": current_dir,
        "pre_dir": pre_dir,
        "post_dir": post_dir,
        "direction_consistency": direction_consistency,
    }


def candidate_rt_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    """Realtime-feasible candidate.

    This simulates the car-side policy with bounded delay:
    - an enter candidate is a local sensor peak, confirmed after 1.1s;
    - enter detection keeps running while exit search is open;
    - weak peaks yield to a later strong peak inside 1.1s;
    - exit only closes the current on-beacon interval and never blocks enter.
    """
    times = feats["time_s"].to_numpy()
    impact = feats["impact_peak_score"].to_numpy()
    raw_impact = feats["impact_score"].to_numpy()
    z_score = feats["impact_robust_z"].to_numpy()
    gyro_z = feats["gyro_z_abs_dps"].to_numpy()
    accel_error = feats["accel_norm_error_g"].to_numpy()
    wheel_sum = feats["wheel_highpass_sum"].to_numpy()
    speed = feats["speed_mps"].to_numpy()

    key = impact + (0.05 * z_score)
    raw_items: list[dict[str, object]] = []

    def maybe_add_candidate(idx: int) -> None:
        latest_s = float(times[idx] + 1.10)
        context = rt_event_motion_context(feats, idx, latest_s)
        adaptive_hit = (
            (impact[idx] >= 1.10)
            and (float(context["pre_speed"]) <= 1.20)
            and (z_score[idx] >= 2.0)
        )
        low_speed_hit = (
            (impact[idx] >= 1.25)
            and (float(context["post_speed"]) <= 0.25)
            and (gyro_z[idx] <= 45.0)
        )
        strong_hit = (impact[idx] >= 1.90) and (gyro_z[idx] <= 25.0)
        weak_low_speed_hit = (
            (impact[idx] >= 0.70)
            and (z_score[idx] >= 2.0)
            and (gyro_z[idx] <= 5.0)
            and (float(context["pre_speed"]) <= 1.00)
            and (float(context["post_speed"]) <= 0.80)
            and (float(context["post_min_speed"]) >= 0.05)
            and (float(context["post_min_speed"]) <= 0.80)
            and (float(context["post_accel_max"]) <= 0.18)
        )
        front_fast_hit = (
            (impact[idx] >= 1.45)
            and (raw_impact[idx] >= 1.20)
            and (z_score[idx] >= 4.0)
            and (gyro_z[idx] <= 45.0)
            and (accel_error[idx] <= 0.20)
            and (wheel_sum[idx] < 180.0)
            and (context["current_dir"] == "FRONT")
            and (context["pre_dir"] == "FRONT")
            and (float(context["pre_speed"]) >= 1.20)
            and (float(context["post_speed"]) <= 0.90)
            and (float(context["post_min_speed"]) <= 0.35)
            and (float(context["post_accel_max"]) <= 0.25)
        )
        all_stop_hit = (
            (impact[idx] >= 1.20)
            and (z_score[idx] >= 4.0)
            and (gyro_z[idx] <= 8.0)
            and (wheel_sum[idx] >= 600.0)
            and (float(context["pre_speed"]) >= 1.40)
            and (float(context["post_speed"]) <= 0.10)
            and (float(context["post_min_speed"]) <= 0.05)
            and (float(context["post_accel_max"]) <= 0.08)
        )
        unstable_duplicate = (
            (impact[idx] < 1.30)
            and (float(context["post_accel_max"]) >= 0.24)
            and (float(context["post_min_speed"]) >= 0.60)
            and (int(context["direction_consistency"]) == 0)
        )
        hard_stop_noise = (
            (float(context["pre_speed"]) > 1.80)
            and (float(context["post_speed"]) < 0.20)
            and (impact[idx] < 3.0)
        )
        spin_noise = (gyro_z[idx] > 40.0) and (impact[idx] < 1.30)

        valid = (
            adaptive_hit
            or low_speed_hit
            or strong_hit
            or weak_low_speed_hit
            or front_fast_hit
            or all_stop_hit
        )
        if (not valid) or ((hard_stop_noise and not all_stop_hit) or unstable_duplicate or spin_noise):
            return

        if all_stop_hit and context["pre_dir"] != "UNKNOWN":
            location = str(context["pre_dir"])
        else:
            location = str(context["current_dir"])
            if location == "UNKNOWN" and context["pre_dir"] != "UNKNOWN":
                location = str(context["pre_dir"])

        reason = "rt_main"
        if weak_low_speed_hit and not (adaptive_hit or low_speed_hit or strong_hit):
            reason = "rt_weak"
        if front_fast_hit:
            reason = "rt_front_fast"
        if all_stop_hit:
            reason = "rt_all_stop"

        raw_items.append({"idx": idx, "reason": reason, "location": location})

    main_peaks = grouped_mask_peaks(
        times,
        (times >= 1.0) & (impact >= 0.75),
        key,
        0.45,
    )
    for peak_idx in main_peaks:
        maybe_add_candidate(peak_idx)

    weak_peaks = grouped_mask_peaks(
        times,
        (times >= 1.0) & (impact >= 0.35) & (impact < 0.75),
        key,
        0.45,
    )
    for peak_idx in weak_peaks:
        maybe_add_candidate(peak_idx)

    front_fast_peaks = grouped_mask_peaks(
        times,
        (times >= 1.0)
        & (impact >= 1.45)
        & (raw_impact >= 1.20)
        & (z_score >= 4.0)
        & (gyro_z <= 45.0)
        & (accel_error <= 0.20)
        & (wheel_sum < 180.0),
        raw_impact + (0.02 * z_score),
        0.45,
    )
    for peak_idx in front_fast_peaks:
        maybe_add_candidate(peak_idx)

    all_stop_peaks = grouped_mask_peaks(
        times,
        (times >= 1.0)
        & (impact >= 1.20)
        & (z_score >= 4.0)
        & (gyro_z <= 8.0)
        & (wheel_sum >= 600.0),
        impact + (0.05 * z_score) + (0.001 * np.minimum(wheel_sum, 1500.0)),
        0.45,
    )
    for peak_idx in all_stop_peaks:
        maybe_add_candidate(peak_idx)

    raw_items.sort(key=lambda item: times[int(item["idx"])])
    keep = [True] * len(raw_items)
    for pos, item in enumerate(raw_items):
        idx = int(item["idx"])
        context = rt_event_motion_context(feats, idx, float(times[idx] + 1.10))
        if (
            item["reason"] != "rt_all_stop"
            and (impact[idx] < 1.0)
            and (float(context["post_min_speed"]) < 0.05)
        ):
            keep[pos] = False
            continue

        if item["reason"] != "rt_weak":
            continue
        for next_pos in range(pos + 1, len(raw_items)):
            next_idx = int(raw_items[next_pos]["idx"])
            delta_s = times[next_idx] - times[idx]
            if delta_s > 1.10:
                break
            if (impact[next_idx] >= 1.10) and (raw_items[next_pos]["reason"] != "rt_weak"):
                keep[pos] = False
                break

    raw_items = [item for item, item_keep in zip(raw_items, keep) if item_keep]

    def item_quality(item: dict[str, object]) -> float:
        idx = int(item["idx"])
        bonus = 0.0
        if item["reason"] == "rt_all_stop":
            bonus += 0.50
        if item["reason"] == "rt_front_fast":
            bonus += 0.25
        return float(impact[idx] + (0.05 * z_score[idx]) + bonus)

    deduped: list[dict[str, object]] = []
    group: list[dict[str, object]] = []
    for item in raw_items:
        idx = int(item["idx"])
        if not group or times[idx] - times[int(group[-1]["idx"])] <= 0.70:
            group.append(item)
        else:
            deduped.append(max(group, key=item_quality))
            group = [item]
    if group:
        deduped.append(max(group, key=item_quality))

    enter_indices = [(int(item["idx"]), str(item["location"])) for item in deduped]
    exit_peak_indices = grouped_mask_peaks(
        times,
        (impact >= 0.75) & (times >= 1.0),
        impact + (0.05 * z_score),
        0.45,
    )

    events: list[Event] = []
    for event_index, (idx, location) in enumerate(enter_indices):
        events.append(make_event(meta.path.name, "enter", idx, feats, location))

        exit_start_s = times[idx] + 0.28
        exit_end_s = times[idx] + 2.80
        if event_index + 1 < len(enter_indices):
            exit_end_s = min(exit_end_s, times[enter_indices[event_index + 1][0]] - 0.10)

        exit_candidates = [
            item for item in exit_peak_indices
            if exit_start_s <= times[item] <= exit_end_s and impact[item] >= 0.82
        ]
        if exit_candidates:
            exit_idx = max(exit_candidates, key=lambda item: impact[item])
        else:
            window = np.flatnonzero((times >= exit_start_s) & (times <= exit_end_s))
            exit_idx = int(window[np.argmin(speed[window])]) if window.size else -1

        if exit_idx >= 0:
            exit_location = direction_from_velocity(
                float(feats.at[exit_idx, "forward_mps"]),
                float(feats.at[exit_idx, "strafe_mps"]),
            )
            events.append(make_event(meta.path.name, "exit", exit_idx, feats, exit_location))

    return events


def event_count(events: list[Event], event_type: str) -> int:
    return sum(1 for event in events if event.event_type == event_type)


def summarize_algorithm(meta: LogMeta, events: list[Event], prefix: str) -> dict[str, object]:
    if prefix == "old":
        detected_enter = len(events)
        detected_exit = 0
    else:
        detected_enter = event_count(events, "enter")
        detected_exit = event_count(events, "exit")
        if detected_enter == 0 and detected_exit == 0:
            detected_enter = len(events)

    false_triggers = len(events) if meta.no_beacon else 0
    direction_items = [event.location for event in events if event.event_type != "exit"]
    if direction_items:
        direction_summary = ",".join(direction_items[:8])
        if len(direction_items) > 8:
            direction_summary += f"...(+{len(direction_items) - 8})"
    else:
        direction_summary = ""

    expected = meta.expected_location
    direction_ok = ""
    if expected is not None and direction_items:
        direction_ok = str(sum(1 for item in direction_items if item == expected)) + "/" + str(len(direction_items))

    note_parts: list[str] = []
    if meta.no_beacon and false_triggers == 0:
        note_parts.append("无信标0误触发")
    elif meta.no_beacon:
        note_parts.append("无信标误触发")
    elif meta.long_log:
        note_parts.append(f"长日志有效enter={detected_enter}")
    elif detected_enter < 1:
        note_parts.append("短日志漏检")
    if expected is not None and direction_ok and not direction_ok.startswith(str(len(direction_items)) + "/"):
        note_parts.append("方向需复核")

    return {
        f"{prefix}_enter_count": detected_enter,
        f"{prefix}_exit_count": detected_exit,
        f"{prefix}_false_trigger_count": false_triggers,
        f"{prefix}_directions": direction_summary,
        f"{prefix}_direction_ok": direction_ok,
        f"{prefix}_notes": ";".join(note_parts),
    }


def scan_log(path: Path, df: pd.DataFrame) -> dict[str, object]:
    tick = df["tick_1000us_cnt"].to_numpy(float)
    _, raw_dt = elapsed_ms_from_tick(tick)
    bad_tick = int(np.sum((raw_dt[1:] != 1.0)))
    old_events = len(old_algorithm_events(df, meta_from_name(path)))
    numeric = df.to_numpy(float)
    return {
        "file_name": path.name,
        "rows": len(df),
        "columns": len(df.columns),
        "channel_format": "debug_40" if len(df.columns) == 40 else "legacy_32",
        "parse_32_channels": len(df.columns) == 32,
        "parse_40_channels": len(df.columns) == 40,
        "bad_tick_delta_count": bad_tick,
        "non_monotonic_tick_count": int(np.sum(raw_dt[1:] <= 0.0)),
        "max_tick_gap_ms": float(np.max(raw_dt[1:])) if len(raw_dt) > 1 else 0.0,
        "nan_count": int(df.isna().sum().sum()),
        "inf_count": int(np.isinf(numeric).sum()),
        "old_rising_events": old_events,
    }


def event_rows(events: list[Event]) -> list[dict[str, object]]:
    return [
        {
            "file_name": event.file_name,
            "event_type": event.event_type,
            "time_s": event.time_s,
            "location": event.location,
            "location_cn": LOCATION_CN.get(event.location, event.location),
            "confidence": event.confidence,
            "score": event.score,
            "speed_mps": event.speed_mps,
            "gyro_xy_dps": event.gyro_xy_dps,
            "accel_norm_error_g": event.accel_norm_error_g,
            "tilt_rate_dps": event.tilt_rate_dps,
            "wheel_highpass_count": event.wheel_highpass_count,
        }
        for event in events
    ]


def plot_log(meta: LogMeta, feats: pd.DataFrame, events: list[Event], output_dir: Path) -> None:
    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)

    t = feats["time_s"]
    fig, axes = plt.subplots(5, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(t, feats["gyro_xy_dps"], label="gyro_xy")
    axes[0].plot(t, feats["gyro_z_abs_dps"], label="abs(gyro_z)", alpha=0.65)
    axes[0].set_ylabel("dps")
    axes[0].legend(loc="upper right")

    axes[1].plot(t, feats["accel_norm_error_g"], label="accel_norm_error")
    axes[1].set_ylabel("g")
    axes[1].legend(loc="upper right")

    axes[2].plot(t, feats["tilt_deg"], label="tilt_deg")
    axes[2].plot(t, feats["tilt_rate_dps"] / 50.0, label="tilt_rate/50", alpha=0.7)
    axes[2].set_ylabel("tilt")
    axes[2].legend(loc="upper right")

    axes[3].plot(t, feats["speed_mps"], label="speed")
    axes[3].plot(t, feats["forward_mps"], label="forward", alpha=0.8)
    axes[3].plot(t, feats["strafe_mps"], label="strafe", alpha=0.8)
    axes[3].set_ylabel("m/s")
    axes[3].legend(loc="upper right")

    axes[4].plot(t, feats["impact_score"], label="impact_score")
    axes[4].plot(t, feats["candidate_a_score"], label="candidate_a_score", alpha=0.8)
    axes[4].plot(t, feats["old_bump"], label="old_bump", alpha=0.6)
    axes[4].set_ylabel("score")
    axes[4].set_xlabel("time (s)")
    axes[4].legend(loc="upper right")

    for event in events:
        color = "tab:red" if event.event_type == "enter" else "tab:purple"
        for ax in axes:
            ax.axvline(event.time_s, color=color, alpha=0.35, linewidth=1.0)

    fig.suptitle(meta.path.name)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    safe_name = re.sub(r"[\\/:*?\"<>|,， ]+", "_", meta.path.stem)
    fig.savefig(figure_dir / f"{safe_name}.png", dpi=150)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    root = args.root.resolve()
    output_dir = root / "analysis" / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    logs = discover_logs(root)
    scan_rows: list[dict[str, object]] = []
    evaluation_rows: list[dict[str, object]] = []
    old_events_all: list[Event] = []
    candidate_a_all: list[Event] = []
    candidate_b_all: list[Event] = []
    candidate_c_all: list[Event] = []
    candidate_d_all: list[Event] = []
    candidate_rt_all: list[Event] = []

    plot_names = {
        "经过20次信标灯.csv",
        "快速跑没有经过信标.csv",
        "实际没有碰到信标灯.csv",
        "信标在正前方,直接经过,快速.csv",
        "信标在正右方,直接经过,中速.csv",
        "信标在正左方,直接经过,快速2.csv",
        "信标在正后方,直接经过,快速.csv",
    }

    for path in logs:
        meta = meta_from_name(path)
        df = read_log(path)
        feats = compute_features(df)
        scan_rows.append(scan_log(path, df))

        old_events = old_algorithm_events(df, meta)
        candidate_a = candidate_a_events(meta, feats)
        candidate_b = candidate_b_events(meta, feats)
        candidate_c = candidate_c_events(meta, feats)
        candidate_d = candidate_d_events(meta, feats)
        candidate_rt = candidate_rt_events(meta, feats)

        old_events_all.extend(old_events)
        candidate_a_all.extend(candidate_a)
        candidate_b_all.extend(candidate_b)
        candidate_c_all.extend(candidate_c)
        candidate_d_all.extend(candidate_d)
        candidate_rt_all.extend(candidate_rt)

        row: dict[str, object] = {
            "file_name": path.name,
            "expected_events": meta.expected_events,
            "expected_location": meta.expected_location or "",
            "speed_label": meta.speed_label,
            "duration_s": float(feats["time_s"].iloc[-1]),
        }
        row.update(summarize_algorithm(meta, old_events, "old"))
        row.update(summarize_algorithm(meta, candidate_a, "candidate_a"))
        row.update(summarize_algorithm(meta, candidate_b, "candidate_b"))
        row.update(summarize_algorithm(meta, candidate_c, "candidate_c"))
        row.update(summarize_algorithm(meta, candidate_d, "candidate_d"))
        row.update(summarize_algorithm(meta, candidate_rt, "candidate_rt"))
        evaluation_rows.append(row)

        if not args.no_plots and path.name in plot_names:
            plot_log(meta, feats, candidate_rt, output_dir)

    scan_df = pd.DataFrame(scan_rows)
    eval_df = pd.DataFrame(evaluation_rows)
    old_df = pd.DataFrame(event_rows(old_events_all))
    candidate_a_df = pd.DataFrame(event_rows(candidate_a_all))
    candidate_b_df = pd.DataFrame(event_rows(candidate_b_all))
    candidate_c_df = pd.DataFrame(event_rows(candidate_c_all))
    candidate_d_df = pd.DataFrame(event_rows(candidate_d_all))
    candidate_rt_df = pd.DataFrame(event_rows(candidate_rt_all))

    scan_df.to_csv(output_dir / "scan_summary.csv", index=False, encoding="utf-8-sig")
    eval_df.to_csv(output_dir / "evaluation_summary.csv", index=False, encoding="utf-8-sig")
    old_df.to_csv(output_dir / "events_old.csv", index=False, encoding="utf-8-sig")
    candidate_a_df.to_csv(output_dir / "events_candidate_a.csv", index=False, encoding="utf-8-sig")
    candidate_b_df.to_csv(output_dir / "events_candidate_b.csv", index=False, encoding="utf-8-sig")
    candidate_c_df.to_csv(output_dir / "events_candidate_c.csv", index=False, encoding="utf-8-sig")
    candidate_d_df.to_csv(output_dir / "events_candidate_d.csv", index=False, encoding="utf-8-sig")
    candidate_rt_df.to_csv(output_dir / "events_candidate_rt.csv", index=False, encoding="utf-8-sig")

    print(f"扫描 CSV: {len(logs)} 份")
    print(f"扫描表: {output_dir / 'scan_summary.csv'}")
    print(f"评估表: {output_dir / 'evaluation_summary.csv'}")
    print(f"候选B事件: {output_dir / 'events_candidate_b.csv'}")
    print(f"候选C事件: {output_dir / 'events_candidate_c.csv'}")

    print(f"candidate_d events: {output_dir / 'events_candidate_d.csv'}")
    print(f"candidate_rt events: {output_dir / 'events_candidate_rt.csv'}")

    no_beacon = eval_df[eval_df["expected_events"] == 0]
    long_rows = eval_df[eval_df["file_name"].str.contains("20", regex=False)]
    if not no_beacon.empty:
        print(
            "候选B无信标误触发:",
            int(no_beacon["candidate_b_false_trigger_count"].sum()),
        )
        print("candidate_d no-beacon false triggers:", int(no_beacon["candidate_d_false_trigger_count"].sum()))
        print("candidate_rt no-beacon false triggers:", int(no_beacon["candidate_rt_false_trigger_count"].sum()))
    if not long_rows.empty:
        print(
            "候选B长日志 enter:",
            int(long_rows["candidate_b_enter_count"].iloc[0]),
            "exit:",
            int(long_rows["candidate_b_exit_count"].iloc[0]),
        )
        print(
            "候选C长日志 enter:",
            int(long_rows["candidate_c_enter_count"].iloc[0]),
            "exit:",
            int(long_rows["candidate_c_exit_count"].iloc[0]),
        )
        print(
            "candidate_d long log enter:",
            int(long_rows["candidate_d_enter_count"].iloc[0]),
            "exit:",
            int(long_rows["candidate_d_exit_count"].iloc[0]),
        )
        print(
            "candidate_rt long log enter:",
            int(long_rows["candidate_rt_enter_count"].iloc[0]),
            "exit:",
            int(long_rows["candidate_rt_exit_count"].iloc[0]),
        )


if __name__ == "__main__":
    main()
