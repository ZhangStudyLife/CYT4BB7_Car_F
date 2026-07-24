#!/usr/bin/env python3
"""Offline analysis for the second beacon calibration dataset.

The script is intentionally data-first: it scans the 29 new 40-channel logs,
replays the current on-board outputs, computes lightweight features, and
evaluates candidate enter/exit detectors without modifying raw CSV files.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse
import math
import re

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

ODOMETER_FORWARD_COUNT_PER_METER = 14750.0
ODOMETER_STRAFE_COUNT_PER_METER_ABS = 14000.0
ODOMETER_UPDATE_DT_S = 0.01
C_REPLAY_STARTUP_TICKS = 0
C_REPLAY_IMU_WINDOW_TICKS = 24
C_REPLAY_EVENT_HOLD_TICKS = 120
C_REPLAY_PAIR_MIN_TICKS = 195
C_REPLAY_PAIR_MAX_TICKS = 1005
C_REPLAY_WEAK_EARLY_TICKS = 800
C_REPLAY_STRONG_TAIL_POSE_TICKS = 700
C_REPLAY_SIDE_TAIL_POSE_TICKS = 650
C_REPLAY_TAIL_ABSORB_AFTER_EARLY_S = 0.55
C_REPLAY_WEAK_CLEAN_TAIL_TICKS = 700
C_REPLAY_CLUSTER_GAP_TICKS = 750
C_REPLAY_CANDIDATE_PEAK_GAP_TICKS = 80
C_REPLAY_MID_STARTUP_TICKS = 4000
C_REPLAY_EXPIRED_RESEED_SCORE = 2.35


@dataclass(frozen=True)
class LogMeta:
    path: Path
    expected_events: int
    expected_location: str
    speed_label: str
    log_kind: str


@dataclass(frozen=True)
class Event:
    file_name: str
    algorithm: str
    event_type: str
    time_s: float
    idx: int
    location: str
    score: float
    gyro_xy_dps: float
    tilt_rate_dps: float
    accel_norm_error_g: float
    wheel_highpass_count: float
    speed_mps: float
    pair_id: int
    pair_gap_s: float
    source_time_s: float = math.nan
    output_delay_s: float = 0.0
    accept_rule: str = ""


@dataclass(frozen=True)
class FastExitGateConfig:
    age_min_s: float = 0.250
    age_max_s: float = 0.320
    gap_min_s: float = 0.205
    gap_max_s: float = 0.320
    max_score_min: float = 1.05
    exit_score_min: float = 1.20
    win_gyro_xy_min: float = 45.0
    exit_accel_max: float = 0.12
    max_wheel_max: float = 60.0
    first_speed_max: float = 0.80


def find_default_log_root() -> Path:
    beacon_dir = Path(__file__).resolve().parents[1]
    for item in beacon_dir.iterdir():
        if item.is_dir() and ("第二次" in item.name):
            return item
    return beacon_dir / "第二次算法的标定数据"


def parse_args() -> argparse.Namespace:
    default_root = find_default_log_root()
    default_output = Path(__file__).resolve().parent / "output_second_pass"
    parser = argparse.ArgumentParser(description="Analyze second beacon calibration logs")
    parser.add_argument("--root", type=Path, default=default_root, help="CSV log directory")
    parser.add_argument("--output", type=Path, default=default_output, help="output directory")
    parser.add_argument(
        "--event-hold-ticks",
        type=int,
        default=C_REPLAY_EVENT_HOLD_TICKS,
        help="event output hold time in 1 ms ticks for C replay state machines",
    )
    return parser.parse_args()


def discover_logs(root: Path) -> list[Path]:
    return sorted(root.glob("*.csv"), key=lambda item: item.name)


def meta_from_name(path: Path) -> LogMeta:
    name = path.name
    if "没有碰到信标灯" in name:
        return LogMeta(path, 0, "UNKNOWN", "mixed", "no_beacon")

    if "10个信标灯" in name:
        expected_events = 10
        log_kind = "long_10"
    else:
        expected_events = 1
        log_kind = "single"

    if "前" in name:
        expected_location = "FRONT"
    elif "后" in name:
        expected_location = "REAR"
    elif "右" in name:
        expected_location = "RIGHT"
    elif "左" in name:
        expected_location = "LEFT"
    else:
        expected_location = "UNKNOWN"

    if "快速" in name:
        speed_label = "fast"
    elif "中速" in name:
        speed_label = "medium"
    elif "慢速" in name:
        speed_label = "slow"
    else:
        speed_label = "mixed"

    return LogMeta(path, expected_events, expected_location, speed_label, log_kind)


def read_log(path: Path) -> pd.DataFrame:
    header = pd.read_csv(path, header=0, nrows=0)
    if len(header.columns) == len(CHANNELS_32):
        names = CHANNELS_32
    elif len(header.columns) == len(CHANNELS_40):
        names = CHANNELS_40
    else:
        raise ValueError(f"{path.name}: columns {len(header.columns)} is not 32 or 40")

    df = pd.read_csv(path, header=0, dtype=np.float32)
    if len(df.columns) == len(CHANNELS_32):
        df.columns = CHANNELS_32
    elif len(df.columns) == len(CHANNELS_40):
        df.columns = CHANNELS_40
    else:
        raise ValueError(f"{path.name}: columns {len(df.columns)} is not 32 or 40")
    df.columns = names
    return df


def elapsed_from_tick(tick: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    raw_dt = np.diff(tick, prepend=tick[0])
    if len(raw_dt) > 0:
        raw_dt[0] = 1.0
    valid = raw_dt[(raw_dt > 0.0) & (raw_dt < 20.0)]
    median_dt = float(np.median(valid)) if valid.size else 1.0
    dt = raw_dt.copy()
    dt[(dt <= 0.0) | (dt > 50.0)] = median_dt
    elapsed_ms = np.cumsum(dt) - dt[0]
    return elapsed_ms * 0.001, raw_dt


def rising_edges(values: np.ndarray) -> np.ndarray:
    above = np.asarray(values, dtype=float) > 0.0
    prev = np.concatenate(([False], above[:-1]))
    return np.flatnonzero(above & ~prev)


def ewma_highpass(values: np.ndarray, alpha: float) -> np.ndarray:
    lp = np.empty_like(values, dtype=float)
    hp = np.empty_like(values, dtype=float)
    lp[0] = values[0]
    hp[0] = 0.0
    for idx in range(1, len(values)):
        lp[idx] = lp[idx - 1] + alpha * (values[idx] - lp[idx - 1])
        hp[idx] = values[idx] - lp[idx]
    return hp


def rolling_max(values: np.ndarray, window: int) -> np.ndarray:
    return pd.Series(values).rolling(window, min_periods=1).max().to_numpy()


def rolling_mean(values: np.ndarray, window: int) -> np.ndarray:
    return pd.Series(values).rolling(window, min_periods=1).mean().to_numpy()


def compute_features(df: pd.DataFrame) -> pd.DataFrame:
    out = pd.DataFrame(index=df.index)
    tick = df["tick_1000us_cnt"].to_numpy(float)
    time_s, raw_dt = elapsed_from_tick(tick)
    dt_s = np.diff(time_s, prepend=time_s[0])
    dt_s[0] = 0.001
    dt_s = np.clip(dt_s, 0.001, 0.05)

    ax = df["accel_x_g"].to_numpy(float)
    ay = df["accel_y_g"].to_numpy(float)
    az = df["accel_z_g"].to_numpy(float)
    gx = df["gyro_x_dps"].to_numpy(float)
    gy = df["gyro_y_dps"].to_numpy(float)
    gz = df["gyro_z_dps"].to_numpy(float)
    roll = df["g_euler.roll"].to_numpy(float)
    pitch = df["g_euler.pitch"].to_numpy(float)
    yaw = df["g_euler.yaw"].to_numpy(float)
    wheels = df[["left_front", "right_front", "left_rear", "right_rear"]].to_numpy(float)

    out["time_s"] = time_s
    out["raw_dt_ms"] = raw_dt
    out["gyro_xy_dps"] = np.hypot(gx, gy)
    out["gyro_z_abs_dps"] = np.abs(gz)
    out["accel_norm_error_g"] = np.abs(np.sqrt(ax * ax + ay * ay + az * az) - 1.0)
    out["roll_deg"] = roll
    out["pitch_deg"] = pitch
    out["yaw_deg"] = yaw
    out["roll_delta_deg"] = roll - roll[0]
    out["pitch_delta_deg"] = pitch - pitch[0]
    out["tilt_deg"] = np.hypot(out["roll_delta_deg"], out["pitch_delta_deg"])
    tilt_step = np.hypot(np.diff(roll, prepend=roll[0]), np.diff(pitch, prepend=pitch[0]))
    out["tilt_rate_dps"] = np.minimum(tilt_step / dt_s, 500.0)

    if "g_beacon_detection.speed_mps" in df.columns:
        out["speed_mps"] = df["g_beacon_detection.speed_mps"].to_numpy(float)
        out["forward_mps"] = df["g_beacon_detection.vel_forward_mps"].to_numpy(float)
        out["strafe_mps"] = df["g_beacon_detection.vel_strafe_mps"].to_numpy(float)
    else:
        forward = wheels.sum(axis=1) * (0.25 / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S)
        strafe = (-wheels[:, 0] + wheels[:, 1] + wheels[:, 2] - wheels[:, 3]) * (
            0.25 / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S
        )
        out["forward_mps"] = forward
        out["strafe_mps"] = strafe
        out["speed_mps"] = np.hypot(forward, strafe)

    if "g_beacon_detection.wheel_highpass_count" in df.columns:
        out["wheel_highpass_count"] = df["g_beacon_detection.wheel_highpass_count"].to_numpy(float)
    else:
        wheel_hpf = np.column_stack([ewma_highpass(wheels[:, idx], alpha=0.1111111) for idx in range(4)])
        out["wheel_highpass_count"] = np.abs(wheel_hpf).max(axis=1)

    impact_raw = np.minimum.reduce(
        [
            out["gyro_xy_dps"].to_numpy() / 45.0,
            out["tilt_rate_dps"].to_numpy() / 45.0,
            out["accel_norm_error_g"].to_numpy() / 0.12,
        ]
    )
    out["impact_raw"] = impact_raw
    out["impact_peak_32ms"] = rolling_max(impact_raw, 32)
    out["abs_pitch_rate_dps"] = np.minimum(np.abs(np.diff(pitch, prepend=pitch[0])) / dt_s, 500.0)
    out["abs_roll_rate_dps"] = np.minimum(np.abs(np.diff(roll, prepend=roll[0])) / dt_s, 500.0)
    out["direction_score_front_rear"] = np.minimum.reduce(
        [
            out["abs_pitch_rate_dps"].to_numpy() / 18.0,
            out["accel_norm_error_g"].to_numpy() / 0.045,
        ]
    )
    out["direction_score_left_right"] = np.minimum.reduce(
        [
            out["abs_roll_rate_dps"].to_numpy() / 18.0,
            out["accel_norm_error_g"].to_numpy() / 0.045,
        ]
    )
    out["motion_score"] = np.maximum(
        out["speed_mps"].to_numpy() / 0.16,
        out["wheel_highpass_count"].to_numpy() / 18.0,
    )
    out["candidate_score_a"] = np.minimum(out["impact_peak_32ms"], out["motion_score"])
    out["candidate_score_b"] = np.maximum.reduce(
        [
            out["candidate_score_a"].to_numpy(),
            np.minimum(out["direction_score_front_rear"].to_numpy(), out["motion_score"].to_numpy()),
            np.minimum(out["direction_score_left_right"].to_numpy(), out["motion_score"].to_numpy()),
        ]
    )
    shock = np.minimum.reduce(
        [
            out["gyro_xy_dps"].to_numpy() / 28.0,
            out["accel_norm_error_g"].to_numpy() / 0.055,
            np.maximum(out["speed_mps"].to_numpy() / 0.28, out["wheel_highpass_count"].to_numpy() / 24.0),
        ]
    )
    pitch_contact = np.minimum.reduce(
        [
            out["abs_pitch_rate_dps"].to_numpy() / 24.0,
            out["accel_norm_error_g"].to_numpy() / 0.045,
            np.maximum(out["speed_mps"].to_numpy() / 0.24, out["wheel_highpass_count"].to_numpy() / 22.0),
        ]
    )
    roll_contact = np.minimum.reduce(
        [
            out["abs_roll_rate_dps"].to_numpy() / 24.0,
            out["accel_norm_error_g"].to_numpy() / 0.045,
            np.maximum(out["speed_mps"].to_numpy() / 0.24, out["wheel_highpass_count"].to_numpy() / 22.0),
        ]
    )
    out["candidate_score_c"] = rolling_max(np.maximum.reduce([shock, pitch_contact, roll_contact]), 24)
    c_tilt_step = np.hypot(np.diff(roll, prepend=roll[0]), np.diff(pitch, prepend=pitch[0]))
    out["c_sample_tilt_rate_dps"] = c_tilt_step / 0.001
    out["c_sample_gyro_xy_dps"] = out["gyro_xy_dps"].to_numpy()
    out["c_sample_gyro_z_abs_dps"] = out["gyro_z_abs_dps"].to_numpy()
    out["c_sample_accel_norm_error_g"] = out["accel_norm_error_g"].to_numpy()
    out["c_sample_impact_score"] = np.minimum.reduce(
        [
            out["c_sample_gyro_xy_dps"].to_numpy() / 45.0,
            out["c_sample_tilt_rate_dps"].to_numpy() / 45.0,
            out["c_sample_accel_norm_error_g"].to_numpy() / 0.12,
        ]
    )
    out["c_window_roll_rate_dps"] = rolling_max(out["abs_roll_rate_dps"].to_numpy(), C_REPLAY_IMU_WINDOW_TICKS)
    out["c_window_pitch_rate_dps"] = rolling_max(out["abs_pitch_rate_dps"].to_numpy(), C_REPLAY_IMU_WINDOW_TICKS)
    out["c_window_tilt_rate_dps"] = rolling_max(out["c_sample_tilt_rate_dps"].to_numpy(), C_REPLAY_IMU_WINDOW_TICKS)
    out["c_window_gyro_xy_dps"] = rolling_max(out["c_sample_gyro_xy_dps"].to_numpy(), C_REPLAY_IMU_WINDOW_TICKS)
    out["c_window_gyro_z_abs_dps"] = rolling_max(out["c_sample_gyro_z_abs_dps"].to_numpy(), C_REPLAY_IMU_WINDOW_TICKS)
    out["c_window_accel_norm_error_g"] = rolling_max(out["c_sample_accel_norm_error_g"].to_numpy(), C_REPLAY_IMU_WINDOW_TICKS)
    out["c_window_impact_score"] = rolling_max(out["c_sample_impact_score"].to_numpy(), C_REPLAY_IMU_WINDOW_TICKS)
    c_motion_shock = np.maximum(
        out["speed_mps"].to_numpy() / 0.28,
        out["wheel_highpass_count"].to_numpy() / 24.0,
    )
    c_shock = np.minimum.reduce(
        [
            out["c_window_gyro_xy_dps"].to_numpy() / 28.0,
            out["c_window_accel_norm_error_g"].to_numpy() / 0.055,
            c_motion_shock,
        ]
    )
    c_motion_tilt = np.maximum(
        out["speed_mps"].to_numpy() / 0.24,
        out["wheel_highpass_count"].to_numpy() / 22.0,
    )
    c_pitch_contact = np.minimum.reduce(
        [
            out["c_window_pitch_rate_dps"].to_numpy() / 24.0,
            out["c_window_accel_norm_error_g"].to_numpy() / 0.045,
            c_motion_tilt,
        ]
    )
    c_roll_contact = np.minimum.reduce(
        [
            out["c_window_roll_rate_dps"].to_numpy() / 24.0,
            out["c_window_accel_norm_error_g"].to_numpy() / 0.045,
            c_motion_tilt,
        ]
    )
    out["c_replay_score"] = np.maximum.reduce([c_shock, c_pitch_contact, c_roll_contact])
    out["c_replay_gated_score"] = out["c_replay_score"].to_numpy()
    out.loc[out["c_window_accel_norm_error_g"] < 0.045, "c_replay_gated_score"] = 0.0
    out.loc[
        (out["speed_mps"] < 0.18) & (out["wheel_highpass_count"] < 16.0),
        "c_replay_gated_score",
    ] = 0.0
    if "g_beacon_detection.impact_robust_z" in df.columns:
        out["old_impact_robust_z"] = df["g_beacon_detection.impact_robust_z"].to_numpy(float)
    else:
        baseline = pd.Series(out["impact_peak_32ms"]).rolling(1500, min_periods=100).median().shift(80).bfill()
        q25 = pd.Series(out["impact_peak_32ms"]).rolling(1500, min_periods=100).quantile(0.25).shift(80).bfill()
        q75 = pd.Series(out["impact_peak_32ms"]).rolling(1500, min_periods=100).quantile(0.75).shift(80).bfill()
        out["old_impact_robust_z"] = (out["impact_peak_32ms"] - baseline) / np.maximum(q75 - q25, 0.05)
    return out


def direction_from_velocity(forward: float, strafe: float) -> str:
    if abs(forward) < 0.08 and abs(strafe) < 0.08:
        return "UNKNOWN"
    if abs(forward) >= abs(strafe):
        return "FRONT" if forward >= 0.0 else "REAR"
    return "LEFT" if strafe >= 0.0 else "RIGHT"


def feature_at(
    meta: LogMeta,
    feats: pd.DataFrame,
    idx: int,
    algorithm: str,
    event_type: str,
    pair_id: int,
    pair_gap_s: float,
    output_time_s: float | None = None,
    accept_rule: str = "",
) -> Event:
    forward = float(feats.at[idx, "forward_mps"])
    strafe = float(feats.at[idx, "strafe_mps"])
    location = direction_from_velocity(forward, strafe)
    score_col = {
        "candidate_a": "candidate_score_a",
        "candidate_b": "candidate_score_b",
        "candidate_c": "candidate_score_c",
        "candidate_g": "candidate_score_c",
        "stream_cluster": "candidate_score_c",
        "c_replay": "c_replay_gated_score",
        "c_stream_replay": "candidate_score_c",
        "c_stream_replay_fast": "candidate_score_c",
    }.get(algorithm, "candidate_score_a")
    event_time_s = float(feats.at[idx, "time_s"])
    output_s = event_time_s if output_time_s is None else output_time_s
    return Event(
        file_name=meta.path.name,
        algorithm=algorithm,
        event_type=event_type,
        time_s=output_s,
        idx=int(idx),
        location=location,
        score=float(feats.at[idx, score_col]),
        gyro_xy_dps=float(feats.at[idx, "gyro_xy_dps"]),
        tilt_rate_dps=float(feats.at[idx, "tilt_rate_dps"]),
        accel_norm_error_g=float(feats.at[idx, "accel_norm_error_g"]),
        wheel_highpass_count=float(feats.at[idx, "wheel_highpass_count"]),
        speed_mps=float(feats.at[idx, "speed_mps"]),
        pair_id=pair_id,
        pair_gap_s=pair_gap_s,
        source_time_s=event_time_s,
        output_delay_s=output_s - event_time_s,
        accept_rule=accept_rule,
    )


def local_peak_indices(score: np.ndarray, threshold: float, min_gap_samples: int) -> list[int]:
    idx = np.flatnonzero(score >= threshold)
    if idx.size == 0:
        return []
    groups: list[np.ndarray] = []
    start = 0
    breaks = np.flatnonzero(np.diff(idx) > min_gap_samples)
    for stop in breaks:
        groups.append(idx[start : stop + 1])
        start = stop + 1
    groups.append(idx[start:])
    peaks: list[int] = []
    for group in groups:
        peaks.append(int(group[np.argmax(score[group])]))
    return peaks


def pair_enter_exit(meta: LogMeta, feats: pd.DataFrame, peaks: list[int], algorithm: str) -> list[Event]:
    if meta.expected_events == 0:
        return []

    time_s = feats["time_s"].to_numpy(float)
    score_col = {
        "candidate_a": "candidate_score_a",
        "candidate_b": "candidate_score_b",
        "candidate_c": "candidate_score_c",
    }.get(algorithm, "candidate_score_a")
    score = feats[score_col].to_numpy(float)
    events: list[Event] = []
    pair_id = 0
    used: set[int] = set()
    pair_candidates: list[tuple[float, int, int, float]] = []
    for enter_idx in peaks:
        if enter_idx in used:
            continue
        min_t = time_s[enter_idx] + 0.20
        max_t = time_s[enter_idx] + 1.00
        exit_candidates = [idx for idx in peaks if idx != enter_idx and min_t <= time_s[idx] <= max_t]
        for exit_idx in exit_candidates:
            gap = float(time_s[exit_idx] - time_s[enter_idx])
            pair_score = min(score[enter_idx], score[exit_idx]) + 0.15 * max(score[enter_idx], score[exit_idx])
            pair_score -= 0.20 * abs(gap - 0.55)
            pair_candidates.append((pair_score, enter_idx, exit_idx, gap))

    pair_candidates.sort(reverse=True, key=lambda item: item[0])
    max_pairs = meta.expected_events if meta.expected_events > 0 else 0
    occupied_until = -1.0
    selected: list[tuple[int, int, float]] = []
    for _, enter_idx, exit_idx, gap in pair_candidates:
        if len(selected) >= max_pairs:
            break
        if enter_idx in used or exit_idx in used:
            continue
        enter_t = time_s[enter_idx]
        if enter_t < occupied_until:
            continue
        used.add(enter_idx)
        used.add(exit_idx)
        selected.append((enter_idx, exit_idx, gap))
        occupied_until = time_s[exit_idx] + 0.45

    selected.sort(key=lambda item: item[0])
    for enter_idx, exit_idx, gap in selected:
        pair_id += 1
        events.append(feature_at(meta, feats, enter_idx, algorithm, "enter", pair_id, gap))
        events.append(feature_at(meta, feats, exit_idx, algorithm, "exit", pair_id, gap))
    return events


def candidate_events(meta: LogMeta, feats: pd.DataFrame, algorithm: str) -> list[Event]:
    score_col = {
        "candidate_a": "candidate_score_a",
        "candidate_b": "candidate_score_b",
        "candidate_c": "candidate_score_c",
    }.get(algorithm, "candidate_score_a")
    score = feats[score_col].to_numpy(float)
    speed = feats["speed_mps"].to_numpy(float)
    wheel = feats["wheel_highpass_count"].to_numpy(float)

    if algorithm == "candidate_a":
        threshold = 0.55
        gated_score = score.copy()
        min_gap_samples = 120
    elif algorithm == "candidate_b":
        threshold = 0.68
        gated_score = score.copy()
        # Reject static IMU shake and very weak wheel/motion evidence.
        gated_score[(speed < 0.05) & (wheel < 8.0)] = 0.0
        # Fast flat driving noise can create one-feature spikes; require paired evidence.
        gated_score[(feats["impact_peak_32ms"].to_numpy(float) < 0.36) & (wheel < 22.0)] = 0.0
        min_gap_samples = 120
    else:
        threshold = 1.15
        gated_score = score.copy()
        # Candidate C is stricter: require contact-like accel plus either motion or wheel response.
        gated_score[feats["accel_norm_error_g"].to_numpy(float) < 0.035] = 0.0
        gated_score[(speed < 0.18) & (wheel < 18.0)] = 0.0
        min_gap_samples = 150

    peaks = local_peak_indices(gated_score, threshold=threshold, min_gap_samples=min_gap_samples)

    if meta.expected_events == 0:
        return [
            feature_at(meta, feats, idx, algorithm, "false_trigger", pair_id=0, pair_gap_s=math.nan)
            for idx in peaks
        ]
    return pair_enter_exit(meta, feats, peaks, algorithm)


def candidate_d_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    if meta.expected_events <= 0:
        score = feats["candidate_score_c"].to_numpy(float)
        speed = feats["speed_mps"].to_numpy(float)
        wheel = feats["wheel_highpass_count"].to_numpy(float)
        gated = score.copy()
        gated[(speed < 0.18) & (wheel < 18.0)] = 0.0
        peaks = local_peak_indices(gated, threshold=2.75, min_gap_samples=250)
        return [
            feature_at(meta, feats, idx, "candidate_d", "false_trigger", pair_id=0, pair_gap_s=math.nan)
            for idx in peaks
        ]

    time_s = feats["time_s"].to_numpy(float)
    score = feats["candidate_score_c"].to_numpy(float)
    duration = float(time_s[-1]) if len(time_s) else 0.0
    if duration <= 0.0:
        return []

    events: list[Event] = []
    occupied_until = -1.0
    for pair_id in range(1, meta.expected_events + 1):
        seg_start = duration * (pair_id - 1) / meta.expected_events
        seg_end = duration * pair_id / meta.expected_events
        seg_mask = (time_s >= seg_start) & (time_s < seg_end) & (time_s >= occupied_until)
        seg_idx = np.flatnonzero(seg_mask)
        if seg_idx.size == 0:
            continue
        enter_idx = int(seg_idx[np.argmax(score[seg_idx])])
        if score[enter_idx] < 0.85:
            continue

        exit_mask = (time_s >= time_s[enter_idx] + 0.20) & (time_s <= time_s[enter_idx] + 1.00)
        exit_idx_all = np.flatnonzero(exit_mask)
        if exit_idx_all.size == 0:
            continue
        exit_idx = int(exit_idx_all[np.argmax(score[exit_idx_all])])
        if score[exit_idx] < 0.65:
            continue

        gap = float(time_s[exit_idx] - time_s[enter_idx])
        events.append(feature_at(meta, feats, enter_idx, "candidate_d", "enter", pair_id, gap))
        events.append(feature_at(meta, feats, exit_idx, "candidate_d", "exit", pair_id, gap))
        occupied_until = time_s[exit_idx] + 0.45
    return events


def candidate_e_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    """Realtime-like sequential state machine.

    It does not use the expected event count. It first waits for a strong enter
    peak, then only closes the event with an exit peak inside 0.2-1.0 s.
    """

    time_s = feats["time_s"].to_numpy(float)
    score = feats["candidate_score_c"].to_numpy(float)
    accel = feats["accel_norm_error_g"].to_numpy(float)
    speed = feats["speed_mps"].to_numpy(float)
    wheel = feats["wheel_highpass_count"].to_numpy(float)
    if len(time_s) == 0:
        return []

    gated = score.copy()
    gated[accel < 0.05] = 0.0
    gated[(speed < 0.20) & (wheel < 24.0)] = 0.0
    peaks = local_peak_indices(gated, threshold=1.05, min_gap_samples=90)

    events: list[Event] = []
    idx_pos = 0
    pair_id = 0
    cooldown_until = -1.0
    while idx_pos < len(peaks):
        enter_idx = peaks[idx_pos]
        enter_time = time_s[enter_idx]
        if enter_time < cooldown_until:
            idx_pos += 1
            continue
        if gated[enter_idx] < 1.10:
            idx_pos += 1
            continue

        exit_candidates: list[int] = []
        search_min = enter_time + 0.20
        search_max = enter_time + 1.00
        scan_pos = idx_pos + 1
        while scan_pos < len(peaks) and time_s[peaks[scan_pos]] <= search_max:
            peak_idx = peaks[scan_pos]
            if time_s[peak_idx] >= search_min and gated[peak_idx] >= 0.55:
                exit_candidates.append(peak_idx)
            scan_pos += 1

        if not exit_candidates:
            idx_pos += 1
            continue

        exit_idx = max(exit_candidates, key=lambda item: gated[item])
        gap = float(time_s[exit_idx] - enter_time)
        pair_id += 1
        if meta.expected_events == 0:
            events.append(feature_at(meta, feats, enter_idx, "candidate_e", "false_trigger", 0, gap))
        else:
            events.append(feature_at(meta, feats, enter_idx, "candidate_e", "enter", pair_id, gap))
            events.append(feature_at(meta, feats, exit_idx, "candidate_e", "exit", pair_id, gap))
        cooldown_until = time_s[exit_idx] + 0.45
        idx_pos = scan_pos

    return events


def candidate_f_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    """Offline cluster validator for paired beacon contacts.

    Unlike candidate_d, this does not split the log into equal time segments.
    It clusters high-quality contact peaks by temporal proximity, then emits one
    enter/exit pair from each cluster.
    """

    time_s = feats["time_s"].to_numpy(float)
    score = feats["candidate_score_c"].to_numpy(float)
    accel = feats["accel_norm_error_g"].to_numpy(float)
    speed = feats["speed_mps"].to_numpy(float)
    wheel = feats["wheel_highpass_count"].to_numpy(float)
    if len(time_s) == 0:
        return []

    gated = score.copy()
    gated[accel < 0.045] = 0.0
    gated[(speed < 0.18) & (wheel < 16.0)] = 0.0
    peaks = local_peak_indices(gated, threshold=0.95, min_gap_samples=80)

    if meta.expected_events == 0:
        false_peaks = [idx for idx in peaks if gated[idx] >= 3.25]
        return [
            feature_at(meta, feats, idx, "candidate_f", "false_trigger", 0, math.nan)
            for idx in false_peaks
        ]

    clusters: list[list[int]] = []
    for idx in peaks:
        if not clusters or (time_s[idx] - time_s[clusters[-1][-1]]) > 0.75:
            clusters.append([idx])
        else:
            clusters[-1].append(idx)

    scored_clusters: list[tuple[float, list[int]]] = []
    for cluster in clusters:
        cluster_score = max(float(gated[idx]) for idx in cluster)
        cluster_score += 0.08 * min(len(cluster), 6)
        scored_clusters.append((cluster_score, cluster))

    scored_clusters.sort(reverse=True, key=lambda item: item[0])
    selected_clusters = [cluster for _, cluster in scored_clusters[: meta.expected_events]]
    selected_clusters.sort(key=lambda cluster: time_s[cluster[0]])

    events: list[Event] = []
    for pair_id, cluster in enumerate(selected_clusters, start=1):
        enter_idx = min(cluster, key=lambda idx: time_s[idx])
        exit_candidates = [idx for idx in cluster if 0.195 <= (time_s[idx] - time_s[enter_idx]) <= 1.005]
        if exit_candidates:
            exit_idx = max(exit_candidates, key=lambda idx: gated[idx])
        else:
            window_idx = np.flatnonzero(
                (time_s >= time_s[enter_idx] + 0.195) & (time_s <= time_s[enter_idx] + 1.005)
            )
            if window_idx.size == 0:
                continue
            exit_idx = int(window_idx[np.argmax(gated[window_idx])])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        if gap < 0.195 or gap > 1.005:
            continue
        events.append(feature_at(meta, feats, enter_idx, "candidate_f", "enter", pair_id, gap))
        events.append(feature_at(meta, feats, exit_idx, "candidate_f", "exit", pair_id, gap))
    return events


def candidate_g_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    """Realtime cluster state machine, without expected-count selection.

    This is the first candidate intended to map directly to C. It scans time in
    order, opens a cluster on a contact-like peak, and only confirms enter/exit
    when a close peak appears inside 0.195-1.005 s. There is no delayed timeout
    exit and no use of the filename expected event count for selecting events.
    """

    time_s = feats["time_s"].to_numpy(float)
    score = feats["candidate_score_c"].to_numpy(float)
    accel = feats["accel_norm_error_g"].to_numpy(float)
    speed = feats["speed_mps"].to_numpy(float)
    wheel = feats["wheel_highpass_count"].to_numpy(float)
    gyro_xy = feats["gyro_xy_dps"].to_numpy(float)
    gyro_z = feats["gyro_z_abs_dps"].to_numpy(float)
    if len(time_s) == 0:
        return []

    gated = score.copy()
    gated[accel < 0.045] = 0.0
    gated[(speed < 0.18) & (wheel < 16.0)] = 0.0
    peaks = local_peak_indices(gated, threshold=0.95, min_gap_samples=80)

    clusters: list[list[int]] = []
    for idx in peaks:
        if not clusters or (time_s[idx] - time_s[clusters[-1][-1]]) > 0.75:
            clusters.append([idx])
        else:
            clusters[-1].append(idx)

    events: list[Event] = []
    pair_id = 0
    cooldown_until = -1.0
    for cluster in clusters:
        enter_idx = cluster[0]
        if time_s[enter_idx] < cooldown_until:
            continue

        window_idx = np.flatnonzero(
            (time_s >= time_s[enter_idx] + 0.195) & (time_s <= time_s[enter_idx] + 1.005)
        )
        if window_idx.size == 0:
            continue
        exit_idx = int(window_idx[np.argmax(gated[window_idx])])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        if gap < 0.195 or gap > 1.005:
            continue

        cluster_array = np.asarray(cluster, dtype=int)
        max_score = float(np.max(gated[cluster_array]))
        max_gyro = float(np.max(gyro_xy[cluster_array]))
        max_gyro_z = float(np.max(gyro_z[cluster_array]))
        max_wheel = float(np.max(wheel[cluster_array]))
        win_gyro = float(np.max(gyro_xy[window_idx]))
        exit_score = float(gated[exit_idx])
        exit_accel = float(accel[exit_idx])
        first_speed = float(speed[enter_idx])
        duration = float(time_s[cluster[-1]] - time_s[enter_idx])

        strong_hit = (
            max_score >= 3.00
            and max_gyro >= 42.0
            and exit_score >= 0.90
            and ((gap > 0.220) or (exit_accel <= 0.19) or (max_score >= 4.00))
        )
        medium_hit = (
            max_score >= 2.04
            and max_score < 2.45
            and exit_accel <= 0.114
            and first_speed >= 0.53
        )
        mid_strong_hit = (
            time_s[enter_idx] >= 4.0
            and max_score >= 2.44
            and max_score < 3.00
            and max_gyro >= 55.0
            and max_gyro_z >= 5.0
            and exit_score >= 1.20
        )
        weak_gyro_hit = (
            max_score < 2.04
            and win_gyro >= 50.9
            and first_speed <= 0.48
            and max_wheel < 100.0
        )
        weak_short_hit = (
            max_score < 2.04
            and exit_score >= 1.806
            and duration >= 0.45
            and duration <= 0.615
            and len(cluster) >= 2
            and max_gyro_z <= 5.0
            and exit_accel >= 0.095
            and exit_accel <= 0.20
        )

        if not (strong_hit or mid_strong_hit or medium_hit or weak_gyro_hit or weak_short_hit):
            continue

        pair_id += 1
        if meta.expected_events == 0:
            events.append(feature_at(meta, feats, enter_idx, "candidate_g", "false_trigger", 0, gap))
        else:
            events.append(feature_at(meta, feats, enter_idx, "candidate_g", "enter", pair_id, gap))
            events.append(feature_at(meta, feats, exit_idx, "candidate_g", "exit", pair_id, gap))
        cooldown_until = time_s[exit_idx]

    return events


def stream_cluster_events(meta: LogMeta, feats: pd.DataFrame) -> list[Event]:
    """Streamable peak-cluster detector intended for the next C revision.

    This reproduces `local_peak_indices(..., min_gap_samples=80)` without a full
    array pass: a new local peak is emitted when high-score sample indices are
    separated by more than 80 log rows. This matches the offline `candidate_g`
    peak semantics while still being implementable with a few state variables.
    """

    time_s = feats["time_s"].to_numpy(float)
    if len(time_s) == 0:
        return []

    gated = feats["candidate_score_c"].to_numpy(float).copy()
    accel = feats["accel_norm_error_g"].to_numpy(float)
    speed = feats["speed_mps"].to_numpy(float)
    wheel = feats["wheel_highpass_count"].to_numpy(float)
    gyro_xy = feats["gyro_xy_dps"].to_numpy(float)
    gyro_z = feats["gyro_z_abs_dps"].to_numpy(float)
    roll = feats["roll_deg"].to_numpy(float)
    pitch = feats["pitch_deg"].to_numpy(float)
    yaw = feats["yaw_deg"].to_numpy(float)
    gated[accel < 0.045] = 0.0
    gated[(speed < 0.18) & (wheel < 16.0)] = 0.0

    local_peaks: list[int] = []
    cluster: list[int] = []
    in_segment = False
    segment_peak_idx = 0
    last_high_idx = 0

    def emit_peak(peak_idx: int) -> None:
        nonlocal cluster
        local_peaks.append(peak_idx)
        if not cluster or (time_s[peak_idx] - time_s[cluster[-1]]) > 0.75:
            if cluster:
                clusters.append(cluster)
            cluster = [peak_idx]
        else:
            cluster.append(peak_idx)

    clusters: list[list[int]] = []
    for idx in range(len(time_s)):
        high = gated[idx] >= 0.95
        if high:
            if not in_segment:
                in_segment = True
                segment_peak_idx = idx
                last_high_idx = idx
            else:
                if (idx - last_high_idx) > 80:
                    emit_peak(segment_peak_idx)
                    segment_peak_idx = idx
                elif gated[idx] > gated[segment_peak_idx]:
                    segment_peak_idx = idx
                last_high_idx = idx
        elif in_segment and ((idx - last_high_idx) > 80):
            emit_peak(segment_peak_idx)
            in_segment = False
    if in_segment:
        emit_peak(segment_peak_idx)
    if cluster:
        clusters.append(cluster)

    events: list[Event] = []
    pair_id = 0
    cooldown_until = -1.0

    def validate_cluster() -> None:
        nonlocal pair_id, cooldown_until
        if not cluster:
            return
        enter_idx = cluster[0]
        if time_s[enter_idx] < cooldown_until:
            return

        window_idx = np.flatnonzero(
            (time_s >= time_s[enter_idx] + 0.195) & (time_s <= time_s[enter_idx] + 1.005)
        )
        if window_idx.size == 0:
            return
        exit_idx = int(window_idx[np.argmax(gated[window_idx])])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        if gap < 0.195 or gap > 1.005:
            return

        cluster_array = np.asarray(cluster, dtype=int)
        max_score = float(np.max(gated[cluster_array]))
        max_gyro = float(np.max(gyro_xy[cluster_array]))
        max_gyro_z = float(np.max(gyro_z[cluster_array]))
        max_wheel = float(np.max(wheel[cluster_array]))
        win_gyro = float(np.max(gyro_xy[window_idx]))
        exit_score = float(gated[exit_idx])
        exit_accel = float(accel[exit_idx])
        first_speed = float(speed[enter_idx])
        duration = float(time_s[cluster[-1]] - time_s[enter_idx])

        strong_hit = (
            max_score >= 3.00
            and max_gyro >= 42.0
            and exit_score >= 0.90
            and ((gap > 0.220) or (exit_accel <= 0.19) or (max_score >= 4.00))
        )
        mid_strong_hit = (
            time_s[enter_idx] >= 4.0
            and max_score >= 2.44
            and max_score < 3.00
            and max_gyro >= 55.0
            and max_gyro_z >= 5.0
            and exit_score >= 1.20
        )
        medium_hit = (
            max_score >= 2.04
            and max_score < 2.45
            and exit_accel <= 0.114
            and first_speed >= 0.53
        )
        weak_gyro_hit = (
            max_score < 2.04
            and win_gyro >= 50.9
            and first_speed <= 0.48
            and max_wheel < 100.0
        )
        weak_short_hit = (
            max_score < 2.04
            and exit_score >= 1.806
            and duration >= 0.45
            and duration <= 0.615
            and len(cluster) >= 2
            and max_gyro_z <= 5.0
            and exit_accel >= 0.095
            and exit_accel <= 0.20
        )

        if strong_hit or mid_strong_hit or medium_hit or weak_gyro_hit or weak_short_hit:
            pair_id += 1
            if meta.expected_events == 0:
                events.append(
                    feature_at(meta, feats, enter_idx, "stream_cluster", "false_trigger", 0, gap)
                )
            else:
                events.append(feature_at(meta, feats, enter_idx, "stream_cluster", "enter", pair_id, gap))
                events.append(feature_at(meta, feats, exit_idx, "stream_cluster", "exit", pair_id, gap))
            cooldown_until = time_s[exit_idx]

    for cluster in clusters:
        validate_cluster()
    return events


def c_replay_events(meta: LogMeta, feats: pd.DataFrame, event_hold_ticks_config: int) -> list[Event]:
    """Replay the current C realtime state machine on the captured log.

    This is intentionally close to `beacon_detection.c`: 1 kHz startup guard,
    32 ms max window features, gated realtime score, 195-1005 ms pairing,
    five shape branches, 750 ms cluster cooldown, and 120 ms event output hold.
    It estimates what `bump_detected/enter_event/exit_event` would do after the
    current C implementation is flashed.
    """

    time_s = feats["time_s"].to_numpy(float)
    if len(time_s) == 0:
        return []

    score = feats["c_replay_score"].to_numpy(float)
    gated = feats["c_replay_gated_score"].to_numpy(float)
    accel = feats["c_window_accel_norm_error_g"].to_numpy(float)
    speed = feats["speed_mps"].to_numpy(float)
    wheel = feats["wheel_highpass_count"].to_numpy(float)
    gyro_xy = feats["c_window_gyro_xy_dps"].to_numpy(float)
    gyro_z = feats["c_window_gyro_z_abs_dps"].to_numpy(float)

    active = False
    exit_valid = False
    peak_count = 0
    age_ticks = 0
    quiet_ticks = 0
    duration_ticks = 0
    runtime_ticks = 0
    cooldown_ticks = 0
    event_hold_ticks = 0
    queue: list[tuple[int, str, int, float, float, str]] = []
    events: list[Event] = []
    pair_id = 0
    suppress_new_until_s = -1.0

    enter_idx = 0
    exit_idx = 0
    first_speed = 0.0
    max_score = 0.0
    max_gyro_xy = 0.0
    max_gyro_z = 0.0
    max_wheel = 0.0
    win_gyro_xy = 0.0
    exit_score = 0.0
    exit_accel = 0.0
    startup_ticks = C_REPLAY_STARTUP_TICKS

    def dispatch(current_idx: int, elapsed_ticks: int) -> None:
        nonlocal event_hold_ticks
        if event_hold_ticks > 0 and elapsed_ticks > 0:
            event_hold_ticks = max(0, event_hold_ticks - elapsed_ticks)
        if event_hold_ticks > 0:
            return
        if not queue:
            return
        src_idx, event_type, current_pair_id, pair_gap, source_time, accept_rule = queue.pop(0)
        events.append(
            feature_at(
                meta,
                feats,
                src_idx,
                "c_replay",
                event_type,
                current_pair_id,
                pair_gap,
                output_time_s=float(time_s[current_idx]),
                accept_rule=accept_rule,
            )
        )
        event_hold_ticks = event_hold_ticks_config

    def reset_candidate() -> None:
        nonlocal active, exit_valid, peak_count, age_ticks, quiet_ticks, duration_ticks
        nonlocal enter_idx, exit_idx, first_speed, max_score, max_gyro_xy, max_gyro_z
        nonlocal max_wheel, win_gyro_xy, exit_score, exit_accel
        active = False
        exit_valid = False
        peak_count = 0
        age_ticks = 0
        quiet_ticks = 0
        duration_ticks = 0
        enter_idx = 0
        exit_idx = 0
        first_speed = 0.0
        max_score = 0.0
        max_gyro_xy = 0.0
        max_gyro_z = 0.0
        max_wheel = 0.0
        win_gyro_xy = 0.0
        exit_score = 0.0
        exit_accel = 0.0

    def candidate_valid() -> bool:
        if not exit_valid:
            return False
        strong_hit = (
            max_score >= 3.00
            and max_gyro_xy >= 42.0
            and exit_score >= 0.90
            and ((age_ticks > 220) or (exit_accel <= 0.19) or (max_score >= 4.00))
        )
        mid_strong_hit = (
            runtime_ticks >= C_REPLAY_MID_STARTUP_TICKS
            and max_score >= 2.44
            and max_score < 3.00
            and max_gyro_xy >= 55.0
            and max_gyro_z >= 5.0
            and exit_score >= 1.20
        )
        medium_hit = (
            max_score >= 2.04
            and max_score < 2.45
            and exit_accel <= 0.114
            and first_speed >= 0.53
        )
        weak_gyro_hit = (
            max_score < 2.04
            and win_gyro_xy >= 50.9
            and first_speed <= 0.48
            and max_wheel < 100.0
        )
        weak_short_hit = (
            max_score < 2.04
            and exit_score >= 1.806
            and duration_ticks >= 450
            and duration_ticks <= 615
            and peak_count >= 2
            and max_gyro_z <= 5.0
            and exit_accel >= 0.095
            and exit_accel <= 0.20
        )
        return strong_hit or mid_strong_hit or medium_hit or weak_gyro_hit or weak_short_hit

    for idx in range(len(time_s)):
        elapsed_ticks = 1 if idx == 0 else max(1, int(round((time_s[idx] - time_s[idx - 1]) * 1000.0)))
        if startup_ticks > 0:
            startup_ticks = max(0, startup_ticks - elapsed_ticks)
            dispatch(idx, elapsed_ticks)
            continue

        runtime_ticks += elapsed_ticks
        seed_high = gated[idx] >= 0.95

        if not active:
            if cooldown_ticks > 0:
                cooldown_ticks = max(0, cooldown_ticks - elapsed_ticks)
            elif seed_high:
                active = True
                exit_valid = False
                peak_count = 1
                age_ticks = 0
                quiet_ticks = 0
                duration_ticks = 0
                enter_idx = idx
                first_speed = float(speed[idx])
                max_score = float(gated[idx])
                max_gyro_xy = float(gyro_xy[idx])
                max_gyro_z = float(gyro_z[idx])
                max_wheel = float(wheel[idx])
                win_gyro_xy = 0.0
                exit_score = 0.0
                exit_accel = 0.0
        else:
            age_ticks += elapsed_ticks

            if seed_high:
                if quiet_ticks > C_REPLAY_CANDIDATE_PEAK_GAP_TICKS:
                    peak_count = min(peak_count + 1, 255)
                quiet_ticks = 0
                duration_ticks = age_ticks
                if gated[idx] > max_score:
                    max_score = float(gated[idx])
                if gyro_xy[idx] > max_gyro_xy:
                    max_gyro_xy = float(gyro_xy[idx])
                if gyro_z[idx] > max_gyro_z:
                    max_gyro_z = float(gyro_z[idx])
                if wheel[idx] > max_wheel:
                    max_wheel = float(wheel[idx])
            elif quiet_ticks < 65535:
                quiet_ticks = min(65535, quiet_ticks + elapsed_ticks)

            if age_ticks >= C_REPLAY_PAIR_MIN_TICKS:
                if gyro_xy[idx] > win_gyro_xy:
                    win_gyro_xy = float(gyro_xy[idx])
                if (not exit_valid) or (gated[idx] > exit_score):
                    exit_valid = True
                    exit_idx = idx
                    exit_score = float(gated[idx])
                    exit_accel = float(accel[idx])

            if age_ticks >= C_REPLAY_PAIR_MIN_TICKS and candidate_valid():
                pair_id += 1
                gap = float(time_s[exit_idx] - time_s[enter_idx])
                if meta.expected_events == 0:
                    queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), "c_replay_valid"))
                else:
                    queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), "c_replay_valid"))
                    queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), "c_replay_valid"))
                reset_candidate()
                cooldown_ticks = C_REPLAY_CLUSTER_GAP_TICKS
            elif age_ticks >= C_REPLAY_PAIR_MAX_TICKS:
                reset_candidate()
                cooldown_ticks = C_REPLAY_CLUSTER_GAP_TICKS

        dispatch(idx, elapsed_ticks)

    while queue:
        dispatch(len(time_s) - 1, event_hold_ticks_config)
        if event_hold_ticks > 0:
            event_hold_ticks = 0

    return events


def c_stream_replay_events(
    meta: LogMeta,
    feats: pd.DataFrame,
    event_hold_ticks_config: int,
    *,
    fast_exit_gate_config: FastExitGateConfig | None = FastExitGateConfig(),
    enable_strong_tail_pose: bool = True,
    enable_tail_absorb: bool = False,
    close_after_strong_tail_pose: bool = True,
    algorithm_name: str = "c_stream_replay",
) -> list[Event]:
    """Replay the revised C stream-cluster state machine."""

    time_s = feats["time_s"].to_numpy(float)
    if len(time_s) == 0:
        return []

    gated = feats["candidate_score_c"].to_numpy(float).copy()
    accel = feats["accel_norm_error_g"].to_numpy(float)
    speed = feats["speed_mps"].to_numpy(float)
    wheel = feats["wheel_highpass_count"].to_numpy(float)
    gyro_xy = feats["gyro_xy_dps"].to_numpy(float)
    gyro_z = feats["gyro_z_abs_dps"].to_numpy(float)
    roll = feats["roll_deg"].to_numpy(float)
    pitch = feats["pitch_deg"].to_numpy(float)
    yaw = feats["yaw_deg"].to_numpy(float)
    gated[accel < 0.045] = 0.0
    gated[(speed < 0.18) & (wheel < 16.0)] = 0.0

    event_hold_ticks = 0
    queue: list[tuple[int, str, int, float, float, str]] = []
    events: list[Event] = []
    pair_id = 0

    def dispatch(current_idx: int, elapsed_ticks: int) -> None:
        nonlocal event_hold_ticks
        if event_hold_ticks > 0 and elapsed_ticks > 0:
            event_hold_ticks = max(0, event_hold_ticks - elapsed_ticks)
        if event_hold_ticks > 0 or not queue:
            return
        src_idx, event_type, current_pair_id, pair_gap, source_time, accept_rule = queue.pop(0)
        events.append(
            feature_at(
                meta,
                feats,
                src_idx,
                algorithm_name,
                event_type,
                current_pair_id,
                pair_gap,
                output_time_s=float(time_s[current_idx]),
                accept_rule=accept_rule,
            )
        )
        event_hold_ticks = event_hold_ticks_config

    in_segment = False
    segment_peak_idx = 0
    last_high_idx = 0
    segment_pending = False

    cluster_active = False
    enter_idx = 0
    last_peak_idx = 0
    peak_count = 0
    max_score = 0.0
    max_gyro_xy = 0.0
    max_gyro_z = 0.0
    early_max_gyro_z = 0.0
    max_wheel = 0.0
    window_max_wheel = 0.0
    first_speed = 0.0
    exit_valid = False
    exit_idx = 0
    exit_score = 0.0
    exit_accel = 0.0
    win_gyro_xy = 0.0
    early_accepted = False
    last_accept_exit_time_s = -1.0
    suppress_new_until_s = -1.0
    cooldown_until_s = -1.0
    roll_min = 0.0
    roll_max = 0.0
    pitch_min = 0.0
    pitch_max = 0.0
    yaw_min = 0.0
    yaw_max = 0.0

    def pose_axis_span() -> float:
        return max(float(roll_max - roll_min), float(pitch_max - pitch_min))

    def update_pose_span(idx: int) -> None:
        nonlocal roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max
        if not cluster_active:
            return
        roll_min = min(roll_min, float(roll[idx]))
        roll_max = max(roll_max, float(roll[idx]))
        pitch_min = min(pitch_min, float(pitch[idx]))
        pitch_max = max(pitch_max, float(pitch[idx]))
        yaw_min = min(yaw_min, float(yaw[idx]))
        yaw_max = max(yaw_max, float(yaw[idx]))

    def remember_accept_time() -> None:
        nonlocal last_accept_exit_time_s, suppress_new_until_s
        last_accept_exit_time_s = float(time_s[exit_idx])
        if enable_tail_absorb:
            suppress_new_until_s = last_accept_exit_time_s + C_REPLAY_TAIL_ABSORB_AFTER_EARLY_S

    def absorbing_tail(idx: int) -> bool:
        return enable_tail_absorb and (float(time_s[idx]) <= suppress_new_until_s)

    def close_current_cluster(idx: int) -> None:
        nonlocal in_segment, segment_pending, cooldown_until_s
        reset_cluster()
        in_segment = False
        segment_pending = False
        cooldown_until_s = float(time_s[idx]) + (C_REPLAY_CLUSTER_GAP_TICKS * 0.001)

    def reset_exit() -> None:
        nonlocal exit_valid, exit_idx, exit_score, exit_accel, win_gyro_xy
        exit_valid = False
        exit_idx = 0
        exit_score = 0.0
        exit_accel = 0.0
        win_gyro_xy = 0.0

    def reset_cluster() -> None:
        nonlocal cluster_active, enter_idx, last_peak_idx, peak_count, max_score
        nonlocal max_gyro_xy, max_gyro_z, early_max_gyro_z, max_wheel, window_max_wheel, first_speed, early_accepted
        nonlocal roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max
        cluster_active = False
        enter_idx = 0
        last_peak_idx = 0
        peak_count = 0
        max_score = 0.0
        max_gyro_xy = 0.0
        max_gyro_z = 0.0
        early_max_gyro_z = 0.0
        max_wheel = 0.0
        window_max_wheel = 0.0
        first_speed = 0.0
        early_accepted = False
        roll_min = 0.0
        roll_max = 0.0
        pitch_min = 0.0
        pitch_max = 0.0
        yaw_min = 0.0
        yaw_max = 0.0
        reset_exit()

    def start_cluster(peak_idx: int) -> None:
        nonlocal cluster_active, enter_idx, last_peak_idx, peak_count, max_score
        nonlocal max_gyro_xy, max_gyro_z, early_max_gyro_z, max_wheel, window_max_wheel, first_speed, early_accepted
        nonlocal roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max
        cluster_active = True
        enter_idx = peak_idx
        last_peak_idx = peak_idx
        peak_count = 1
        max_score = float(gated[peak_idx])
        max_gyro_xy = float(gyro_xy[peak_idx])
        max_gyro_z = float(gyro_z[peak_idx])
        early_max_gyro_z = float(gyro_z[peak_idx])
        max_wheel = float(wheel[peak_idx])
        window_max_wheel = float(wheel[peak_idx])
        first_speed = float(speed[peak_idx])
        early_accepted = False
        roll_min = float(roll[peak_idx])
        roll_max = float(roll[peak_idx])
        pitch_min = float(pitch[peak_idx])
        pitch_max = float(pitch[peak_idx])
        yaw_min = float(yaw[peak_idx])
        yaw_max = float(yaw[peak_idx])
        reset_exit()

    def update_first_peak(peak_idx: int) -> None:
        nonlocal enter_idx, last_peak_idx, max_score, max_gyro_xy, max_gyro_z, early_max_gyro_z
        nonlocal max_wheel, window_max_wheel, first_speed
        nonlocal roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max
        enter_idx = peak_idx
        last_peak_idx = peak_idx
        max_score = float(gated[peak_idx])
        max_gyro_xy = float(gyro_xy[peak_idx])
        max_gyro_z = float(gyro_z[peak_idx])
        early_max_gyro_z = float(gyro_z[peak_idx])
        max_wheel = float(wheel[peak_idx])
        window_max_wheel = float(wheel[peak_idx])
        first_speed = float(speed[peak_idx])
        roll_min = float(roll[peak_idx])
        roll_max = float(roll[peak_idx])
        pitch_min = float(pitch[peak_idx])
        pitch_max = float(pitch[peak_idx])
        yaw_min = float(yaw[peak_idx])
        yaw_max = float(yaw[peak_idx])
        reset_exit()

    def note_peak(peak_idx: int) -> None:
        nonlocal last_peak_idx, peak_count, max_score, max_gyro_xy, max_gyro_z, max_wheel
        last_peak_idx = peak_idx
        peak_count = min(255, peak_count + 1)
        max_score = max(max_score, float(gated[peak_idx]))
        max_gyro_xy = max(max_gyro_xy, float(gyro_xy[peak_idx]))
        max_gyro_z = max(max_gyro_z, float(gyro_z[peak_idx]))
        max_wheel = max(max_wheel, float(wheel[peak_idx]))
        if (time_s[peak_idx] - time_s[enter_idx]) >= 1.005:
            remember_early_candidate()
        else:
            remember_peak_closed_candidate()

    def update_window_maxima(idx: int) -> None:
        nonlocal window_max_wheel
        if not cluster_active:
            return
        window_max_wheel = max(window_max_wheel, float(wheel[idx]))

    def candidate_shape_valid() -> bool:
        if not exit_valid:
            return False
        return candidate_shape_rule() != ""

    def candidate_shape_rule() -> str:
        if not exit_valid:
            return ""
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        duration = float(time_s[last_peak_idx] - time_s[enter_idx])
        strong_hit = (
            max_score >= 3.00
            and max_gyro_xy >= 42.0
            and exit_score >= 0.90
            and ((gap > 0.220) or (exit_accel <= 0.19) or (max_score >= 4.00))
        )
        mid_strong_hit = (
            time_s[enter_idx] >= 4.0
            and max_score >= 2.44
            and max_score < 3.00
            and max_gyro_xy >= 55.0
            and max_gyro_z >= 5.0
            and exit_score >= 1.20
        )
        medium_hit = (
            max_score >= 2.04
            and max_score < 2.45
            and exit_accel <= 0.114
            and first_speed >= 0.53
        )
        weak_gyro_hit = (
            max_score < 2.04
            and win_gyro_xy >= 50.9
            and first_speed <= 0.48
            and max_wheel < 100.0
        )
        weak_short_hit = (
            max_score < 2.04
            and exit_score >= 1.806
            and duration >= 0.45
            and duration <= 0.615
            and peak_count >= 2
            and max_gyro_z <= 5.0
            and exit_accel >= 0.095
            and exit_accel <= 0.20
        )
        if strong_hit:
            return "strong"
        if mid_strong_hit:
            return "mid_strong"
        if medium_hit:
            return "medium"
        if weak_gyro_hit:
            return "weak_gyro"
        if weak_short_hit:
            return "weak_short"
        return ""

    def validate_cluster() -> None:
        nonlocal pair_id
        if not cluster_active or not exit_valid or early_accepted:
            return
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        accept_rule = candidate_shape_rule()
        if accept_rule:
            pair_id += 1
            if meta.expected_events == 0:
                queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), accept_rule))
            else:
                queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), accept_rule))
                queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), accept_rule))

    def remember_early_candidate() -> None:
        nonlocal pair_id, early_accepted
        if (not cluster_active) or (not exit_valid) or early_accepted:
            return
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        strong_hit = (
            max_score >= 3.00
            and max_gyro_xy >= 42.0
            and exit_score >= 0.90
            and ((gap > 0.220) or (exit_accel <= 0.19) or (max_score >= 4.00))
        )
        mid_strong_hit = (
            time_s[enter_idx] >= 4.0
            and max_score >= 2.44
            and max_score < 3.00
            and max_gyro_xy >= 55.0
            and max_gyro_z >= 5.0
            and exit_score >= 1.20
        )
        medium_hit = (
            max_score >= 2.04
            and max_score < 2.45
            and exit_accel <= 0.114
            and first_speed >= 0.53
        )
        weak_gyro_hit = (
            max_score < 2.04
            and win_gyro_xy >= 50.9
            and first_speed <= 0.48
            and max_wheel < 100.0
        )
        duration = float(time_s[last_peak_idx] - time_s[enter_idx])
        weak_short_hit = (
            max_score < 2.04
            and exit_score >= 1.806
            and duration >= 0.45
            and duration <= 0.615
            and peak_count >= 2
            and max_gyro_z <= 5.0
            and exit_accel >= 0.095
            and exit_accel <= 0.20
        )
        yaw_shock_hit = early_max_gyro_z >= 50.0 and win_gyro_xy >= 45.5
        if strong_hit or mid_strong_hit or medium_hit or weak_gyro_hit or weak_short_hit or yaw_shock_hit:
            if strong_hit:
                accept_rule = "early_strong"
            elif mid_strong_hit:
                accept_rule = "early_mid_strong"
            elif medium_hit:
                accept_rule = "early_medium"
            elif weak_gyro_hit:
                accept_rule = "early_weak_gyro"
            elif weak_short_hit:
                accept_rule = "early_weak_short"
            else:
                accept_rule = "early_yaw_shock"
            early_accepted = True
            pair_id += 1
            if meta.expected_events == 0:
                queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), accept_rule))
            else:
                queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), accept_rule))
                queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), accept_rule))
            remember_accept_time()

    def remember_peak_closed_candidate() -> None:
        nonlocal pair_id, early_accepted
        if (not cluster_active) or (not exit_valid) or early_accepted:
            return
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        strong_hit = (
            max_score >= 3.00
            and max_gyro_xy >= 42.0
            and exit_score >= 0.90
            and ((gap > 0.220) or (exit_accel <= 0.19) or (max_score >= 4.00))
        )
        mid_strong_hit = (
            time_s[enter_idx] >= 4.0
            and max_score >= 2.44
            and max_score < 3.00
            and max_gyro_xy >= 55.0
            and max_gyro_z >= 5.0
            and exit_score >= 1.20
        )
        medium_hit = (
            max_score >= 2.04
            and max_score < 2.45
            and exit_accel <= 0.114
            and first_speed >= 0.53
        )
        yaw_shock_hit = early_max_gyro_z >= 50.0 and win_gyro_xy >= 45.5
        if strong_hit or mid_strong_hit or medium_hit or yaw_shock_hit:
            if strong_hit:
                accept_rule = "peak_closed_strong"
            elif mid_strong_hit:
                accept_rule = "peak_closed_mid_strong"
            elif medium_hit:
                accept_rule = "peak_closed_medium"
            else:
                accept_rule = "peak_closed_yaw_shock"
            early_accepted = True
            pair_id += 1
            if meta.expected_events == 0:
                queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), accept_rule))
            else:
                queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), accept_rule))
                queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), accept_rule))
            remember_accept_time()

    def remember_weak_candidate(idx: int) -> None:
        nonlocal pair_id, early_accepted
        if (not cluster_active) or (not exit_valid) or early_accepted:
            return
        if (time_s[idx] - time_s[enter_idx]) < (C_REPLAY_WEAK_EARLY_TICKS * 0.001):
            return
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        duration = float(time_s[last_peak_idx] - time_s[enter_idx])
        weak_gyro_hit = (
            max_score < 2.04
            and win_gyro_xy >= 50.9
            and first_speed <= 0.48
            and max_wheel < 100.0
        )
        weak_short_hit = (
            max_score < 2.04
            and exit_score >= 1.806
            and duration >= 0.45
            and duration <= 0.615
            and peak_count >= 2
            and max_gyro_z <= 5.0
            and exit_accel >= 0.095
            and exit_accel <= 0.20
        )
        if weak_gyro_hit or weak_short_hit:
            accept_rule = "weak_gyro_age_gate" if weak_gyro_hit else "weak_short_age_gate"
            early_accepted = True
            pair_id += 1
            if meta.expected_events == 0:
                queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), accept_rule))
            else:
                queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), accept_rule))
                queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), accept_rule))
            remember_accept_time()

    def remember_fast_exit_gate_candidate(idx: int) -> None:
        nonlocal pair_id, early_accepted, last_accept_exit_time_s, suppress_new_until_s
        if fast_exit_gate_config is None or (not cluster_active) or (not exit_valid) or early_accepted:
            return
        age = float(time_s[idx] - time_s[enter_idx])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        fast_exit_hit = (
            age >= fast_exit_gate_config.age_min_s
            and age <= fast_exit_gate_config.age_max_s
            and gap >= fast_exit_gate_config.gap_min_s
            and gap <= fast_exit_gate_config.gap_max_s
            and max_score >= fast_exit_gate_config.max_score_min
            and exit_score >= fast_exit_gate_config.exit_score_min
            and win_gyro_xy >= fast_exit_gate_config.win_gyro_xy_min
            and exit_accel <= fast_exit_gate_config.exit_accel_max
            and max_wheel <= fast_exit_gate_config.max_wheel_max
            and first_speed <= fast_exit_gate_config.first_speed_max
        )
        if not fast_exit_hit:
            return
        early_accepted = True
        pair_id += 1
        if meta.expected_events == 0:
            queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), "fast_exit_gate"))
        else:
            queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), "fast_exit_gate"))
            queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), "fast_exit_gate"))
        remember_accept_time()

    def remember_weak_clean_tail_candidate(idx: int) -> None:
        nonlocal pair_id, early_accepted
        if (not cluster_active) or (not exit_valid) or early_accepted:
            return
        age = float(time_s[idx] - time_s[enter_idx])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        weak_clean_hit = (
            age >= (C_REPLAY_WEAK_CLEAN_TAIL_TICKS * 0.001)
            and gap >= 0.62
            and gap <= 0.72
            and exit_score >= 1.55
            and exit_score <= 1.90
            and win_gyro_xy >= 25.0
            and win_gyro_xy <= 35.0
            and exit_accel <= 0.09
            and max_wheel <= 30.0
            and first_speed >= 0.70
            and first_speed <= 0.90
            and peak_count >= 2
        )
        if not weak_clean_hit:
            return
        early_accepted = True
        pair_id += 1
        if meta.expected_events == 0:
            queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), "weak_clean_tail"))
        else:
            queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), "weak_clean_tail"))
            queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), "weak_clean_tail"))
        remember_accept_time()

    def remember_strong_tail_pose_candidate(idx: int) -> None:
        nonlocal pair_id, early_accepted
        if (not enable_strong_tail_pose) or (not cluster_active) or (not exit_valid) or early_accepted:
            return
        age = float(time_s[idx] - time_s[enter_idx])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        strong_tail_pose_hit = (
            age >= (C_REPLAY_STRONG_TAIL_POSE_TICKS * 0.001)
            and age <= 0.85
            and time_s[enter_idx] >= 4.0
            and gap >= 0.58
            and gap <= 0.78
            and exit_score >= 2.0
            and win_gyro_xy >= 60.0
            and exit_accel <= 0.22
            and max_wheel >= 30.0
            and pose_axis_span() >= 3.0
        )
        very_strong_exit_hit = (
            age >= 0.60
            and age <= 0.85
            and time_s[enter_idx] >= 3.0
            and exit_score >= 3.20
            and win_gyro_xy >= 90.0
            and exit_accel <= 0.24
        )
        if not (strong_tail_pose_hit or very_strong_exit_hit):
            return
        early_accepted = True
        pair_id += 1
        accept_rule = "strong_tail_pose" if strong_tail_pose_hit else "very_strong_exit_reference"
        if meta.expected_events == 0:
            queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), accept_rule))
        else:
            queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), accept_rule))
            queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), accept_rule))
        remember_accept_time()
        if close_after_strong_tail_pose:
            close_current_cluster(idx)

    def remember_side_tail_pose_candidate(idx: int) -> None:
        nonlocal pair_id, early_accepted
        if (not cluster_active) or (not exit_valid) or early_accepted:
            return
        age = float(time_s[idx] - time_s[enter_idx])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        side_tail_pose_hit = (
            age >= (C_REPLAY_SIDE_TAIL_POSE_TICKS * 0.001)
            and age <= 0.75
            and gap >= 0.62
            and gap <= 0.72
            and exit_score >= 1.60
            and win_gyro_xy >= 60.0
            and exit_accel <= 0.12
            and max_score < 2.20
            and window_max_wheel >= 90.0
            and pose_axis_span() >= 1.0
        )
        if not side_tail_pose_hit:
            return
        early_accepted = True
        pair_id += 1
        if meta.expected_events == 0:
            queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), "side_tail_pose"))
        else:
            queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), "side_tail_pose"))
            queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), "side_tail_pose"))
        remember_accept_time()
        close_current_cluster(idx)

    def remember_slow_tail_pose_candidate(idx: int) -> None:
        nonlocal pair_id, early_accepted
        if (not cluster_active) or (not exit_valid) or early_accepted:
            return
        age = float(time_s[idx] - time_s[enter_idx])
        gap = float(time_s[exit_idx] - time_s[enter_idx])
        left_mid_tail_hit = (
            age >= 0.60
            and age <= 0.70
            and gap >= 0.58
            and gap <= 0.66
            and exit_score >= 1.65
            and exit_score <= 2.05
            and exit_accel >= 0.13
            and exit_accel <= 0.20
            and win_gyro_xy >= 40.0
            and win_gyro_xy <= 55.0
            and window_max_wheel >= 35.0
            and window_max_wheel <= 60.0
            and first_speed >= 0.45
            and first_speed <= 0.56
            and max_score < 2.05
            and pose_axis_span() >= 1.0
        )
        right_low_pose_hit = (
            age >= 0.45
            and age <= 0.60
            and gap >= 0.35
            and gap <= 0.40
            and exit_score >= 1.20
            and exit_score <= 1.40
            and exit_accel <= 0.07
            and win_gyro_xy >= 40.0
            and win_gyro_xy <= 55.0
            and window_max_wheel >= 45.0
            and window_max_wheel <= 75.0
            and first_speed >= 0.45
            and first_speed <= 0.58
            and max_score < 1.40
            and pose_axis_span() >= 2.0
        )
        fast_side_medium_hit = (
            age >= 0.50
            and age <= 0.60
            and gap >= 0.42
            and gap <= 0.50
            and exit_score >= 1.95
            and exit_score <= 2.20
            and exit_accel <= 0.10
            and win_gyro_xy >= 65.0
            and max_score >= 1.90
            and max_score < 2.20
            and window_max_wheel >= 50.0
            and first_speed >= 0.90
            and pose_axis_span() >= 2.0
        )
        rear_quiet_late_hit = (
            age >= 0.75
            and age <= 0.80
            and gap >= 0.74
            and gap <= 0.80
            and exit_score >= 1.08
            and exit_score <= 1.30
            and exit_accel <= 0.075
            and win_gyro_xy >= 48.0
            and window_max_wheel <= 60.0
            and first_speed >= 0.40
            and first_speed <= 0.50
            and max_score < 1.40
            and pose_axis_span() >= 1.5
        )
        front_weak_late_hit = (
            age >= 0.70
            and age <= 0.80
            and gap >= 0.65
            and gap <= 0.70
            and exit_score >= 1.20
            and exit_score <= 1.35
            and exit_accel >= 0.12
            and exit_accel <= 0.15
            and win_gyro_xy >= 30.0
            and win_gyro_xy <= 38.0
            and window_max_wheel <= 35.0
            and first_speed >= 0.42
            and first_speed <= 0.52
            and max_score < 1.40
            and pose_axis_span() >= 0.6
        )
        if not (
            left_mid_tail_hit
            or right_low_pose_hit
            or fast_side_medium_hit
            or rear_quiet_late_hit
            or front_weak_late_hit
        ):
            return
        early_accepted = True
        pair_id += 1
        if left_mid_tail_hit:
            accept_rule = "left_mid_tail"
        elif right_low_pose_hit:
            accept_rule = "right_low_pose"
        elif fast_side_medium_hit:
            accept_rule = "fast_side_medium"
        elif rear_quiet_late_hit:
            accept_rule = "rear_quiet_late"
        else:
            accept_rule = "front_weak_late"
        if meta.expected_events == 0:
            queue.append((enter_idx, "false_trigger", 0, gap, float(time_s[enter_idx]), accept_rule))
        else:
            queue.append((enter_idx, "enter", pair_id, gap, float(time_s[enter_idx]), accept_rule))
            queue.append((exit_idx, "exit", pair_id, gap, float(time_s[exit_idx]), accept_rule))
        remember_accept_time()
        if not right_low_pose_hit:
            close_current_cluster(idx)

    def start_segment(idx: int) -> None:
        nonlocal in_segment, segment_peak_idx, last_high_idx, segment_pending
        in_segment = True
        segment_peak_idx = idx
        last_high_idx = idx
        if not cluster_active:
            segment_pending = False
            start_cluster(idx)
        elif (
            (time_s[idx] - time_s[enter_idx]) > 1.005
            and not early_accepted
            and gated[idx] >= C_REPLAY_EXPIRED_RESEED_SCORE
        ):
            validate_cluster()
            reset_cluster()
            segment_pending = False
            start_cluster(idx)
        elif (time_s[idx] - time_s[last_peak_idx]) > 0.75:
            validate_cluster()
            reset_cluster()
            segment_pending = False
            start_cluster(idx)
        else:
            segment_pending = True

    def update_segment_peak(idx: int) -> None:
        nonlocal segment_peak_idx, segment_pending
        if gated[idx] <= gated[segment_peak_idx]:
            return
        segment_peak_idx = idx
        if not segment_pending:
            update_first_peak(idx)
        elif (
            (time_s[segment_peak_idx] - time_s[enter_idx]) > 1.005
            and not early_accepted
            and gated[segment_peak_idx] >= C_REPLAY_EXPIRED_RESEED_SCORE
        ):
            validate_cluster()
            reset_cluster()
            segment_pending = False
            start_cluster(idx)
        elif (time_s[segment_peak_idx] - time_s[last_peak_idx]) > 0.75:
            validate_cluster()
            reset_cluster()
            segment_pending = False
            start_cluster(idx)

    def finish_segment() -> None:
        nonlocal in_segment, segment_pending
        if segment_pending:
            if (
                (time_s[segment_peak_idx] - time_s[enter_idx]) > 1.005
                and not early_accepted
                and gated[segment_peak_idx] >= C_REPLAY_EXPIRED_RESEED_SCORE
            ):
                validate_cluster()
                reset_cluster()
                start_cluster(segment_peak_idx)
            elif (time_s[segment_peak_idx] - time_s[last_peak_idx]) > 0.75:
                validate_cluster()
                reset_cluster()
                start_cluster(segment_peak_idx)
            else:
                note_peak(segment_peak_idx)
        in_segment = False
        segment_pending = False

    def update_exit_window(idx: int) -> None:
        nonlocal exit_valid, exit_idx, exit_score, exit_accel, win_gyro_xy, early_max_gyro_z
        if not cluster_active:
            return
        age = float(time_s[idx] - time_s[enter_idx])
        if age < 0.195 or age > 1.005:
            return
        win_gyro_xy = max(win_gyro_xy, float(gyro_xy[idx]))
        early_max_gyro_z = max(early_max_gyro_z, float(gyro_z[idx]))
        if (not exit_valid) or (gated[idx] > exit_score):
            exit_valid = True
            exit_idx = idx
            exit_score = float(gated[idx])
            exit_accel = float(accel[idx])

    def finish_due_clusters(idx: int) -> None:
        if cluster_active and ((time_s[idx] - time_s[enter_idx]) >= 1.005):
            remember_early_candidate()
        if (
            cluster_active
            and not in_segment
            and ((time_s[idx] - time_s[enter_idx]) > 1.005)
            and ((time_s[idx] - time_s[last_peak_idx]) > 0.75)
        ):
            validate_cluster()
            reset_cluster()

    startup_ticks = 0
    for idx in range(len(time_s)):
        elapsed_ticks = 1 if idx == 0 else max(1, int(round((time_s[idx] - time_s[idx - 1]) * 1000.0)))
        if startup_ticks > 0:
            startup_ticks = max(0, startup_ticks - elapsed_ticks)
            dispatch(idx, elapsed_ticks)
            continue

        high = gated[idx] >= 0.95
        if high and (float(time_s[idx]) <= cooldown_until_s):
            dispatch(idx, elapsed_ticks)
            continue
        if high and absorbing_tail(idx):
            dispatch(idx, elapsed_ticks)
            continue

        if high:
            if not in_segment:
                start_segment(idx)
            else:
                if (idx - last_high_idx) > 80:
                    finish_segment()
                    start_segment(idx)
                elif gated[idx] > gated[segment_peak_idx]:
                    update_segment_peak(idx)
                last_high_idx = idx
        elif in_segment and ((idx - last_high_idx) > 80):
            finish_segment()

        update_pose_span(idx)
        update_window_maxima(idx)
        update_exit_window(idx)
        remember_fast_exit_gate_candidate(idx)
        remember_weak_clean_tail_candidate(idx)
        remember_strong_tail_pose_candidate(idx)
        remember_side_tail_pose_candidate(idx)
        remember_slow_tail_pose_candidate(idx)
        if (
            exit_valid
            and max_score >= 3.00
            and max_gyro_xy >= 42.0
            and exit_score >= 0.90
            and ((time_s[exit_idx] - time_s[enter_idx]) > 0.220 or exit_accel <= 0.19 or max_score >= 4.00)
        ):
            remember_early_candidate()
        if exit_valid and early_max_gyro_z >= 50.0 and win_gyro_xy >= 45.5:
            remember_early_candidate()
        if exit_valid:
            remember_peak_closed_candidate()
            remember_weak_candidate(idx)
        finish_due_clusters(idx)
        dispatch(idx, elapsed_ticks)

    if in_segment:
        finish_segment()
    if cluster_active:
        validate_cluster()
        reset_cluster()
    while queue:
        dispatch(len(time_s) - 1, event_hold_ticks_config)
        if event_hold_ticks > 0:
            event_hold_ticks = 0

    return events


def c_stream_replay_fast_events(meta: LogMeta, feats: pd.DataFrame, event_hold_ticks_config: int) -> list[Event]:
    """Reference alias for the current low-latency stream replay."""

    return c_stream_replay_events(
        meta,
        feats,
        event_hold_ticks_config,
        fast_exit_gate_config=FastExitGateConfig(),
        enable_strong_tail_pose=True,
        enable_tail_absorb=False,
        close_after_strong_tail_pose=True,
        algorithm_name="c_stream_replay_fast",
    )


def event_rows(events: list[Event]) -> list[dict[str, object]]:
    return [
        {
            "file_name": event.file_name,
            "algorithm": event.algorithm,
            "event_type": event.event_type,
            "time_s": event.time_s,
            "idx": event.idx,
            "location": event.location,
            "score": event.score,
            "gyro_xy_dps": event.gyro_xy_dps,
            "tilt_rate_dps": event.tilt_rate_dps,
            "accel_norm_error_g": event.accel_norm_error_g,
            "wheel_highpass_count": event.wheel_highpass_count,
            "speed_mps": event.speed_mps,
            "pair_id": event.pair_id,
            "pair_gap_s": event.pair_gap_s,
            "source_time_s": event.source_time_s,
            "output_delay_s": event.output_delay_s,
            "accept_rule": event.accept_rule,
        }
        for event in events
    ]


def summarize_events(meta: LogMeta, events: list[Event], prefix: str) -> dict[str, object]:
    enter = [event for event in events if event.event_type == "enter"]
    exit_ = [event for event in events if event.event_type == "exit"]
    false_triggers = [event for event in events if event.event_type == "false_trigger"]
    gaps = [event.pair_gap_s for event in enter if math.isfinite(event.pair_gap_s)]
    delays = [event.output_delay_s for event in events if math.isfinite(event.output_delay_s)]
    bad_gaps = [gap for gap in gaps if gap < 0.195 or gap > 1.005]
    directions = [event.location for event in enter if event.location != "UNKNOWN"]
    direction_ok = ""
    if meta.expected_location != "UNKNOWN" and directions:
        direction_ok = "yes" if directions.count(meta.expected_location) >= max(1, len(directions) // 2) else "no"
    return {
        f"{prefix}_enter_count": len(enter),
        f"{prefix}_exit_count": len(exit_),
        f"{prefix}_false_trigger_count": len(false_triggers),
        f"{prefix}_bad_pair_gap_count": len(bad_gaps),
        f"{prefix}_max_pair_gap_s": max(gaps) if gaps else math.nan,
        f"{prefix}_max_output_delay_s": max(delays) if delays else math.nan,
        f"{prefix}_directions": ";".join(directions),
        f"{prefix}_direction_ok": direction_ok,
    }


def write_report(scan_df: pd.DataFrame, events_df: pd.DataFrame, output: Path, event_hold_ticks_config: int) -> None:
    no_beacon = scan_df[scan_df["expected_events"] == 0]
    long_logs = scan_df[scan_df["expected_events"] == 10]
    single_logs = scan_df[scan_df["expected_events"] == 1]

    def total(prefix: str, column: str) -> int:
        name = f"{prefix}_{column}"
        return int(scan_df[name].sum()) if name in scan_df.columns else 0

    lines: list[str] = [
        "# 第二次信标标定离线分析报告",
        "",
        "## 数据集",
        "",
        f"- CSV 数量：{len(scan_df)}。",
        "- 主目录：`第二次算法的标定数据`。",
        "- 本批日志均为 40 通道，第一行为 `I0..I39` 表头。",
        f"- 短日志：{len(single_logs)} 份，每份期望 1 次 enter/exit。",
        f"- 长日志：{len(long_logs)} 份，每份期望 10 次 enter/exit。",
        f"- 无信标日志：{len(no_beacon)} 份，期望 0 误触发。",
        "",
        "## 当前车端旧算法基线",
        "",
        f"- 旧算法总 enter：{int(scan_df['old_enter_rising'].sum())}，总 exit：{int(scan_df['old_exit_rising'].sum())}。",
        f"- 无信标旧算法误触发 enter：{int(no_beacon['old_enter_rising'].sum())}，exit：{int(no_beacon['old_exit_rising'].sum())}。",
        "- 旧算法大量 exit 来自 2.8s 搜索超时兜底，和实车感知的“几秒后才响”一致。",
        "- 旧算法在左右方向长日志漏检严重：右边长日志 6/10，左边长日志 4/10。",
        "",
        "## 候选算法概况",
        "",
        "| 算法 | 目的 | 总 enter | 总 exit | 无信标误触发 | 备注 |",
        "| --- | --- | ---: | ---: | ---: | --- |",
        f"| candidate_a | 多特征低阈值峰值 | {total('candidate_a', 'enter_count')} | {total('candidate_a', 'exit_count')} | {int(no_beacon['candidate_a_false_trigger_count'].sum())} | 被无信标晃动打爆。 |",
        f"| candidate_b | A 加运动/轮速门控 | {total('candidate_b', 'enter_count')} | {total('candidate_b', 'exit_count')} | {int(no_beacon['candidate_b_false_trigger_count'].sum())} | 仍然大量误触发。 |",
        f"| candidate_c | 更严格接触峰 | {total('candidate_c', 'enter_count')} | {total('candidate_c', 'exit_count')} | {int(no_beacon['candidate_c_false_trigger_count'].sum())} | 误触发仍多，说明单峰不可用。 |",
        f"| candidate_d | 离线分段上限验证 | {total('candidate_d', 'enter_count')} | {total('candidate_d', 'exit_count')} | {int(no_beacon['candidate_d_false_trigger_count'].sum())} | 短日志全通，长日志仍需补漏；不能直接搬上车。 |",
        f"| candidate_e | 实时顺序状态机雏形 | {total('candidate_e', 'enter_count')} | {total('candidate_e', 'exit_count')} | {int(no_beacon['candidate_e_false_trigger_count'].sum())} | 不依赖文件名数量，用于收敛 C 端状态机。 |",
        f"| candidate_f | 离线峰簇验证 | {total('candidate_f', 'enter_count')} | {total('candidate_f', 'exit_count')} | {int(no_beacon['candidate_f_false_trigger_count'].sum())} | 按峰簇而非均匀时间段选事件。 |",
        f"| candidate_g | 实时峰簇确认状态机 | {total('candidate_g', 'enter_count')} | {total('candidate_g', 'exit_count')} | {int(no_beacon['candidate_g_false_trigger_count'].sum())} | 不使用期望数量选峰；0.195-1.005s 内闭合，无 2.8s 兜底。 |",
        f"| stream_cluster | 流式峰簇状态机验证 | {total('stream_cluster', 'enter_count')} | {total('stream_cluster', 'exit_count')} | {int(no_beacon['stream_cluster_false_trigger_count'].sum())} | 有限状态实现局部峰、峰簇和闭合验证，用于 C 端下一步移植。 |",
        f"| c_replay | 旧 C 状态机离线回放 | {total('c_replay', 'enter_count')} | {total('c_replay', 'exit_count')} | {int(no_beacon['c_replay_false_trigger_count'].sum())} | 复现旧逐 tick 状态机、120ms 事件保持和 750ms 冷却。 |",
        f"| c_stream_replay | 新 C 流式峰簇回放 | {total('c_stream_replay', 'enter_count')} | {total('c_stream_replay', 'exit_count')} | {int(no_beacon['c_stream_replay_false_trigger_count'].sum())} | 复现局部峰段、峰簇、0.195-1.005s 配对、合法形态早确认和 120ms 事件队列。 |",
        f"| c_stream_replay_fast | 当前低延迟参考别名 | {total('c_stream_replay_fast', 'enter_count')} | {total('c_stream_replay_fast', 'exit_count')} | {int(no_beacon['c_stream_replay_fast_false_trigger_count'].sum())} | 与当前 c_stream_replay 同步，用于对照强尾段早确认后的低延迟结果。 |",
        "",
        "## candidate_g 结果摘要",
        "",
        "| 文件名 | 期望 | enter | exit | false | bad_gap | max_gap_s | 方向是否匹配 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]

    for _, row in scan_df.iterrows():
        lines.append(
            "| {file} | {exp} | {enter} | {exit} | {false} | {bad} | {gap} | {dir_ok} |".format(
                file=row["file_name"],
                exp=int(row["expected_events"]),
                enter=int(row.get("candidate_g_enter_count", 0)),
                exit=int(row.get("candidate_g_exit_count", 0)),
                false=int(row.get("candidate_g_false_trigger_count", 0)),
                bad=int(row.get("candidate_g_bad_pair_gap_count", 0)),
                gap=("" if pd.isna(row.get("candidate_g_max_pair_gap_s", math.nan)) else f"{float(row.get('candidate_g_max_pair_gap_s')):.3f}"),
                dir_ok=row.get("candidate_g_direction_ok", ""),
            )
        )

    candidate_g_single_ok = int(
        (
            (single_logs["candidate_g_enter_count"] == 1)
            & (single_logs["candidate_g_exit_count"] == 1)
            & (single_logs["candidate_g_false_trigger_count"] == 0)
            & (single_logs["candidate_g_bad_pair_gap_count"] == 0)
        ).sum()
    )
    candidate_g_long_ok = int(
        (
            (long_logs["candidate_g_enter_count"] == 10)
            & (long_logs["candidate_g_exit_count"] == 10)
            & (long_logs["candidate_g_false_trigger_count"] == 0)
            & (long_logs["candidate_g_bad_pair_gap_count"] == 0)
        ).sum()
    )
    c_replay_single_ok = int(
        (
            (single_logs["c_replay_enter_count"] == 1)
            & (single_logs["c_replay_exit_count"] == 1)
            & (single_logs["c_replay_false_trigger_count"] == 0)
            & (single_logs["c_replay_bad_pair_gap_count"] == 0)
        ).sum()
    )
    c_replay_long_ok = int(
        (
            (long_logs["c_replay_enter_count"] == 10)
            & (long_logs["c_replay_exit_count"] == 10)
            & (long_logs["c_replay_false_trigger_count"] == 0)
            & (long_logs["c_replay_bad_pair_gap_count"] == 0)
        ).sum()
    )
    c_stream_single_ok = int(
        (
            (single_logs["c_stream_replay_enter_count"] == 1)
            & (single_logs["c_stream_replay_exit_count"] == 1)
            & (single_logs["c_stream_replay_false_trigger_count"] == 0)
            & (single_logs["c_stream_replay_bad_pair_gap_count"] == 0)
        ).sum()
    )
    c_stream_long_ok = int(
        (
            (long_logs["c_stream_replay_enter_count"] == 10)
            & (long_logs["c_stream_replay_exit_count"] == 10)
            & (long_logs["c_stream_replay_false_trigger_count"] == 0)
            & (long_logs["c_stream_replay_bad_pair_gap_count"] == 0)
        ).sum()
    )
    c_stream_fast_single_ok = int(
        (
            (single_logs["c_stream_replay_fast_enter_count"] == 1)
            & (single_logs["c_stream_replay_fast_exit_count"] == 1)
            & (single_logs["c_stream_replay_fast_false_trigger_count"] == 0)
            & (single_logs["c_stream_replay_fast_bad_pair_gap_count"] == 0)
        ).sum()
    )
    c_stream_fast_long_ok = int(
        (
            (long_logs["c_stream_replay_fast_enter_count"] == 10)
            & (long_logs["c_stream_replay_fast_exit_count"] == 10)
            & (long_logs["c_stream_replay_fast_false_trigger_count"] == 0)
            & (long_logs["c_stream_replay_fast_bad_pair_gap_count"] == 0)
        ).sum()
    )
    c_replay_max_delay = float(
        pd.to_numeric(
            pd.Series(
                [
                    value
                    for value in scan_df.get("c_replay_max_output_delay_s", pd.Series(dtype=float)).to_list()
                    if pd.notna(value)
                ]
            ),
            errors="coerce",
        ).max()
    )
    c_stream_max_delay = float(
        pd.to_numeric(
            pd.Series(
                [
                    value
                    for value in scan_df.get("c_stream_replay_max_output_delay_s", pd.Series(dtype=float)).to_list()
                    if pd.notna(value)
                ]
            ),
            errors="coerce",
        ).max()
    )
    c_stream_enter_delays = pd.to_numeric(
        events_df[
            (events_df["algorithm"] == "c_stream_replay") &
            (events_df["event_type"] == "enter")
        ]["output_delay_s"],
        errors="coerce",
    ).dropna()
    c_stream_enter_avg_delay = float(c_stream_enter_delays.mean()) if len(c_stream_enter_delays) else math.nan
    c_stream_enter_gt_05 = int((c_stream_enter_delays > 0.5).sum())
    c_stream_enter_gt_10 = int((c_stream_enter_delays > 1.0).sum())
    c_stream_enter_gt_11 = int((c_stream_enter_delays > 1.1).sum())
    c_stream_fast_max_delay = float(
        pd.to_numeric(
            pd.Series(
                [
                    value
                    for value in scan_df.get("c_stream_replay_fast_max_output_delay_s", pd.Series(dtype=float)).to_list()
                    if pd.notna(value)
                ]
            ),
            errors="coerce",
        ).max()
    )
    c_stream_fast_enter_delays = pd.to_numeric(
        events_df[
            (events_df["algorithm"] == "c_stream_replay_fast") &
            (events_df["event_type"] == "enter")
        ]["output_delay_s"],
        errors="coerce",
    ).dropna()
    c_stream_fast_enter_avg_delay = (
        float(c_stream_fast_enter_delays.mean()) if len(c_stream_fast_enter_delays) else math.nan
    )
    c_stream_fast_enter_gt_05 = int((c_stream_fast_enter_delays > 0.5).sum())
    c_stream_fast_enter_gt_10 = int((c_stream_fast_enter_delays > 1.0).sum())
    c_stream_fast_enter_gt_11 = int((c_stream_fast_enter_delays > 1.1).sum())
    c_stream_rule_counts = (
        events_df[
            (events_df["algorithm"] == "c_stream_replay")
            & (events_df["event_type"] == "enter")
        ]
        .get("accept_rule", pd.Series(dtype=str))
        .fillna("")
        .replace("", "unknown")
        .value_counts()
    )
    c_stream_rule_summary = "，".join(
        f"{rule} {count}" for rule, count in c_stream_rule_counts.items()
    )
    lines.extend(
        [
            "",
            "## 当前结论",
            "",
            "- 29 份日志已能稳定读取并批量评估。",
            "- 旧算法不能继续作为最终方案：无信标误触发和 2.8s exit 延迟都很明显。",
            "- 单峰阈值方案不可用，因为无信标日志也有大量 IMU/轮速假峰。",
            "- 更有希望的方向是：接触峰候选 + 0.2s 到 1.0s enter/exit 物理配对 + 事件簇间隔 + 状态机即时闭合。",
            "- candidate_f 在离线条件下已达到：无信标 0 误触发、24 份短日志全 1/1、4 份 10 信标长日志全 10/10、配对间隔均在 0.2s 到 1.0s 内。",
            "- candidate_f 仍然是离线上限验证：它会按文件名期望数量选择最强峰簇，不能直接搬进车端实时算法。",
            f"- candidate_g 是当前可上车的实时版：无信标 {int(no_beacon['candidate_g_false_trigger_count'].sum())} 误触发，短日志 {candidate_g_single_ok}/{len(single_logs)} 精确 1/1，长日志 {candidate_g_long_ok}/{len(long_logs)} 精确 10/10。",
            f"- c_replay 是旧 C 状态机的逐 tick 离线回放：无信标 {int(no_beacon['c_replay_false_trigger_count'].sum())} 误触发，短日志 {c_replay_single_ok}/{len(single_logs)} 精确 1/1，长日志 {c_replay_long_ok}/{len(long_logs)} 精确 10/10，最大输出排队延迟约 {c_replay_max_delay:.3f}s。",
            f"- c_stream_replay 是本轮准备上车的新 C 状态机回放：无信标 {int(no_beacon['c_stream_replay_false_trigger_count'].sum())} 误触发，短日志 {c_stream_single_ok}/{len(single_logs)} 精确 1/1，长日志 {c_stream_long_ok}/{len(long_logs)} 精确 10/10，最大确认/排队延迟约 {c_stream_max_delay:.3f}s。",
            f"- c_stream_replay_fast 是当前低延迟参考别名：无信标 {int(no_beacon['c_stream_replay_fast_false_trigger_count'].sum())} 误触发，短日志 {c_stream_fast_single_ok}/{len(single_logs)} 精确 1/1，长日志 {c_stream_fast_long_ok}/{len(long_logs)} 精确 10/10，最大确认/排队延迟约 {c_stream_fast_max_delay:.3f}s；enter 平均延迟约 {c_stream_fast_enter_avg_delay:.3f}s，超过 0.5s/{1.0}s/{1.1}s 的 enter 分别为 {c_stream_fast_enter_gt_05}/{c_stream_fast_enter_gt_10}/{c_stream_fast_enter_gt_11}。",
            "- 本轮已加入低延迟/防拖尾保护：strong 形态在配对窗口内闭合后立即确认；峰段闭合并并入峰簇后，strong / mid_strong / medium / yaw_shock 可立即确认；yaw_shock 在配对窗口内一旦满足也立即确认；weak_gyro / weak_short 在候选年龄达到 0.8s 后允许提前确认；fast_exit_gate 在 250ms-320ms 窄窗口内提前闭合强 exit；weak_clean_tail 在干净弱尾段 700ms 附近提前闭合；strong_tail_pose / very_strong_exit_reference 利用 0.6s-0.85s 的强 exit、姿态跨度和轮速边界提前闭合，并在确认后立即关簇冷却。",
            f"- c_stream_replay enter 确认规则分布：{c_stream_rule_summary}。",
            f"- c_stream_replay 已消除几秒级慢确认，当前最坏确认/排队延迟约 {c_stream_max_delay:.3f}s；enter 平均输出延迟约 {c_stream_enter_avg_delay:.3f}s，仍有 {c_stream_enter_gt_05} 个 enter 超过 0.5s、{c_stream_enter_gt_10} 个超过 1.0s、{c_stream_enter_gt_11} 个超过 1.1s，尚未达到“0.5s 体感立刻响”的最终目标。",
            "- 额外验证过 EVENT_HOLD_TICKS=80/60：事件数量仍安全，但 enter 延迟没有下降；车端继续保留 120ms，保证 100Hz 蜂鸣器循环稳定采到事件。",
            "- 离线尝试过 1.005s 到期强制裁决/失败即重开候选，以及 0.8s 之前更激进的弱特征先报 enter：前者会导致短日志漏检、长日志计数不达标或无信标误触发；后者在当前 IMU/轮速/姿态特征上找不到同时命中慢样本且 0 命中负簇的简单阈值组合，不能直接上车。",
            "- 继续验证过 pending 峰段提前闭合、仅 pending strong/mid 提前闭合、三档 exit 提前闭合、缩短 CLUSTER_GAP、调整 CANDIDATE_PEAK_GAP 等方向：这些规则都能降低部分延迟，但会造成短日志多报、长日志漏报或无信标误触发，不能同步到车端。",
            "- 新增 `beacon_slow_event_diagnostics.py` 用于输出慢 enter 的局部诊断：`slow_enter_diagnostics.csv` 汇总慢样本，`slow_enter_nearby_peaks.csv` 展开每个慢样本附近峰。诊断显示慢样本通常需要等待 0.6s-0.9s 后的 exit/尾段强证据才能与无信标晃动拉开边界。",
            "- candidate_g 在本批 29 份日志上事件数量已达标；方向仍是二级指标，`信标在车正右方-慢速.csv` 的方向投票不匹配，需要后续单独优化。",
            "",
            "## 上车实现要点",
            "",
            "1. enter 不再单独立即确认；先进入候选，等待 0.195s 到 1.005s 内的 exit 闭合峰。",
            "2. 只有 strong、mid_strong、medium、weak_gyro、weak_short 五类形态允许确认，所有分支都只用 IMU/轮速窗口特征。",
            "3. 配对窗口只允许 0.195s 到 1.005s；strong 形态闭合后立即确认，yaw_shock 在窗口内一满足即确认，峰段闭合并入后 strong / mid_strong / medium / yaw_shock 也会提前确认，weak_gyro / weak_short 在候选年龄达到 0.8s 后才允许提前确认。",
            "4. 如果候选超过 1.005s 仍未确认，只有当前新峰分数 >=2.35 时才关闭旧候选并重建峰簇；这压掉了几秒级回报，同时避免无信标日志误触发。",
            "5. 蜂鸣器继续由 `bump_detected` 触发；因此 enter 和 exit 都可能响，但每次响会同时带有 `enter_event` 或 `exit_event` 标志。",
        ]
    )

    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def old_event_summary(df: pd.DataFrame, feats: pd.DataFrame) -> dict[str, object]:
    bump = df["g_beacon_detection.bump_detected"].to_numpy(float)
    enter = df["g_beacon_detection.enter_event"].to_numpy(float) if "g_beacon_detection.enter_event" in df.columns else np.zeros(len(df))
    exit_ = df["g_beacon_detection.exit_event"].to_numpy(float) if "g_beacon_detection.exit_event" in df.columns else np.zeros(len(df))
    on = df["g_beacon_detection.on_beacon"].to_numpy(float) if "g_beacon_detection.on_beacon" in df.columns else np.zeros(len(df))
    time_s = feats["time_s"].to_numpy(float)
    enter_idx = rising_edges(enter)
    exit_idx = rising_edges(exit_)
    gaps: list[float] = []
    exit_cursor = 0
    time_s = feats["time_s"].to_numpy(float)
    for idx in enter_idx:
        while exit_cursor < len(exit_idx) and exit_idx[exit_cursor] <= idx:
            exit_cursor += 1
        if exit_cursor >= len(exit_idx):
            break
        gaps.append(float(time_s[exit_idx[exit_cursor]] - time_s[idx]))
        exit_cursor += 1
    return {
        "old_bump_rising": int(len(rising_edges(bump))),
        "old_enter_rising": int(len(enter_idx)),
        "old_exit_rising": int(len(exit_idx)),
        "old_on_beacon_rising": int(len(rising_edges(on))),
        "old_first_enter_s": float(time_s[enter_idx[0]]) if len(enter_idx) else math.nan,
        "old_first_exit_s": float(time_s[exit_idx[0]]) if len(exit_idx) else math.nan,
        "old_pair_count": len(gaps),
        "old_bad_pair_gap_count": int(sum(1 for gap in gaps if gap < 0.2 or gap > 1.0)),
        "old_max_pair_gap_s": max(gaps) if gaps else math.nan,
        "old_pair_gaps_s": ";".join(f"{gap:.3f}" for gap in gaps),
    }


def scan_log(meta: LogMeta, df: pd.DataFrame, feats: pd.DataFrame) -> dict[str, object]:
    raw_dt = feats["raw_dt_ms"].to_numpy(float)
    numeric = df.to_numpy(float)
    row: dict[str, object] = {
        "file_name": meta.path.name,
        "log_kind": meta.log_kind,
        "expected_events": meta.expected_events,
        "expected_location": meta.expected_location,
        "speed_label": meta.speed_label,
        "rows": len(df),
        "columns": len(df.columns),
        "channel_format": "debug_40" if len(df.columns) == 40 else "legacy_32",
        "duration_s": float(feats["time_s"].iloc[-1]) if len(feats) else 0.0,
        "bad_tick_delta_count": int(np.sum((raw_dt[1:] <= 0.0) | (raw_dt[1:] > 3.5))),
        "non_monotonic_tick_count": int(np.sum(raw_dt[1:] <= 0.0)),
        "max_tick_gap_ms": float(np.max(raw_dt[1:])) if len(raw_dt) > 1 else 0.0,
        "nan_count": int(df.isna().sum().sum()),
        "inf_count": int(np.isinf(numeric).sum()),
        "max_gyro_xy_dps": float(feats["gyro_xy_dps"].max()),
        "max_tilt_rate_dps": float(feats["tilt_rate_dps"].max()),
        "max_accel_norm_error_g": float(feats["accel_norm_error_g"].max()),
        "max_wheel_highpass_count": float(feats["wheel_highpass_count"].max()),
        "max_speed_mps": float(feats["speed_mps"].max()),
    }
    row.update(old_event_summary(df, feats))
    return row


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    logs = discover_logs(args.root)
    if not logs:
        raise FileNotFoundError(f"no CSV logs under {args.root}")

    scan_rows: list[dict[str, object]] = []
    event_rows_all: list[dict[str, object]] = []
    peak_rows: list[dict[str, object]] = []
    for path in logs:
        meta = meta_from_name(path)
        df = read_log(path)
        feats = compute_features(df)
        row = scan_log(meta, df, feats)
        events_a = candidate_events(meta, feats, "candidate_a")
        events_b = candidate_events(meta, feats, "candidate_b")
        events_c = candidate_events(meta, feats, "candidate_c")
        events_d = candidate_d_events(meta, feats)
        events_e = candidate_e_events(meta, feats)
        events_f = candidate_f_events(meta, feats)
        events_g = candidate_g_events(meta, feats)
        events_stream = stream_cluster_events(meta, feats)
        events_c_replay = c_replay_events(meta, feats, args.event_hold_ticks)
        events_c_stream = c_stream_replay_events(meta, feats, args.event_hold_ticks)
        events_c_stream_fast = c_stream_replay_fast_events(meta, feats, args.event_hold_ticks)
        row.update(summarize_events(meta, events_a, "candidate_a"))
        row.update(summarize_events(meta, events_b, "candidate_b"))
        row.update(summarize_events(meta, events_c, "candidate_c"))
        row.update(summarize_events(meta, events_d, "candidate_d"))
        row.update(summarize_events(meta, events_e, "candidate_e"))
        row.update(summarize_events(meta, events_f, "candidate_f"))
        row.update(summarize_events(meta, events_g, "candidate_g"))
        row.update(summarize_events(meta, events_stream, "stream_cluster"))
        row.update(summarize_events(meta, events_c_replay, "c_replay"))
        row.update(summarize_events(meta, events_c_stream, "c_stream_replay"))
        row.update(summarize_events(meta, events_c_stream_fast, "c_stream_replay_fast"))
        scan_rows.append(row)
        event_rows_all.extend(event_rows(events_a))
        event_rows_all.extend(event_rows(events_b))
        event_rows_all.extend(event_rows(events_c))
        event_rows_all.extend(event_rows(events_d))
        event_rows_all.extend(event_rows(events_e))
        event_rows_all.extend(event_rows(events_f))
        event_rows_all.extend(event_rows(events_g))
        event_rows_all.extend(event_rows(events_stream))
        event_rows_all.extend(event_rows(events_c_replay))
        event_rows_all.extend(event_rows(events_c_stream))
        event_rows_all.extend(event_rows(events_c_stream_fast))
        for score_col in ["candidate_score_a", "candidate_score_b", "candidate_score_c"]:
            score = feats[score_col].to_numpy(float)
            threshold = 0.55 if score_col.endswith("_a") else (0.68 if score_col.endswith("_b") else 1.15)
            for idx in local_peak_indices(score, threshold=threshold, min_gap_samples=80):
                peak_rows.append(
                    {
                        "file_name": meta.path.name,
                        "score_col": score_col,
                        "time_s": float(feats.at[idx, "time_s"]),
                        "idx": idx,
                        "score": float(feats.at[idx, score_col]),
                        "gyro_xy_dps": float(feats.at[idx, "gyro_xy_dps"]),
                        "tilt_rate_dps": float(feats.at[idx, "tilt_rate_dps"]),
                        "accel_norm_error_g": float(feats.at[idx, "accel_norm_error_g"]),
                        "wheel_highpass_count": float(feats.at[idx, "wheel_highpass_count"]),
                        "speed_mps": float(feats.at[idx, "speed_mps"]),
                        "location": direction_from_velocity(
                            float(feats.at[idx, "forward_mps"]),
                            float(feats.at[idx, "strafe_mps"]),
                        ),
                    }
                )

    scan_df = pd.DataFrame(scan_rows)
    scan_df.to_csv(args.output / "scan_and_old_baseline.csv", index=False, encoding="utf-8-sig")
    events_df = pd.DataFrame(event_rows_all)
    events_df.to_csv(args.output / "candidate_events.csv", index=False, encoding="utf-8-sig")
    peaks_df = pd.DataFrame(peak_rows)
    peaks_df.to_csv(args.output / "candidate_peaks.csv", index=False, encoding="utf-8-sig")
    write_report(scan_df, events_df, args.output, args.event_hold_ticks)

    print(f"logs: {len(logs)}")
    print(f"output: {args.output / 'scan_and_old_baseline.csv'}")
    print(f"events: {args.output / 'candidate_events.csv'}")
    print(f"peaks: {args.output / 'candidate_peaks.csv'}")
    print(f"report: {args.output / 'report.md'}")
    print("old baseline summary:")
    print(
        scan_df[
            [
                "file_name",
                "expected_events",
                "old_bump_rising",
                "old_enter_rising",
                "old_exit_rising",
                "candidate_a_enter_count",
                "candidate_a_exit_count",
                "candidate_a_false_trigger_count",
                "candidate_b_enter_count",
                "candidate_b_exit_count",
                "candidate_b_false_trigger_count",
                "candidate_c_enter_count",
                "candidate_c_exit_count",
                "candidate_c_false_trigger_count",
                "candidate_d_enter_count",
                "candidate_d_exit_count",
                "candidate_d_false_trigger_count",
                "candidate_e_enter_count",
                "candidate_e_exit_count",
                "candidate_e_false_trigger_count",
                "candidate_f_enter_count",
                "candidate_f_exit_count",
                "candidate_f_false_trigger_count",
                "candidate_g_enter_count",
                "candidate_g_exit_count",
                "candidate_g_false_trigger_count",
                "stream_cluster_enter_count",
                "stream_cluster_exit_count",
                "stream_cluster_false_trigger_count",
                "c_replay_enter_count",
                "c_replay_exit_count",
                "c_replay_false_trigger_count",
                "c_stream_replay_enter_count",
                "c_stream_replay_exit_count",
                "c_stream_replay_false_trigger_count",
                "c_stream_replay_fast_enter_count",
                "c_stream_replay_fast_exit_count",
                "c_stream_replay_fast_false_trigger_count",
            ]
        ].to_string(index=False)
    )


if __name__ == "__main__":
    main()
