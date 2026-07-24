#!/usr/bin/env python3
"""Generate focused diagnostics for delayed c_stream_replay enter events."""

from __future__ import annotations

from pathlib import Path
import importlib.util
import math
import sys

import numpy as np
import pandas as pd


SCRIPT_DIR = Path(__file__).resolve().parent
SECOND_PASS_PATH = SCRIPT_DIR / "beacon_second_pass.py"
OUTPUT_DIR = SCRIPT_DIR / "output_second_pass"


def load_second_pass_module():
    spec = importlib.util.spec_from_file_location("beacon_second_pass", SECOND_PASS_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {SECOND_PASS_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def window_stats(feats: pd.DataFrame, center_idx: int, before_s: float, after_s: float) -> dict[str, float]:
    time_s = feats["time_s"].to_numpy(float)
    center_t = float(time_s[center_idx])
    mask = (time_s >= center_t - before_s) & (time_s <= center_t + after_s)
    window = feats.loc[mask]
    if window.empty:
        return {
            "win_max_score": math.nan,
            "win_max_gyro_xy": math.nan,
            "win_max_gyro_z": math.nan,
            "win_max_accel": math.nan,
            "win_max_wheel": math.nan,
            "win_max_speed": math.nan,
        }
    return {
        "win_max_score": float(window["candidate_score_c"].max()),
        "win_max_gyro_xy": float(window["gyro_xy_dps"].max()),
        "win_max_gyro_z": float(window["gyro_z_abs_dps"].max()),
        "win_max_accel": float(window["accel_norm_error_g"].max()),
        "win_max_wheel": float(window["wheel_highpass_count"].max()),
        "win_max_speed": float(window["speed_mps"].max()),
    }


def nearby_peak_rows(meta, feats: pd.DataFrame, event, bd) -> list[dict[str, object]]:
    time_s = feats["time_s"].to_numpy(float)
    score = feats["candidate_score_c"].to_numpy(float)
    peaks = bd.local_peak_indices(score, threshold=0.95, min_gap_samples=80)
    rows: list[dict[str, object]] = []
    for peak_idx in peaks:
        dt = float(time_s[peak_idx] - event.source_time_s)
        if -0.35 <= dt <= 1.60:
            rows.append(
                {
                    "file_name": meta.path.name,
                    "pair_id": event.pair_id,
                    "event_enter_source_s": event.source_time_s,
                    "event_enter_output_s": event.time_s,
                    "event_enter_delay_s": event.output_delay_s,
                    "peak_dt_s": dt,
                    "peak_time_s": float(time_s[peak_idx]),
                    "peak_score": float(score[peak_idx]),
                    "peak_gyro_xy": float(feats.at[peak_idx, "gyro_xy_dps"]),
                    "peak_gyro_z": float(feats.at[peak_idx, "gyro_z_abs_dps"]),
                    "peak_accel": float(feats.at[peak_idx, "accel_norm_error_g"]),
                    "peak_wheel": float(feats.at[peak_idx, "wheel_highpass_count"]),
                    "peak_speed": float(feats.at[peak_idx, "speed_mps"]),
                }
            )
    return rows


def main() -> None:
    bd = load_second_pass_module()
    root = bd.find_default_log_root()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    events_df = pd.read_csv(OUTPUT_DIR / "candidate_events.csv")
    slow = events_df[
        (events_df["algorithm"] == "c_stream_replay")
        & (events_df["event_type"] == "enter")
        & (events_df["output_delay_s"] > 0.8)
    ].copy()
    slow = slow.sort_values(["output_delay_s", "file_name"], ascending=[False, True])

    summary_rows: list[dict[str, object]] = []
    peak_rows: list[dict[str, object]] = []
    logs_by_name = {path.name: path for path in bd.discover_logs(root)}

    for _, row in slow.iterrows():
        path = logs_by_name[str(row["file_name"])]
        meta = bd.meta_from_name(path)
        df = bd.read_log(path)
        feats = bd.compute_features(df)
        events = bd.c_stream_replay_events(meta, feats, bd.C_REPLAY_EVENT_HOLD_TICKS)
        enter_events = [
            event
            for event in events
            if event.event_type == "enter"
            and event.pair_id == int(row["pair_id"])
            and abs(event.source_time_s - float(row["source_time_s"])) < 0.002
        ]
        if not enter_events:
            continue
        event = enter_events[0]
        exit_events = [
            item
            for item in events
            if item.event_type == "exit" and item.pair_id == event.pair_id
        ]
        exit_event = exit_events[0] if exit_events else None
        enter_idx = int(event.idx)
        exit_idx = int(exit_event.idx) if exit_event is not None else enter_idx
        enter_window = window_stats(feats, enter_idx, before_s=0.10, after_s=1.10)
        exit_window = window_stats(feats, exit_idx, before_s=0.10, after_s=0.35)
        summary = {
            "file_name": event.file_name,
            "pair_id": event.pair_id,
            "enter_source_s": event.source_time_s,
            "enter_output_s": event.time_s,
            "enter_delay_s": event.output_delay_s,
            "pair_gap_s": event.pair_gap_s,
            "enter_score": event.score,
            "enter_gyro_xy": event.gyro_xy_dps,
            "enter_accel": event.accel_norm_error_g,
            "enter_wheel": event.wheel_highpass_count,
            "enter_speed": event.speed_mps,
            "exit_source_s": exit_event.source_time_s if exit_event is not None else math.nan,
            "exit_output_s": exit_event.time_s if exit_event is not None else math.nan,
            "exit_delay_s": exit_event.output_delay_s if exit_event is not None else math.nan,
            "exit_score": exit_event.score if exit_event is not None else math.nan,
            "exit_gyro_xy": exit_event.gyro_xy_dps if exit_event is not None else math.nan,
            "exit_accel": exit_event.accel_norm_error_g if exit_event is not None else math.nan,
            "exit_wheel": exit_event.wheel_highpass_count if exit_event is not None else math.nan,
            "exit_speed": exit_event.speed_mps if exit_event is not None else math.nan,
        }
        summary.update({f"enter_{key}": value for key, value in enter_window.items()})
        summary.update({f"exit_{key}": value for key, value in exit_window.items()})
        summary_rows.append(summary)
        peak_rows.extend(nearby_peak_rows(meta, feats, event, bd))

    pd.DataFrame(summary_rows).to_csv(OUTPUT_DIR / "slow_enter_diagnostics.csv", index=False, encoding="utf-8-sig")
    pd.DataFrame(peak_rows).to_csv(OUTPUT_DIR / "slow_enter_nearby_peaks.csv", index=False, encoding="utf-8-sig")
    print(f"slow enter diagnostics: {len(summary_rows)}")
    print(f"nearby peaks: {len(peak_rows)}")
    print(OUTPUT_DIR / "slow_enter_diagnostics.csv")
    print(OUTPUT_DIR / "slow_enter_nearby_peaks.csv")


if __name__ == "__main__":
    main()
