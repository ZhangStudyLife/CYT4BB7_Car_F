#!/usr/bin/env python3
"""Summarize actual accept-rule usage from the stream replay output."""

from __future__ import annotations

from pathlib import Path

import pandas as pd


SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output_second_pass"
EVENTS_PATH = OUTPUT_DIR / "candidate_events.csv"
SCAN_PATH = OUTPUT_DIR / "scan_and_old_baseline.csv"
SLOW_PATH = OUTPUT_DIR / "slow_enter_diagnostics.csv"


def load_inputs() -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    events = pd.read_csv(EVENTS_PATH, encoding="utf-8-sig")
    scan = pd.read_csv(SCAN_PATH, encoding="utf-8-sig")
    slow = pd.read_csv(SLOW_PATH, encoding="utf-8-sig") if SLOW_PATH.exists() else pd.DataFrame()
    if "accept_rule" not in events.columns:
        events["accept_rule"] = ""
    events["accept_rule"] = events["accept_rule"].fillna("").replace("", "unknown")
    return events, scan, slow


def summarize_rules(events: pd.DataFrame, scan: pd.DataFrame, slow: pd.DataFrame) -> pd.DataFrame:
    c_stream = events[
        (events["algorithm"] == "c_stream_replay")
        & (events["event_type"].isin(["enter", "false_trigger"]))
    ].copy()
    if c_stream.empty:
        return pd.DataFrame()

    expected_by_file = scan.set_index("file_name")["expected_events"].to_dict()
    c_stream["expected_events"] = c_stream["file_name"].map(expected_by_file).fillna(-1).astype(int)
    c_stream["is_false_file"] = c_stream["expected_events"].eq(0)
    c_stream["is_enter"] = c_stream["event_type"].eq("enter")

    slow_keys: set[tuple[str, int]] = set()
    if not slow.empty:
        for row in slow.itertuples(index=False):
            slow_keys.add((str(row.file_name), int(row.pair_id)))

    rows: list[dict[str, object]] = []
    for rule, group in c_stream.groupby("accept_rule", dropna=False):
        pair_keys = set(zip(group["file_name"].astype(str), group["pair_id"].astype(int)))
        rows.append(
            {
                "rule": rule,
                "enter_or_false_rows": len(group),
                "enter_rows": int(group["is_enter"].sum()),
                "false_trigger_rows": int((~group["is_enter"]).sum()),
                "no_beacon_rows": int(group["is_false_file"].sum()),
                "unique_files": group["file_name"].nunique(),
                "slow_gt_0_8s_pairs": len(pair_keys & slow_keys),
                "max_enter_output_delay_s": float(
                    pd.to_numeric(group["output_delay_s"], errors="coerce").max()
                ),
                "avg_enter_output_delay_s": float(
                    pd.to_numeric(group["output_delay_s"], errors="coerce").mean()
                ),
            }
        )

    return pd.DataFrame(rows).sort_values(
        ["false_trigger_rows", "enter_rows", "rule"],
        ascending=[True, False, True],
    )


def main() -> None:
    events, scan, slow = load_inputs()
    summary = summarize_rules(events, scan, slow)
    hits = events[
        (events["algorithm"] == "c_stream_replay")
        & (events["event_type"].isin(["enter", "false_trigger"]))
    ].copy()
    hits = hits.sort_values(["file_name", "pair_id", "event_type"])

    summary.to_csv(OUTPUT_DIR / "rule_search_summary.csv", index=False, encoding="utf-8-sig")
    hits.to_csv(OUTPUT_DIR / "rule_search_hits.csv", index=False, encoding="utf-8-sig")
    print(summary.to_string(index=False))
    print(OUTPUT_DIR / "rule_search_summary.csv")
    print(OUTPUT_DIR / "rule_search_hits.csv")


if __name__ == "__main__":
    main()
