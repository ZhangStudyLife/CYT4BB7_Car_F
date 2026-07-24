#!/usr/bin/env python3
"""Offline evaluator for Position beacon-fix logs.

The script intentionally keeps the raw CSV logs read-only. It scans every CSV
under ../惯导结合信标检测矫正, replays the current C output, runs a conservative
offline candidate matcher, and writes transparent verification artifacts.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
POSITION_DIR = SCRIPT_DIR.parent
LOG_DIR_NAME = "惯导结合信标检测矫正"
OUTPUT_DIR = SCRIPT_DIR / "output"

NO_BEACON_INDEX = 65535
BEACONS: tuple[tuple[float, float], ...] = (
    (4.0, 4.5),
    (3.0, 4.5),
    (1.0, 3.5),
    (2.0, 2.0),
    (2.0, 0.5),
    (4.0, 2.0),
    (5.0, 1.0),
)

# First-pass offline parameters. These are deliberately visible in the report
# so failures are reviewable instead of hidden behind magic constants.
EVENT_STRICT_RADIUS_M = 0.35
EVENT_LOOSE_RADIUS_M = 0.90
EVENT_AMBIGUITY_MARGIN_M = 0.12
EVENT_MIN_SCORE = 0.50
EVENT_MIN_CONFIDENCE = 1.0
TRACK_DIAG_RADIUS_M = 0.12
TRACK_DIAG_MIN_SEPARATION_TICK = 650
TRACK_DIAG_MAX_ITEMS = 32
FINAL_ERROR_LIMIT_M = 0.50
ODOMETER_REPLAY_SCALE_X = 1.00
ODOMETER_REPLAY_SCALE_Y = 1.06
TRANSITION_RADIUS_M = 1.10
TRANSITION_WEAK_RADIUS_M = 0.40
TRACK_ACCEPT_RADIUS_M = 0.14
TRACK_ACCEPT_MIN_SCORE = 0.55
TRACK_ACCEPT_MIN_ABS_Z = 0.35
TRACK_ENTER_LOOKAHEAD_TICK = 900
INITIAL_TRACK_BEACON7_RADIUS_M = 0.07
INITIAL_TRACK_BEACON7_MIN_SCORE = 0.60
FIX_ALPHA_INITIAL = 0.75
FIX_ALPHA_TRANSITION = 0.50
FIX_ALPHA_TRACK = 0.00
FIX_ALPHA_WEAK = 0.00
FIX_ALPHA_STRICT = 0.75
FIX_ALPHA_REPEAT_ENTER = 0.50

TOPOLOGY: dict[int, tuple[int, ...]] = {
    2: (3, 6),
    3: (2,),
    4: (3, 4),
    5: (4, 5),
    6: (3, 4, 5, 7),
    7: (6,),
}


@dataclass
class LogRow:
    values: dict[str, float]

    def __getitem__(self, key: str) -> float:
        return self.values[key]


@dataclass
class CandidateEvent:
    source: str
    tick: int
    enter_count: int
    position: tuple[float, float]
    velocity: tuple[float, float]
    location: int
    confidence: float
    score: float
    impact_robust_z: float
    beacon_id: int | None
    distance_m: float | None
    second_distance_m: float | None
    accepted: bool
    reason: str
    applied: bool
    counts_in_sequence: bool = True


@dataclass
class ReplayPoint:
    row: LogRow
    position: tuple[float, float]


@dataclass
class LogResult:
    file_name: str
    row_count: int
    expected_sequence: list[int]
    truth_position: tuple[float, float]
    baseline_sequence: list[int]
    baseline_enter_events: int
    baseline_fix_applied: int
    baseline_final_position: tuple[float, float]
    baseline_final_error_m: float
    offline_sequence: list[int]
    offline_final_position: tuple[float, float]
    offline_final_error_m: float
    events: list[CandidateEvent]
    rejections: list[CandidateEvent]
    needs_manual_review: bool
    status: str
    notes: list[str]


@dataclass
class ScaleSweepResult:
    scale_x: float
    scale_y: float
    pass_count: int
    sequence_pass_count: int
    error_pass_count: int
    total_error_m: float
    failed_files: list[str]


def find_log_dir() -> Path:
    direct = POSITION_DIR / LOG_DIR_NAME
    if direct.is_dir():
        return direct

    csv_dirs = [path for path in POSITION_DIR.iterdir() if path.is_dir() and list(path.glob("*.csv"))]
    if not csv_dirs:
        raise FileNotFoundError(f"No CSV log directory found under {POSITION_DIR}")
    return max(csv_dirs, key=lambda path: len(list(path.glob("*.csv"))))


def parse_truth_position(file_name: str) -> tuple[float, float]:
    pairs = re.findall(r"(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)", file_name)
    if not pairs:
        raise ValueError(f"Cannot parse final truth position from file name: {file_name}")
    x_value, y_value = pairs[-1]
    return float(x_value), float(y_value)


def parse_expected_sequence(file_name: str) -> list[int]:
    prefix = file_name.split("-", 1)[0]
    if "没有碰到信标灯" in file_name:
        return []
    if "7655443267" in prefix:
        return [int(ch) for ch in "7655443267"]
    return [int(ch) for ch in re.findall(r"[1-7]", prefix)]


def read_log(path: Path) -> list[LogRow]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = [f"I{i}" for i in range(28)]
        missing = [name for name in required if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path.name} missing columns: {', '.join(missing)}")
        return [LogRow({key: float(value) for key, value in row.items() if key in required}) for row in reader]


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def blend_position(position: tuple[float, float], target: tuple[float, float], alpha: float) -> tuple[float, float]:
    return (
        position[0] + ((target[0] - position[0]) * alpha),
        position[1] + ((target[1] - position[1]) * alpha),
    )


def fix_alpha_for_event(event: CandidateEvent) -> float:
    if not event.counts_in_sequence and event.reason == "repeat enter recalibrates previous beacon":
        return FIX_ALPHA_REPEAT_ENTER
    if event.source == "track":
        return FIX_ALPHA_TRACK
    if "strict" in event.reason:
        return FIX_ALPHA_STRICT
    if "initial" in event.reason:
        return FIX_ALPHA_INITIAL
    if "weak" in event.reason:
        return FIX_ALPHA_WEAK
    if "transition" in event.reason:
        return FIX_ALPHA_TRANSITION
    return 1.0


def make_repeat_enter_fix(row: LogRow, replay_position: tuple[float, float], previous_beacon: int | None) -> CandidateEvent | None:
    if previous_beacon != 4:
        return None
    if int(round(row["I8"])) != 4:
        return None
    if row["I10"] < 1.0:
        return None
    return make_event(
        row,
        "enter",
        True,
        "repeat enter recalibrates previous beacon",
        previous_beacon,
        replay_position,
        counts_in_sequence=False,
    )


def beacon_distances(position: tuple[float, float]) -> list[tuple[int, float]]:
    items = [(index + 1, distance(position, beacon)) for index, beacon in enumerate(BEACONS)]
    return sorted(items, key=lambda item: item[1])


def baseline_sequence(rows: list[LogRow]) -> list[int]:
    sequence: list[int] = []
    for row in rows:
        if int(round(row["I15"])) == 0:
            continue
        index = int(round(row["I18"]))
        if 0 <= index < len(BEACONS):
            sequence.append(index + 1)
    return sequence


def extract_enter_events(rows: list[LogRow]) -> list[LogRow]:
    if not rows:
        return []

    events: list[LogRow] = []
    last_count = int(round(rows[0]["I6"]))
    for row in rows[1:]:
        count = int(round(row["I6"]))
        if count != last_count:
            events.append(row)
            last_count = count
    return events


def make_event(
    row: LogRow,
    source: str,
    accepted: bool,
    reason: str,
    beacon_id: int | None = None,
    position_override: tuple[float, float] | None = None,
    counts_in_sequence: bool = True,
) -> CandidateEvent:
    position = position_override if position_override is not None else (row["I1"], row["I2"])
    nearest = beacon_distances(position)
    selected = beacon_id if beacon_id is not None else nearest[0][0]
    selected_distance = dict(nearest).get(selected)
    second_distance = next((dist for bid, dist in nearest if bid != selected), None)
    return CandidateEvent(
        source=source,
        tick=int(round(row["I0"])),
        enter_count=int(round(row["I6"])),
        position=position,
        velocity=(row["I3"], row["I4"]),
        location=int(round(row["I8"])),
        confidence=row["I9"],
        score=row["I10"],
        impact_robust_z=row["I14"],
        beacon_id=selected if accepted else beacon_id,
        distance_m=selected_distance,
        second_distance_m=second_distance,
        accepted=accepted,
        reason=reason,
        applied=accepted,
        counts_in_sequence=counts_in_sequence,
    )


def initial_loose_allowed(beacon_id: int, row: LogRow, distance_m: float) -> bool:
    location = int(round(row["I8"]))
    score = row["I10"]

    if beacon_id == 2:
        return location == 4 and distance_m <= 0.70 and score >= 1.20
    if beacon_id == 3:
        return location == 2 and distance_m <= 0.90 and score >= 1.00
    if beacon_id == 4:
        return location == 2 and distance_m <= 0.90 and score >= 1.00
    if beacon_id == 6:
        return location == 3 and distance_m <= 0.80 and score >= 1.00
    if beacon_id == 7:
        return distance_m <= 0.50 and score >= 1.00
    return False


def candidate_rank(
    beacon_id: int,
    distance_m: float,
    replay_position: tuple[float, float],
    previous_beacon: int | None,
) -> float:
    rank = distance_m

    if previous_beacon == 6:
        if beacon_id == 4 and replay_position[1] < 2.80:
            rank -= 0.20
        if beacon_id == 3 and replay_position[1] >= 2.80:
            rank -= 0.20
        if beacon_id == 7 and replay_position[0] > 4.50:
            rank -= 0.25
    if previous_beacon == 4 and beacon_id == 3:
        rank -= 0.15
    if previous_beacon == 4 and beacon_id == 4:
        rank += 0.35
    if previous_beacon == 5 and beacon_id == 5:
        rank += 0.20
    if previous_beacon == 3 and beacon_id == 2:
        rank -= 0.20
    if previous_beacon == 2 and beacon_id == 6:
        rank -= 0.20

    return rank


def accept_enter_event(
    row: LogRow,
    replay_position: tuple[float, float],
    previous_beacon: int | None,
) -> CandidateEvent:
    position = replay_position
    nearest = beacon_distances(position)
    confidence = row["I9"]
    score = row["I10"]

    if confidence < EVENT_MIN_CONFIDENCE:
        return make_event(row, "enter", False, "confidence below threshold", nearest[0][0], position)

    allowed = set(range(1, len(BEACONS) + 1)) if previous_beacon is None else set(TOPOLOGY.get(previous_beacon, ()))
    if previous_beacon is not None:
        allowed.add(previous_beacon)

    scored: list[tuple[float, int, float, str]] = []
    for beacon_id, distance_m in nearest:
        if beacon_id not in allowed:
            continue

        reason = ""
        if distance_m <= EVENT_STRICT_RADIUS_M and score >= EVENT_MIN_SCORE:
            reason = "strict distance gate"
        elif previous_beacon is not None and distance_m <= TRANSITION_WEAK_RADIUS_M:
            reason = "weak detector score accepted by transition gate"
        elif previous_beacon is not None and distance_m <= TRANSITION_RADIUS_M and score >= EVENT_MIN_SCORE:
            reason = "transition distance gate"
        elif previous_beacon is None and initial_loose_allowed(beacon_id, row, distance_m):
            reason = "initial loose gate"

        if reason:
            scored.append((candidate_rank(beacon_id, distance_m, position, previous_beacon), beacon_id, distance_m, reason))

    if scored:
        scored.sort(key=lambda item: item[0])
        _, beacon_id, _, reason = scored[0]
        return make_event(row, "enter", True, reason, beacon_id, position)

    best_id, best_distance = nearest[0]
    _, second_distance = nearest[1]
    if previous_beacon is not None and not (set(TOPOLOGY.get(previous_beacon, ())) & {item[0] for item in nearest[:3]}):
        return make_event(row, "enter", False, "nearest candidates violate topology", best_id, position)
    if score < EVENT_MIN_SCORE:
        return make_event(row, "enter", False, "score below threshold", best_id, position)
    if best_distance > max(EVENT_LOOSE_RADIUS_M, TRANSITION_RADIUS_M):
        return make_event(row, "enter", False, "outside loose distance gate", best_id, position)
    if (second_distance - best_distance) < EVENT_AMBIGUITY_MARGIN_M:
        return make_event(row, "enter", False, "ambiguous nearest beacons", best_id, position)
    return make_event(row, "enter", False, "candidate failed initial/history gate", best_id, position)


def track_diagnostics(trace: list[ReplayPoint], accepted: list[CandidateEvent]) -> list[CandidateEvent]:
    diagnostics: list[CandidateEvent] = []
    accepted_keys = {(event.beacon_id, event.tick) for event in accepted if event.beacon_id is not None}

    for beacon_id, beacon in enumerate(BEACONS, start=1):
        near_rows = [
            point for point in trace
            if distance(point.position, beacon) <= TRACK_DIAG_RADIUS_M
        ]
        if not near_rows:
            continue

        groups: list[list[ReplayPoint]] = []
        for point in near_rows:
            if not groups or (point.row["I0"] - groups[-1][-1].row["I0"]) > TRACK_DIAG_MIN_SEPARATION_TICK:
                groups.append([point])
            else:
                groups[-1].append(point)

        for group in groups:
            best = min(group, key=lambda item: distance(item.position, beacon))
            tick = int(round(best.row["I0"]))
            if any(key[0] == beacon_id and abs(key[1] - tick) <= TRACK_DIAG_MIN_SEPARATION_TICK for key in accepted_keys):
                continue
            event = make_event(
                best.row,
                "track",
                False,
                "replayed trajectory passed beacon without accepted enter event",
                beacon_id,
                best.position,
            )
            diagnostics.append(event)

    diagnostics.sort(key=lambda item: item.tick)
    return diagnostics[:TRACK_DIAG_MAX_ITEMS]


def can_accept_track_candidate(
    beacon_id: int,
    point: ReplayPoint,
    previous_beacon: int | None,
    accepted: list[CandidateEvent],
) -> tuple[bool, str]:
    if previous_beacon is None:
        return False, "track candidate needs previous confirmed beacon"
    if beacon_id not in TOPOLOGY.get(previous_beacon, ()):
        return False, "track candidate violates topology"
    if accepted and abs(point.row["I0"] - accepted[-1].tick) <= TRACK_DIAG_MIN_SEPARATION_TICK:
        return False, "track candidate too close to previous fix"
    if accepted and beacon_id == accepted[-1].beacon_id and abs(point.row["I0"] - accepted[-1].tick) <= 1800:
        return False, "track candidate duplicates previous beacon"
    if any(event.source == "track" and event.beacon_id == beacon_id for event in accepted):
        return False, "track candidate duplicates prior track-confirmed beacon"
    if accepted and beacon_id == accepted[-1].beacon_id:
        if beacon_id not in (4, 5):
            return False, "track candidate duplicates previous beacon"
    if point.row["I10"] < TRACK_ACCEPT_MIN_SCORE and abs(point.row["I14"]) < TRACK_ACCEPT_MIN_ABS_Z:
        return False, "track candidate signal too weak"
    return True, "track proximity accepted by topology and signal gate"


def try_track_candidate(
    row: LogRow,
    replay_position: tuple[float, float],
    previous_beacon: int | None,
    accepted: list[CandidateEvent],
    next_enter_tick: int | None,
) -> CandidateEvent | None:
    if previous_beacon is None:
        beacon_id = 7
        distance_m = distance(replay_position, BEACONS[beacon_id - 1])
        if (
            distance_m <= INITIAL_TRACK_BEACON7_RADIUS_M
            and row["I10"] >= INITIAL_TRACK_BEACON7_MIN_SCORE
        ):
            return make_event(
                row,
                "track",
                True,
                "initial beacon 7 track proximity accepted by tight gate",
                beacon_id,
                replay_position,
            )
        return None

    allowed = TOPOLOGY.get(previous_beacon, ())
    candidates = [
        (beacon_id, distance(replay_position, BEACONS[beacon_id - 1]))
        for beacon_id in allowed
    ]
    candidates = [(beacon_id, dist) for beacon_id, dist in candidates if dist <= TRACK_ACCEPT_RADIUS_M]
    if not candidates:
        return None

    candidates.sort(key=lambda item: candidate_rank(item[0], item[1], replay_position, previous_beacon))
    beacon_id, _ = candidates[0]
    if next_enter_tick is not None and (next_enter_tick - row["I0"]) <= TRACK_ENTER_LOOKAHEAD_TICK:
        return make_event(
            row,
            "track",
            False,
            "track candidate deferred to nearby enter event",
            beacon_id,
            replay_position,
        )
    point = ReplayPoint(row, replay_position)
    can_accept, reason = can_accept_track_candidate(beacon_id, point, previous_beacon, accepted)
    return make_event(row, "track", can_accept, reason, beacon_id, replay_position)


def simulate_offline(rows: list[LogRow], expected_sequence: list[int]) -> tuple[list[int], tuple[float, float], list[CandidateEvent], list[CandidateEvent], list[str]]:
    notes: list[str] = []
    accepted: list[CandidateEvent] = []
    rejected: list[CandidateEvent] = []
    trace: list[ReplayPoint] = []

    replay_position = (6.0, 1.0)
    last_tick = rows[0]["I0"]
    last_enter_count = int(round(rows[0]["I6"]))
    previous_beacon: int | None = None
    trace.append(ReplayPoint(rows[0], replay_position))
    enter_ticks = [int(round(row["I0"])) for row in extract_enter_events(rows)]
    enter_tick_pos = 0

    for row in rows[1:]:
        tick = row["I0"]
        while enter_tick_pos < len(enter_ticks) and enter_ticks[enter_tick_pos] <= int(round(tick)):
            enter_tick_pos += 1
        next_enter_tick = enter_ticks[enter_tick_pos] if enter_tick_pos < len(enter_ticks) else None
        dt_s = max(0.0, (tick - last_tick) * 0.001)
        replay_position = (
            replay_position[0] + (row["I3"] * ODOMETER_REPLAY_SCALE_X * dt_s),
            replay_position[1] + (row["I4"] * ODOMETER_REPLAY_SCALE_Y * dt_s),
        )
        last_tick = tick

        enter_count = int(round(row["I6"]))
        if enter_count != last_enter_count:
            event = accept_enter_event(row, replay_position, previous_beacon)
            if event.accepted:
                duplicate_track_fix = (
                    accepted
                    and accepted[-1].source == "track"
                    and accepted[-1].beacon_id == event.beacon_id
                )
                duplicate_disallowed = (
                    accepted
                    and accepted[-1].beacon_id == event.beacon_id
                    and event.beacon_id not in (4, 5)
                )
                if duplicate_track_fix or duplicate_disallowed:
                    event.accepted = False
                    event.applied = False
                    event.reason = "enter duplicates previous confirmed beacon"
                    rejected.append(event)
                else:
                    accepted.append(event)
                    beacon = BEACONS[event.beacon_id - 1] if event.beacon_id is not None else replay_position
                    replay_position = blend_position(replay_position, beacon, fix_alpha_for_event(event))
                    previous_beacon = event.beacon_id
            else:
                repeat_fix = make_repeat_enter_fix(row, replay_position, previous_beacon)
                if repeat_fix is not None:
                    accepted.append(repeat_fix)
                    beacon = BEACONS[repeat_fix.beacon_id - 1]
                    replay_position = blend_position(replay_position, beacon, fix_alpha_for_event(repeat_fix))
                else:
                    rejected.append(event)
            last_enter_count = enter_count

        track_event = try_track_candidate(row, replay_position, previous_beacon, accepted, next_enter_tick)
        if track_event is not None:
            if track_event.accepted:
                accepted.append(track_event)
                beacon = BEACONS[track_event.beacon_id - 1] if track_event.beacon_id is not None else replay_position
                replay_position = blend_position(replay_position, beacon, fix_alpha_for_event(track_event))
                previous_beacon = track_event.beacon_id
            else:
                rejected.append(track_event)

        trace.append(ReplayPoint(row, replay_position))

    diagnostics = track_diagnostics(trace, accepted)
    rejected.extend(diagnostics)

    sequence = [event.beacon_id for event in accepted if event.beacon_id is not None and event.counts_in_sequence]
    if expected_sequence and len(sequence) < len(expected_sequence):
        notes.append("detector enter events are fewer than expected beacons; see track diagnostics")

    return sequence, replay_position, accepted, rejected, notes


def simulate_offline_scaled(rows: list[LogRow], scale_x: float, scale_y: float) -> tuple[list[int], tuple[float, float]]:
    accepted: list[CandidateEvent] = []
    replay_position = (6.0, 1.0)
    last_tick = rows[0]["I0"]
    last_enter_count = int(round(rows[0]["I6"]))
    previous_beacon: int | None = None
    enter_ticks = [int(round(row["I0"])) for row in extract_enter_events(rows)]
    enter_tick_pos = 0

    for row in rows[1:]:
        tick = row["I0"]
        while enter_tick_pos < len(enter_ticks) and enter_ticks[enter_tick_pos] <= int(round(tick)):
            enter_tick_pos += 1
        next_enter_tick = enter_ticks[enter_tick_pos] if enter_tick_pos < len(enter_ticks) else None

        dt_s = max(0.0, (tick - last_tick) * 0.001)
        replay_position = (
            replay_position[0] + (row["I3"] * scale_x * dt_s),
            replay_position[1] + (row["I4"] * scale_y * dt_s),
        )
        last_tick = tick

        enter_count = int(round(row["I6"]))
        if enter_count != last_enter_count:
            event = accept_enter_event(row, replay_position, previous_beacon)
            if event.accepted:
                duplicate_track_fix = (
                    accepted
                    and accepted[-1].source == "track"
                    and accepted[-1].beacon_id == event.beacon_id
                )
                duplicate_disallowed = (
                    accepted
                    and accepted[-1].beacon_id == event.beacon_id
                    and event.beacon_id not in (4, 5)
                )
                if not (duplicate_track_fix or duplicate_disallowed):
                    accepted.append(event)
                    beacon = BEACONS[event.beacon_id - 1]
                    replay_position = blend_position(replay_position, beacon, fix_alpha_for_event(event))
                    previous_beacon = event.beacon_id
            else:
                repeat_fix = make_repeat_enter_fix(row, replay_position, previous_beacon)
                if repeat_fix is not None:
                    accepted.append(repeat_fix)
                    beacon = BEACONS[repeat_fix.beacon_id - 1]
                    replay_position = blend_position(replay_position, beacon, fix_alpha_for_event(repeat_fix))
            last_enter_count = enter_count

        track_event = try_track_candidate(row, replay_position, previous_beacon, accepted, next_enter_tick)
        if track_event is not None and track_event.accepted:
            accepted.append(track_event)
            beacon = BEACONS[track_event.beacon_id - 1]
            replay_position = blend_position(replay_position, beacon, fix_alpha_for_event(track_event))
            previous_beacon = track_event.beacon_id

    sequence = [event.beacon_id for event in accepted if event.beacon_id is not None and event.counts_in_sequence]
    return sequence, replay_position


def evaluate_log(path: Path) -> LogResult:
    rows = read_log(path)
    if not rows:
        raise ValueError(f"{path.name} has no data rows")

    expected = parse_expected_sequence(path.name)
    truth = parse_truth_position(path.name)
    baseline_seq = baseline_sequence(rows)
    baseline_final = (rows[-1]["I23"], rows[-1]["I24"])
    baseline_error = distance(baseline_final, truth)
    offline_seq, offline_final, accepted, rejected, notes = simulate_offline(rows, expected)
    offline_error = distance(offline_final, truth)
    baseline_fix_count = sum(1 for row in rows if int(round(row["I15"])) != 0)
    enter_count = len(extract_enter_events(rows))

    status_items: list[str] = []
    if offline_seq != expected:
        status_items.append("SEQ_FAIL")
    if offline_error > FINAL_ERROR_LIMIT_M:
        status_items.append("ERR_FAIL")
    if expected == [] and offline_seq:
        status_items.append("FALSE_POSITIVE")
    if not status_items:
        status_items.append("PASS")

    needs_manual_review = status_items != ["PASS"] or bool(notes)
    return LogResult(
        file_name=path.name,
        row_count=len(rows),
        expected_sequence=expected,
        truth_position=truth,
        baseline_sequence=baseline_seq,
        baseline_enter_events=enter_count,
        baseline_fix_applied=baseline_fix_count,
        baseline_final_position=baseline_final,
        baseline_final_error_m=baseline_error,
        offline_sequence=offline_seq,
        offline_final_position=offline_final,
        offline_final_error_m=offline_error,
        events=accepted,
        rejections=rejected,
        needs_manual_review=needs_manual_review,
        status=";".join(status_items),
        notes=notes,
    )


def seq_text(sequence: Iterable[int]) -> str:
    items = list(sequence)
    return ",".join(str(item) for item in items) if items else "-"


def write_summary(results: list[LogResult], output_dir: Path) -> None:
    with (output_dir / "position_fix_summary.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "file",
            "rows",
            "expected_sequence",
            "baseline_sequence",
            "offline_sequence",
            "baseline_enter_events",
            "baseline_fix_applied",
            "truth_x",
            "truth_y",
            "baseline_final_x",
            "baseline_final_y",
            "baseline_error_m",
            "offline_final_x",
            "offline_final_y",
            "offline_error_m",
            "status",
            "needs_manual_review",
            "notes",
        ])
        for result in results:
            writer.writerow([
                result.file_name,
                result.row_count,
                seq_text(result.expected_sequence),
                seq_text(result.baseline_sequence),
                seq_text(result.offline_sequence),
                result.baseline_enter_events,
                result.baseline_fix_applied,
                f"{result.truth_position[0]:.6f}",
                f"{result.truth_position[1]:.6f}",
                f"{result.baseline_final_position[0]:.6f}",
                f"{result.baseline_final_position[1]:.6f}",
                f"{result.baseline_final_error_m:.6f}",
                f"{result.offline_final_position[0]:.6f}",
                f"{result.offline_final_position[1]:.6f}",
                f"{result.offline_final_error_m:.6f}",
                result.status,
                int(result.needs_manual_review),
                " | ".join(result.notes),
            ])


def write_events(results: list[LogResult], output_dir: Path) -> None:
    with (output_dir / "position_fix_events.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "file",
            "source",
            "tick",
            "enter_count",
            "beacon_id",
            "distance_m",
            "second_distance_m",
            "position_x",
            "position_y",
            "velocity_x",
            "velocity_y",
            "location",
            "confidence",
            "score",
            "impact_robust_z",
            "applied",
            "counts_in_sequence",
            "reason",
        ])
        for result in results:
            for event in result.events:
                writer.writerow(event_row(result.file_name, event))


def event_row(file_name: str, event: CandidateEvent) -> list[object]:
    return [
        file_name,
        event.source,
        event.tick,
        event.enter_count,
        event.beacon_id if event.beacon_id is not None else "",
        f"{event.distance_m:.6f}" if event.distance_m is not None else "",
        f"{event.second_distance_m:.6f}" if event.second_distance_m is not None else "",
        f"{event.position[0]:.6f}",
        f"{event.position[1]:.6f}",
        f"{event.velocity[0]:.6f}",
        f"{event.velocity[1]:.6f}",
        event.location,
        f"{event.confidence:.6f}",
        f"{event.score:.6f}",
        f"{event.impact_robust_z:.6f}",
        int(event.applied),
        int(event.counts_in_sequence),
        event.reason,
    ]


def write_rejections(results: list[LogResult], output_dir: Path) -> None:
    with (output_dir / "position_fix_rejections.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "file",
            "source",
            "tick",
            "enter_count",
            "nearest_or_diagnostic_beacon_id",
            "distance_m",
            "second_distance_m",
            "position_x",
            "position_y",
            "velocity_x",
            "velocity_y",
            "location",
            "confidence",
            "score",
            "impact_robust_z",
            "reason",
        ])
        for result in results:
            for event in result.rejections:
                row = event_row(result.file_name, event)
                writer.writerow(row[:-3] + [row[-1]])


def run_scale_sweep(logs: list[tuple[Path, list[LogRow], list[int], tuple[float, float]]]) -> list[ScaleSweepResult]:
    sweep_results: list[ScaleSweepResult] = []
    for scale_x_i in range(94, 107, 2):
        for scale_y_i in range(94, 107, 2):
            scale_x = scale_x_i / 100.0
            scale_y = scale_y_i / 100.0
            pass_count = 0
            sequence_pass_count = 0
            error_pass_count = 0
            total_error_m = 0.0
            failed_files: list[str] = []

            for path, rows, expected_sequence, truth_position in logs:
                sequence, final_position = simulate_offline_scaled(rows, scale_x, scale_y)
                final_error_m = distance(final_position, truth_position)
                sequence_ok = sequence == expected_sequence
                error_ok = final_error_m <= FINAL_ERROR_LIMIT_M
                sequence_pass_count += int(sequence_ok)
                error_pass_count += int(error_ok)
                pass_count += int(sequence_ok and error_ok)
                total_error_m += final_error_m
                if not (sequence_ok and error_ok):
                    failed_files.append(path.name)

            sweep_results.append(ScaleSweepResult(
                scale_x=scale_x,
                scale_y=scale_y,
                pass_count=pass_count,
                sequence_pass_count=sequence_pass_count,
                error_pass_count=error_pass_count,
                total_error_m=total_error_m,
                failed_files=failed_files,
            ))

    sweep_results.sort(key=lambda item: (
        -item.pass_count,
        -item.sequence_pass_count,
        -item.error_pass_count,
        item.total_error_m,
    ))
    return sweep_results


def write_scale_sweep(sweep_results: list[ScaleSweepResult], output_dir: Path) -> None:
    with (output_dir / "position_fix_scale_sweep.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "scale_x",
            "scale_y",
            "pass_count",
            "sequence_pass_count",
            "error_pass_count",
            "total_error_m",
            "failed_files",
        ])
        for result in sweep_results:
            writer.writerow([
                f"{result.scale_x:.2f}",
                f"{result.scale_y:.2f}",
                result.pass_count,
                result.sequence_pass_count,
                result.error_pass_count,
                f"{result.total_error_m:.6f}",
                " | ".join(result.failed_files),
            ])


def write_report(
    results: list[LogResult],
    log_dir: Path,
    output_dir: Path,
    scale_sweep_results: list[ScaleSweepResult] | None = None,
) -> None:
    total = len(results)
    pass_count = sum(1 for result in results if result.status == "PASS")
    no_beacon_results = [result for result in results if not result.expected_sequence]
    beacon_results = [result for result in results if result.expected_sequence]
    no_beacon_ok = sum(1 for result in no_beacon_results if not result.offline_sequence)
    seq_ok = sum(1 for result in beacon_results if result.offline_sequence == result.expected_sequence)
    error_ok = sum(1 for result in results if result.offline_final_error_m <= FINAL_ERROR_LIMIT_M)

    lines = [
        "# Position Beacon Fix Offline Report",
        "",
        f"- Log directory: `{log_dir}`",
        f"- CSV logs scanned: {total}",
        f"- Overall pass: {pass_count}/{total}",
        f"- No-beacon rejection pass: {no_beacon_ok}/{len(no_beacon_results)}",
        f"- Beacon sequence pass: {seq_ok}/{len(beacon_results)}",
        f"- Final error <= {FINAL_ERROR_LIMIT_M:.2f}m: {error_ok}/{total}",
        "",
        "## Parameters",
        "",
        f"- EVENT_STRICT_RADIUS_M = {EVENT_STRICT_RADIUS_M}",
        f"- EVENT_LOOSE_RADIUS_M = {EVENT_LOOSE_RADIUS_M}",
        f"- EVENT_AMBIGUITY_MARGIN_M = {EVENT_AMBIGUITY_MARGIN_M}",
        f"- EVENT_MIN_SCORE = {EVENT_MIN_SCORE}",
        f"- EVENT_MIN_CONFIDENCE = {EVENT_MIN_CONFIDENCE}",
        f"- TRACK_DIAG_RADIUS_M = {TRACK_DIAG_RADIUS_M}",
        f"- ODOMETER_REPLAY_SCALE_X = {ODOMETER_REPLAY_SCALE_X}",
        f"- ODOMETER_REPLAY_SCALE_Y = {ODOMETER_REPLAY_SCALE_Y}",
        f"- INITIAL_TRACK_BEACON7_RADIUS_M = {INITIAL_TRACK_BEACON7_RADIUS_M}",
        f"- INITIAL_TRACK_BEACON7_MIN_SCORE = {INITIAL_TRACK_BEACON7_MIN_SCORE}",
        f"- FIX_ALPHA_INITIAL = {FIX_ALPHA_INITIAL}",
        f"- FIX_ALPHA_TRANSITION = {FIX_ALPHA_TRANSITION}",
        f"- FIX_ALPHA_TRACK = {FIX_ALPHA_TRACK}",
        f"- FIX_ALPHA_WEAK = {FIX_ALPHA_WEAK}",
        f"- FIX_ALPHA_STRICT = {FIX_ALPHA_STRICT}",
        f"- FIX_ALPHA_REPEAT_ENTER = {FIX_ALPHA_REPEAT_ENTER}",
        "",
        "## Per-log Summary",
        "",
        "| File | Expected | Baseline | Offline | Truth | Offline final | Error/m | Status | Notes |",
        "| --- | --- | --- | --- | --- | --- | ---: | --- | --- |",
    ]

    for result in results:
        notes = "; ".join(result.notes) if result.notes else ""
        lines.append(
            "| "
            + " | ".join([
                result.file_name,
                seq_text(result.expected_sequence),
                seq_text(result.baseline_sequence),
                seq_text(result.offline_sequence),
                f"({result.truth_position[0]:.2f},{result.truth_position[1]:.2f})",
                f"({result.offline_final_position[0]:.3f},{result.offline_final_position[1]:.3f})",
                f"{result.offline_final_error_m:.3f}",
                result.status,
                notes,
            ])
            + " |"
        )

    lines.extend([
        "",
        "## Rejections And Diagnostics",
        "",
        "Rejected detector events and trajectory-only diagnostics are written to "
        "`position_fix_rejections.csv`. A `track` row means the inertial path passed "
        "near a beacon coordinate but no accepted detector event confirmed it; it is "
        "diagnostic evidence, not an applied fix.",
        "",
        "Accepted events are written to `position_fix_events.csv`. The "
        "`counts_in_sequence` column is the authoritative sequence gate: repeat "
        "calibration events may be applied to position while staying out of the "
        "recognized beacon sequence.",
        "",
        "## Manual Review Items",
        "",
    ])

    failed_results = [result for result in results if result.status != "PASS"]
    if failed_results:
        for result in failed_results:
            details: list[str] = []
            if result.offline_sequence != result.expected_sequence:
                details.append(
                    f"sequence mismatch expected `{seq_text(result.expected_sequence)}` "
                    f"but got `{seq_text(result.offline_sequence)}`"
                )
            if result.offline_final_error_m > FINAL_ERROR_LIMIT_M:
                details.append(f"final error {result.offline_final_error_m:.3f}m exceeds {FINAL_ERROR_LIMIT_M:.2f}m")
            if not result.expected_sequence and result.baseline_sequence:
                details.append(f"baseline falsely confirmed `{seq_text(result.baseline_sequence)}` on a no-beacon log")
            if result.expected_sequence and not result.offline_sequence:
                details.append("no accepted detector/history evidence was available for the expected beacon")
            if result.offline_final_error_m > FINAL_ERROR_LIMIT_M and result.offline_sequence == result.expected_sequence:
                details.append("sequence is correct, so remaining failure is odometer/final-position drift after beacon fixes")
            if "经过7,6,4号信标灯-最终坐标2,1.csv" in result.file_name:
                details.append("third detector event occurs after the 4-beacon pass and is rejected by topology; repeated 4-beacon recalibration was tested and still cannot bring the final error under 0.50m")
            if "经过7号信标灯-最终坐标4,1.csv" in result.file_name:
                details.append("trajectory passes near beacon 7, but detector score/z evidence is weaker than several no-beacon near-pass cases")
            if result.notes:
                details.extend(result.notes)
            lines.append(f"- `{result.file_name}`: " + "; ".join(details))
    else:
        lines.append("- None.")

    if scale_sweep_results:
        best = scale_sweep_results[0]
        lines.extend([
            "",
            "## Scale Sweep",
            "",
            "A coarse velocity-scale sweep was run for x/y scales 0.94..1.06 step 0.02. "
            "This diagnostic checks whether a simple odometer scale change can satisfy "
            "the same sequence and final-error gates.",
            f"- Best scale: x={best.scale_x:.2f}, y={best.scale_y:.2f}",
            f"- Best pass count: {best.pass_count}/{total}",
            f"- Failed files at best scale: {', '.join(best.failed_files) if best.failed_files else 'none'}",
            "",
        ])

    lines.extend([
        "",
        "## C Port Gate",
        "",
        "The offline matcher is the required gate before changing `fixator.c`. "
        "If this report is not fully passing, the current rules must not be "
        "ported to C yet. Failed rows need better detector evidence, additional "
        "logs, or a separate odometer calibration pass before the embedded "
        "fixator can be changed safely.",
        "",
        "Do not directly map every applied offline event to the current 28-channel "
        "C log as `fix_applied + beacon_index`. The offline model distinguishes "
        "position calibration from sequence recognition with `counts_in_sequence`; "
        "the current C telemetry does not expose that bit. Porting the repeat-enter "
        "calibration without extending diagnostics would make a valid calibration "
        "look like an extra beacon in sequence extraction.",
        "",
        "## Verdict",
        "",
    ])

    if pass_count == total:
        lines.append("All offline acceptance gates passed for the simulated matcher.")
    else:
        lines.append(
            "Offline acceptance is not complete. Do not port the matcher to C until "
            "the failed sequence/error rows are resolved or explicitly accepted as "
            "manual-review exceptions."
        )

    (output_dir / "position_fix_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run() -> int:
    parser = argparse.ArgumentParser(description="Evaluate Position beacon-fix CSV logs.")
    parser.add_argument("--log-dir", type=Path, default=None, help="Override CSV log directory.")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR, help="Output artifact directory.")
    parser.add_argument("--sweep-scale", action="store_true", help="Run coarse x/y velocity scale sweep diagnostics.")
    args = parser.parse_args()

    log_dir = args.log_dir if args.log_dir is not None else find_log_dir()
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    paths = sorted(log_dir.glob("*.csv"), key=lambda item: item.name)
    if not paths:
        raise FileNotFoundError(f"No CSV logs found in {log_dir}")

    logs = [(path, read_log(path), parse_expected_sequence(path.name), parse_truth_position(path.name)) for path in paths]
    results = [evaluate_log(path) for path in paths]
    scale_sweep_results = run_scale_sweep(logs) if args.sweep_scale else None
    write_summary(results, output_dir)
    write_events(results, output_dir)
    write_rejections(results, output_dir)
    if scale_sweep_results is not None:
        write_scale_sweep(scale_sweep_results, output_dir)
    write_report(results, log_dir, output_dir, scale_sweep_results)

    pass_count = sum(1 for result in results if result.status == "PASS")
    print(f"Scanned {len(results)} CSV logs from {log_dir}")
    print(f"PASS {pass_count}/{len(results)}")
    print(f"Summary: {output_dir / 'position_fix_summary.csv'}")
    print(f"Report: {output_dir / 'position_fix_report.md'}")
    if scale_sweep_results is not None:
        best = scale_sweep_results[0]
        print(f"Scale sweep best PASS {best.pass_count}/{len(results)} at x={best.scale_x:.2f}, y={best.scale_y:.2f}")
    return 0 if pass_count == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(run())
