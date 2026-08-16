#!/usr/bin/env python3
"""Validate VLF5 flight-log duration test dumps."""

import csv
import sys
from pathlib import Path

NOMINAL_HZ = 214.0  # measured sustained rate (IMU 416 Hz, loop achieves ~214)
MIN_HZ = 200.0
MAX_HZ = 228.0


def analyze_csv(path: Path, duration_s: float, baseline_rows: int = 0) -> dict:
    rows = list(csv.DictReader(path.open()))
    n = len(rows)
    if n < 2:
        return {"ok": False, "error": f"too few rows: {n}", "rows": n}

    counts = [int(r["record_count"]) for r in rows]
    ts = [int(r["timestamp_us"]) for r in rows]

    # `record_count` restarts at 0 each armed session — the logger lives inside
    # armed mode, so a re-arm (or a reboot) begins a new count. A step backwards
    # is therefore a session boundary, not a dropped record; any other step that
    # isn't +1 is a real drop on the way to the SD card. Records lost at the
    # exact boundary hide inside it, which is the one blind spot of this rule.
    boundaries = {i for i in range(1, len(counts)) if counts[i] < counts[i - 1]}
    sessions = len(boundaries) + 1
    gaps = sum(
        1
        for i in range(1, len(counts))
        if i not in boundaries and counts[i] != counts[i - 1] + 1
    )
    ts_back = sum(1 for i in range(1, len(ts)) if ts[i] < ts[i - 1])

    test_rows = rows[baseline_rows:] if baseline_rows < n else rows
    if len(test_rows) < 2:
        return {"ok": False, "error": "too few test rows after baseline", "rows": n}
    test_ts = [int(r["timestamp_us"]) for r in test_rows]
    span_us = test_ts[-1] - test_ts[0]
    effective_hz = (len(test_rows) - 1) / (span_us / 1e6) if span_us > 0 else 0.0
    new_records = len(test_rows)
    expected = duration_s * NOMINAL_HZ
    pct_of_expected = 100.0 * new_records / expected if expected else 0.0

    imu_valid = sum(1 for r in rows if r.get("imu_valid") == "true")
    baro_valid = sum(1 for r in rows if r.get("baro_valid") == "true")

    # A dump spanning more than one armed session cannot answer this test's
    # question: the timestamp span then includes time the board spent not
    # logging, so the rate math below is not meaningful. Fail on it explicitly
    # rather than let it surface as a mysteriously low Hz.
    ok = (
        gaps == 0
        and sessions == 1
        and ts_back == 0
        and MIN_HZ <= effective_hz <= MAX_HZ
        and pct_of_expected >= 95.0
        and imu_valid >= n * 0.95
        and baro_valid >= n * 0.95
    )

    return {
        "ok": ok,
        "rows": n,
        "test_rows": new_records,
        "first_count": counts[0],
        "last_count": counts[-1],
        "sessions": sessions,
        "gaps": gaps,
        "ts_backwards": ts_back,
        "span_s": span_us / 1e6,
        "effective_hz": effective_hz,
        "expected_rows": int(expected),
        "pct_of_expected": pct_of_expected,
        "imu_valid_pct": 100.0 * imu_valid / n,
        "baro_valid_pct": 100.0 * baro_valid / n,
    }


def main() -> int:
    if len(sys.argv) < 3:
        print(f"usage: {sys.argv[0]} <csv> <duration_seconds> [baseline_rows]", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    duration = float(sys.argv[2])
    baseline_rows = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    r = analyze_csv(path, duration, baseline_rows)
    if "error" in r:
        print(f"FAIL {path}: {r['error']}")
        return 1
    status = "PASS" if r["ok"] else "FAIL"
    print(
        f"{status} {path.name}: total_rows={r['rows']} test_rows={r['test_rows']} "
        f"(expected ~{r['expected_rows']}, {r['pct_of_expected']:.1f}%), "
        f"hz={r['effective_hz']:.1f}, gaps={r['gaps']}, sessions={r['sessions']}, "
        f"span={r['span_s']:.1f}s, imu={r['imu_valid_pct']:.1f}%, baro={r['baro_valid_pct']:.1f}%"
    )
    if r["sessions"] > 1:
        print(
            f"  note: dump spans {r['sessions']} armed sessions (the board was re-armed "
            "or rebooted mid-log), so the rate figures above cover time it was not "
            "logging — clear the log, stay armed for the whole phase, and re-run"
        )
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
