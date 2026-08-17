#!/usr/bin/env python3
"""Aggregate an `imu_bench` RTT capture into the numbers the firmware needs.

Usage:
    cargo run --release --bin imu_bench -- --probe <serial> > imu_bench.log
    scripts/imu_bench_stats.py imu_bench.log

Reports three things, because three different constants depend on them:

* **within-window sigma** — the per-sample sensor noise. Sets
  `ACCEL_NOISE_MS2` / `GYRO_NOISE_DPS` in `src/hil/imu_sim.rs` and the
  sensor model in the Osiris replay tests.
* **spread of the window MEANS** — what the airbrakes estimator's pad
  calibration actually screens on (`PAD_ACCEL_REJECT_M_S2` = 0.1 m/s^2,
  `BIAS_REJECT_RAD_S` = 0.15 deg/s). A quiet board must sit far inside
  both, or calibration would throw out its own good windows.
* **sample interval** — every integration in the airbrakes estimator uses
  measured dt, and `NOMINAL_SAMPLES_PER_S` (416) is an assumption about the
  part that this measures directly.
"""

import re
import statistics as st
import sys

FIELDS = re.compile(
    r"IMUW (\d+) n=(\d+) "
    r"accmean=(\S+) (\S+) (\S+) accstd=(\S+) (\S+) (\S+) "
    r"gyromean=(\S+) (\S+) (\S+) gyrostd=(\S+) (\S+) (\S+) "
    r"dt_us=(\S+) min=(\d+) max=(\d+)"
)

AXES = "xyz"


def main(path):
    rows = []
    with open(path) as f:
        for line in f:
            m = FIELDS.search(line)
            if m:
                g = m.groups()
                rows.append(
                    dict(
                        n=int(g[1]),
                        accmean=[float(x) for x in g[2:5]],
                        accstd=[float(x) for x in g[5:8]],
                        gyromean=[float(x) for x in g[8:11]],
                        gyrostd=[float(x) for x in g[11:14]],
                        dt=float(g[14]),
                        dtmin=int(g[15]),
                        dtmax=int(g[16]),
                    )
                )
    if len(rows) < 3:
        sys.exit(f"{path}: only {len(rows)} windows parsed — capture longer")

    print(f"{len(rows)} windows, {sum(r['n'] for r in rows)} samples "
          f"({sum(r['n'] for r in rows) * rows[0]['dt'] / 1e6:.0f} s)\n")

    # --- sample timing -----------------------------------------------------
    dt = st.mean(r["dt"] for r in rows)
    print("sample interval")
    print(f"  mean dt   {dt:8.1f} us  ->  {1e6 / dt:6.2f} Hz")
    print(f"  min/max   {min(r['dtmin'] for r in rows)} / "
          f"{max(r['dtmax'] for r in rows)} us")
    print(f"  vs NOMINAL_SAMPLES_PER_S=416 (2403.8 us): "
          f"{(1e6 / dt - 416) / 416 * 100:+.2f}%\n")

    # --- per-sample noise --------------------------------------------------
    # Median, not mean: a bench is never perfectly still for two minutes and
    # a few windows always catch a footstep or a door. Those land in the max
    # and would drag a mean well above the part's actual noise floor.
    print("within-window sigma (per-sample sensor noise) — median over windows")
    for i, a in enumerate(AXES):
        v = [r["accstd"][i] for r in rows]
        print(f"  acc  {a}   {st.median(v):7.4f} m/s^2   "
              f"(quietest {min(v):.4f}, worst {max(v):.4f})")
    for i, a in enumerate(AXES):
        v = [r["gyrostd"][i] for r in rows]
        print(f"  gyro {a}   {st.median(v):7.4f} deg/s   "
              f"(quietest {min(v):.4f}, worst {max(v):.4f})")
    print()

    # --- what the pad calibration screens on -------------------------------
    print("window-MEAN spread (what the pad calibration screens on)")
    worst_acc = 0.0
    for i, a in enumerate(AXES):
        v = [r["accmean"][i] for r in rows]
        med = st.median(v)
        dev = max(abs(x - med) for x in v)
        worst_acc = max(worst_acc, dev)
        print(f"  acc  {a}   median {med:+8.4f}  max |dev| {dev:.4f} m/s^2")
    print(f"           -> PAD_ACCEL_REJECT_M_S2 = 0.1: worst window sits at "
          f"{worst_acc / 0.1 * 100:.0f}% of the reject radius")

    worst_gyro = 0.0
    for i, a in enumerate(AXES):
        v = [r["gyromean"][i] for r in rows]
        med = st.median(v)
        dev = max(abs(x - med) for x in v)
        worst_gyro = max(worst_gyro, dev)
        print(f"  gyro {a}   median {med:+8.4f}  max |dev| {dev:.4f} deg/s")
    print(f"           -> BIAS_REJECT_RAD_S = 0.15 deg/s: worst window sits at "
          f"{worst_gyro / 0.15 * 100:.0f}% of the reject radius\n")

    # --- static readings ---------------------------------------------------
    mag = st.mean(
        sum(x * x for x in r["accmean"]) ** 0.5 for r in rows
    )
    print("static readings")
    print(f"  |gravity|      {mag:.4f} m/s^2  ({mag / 9.80665:.4f} g)")
    print("  gyro bias      " + "  ".join(
        f"{a}={st.median([r['gyromean'][i] for r in rows]):+.4f}"
        for i, a in enumerate(AXES)
    ) + " deg/s")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "imu_bench.log")
