#!/usr/bin/env python3
"""Bake the Osiris OpenRocket trajectory into a Rust table for the HIL replay.

    scripts/gen_osiris_hil_table.py > src/hil/osiris_table.rs

Source is the same CSV the host-side estimator tests replay
(`Rust_Monorepo/air-brakes-controller-core/test_data/osiris_o3400.csv`,
produced by `DumpSim.java` from `2026_06_26 - Osiris LC FDR.ork`, sim
"2026_07_02 - LC Comp Final"), so the bench and the test suite fly the same
flight.

# What each column is, and why it is that column

The board's HIL seam needs, at any flight time: a barometer pressure, an
accelerometer vector, and a gyro vector. Rather than bake the two sensor
vectors directly, this bakes the *physics* and lets the device assemble the
vectors, because the roll then stays exact at any sample rate instead of
being interpolated:

* `pressure_pa`  — OpenRocket's static pressure. The only altitude the board
  can see.
* `axial_sf`     — specific force along the airframe axis, (thrust - drag)/mass.
  Gravity is deliberately absent: an accelerometer in free flight does not
  sense it. Before the rocket leaves the rail the pad carries the weight, so
  those rows read +g instead; the two agree at the instant thrust/mass
  reaches g, which is exactly where liftoff is defined.
* `wx0, wy0, wz0` — body rates in a NON-ROLLING body frame (deg/s), from the
  zenith/azimuth columns:
      w = (-az_dot*sin(tilt),  tilt_dot,  az_dot*cos(tilt))
  The device rotates these into the rolling frame itself.

Sampling is dense where the signal is (100 Hz through burn, coast and
apogee) and sparse where it is not (4 Hz under the parachutes), which is the
difference between a 150 kB table and a 3 MB one.

After apogee the axis stops meaning anything — the airframe is hanging under
a chute, and OpenRocket stops reporting body rates there anyway — so the
rates are zeroed and the specific force is the 1 g of a body in
aerodynamically supported descent.
"""

import csv
import math
import os
import sys

CSV = os.path.expanduser(
    "~/Projects/RocketryProjects/Rust_Monorepo/air-brakes-controller-core"
    "/test_data/osiris_o3400.csv"
)

FINE_UNTIL_S = 45.0
FINE_HZ = 100.0
COARSE_HZ = 4.0
END_S = 480.0


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            def g(k):
                v = r[k]
                return float(v) if v else None
            rows.append(dict(
                t=g("time_s"),
                pressure=g("pressure_pa"),
                zenith=g("zenith_rad"),
                azimuth=g("azimuth_rad"),
                mass=g("mass_kg"),
                thrust=g("thrust_n"),
                drag=g("drag_n"),
                gravity=g("gravity_mps2"),
                alt=g("altitude_agl_m"),
            ))
    return rows


def interp(rows, key, t):
    """Linear interpolation on the source grid, clamped at both ends."""
    if t <= rows[0]["t"]:
        return rows[0][key]
    if t >= rows[-1]["t"]:
        return rows[-1][key]
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid]["t"] <= t:
            lo = mid
        else:
            hi = mid
    a, b = rows[lo], rows[hi]
    s = (t - a["t"]) / (b["t"] - a["t"])
    return a[key] + (b[key] - a[key]) * s


def main():
    rows = load(CSV)
    apogee = max(rows, key=lambda r: r["alt"])
    g0 = rows[0]["gravity"]

    # Liftoff: the instant the axial specific force reaches 1 g. Below it the
    # pad is still carrying the rocket and the accelerometer reads +g; above
    # it the rocket is flying and reads (thrust - drag)/mass. Defining it this
    # way makes the two expressions equal at the crossing, so the modelled
    # accelerometer has no step in it.
    liftoff_t = next(
        r["t"] for r in rows if (r["thrust"] - r["drag"]) / r["mass"] >= g0
    )

    # Sample grid
    times = []
    t = 0.0
    while t < FINE_UNTIL_S:
        times.append(t)
        t += 1.0 / FINE_HZ
    while t <= END_S:
        times.append(t)
        t += 1.0 / COARSE_HZ

    out = []
    for t in times:
        pressure = interp(rows, "pressure", t)

        if t > apogee["t"]:
            # under the chutes: axis is meaningless, 1 g of aerodynamic support
            axial, w = g0, (0.0, 0.0, 0.0)
        elif t < liftoff_t:
            axial, w = g0, (0.0, 0.0, 0.0)
        else:
            mass = interp(rows, "mass", t)
            axial = (interp(rows, "thrust", t) - interp(rows, "drag", t)) / mass

            h = 0.005
            zen_a = interp(rows, "zenith", max(t - h, 0.0))
            zen_b = interp(rows, "zenith", t + h)
            az_a = interp(rows, "azimuth", max(t - h, 0.0))
            az_b = interp(rows, "azimuth", t + h)
            span = (t + h) - max(t - h, 0.0)
            tilt_dot = -(zen_b - zen_a) / span
            az_dot = (az_b - az_a) / span
            tilt = math.pi / 2 - interp(rows, "zenith", t)
            w = (
                math.degrees(-az_dot * math.sin(tilt)),
                math.degrees(tilt_dot),
                math.degrees(az_dot * math.cos(tilt)),
            )
        out.append((t, pressure, axial, *w))

    w = sys.stdout.write
    w("//! Pre-baked Osiris trajectory for the HIL replay — GENERATED, do not edit.\n")
    w("//!\n")
    w("//! Regenerate with `scripts/gen_osiris_hil_table.py > src/hil/osiris_table.rs`.\n")
    w("//! See that script for what each column means and why it is stored this way,\n")
    w("//! and `super::osiris` for how the device turns these rows back into sensor\n")
    w("//! readings.\n")
    w("//!\n")
    w(f"//! Source: `osiris_o3400.csv` (OpenRocket, sim \"2026_07_02 - LC Comp Final\").\n")
    w(f"//! Apogee {apogee['alt']:.0f} m AGL at t={apogee['t']:.2f} s; liftoff at "
      f"t={liftoff_t:.3f} s.\n")
    w(f"//! {len(out)} rows: {FINE_HZ:.0f} Hz to {FINE_UNTIL_S:.0f} s, then "
      f"{COARSE_HZ:.0f} Hz to {END_S:.0f} s.\n\n")
    w("/// One row: `[t_s, pressure_pa, axial_specific_force_ms2, "
      "wx_dps, wy_dps, wz_dps]`.\n")
    w("///\n")
    w("/// The three rates are in a NON-ROLLING body frame; the roll is applied on\n")
    w("/// the device so it stays exact between rows.\n")
    w(f"pub static TABLE: [[f32; 6]; {len(out)}] = [\n")
    for r in out:
        # explicit decimal places, NOT {:.4} — that is significant figures,
        # which would quantise the time column to 0.1 s past t=100 and can
        # collide two rows onto one timestamp
        w("    [{:.4f}, {:.2f}, {:.4f}, {:.5f}, {:.5f}, {:.5f}],\n".format(*r))
    w("];\n\n")
    w(f"/// Flight time of the last row (s).\n")
    w(f"pub const END_S: f32 = {out[-1][0]:.4f};\n\n")
    w(f"/// True apogee, for the RTT log to score the run against.\n")
    w(f"pub const TRUE_APOGEE_AGL_M: f32 = {apogee['alt']:.1f};\n")
    w(f"pub const TRUE_APOGEE_T_S: f32 = {apogee['t']:.3f};\n")

    print(
        f"// generated {len(out)} rows, ~{len(out) * 24 / 1024:.0f} kB of flash",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
