"""
The 2026-07-20 load-cell runs, which are the only measurements that reach FULL
THROTTLE (PWM 2000). Every later session tops out at 1850 us, so without these
the map's high end is extrapolation.

Why they are kept separate from the main map rather than merged into it:

- **One PWM axis, not two.** On 2026-07-20 the STM32 drove the motors directly
  and logged its own `pwm` column, before the Pi/Pixhawk took over. Both rotors
  followed that one signal, so these points are the BALANCED (A = B) case --
  comparable to the coax map's diagonal, but carrying no independent A/B
  information.
- **No voltage record at all.** These files predate the Pixhawk battery
  monitor, so thrust cannot be normalized to a pack voltage the way
  analyze.normalize_to_voltage does for later runs. Pack state is unknown, and
  that is the dominant uncertainty here.

So: use them for the shape and reach of the high end, not as drop-in map rows.
"""
import csv
import glob
import os

import numpy as np

from .analyze import integrated_autocorr_time, robust_stats

# 2026-07-20 is the whole legacy era -- everything after it has a thrust_map
# file and a real voltage record.
LEGACY_DIR = os.path.join("raw", "2026-07-20", "loadcell")
MIN_SAMPLES = 20        # below this a "level" is a ramp passing through, not a hold


def load_legacy_runs(root="."):
    """Per-file, per-PWM-level mean thrust from the 2026-07-20 load-cell runs.

    Returns [{name, points: [{pwm_us, thrust_N, thrust_sem, n, n_eff}]}].
    Thrust is -Fz, the same sign convention analyze.py uses.
    """
    pattern = os.path.join(root, LEGACY_DIR, "*.csv")
    out = []
    for path in sorted(glob.glob(pattern)):
        with open(path, newline="", encoding="utf-8", errors="replace") as f:
            rows = list(csv.DictReader(f))
        by_pwm = {}
        for r in rows:
            try:
                pwm = int(float(r["pwm"]))
                fz = float(r["Fz"])
            except (KeyError, TypeError, ValueError):
                continue
            by_pwm.setdefault(pwm, []).append(-fz)

        points = []
        for pwm, vals in sorted(by_pwm.items()):
            if len(vals) < MIN_SAMPLES:
                continue
            v = np.asarray(vals, dtype=float)
            # Same autocorrelation correction the main pipeline uses: prop
            # vibration is not white, so sd/sqrt(n) would overstate precision.
            tau = integrated_autocorr_time(v)
            n_eff = max(len(v) / max(tau, 1.0), 2.0)
            points.append({
                "pwm_us": pwm,
                "thrust_N": round(float(v.mean()), 4),
                "thrust_sem": round(float(v.std(ddof=1) / np.sqrt(n_eff)), 4),
                "n": len(v), "n_eff": round(n_eff, 1),
            })
        if points:
            out.append({"name": os.path.basename(path).replace(".csv", ""),
                        "points": points})
    return out


def build_legacy_map_rows(root="."):
    """
    2026-07-20 balanced points (A = B), in the same row schema as the rest of
    the map -- for the coax grid and the 3D identification plots, which handle
    arbitrary scattered (A, B) points fine and are exactly where this data is
    missing (the grid had no A=2000/B=2000 cell at all before this).

    Uses robust_stats (same settle/tail trim, MAD rejection, autocorrelation-
    corrected SEM as the rest of the pipeline) rather than load_legacy_runs'
    plain mean, so these rows are comparable to the bench/2026-07-24 ones they
    sit alongside. voltage_v is left None -- these files predate the Pixhawk
    battery monitor, so state of charge here is genuinely unknown, not just
    unrecorded; nothing downstream should assume a value for it.

    Deliberately NOT merged into pwm_thrust_torque_map.png's per-A-level curve
    panel (see cli.py) -- that chart groups by a_cmd_us, and since A=B here
    every level is its own single-point "series", which would just clutter a
    chart the existing dashed overlay (plot_map's legacy_runs param) already
    covers more clearly.
    """
    pattern = os.path.join(root, LEGACY_DIR, "*.csv")
    rows = []
    for path in sorted(glob.glob(pattern)):
        with open(path, newline="", encoding="utf-8", errors="replace") as f:
            raw = list(csv.DictReader(f))
        by_pwm = {}
        for r in raw:
            try:
                pwm = int(float(r["pwm"]))
            except (KeyError, TypeError, ValueError):
                continue
            by_pwm.setdefault(pwm, []).append(r)

        name = os.path.basename(path).replace(".csv", "")
        for pwm, rs in sorted(by_pwm.items()):
            if len(rs) < MIN_SAMPLES:
                continue
            if pwm % 100 != 0:
                # Everywhere else in the dataset commands step in multiples of
                # 100 us. Two of the three 2026-07-20 files are continuous
                # manual ramps rather than discrete holds, and a ramp lingering
                # near an arbitrary value (1158, 1332, 1951, ...) can still
                # clear MIN_SAMPLES without being a real held step -- those
                # would otherwise litter the grid axis with noise. Only
                # data_20260720_183744.csv's clean 1000-2000 staircase is
                # 100-us-aligned throughout, so this filter keeps that one and
                # drops the incidental ramp points without naming the file.
                continue
            thrust = robust_stats([-float(r["Fz"]) for r in rs])
            if thrust is None:
                continue
            torque = robust_stats([float(r["Tz"]) for r in rs
                                   if r.get("Tz") not in (None, "")])
            entry = {
                "run": "legacy_%s" % name, "session": "legacy_2026-07-20",
                "phase": "A%d_B%d" % (pwm, pwm),
                "a_cmd_us": pwm, "b_cmd_us": pwm,
                "n": thrust["n"], "n_eff": thrust["n_eff"],
                "thrust_N": round(thrust["mean"], 4),
                "thrust_sem": round(thrust["sem"], 4),
                "torque_Nm": round(torque["mean"], 5) if torque else None,
                "torque_sem": round(torque["sem"], 5) if torque else None,
                "voltage_v": None, "current_a": None,
            }
            rows.append(entry)
    return rows


def full_throttle_summary(root="."):
    """Max thrust reached at/near full throttle, across the legacy runs.

    This is the number that sets the vehicle's thrust-to-weight ceiling, and
    it exists nowhere else in the dataset.
    """
    runs = load_legacy_runs(root)
    best = []
    for run in runs:
        top = max(run["points"], key=lambda p: p["thrust_N"])
        at_2000 = next((p for p in run["points"] if p["pwm_us"] >= 2000), None)
        best.append({
            "run": run["name"],
            "max_thrust_N": top["thrust_N"], "max_at_pwm": top["pwm_us"],
            "n": top["n"],
            "thrust_at_2000_N": at_2000["thrust_N"] if at_2000 else None,
        })
    return best
