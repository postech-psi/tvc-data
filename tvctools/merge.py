"""
Merge a run's two data streams into one time-aligned table.

Output is a single CSV per run on a common 20 Hz epoch grid carrying the
commanded PWM, the measured voltage/current that resulted, and the force/torque
the stand recorded -- the join that makes a thrust/torque-vs-PWM map possible.
"""

import csv
import os

import numpy as np

from .align import estimate_lag, thrust_signal, drive_signal, edge_lag
from .schema import PWM_MAP, LOADCELL, load_any, time_axis

# The load cell samples at 50 Hz on every file recorded so far, and thrust is
# the noisy channel that benefits most from having every sample available. The
# grid therefore matches the load cell rather than the 20 Hz Pixhawk stream:
# downsampling to 20 Hz would throw away 60% of the force data. Commands are
# zero-order held (exact) and voltage/current interpolated, so nothing is
# invented by carrying the slower stream onto the faster grid.
GRID_HZ = 50.0

MERGED_FIELDS = [
    "t_rel_s", "t_epoch",
    "phase", "sweep_idx", "a_cmd_us", "b_cmd_us", "servo1_raw", "servo2_raw",
    "voltage_v", "current_a", "power_w",
    "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "thrust_N", "torque_Nm", "rpm",
]

PAD_S = 2.0  # keep a little idle either side of the commanded window
EDGE_DISAGREE_S = 1.5  # xcorr vs single-edge estimates further apart than this are suspect


def _interp(grid, t, v):
    """Linear interpolation with NaN outside the source span."""
    out = np.interp(grid, t, v, left=np.nan, right=np.nan)
    out[(grid < t[0]) | (grid > t[-1])] = np.nan
    return out


def _hold_idx(grid, t):
    """Index of the most recent sample at or before each grid point."""
    return np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)


def _hold(grid, t, v):
    """
    Zero-order hold. Commands are staircases -- a commanded 1000 followed by a
    commanded 1100 was never 1043 in between, so these must not be interpolated.
    """
    out = np.asarray(v, dtype=float)[_hold_idx(grid, t)]
    out = out.copy()
    out[(grid < t[0]) | (grid > t[-1])] = np.nan
    return out


def _hold_text(grid, t, values):
    """Zero-order hold for string columns (phase), blank outside the span."""
    out = np.asarray(values, dtype=object)[_hold_idx(grid, t)].copy()
    out[(grid < t[0]) | (grid > t[-1])] = ""
    return out


def estimate_file_lags(runs, root):
    """
    One clock offset per load-cell file, shared by every run that uses it.

    The offset is a property of the recording (when the GUI opened the file
    relative to the MCU's uptime), not of the sweep, so two runs backed by the
    same thrust.csv cannot have different lags.

    Estimating per run gets this wrong: a short sweep cross-correlated against a
    long recording that contains several similar staircases can lock onto the
    neighbouring sweep and report a confident but nonsense offset. Estimating
    once from the *longest* overlapping sweep uses the most distinctive signal
    available and keeps every run on that file consistent.

    Returns {loadcell_rel_path: (lag_s, corr, method, drive_signal)}.
    """
    by_file = {}
    for run in runs:
        if run["pwm_map"] and run["loadcell"]:
            by_file.setdefault(run["loadcell"]["rel_path"], []).append(run)

    lags = {}
    for rel, group in by_file.items():
        # Longest pwm_map == most staircase structure == least ambiguous match
        best = max(group, key=lambda r: r["pwm_map"]["duration_s"] or 0)
        _, pm_cols, pm_meta = load_any(os.path.join(root, best["pwm_map"]["rel_path"]))
        _, lc_cols, lc_meta = load_any(os.path.join(root, rel))
        if "Fz" not in lc_cols:
            continue
        pm_t = time_axis(PWM_MAP, pm_cols, pm_meta["start_epoch"])
        lc_t = time_axis(LOADCELL, lc_cols, lc_meta["start_epoch"])
        drive, src = drive_signal(pm_cols)
        lag, corr, how = estimate_lag(lc_t, thrust_signal(lc_cols), pm_t, drive)
        lags[rel] = (lag, corr, how, src, best["name"])
    return lags


def merge_run(run, root, grid_hz=GRID_HZ, file_lags=None):
    """
    Build the merged table for one run.

    Returns (rows, info). info records the estimated lag, its correlation and
    what was actually available, and is stored in run.json so a later reader can
    tell a solid alignment from a coarse filename guess.
    """
    pm_rec, lc_rec = run["pwm_map"], run["loadcell"]
    info = {"lag_s": None, "lag_corr": None, "lag_method": None,
            "drive_signal": None, "grid_hz": grid_hz, "n_rows": 0}

    pm_cols = pm_t = lc_cols = lc_t = None
    if pm_rec:
        _, pm_cols, pm_meta = load_any(os.path.join(root, pm_rec["rel_path"]))
        pm_t = time_axis(PWM_MAP, pm_cols, pm_meta["start_epoch"])
    if lc_rec:
        _, lc_cols, lc_meta = load_any(os.path.join(root, lc_rec["rel_path"]))
        lc_t = time_axis(LOADCELL, lc_cols, lc_meta["start_epoch"])

    # Refine the load-cell clock against the Pixhawk when both are present
    if pm_cols is not None and lc_cols is not None and "Fz" in lc_cols:
        cached = (file_lags or {}).get(lc_rec["rel_path"])
        if cached:
            lag, corr, how, src, from_run = cached
            if from_run != run["name"]:
                how = "%s (from %s)" % (how, from_run)
        else:
            drive, src = drive_signal(pm_cols)
            lag, corr, how = estimate_lag(lc_t, thrust_signal(lc_cols), pm_t, drive)
        # Independent check from the single biggest transition. Cross-correlation
        # fits the whole record and can, on a repetitive staircase, lock onto the
        # wrong cycle; the largest edge cannot. Disagreement is the symptom.
        e_lag, e_jump = edge_lag(lc_t, thrust_signal(lc_cols), pm_t,
                                 pm_cols.get("b_cmd_us", []))
        lc_t = lc_t + lag
        info.update(lag_s=lag, lag_corr=corr, lag_method=how, drive_signal=src,
                    edge_lag_s=e_lag, edge_jump_n=e_jump)
        if e_lag is not None:
            info["edge_vs_xcorr_s"] = round(e_lag - lag, 3)
            if abs(e_lag - lag) > EDGE_DISAGREE_S:
                info["flags"] = info.get("flags", []) + ["edge_disagrees"]

    lo, hi = run["window"]
    lo, hi = lo - PAD_S, hi + PAD_S
    grid = np.arange(lo, hi, 1.0 / grid_hz)
    if grid.size == 0:
        return [], info

    out = {"t_epoch": grid, "t_rel_s": grid - grid[0]}

    if pm_cols is not None:
        # Measured signals interpolate; commanded ones are held.
        for key in ("servo1_raw", "servo2_raw", "voltage_v", "current_a"):
            if key in pm_cols:
                out[key] = _interp(grid, pm_t, pm_cols[key])
        for key in ("a_cmd_us", "b_cmd_us", "sweep_idx"):
            if key in pm_cols:
                out[key] = _hold(grid, pm_t, pm_cols[key])
        if "voltage_v" in out and "current_a" in out:
            out["power_w"] = out["voltage_v"] * out["current_a"]
        if "phase" in pm_cols:
            out["phase"] = _hold_text(grid, pm_t, pm_cols["phase"])

    if lc_cols is not None:
        for key in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "rpm"):
            if key in lc_cols:
                out[key] = _interp(grid, lc_t, lc_cols[key])
        if "Fz" in out:
            out["thrust_N"] = -out["Fz"]
        if "Tz" in out:
            out["torque_Nm"] = out["Tz"]

    rows = []
    for i in range(grid.size):
        row = {}
        for key in MERGED_FIELDS:
            if key not in out:
                row[key] = ""
                continue
            val = out[key][i]
            if key == "phase":
                row[key] = val
            elif not np.isfinite(val):
                row[key] = ""
            elif key in ("a_cmd_us", "b_cmd_us", "sweep_idx", "servo1_raw", "servo2_raw"):
                row[key] = int(val)
            else:
                row[key] = round(float(val), 5)
        rows.append(row)

    # Trim the leading/trailing padding where neither stream had data
    keep = [i for i, r in enumerate(rows)
            if r["thrust_N"] != "" or r["b_cmd_us"] != ""]
    if keep:
        rows = rows[keep[0]:keep[-1] + 1]
        t0 = float(rows[0]["t_epoch"])
        for r in rows:
            r["t_rel_s"] = round(float(r["t_epoch"]) - t0, 3)

    info["n_rows"] = len(rows)
    return rows, info


def write_merged(rows, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=MERGED_FIELDS)
        w.writeheader()
        w.writerows(rows)


def steady_state_by_phase(rows, settle_frac=0.5, min_samples=5):
    """
    Steady-state average per commanded phase.

    Drops the first `settle_frac` of each phase to skip the spin-up transient --
    the same settling convention plot.py:aggregate_by_pwm uses for PWM steps.
    """
    order, groups = [], {}
    for r in rows:
        ph = r.get("phase") or ""
        if not ph:
            continue
        if ph not in groups:
            groups[ph] = []
            order.append(ph)
        groups[ph].append(r)

    out = []
    for ph in order:
        g = groups[ph]
        g = g[int(len(g) * settle_frac):]
        if len(g) < min_samples:
            continue
        entry = {"phase": ph, "n": len(g)}
        for key in ("a_cmd_us", "b_cmd_us", "voltage_v", "current_a", "power_w",
                    "thrust_N", "torque_Nm"):
            vals = [float(r[key]) for r in g if r.get(key) not in ("", None)]
            entry[key] = round(sum(vals) / len(vals), 4) if vals else None
        if entry.get("thrust_N") and entry.get("power_w"):
            # N per electrical watt -- the number that actually matters for sizing
            entry["efficiency_N_per_W"] = round(entry["thrust_N"] / entry["power_w"], 5)
        out.append(entry)
    return out
