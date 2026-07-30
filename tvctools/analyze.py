"""
Steady-state extraction and the two maps the bench exists to produce:

    thrust/torque vs commanded PWM        (build_map)
    thrust vs battery voltage at fixed PWM (build_sag)

Why this module exists
----------------------
A single load-cell sample cannot resolve a PWM step. Measured on this bench:
within-step noise is sigma ~= 0.7 N while a 100 us step changes thrust by only
0.65-1.5 N, so consecutive steps overlap at ratios of 1.1-2.1 sigma. Trying to
find step edges *in the thrust signal* is therefore hopeless.

It is also unnecessary. The Raspberry Pi commanded those steps and logged them
with timestamps, so the step boundaries are known exactly. Segment by the
command, average within each segment, and the steps separate cleanly: the
standard error of a 160-sample step mean is ~0.11 N against increments of
0.65-1.5 N, i.e. 6-14 sigma.

The noise is vibration, not white measurement error: lag-1 autocorrelation is
~0.5 and the integrated autocorrelation time is ~4 samples. Naive sigma/sqrt(n)
therefore understates the uncertainty by about 2x, so the effective sample size
is corrected below. Better an error bar that is honest than one that is small.
"""

import json
import os

import numpy as np

# Reuse the viewer's robust rejection rather than a second implementation
try:
    from plot import mad_mask
except Exception:  # plot.py needs tkinter/matplotlib to import cleanly
    def mad_mask(x, k=3.5):
        finite = np.isfinite(x)
        if finite.sum() < 4:
            return finite
        med = np.nanmedian(x[finite])
        mad = np.nanmedian(np.abs(x[finite] - med)) * 1.4826
        if mad == 0:
            return finite
        return finite & (np.abs(x - med) <= k * mad)

SETTLE_FRAC = 0.5   # discard this fraction of each step as spin-up transient
TAIL_GUARD_FRAC = 0.12  # ...and this much off the end, against alignment-lag error
MIN_SAMPLES = 8
MAX_AC_LAG = 8      # lags summed for the integrated autocorrelation time


def integrated_autocorr_time(x):
    """
    Bartlett integrated autocorrelation time tau, in samples.

    n/tau is the effective number of independent samples. Returns >= 1.0.
    """
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size < 4 * MAX_AC_LAG:
        return 1.0
    x = x - x.mean()
    denom = float(np.dot(x, x))
    if denom <= 0:
        return 1.0
    tau = 1.0
    for lag in range(1, MAX_AC_LAG + 1):
        r = float(np.dot(x[:-lag], x[lag:])) / denom
        if r <= 0:          # stop at the first non-positive lag (standard practice)
            break
        tau += 2.0 * r
    return max(1.0, tau)


def robust_stats(values, settle_frac=SETTLE_FRAC, reject=True):
    """
    Steady-state estimate for one held command step.

    Drops the leading transient, rejects sensor glitches by MAD, then reports
    the mean with an uncertainty corrected for the vibration autocorrelation.
    """
    a = np.asarray([v for v in values if v is not None], dtype=float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None
    lo = int(a.size * settle_frac)
    # Trim the tail as well. The alignment lag carries a systematic uncertainty
    # of ~0.3 s (different Pixhawk reference signals disagree by that much), and
    # without a trailing guard that error lets the *next* step bleed into the end
    # of this one -- worth up to 0.144 N, comparable to the reported SEM.
    hi = a.size - max(1, int(a.size * TAIL_GUARD_FRAC))
    a = a[lo:hi] if hi - lo >= MIN_SAMPLES else a[lo:]
    if a.size < MIN_SAMPLES:
        return None
    if reject:
        keep = mad_mask(a)
        if keep.sum() >= MIN_SAMPLES:
            a = a[keep]

    tau = integrated_autocorr_time(a)
    n_eff = max(1.0, a.size / tau)
    sd = float(a.std(ddof=1)) if a.size > 1 else 0.0
    return {
        "mean": float(a.mean()),
        "std": sd,
        "n": int(a.size),
        "n_eff": round(n_eff, 1),
        "sem": float(sd / np.sqrt(n_eff)),
        "min": float(a.min()),
        "max": float(a.max()),
    }


# ----------------------------- run loading --------------------------------- #

def load_runs(runs_root):
    """Read every run.json under runs/, newest layout only. Returns list of dicts."""
    out = []
    if not os.path.isdir(runs_root):
        return out
    for session in sorted(os.listdir(runs_root)):
        sdir = os.path.join(runs_root, session)
        if not os.path.isdir(sdir):
            continue
        for run in sorted(os.listdir(sdir)):
            rj = os.path.join(sdir, run, "run.json")
            if os.path.exists(rj):
                with open(rj, "r", encoding="utf-8") as f:
                    d = json.load(f)
                d["_dir"] = os.path.join(sdir, run)
                out.append(d)
    return out


def load_merged(run_dir):
    """merged.csv as a dict of lists, or None when the run had no thrust data."""
    import csv
    path = os.path.join(run_dir, "merged.csv")
    if not os.path.exists(path):
        return None
    cols = None
    with open(path, "r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if cols is None:
                cols = {k: [] for k in row}
            for k, v in row.items():
                cols[k].append(v)
    return cols


# Phases that are not held measurement points. idle_pre/idle_post are the
# no-load windows (state of charge), ramp_* are the smooth transitions --
# all deliberately recorded, none of them a steady state to average.
NON_STEP_PHASES = ("idle_pre", "idle_post", "ramp_down", "ramp_between", "warmup")


def steps_from_merged(cols):
    """
    Group merged rows into command steps.

    Segmentation comes from the commanded phase, never from the thrust signal.
    Transition and idle phases are skipped: they are recorded on purpose (the
    ramp carries the largest thrust edge, the idle windows carry the state of
    charge) but they are not steady states, so averaging them would put
    meaningless points in the map.
    """
    if not cols or "phase" not in cols:
        return []
    steps, cur = [], None
    for i, ph in enumerate(cols["phase"]):
        if not ph or ph in NON_STEP_PHASES:
            cur = None
            continue
        if cur is None or ph != cur["phase"]:
            cur = {"phase": ph, "idx": []}
            steps.append(cur)
        cur["idx"].append(i)
    return steps


def _fnum(cols, key, idx):
    return [float(cols[key][i]) for i in idx
            if key in cols and cols[key][i] not in ("", None)]


def summarize_run(run, settle_frac=SETTLE_FRAC):
    """Per-step steady-state table for one run, with honest uncertainties."""
    cols = load_merged(run["_dir"])
    if not cols:
        return []
    out = []
    for step in steps_from_merged(cols):
        idx = step["idx"]
        thrust = robust_stats(_fnum(cols, "thrust_N", idx), settle_frac)
        if thrust is None:
            continue
        torque = robust_stats(_fnum(cols, "torque_Nm", idx), settle_frac)
        volt = robust_stats(_fnum(cols, "voltage_v", idx), settle_frac)
        cur = robust_stats(_fnum(cols, "current_a", idx), settle_frac)
        b = _fnum(cols, "b_cmd_us", idx)
        a = _fnum(cols, "a_cmd_us", idx)
        entry = {
            "run": run["name"], "session": run["session"], "phase": step["phase"],
            "a_cmd_us": int(round(np.median(a))) if a else None,
            "b_cmd_us": int(round(np.median(b))) if b else None,
            "n": thrust["n"], "n_eff": thrust["n_eff"],
            "thrust_N": round(thrust["mean"], 4),
            "thrust_sd": round(thrust["std"], 4),
            "thrust_sem": round(thrust["sem"], 4),
            "torque_Nm": round(torque["mean"], 5) if torque else None,
            "torque_sem": round(torque["sem"], 5) if torque else None,
            "voltage_v": round(volt["mean"], 3) if volt else None,
            "current_a": round(cur["mean"], 3) if cur else None,
        }
        if entry["voltage_v"] and entry["current_a"]:
            entry["power_w"] = round(entry["voltage_v"] * entry["current_a"], 2)
            if entry["power_w"]:
                entry["efficiency_N_per_W"] = round(entry["thrust_N"] / entry["power_w"], 5)
        out.append(entry)
    return out


# ------------------------------- the maps ---------------------------------- #

def build_map(runs_root, settle_frac=SETTLE_FRAC):
    """
    Every steady-state step from every run, one row each.

    Deliberately NOT averaged across runs: the same PWM produces different
    thrust at different battery voltages, so collapsing runs would hide exactly
    the effect build_sag measures. Filter or group downstream as needed.
    """
    rows = []
    for run in load_runs(runs_root):
        got = summarize_run(run, settle_frac)
        # State of charge, measured with the motor off. Kept separate from the
        # loaded voltage: that one includes an IR sag which varied 0.66-1.18 V
        # between runs, so it is not a usable basis for comparing them.
        nl = (run.get("voltage_noload_v") or {}).get("v")
        for r in got:
            r["voltage_noload_v"] = nl
        rows.extend(got)
    rows.sort(key=lambda r: (r["b_cmd_us"] or 0, r["a_cmd_us"] or 0, r["run"]))
    return rows


MIN_SAG_RUNS = 3        # distinct runs, i.e. distinct battery states
MIN_SAG_VOLT_SPAN = 0.4  # volts; below this the fit is dominated by noise


def build_sag(runs_root, min_runs=MIN_SAG_RUNS, min_span=MIN_SAG_VOLT_SPAN,
              settle_frac=SETTLE_FRAC, include_rejected=False):
    """
    Thrust vs battery voltage at constant commanded PWM.

    This measures a real effect only when the voltage differences come from the
    *pack draining between runs*. Two confounds have to be excluded first, or
    the fit measures nothing:

    1. Repeated sweeps inside a single run are the same battery state a few
       seconds apart. Counting them as independent points fakes a large sample
       from a tiny voltage span, so points are grouped by distinct run.

    2. Within a sweep, voltage is low *because* thrust is high -- the motor's
       own current causes the sag. Correlating the two then measures reverse
       causation and yields a physically absurd negative slope. Groups spanning
       less than `min_span` volts, or fitting a negative slope, are rejected.

    On this data only A1850_B1850 survives, which is exactly the 2026-07-24
    sag test: seven separate holds as the pack drained from 11.32 V to 10.46 V.

    Fits a linear slope and the exponent in thrust ~ V**k; momentum theory gives
    thrust proportional to RPM squared, and RPM roughly proportional to voltage,
    so k near 2 is the expected result.
    """
    groups = {}
    for r in build_map(runs_root, settle_frac):
        # Prefer the no-load voltage: it is the actual state of charge. The
        # loaded value carries a throttle-dependent IR sag on top of it.
        r = dict(r)
        r["_v"] = r.get("voltage_noload_v") or r.get("voltage_v")
        r["_v_basis"] = ("noload" if r.get("voltage_noload_v") else "loaded")
        if r["_v"] is None:
            continue
        groups.setdefault((r["a_cmd_us"], r["b_cmd_us"]), []).append(r)

    out = []
    for (a_us, b_us), pts in sorted(groups.items()):
        # One point per run: average repeated sweeps at the same battery state
        by_run = {}
        for p in pts:
            by_run.setdefault(p["run"], []).append(p)
        pts = [{"run": run,
                "voltage_v": float(np.mean([q["_v"] for q in group])),
                "v_basis": group[0]["_v_basis"],
                "thrust_N": float(np.mean([q["thrust_N"] for q in group])),
                "thrust_sem": float(np.mean([q["thrust_sem"] for q in group]))}
               for run, group in by_run.items()]

        v = np.array([p["voltage_v"] for p in pts], dtype=float)
        t = np.array([p["thrust_N"] for p in pts], dtype=float)

        reject = None
        if len(pts) < min_runs:
            reject = "only %d distinct runs (need %d)" % (len(pts), min_runs)
        elif np.ptp(v) < min_span:
            reject = ("voltage span %.2f V < %.2f V -- within-sweep sag, "
                      "not battery drain" % (np.ptp(v), min_span))
        elif np.ptp(t) <= 0:
            reject = "no thrust variation"

        entry = {"a_cmd_us": a_us, "b_cmd_us": b_us, "n_runs": len(pts),
                 "voltage_basis": pts[0]["v_basis"],
                 "voltage_min": round(float(v.min()), 3),
                 "voltage_max": round(float(v.max()), 3),
                 "voltage_span": round(float(np.ptp(v)), 3),
                 "runs": sorted(p["run"] for p in pts)}

        if reject is None:
            slope, intercept = np.polyfit(v, t, 1)
            r = float(np.corrcoef(v, t)[0, 1])
            k = float(np.polyfit(np.log(v), np.log(t), 1)[0])
            if slope <= 0:
                reject = ("negative slope %.2f N/V -- thrust cannot rise as the "
                          "pack drains; confounded" % slope)
            else:
                entry.update({
                    "thrust_min": round(float(t.min()), 3),
                    "thrust_max": round(float(t.max()), 3),
                    "dthrust_dV_N_per_V": round(float(slope), 4),
                    "intercept_N": round(float(intercept), 4),
                    "r": round(r, 4),
                    "exponent_k": round(k, 3),
                    "pct_thrust_loss": round(100.0 * (t.max() - t.min()) / t.max(), 2),
                    # The exact per-run means the fit was computed on, so the plot
                    # draws scatter and line on the same voltage basis. write_rows
                    # drops list fields, so this stays out of the CSV.
                    "points": pts,
                })
                out.append(entry)

        if reject is not None and include_rejected:
            entry["rejected"] = reject
            out.append(entry)

    return out


DEFAULT_EXPONENT = 1.73   # measured on 2026-07-24 s2; see build_sag


def normalize_to_voltage(rows, v_ref=None, exponent=None):
    """
    Correct each thrust point to a common pack voltage.

    A staircase sweep is not measured at constant voltage: current rises with
    throttle, so the pack sags as the sweep climbs. On r04_005108_1300 the pack
    fell 0.54 V from B=1000 to B=2000, which understates full-throttle thrust by
    ~8.8% relative to the low-throttle points. A raw thrust-vs-PWM curve is
    therefore measured along a voltage gradient, not at one operating point.

    Using thrust ~ V**k from the sag test:

        thrust_corrected = thrust * (v_ref / v_measured) ** k

    This is a first-order correction, not a substitute for testing at constant
    voltage -- it assumes the same exponent everywhere and cannot undo thermal
    drift. Both raw and corrected values are kept so the size of the correction
    stays visible.
    """
    rows = [dict(r) for r in rows]
    vals = [(r.get("voltage_noload_v") or r.get("voltage_v"))
            for r in rows if (r.get("voltage_noload_v") or r.get("voltage_v"))]
    if not vals:
        return rows, None, None
    v_ref = float(v_ref if v_ref is not None else np.median(vals))
    k = float(exponent if exponent is not None else DEFAULT_EXPONENT)
    for r in rows:
        v = r.get("voltage_noload_v") or r.get("voltage_v")
        if not v:
            continue
        factor = (v_ref / v) ** k
        r["thrust_N_raw"] = r["thrust_N"]
        r["thrust_N"] = round(r["thrust_N"] * factor, 4)
        r["thrust_sem"] = round(r["thrust_sem"] * factor, 4)
        if r.get("torque_Nm") is not None:
            r["torque_Nm_raw"] = r["torque_Nm"]
            r["torque_Nm"] = round(r["torque_Nm"] * factor, 5)
        r["v_correction"] = round(factor, 4)
        r["v_ref"] = v_ref
    return rows, v_ref, k


def sag_exponent(runs_root):
    """Best measured thrust ~ V**k exponent, or the default if none available."""
    groups = build_sag(runs_root)
    if not groups:
        return DEFAULT_EXPONENT, None
    best = max(groups, key=lambda g: abs(g.get("r") or 0))
    return best["exponent_k"], best


def write_rows(rows, path, fields=None):
    import csv
    if not rows:
        return None
    fields = fields or [k for k in rows[0] if not isinstance(rows[0][k], list)]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return path
