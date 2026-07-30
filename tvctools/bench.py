"""
Ingest a `bench/<run_id>/` directory from the newer multi-stream acquisition
harness (manifest.json + sequence.csv + per-device CSVs).

This is a different generation of logger from `pwm_thrust_map.py` /
`gui.py`: every sample on every device carries a `seg_id` assigned at
acquisition time, and `manifest.json` already carries a fitted clock (slope,
offset, ppm drift) per device against a common `t_mono`. That means there is
no cross-correlation alignment step here -- the hard problem the rest of this
package exists to solve (`align.py`, `merge.py`) is already solved upstream,
by construction, for any run recorded this way.

What this module does is the part that IS still needed: turn each `step` /
`reference` segment into one steady-state map point, using the same
settle-trim, robust rejection, and autocorrelation-corrected uncertainty as
`analyze.py` uses for the older data, so the two sources are comparable.
"""

import csv
import json
import os

import numpy as np

from .analyze import robust_stats, SETTLE_FRAC, TAIL_GUARD_FRAC

# Segment kinds that are steady commanded holds, worth a map point.
# tare/warmup/idle/chirp/ramp_* are deliberate but not steady states.
MAP_KINDS = ("step", "reference")

# The torque channel on this rig is far less noisy than thrust: repeated
# combos agree to 4 decimal places and B=1500 (=A) reads exactly 0.0000 Nm on
# every visit (verified against tvctools/bench.py's own run -- torque_count
# increments normally and Fz shows ordinary vibration in the same window, so
# this is a well-filtered channel, not a stuck sensor). robust_stats()
# therefore reports SEM as literally 0.0 for most segments. That is an honest
# reflection of the data, but a literal zero is a nuisance downstream (a
# weighted fit dividing by sem**2, a plotted error bar with no visible extent),
# so a small floor is applied at report time -- half the smallest real
# distance between two distinct commanded torque levels seen in this run.
TORQUE_SEM_FLOOR_NM = 0.0015


def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def _by_seg(rows, key="seg_id"):
    """Group CSV rows by segment id, values as float where possible."""
    out = {}
    for r in rows:
        out.setdefault(r[key], []).append(r)
    return out


def _floats(rows, col):
    vals = []
    for r in rows:
        v = r.get(col)
        if v not in (None, ""):
            try:
                vals.append(float(v))
            except ValueError:
                pass
    return vals


def load_bench_run(run_dir, settle_frac=SETTLE_FRAC, tail_guard=TAIL_GUARD_FRAC):
    """
    Load one bench run directory into map-point rows.

    Returns (rows, manifest). Each row has the same keys build_map() produces
    (a_cmd_us, b_cmd_us, thrust_N, thrust_sem, torque_Nm, torque_sem,
    voltage_v, current_a, power_w, efficiency_N_per_W, run, session, n, n_eff)
    so it can be concatenated directly with the older pipeline's output.
    """
    manifest_path = os.path.join(run_dir, "manifest.json")
    if not os.path.exists(manifest_path):
        return [], None
    with open(manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    run_id = manifest.get("run_id", os.path.basename(run_dir))

    seq = _read_csv(os.path.join(run_dir, "sequence.csv"))
    loadcell = _by_seg(_read_csv(os.path.join(run_dir, "loadcell.csv")))
    battery = _by_seg(_read_csv(os.path.join(run_dir, "fc_battery.csv")))

    rows = []
    for seg in seq:
        if seg["kind"] not in MAP_KINDS:
            continue
        sid = seg["seg_id"]
        lc = loadcell.get(sid, [])
        if not lc:
            continue
        # seg_id already delimits this exact command hold -- sorted by t_mono
        # so settle_frac/tail_guard trim the same way analyze.py trims a
        # command-segmented step from the older pipeline.
        lc = sorted(lc, key=lambda r: float(r["t_mono"]))
        thrust = robust_stats([-float(r["Fz"]) for r in lc], settle_frac)
        if thrust is None:
            continue
        torque = robust_stats(_floats(lc, "Tz"), settle_frac)

        bat = sorted(battery.get(sid, []), key=lambda r: float(r["t_mono"]))
        volt = robust_stats(_floats(bat, "voltage_v"), settle_frac) if bat else None
        cur = robust_stats(_floats(bat, "current_a"), settle_frac) if bat else None

        entry = {
            "run": "bench_%s" % run_id, "session": "bench_%s" % run_id,
            "phase": "A%s_B%s" % (seg["a_us"], seg["b_us"]),
            "a_cmd_us": int(float(seg["a_us"])), "b_cmd_us": int(float(seg["b_us"])),
            "n": thrust["n"], "n_eff": thrust["n_eff"],
            "thrust_N": round(thrust["mean"], 4),
            "thrust_sd": round(thrust["std"], 4),
            "thrust_sem": round(thrust["sem"], 4),
            "torque_Nm": round(torque["mean"], 5) if torque else None,
            "torque_sem": (round(max(torque["sem"], TORQUE_SEM_FLOOR_NM), 5)
                          if torque else None),
            "voltage_v": round(volt["mean"], 3) if volt else None,
            "current_a": round(cur["mean"], 3) if cur else None,
            "seg_kind": seg["kind"], "seg_id": int(sid),
            "t_mono_start": float(seg["t_mono_start"]),
        }
        if entry["voltage_v"] and entry["current_a"]:
            entry["power_w"] = round(entry["voltage_v"] * entry["current_a"], 2)
            if entry["power_w"]:
                entry["efficiency_N_per_W"] = round(entry["thrust_N"] / entry["power_w"], 5)
        rows.append(entry)

    return rows, manifest


def find_bench_runs(root="."):
    """Every bench/<run_id>/ directory with a manifest, oldest first."""
    base = os.path.join(root, "bench")
    if not os.path.isdir(base):
        return []
    out = []
    for name in sorted(os.listdir(base)):
        d = os.path.join(base, name)
        if os.path.isfile(os.path.join(d, "manifest.json")):
            out.append(d)
    return out


def load_all_bench_runs(root=".", settle_frac=SETTLE_FRAC, tail_guard=TAIL_GUARD_FRAC):
    """Concatenated map rows from every bench/<run_id>/ directory found."""
    rows = []
    for d in find_bench_runs(root):
        r, _m = load_bench_run(d, settle_frac, tail_guard)
        rows.extend(r)
    return rows


def reference_drift(manifest, seq_rows, run_dir):
    """
    Thrust at the repeated reference point over the run, for a drift check.

    The plan re-visits one fixed (A, B) combo every N steps specifically so
    drift (battery, thermal) can be measured directly instead of assumed. This
    reads the acquisition system's own precomputed segment means -- no need to
    re-touch the loadcell CSV.
    """
    ref_cfg = (manifest or {}).get("plan", {}).get("reference", {})
    a_ref, b_ref = ref_cfg.get("a"), ref_cfg.get("b")
    refs = [r for r in seq_rows if r["kind"] == "reference"]
    out = []
    for r in refs:
        out.append({
            "seg_id": int(r["seg_id"]), "t_mono": float(r["t_mono_start"]),
            "thrust_N": float(r["thrust_mean_n"]),
            "thrust_sem_n": float(r["thrust_sem_n"]),
        })
    return {"a_us": a_ref, "b_us": b_ref, "points": out}
