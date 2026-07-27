"""
Segment a .ulg into its individual runs, using motor activity.

The flight-controller log is the only continuous record of a session. It covers
what the Pi's CSV structurally cannot:

  - the idle stretches between runs, where the pack is unloaded and its voltage
    is the true state of charge (98-545 s of them, against the 40 s window the
    old code guessed at)
  - the motor actually starting from a stop

That second point matters for alignment. The Pi begins logging once the sweep is
already underway -- its warmup rows are deliberately not recorded -- so the
transition from stopped to spinning is absent from `thrust_map_*.csv` but present
in the ulog. Verified on the real logs: every run is preceded by both channels
sitting at exactly 1000 us.

Segmentation is exact on the current data: log_3_..00-27-40 yields 2 intervals
against 2 known runs, log_3_..17-11-46 yields 10 against 10.
"""

import numpy as np

IDLE_PWM_US = 1050        # below this a channel is at rest
MERGE_GAP_S = 15.0        # activity separated by less than this is one run
MIN_RUN_S = 3.0           # shorter bursts are noise, not a run
IDLE_CURRENT_A = 1.0


def _live_channels(ao):
    """Actuator channels that actually moved, as {index: values}."""
    out = {}
    for i in range(16):
        key = "output[%d]" % i
        if key in ao.data:
            v = np.asarray(ao.data[key], dtype=float)
            if np.ptp(v) > 0:
                out[i] = v
    return out


def load_session(ulog_path):
    """
    Everything needed to segment one log, on the FC boot clock.

    Returns a dict, or None if the log has no actuator data.
    """
    try:
        from pyulog import ULog
        ulog = ULog(ulog_path, ["actuator_outputs", "battery_status"])
    except Exception:
        return None
    aos = [d for d in ulog.data_list
           if d.name == "actuator_outputs" and d.multi_id == 0]
    if not aos:
        return None
    ao = aos[0]
    t = np.asarray(ao.data["timestamp"], dtype=float) / 1e6
    chans = _live_channels(ao)
    if not chans:
        return None

    out = {"t": t, "channels": chans, "path": ulog_path}
    bats = [d for d in ulog.data_list if d.name == "battery_status"]
    if bats:
        b = bats[0]
        out["bat_t"] = np.asarray(b.data["timestamp"], dtype=float) / 1e6
        out["voltage"] = np.asarray(b.data["voltage_v"], dtype=float)
        out["current"] = np.asarray(b.data["current_a"], dtype=float)
    return out


def segment(session, merge_gap_s=MERGE_GAP_S, min_run_s=MIN_RUN_S):
    """
    Motor-active intervals, one per run.

    A run is active while *any* live channel is above idle -- on a coaxial rig
    either rotor turning means the pack is loaded. Bursts closer together than
    `merge_gap_s` are one run, which keeps a momentary dip between steps from
    splitting a sweep in two.
    """
    t, chans = session["t"], session["channels"]
    active = np.zeros(t.size, dtype=bool)
    for v in chans.values():
        active |= v > IDLE_PWM_US
    if not active.any():
        return []

    d = np.diff(active.astype(np.int8))
    starts = list(np.where(d == 1)[0] + 1)
    ends = list(np.where(d == -1)[0] + 1)
    if active[0]:
        starts.insert(0, 0)
    if active[-1]:
        ends.append(t.size - 1)

    runs = []
    for s, e in zip(starts, ends):
        if runs and t[s] - t[runs[-1][1]] < merge_gap_s:
            runs[-1] = (runs[-1][0], e)
        else:
            runs.append((s, e))

    out = []
    for i, (s, e) in enumerate(runs):
        if t[e] - t[s] < min_run_s:
            continue
        out.append({
            "index": len(out) + 1,
            "fc_start": float(t[s]), "fc_end": float(t[e]),
            "duration_s": round(float(t[e] - t[s]), 2),
            "i_start": int(s), "i_end": int(e),
            "idle_before_s": round(float(t[s] - t[runs[i - 1][1]]), 1) if i else round(float(t[s] - t[0]), 1),
            "pwm_at_start": {c: float(v[max(0, s - 2)]) for c, v in chans.items()},
            "pwm_max": {c: float(v[s:e + 1].max()) for c, v in chans.items()},
        })
    return out


def noload_windows(session, runs, guard_s=3.0, max_window_s=120.0):
    """
    State-of-charge voltage before and after each run.

    Uses the whole idle gap up to `max_window_s`, not a fixed guess -- the real
    gaps here run 98-545 s. Requires current near zero *and* every live channel
    at idle, so a single coasting rotor cannot be mistaken for an unloaded pack.
    """
    if "bat_t" not in session or not runs:
        return {}
    bt, v, i = session["bat_t"], session["voltage"], session["current"]
    t, chans = session["t"], session["channels"]

    idle = i < IDLE_CURRENT_A
    for ch in chans.values():
        idle &= np.interp(bt, t, ch) < IDLE_PWM_US

    out = {}
    for k, r in enumerate(runs):
        prev_end = runs[k - 1]["fc_end"] if k else bt[0]
        next_start = runs[k + 1]["fc_start"] if k + 1 < len(runs) else bt[-1]

        before = idle & (bt < r["fc_start"] - guard_s) & \
            (bt > max(prev_end + guard_s, r["fc_start"] - max_window_s))
        after = idle & (bt > r["fc_end"] + guard_s) & \
            (bt < min(next_start - guard_s, r["fc_end"] + max_window_s))

        entry = {
            "before_v": round(float(np.median(v[before])), 3) if before.sum() >= 3 else None,
            "after_v": round(float(np.median(v[after])), 3) if after.sum() >= 3 else None,
            "n_before": int(before.sum()), "n_after": int(after.sum()),
        }
        if entry["before_v"] and entry["after_v"]:
            entry["drop_v"] = round(entry["before_v"] - entry["after_v"], 3)
        entry["v"] = entry["before_v"] or entry["after_v"]
        out[r["index"]] = entry
    return out


def describe(ulog_path):
    """Text report of the runs inside one log."""
    s = load_session(ulog_path)
    if not s:
        return "no actuator data in %s" % ulog_path
    runs = segment(s)
    nl = noload_windows(s, runs)
    lines = ["%s -- %d runs, live channels %s"
             % (ulog_path.split("/")[-1], len(runs), sorted(s["channels"]))]
    lines.append("  %-3s %9s %9s %8s %11s %9s %9s"
                 % ("#", "fc_start", "fc_end", "dur_s", "idle_before", "V_before", "V_after"))
    for r in runs:
        n = nl.get(r["index"], {})
        lines.append("  %-3d %9.1f %9.1f %8.1f %11.1f %9s %9s"
                     % (r["index"], r["fc_start"], r["fc_end"], r["duration_s"],
                        r["idle_before_s"], n.get("before_v", "-"), n.get("after_v", "-")))
    return "\n".join(lines)
