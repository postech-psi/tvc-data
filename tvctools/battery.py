"""
No-load battery voltage: the actual state of charge, not the transient sag.

The voltage recorded *during* a run is the pack terminal voltage under 20-25 A
of load, which is the open-circuit voltage minus an I*R drop. Two quite
different things are mixed into that one number:

  - real depletion of the pack over the session (what we want)
  - instantaneous IR sag under whatever throttle is applied (what we do not)

They are not separable after the fact and the IR term is not even constant --
measured across the 2026-07-24 session-2 runs it ranged 0.66 V to 1.18 V. So
comparing runs by their loaded voltage compares two confounded quantities.

The fix is to read the voltage when the motor is *off*. The ulog is the only
source that covers the idle stretches between runs (thrust_map logs the sweep
only), so no-load voltage is measured there: battery samples where current is
near zero and the throttle is at idle, taken from the window just before a run
starts.

Measured on session 2: no-load voltage fell 12.61 -> 11.06 V over the session
(1.55 V of genuine depletion) while the mean IR sag under load was 1.04 V.
"""

import numpy as np

IDLE_CURRENT_A = 1.0     # below this the pack is effectively unloaded
IDLE_PWM_US = 1050       # and the throttle is at idle
PRE_WINDOW_S = 40.0      # look this far back for idle samples before a run
PRE_GUARD_S = 2.0        # ...but stop short of the run, to miss the spin-up
MIN_IDLE_SAMPLES = 3


def load_battery_idle(ulog_path):
    """
    Battery stream plus an idle mask, on the FC boot clock.

    Returns (t, voltage, current, idle_mask) or None.
    """
    try:
        from pyulog import ULog
        ulog = ULog(ulog_path, ["actuator_outputs", "battery_status"])
    except Exception:
        return None
    bats = [d for d in ulog.data_list if d.name == "battery_status"]
    aos = [d for d in ulog.data_list
           if d.name == "actuator_outputs" and d.multi_id == 0]
    if not bats:
        return None

    b = bats[0]
    t = np.asarray(b.data["timestamp"], dtype=float) / 1e6
    v = np.asarray(b.data["voltage_v"], dtype=float)
    i = np.asarray(b.data["current_a"], dtype=float)
    idle = i < IDLE_CURRENT_A

    if aos:
        ao = aos[0]
        at = np.asarray(ao.data["timestamp"], dtype=float) / 1e6
        # Coaxial rig: BOTH rotors must be at idle, not just the throttle
        # channel. A single spinning rotor still loads the pack, so requiring
        # only one channel would let a partially-driven state count as no-load.
        for ch in range(8):
            key = "output[%d]" % ch
            if key not in ao.data:
                continue
            arr = np.asarray(ao.data[key], dtype=float)
            if np.ptp(arr) <= 0:
                continue                      # unused/constant channel
            idle &= np.interp(t, at, arr) < IDLE_PWM_US
    return t, v, i, idle


def noload_before(bat, fc_start, window_s=PRE_WINDOW_S, guard_s=PRE_GUARD_S):
    """
    Median no-load voltage in the idle window just before `fc_start`.

    Returns (voltage, n_samples) or (None, 0). Uses the median so a single
    glitch cannot move it.
    """
    if bat is None:
        return None, 0
    t, v, _i, idle = bat
    sel = idle & (t > fc_start - window_s) & (t < fc_start - guard_s)
    if sel.sum() < MIN_IDLE_SAMPLES:
        return None, int(sel.sum())
    return float(np.median(v[sel])), int(sel.sum())


def noload_after(bat, fc_end, window_s=PRE_WINDOW_S, guard_s=PRE_GUARD_S):
    """
    Median no-load voltage in the idle window just after `fc_end`.

    The guard skips the spin-down; note this value still includes some recovery
    (the pack relaxes back toward its true OCV over tens of seconds after a
    load), so an immediately-after reading sits below the fully-rested voltage.
    """
    if bat is None:
        return None, 0
    t, v, _i, idle = bat
    sel = idle & (t > fc_end + guard_s) & (t < fc_end + window_s)
    if sel.sum() < MIN_IDLE_SAMPLES:
        return None, int(sel.sum())
    return float(np.median(v[sel])), int(sel.sum())


def noload_around(bat, fc_start, fc_end, window_s=PRE_WINDOW_S):
    """
    Steady-state no-load voltage before and after a run, and the drop between.

    This is the quantity to compare across runs: both readings are taken with
    every rotor commanded to 1000 us and current near zero, so neither carries
    the throttle-dependent IR sag that contaminates the loaded voltage.
    """
    before, n_b = noload_before(bat, fc_start, window_s)
    after, n_a = noload_after(bat, fc_end, window_s)
    return {
        "before_v": round(before, 3) if before else None,
        "after_v": round(after, 3) if after else None,
        "drop_v": round(before - after, 3) if (before and after) else None,
        "n_before": n_b, "n_after": n_a,
    }


IDLE_PHASES = ("idle_pre", "idle_post")


def noload_from_pwm_map(cols):
    """
    No-load voltage straight from a thrust_map CSV's idle phases.

    Newer runs record `idle_pre` / `idle_post` rows with both rotors commanded
    to 1000 us, so the state of charge is in the Pi's own log and no ulog is
    needed. Returns the same shape as noload_around, or None for older files
    that predate those phases.
    """
    if not cols or "phase" not in cols or "voltage_v" not in cols:
        return None
    phases = list(cols["phase"])
    volts = np.asarray(cols["voltage_v"], dtype=float)

    out = {}
    for name in IDLE_PHASES:
        sel = [i for i, p in enumerate(phases)
               if p == name and np.isfinite(volts[i])]
        key = "before_v" if name == "idle_pre" else "after_v"
        out[key] = round(float(np.median(volts[sel])), 3) if sel else None
        out["n_" + ("before" if name == "idle_pre" else "after")] = len(sel)

    if out["before_v"] is None and out["after_v"] is None:
        return None
    if out["before_v"] and out["after_v"]:
        out["drop_v"] = round(out["before_v"] - out["after_v"], 3)
    else:
        out["drop_v"] = None
    out["v"] = out["before_v"] or out["after_v"]
    out["source"] = "pwm_map idle phases"
    return out


def session_depletion(ulog_path):
    """
    Genuine pack depletion across a log: first vs last no-load voltage.

    Reports the IR sag separately so the transient is visible but never mixed
    into the state-of-charge number.
    """
    bat = load_battery_idle(ulog_path)
    if bat is None:
        return None
    t, v, i, idle = bat
    if idle.sum() < 2 * MIN_IDLE_SAMPLES:
        return None
    vi = v[idle]
    n = max(MIN_IDLE_SAMPLES, idle.sum() // 50)
    start, end = float(np.median(vi[:n])), float(np.median(vi[-n:]))
    loaded = ~idle
    return {
        "noload_start_v": round(start, 3),
        "noload_end_v": round(end, 3),
        "depletion_v": round(start - end, 3),
        "mean_ir_sag_v": (round(float(np.mean(np.interp(t[loaded], t[idle], vi)
                                              - v[loaded])), 3)
                          if loaded.sum() else None),
        "idle_samples": int(idle.sum()),
    }
