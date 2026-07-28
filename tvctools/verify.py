"""
Cross-check Pi thrust_map voltage/current against the ulog battery_status, using
t_fc_us as the shared clock. Both come from the same BATTERY_STATUS field, but
via different paths (MAVLink telemetry vs internal uORB log), so they are never
bit-identical -- this checks they agree, not that they are the same message.
"""
import numpy as np


def compare_source(pm_cols, ulog_path, max_dt_s=0.03):
    from pyulog import ULog

    tf = np.asarray(pm_cols["t_fc_us"], dtype=float) / 1e6
    pv = np.asarray(pm_cols["voltage_v"], dtype=float)
    pc = np.asarray(pm_cols["current_a"], dtype=float)

    u = ULog(ulog_path, ["battery_status"])
    bats = [d for d in u.data_list if d.name == "battery_status"]
    if not bats:
        return None
    b = bats[0]
    ut = np.asarray(b.data["timestamp"], dtype=float) / 1e6
    uv = np.asarray(b.data["voltage_v"], dtype=float)
    uc = np.asarray(b.data["current_a"], dtype=float)

    idx = np.clip(np.searchsorted(ut, tf), 0, len(ut) - 1)
    dt = np.abs(ut[idx] - tf)
    close = dt < max_dt_s
    if close.sum() < 5:
        return None

    dv = pv[close] - uv[idx[close]]
    dc = pc[close] - uc[idx[close]]
    return {
        "n": int(close.sum()),
        "dv_mean": round(float(dv.mean()), 4), "dv_max": round(float(np.abs(dv).max()), 4),
        "dc_mean": round(float(dc.mean()), 4), "dc_max": round(float(np.abs(dc).max()), 4),
    }
