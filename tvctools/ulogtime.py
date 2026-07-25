"""
Put a .ulg on the wall clock.

A PX4 log has no trustworthy absolute time on this bench:

- `time_ref_utc` is 0 in every log -- no GPS time reference.
- The filename is wrong. `log_3_2026-7-24-00-27-40.ulg` actually starts at
  00:21:42 local, six minutes earlier than its name claims.
- The `/fs/microsd/log/...` path in the logged messages records the *first* log
  of the boot session, not the file it appears in, so `log_1_..01-07-24` and
  `log_1_..01-16-18` both report the same internal time.

What the log does have is the flight controller's boot clock. And
`thrust_map_*.csv` records both `t_fc_us` (that same boot clock, from
SERVO_OUTPUT_RAW.time_usec) and `t_epoch` (the Pi's wall clock) on every row.

That pairing is the only bridge between the two. Median `t_epoch - t_fc_us`
over a sweep gives the boot-to-epoch offset, which then dates every sample in
any ulog from the same boot session.

This is the reason `thrust_map_*.csv` cannot be discarded in favour of the ulog
even though the PWM and voltage values are duplicated: delete it and nothing
timestamps the ulog, so it can never be aligned with the load cell.
"""

import os

import numpy as np

from .schema import PWM_MAP, load_any

# A pwm_map belongs to the same boot as a ulog only if its FC-clock window sits
# inside the ulog's, with a little slack for the rows logged either side.
BOOT_SLACK_S = 5.0


MIN_BOOT_MATCH_CORR = 0.9   # PWM traces must genuinely coincide, not merely overlap


def ulog_outputs(path):
    """
    (t_fc_seconds, {channel: values}) from actuator_outputs, or None.

    The timestamps are the flight controller's boot clock, which restarts from
    zero on every power cycle -- so these numbers alone cannot identify *which*
    boot a log came from. The channel values are returned precisely so a
    candidate match can be confirmed against the recorded PWM.
    """
    try:
        from pyulog import ULog
        ulog = ULog(path, ["actuator_outputs"])
    except Exception:
        return None
    streams = [d for d in ulog.data_list
               if d.name == "actuator_outputs" and d.multi_id == 0]
    if not streams:
        return None
    d = streams[0]
    t = np.asarray(d.data["timestamp"], dtype=float) / 1e6
    if t.size == 0:
        return None
    chans = {}
    for i in range(8):
        key = "output[%d]" % i
        if key in d.data:
            chans[i] = np.asarray(d.data[key], dtype=float)
    return t, chans


def boot_match_score(u_t, u_chans, fc, servo):
    """
    How well a thrust_map's measured PWM matches the ulog over the same FC times.

    Both recorded the same physical outputs, so if they come from the same boot
    the traces coincide; if the FC windows merely happen to overlap numerically
    (different power cycles) they do not. Returns the best correlation found
    across channel pairings, or -1.0 when there is nothing to compare.
    """
    inside = (fc >= u_t[0]) & (fc <= u_t[-1])
    if inside.sum() < 20:
        return -1.0
    best = -1.0
    for u_ch in u_chans.values():
        resampled = np.interp(fc[inside], u_t, u_ch)
        for s_ch in servo:
            s = s_ch[inside]
            if not (np.isfinite(s).all() and np.isfinite(resampled).all()):
                continue
            if np.std(s) < 1e-6 or np.std(resampled) < 1e-6:
                # A constant hold has no shape to correlate, and matching on the
                # PWM *level* is worthless as a discriminator: an idle 1000 us
                # matches every quiet stretch of every log. Such runs are located
                # by epoch containment instead, once a varying sweep has fixed
                # the boot offset -- see resolve_ulog_epochs.
                continue
            c = float(np.corrcoef(resampled, s)[0, 1])
            if np.isfinite(c) and c > best:
                best = c
    return best


def fc_to_epoch_offset(pwm_map_path):
    """
    Seconds to add to an FC boot timestamp to get a UTC epoch.

    Returns a dict with the offset, the FC window it was measured over, the
    scatter of the per-row offsets (a healthy link is tens of milliseconds) and
    the measured servo traces used to confirm which boot it belongs to.
    """
    kind, cols, _meta = load_any(pwm_map_path)
    if kind != PWM_MAP or "t_fc_us" not in cols or "t_epoch" not in cols:
        return None
    fc = np.asarray(cols["t_fc_us"], dtype=float) / 1e6
    ep = np.asarray(cols["t_epoch"], dtype=float)
    good = np.isfinite(fc) & np.isfinite(ep) & (fc > 0)
    if good.sum() < 10:
        return None
    servo = []
    for i in (1, 2):
        key = "servo%d_raw" % i
        if key in cols:
            servo.append(np.asarray(cols[key], dtype=float)[good])
    fc, ep = fc[good], ep[good]
    deltas = ep - fc
    return {
        "offset": float(np.median(deltas)),
        "fc0": float(fc.min()), "fc1": float(fc.max()),
        "spread": float(np.percentile(deltas, 95) - np.percentile(deltas, 5)),
        "fc": fc, "servo": servo,
    }


def resolve_ulog_epochs(ulog_records, pwm_map_records, root):
    """
    Date every ulog using the thrust_map files that share its boot session.

    Returns {ulog rel_path: dict(start_epoch, end_epoch, offset, source, spread)}
    for the logs that could be dated. Logs from a boot with no thrust_map -- and
    on this data that includes the two with no motor activity at all -- stay
    undated, which is reported rather than guessed.
    """
    bridges = []
    for rec in pwm_map_records:
        got = fc_to_epoch_offset(os.path.join(root, rec["rel_path"]))
        if got:
            got["name"] = rec["name"]
            bridges.append(got)

    out = {}
    signatures = {}
    for rec in ulog_records:
        got = ulog_outputs(os.path.join(root, rec["rel_path"]))
        if not got:
            continue
        u_t, u_chans = got
        u0, u1 = float(u_t[0]), float(u_t[-1])

        # A single log spans a whole boot and therefore usually covers SEVERAL
        # runs. Collect every sweep that matches, not just the best one: each
        # match contributes an independent estimate of the same offset, and the
        # set of matches is exactly the list of runs inside this log.
        matches = []
        for b in bridges:
            if not (u0 - BOOT_SLACK_S <= b["fc0"] and b["fc1"] <= u1 + BOOT_SLACK_S):
                continue
            # Time containment alone proves nothing -- every boot restarts the FC
            # clock near zero, so unrelated logs overlap numerically. The PWM
            # traces must actually coincide.
            score = boot_match_score(u_t, u_chans, b["fc"], b["servo"])
            if score >= MIN_BOOT_MATCH_CORR:
                matches.append((score, b))
        if not matches:
            continue

        offsets = [b["offset"] for _s, b in matches]
        best_score, best = max(matches, key=lambda m: m[0])
        entry = {
            "start_epoch": u0 + best["offset"],
            "end_epoch": u1 + best["offset"],
            "offset": best["offset"],
            "clock_spread_s": round(best["spread"], 3),
            "boot_match_corr": round(best_score, 4),
            "source": best["name"],
            "fc_window": [u0, u1],
            # Every sweep found inside this log, with where it sits in FC time
            "anchored_by": sorted(
                {b["name"]: {"pwm_map": b["name"],
                             "fc_start": round(b["fc0"], 2),
                             "fc_end": round(b["fc1"], 2),
                             "corr": round(s, 4)}
                 for s, b in matches}.values(),
                key=lambda d: d["fc_start"]),
            # Independent offsets from independent sweeps should agree closely;
            # a wide spread means the match set is not really one boot.
            "offset_agreement_s": round(max(offsets) - min(offsets), 3),
        }
        # Two downloads of the same boot: one file is a prefix of the other.
        # Keep the longer one as canonical and mark the truncated copy.
        sig = (round(u0, 2), tuple(np.round(u_chans.get(1, u_t)[:200], 3)))
        prev = signatures.get(sig)
        if prev is None:
            signatures[sig] = (rec["name"], rec["rel_path"], u1 - u0)
        else:
            prev_name, prev_rel, prev_span = prev
            if (u1 - u0) > prev_span:
                out[prev_rel]["duplicate_of"] = rec["name"]   # older, shorter copy
                signatures[sig] = (rec["name"], rec["rel_path"], u1 - u0)
            else:
                entry["duplicate_of"] = prev_name
        out[rec["rel_path"]] = entry

    # With the boot offset fixed by a varying sweep, the log's wall-clock window
    # is known -- so every run inside that window is covered, including the
    # constant holds that carry no correlatable shape.
    for rel, entry in out.items():
        covers = []
        for rec in pwm_map_records:
            s, e = rec.get("start_epoch"), rec.get("end_epoch")
            if s is None or e is None:
                continue
            if entry["start_epoch"] - BOOT_SLACK_S <= s and e <= entry["end_epoch"] + BOOT_SLACK_S:
                anchored = next((a for a in entry["anchored_by"]
                                 if a["pwm_map"] == rec["name"]), None)
                covers.append({
                    "pwm_map": rec["name"],
                    "start_local": rec.get("start_local"),
                    "fc_start": round(s - entry["offset"], 2),
                    "fc_end": round(e - entry["offset"], 2),
                    "how": "pwm_correlation" if anchored else "epoch_containment",
                })
        entry["covers"] = sorted(covers, key=lambda d: d["fc_start"])
    return out
