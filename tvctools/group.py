"""
Group catalogued files into runs and sessions.

A *run* is one test point: a pwm_map file plus whatever load-cell file overlaps
it in wall-clock time. Pairing is many-to-many by interval overlap, never by
rank -- session 1 has 6 pwm_map files against 4 load-cell files, session 2 has
10 against 9, so any positional pairing would silently misalign.

A *session* is a burst of runs separated from the next by a long idle gap. A
.ulg spans a whole session (the flight controller logs continuously across many
runs), so ulogs attach at session level, not to individual runs.
"""

import os
import re

from .align import overlap_seconds
from .schema import PWM_MAP, LOADCELL, ULOG, epoch_to_local_str

MIN_OVERLAP_FRAC = 0.5      # of the shorter of the two files
SESSION_GAP_S = 30 * 60     # runs more than 30 min apart belong to different sessions


def _slug(text, maxlen=24):
    """Filesystem-safe token from a hand-written label like '1500, 3s'."""
    s = re.sub(r"[^0-9A-Za-z]+", "_", (text or "")).strip("_")
    return s[:maxlen].lower()


def pair_runs(records):
    """
    Build runs from a catalog.

    Each pwm_map file seeds a run and claims every load-cell file that overlaps
    it by more than MIN_OVERLAP_FRAC of the shorter file. Load-cell files that
    match nothing become single-source runs so no data is quietly dropped.
    """
    timed = [r for r in records if r.get("start_epoch") and r.get("end_epoch")]
    pwm_maps = sorted((r for r in timed if r["kind"] == PWM_MAP),
                      key=lambda r: r["start_epoch"])
    loadcells = sorted((r for r in timed if r["kind"] == LOADCELL),
                       key=lambda r: r["start_epoch"])

    claimed = set()
    runs = []
    for pm in pwm_maps:
        members = []
        for lc in loadcells:
            ov = overlap_seconds(pm["start_epoch"], pm["end_epoch"],
                                 lc["start_epoch"], lc["end_epoch"])
            shorter = min(pm["duration_s"] or 1, lc["duration_s"] or 1)
            if shorter > 0 and ov / shorter >= MIN_OVERLAP_FRAC:
                members.append((lc, ov))
        # If several load-cell files overlap, the one sharing the most time wins
        members.sort(key=lambda t: -t[1])
        lc = members[0][0] if members else None
        if lc is not None:
            claimed.add(lc["rel_path"])
        runs.append(_make_run(pm, lc))

    for lc in loadcells:
        if lc["rel_path"] not in claimed:
            runs.append(_make_run(None, lc))

    runs.sort(key=lambda r: r["start_epoch"])
    return runs


def _make_run(pm, lc):
    """Assemble a run dict from an optional pwm_map and optional load-cell file."""
    anchor = pm or lc
    starts = [r["start_epoch"] for r in (pm, lc) if r]
    ends = [r["end_epoch"] for r in (pm, lc) if r]

    flags = []
    if pm is None:
        flags.append("no_pwm_data")     # force/torque only: PWM is unknown
    if lc is None:
        flags.append("no_thrust_data")  # PWM/voltage only: nothing to merge
    for r in (pm, lc):
        if r:
            flags.extend(r.get("flags", []))

    label = _slug(lc["label"] if lc and lc.get("label") else
                  (pm.get("label") if pm else ""))

    return {
        "start_epoch": min(starts),
        "end_epoch": max(ends),
        # The pwm_map span defines the actual test point. One continuous
        # load-cell recording can cover two consecutive Pixhawk sweeps, so the
        # run is named and windowed by the Pixhawk file when there is one.
        "anchor_epoch": pm["start_epoch"] if pm else lc["start_epoch"],
        "window": ((pm["start_epoch"], pm["end_epoch"]) if pm
                   else (lc["start_epoch"], lc["end_epoch"])),
        "start_local": epoch_to_local_str(min(starts)),
        "duration_s": round(max(ends) - min(starts), 1),
        "pwm_map": pm,
        "loadcell": lc,
        "label": label,
        "flags": sorted(set(flags)),
        "name": None,       # assigned by build_sessions once ordering is known
        "session": None,
    }


def run_name(run):
    """
    Self-describing run name: what was commanded, and when.

        A1400_B1000-2000_0051     A held at 1400, B swept 1000->2000, 00:51
        A1850_B1850_1630          both held at 1850, 16:30
        A1400_B1000-2000_0051_1300   trailing token is the hand-written label

    Both rotor commands are in the name because this is a coaxial rig -- thrust
    and especially torque depend on the pair, so a name carrying only one of
    them would not identify the test point.
    """
    pm = run.get("pwm_map")
    hhmm = epoch_to_local_str(run["anchor_epoch"], "%H%M")
    if not pm:
        # Load-cell only: nothing commanded the motor through the Pixhawk
        parts = ["thrustonly", hhmm]
    else:
        parts = [_axis("A", pm.get("a_us_min"), pm.get("a_us_max")),
                 _axis("B", pm.get("b_us_min"), pm.get("b_us_max")),
                 hhmm]
    label = _trim_label(run.get("label"), pm)
    if label:
        parts.append(label)
    return "_".join(p for p in parts if p)


def _trim_label(label, pm):
    """
    Drop label tokens that the name already states.

    The hand-written suffixes were shorthand for the fixed axis ("1300", "1400"),
    which the A/B tokens now carry explicitly. Keeping both would read as
    `A1400_B1000-2000_0104_1400`. Non-numeric notes like "3s" survive, since
    those record something the commands do not.
    """
    if not label or not pm:
        return label or ""
    known = set()
    for key in ("a_us_min", "a_us_max", "b_us_min", "b_us_max"):
        v = pm.get(key)
        if v is not None:
            known.add(str(int(round(v))))
    kept = [tok for tok in label.split("_") if tok and tok not in known]
    return "_".join(kept)


def _axis(prefix, lo, hi):
    """`A1400` when the axis was held, `B1000-2000` when it was swept."""
    if lo is None or hi is None:
        return prefix + "????"
    lo, hi = int(round(lo)), int(round(hi))
    return "%s%d" % (prefix, lo) if lo == hi else "%s%d-%d" % (prefix, lo, hi)


def build_sessions(runs, records):
    """
    Split runs into sessions on idle gaps and attach ulogs by time containment.

    Returns a list of session dicts, each with its ordered runs, its ulogs, and
    stable names (`2026-07-24_s2`, `r03_163010_1300`).
    """
    sessions = []
    current = None
    for run in runs:
        if current is None or run["start_epoch"] - current["end_epoch"] > SESSION_GAP_S:
            current = {"runs": [], "ulogs": [],
                       "start_epoch": run["start_epoch"], "end_epoch": run["end_epoch"]}
            sessions.append(current)
        current["runs"].append(run)
        current["end_epoch"] = max(current["end_epoch"], run["end_epoch"])

    # Name sessions: <date>_s<n>, numbered per day so a date with one session
    # still reads naturally.
    per_day = {}
    for s in sessions:
        day = epoch_to_local_str(s["start_epoch"], "%Y-%m-%d")
        per_day.setdefault(day, []).append(s)
    for day, day_sessions in per_day.items():
        for i, s in enumerate(day_sessions, 1):
            # One folder per date. Only a same-day second session needs a suffix,
            # so the common case reads as a plain date.
            s["name"] = day if len(day_sessions) == 1 else "%s_s%d" % (day, i)
            s["date"] = day

    for s in sessions:
        for i, run in enumerate(s["runs"], 1):
            run["name"] = run_name(run)
            run["session"] = s["name"]

    # A ulog covers a whole session; attach on overlap rather than containment
    # because the FC is usually started before, and stopped after, the runs.
    #
    # ulog_epochs comes from the t_fc_us bridge and is authoritative. The
    # filename is NOT: it can be minutes off (log_3_..00-27-40 really starts at
    # 00:21:42), so a filename-dated log attaches to the wrong session.
    ulogs = [r for r in records if r["kind"] == ULOG]
    for u in sorted(ulogs, key=lambda r: r.get("start_epoch") or 0):
        resolved = u.get("epoch_resolved")
        if resolved:
            u_start, u_end = resolved["start_epoch"], resolved["end_epoch"]
        elif u.get("start_epoch"):
            u_start = u_end = u["start_epoch"]   # filename only; treat as coarse
        else:
            continue
        best, best_ov = None, 0.0
        for s in sessions:
            ov = overlap_seconds(u_start, u_end, s["start_epoch"], s["end_epoch"])
            if ov > best_ov:
                best, best_ov = s, ov
        if best is None:
            # No time overlap (undated log, or the FC ran outside every run):
            # fall back to nearest session start rather than dropping it.
            for s in sessions:
                if s["start_epoch"] - 1800 <= u_start <= s["end_epoch"] + 1800:
                    gap = abs(u_start - s["start_epoch"])
                    if best is None or gap < best_gap:
                        best, best_gap = s, gap
        if best is not None:
            best["ulogs"].append(u)

    return sessions


def format_sessions(sessions):
    """Readable plan of what build would create."""
    lines = []
    for s in sessions:
        lines.append("")
        lines.append("%s  (%s -> %s, %d runs, %d ulogs)" % (
            s["name"],
            epoch_to_local_str(s["start_epoch"], "%H:%M:%S"),
            epoch_to_local_str(s["end_epoch"], "%H:%M:%S"),
            len(s["runs"]), len(s["ulogs"])))
        for u in s["ulogs"]:
            lines.append("    ulog  %s (%.0f MB)" % (u["name"], u.get("size_mb") or 0))
        for run in s["runs"]:
            pm = run["pwm_map"]["name"] if run["pwm_map"] else "-"
            lc = run["loadcell"]["name"] if run["loadcell"] else "-"
            lines.append("    %-22s %6.1fs  pwm=%-32s thrust=%s"
                         % (run["name"], run["duration_s"], pm[:32], lc))
            if run["flags"]:
                lines.append("    %-22s   flags: %s" % ("", ", ".join(run["flags"])))
    return "\n".join(lines)
