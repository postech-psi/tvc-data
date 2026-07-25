"""
Header-sniffing loader for every file type on the bench.

Kind is decided by the CSV header, never by filename or folder -- one file
(motor test/thrust/1000_F.csv) carries the pwm_map schema despite living in the
thrust folder and being named like a load-cell run.
"""

import csv
import json
import os
import re
import datetime as dt

import numpy as np

from . import LOCAL_TZ_OFFSET_H

# ------------------------------- kinds ------------------------------------- #

PWM_MAP = "pwm_map"
LOADCELL = "loadcell"
ULOG = "ulog"
DERIVED = "derived"   # output of this toolkit, not raw measurement
UNKNOWN = "unknown"

# Written by tvctools.ulog.write_csv and merge.write_merged. Recognized so a
# re-scan classifies them instead of reporting its own output as unknown.
DERIVED_SIGNATURES = (
    {"t_s", "pwm_us", "voltage_v"},        # ulog extract
    {"t_rel_s", "t_epoch", "thrust_N"},    # merged run
)

# Written by pwm_thrust_map.py:CSV_HEADER
PWM_MAP_COLS = [
    "t_epoch", "t_fc_us", "phase", "a_cmd_us", "b_cmd_us", "a_cmd_norm", "b_cmd_norm",
    "servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw",
    "servo5_raw", "servo6_raw", "servo7_raw", "servo8_raw",
    "voltage_v", "current_a", "sweep_idx",
]

# Written by gui.py:_csv_columns_for_mode() -- three modes exist
LOADCELL_MODES = {
    "standard": ["t_ms", "pwm", "rpm", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz",
                 "Current_mA", "ADC_Current_mA"],
    "forces":   ["t_ms", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
    "signals":  ["t_ms", "pwm", "rpm", "Current_mA", "ADC_Current_mA",
                 "CC_raw", "ADC_raw", "ForceCount", "TorqueCount"],
}

# Text columns must not be coerced to float
TEXT_COLS = {"phase"}

# thrust_map_1784819132.1200F.csv -- the extra dot breaks os.path.splitext, and
# the trailing token is a hand-written annotation ("A axis Fixed", B starts 1200)
RE_PWM_MAP_NAME = re.compile(r"^thrust_map_(\d{9,11})(?:\.(.+?))?\.csv$", re.I)
# data_20260724_005059, 1300.csv -- suffix after the timestamp is a manual note
RE_LOADCELL_NAME = re.compile(r"^data_(\d{8})_(\d{6})(.*)\.csv$", re.I)
# PX4 SD card, non-zero-padded month/day, local time. Session number is NOT unique.
RE_ULOG_NAME = re.compile(r"^log_(\d+)_(\d{4})-(\d{1,2})-(\d{1,2})-(\d{2})-(\d{2})-(\d{2})\.ulg$", re.I)


def local_naive_to_epoch(naive):
    """Interpret a naive local-bench datetime (KST) as a UTC epoch."""
    return (naive - dt.timedelta(hours=LOCAL_TZ_OFFSET_H)).replace(
        tzinfo=dt.timezone.utc).timestamp()


def epoch_to_local_str(epoch, fmt="%Y-%m-%d %H:%M:%S"):
    """Format a UTC epoch in bench-local time."""
    return (dt.datetime.fromtimestamp(epoch, dt.timezone.utc)
            + dt.timedelta(hours=LOCAL_TZ_OFFSET_H)).strftime(fmt)


# ------------------------------ sniffing ----------------------------------- #

def read_header(path):
    """First CSV row as a list of stripped names, or None if unreadable."""
    try:
        with open(path, "r", newline="", encoding="utf-8-sig", errors="replace") as f:
            row = next(csv.reader(f), None)
    except OSError:
        return None
    return [h.strip() for h in row] if row else None


def sniff_kind(path):
    """Classify a file by content. Returns (kind, mode_or_None)."""
    if path.lower().endswith(".ulg"):
        return ULOG, None
    header = read_header(path)
    if not header:
        return UNKNOWN, None
    hset = set(header)
    for sig in DERIVED_SIGNATURES:
        if sig <= hset:
            return DERIVED, None
    if {"t_epoch", "a_cmd_us", "b_cmd_us"} <= hset:
        return PWM_MAP, None
    if "t_ms" in hset:
        for mode, cols in LOADCELL_MODES.items():
            if set(cols) == hset:
                return LOADCELL, mode
        # Unrecognized combination but clearly a stand log -- still usable if it
        # carries force data, so don't throw it away.
        return (LOADCELL, "custom") if "Fz" in hset else (UNKNOWN, None)
    return UNKNOWN, None


# ------------------------------- loading ----------------------------------- #

def read_table(path):
    """
    Tolerant CSV -> dict of arrays. Mirrors plot.py:load_csv's handling of blank
    cells and short rows (every load-cell row ends in a bare comma because
    ADC_Current_mA is never populated) but without its pwm/rpm/Fz gate, so it
    also accepts the pwm_map schema and gui.py's "Forces only" mode.
    """
    with open(path, "r", newline="", encoding="utf-8-sig", errors="replace") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return {}
        header = [h.strip() for h in header]
        data = {name: [] for name in header}
        for row in reader:
            if not row:
                continue
            for i, name in enumerate(header):
                cell = row[i].strip() if i < len(row) else ""
                if name in TEXT_COLS:
                    data[name].append(cell)
                elif cell == "":
                    data[name].append(np.nan)
                else:
                    try:
                        data[name].append(float(cell))
                    except ValueError:
                        data[name].append(np.nan)
    out = {}
    for name, vals in data.items():
        out[name] = (np.asarray(vals, dtype=object) if name in TEXT_COLS
                     else np.asarray(vals, dtype=float))
    return out


def time_axis(kind, cols, start_epoch):
    """
    Absolute UTC epoch per sample.

    pwm_map carries a true epoch. The load cell only has free-running MCU uptime,
    so it is anchored on its filename timestamp and advanced by elapsed t_ms --
    this is the coarse anchor that align.py then refines.
    """
    if kind == PWM_MAP:
        return np.asarray(cols["t_epoch"], dtype=float)
    t_ms = np.asarray(cols["t_ms"], dtype=float)
    return start_epoch + (t_ms - t_ms[0]) / 1000.0


# --------------------------- filename parsing ------------------------------ #

def parse_name(path):
    """
    Extract (kind_hint, start_epoch, label) from the filename.

    Returns start_epoch=None when the name carries no timestamp. Never use mtime
    for the root thrust_map_*.csv files -- they are git-tracked and all share a
    single checkout mtime.
    """
    name = os.path.basename(path)

    m = RE_PWM_MAP_NAME.match(name)
    if m:
        return PWM_MAP, float(m.group(1)), (m.group(2) or "")

    m = RE_LOADCELL_NAME.match(name)
    if m:
        stamp = dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
        label = m.group(3).strip(" ,_")
        return LOADCELL, local_naive_to_epoch(stamp), label

    m = RE_ULOG_NAME.match(name)
    if m:
        _, y, mo, d, hh, mm, ss = m.groups()
        stamp = dt.datetime(int(y), int(mo), int(d), int(hh), int(mm), int(ss))
        return ULOG, local_naive_to_epoch(stamp), "log_%s" % m.group(1)

    return UNKNOWN, None, ""


def read_sidecar(path):
    """
    Load the <stem>.run.json companion emitted next to a CSV, if present.

    Older files predate the sidecar, so its absence is normal, not an error.
    """
    side = os.path.splitext(path)[0] + ".run.json"
    if not os.path.exists(side):
        return None
    try:
        with open(side, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_any(path):
    """
    Load any bench file.

    Returns (kind, cols, meta). meta always has: mode, label, start_epoch,
    end_epoch, n_rows, rate_hz, flags. ULOG files are catalogued but not parsed
    here -- see tvctools.ulog (pyulog is slow, so it stays opt-in).
    """
    kind, mode = sniff_kind(path)
    name_kind, name_epoch, label = parse_name(path)

    meta = {
        "path": path,
        "name": os.path.basename(path),
        "kind": kind,
        "mode": mode,
        "label": label,
        "start_epoch": name_epoch,
        "end_epoch": None,
        "n_rows": 0,
        "rate_hz": None,
        "flags": [],
    }

    if kind in (UNKNOWN, DERIVED):
        return kind, {}, meta
    if name_kind == UNKNOWN:
        # Hand-renamed file (e.g. 1000_F.csv, which carries the pwm_map schema
        # despite sitting in the load-cell folder). Usable, but its run time has
        # to come from the contents rather than the name.
        meta["flags"].append("nonstandard_name")
    elif kind != UNKNOWN and name_kind != kind:
        meta["flags"].append("schema_mismatch")
    if kind == ULOG:
        return kind, {}, meta

    cols = read_table(path)
    if not cols:
        meta["flags"].append("empty")
        return kind, cols, meta

    n = len(next(iter(cols.values())))
    meta["n_rows"] = n
    if n < 2:
        meta["flags"].append("too_short")
        return kind, cols, meta

    # pwm_map files carry their own epoch, which beats the filename
    if kind == PWM_MAP:
        meta["start_epoch"] = float(cols["t_epoch"][0])
    elif meta["start_epoch"] is None:
        meta["flags"].append("no_timestamp")
        return kind, cols, meta

    sidecar = read_sidecar(path)
    if sidecar:
        # Written by pwm_thrust_map.py: the grid/dwell/repeats settings and any
        # prop/battery notes, none of which survive in the CSV itself.
        meta["run_cfg"] = sidecar
        for key in ("prop", "battery", "notes"):
            if sidecar.get(key):
                meta[key] = sidecar[key]

    t = time_axis(kind, cols, meta["start_epoch"])
    meta["end_epoch"] = float(t[-1])
    span = t[-1] - t[0]
    meta["rate_hz"] = round(n / span, 2) if span > 0 else None
    meta.update(_describe(kind, cols, meta))
    return kind, cols, meta


def _finite(a):
    a = np.asarray(a, dtype=float)
    return a[np.isfinite(a)]


def _rng(cols, key):
    """(min, max) of a column, or (None, None) if absent/all-NaN."""
    if key not in cols:
        return None, None
    v = _finite(cols[key])
    return (float(v.min()), float(v.max())) if v.size else (None, None)


def _describe(kind, cols, meta):
    """Kind-specific summary fields plus data-quality flags."""
    d, flags = {}, meta["flags"]

    if kind == PWM_MAP:
        phases = [p for p in cols.get("phase", []) if p]
        d["phases"] = sorted(set(phases))
        d["n_phases"] = len(d["phases"])
        sw = _finite(cols.get("sweep_idx", []))
        d["sweeps"] = int(sw.max()) if sw.size else None
        for key, out in (("a_cmd_us", "a_us"), ("b_cmd_us", "b_us"),
                         ("voltage_v", "voltage"), ("current_a", "current")):
            lo, hi = _rng(cols, key)
            d[out + "_min"], d[out + "_max"] = lo, hi
        if not _finite(cols.get("current_a", [])).size:
            flags.append("no_battery_data")

    elif kind == LOADCELL:
        for key, out in (("Fz", "fz"), ("Tz", "tz")):
            lo, hi = _rng(cols, key)
            d[out + "_min"], d[out + "_max"] = lo, hi
        # Thrust is -Fz (the stand logs vertical load signed negative)
        if d.get("fz_min") is not None:
            d["thrust_max_n"] = -d["fz_min"]
        pwm = _finite(cols.get("pwm", []))
        rpm = _finite(cols.get("rpm", []))
        cur = _finite(cols.get("Current_mA", []))
        d["pwm_levels"] = int(np.unique(pwm).size) if pwm.size else 0
        # The stand's own pwm/rpm/current columns are vestigial in the current
        # architecture: the Pi commands the motor through the Pixhawk, so the
        # STM32 never sees the throttle, and voltage/current come from the
        # Pixhawk's battery monitor. Empty here is EXPECTED, not a fault -- the
        # "gui_*_unused" prefix marks it as by-design so nobody treats these
        # files as broken or tries to aggregate against a constant column.
        if pwm.size and np.unique(pwm).size <= 1:
            flags.append("gui_pwm_unused")
        if rpm.size and np.nanmax(rpm) <= 0:
            flags.append("gui_rpm_unused")
        if cur.size and np.nanmax(cur) <= 0:
            flags.append("gui_current_unused")

    return d
