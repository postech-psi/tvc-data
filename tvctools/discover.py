"""
Walk the data tree and catalog every bench file.

Produces runs_index.csv / runs_index.json: one row per file, with its true run
time, duration, sample rate, kind-specific summary and data-quality flags.
"""

import csv
import json
import os

from .schema import (PWM_MAP, LOADCELL, ULOG, DERIVED, UNKNOWN,
                     load_any, parse_name, epoch_to_local_str)

# Never descend into generated output, VCS internals or caches
SKIP_DIRS = {".git", "__pycache__", "runs", "out", ".venv", "venv", "node_modules"}
SKIP_FILES = {"runs_index.csv",              # this tool's own catalog
              "pwm_thrust_torque_map.csv",   # and its own analysis products
              "voltage_sag.csv"}
DATA_EXT = {".csv", ".ulg"}

INDEX_FIELDS = [
    "name", "rel_path", "kind", "mode", "label",
    "start_local", "end_local", "start_epoch", "end_epoch", "duration_s",
    "n_rows", "rate_hz", "size_mb", "flags",
    # pwm_map
    "n_phases", "sweeps", "a_us_min", "a_us_max", "b_us_min", "b_us_max",
    "voltage_min", "voltage_max", "current_min", "current_max",
    # loadcell
    "thrust_max_n", "fz_min", "fz_max", "tz_min", "tz_max", "pwm_levels",
]


def iter_data_files(root):
    """Yield every .csv/.ulg path under root, skipping generated/VCS folders."""
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in sorted(filenames):
            if fn in SKIP_FILES:
                continue
            if os.path.splitext(fn)[1].lower() in DATA_EXT:
                yield os.path.join(dirpath, fn)


def scan(root, verbose=False):
    """Classify every data file under root. Returns a list of meta dicts."""
    records = []
    for path in iter_data_files(root):
        try:
            kind, _cols, meta = load_any(path)
        except Exception as exc:  # a corrupt file must not abort the whole scan
            kind = UNKNOWN
            meta = {"path": path, "name": os.path.basename(path), "kind": UNKNOWN,
                    "mode": None, "label": "", "start_epoch": None, "end_epoch": None,
                    "n_rows": 0, "rate_hz": None, "flags": ["read_error: %s" % exc]}

        if kind == ULOG:
            # Size and filename time only -- parsing 194 MB of ulog here would
            # make `index` unusably slow. tvctools.ulog does the real work.
            _, epoch, label = parse_name(path)
            meta["start_epoch"], meta["label"] = epoch, label

        meta["rel_path"] = os.path.relpath(path, root).replace("\\", "/")
        meta["size_mb"] = round(os.path.getsize(path) / 1e6, 2)
        if meta.get("start_epoch") and meta.get("end_epoch"):
            meta["duration_s"] = round(meta["end_epoch"] - meta["start_epoch"], 1)
        else:
            meta["duration_s"] = None
        meta["start_local"] = (epoch_to_local_str(meta["start_epoch"])
                               if meta.get("start_epoch") else "")
        meta["end_local"] = (epoch_to_local_str(meta["end_epoch"])
                             if meta.get("end_epoch") else "")
        records.append(meta)
        if verbose:
            print("  %-46s %-9s %s" % (meta["name"][:46], kind, meta["start_local"]))

    records.sort(key=lambda m: (m.get("start_epoch") or 0, m["name"]))
    return records


def write_index(records, root):
    """Write runs_index.csv and runs_index.json. Returns both paths."""
    out_dir = os.path.join(root, "out")
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "runs_index.csv")
    json_path = os.path.join(out_dir, "runs_index.json")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=INDEX_FIELDS, extrasaction="ignore")
        w.writeheader()
        for m in records:
            row = dict(m)
            row["flags"] = ";".join(m.get("flags", []))
            w.writerow(row)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump([{k: v for k, v in m.items() if k != "path"} for m in records],
                  f, indent=2, ensure_ascii=False, default=str)

    return csv_path, json_path


def summarize(records):
    """Human-readable counts by kind plus every flagged file."""
    by_kind = {}
    for m in records:
        by_kind.setdefault(m["kind"], []).append(m)

    lines = ["", "Catalogued %d files:" % len(records)]
    for kind in (PWM_MAP, LOADCELL, ULOG, DERIVED, UNKNOWN):
        group = by_kind.get(kind, [])
        if group:
            mb = sum(m.get("size_mb") or 0 for m in group)
            lines.append("  %-9s %3d files  %8.1f MB" % (kind, len(group), mb))

    # "gui_*_unused" is expected in the Pi-driven architecture (the stand never
    # sees throttle or battery), so it is reported separately from real problems.
    def _expected(m):
        return all(f.startswith("gui_") for f in m["flags"])

    problems = [m for m in records if m.get("flags") and not _expected(m)]
    expected = [m for m in records if m.get("flags") and _expected(m)]

    if problems:
        lines.append("")
        lines.append("Needs attention (%d files):" % len(problems))
        for m in problems:
            lines.append("  %-46s %s" % (m["name"][:46], ", ".join(m["flags"])))
    if expected:
        lines.append("")
        lines.append("By design -- stand's pwm/rpm/current unused, Pi drives the "
                     "motor (%d files)" % len(expected))
    return "\n".join(lines)
