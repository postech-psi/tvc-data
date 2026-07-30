"""
Batch .ulg -> .csv, one CSV per log, nothing dropped.

Drop the logs in convert/ulog/ and run this; every one of them lands in
convert/csv/ as a single wide table -- the same shape as a run's merged.csv,
extended to every topic the log contains:

    t_s,timestamp,actuator_outputs.output[0],battery_status.voltage_v,...
    0.0,1784878912001170,1000,,
    0.01,1784878912011170,,16.72,

One row per recorded message, in time order, with only that message's columns
filled. Nothing is resampled, interpolated, rounded or forward-filled, so the
CSV holds exactly what the log held -- a blank cell means "no sample at this
instant", never "value unknown". That fidelity costs width: a full log is
hundreds of columns and millions of mostly-empty rows, and a 120 MB .ulg can
turn into several GB of CSV. Use --list to see what is in a log and --topics to
take only the streams you need.

    python convert/convert.py                  # everything in convert/ulog/
    python convert/convert.py --list           # topics and message counts only
    python convert/convert.py --topics actuator_outputs,battery_status
"""

import argparse
import csv
import heapq
import itertools
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IN_DIR = os.path.join(HERE, "ulog")
OUT_DIR = os.path.join(HERE, "csv")

TIME_FIELD = "timestamp"  # every PX4 topic carries it; it becomes the shared axis


def load_ulog(path, topics=None):
    """Parse a .ulg, optionally only the named topics (much faster and smaller)."""
    try:
        from pyulog import ULog
    except ImportError:
        raise SystemExit(
            "pyulog is required to read .ulg files.\n"
            "    pip install pyulog")
    try:
        return ULog(path, message_name_filter_list=topics)
    except TypeError:
        # Older pyulog takes the filter positionally
        return ULog(path, topics)


def find_ulogs(indir):
    """Every .ulg under indir, as paths relative to it, sorted."""
    found = []
    for dirpath, _dirnames, filenames in os.walk(indir):
        for name in filenames:
            if name.lower().endswith(".ulg"):
                full = os.path.join(dirpath, name)
                found.append(os.path.relpath(full, indir))
    return sorted(found)


def _prefix(data):
    """Column prefix for one topic instance: multi_id 0 stays unadorned."""
    return data.name if data.multi_id == 0 else "%s_%d" % (data.name, data.multi_id)


def _field_names(data):
    """Field order as the log defines it, timestamp excluded (it is the axis)."""
    fields = None
    if getattr(data, "field_data", None):
        fields = [f.field_name for f in data.field_data if f.field_name in data.data]
    if not fields:
        # dict order is definition order on py3.7+, so this matches field_data
        fields = list(data.data.keys())
    return [f for f in fields if f != TIME_FIELD]


def build_columns(ulog, topics=None):
    """
    Fix the CSV schema up front.

    Returns (columns, instances). Each instance carries the arrays for one
    topic instance plus the column index its fields write into, so the row
    writer never has to look anything up by name.
    """
    columns = ["t_s", TIME_FIELD]
    instances = []
    wanted = set(topics) if topics else None

    for data in sorted(ulog.data_list, key=lambda d: (d.name, d.multi_id)):
        if wanted is not None and data.name not in wanted:
            continue
        if TIME_FIELD not in data.data:
            # Without a timestamp there is no row to put it on
            print("  warning: %s has no %s, skipped" % (data.name, TIME_FIELD))
            continue
        fields = _field_names(data)
        if not fields:
            continue
        ts = np.asarray(data.data[TIME_FIELD])
        instances.append({
            "prefix": _prefix(data),
            "fields": fields,
            "values": [data.data[f] for f in fields],
            "ts": ts,
            # Log order is already time order, but a stable argsort costs little
            # and guarantees the merge below sees sorted streams.
            "order": np.argsort(ts, kind="stable"),
            "col0": len(columns),
            "n": len(ts),
        })
        columns.extend("%s.%s" % (instances[-1]["prefix"], f) for f in fields)
    return columns, instances


def _fmt(v):
    """
    The value as text, round-tripping exactly, without scientific noise.

    format_float_positional with unique=True gives the shortest string that
    reads back as the same float32/float64 -- 16.72, not 16.719999313354492.
    NaN is written out rather than blanked: the log recorded a NaN there, and a
    blank cell already means something else (no sample at all).
    """
    if isinstance(v, (float, np.floating)):
        if np.isnan(v):
            return "nan"
        if np.isinf(v):
            return "inf" if v > 0 else "-inf"
        if v != 0 and not (1e-10 <= abs(v) < 1e16):
            return np.format_float_scientific(v, unique=True, trim="-")
        return np.format_float_positional(v, unique=True, trim="-")
    return str(v)


def _fmt_t(us):
    """Seconds from log start, exact: the source is integer microseconds."""
    s = "%.6f" % (us / 1e6)
    return s.rstrip("0").rstrip(".") if "." in s else s


def _stream(inst, k):
    """(timestamp, instance, sample index) for one topic, in time order."""
    ts = inst["ts"]
    for j in inst["order"]:
        yield int(ts[j]), k, int(j)


def iter_rows(ulog, instances, n_cols):
    """
    Yield every row of the table, in time order, one message at a time.

    Rows are built streaming rather than collected: a full log is millions of
    rows wide enough that holding the table would cost more memory than the log
    itself. Messages sharing a timestamp share a row, except when the same topic
    repeats within that timestamp -- then the repeat opens another row, so no
    sample is ever overwritten by another.
    """
    t0 = ulog.start_timestamp
    streams = [_stream(inst, k) for k, inst in enumerate(instances)]

    for ts, group in itertools.groupby(heapq.merge(*streams), key=lambda e: e[0]):
        rows = []
        slots = {}
        for _ts, k, j in group:
            inst = instances[k]
            slot = slots.get(k, 0)
            slots[k] = slot + 1
            while len(rows) <= slot:
                rows.append([""] * n_cols)
            row = rows[slot]
            col = inst["col0"]
            for values in inst["values"]:
                row[col] = _fmt(values[j])
                col += 1
        t_s = _fmt_t(ts - t0)
        for row in rows:
            row[0] = t_s
            row[1] = ts
            yield row


def write_wide_csv(ulog, path, topics=None):
    """Write one log to one CSV. Takes a parsed ULog, not a path."""
    columns, instances = build_columns(ulog, topics)
    n_rows = 0
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(columns)
        for row in iter_rows(ulog, instances, len(columns)):
            w.writerow(row)
            n_rows += 1
    return {"path": path, "n_topics": len(instances), "n_cols": len(columns),
            "n_rows": n_rows, "n_msgs": sum(i["n"] for i in instances)}


def describe(ulog, topics=None):
    """What is in this log: one line per topic instance, biggest stream first."""
    _columns, instances = build_columns(ulog, topics)
    lines = []
    for inst in sorted(instances, key=lambda i: -i["n"]):
        lines.append("    %-40s %9d msgs %4d fields"
                     % (inst["prefix"], inst["n"], len(inst["fields"])))
    lines.append("    %-40s %9d msgs %4d columns"
                 % ("TOTAL (%d topics)" % len(instances),
                    sum(i["n"] for i in instances),
                    sum(len(i["fields"]) for i in instances) + 2))
    return "\n".join(lines)


def _human(n):
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024 or unit == "TB":
            return "%.1f %s" % (n, unit)
        n /= 1024.0


def _up_to_date(src, dst):
    return os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src)


def convert_one(path, outpath, topics=None):
    """Parse one .ulg and write its CSV. Returns the write_wide_csv stats."""
    ulog = load_ulog(path, topics)
    duration = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    print("%s  (%s, %.1f s)"
          % (os.path.basename(path), _human(os.path.getsize(path)), duration))
    stats = write_wide_csv(ulog, outpath, topics)
    print("  %d topics, %d columns, %d rows -> %s (%s)"
          % (stats["n_topics"], stats["n_cols"], stats["n_rows"], outpath,
             _human(os.path.getsize(outpath))))
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Convert every .ulg in convert/ulog/ to one CSV each in convert/csv/")
    ap.add_argument("--in", dest="indir", default=IN_DIR,
                    help="folder holding the .ulg originals (default: convert/ulog)")
    ap.add_argument("--out", dest="outdir", default=OUT_DIR,
                    help="where the CSVs go (default: convert/csv)")
    ap.add_argument("--topics", help="comma-separated topics to keep (default: all)")
    ap.add_argument("--list", action="store_true",
                    help="print each log's topics and message counts, convert nothing")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would be written without reading the logs")
    ap.add_argument("--force", action="store_true",
                    help="reconvert even when the CSV is newer than the .ulg")
    args = ap.parse_args(argv)

    topics = [t.strip() for t in args.topics.split(",") if t.strip()] if args.topics else None

    if not os.path.isdir(args.indir):
        print("No input folder %s -- create it and drop .ulg files in." % args.indir)
        return 1
    rels = find_ulogs(args.indir)
    if not rels:
        print("No .ulg files under %s" % args.indir)
        return 0

    n_done = n_skipped = 0
    failed = []
    for rel in rels:
        src = os.path.join(args.indir, rel)
        dst = os.path.join(args.outdir, os.path.splitext(rel)[0] + ".csv")

        if args.list:
            try:
                print("%s  (%s)" % (rel, _human(os.path.getsize(src))))
                print(describe(load_ulog(src, topics), topics))
            except Exception as e:
                failed.append((rel, e))
                print("  FAILED: %s" % e)
            continue

        if args.dry_run:
            print("%s -> %s%s"
                  % (rel, dst, "" if args.force or not _up_to_date(src, dst)
                     else "  (would skip, up to date)"))
            continue

        if not args.force and _up_to_date(src, dst):
            print("%s  skipped (up to date)" % rel)
            n_skipped += 1
            continue

        # One bad log must not take the batch down with it
        try:
            convert_one(src, dst, topics)
            n_done += 1
        except Exception as e:
            failed.append((rel, e))
            print("  FAILED: %s" % e)

    if args.dry_run:
        print("\n(dry run -- nothing written. Re-run without --dry-run to convert.)")
        return 0
    if not args.list:
        print("\n%d logs, %d converted, %d skipped" % (len(rels), n_done, n_skipped))
    if failed:
        print("%d failed:" % len(failed))
        for rel, e in failed:
            print("  %-46s %s" % (rel[:46], e))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
