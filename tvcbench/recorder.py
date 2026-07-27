"""
Writing a run to disk.

Layout, one directory per run:

    bench/<run_id>/
      manifest.json   plan, seed, clock anchors and fits, tare, rates, outcome
      sequence.csv    one row per segment: planned command, actual start/end
      events.jsonl    every transition, command send, warning and abort
      loadcell.csv    t_mono, dev_t, seg_id, Fx..Tz raw and tared
      fc_servo.csv    t_mono, dev_t, seg_id, servo1..8
      fc_battery.csv  t_mono, seg_id, voltage, current
      fc_esc.csv      t_mono, dev_t, seg_id, rpm/voltage/current

Two decisions worth stating.

**The data files are append-only and carry raw clock values.** They do not carry
a derived `t_epoch` column, because that would have to be written either before
the clock fit exists or by rewriting every file at the end. Append-only means a
run killed mid-way still leaves valid, complete-up-to-that-point data; the fit
coefficients and the epoch anchors live in the manifest, and `tvcbench.reader`
applies them. Crash safety is worth more than a pre-computed column.

**Nothing is joined here.** Each stream is written at its own rate with no
carry-forward, which is the defect this whole design exists to remove.
"""

import csv
import json
import os
import platform
import socket
import subprocess
import time

from tvcbench import SCHEMA_VERSION

#: Columns the recorder prepends to every stream. `dev_t` is the device's own
#: counter, kept raw so the clock fit in the manifest is the only interpretation.
CLOCK_COLUMNS = ("t_mono", "dev_t", "seg_id")

SEQUENCE_COLUMNS = (
    "seg_id", "kind", "a_us", "b_us", "sweep",
    "planned_dwell_s", "min_dwell_s", "max_dwell_s",
    "t_mono_start", "t_mono_end", "actual_dwell_s",
    "n_loadcell", "dwell_reason", "thrust_mean_n", "thrust_sem_n",
)

PART_SUFFIX = ".part"

#: Decimal places kept for floats. Six is microseconds on `t_mono` and micronewtons
#: on force -- both several orders below anything the instruments resolve -- and it
#: cuts the file size roughly threefold against full repr precision.
FLOAT_DECIMALS = 6


def _cell(value):
    """Render one CSV cell: blanks for missing, bounded precision for floats."""
    if value is None:
        return ""
    if isinstance(value, float):
        return round(value, FLOAT_DECIMALS)
    return value


def make_run_id(t_epoch=None):
    """Local-time run id: sorts chronologically and reads as a wall-clock time."""
    t_epoch = time.time() if t_epoch is None else t_epoch
    return time.strftime("%Y-%m-%d_%H%M%S", time.localtime(t_epoch))


def git_revision(cwd="."):
    """Current commit, so a run can be tied to the code that took it."""
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=cwd, timeout=5,
                             capture_output=True, text=True, check=True)
        return out.stdout.strip()
    except Exception:                        # noqa: BLE001 - not worth failing a run
        return None


class Recorder:
    """Owns every file a run produces."""

    def __init__(self, root, run_id, streams, plan=None):
        self.root = root
        self.run_id = run_id
        self.streams = dict(streams)
        self.plan = plan

        self.part_dir = os.path.join(root, run_id + PART_SUFFIX)
        self.final_dir = os.path.join(root, run_id)

        self._files = {}
        self._writers = {}
        self._rows = {name: 0 for name in self.streams}
        self._events = None
        self._segments = []
        self.warnings = []

    # --- lifecycle ----------------------------------------------------------
    def open(self):
        os.makedirs(self.part_dir, exist_ok=True)
        for name, columns in self.streams.items():
            path = os.path.join(self.part_dir, f"{name}.csv")
            fp = open(path, "w", newline="", encoding="utf-8")
            writer = csv.writer(fp)
            writer.writerow(list(CLOCK_COLUMNS) + list(columns))
            self._files[name] = fp
            self._writers[name] = writer
        self._events = open(os.path.join(self.part_dir, "events.jsonl"), "w",
                            encoding="utf-8")
        return self

    def close(self):
        for fp in list(self._files.values()) + ([self._events] if self._events else []):
            try:
                fp.flush()
                fp.close()
            except Exception:                # noqa: BLE001
                pass
        self._files.clear()
        self._writers.clear()
        self._events = None

    # --- data ---------------------------------------------------------------
    def write_samples(self, samples, seg_id):
        """Route samples to their stream's file. Unknown streams are dropped loudly."""
        for s in samples:
            writer = self._writers.get(s.stream)
            if writer is None:
                self.warn(f"sample for unknown stream {s.stream!r}")
                continue
            columns = self.streams[s.stream]
            row = [_cell(s.t_mono), _cell(s.dev_t), seg_id]
            row += [_cell(s.fields.get(c)) for c in columns]
            writer.writerow(row)
            self._rows[s.stream] += 1

    def flush(self, sync=False):
        """
        Flush buffered rows; optionally force them to the medium.

        Called at every segment boundary. `fsync` costs a few milliseconds and is
        taken between segments rather than during one, so it cannot perturb the
        timing of a step that is being measured.
        """
        for fp in self._files.values():
            fp.flush()
            if sync:
                os.fsync(fp.fileno())
        if self._events:
            self._events.flush()

    # --- events -------------------------------------------------------------
    def event(self, kind, /, *, t_mono=None, **fields):
        """
        Append one structured event. Never raises -- logging must not kill a run.

        `kind` is positional-only so that a caller passing `kind=` as *data*
        lands in `fields` and gets filtered, rather than raising a TypeError deep
        inside a run. Payload keys never overwrite the event's own.
        """
        from tvcbench.clock import now

        record = {"t_mono": now() if t_mono is None else t_mono, "kind": kind}
        record.update({k: v for k, v in fields.items() if k not in record})
        try:
            self._events.write(json.dumps(record, default=str) + "\n")
        except Exception:                    # noqa: BLE001
            pass
        return record

    def warn(self, message, **fields):
        self.warnings.append(message)
        self.event("warning", message=message, **fields)

    # --- sequence -----------------------------------------------------------
    def segment_record(self, segment, t_start, t_end, n_loadcell=0,
                       dwell_reason="", thrust_mean_n=None, thrust_sem_n=None):
        """Record what a segment actually did, next to what it was planned to do."""
        row = segment.as_dict()
        row.update({
            "t_mono_start": t_start,
            "t_mono_end": t_end,
            "actual_dwell_s": None if t_end is None else t_end - t_start,
            "n_loadcell": n_loadcell,
            "dwell_reason": dwell_reason,
            "thrust_mean_n": thrust_mean_n,
            "thrust_sem_n": thrust_sem_n,
        })
        self._segments.append(row)
        return row

    def _write_sequence(self):
        path = os.path.join(self.part_dir, "sequence.csv")
        with open(path, "w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(SEQUENCE_COLUMNS)
            for row in self._segments:
                writer.writerow([_cell(row.get(c)) for c in SEQUENCE_COLUMNS])

    # --- manifest and finalize ---------------------------------------------
    def _write_manifest(self, extra):
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "host": {"hostname": socket.gethostname(),
                     "platform": platform.platform(),
                     "python": platform.python_version()},
            "git_revision": git_revision(),
            "streams": {name: {"columns": list(CLOCK_COLUMNS) + list(cols),
                               "rows": self._rows[name]}
                        for name, cols in self.streams.items()},
            "n_segments": len(self._segments),
            "warnings": self.warnings,
        }
        if self.plan is not None:
            # Verbatim first, resolved second: the file the operator wrote is the
            # record of intent, the merged one is what actually ran.
            manifest["plan_source"] = self.plan.source
            manifest["plan_raw"] = self.plan.raw
            manifest["plan"] = self.plan.as_dict()
        manifest.update(extra or {})

        with open(os.path.join(self.part_dir, "manifest.json"), "w",
                  encoding="utf-8") as fp:
            json.dump(manifest, fp, indent=2, default=str)
        return manifest

    def finalize(self, outcome="completed", **extra):
        """
        Write the manifest and sequence, then rename the directory into place.

        Until the rename, the run carries a `.part` suffix. A killed process
        therefore leaves something visibly incomplete rather than a directory
        that looks finished but is missing its manifest.
        """
        self._write_sequence()
        manifest = self._write_manifest(dict(extra, outcome=outcome))
        self.flush(sync=True)
        self.close()

        target = self.final_dir
        if os.path.exists(target):
            # Two runs inside one second. Rare, but silently overwriting data is
            # not an acceptable way to handle it.
            suffix = 2
            while os.path.exists(f"{target}_{suffix}"):
                suffix += 1
            target = f"{target}_{suffix}"
        os.rename(self.part_dir, target)
        self.final_dir = target
        return manifest
