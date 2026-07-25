"""
Move raw acquisition files into a self-describing layout.

Raw data arrives wherever the acquisition scripts happened to be run, named by
epoch or by a bare timestamp. Neither tells you what the file contains without
opening it. This puts every raw file under its measurement date and source:

    raw/2026-07-24/pwm/thrust_map_<epoch>.csv     commands + voltage/current
    raw/2026-07-24/loadcell/data_<date>_<time>.csv  thrust + torque
    raw/2026-07-24/ulog/log_*.ulg                 flight-controller log
    raw/2026-07-24/ulog/_redundant/               duplicates, no-motor logs

The date comes from the file's *contents* where possible (a thrust_map carries a
true epoch), never from its mtime -- git checkouts rewrite mtimes and would put
everything on the day of the clone.

Moves are planned first and printed; nothing is written without `apply=True`.
Git-tracked files move with `git mv` so history follows them.
"""

import os
import shutil
import subprocess

from .discover import scan
from .schema import PWM_MAP, LOADCELL, ULOG, DERIVED, epoch_to_local_str

RAW_DIR = "raw"
SUBDIR = {PWM_MAP: "pwm", LOADCELL: "loadcell", ULOG: "ulog"}
REDUNDANT = "_redundant"


def _tracked(root):
    """Set of git-tracked paths, so moves can preserve history."""
    try:
        out = subprocess.run(["git", "-C", root, "ls-files"],
                             capture_output=True, text=True, timeout=30)
        return set(out.stdout.split("\n")) if out.returncode == 0 else set()
    except (OSError, subprocess.SubprocessError):
        return set()


def plan_moves(root, redundant=None):
    """
    Decide where every raw file should live.

    `redundant` is a set of ulog basenames to file under _redundant/ -- truncated
    duplicates and logs with no motor activity, which are kept (nothing is
    deleted) but moved out of the way.

    Returns a list of (src_rel, dst_rel) and a list of skipped (path, reason).
    """
    redundant = redundant or set()
    moves, skipped = [], []

    for rec in scan(root):
        kind = rec["kind"]
        rel = rec["rel_path"]
        if kind == DERIVED:
            skipped.append((rel, "generated output"))
            continue
        if kind not in SUBDIR:
            skipped.append((rel, "unrecognized: %s" % kind))
            continue
        if rel.startswith(RAW_DIR + "/"):
            continue                      # already in place
        if not rec.get("start_epoch"):
            skipped.append((rel, "no timestamp -- cannot date it"))
            continue

        day = epoch_to_local_str(rec["start_epoch"], "%Y-%m-%d")
        parts = [RAW_DIR, day, SUBDIR[kind]]
        if kind == ULOG and rec["name"] in redundant:
            parts.append(REDUNDANT)
        dst = "/".join(parts + [rec["name"]])
        if dst != rel:
            moves.append((rel, dst))

    return moves, skipped


def find_redundant_ulogs(root):
    """
    ulogs that carry no unique information.

    Two cases, both discovered rather than hard-coded: a log with no varying
    actuator channel never saw the motor move, and a log whose samples are a
    prefix of a longer one is the same boot downloaded twice.
    """
    from .ulogtime import ulog_outputs
    import numpy as np

    infos = []
    for rec in scan(root):
        if rec["kind"] != ULOG:
            continue
        got = ulog_outputs(os.path.join(root, rec["rel_path"]))
        if not got:
            continue
        t, chans = got
        live = {c: v for c, v in chans.items() if np.ptp(v) > 0}
        infos.append({"name": rec["name"], "t": t, "live": live})

    out = {}
    for info in infos:
        if not info["live"]:
            out[info["name"]] = "no motor activity"
    for a in infos:
        if a["name"] in out:
            continue
        for b in infos:
            if a is b or b["name"] in out or len(b["t"]) <= len(a["t"]):
                continue
            n = len(a["t"])
            if np.allclose(a["t"], b["t"][:n]):
                out[a["name"]] = "truncated copy of %s" % b["name"]
                break
    return out


def apply_moves(root, moves, tracked=None):
    """Execute the planned moves, using git mv for tracked files."""
    tracked = tracked if tracked is not None else _tracked(root)
    done = 0
    for src, dst in moves:
        src_abs = os.path.join(root, src.replace("/", os.sep))
        dst_abs = os.path.join(root, dst.replace("/", os.sep))
        if not os.path.exists(src_abs):
            continue
        os.makedirs(os.path.dirname(dst_abs), exist_ok=True)
        moved = False
        if src in tracked:
            r = subprocess.run(["git", "-C", root, "mv", src, dst],
                               capture_output=True, text=True)
            moved = r.returncode == 0
        if not moved:
            shutil.move(src_abs, dst_abs)
        done += 1
    return done


def prune_empty_dirs(root, *names):
    """Remove leftover empty directories after a reorganization."""
    removed = []
    for name in names:
        base = os.path.join(root, name)
        if not os.path.isdir(base):
            continue
        for dirpath, dirnames, filenames in os.walk(base, topdown=False):
            if not dirnames and not filenames:
                os.rmdir(dirpath)
                removed.append(os.path.relpath(dirpath, root))
    return removed
