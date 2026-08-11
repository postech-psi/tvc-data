"""
Coaxial thrust over the (A, B) PWM plane, from the 2026-07-31 sweep session.

`raw/pwm/` holds 11 completed B-sweeps -- rotor A held at 1000, 1100, ... 2000 us
while rotor B walks 1000 -> 2000 in 100 us steps -- which together tile a full
11x11 grid of commanded pairs. This script turns that into thrust curves.

Where the segmentation comes from
---------------------------------
These runs were written by the `pwm_thrust_map.py`-lineage logger, so each row
carries a `phase` string ("A1400_B1700") and there is no `seg_id`. That means
`tvcbench.sequence`'s integer-keyed scheme -- and `tvctools/bench.py`, which
reads it -- does not apply here; segmenting by (phase, a_cmd_us, b_cmd_us) is
the correct reader for this format. The commands were logged by the Pi with
timestamps, so step boundaries are known exactly and never inferred from force.

Verified against the files: every step is a clean 3.97-3.98 s of the commanded
4.0 s dwell at 50 Hz (~200 samples), walked sequentially B=1000..2000, bracketed
by `idle_pre` / `ramp_down` / `idle_post`. `--timeline` prints that table per run.

Reduction matches the rest of the package: thrust is -Fz, reduced with
`analyze.robust_stats`, which drops the leading 50 % of each step as spin-up,
trims 12 % off the tail against lag error, rejects glitches by MAD, and corrects
the SEM for vibration autocorrelation.

Battery caveat
--------------
Each sweep walks B in order, so within one run high B is also low voltage -- the
confound `tvcbench.sequence` was later written to randomise away. The A-to-A
comparison is across runs recorded over ~2 hours of draining pack, so curves are
not all at one state of charge. `--voltage` reports the per-step voltage so the
size of that effect is visible rather than hidden.
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tvctools.analyze import robust_stats

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PWM_DIR = os.path.join(ROOT, "raw", "pwm")

# Bracketing phases: real, deliberate, and not steady commanded holds.
NON_STEP = ("idle_pre", "idle_post", "ramp_down", "warmup", "tare")


def _rows(path):
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def _f(row, key):
    try:
        return float(row[key])
    except (TypeError, ValueError, KeyError):
        return float("nan")


def segments(rows):
    """
    Contiguous runs of one commanded (phase, A, B), in file order.

    Contiguity matters: a command revisited later in the same run is a separate
    segment, not more samples of the first one.
    """
    segs = []
    prev = None
    for r in rows:
        key = (r["phase"], r["a_cmd_us"], r["b_cmd_us"], r.get("sweep_idx"))
        if key != prev:
            segs.append({"phase": r["phase"], "a_us": int(r["a_cmd_us"]),
                         "b_us": int(r["b_cmd_us"]), "rows": []})
            prev = key
        segs[-1]["rows"].append(r)
    for s in segs:
        t = [_f(r, "t_epoch") for r in s["rows"]]
        s["t0"], s["t1"], s["n"] = t[0], t[-1], len(t)
    return segs


def load_run(rdir):
    """Steady-state points for one run directory, plus its timeline."""
    with open(os.path.join(rdir, "run.json"), encoding="utf-8") as f:
        meta = json.load(f)

    lc = segments(_rows(os.path.join(rdir, "loadcell.csv")))
    bat = _rows(os.path.join(rdir, "battery.csv"))

    # Battery is a separate 50 Hz stream on the same clock -- index it by time
    # so each load-cell segment can be given the voltage that applied during it.
    bt = np.array([_f(r, "t_epoch") for r in bat])
    bv = np.array([_f(r, "voltage_v") for r in bat])
    bi = np.array([_f(r, "current_a") for r in bat])

    name = os.path.basename(rdir)
    points, timeline = [], []
    for s in lc:
        dur = s["t1"] - s["t0"]
        entry = {"phase": s["phase"], "a_us": s["a_us"], "b_us": s["b_us"],
                 "n": s["n"], "dur_s": dur, "t0": s["t0"], "t1": s["t1"]}
        timeline.append(entry)
        if s["phase"] in NON_STEP:
            continue

        st = robust_stats(np.array([-_f(r, "Fz") for r in s["rows"]]))
        tq = robust_stats(np.array([_f(r, "Tz") for r in s["rows"]]))
        if st is None:
            continue

        sel = (bt >= s["t0"]) & (bt <= s["t1"])
        volt = float(np.median(bv[sel])) if sel.any() else float("nan")
        curr = float(np.median(bi[sel])) if sel.any() else float("nan")

        points.append({
            "run": name, "a_us": s["a_us"], "b_us": s["b_us"],
            "thrust_N": st["mean"], "thrust_sem": st["sem"], "n": st["n"],
            "torque_Nm": tq["mean"] if tq else float("nan"),
            "voltage_v": volt, "current_a": curr,
            "power_w": volt * curr,
            "t_rel_s": s["t0"] - lc[0]["t0"], "dur_s": dur,
        })
    return points, timeline, meta, name


def collect():
    points, timelines, skipped = [], {}, []
    for rdir in sorted(glob.glob(os.path.join(PWM_DIR, "*"))):
        if not os.path.isdir(rdir):
            continue
        with open(os.path.join(rdir, "run.json"), encoding="utf-8") as f:
            meta = json.load(f)
        name = os.path.basename(rdir)
        if meta.get("outcome") != "completed":
            skipped.append("%s (%s: %s)" % (name, meta.get("outcome"),
                                            meta.get("abort_reason")))
            continue
        # The A1850 holds are a different experiment (one command, 60 s dwell);
        # the grid is built from the B-sweeps only.
        if meta["grid"]["n_grid_points"] < 2:
            skipped.append("%s (single-point hold, not a sweep)" % name)
            continue
        pts, tl, _m, _n = load_run(rdir)
        points.extend(pts)
        timelines[name] = tl
    return points, timelines, skipped


def print_timeline(timelines):
    print("\n=== PWM segment -> time interval (relative to each run's first sample) ===")
    for name, tl in timelines.items():
        t0 = tl[0]["t0"]
        print("\n%s" % name)
        print("  %-12s %6s %6s %5s %8s %8s %7s"
              % ("phase", "A", "B", "n", "t_start", "t_end", "dur"))
        ramp = [e for e in tl if e["phase"] == "ramp_down"]
        for e in tl:
            if e["phase"] == "ramp_down" and e is not ramp[0]:
                continue
            if e["phase"] == "ramp_down":
                span = (ramp[-1]["t1"] - ramp[0]["t0"])
                print("  %-12s %6s %6s %5d %8.2f %8.2f %7.2f  (%d steps)"
                      % ("ramp_down", "->1000", "->1000",
                         sum(r["n"] for r in ramp), ramp[0]["t0"] - t0,
                         ramp[-1]["t1"] - t0, span, len(ramp)))
                continue
            print("  %-12s %6d %6d %5d %8.2f %8.2f %7.2f"
                  % (e["phase"], e["a_us"], e["b_us"], e["n"],
                     e["t0"] - t0, e["t1"] - t0, e["dur_s"]))


def print_table(points, show_voltage=False):
    print("\n=== Steady-state grid points (thrust = -Fz, robust_stats) ===")
    hdr = "%6s %6s %9s %8s %6s %8s"
    cols = ["A_us", "B_us", "thrust_N", "+/-SEM", "n", "torque"]
    if show_voltage:
        hdr += " %8s %8s %8s"
        cols += ["V", "I_A", "P_W"]
    print(hdr % tuple(cols))
    for p in sorted(points, key=lambda q: (q["a_us"], q["b_us"])):
        vals = [p["a_us"], p["b_us"], p["thrust_N"], p["thrust_sem"],
                p["n"], p["torque_Nm"]]
        line = "%6d %6d %9.3f %8.3f %6d %8.4f" % tuple(vals)
        if show_voltage:
            line += " %8.3f %8.2f %8.1f" % (p["voltage_v"], p["current_a"],
                                            p["power_w"])
        print(line)


def plot(points):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a_vals = sorted({p["a_us"] for p in points})
    cmap = plt.get_cmap("viridis")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    ax, ax2 = axes

    for i, a in enumerate(a_vals):
        sel = sorted([p for p in points if p["a_us"] == a],
                     key=lambda q: q["b_us"])
        b = np.array([p["b_us"] for p in sel])
        t = np.array([p["thrust_N"] for p in sel])
        e = np.array([p["thrust_sem"] for p in sel])
        color = cmap(i / max(1, len(a_vals) - 1))
        ax.errorbar(b, t, yerr=e, marker="o", ms=4, lw=1.4, capsize=2,
                    color=color, label="A=%d" % a)

    ax.set_xlabel("rotor B command  [$\\mu$s]")
    ax.set_ylabel("thrust  [N]")
    ax.set_title("Coaxial thrust vs B, one curve per A")
    ax.grid(alpha=0.3)
    ax.axhline(0, color="0.6", lw=0.8)
    ax.legend(fontsize=7.5, ncol=2, loc="upper left")

    # Same data as a surface over the commanded plane.
    b_vals = sorted({p["b_us"] for p in points})
    grid = np.full((len(a_vals), len(b_vals)), np.nan)
    lookup = {(p["a_us"], p["b_us"]): p["thrust_N"] for p in points}
    for i, a in enumerate(a_vals):
        for j, b in enumerate(b_vals):
            if (a, b) in lookup:
                grid[i, j] = lookup[(a, b)]

    im = ax2.imshow(grid, origin="lower", aspect="auto", cmap="magma",
                    extent=[min(b_vals) - 50, max(b_vals) + 50,
                            min(a_vals) - 50, max(a_vals) + 50])
    cs = ax2.contour(b_vals, a_vals, grid, levels=8, colors="w",
                     linewidths=0.7, alpha=0.8)
    ax2.clabel(cs, inline=True, fontsize=7, fmt="%.0f N")
    ax2.set_xlabel("rotor B command  [$\\mu$s]")
    ax2.set_ylabel("rotor A command  [$\\mu$s]")
    ax2.set_title("Thrust over the (A, B) plane")
    fig.colorbar(im, ax=ax2, label="thrust  [N]")

    fig.suptitle("Coaxial thrust map -- raw/pwm 2026-07-31 session "
                 "(%d points, 4 s dwells)" % len(points), y=1.00)
    fig.tight_layout()

    out = os.path.join(ROOT, "out", "pwm_grid_thrust.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=160, bbox_inches="tight")
    print("\nwrote %s" % out)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeline", action="store_true",
                    help="print each PWM segment's time interval, per run")
    ap.add_argument("--voltage", action="store_true",
                    help="include per-step voltage/current/power in the table")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    points, timelines, skipped = collect()
    if not points:
        print("no usable sweeps found in %s" % PWM_DIR)
        return 1

    a_vals = sorted({p["a_us"] for p in points})
    b_vals = sorted({p["b_us"] for p in points})
    print("%d sweeps -> %d grid points   A: %s   B: %d..%d step %d"
          % (len(timelines), len(points), a_vals, min(b_vals), max(b_vals),
             b_vals[1] - b_vals[0] if len(b_vals) > 1 else 0))
    for s in skipped:
        print("skipped: %s" % s)

    if args.timeline:
        print_timeline(timelines)
    print_table(points, show_voltage=args.voltage)

    csv_out = os.path.join(ROOT, "out", "pwm_grid_thrust.csv")
    os.makedirs(os.path.dirname(csv_out), exist_ok=True)
    cols = ["run", "a_us", "b_us", "thrust_N", "thrust_sem", "n", "torque_Nm",
            "voltage_v", "current_a", "power_w", "t_rel_s", "dur_s"]
    with open(csv_out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for p in sorted(points, key=lambda q: (q["a_us"], q["b_us"])):
            w.writerow({k: p[k] for k in cols})
    print("\nwrote %s" % csv_out)

    if not args.no_plot:
        plot(points)
    return 0


if __name__ == "__main__":
    sys.exit(main())
