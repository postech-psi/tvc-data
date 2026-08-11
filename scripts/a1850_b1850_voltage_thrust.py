"""
Thrust vs rested battery voltage, both rotors held at 1850 us.

Every run in this set commands the same thing -- A=1850, B=1850 -- so the only
thing that varies between them is the pack's state of charge. Plotting the two
against each other is therefore a direct read of how much thrust the rig loses
as the battery depletes.

Voltage basis
-------------
The x axis is the *no-load* voltage after the run, not the voltage recorded
during it. Under 20-25 A the terminal voltage is the open-circuit voltage minus
an I*R drop that ranged 0.66-1.18 V across this data, so the loaded number mixes
depletion with sag and cannot be compared run to run. See tvctools/battery.py.

"After the run" is what was asked for, and it is also the honest choice here:
the 2026-07-31 runs have an explicit `idle_post` phase, and the 2026-07-24 runs
have `after_v` already resolved from the ulog idle window. One caveat is carried
through to the plot -- the pack is still relaxing when those windows end (the
2026-07-31 tail is climbing at ~2 mV/s at t+30 s), so every point sits a little
below its fully-rested OCV. That bias is in the same direction for all runs and
roughly the same size, so it shifts the curve without distorting its slope.

Two logger generations feed in and are treated identically:

  runs/2026-07-24_s2/A1850_B1850_*   merged.csv + run.json   (old pipeline)
  A1850 B1850*_2026-07-31_*          loadcell.csv/battery.csv (tvcbench)

Thrust is -Fz in both, reduced with analyze.robust_stats so the settle trim,
MAD rejection, and autocorrelation-corrected SEM match the rest of the package.
"""

import csv
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tvctools.analyze import robust_stats

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OLD_GLOB = os.path.join(ROOT, "runs", "2026-07-24_s2", "A1850_B1850_*")
NEW_GLOBS = [os.path.join(ROOT, "A1850 B1850 2026-07-31_*"),
             os.path.join(ROOT, "A1850 B1850_2026-07-31_*")]

HOLD_PHASE = "A1850_B1850"
IDLE_POST = "idle_post"
IDLE_PRE = "idle_pre"


def _rows(path):
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def _col(rows, name, want=None):
    """Float column, optionally restricted to rows whose phase == want."""
    out = []
    for r in rows:
        if want is not None and r.get("phase") != want:
            continue
        try:
            out.append(float(r[name]))
        except (TypeError, ValueError, KeyError):
            pass
    return np.asarray(out, dtype=float)


def load_old(rdir):
    """One point from an old-pipeline run directory, or None."""
    with open(os.path.join(rdir, "run.json"), encoding="utf-8") as f:
        meta = json.load(f)

    merged = os.path.join(rdir, "merged.csv")
    if not os.path.exists(merged):
        # No load-cell file was recorded for this run -- commands only.
        return None, "no loadcell file (%s)" % os.path.basename(rdir)

    rows = [r for r in _rows(merged) if r.get("phase") == HOLD_PHASE]
    thrust = robust_stats(-_col(rows, "Fz"))
    if thrust is None:
        return None, "no usable thrust samples (%s)" % os.path.basename(rdir)

    nl = meta.get("voltage_noload_v") or {}
    if nl.get("after_v") is None:
        return None, "no rested voltage (%s)" % os.path.basename(rdir)

    return {
        "name": os.path.basename(rdir),
        "gen": "2026-07-24 (ulog)",
        "v_rest_after": float(nl["after_v"]),
        "v_rest_before": nl.get("before_v"),
        "v_loaded": float(np.median(_col(rows, "voltage_v"))),
        "current_a": float(np.median(_col(rows, "current_a"))),
        "thrust_N": thrust["mean"],
        "thrust_sem": thrust["sem"],
        "n": thrust["n"],
    }, None


def load_new(rdir):
    """One point from a tvcbench run directory, or None."""
    lc = _rows(os.path.join(rdir, "loadcell.csv"))
    bat = _rows(os.path.join(rdir, "battery.csv"))

    thrust = robust_stats(-_col(lc, "Fz", HOLD_PHASE))
    if thrust is None:
        return None, "no usable thrust samples (%s)" % os.path.basename(rdir)

    post = _col(bat, "voltage_v", IDLE_POST)
    pre = _col(bat, "voltage_v", IDLE_PRE)
    if post.size == 0:
        return None, "no idle_post samples (%s)" % os.path.basename(rdir)

    # Median over the whole idle_post window, matching battery.noload_from_pwm_map.
    return {
        "name": os.path.basename(rdir),
        "gen": "2026-07-31 (tvcbench)",
        "v_rest_after": float(np.median(post)),
        "v_rest_before": float(np.median(pre)) if pre.size else None,
        "v_loaded": float(np.median(_col(bat, "voltage_v", HOLD_PHASE))),
        "current_a": float(np.median(_col(bat, "current_a", HOLD_PHASE))),
        "thrust_N": thrust["mean"],
        "thrust_sem": thrust["sem"],
        "n": thrust["n"],
    }, None


def collect():
    points, skipped = [], []
    for rdir in sorted(glob.glob(OLD_GLOB)):
        p, why = load_old(rdir)
        (points if p else skipped).append(p or why)
    for pat in NEW_GLOBS:
        for rdir in sorted(glob.glob(pat)):
            p, why = load_new(rdir)
            (points if p else skipped).append(p or why)
    points.sort(key=lambda p: p["v_rest_after"])
    return points, skipped


def fit(x, y, sem):
    """Weighted least-squares line; falls back to unweighted if SEMs are zero."""
    w = 1.0 / np.maximum(sem, 1e-9) ** 2 if np.any(sem > 0) else np.ones_like(x)
    slope, icept = np.polyfit(x, y, 1, w=np.sqrt(w))
    pred = slope * x + icept
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    return slope, icept, (1 - ss_res / ss_tot if ss_tot else float("nan"))


def main():
    points, skipped = collect()
    if not points:
        print("no usable runs found")
        return 1

    x = np.array([p["v_rest_after"] for p in points])
    y = np.array([p["thrust_N"] for p in points])
    sem = np.array([p["thrust_sem"] for p in points])
    slope, icept, r2 = fit(x, y, sem)

    print("A=1850, B=1850  --  thrust vs rested (post-run, no-load) voltage\n")
    hdr = "%-26s %-22s %8s %8s %8s %8s %9s %6s"
    print(hdr % ("run", "generation", "V_rest", "V_load", "I_A", "thrust_N",
                 "+/- SEM", "n"))
    print("-" * 104)
    for p in points:
        print("%-26s %-22s %8.3f %8.3f %8.2f %8.3f %9.3f %6d"
              % (p["name"], p["gen"], p["v_rest_after"], p["v_loaded"],
                 p["current_a"], p["thrust_N"], p["thrust_sem"], p["n"]))

    print("\nfit:  thrust = %.4f * V_rest %+.4f   (R^2 = %.4f, %d points)"
          % (slope, icept, r2, len(points)))
    print("      %.3f N per volt; over the %.2f V spanned here, %.2f N"
          % (slope, x.max() - x.min(), slope * (x.max() - x.min())))
    for s in skipped:
        print("skipped: %s" % s)

    plot(points, x, y, sem, slope, icept, r2)
    return 0


def plot(points, x, y, sem, slope, icept, r2):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 6))

    xs = np.linspace(x.min() - 0.05, x.max() + 0.05, 100)
    ax.plot(xs, slope * xs + icept, "-", color="0.45", lw=1.4, zorder=1,
            label="fit: %.3f N/V  (R$^2$=%.3f)" % (slope, r2))

    styles = {"2026-07-24 (ulog)": ("o", "#1f77b4"),
              "2026-07-31 (tvcbench)": ("s", "#d62728")}
    for gen, (marker, color) in styles.items():
        sel = [i for i, p in enumerate(points) if p["gen"] == gen]
        if not sel:
            continue
        ax.errorbar(x[sel], y[sel], yerr=sem[sel], fmt=marker, color=color,
                    ms=7, capsize=3, lw=1.2, zorder=3, label=gen)

    # Label offsets alternate so the two runs that nearly coincide at the top
    # right (1630 and 07-31_014352, 0.05 V apart) do not overprint each other.
    for i, (p, px, py) in enumerate(zip(points, x, y)):
        label = (p["name"].replace("A1850_B1850_", "")
                 .replace("A1850 B1850 ", "").replace("A1850 B1850_", ""))
        dx, dy = (7, -12) if i % 2 == 0 else (7, 7)
        ax.annotate(label, (px, py), textcoords="offset points",
                    xytext=(dx, dy), fontsize=7, color="0.35")

    ax.set_xlabel("rested battery voltage after run, no load  [V]")
    ax.set_ylabel("thrust  [N]")
    ax.set_title("Coaxial thrust vs battery state of charge   (A=1850, B=1850 $\\mu$s)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", framealpha=0.9)

    ax.text(0.98, 0.03,
            "error bars: SEM, autocorrelation-corrected\n"
            "voltage still relaxing at window end -- points sit slightly below true OCV",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=7.5,
            color="0.4")

    out = os.path.join(ROOT, "out", "a1850_b1850_thrust_vs_voltage.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    print("\nwrote %s" % out)

    csv_out = out.replace(".png", ".csv")
    with open(csv_out, "w", newline="", encoding="utf-8") as f:
        cols = ["name", "gen", "v_rest_after", "v_rest_before", "v_loaded",
                "current_a", "thrust_N", "thrust_sem", "n"]
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for p in points:
            w.writerow({k: p.get(k) for k in cols})
    print("wrote %s" % csv_out)


if __name__ == "__main__":
    sys.exit(main())
