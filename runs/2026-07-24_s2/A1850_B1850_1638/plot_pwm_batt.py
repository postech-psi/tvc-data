#!/usr/bin/env python3
"""
3-panel "PWM output vs battery voltage / current / power" figure for a single
merged.csv run. All traces are the measured values as logged -- no correction.

Panels (shared time axis):
  1. PWM commands   - a_cmd_us (ch0), b_cmd_us (ch1 / throttle)   [us]
  2. Pack voltage   - measured (loaded)                           [V]
  3. Current + power - twin y-axes                                 [A] / [W]
"""

import sys
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUN_DIR = (Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent).resolve()
OUT_PATH = Path(sys.argv[2]) if len(sys.argv) > 2 else RUN_DIR / "pwm_batt.png"


def load(path):
    rows = list(csv.DictReader(open(path)))
    def col(k):
        out = np.full(len(rows), np.nan)
        for i, r in enumerate(rows):
            v = r.get(k, "")
            if v not in ("", None):
                out[i] = float(v)
        return out
    return {k: col(k) for k in
            ("t_rel_s", "a_cmd_us", "b_cmd_us", "voltage_v", "current_a", "power_w")}


def main():
    d = load(RUN_DIR / "merged.csv")
    t = d["t_rel_s"]

    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(13.5, 9.5), sharex=True,
        gridspec_kw=dict(hspace=0.12))

    title = "PWM output vs battery voltage / current / power - %s" % RUN_DIR.name
    fig.suptitle(title, y=0.925, fontsize=12)

    # --- Panel 1: PWM commands -------------------------------------------- #
    ax1.plot(t, d["a_cmd_us"], color="#1f77b4", lw=1.2,
             drawstyle="steps-post", label="PWM ch0 (A)")
    ax1.plot(t, d["b_cmd_us"], color="#2ca02c", lw=1.2,
             drawstyle="steps-post", label="PWM ch1 (throttle / B)")
    ax1.set_ylabel("PWM [us]")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # --- Panel 2: voltage (measured, loaded) ------------------------------ #
    ax2.plot(t, d["voltage_v"], color="#d62728", lw=0.9, label="voltage (loaded)")
    ax2.set_ylabel("voltage [V]")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # --- Panel 3: current + power ----------------------------------------- #
    cC, cP = "#7d6fb0", "#8c564b"
    ax3.plot(t, d["current_a"], color=cC, lw=1.0, label="current [A]")
    ax3.set_ylabel("current [A]", color=cC)
    ax3.tick_params(axis="y", labelcolor=cC)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlabel("time [s]")

    ax3p = ax3.twinx()
    ax3p.plot(t, d["power_w"], color=cP, lw=1.0, label="power [W]")
    ax3p.set_ylabel("power [W]", color=cP)
    ax3p.tick_params(axis="y", labelcolor=cP)

    lines = ax3.get_lines() + ax3p.get_lines()
    ax3.legend(lines, [l.get_label() for l in lines],
               loc="upper right", fontsize=8)

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PATH, dpi=140, bbox_inches="tight")
    print("wrote", OUT_PATH)


if __name__ == "__main__":
    main()
