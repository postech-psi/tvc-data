"""
Plots for the two maps, plus the step-resolution figure.

The step figure exists to answer one question directly: the raw thrust trace
looks too noisy to tell PWM steps apart, so does averaging actually recover
them? It overlays the raw samples, the commanded staircase and the per-step
means with error bars, on the same axes.
"""

import os

import numpy as np


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_run_steps(run_dir, out_path=None):
    """Raw thrust vs command-segmented step means for one run."""
    from .analyze import load_merged, steps_from_merged, robust_stats, _fnum
    plt = _mpl()

    cols = load_merged(run_dir)
    if not cols:
        return None
    t = np.array([float(x) for x in cols["t_rel_s"]])
    thrust = np.array([float(x) if x != "" else np.nan for x in cols["thrust_N"]])
    bcmd = np.array([float(x) if x != "" else np.nan for x in cols["b_cmd_us"]])

    fig, ax1 = plt.subplots(figsize=(13, 6))
    ax1.plot(t, thrust, color="#b8c4d0", lw=0.7, zorder=1,
             label="raw thrust (20 Hz, sigma ~ 0.7 N)")

    for step in steps_from_merged(cols):
        idx = step["idx"]
        st = robust_stats(_fnum(cols, "thrust_N", idx))
        if not st:
            continue
        # Error bar spans the settled portion only -- the same samples averaged
        keep = idx[int(len(idx) * 0.5):]
        x0, x1 = t[keep[0]], t[keep[-1]]
        ax1.hlines(st["mean"], x0, x1, color="#e34948", lw=2.6, zorder=3)
        ax1.fill_between([x0, x1], st["mean"] - st["sem"], st["mean"] + st["sem"],
                         color="#e34948", alpha=0.35, zorder=2)
        ax1.annotate("%.2f" % st["mean"], ((x0 + x1) / 2, st["mean"]),
                     textcoords="offset points", xytext=(0, 7),
                     ha="center", fontsize=7, color="#7a1f1f")

    ax1.plot([], [], color="#e34948", lw=2.6, label="step mean +/- SEM (command-segmented)")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("thrust [N]")
    ax1.grid(alpha=0.3)

    # Coaxial rig: both rotors matter, so both commands are always drawn.
    acmd = np.array([float(x) if x != "" else np.nan for x in cols["a_cmd_us"]])
    ax2 = ax1.twinx()
    ax2.plot(t, acmd, color="#eda100", lw=1.2, alpha=0.85, ls="--",
             label="commanded A (rotor 1) [us]")
    ax2.plot(t, bcmd, color="#2a78d6", lw=1.0, alpha=0.65,
             label="commanded B (rotor 2) [us]")
    ax2.set_ylabel("commanded PWM [us]", color="#2a78d6")
    ax2.tick_params(axis="y", labelcolor="#2a78d6")

    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8)
    ax1.set_title("Steps are resolved by averaging, not by the raw signal - %s"
                  % os.path.basename(run_dir))

    out_path = out_path or os.path.join(run_dir, "steps.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*"]


def plot_map(rows, out_path):
    """
    Coaxial map: thrust and torque against rotor B, one curve per rotor A level.

    This is a two-input system -- thrust and especially reaction torque depend
    on *both* rotor commands -- so collapsing onto a single PWM axis would hide
    the effect the rig exists to measure. A is therefore a separate series, not
    a colour.
    """
    plt = _mpl()
    rows = [r for r in rows if r.get("b_cmd_us") and r.get("a_cmd_us")]
    if not rows:
        return None

    a_levels = sorted({r["a_cmd_us"] for r in rows})
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))

    def series(ax, ykey, ylabel, title):
        for i, a in enumerate(a_levels):
            pts = sorted((r for r in rows if r["a_cmd_us"] == a),
                         key=lambda r: r["b_cmd_us"])
            x = np.array([p["b_cmd_us"] for p in pts], dtype=float)
            y = np.array([p.get(ykey) if p.get(ykey) is not None else np.nan
                          for p in pts], dtype=float)
            err = np.array([p.get(ykey.replace("_N", "_sem")
                                  .replace("_Nm", "_sem")) or 0 for p in pts],
                           dtype=float)
            ax.errorbar(x, y, yerr=err if np.any(err) else None,
                        marker=MARKERS[i % len(MARKERS)], ms=5, lw=1.2, capsize=2,
                        label="A = %d us" % a)
        ax.set_xlabel("rotor B command [us]")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, title="rotor A")

    series(axes[0], "thrust_N", "thrust [N]", "Thrust vs B, per A level")
    series(axes[1], "torque_Nm", "reaction torque Tz [N.m]", "Torque vs B, per A level")

    ax = axes[2]
    for i, a in enumerate(a_levels):
        pts = sorted((r for r in rows if r["a_cmd_us"] == a),
                     key=lambda r: r["b_cmd_us"])
        ax.plot([p["b_cmd_us"] for p in pts],
                [p.get("efficiency_N_per_W") or np.nan for p in pts],
                marker=MARKERS[i % len(MARKERS)], ms=5, lw=1.2, label="A = %d us" % a)
    ax.set_xlabel("rotor B command [us]")
    ax.set_ylabel("thrust per electrical watt [N/W]")
    ax.set_title("Efficiency vs B, per A level")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, title="rotor A")

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def plot_coax_grid(rows, out_path):
    """
    The (A, B) plane: thrust and torque as 2-D maps over both rotor commands.

    Cells are coloured by measurement; unmeasured combinations are left blank so
    the coverage of the grid is visible rather than interpolated over.
    """
    plt = _mpl()
    rows = [r for r in rows if r.get("a_cmd_us") and r.get("b_cmd_us")]
    if not rows:
        return None

    a_vals = sorted({r["a_cmd_us"] for r in rows})
    b_vals = sorted({r["b_cmd_us"] for r in rows})
    ai = {a: i for i, a in enumerate(a_vals)}
    bi = {b: i for i, b in enumerate(b_vals)}

    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    for ax, key, label in ((axes[0], "thrust_N", "thrust [N]"),
                           (axes[1], "torque_Nm", "reaction torque Tz [N.m]")):
        grid = np.full((len(a_vals), len(b_vals)), np.nan)
        counts = np.zeros_like(grid)
        for r in rows:
            v = r.get(key)
            if v is None:
                continue
            i, j = ai[r["a_cmd_us"]], bi[r["b_cmd_us"]]
            grid[i, j] = v if np.isnan(grid[i, j]) else grid[i, j] + v
            counts[i, j] += 1
        with np.errstate(invalid="ignore"):
            grid = np.where(counts > 1, grid / np.maximum(counts, 1), grid)

        im = ax.imshow(grid, origin="lower", aspect="auto", cmap="viridis")
        ax.set_xticks(range(len(b_vals)))
        ax.set_xticklabels(b_vals, rotation=45, fontsize=8)
        ax.set_yticks(range(len(a_vals)))
        ax.set_yticklabels(a_vals, fontsize=8)
        ax.set_xlabel("rotor B command [us]")
        ax.set_ylabel("rotor A command [us]")
        ax.set_title("%s over the coaxial (A, B) grid" % label)
        for i in range(len(a_vals)):
            for j in range(len(b_vals)):
                if np.isfinite(grid[i, j]):
                    ax.text(j, i, "%.2f" % grid[i, j], ha="center", va="center",
                            fontsize=6, color="w")
        fig.colorbar(im, ax=ax, label=label)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def plot_sag(rows, sag, out_path):
    """Thrust vs voltage at each constant-PWM setpoint, with the fitted slope."""
    plt = _mpl()
    if not sag:
        return None
    fig, axes = plt.subplots(1, len(sag), figsize=(6 * len(sag), 5), squeeze=False)
    for ax, grp in zip(axes[0], sag):
        # Plot exactly the points build_sag fit -- per-run means on the same
        # voltage basis as the slope/intercept. Re-deriving them from `rows`
        # here would pick the loaded voltage while the fit uses the no-load
        # voltage, leaving the line shifted off the data.
        pts = grp.get("points")
        if not pts:
            continue
        v = np.array([p["voltage_v"] for p in pts], dtype=float)
        t = np.array([p["thrust_N"] for p in pts], dtype=float)
        e = np.array([p["thrust_sem"] for p in pts], dtype=float)
        ax.errorbar(v, t, yerr=e, fmt="o", color="#2a78d6", ecolor="#888",
                    capsize=3, label="run means")
        xs = np.linspace(v.min(), v.max(), 50)
        ax.plot(xs, grp["dthrust_dV_N_per_V"] * xs + grp["intercept_N"],
                color="#e34948", lw=1.6,
                label="%.2f N/V  (r=%.3f)" % (grp["dthrust_dV_N_per_V"], grp["r"]))
        for p in pts:
            ax.annotate(p["run"].split("_")[0], (p["voltage_v"], p["thrust_N"]),
                        textcoords="offset points", xytext=(6, -3), fontsize=7,
                        color="#555")
        basis = grp.get("voltage_basis", "loaded")
        ax.set_xlabel("no-load pack voltage [V]" if basis == "noload"
                      else "pack voltage under load [V]")
        ax.set_ylabel("thrust [N]")
        ax.set_title("Constant command A%d_B%d\nthrust ~ V^%.2f, %.1f%% loss over %.2f V"
                     % (grp["a_cmd_us"], grp["b_cmd_us"], grp["exponent_k"],
                        grp["pct_thrust_loss"], grp["voltage_max"] - grp["voltage_min"]))
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path
