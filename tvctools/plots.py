"""
Plots for the two maps, plus the step-resolution figure.

The step figure exists to answer one question directly: the raw thrust trace
looks too noisy to tell PWM steps apart, so does averaging actually recover
them? It overlays the raw samples, the commanded staircase and the per-step
means with error bars, on the same axes.
"""

import os
import re

import numpy as np


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def fit_pwm_surface(rows, quantity, weight_key=None, degree=2):
    """
    Least-squares surface z = f(A_us, B_us) over the raw commanded PWM values.

    degree=1: plane z = c0 + c1*A + c2*B
    degree=2: adds c3*A^2 + c4*B^2 + c5*A*B

    The quadratic terms matter here, not as decoration: a pure plane fit on
    this rig's torque data leaves a clear systematic residual bulge above
    B=1900-2000 (R^2=0.85 -> 0.90 with the quadratic terms added), consistent
    with reaction torque scaling closer to command^2 than command^1 near full
    throttle -- the standard result for a fixed-pitch prop, where torque
    approximately follows RPM^2 and RPM approximately follows command.

    Weighted by 1/sem**2 when a sem column is given, so a tightly-measured
    point pulls the fit harder than a noisy one.

    Returns (a, b, z, coeffs) with rows lacking the quantity dropped.
    """
    pts = [r for r in rows if r.get("a_cmd_us") and r.get("b_cmd_us")
           and r.get(quantity) is not None]
    if len(pts) < (6 if degree == 2 else 4):
        return None
    a = np.array([r["a_cmd_us"] for r in pts], dtype=float)
    b = np.array([r["b_cmd_us"] for r in pts], dtype=float)
    z = np.array([r[quantity] for r in pts], dtype=float)

    cols = [np.ones_like(a), a, b]
    if degree == 2:
        cols += [a * a, b * b, a * b]
    X = np.column_stack(cols)

    if weight_key:
        sem = np.array([r.get(weight_key) or 1.0 for r in pts], dtype=float)
        w = 1.0 / np.maximum(sem, 1e-6) ** 2
        sw = np.sqrt(w)
        coeffs, *_ = np.linalg.lstsq(X * sw[:, None], z * sw, rcond=None)
    else:
        coeffs, *_ = np.linalg.lstsq(X, z, rcond=None)
    return a, b, z, coeffs


def _eval_surface(coeffs, A, B):
    c = list(coeffs) + [0.0] * (6 - len(coeffs))
    return c[0] + c[1] * A + c[2] * B + c[3] * A**2 + c[4] * B**2 + c[5] * A * B


def plot_pwm_identification(rows, out_path, quantity="torque_Nm",
                            weight_key="torque_sem", zlabel="torque [N.m]",
                            title="Torque identification", degree=2,
                            signed=None):
    """
    3D scatter of measured points plus the fitted surface, over (PWM A, PWM B).

    `signed` controls styling and defaults to whether the quantity actually
    changes sign in this dataset (torque does, at the A=B balance line;
    thrust does not). The two are styled deliberately differently rather than
    sharing one look, since otherwise two surfaces of the same general shape
    are easy to mistake for each other at a glance:

    - signed (torque): points coloured red/blue by sign, a translucent grey
      z=0 plane marks the balance line, warm-toned surface.
    - unsigned (thrust): single warm colour throughout, no zero plane -- a
      magnitude that is never negative doesn't need one.
    """
    plt = _mpl()
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d projection)

    got = fit_pwm_surface(rows, quantity, weight_key, degree)
    if got is None:
        return None
    a, b, z, coeffs = got
    fitted = _eval_surface(coeffs, a, b)
    resid = z - fitted
    rmse = float(np.sqrt(np.mean(resid ** 2)))
    r2 = 1.0 - np.sum(resid ** 2) / max(np.sum((z - z.mean()) ** 2), 1e-12)
    if signed is None:
        signed = bool((z < 0).any() and (z > 0).any())

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ag = np.linspace(a.min(), a.max(), 20)
    bg = np.linspace(b.min(), b.max(), 20)
    AG, BG = np.meshgrid(ag, bg)
    ZG = _eval_surface(coeffs, AG, BG)

    if signed:
        surf_color, wire_color = "#e8a15c", "#b5701f"      # warm orange: torque
        pos, neg = z >= 0, z < 0
        ax.scatter(a[pos], b[pos], z[pos], color="#c0392b", s=34,
                   edgecolor="k", linewidth=0.4, depthshade=True, zorder=3,
                   label="positive (B side leads)")
        ax.scatter(a[neg], b[neg], z[neg], color="#2e6fa7", s=34,
                   edgecolor="k", linewidth=0.4, depthshade=True, zorder=3,
                   label="negative (A side leads)")
        # Zero plane -- the balance line where the two rotors' reaction
        # torques cancel is the physically meaningful reference here, not the
        # data's own min/max.
        ZERO = np.zeros_like(AG)
        ax.plot_surface(AG, BG, ZERO, color="0.6", alpha=0.15, linewidth=0,
                        zorder=0)
        ax.legend(fontsize=8, loc="upper left")
    else:
        surf_color, wire_color = "#7fa8c9", "#3a6ea5"       # cool blue: thrust
        ax.scatter(a, b, z, color="#1f6fb2", s=32, edgecolor="k",
                  linewidth=0.4, depthshade=True, zorder=3)

    ax.plot_surface(AG, BG, ZG, color=surf_color, alpha=0.4, linewidth=0,
                    antialiased=True, zorder=1)
    ax.plot_wireframe(AG, BG, ZG, color=wire_color, alpha=0.4, linewidth=0.5,
                      rstride=2, cstride=2, zorder=2)

    ax.set_xlabel("rotor A command [us]")
    ax.set_ylabel("rotor B command [us]")
    ax.set_zlabel(zlabel)
    deg_label = "quadratic" if degree == 2 else "linear"
    ax.set_title("%s (%s fit)\nR^2=%.3f   RMSE=%.4g   n=%d"
                 % (title, deg_label, r2, rmse, len(z)), fontsize=10)

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path


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


def plot_map(rows, out_path, legacy_runs=None):
    """
    Coaxial map: thrust and torque against rotor B, one curve per rotor A level.

    This is a two-input system -- thrust and especially reaction torque depend
    on *both* rotor commands -- so collapsing onto a single PWM axis would hide
    the effect the rig exists to measure. A is therefore a separate series, not
    a colour.

    `legacy_runs` (from tvctools.legacy) overlays the 2026-07-20 balanced
    sweeps on the thrust panel. Those are the ONLY runs that reach full
    throttle -- every later session stops at 1850 us -- so without them the top
    of the curve is extrapolation. They are drawn dashed and grey to keep the
    distinction obvious: one PWM axis instead of two, and no voltage record.
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

    if legacy_runs:
        ax0 = axes[0]
        for j, run in enumerate(legacy_runs):
            pts = sorted(run["points"], key=lambda p: p["pwm_us"])
            ax0.errorbar(
                [p["pwm_us"] for p in pts], [p["thrust_N"] for p in pts],
                yerr=[p["thrust_sem"] for p in pts],
                color="0.35", ls="--", lw=1.0, marker="x", ms=4, capsize=2,
                alpha=0.85, zorder=1,
                label="2026-07-20 balanced (A=B)" if j == 0 else None)
        top = max(p["thrust_N"] for r in legacy_runs for p in r["points"])
        ax0.axhline(top, color="0.35", ls=":", lw=0.9, alpha=0.7)
        ax0.annotate("full throttle: %.1f N\n(only 07-20 reaches PWM 2000)" % top,
                     xy=(2000, top), xytext=(-8, -28),
                     textcoords="offset points", ha="right", fontsize=7.5,
                     color="0.25")
        ax0.set_xlabel("rotor B command [us]   (07-20: both rotors)")
        ax0.legend(fontsize=7.5, title="rotor A")

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


_SAG_XLABEL = {
    "noload": "no-load pack voltage [V]  (state of charge)",
    "loaded": "pack voltage under load [V]",
}


def _run_tag(name):
    """Drop the A####_B#### prefix -- it is identical for every point here."""
    parts = name.split("_")
    tail = [p for p in parts if not re.fullmatch(r"[AB]\d+(-\d+)?", p)]
    return "_".join(tail) or name


def plot_sag(sag, out_path):
    """Thrust vs voltage at each constant-PWM setpoint, with the fitted slope.

    Points come from the fit itself (`grp["points"]`), not from a re-derivation
    off the run table: the fit runs on the no-load voltage, and scattering the
    loaded voltage against a no-load fit puts the line about an IR-drop away
    from its own data.
    """
    plt = _mpl()
    if not sag:
        return None
    fig, axes = plt.subplots(1, len(sag), figsize=(6 * len(sag), 5), squeeze=False)
    for ax, grp in zip(axes[0], sag):
        pts = grp.get("points") or []
        if not pts:
            continue
        v = np.array([p["voltage_v"] for p in pts], dtype=float)
        t = np.array([p["thrust_N"] for p in pts], dtype=float)
        e = np.array([p["thrust_sem"] for p in pts], dtype=float)
        ax.errorbar(v, t, yerr=e, fmt="o", color="#2a78d6", ecolor="#888",
                    capsize=3, zorder=3, label="run means")
        slope, intercept = grp["dthrust_dV_N_per_V"], grp["intercept_N"]
        xs = np.linspace(v.min(), v.max(), 50)
        ax.plot(xs, slope * xs + intercept, color="#e34948", lw=1.6, zorder=2,
                label="fit %.2f N/V  (r=%.3f)" % (slope, grp["r"]))
        for p in pts:
            ax.annotate(_run_tag(p["run"]), (p["voltage_v"], p["thrust_N"]),
                        textcoords="offset points", xytext=(6, -3), fontsize=7,
                        color="#555")
        ax.set_xlabel(_SAG_XLABEL.get(grp.get("voltage_basis"),
                                      "pack voltage [V]"))
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
