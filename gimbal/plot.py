"""One plotting CLI for the TVC gimbal system-ID project.

Subcommands (run `python plot.py <cmd> -h` for each):

  surface   Clean least-squares cubic 3D surface through any (x, y) -> z grid
            CSV, measured points overlaid. Generic -- works for the gimbal
            pitch/yaw grid AND the motor thrust map. This is the headline plot.
              python plot.py surface grid_from_mapping/grid_points.csv \
                     --x cmd_outer --y cmd_inner --z pitch_deg
              python plot.py surface ../motor_grid.csv --x cmd_a --y cmd_b \
                     --z thrust_N --zlabel "Thrust [N]"

  grid      Convenience for the gimbal joint map: renders the pitch AND yaw
            cubic surfaces from a grid_points.csv (a real run, --from-mapping
            reconstruction, or --demo). Uses the same `surface` renderer.

  mapping   PWM -> angle mapping (2D curve, hysteresis, nonlinearity).
  step      Step-response traces (angle + rate vs time) with metrics.
  bode      Chirp frequency response (gain / phase / coherence).
  deadband  Backlash loop near neutral.

The per-test subcommands take --run / --session / --runs / --show and default to
the newest matching run under runs/ (test-type-first or nested layout).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm, colors

from analyze import (load_run, sample_rows, event_rows, load_calibration,
                     corrected_gyro, detrend_rate, last_value, GIMBAL_NAMES,
                     FINAL_ANGLE_WINDOW_S)

PROJECT_DIR = Path(__file__).resolve().parent

# Style for the summary annotation box every plot draws.
INFO_BOX = dict(boxstyle="round,pad=0.5", fc="#fffbe6", ec="#e0c060")

# Physical command limits from src/main.cpp (used only to bound the demo sweep).
NEUTRAL_US = 1520
OUTER_LO, OUTER_HI = 1370, 1690   # servo A, outer frame
INNER_LO, INNER_HI = 1350, 2040   # servo B, inner frame


def apply_plot_style(font_size: float = 11) -> None:
    """The common light theme shared by every plot."""
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": font_size,
        "axes.edgecolor": "#444", "axes.linewidth": 0.9,
        "axes.grid": True, "grid.color": "#dddddd", "grid.linewidth": 0.7,
        "figure.facecolor": "white", "axes.facecolor": "#fbfbfd",
    })


# ---------------------------------------------------------------------------
# Run discovery (matches the current runs/ layout: test-type-first folders and
# the nested run_experiment layout).
# ---------------------------------------------------------------------------
def latest_run_with(runs: Path, name: str, required_file: str) -> Path | None:
    hits = sorted(runs.glob(f"**/{name}/**/{required_file}"),
                  key=lambda p: p.stat().st_mtime)
    return hits[-1].parent if hits else None


def resolve_run_targets(args, subdirs: tuple[str, ...],
                        required_file: str) -> list[Path]:
    if getattr(args, "run", None):
        return [args.run]
    session = getattr(args, "session", None)
    if session:
        return [session / n for n in subdirs
                if (session / n / required_file).exists()]
    targets = []
    for name in subdirs:
        found = latest_run_with(args.runs, name, required_file)
        if found is not None:
            targets.append(found)
    return targets


def newest_grid_run(runs: Path) -> Path:
    best = None
    for name in ("grid_both_gimbals", "grid"):
        found = latest_run_with(runs, name, "grid_points.csv")
        if found and (best is None or
                      found.stat().st_mtime > best.stat().st_mtime):
            best = found
    if best is None:
        raise SystemExit("no grid run with grid_points.csv under runs/")
    return best


# ===========================================================================
# surface: generic least-squares cubic 3D surface (the headline plot)
# ===========================================================================
_CUBIC_TERMS = [(0, 0), (1, 0), (0, 1), (2, 0), (1, 1), (0, 2),
                (3, 0), (2, 1), (1, 2), (0, 3)]


def fit_cubic_surface(a: np.ndarray, b: np.ndarray, z: np.ndarray):
    """Least-squares fit of a full cubic polynomial z = F(a, b) (10 terms, all
    monomials up to total degree 3). a, b are centered/scaled internally for
    numerical stability. Returns an evaluator f(a, b) -> z."""
    a0, ascale = a.mean(), a.std() or 1.0
    b0, bscale = b.mean(), b.std() or 1.0
    an, bn = (a - a0) / ascale, (b - b0) / bscale
    design = np.column_stack([an**i * bn**j for i, j in _CUBIC_TERMS])
    coeffs, *_ = np.linalg.lstsq(design, z, rcond=None)

    def evaluate(a_eval: np.ndarray, b_eval: np.ndarray) -> np.ndarray:
        an_e = (a_eval - a0) / ascale
        bn_e = (b_eval - b0) / bscale
        out = np.zeros_like(an_e, dtype=float)
        for c, (i, j) in zip(coeffs, _CUBIC_TERMS):
            out += c * an_e**i * bn_e**j
        return out

    return evaluate


def plot_surface(df: pd.DataFrame, xcol: str, ycol: str, zcol: str, out: Path,
                 *, title: str | None = None, xlabel: str | None = None,
                 ylabel: str | None = None, zlabel: str | None = None,
                 cmap=cm.viridis, show: bool = False,
                 resolution: int = 40) -> Path:
    """Clean single-surface plot: smooth least-squares cubic fit through the
    measured (x, y) -> z points, with the raw points overlaid."""
    for col in (xcol, ycol, zcol):
        if col not in df.columns:
            raise SystemExit(f"column '{col}' not in CSV; have {list(df.columns)}")
    a = df[xcol].to_numpy(float)
    b = df[ycol].to_numpy(float)
    z = df[zcol].to_numpy(float)
    f = fit_cubic_surface(a, b, z)

    a_lin = np.linspace(a.min(), a.max(), resolution)
    b_lin = np.linspace(b.min(), b.max(), resolution)
    amesh, bmesh = np.meshgrid(a_lin, b_lin, indexing="ij")
    zfit = f(amesh, bmesh)

    apply_plot_style(10.5)
    fig = plt.figure(figsize=(7.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(amesh, bmesh, zfit, cmap=cmap, linewidth=0, alpha=0.9,
                    antialiased=True, rcount=resolution, ccount=resolution)
    ax.scatter(a, b, z, c="#1a1a1a", s=18, depthshade=True,
               label=f"measured ({len(a)} pts)")
    ax.set_xlabel(xlabel or xcol, labelpad=10)
    ax.set_ylabel(ylabel or ycol, labelpad=10)
    ax.set_zlabel(zlabel or zcol, labelpad=10)
    ax.set_title(title or f"Cubic surface  F({xcol}, {ycol})  fitted by least squares",
                 weight="bold", pad=14)
    ax.view_init(elev=24, azim=-60)
    ax.zaxis.set_rotate_label(False)
    ax.xaxis.pane.set_alpha(0.0)
    ax.yaxis.pane.set_alpha(0.0)
    ax.zaxis.pane.set_alpha(0.0)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.02),
              frameon=True, fontsize=9)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, bbox_inches="tight")
    print("saved", out)
    if show:
        plt.show()
    else:
        plt.close(fig)
    return out


# ===========================================================================
# grid: gimbal joint PWM_A x PWM_B -> angle, as two cubic surfaces
# ===========================================================================
def load_grid(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    need = {"cmd_outer", "cmd_inner", "pitch_deg", "yaw_deg"}
    missing = need - set(df.columns)
    if missing:
        raise SystemExit(f"{path} missing columns: {sorted(missing)}")
    return df


def synth_grid(n: int = 11, noise_deg: float = 0.04, seed: int = 7) -> pd.DataFrame:
    """A physically plausible nested-gimbal surface, for previewing with no
    hardware: per-axis gain, edge saturation, and outer-tilt cross-coupling."""
    rng = np.random.default_rng(seed)
    outer = np.linspace(OUTER_LO, OUTER_HI, n)
    inner = np.linspace(INNER_LO, INNER_HI, n)
    gain_o, gain_i = 0.052, 0.041      # deg/us
    sat_o, sat_i = 9.5, 15.0           # soft travel limits [deg]
    rows = []
    for a in outer:
        phi_o = sat_o * np.tanh(gain_o * (a - NEUTRAL_US) / sat_o)
        for b in inner:
            phi_i = sat_i * np.tanh(gain_i * (b - NEUTRAL_US) / sat_i)
            co, so = np.cos(np.radians(phi_o)), np.sin(np.radians(phi_o))
            pitch = phi_o + 0.14 * phi_i * so + 0.06 * phi_i
            yaw = phi_i * co + 0.10 * phi_o
            rows.append((a, b, pitch + rng.normal(0, noise_deg),
                         yaw + rng.normal(0, noise_deg)))
    return pd.DataFrame(rows, columns=["cmd_outer", "cmd_inner",
                                       "pitch_deg", "yaw_deg"])


def grid_from_mapping(outer_dir: Path, inner_dir: Path, n: int = 15) -> pd.DataFrame:
    """Reconstruct a joint (PWM_A, PWM_B) -> angle surface from the two
    INDEPENDENT per-axis mapping runs (separable superposition; captures the
    first-order cross-coupling only, not the multiplicative term -- treat as an
    estimate/baseline until a real joint grid sweep (test E) is run)."""
    op = pd.read_csv(outer_dir / "mapping_points.csv")
    ip = pd.read_csv(inner_dir / "mapping_points.csv")
    o = op.groupby("pulse_us").agg(main=("theta_deg", "mean"),
                                   cross=("cross_deg", "mean")).reset_index()
    i = ip.groupby("pulse_us").agg(main=("theta_deg", "mean"),
                                   cross=("cross_deg", "mean")).reset_index()
    a_nodes, b_nodes = o["pulse_us"].to_numpy(float), i["pulse_us"].to_numpy(float)
    a_grid = np.linspace(a_nodes.min(), a_nodes.max(), n)
    b_grid = np.linspace(b_nodes.min(), b_nodes.max(), n)
    o_main = np.interp(a_grid, a_nodes, o["main"].to_numpy(float))
    o_cross = np.interp(a_grid, a_nodes, o["cross"].to_numpy(float))
    i_main = np.interp(b_grid, b_nodes, i["main"].to_numpy(float))
    i_cross = np.interp(b_grid, b_nodes, i["cross"].to_numpy(float))
    rows = []
    for ai, a in enumerate(a_grid):
        for bi, b in enumerate(b_grid):
            rows.append((a, b, o_main[ai] + i_cross[bi],     # pitch
                         o_cross[ai] + i_main[bi]))           # yaw
    return pd.DataFrame(rows, columns=["cmd_outer", "cmd_inner",
                                       "pitch_deg", "yaw_deg"])


def find_mapping_pair(runs: Path) -> tuple[Path, Path]:
    outer = latest_run_with(runs, "mapping_outer_gimbal", "mapping_points.csv")
    inner = latest_run_with(runs, "mapping_inner_gimbal", "mapping_points.csv")
    if not outer or not inner:
        raise SystemExit("could not find both mapping_outer_gimbal and "
                         "mapping_inner_gimbal runs; pass --outer/--inner")
    return outer, inner


def render_grid_surfaces(df: pd.DataFrame, out_dir: Path, title_prefix: str,
                         show: bool) -> None:
    """The two clean cubic surfaces (pitch, yaw) for a gimbal grid."""
    plot_surface(df, "cmd_outer", "cmd_inner", "pitch_deg",
                 out_dir / "grid_surface_pitch.png",
                 title=f"{title_prefix}Pitch angle  θ(A,B)  fitted by least squares",
                 xlabel="Command A  (outer) [µs]", ylabel="Command B  (inner) [µs]",
                 zlabel="Pitch [deg]", cmap=cm.viridis, show=show)
    plot_surface(df, "cmd_outer", "cmd_inner", "yaw_deg",
                 out_dir / "grid_surface_yaw.png",
                 title=f"{title_prefix}Yaw angle  θ(A,B)  fitted by least squares",
                 xlabel="Command A  (outer) [µs]", ylabel="Command B  (inner) [µs]",
                 zlabel="Yaw [deg]", cmap=cm.plasma, show=show)


# ===========================================================================
# mapping: PWM -> angle (2D)
# ===========================================================================
def plot_mapping_one(run: Path, show: bool = False) -> Path:
    pts = pd.read_csv(run / "mapping_points.csv")
    lut = pd.read_csv(run / "lut.csv")
    a = json.loads((run / "analysis.json").read_text(encoding="utf-8"))
    gain = a["gain_deg_per_us"]
    neutral = a["neutral_us"]
    intercept = -gain * neutral
    gimbal = a.get("gimbal", run.name)

    apply_plot_style()
    fig = plt.figure(figsize=(11, 10))
    gs = fig.add_gridspec(3, 1, height_ratios=[2.4, 1, 1], hspace=0.28)

    ax0 = fig.add_subplot(gs[0])
    colors_ = {"up": "#2166ac", "up2": "#4393c3", "dn": "#b2182b", "dn2": "#d6604d"}
    labels = {"up": "up ①", "up2": "up ②", "dn": "down ①", "dn2": "down ②"}
    for ph in ["up", "up2", "dn", "dn2"]:
        p = pts[pts["phase"] == ph].sort_values("pulse_us")
        if p.empty:
            continue
        ax0.plot(p["pulse_us"], p["theta_deg"], "-o", ms=4, lw=1.3,
                 color=colors_[ph], label=labels[ph], alpha=0.9)
    xr = np.array([pts["pulse_us"].min(), pts["pulse_us"].max()])
    ax0.plot(xr, gain * xr + intercept, "k--", lw=1.6,
             label=f"linear fit ({gain*1000:.2f} m°/µs)")
    ax0.plot(lut["pulse_us"], lut["theta_deg"], color="#1a9850", lw=2.6, alpha=0.55,
             label="LUT (up/down mean)", zorder=1)
    ax0.axvline(neutral, color="#888", ls=":", lw=1.2)
    ax0.axhline(0, color="#888", ls=":", lw=1.2)
    ax0.plot(neutral, 0, "o", ms=9, mfc="white", mec="#333", mew=1.6, zorder=6)
    ax0.annotate(f"neutral\n{neutral:.0f} µs", (neutral, 0),
                 textcoords="offset points", xytext=(10, -34), fontsize=9, color="#333")
    ax0.set_ylabel("gimbal tilt angle  [deg]")
    ax0.set_title(f"{gimbal.title()} gimbal  —  PWM → angle mapping",
                  fontsize=14, weight="bold", pad=12)
    ax0.legend(ncol=2, framealpha=0.95, fontsize=9.5, loc="upper left")
    box = (f"gain = {gain*1000:.2f} m°/µs   ({1/gain:.1f} µs/°)\n"
           f"range = {a['angle_min_deg']:.2f}° … {a['angle_max_deg']:.2f}°  "
           f"(span {a['travel_span_deg']:.2f}°)\n"
           f"hysteresis max = {a['hysteresis_max_deg']:.3f}°\n"
           f"nonlinearity = {a['nonlinearity_pct']:.1f}%")
    ax0.text(0.985, 0.04, box, transform=ax0.transAxes, ha="right", va="bottom",
             fontsize=9.5, family="monospace", bbox=INFO_BOX)

    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    hy = lut["theta_down_deg"] - lut["theta_up_deg"]
    ax1.axhline(0, color="#888", lw=0.8)
    ax1.fill_between(lut["pulse_us"], 0, hy, color="#7b3294", alpha=0.25)
    ax1.plot(lut["pulse_us"], hy, "-o", ms=3, color="#7b3294", lw=1.3)
    ax1.set_ylabel("hysteresis\n(down−up) [deg]")

    ax2 = fig.add_subplot(gs[2], sharex=ax0)
    resid = pts["theta_deg"] - (gain * pts["pulse_us"] + intercept)
    ax2.axhline(0, color="#888", lw=0.8)
    ax2.plot(pts["pulse_us"], resid, ".", ms=6, color="#ef8a00", label="linear residual")
    if "cross_deg" in pts:
        ax2.plot(pts["pulse_us"], pts["cross_deg"], ".", ms=5, color="#0571b0",
                 alpha=0.6, label="cross-axis coupling")
    ax2.set_ylabel("deg")
    ax2.set_xlabel("servo pulse width  [µs]")
    ax2.legend(fontsize=9, loc="upper right", ncol=2)

    out = run / "mapping_pretty.png"
    fig.savefig(out, dpi=160, bbox_inches="tight")
    if not show:
        plt.close(fig)
    return out


# ===========================================================================
# step: response traces + amplitude panels
# ===========================================================================
def reconstruct_step(run: Path):
    df, meta = load_run(run)
    cal = load_calibration(PROJECT_DIR / "calibration.json")
    s = sample_rows(df).reset_index(drop=True)
    gyro = corrected_gyro(s, meta, cal)
    axis = int(last_value(meta["axis"]))
    gimbal = str(last_value(meta.get("gimbal", GIMBAL_NAMES[axis])))
    pulse_col = "cmd_outer" if axis == 0 else "cmd_inner"
    main = int(np.argmax(np.ptp(gyro, axis=0)))
    ev = event_rows(df)
    arm = {int(r.seq): (r.t, int(r[pulse_col])) for _, r in ev[ev["phase"] == "arm"].iterrows()}
    cmd = {int(r.seq): (r.t, int(r[pulse_col])) for _, r in ev[ev["phase"] == "cmd"].iterrows()}
    traces = []
    for seq, group in s.groupby("seq"):
        seq = int(seq)
        if seq not in arm or seq not in cmd:
            continue
        idx = group.index.to_numpy()
        t = group["t"].to_numpy()
        ctime = cmd[seq][0]
        rate = detrend_rate(t, gyro[idx, main], ctime)
        angle = np.concatenate([[0.0], np.cumsum(np.diff(t) * (rate[1:] + rate[:-1]) / 2.0)])
        tt = t - ctime
        angle -= angle[tt < 0].mean()
        amp_us = cmd[seq][1] - arm[seq][1]
        final = float(np.median(angle[tt >= 0][-max(10, int(FINAL_ANGLE_WINDOW_S / np.median(np.diff(t)))):]))
        traces.append(dict(seq=seq, tt=tt * 1000.0, angle=angle, rate=rate,
                           amp_us=amp_us, final=final))
    return traces, gimbal


def plot_step_one(run: Path, show: bool = False) -> Path:
    traces, gimbal = reconstruct_step(run)
    summ = pd.read_csv(run / "step_summary.csv")
    a = json.loads((run / "analysis.json").read_text(encoding="utf-8"))

    amps = np.array([t["amp_us"] for t in traces], float)
    vmax = np.abs(amps).max() or 1.0
    norm = colors.TwoSlopeNorm(vmin=-vmax, vcenter=0, vmax=vmax)
    cmap = plt.get_cmap("coolwarm")

    apply_plot_style()
    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1], hspace=0.28, wspace=0.24)
    axA = fig.add_subplot(gs[0, 0])
    axR = fig.add_subplot(gs[1, 0], sharex=axA)
    axF = fig.add_subplot(gs[0, 1])
    axS = fig.add_subplot(gs[1, 1])

    for tr in traces:
        c = cmap(norm(tr["amp_us"]))
        m = (tr["tt"] >= -30) & (tr["tt"] <= 800)
        axA.plot(tr["tt"][m], tr["angle"][m], lw=1.2, color=c, alpha=0.9)
        axR.plot(tr["tt"][m], tr["rate"][m], lw=1.0, color=c, alpha=0.85)
    for ax in (axA, axR):
        ax.axvline(0, color="#333", lw=0.8, ls="--")
    axA.set_ylabel("integrated angle [deg]")
    axA.set_title(f"{gimbal.title()} gimbal — step response  (color = signed amplitude)",
                  fontsize=13, weight="bold", pad=10)
    axR.set_ylabel("angular rate [deg/s]")
    axR.set_xlabel("time since command [ms]")
    sm = cm.ScalarMappable(norm=norm, cmap=cmap); sm.set_array([])
    fig.colorbar(sm, ax=[axA, axR], label="commanded amplitude [µs]", pad=0.01, fraction=0.03)

    box = (f"onset {a['direct_onset_ms']:.1f} ms | delay {a['fitted_delay_ms']:.1f} ms\n"
           f"rise 10–90 {a['rise_10_90_ms']:.1f} ms | settle±2% {a['settling_2pct_ms']:.0f} ms\n"
           f"peak slew {a['peak_slew_dps']:.0f} °/s | BW {a['bandwidth_hz']:.1f} Hz\n"
           f"useful {a['useful_step_count']}/{a['step_count']}"
           f" | chirp:{'yes' if a.get('recommend_chirp') else 'no'}")
    axA.text(0.985, 0.03, box, transform=axA.transAxes, ha="right", va="bottom",
             fontsize=9, family="monospace", bbox=INFO_BOX)

    axF.axhline(0, color="#888", lw=0.7); axF.axvline(0, color="#888", lw=0.7)
    axF.scatter(summ["amp_us"], summ["final_deg"], c=summ["amp_us"], cmap=cmap,
                norm=norm, s=45, edgecolor="#333", lw=0.5, zorder=3)
    axF.set_xlabel("commanded amplitude [µs]"); axF.set_ylabel("final angle [deg]")
    axF.set_title("amplitude linearity / symmetry", fontsize=11)

    axS.scatter(summ["amp_us"].abs(), summ["peak_slew_dps"], c=summ["amp_us"],
                cmap=cmap, norm=norm, s=45, edgecolor="#333", lw=0.5, zorder=3)
    axS.set_xlabel("|commanded amplitude| [µs]"); axS.set_ylabel("peak slew [deg/s]")
    axS.set_title("rate saturation", fontsize=11)

    out = run / "step_pretty.png"
    fig.savefig(out, dpi=160, bbox_inches="tight")
    if not show:
        plt.close(fig)
    return out


# ===========================================================================
# bode: chirp frequency response
# ===========================================================================
def plot_bode_one(run: Path, show: bool = False) -> Path:
    r = pd.read_csv(run / "chirp_response.csv")
    a = json.loads((run / "analysis.json").read_text(encoding="utf-8"))
    gimbal = a.get("gimbal", run.name)
    bw = a.get("bandwidth_3db_hz", float("nan"))

    apply_plot_style()
    fig, (axg, axp, axc) = plt.subplots(3, 1, figsize=(10, 9), sharex=True,
                                        gridspec_kw=dict(height_ratios=[2, 1.4, 1]))
    trusted = r["coherence"] >= 0.8

    axg.semilogx(r["freq_hz"], r["gain_db"], color="#bbb", lw=1)
    axg.semilogx(r["freq_hz"][trusted], r["gain_db"][trusted], color="#2166ac", lw=2)
    axg.axhline(-3, color="#b2182b", ls="--", lw=1, label="-3 dB")
    if np.isfinite(bw):
        axg.axvline(bw, color="#b2182b", ls=":", lw=1.2)
        axg.annotate(f"BW {bw:.1f} Hz", (bw, -3), textcoords="offset points",
                     xytext=(6, 8), color="#b2182b", fontsize=10)
    axg.set_ylabel("gain [dB]  (PWM→angle)")
    axg.set_title(f"{gimbal.title()} gimbal — chirp frequency response",
                  fontsize=13, weight="bold", pad=10)
    axg.legend(loc="lower left", fontsize=9)
    box = (f"BW(-3dB) {bw:.2f} Hz\n"
           f"phase delay {a.get('phase_delay_ms', float('nan')):.1f} ms\n"
           f"coherent {a.get('coherent_band_lo_hz', float('nan')):.1f}–"
           f"{a.get('coherent_band_hi_hz', float('nan')):.1f} Hz")
    axg.text(0.985, 0.05, box, transform=axg.transAxes, ha="right", va="bottom",
             fontsize=9, family="monospace", bbox=INFO_BOX)

    axp.semilogx(r["freq_hz"], r["phase_deg"], color="#bbb", lw=1)
    axp.semilogx(r["freq_hz"][trusted], r["phase_deg"][trusted], color="#7b3294", lw=2)
    axp.set_ylabel("phase [deg]")

    axc.semilogx(r["freq_hz"], r["coherence"], color="#1a9850", lw=1.6)
    axc.axhline(0.8, color="#888", ls="--", lw=1)
    axc.set_ylabel("coherence"); axc.set_xlabel("frequency [Hz]"); axc.set_ylim(0, 1.05)

    out = run / "chirp_pretty.png"
    fig.savefig(out, dpi=160, bbox_inches="tight")
    if not show:
        plt.close(fig)
    return out


# ===========================================================================
# deadband: backlash loop near neutral
# ===========================================================================
def plot_deadband_one(run: Path, show: bool = False) -> Path:
    p = pd.read_csv(run / "deadband_points.csv")
    a = json.loads((run / "analysis.json").read_text(encoding="utf-8"))
    gimbal = a.get("gimbal", run.name)

    apply_plot_style()
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(p["pulse_us"], p["theta_up_deg"], "-o", ms=4, color="#2166ac", label="ascending")
    ax.plot(p["pulse_us"], p["theta_dn_deg"], "-o", ms=4, color="#b2182b", label="descending")
    ax.fill_between(p["pulse_us"], p["theta_up_deg"], p["theta_dn_deg"],
                    color="#7b3294", alpha=0.15, label="backlash loop")
    ax.set_xlabel("servo pulse width [µs]")
    ax.set_ylabel("gimbal tilt angle [deg]")
    ax.set_title(f"{gimbal.title()} gimbal — deadband / backlash near neutral",
                 fontsize=13, weight="bold", pad=10)
    ax.legend(loc="upper left", fontsize=10)
    box = (f"backlash = {a.get('backlash_us', float('nan')):.1f} µs "
           f"({a.get('backlash_deg', float('nan')):.3f}°)\n"
           f"local gain = {a.get('local_gain_deg_per_us', float('nan'))*1000:.1f} m°/µs\n"
           f"travel span = {a.get('travel_span_deg', float('nan')):.2f}°")
    ax.text(0.985, 0.05, box, transform=ax.transAxes, ha="right", va="bottom",
            fontsize=10, family="monospace", bbox=INFO_BOX)

    out = run / "deadband_pretty.png"
    fig.savefig(out, dpi=160, bbox_inches="tight")
    if not show:
        plt.close(fig)
    return out


# ===========================================================================
# per-test subcommand driver
# ===========================================================================
def _run_pertest(args, subdirs, required_file, plotter, label) -> int:
    targets = resolve_run_targets(args, subdirs, required_file)
    if not targets:
        raise SystemExit(f"no {label} runs with data under runs/")
    for run in targets:
        print("saved", plotter(run, show=args.show))
    if args.show:
        plt.show()
    return 0


def _add_pertest_args(sp) -> None:
    sp.add_argument("--session", type=Path, help="session folder")
    sp.add_argument("--run", type=Path, help="single run folder")
    sp.add_argument("--runs", type=Path, default=PROJECT_DIR / "runs")
    sp.add_argument("--show", action="store_true", help="interactive window")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    ss = sub.add_parser("surface", help="generic cubic 3D surface from any grid CSV")
    ss.add_argument("csv", type=Path, help="CSV with the x, y, z columns")
    ss.add_argument("--x", required=True, help="x column name")
    ss.add_argument("--y", required=True, help="y column name")
    ss.add_argument("--z", required=True, help="z column name")
    ss.add_argument("--xlabel"); ss.add_argument("--ylabel"); ss.add_argument("--zlabel")
    ss.add_argument("--title")
    ss.add_argument("--cmap", default="viridis")
    ss.add_argument("--out", type=Path, help="output PNG (default: <csv>_surface.png)")
    ss.add_argument("--resolution", type=int, default=40)
    ss.add_argument("--show", action="store_true")

    sg = sub.add_parser("grid", help="gimbal joint map: pitch + yaw cubic surfaces")
    sg.add_argument("--run", type=Path, help="grid run folder with grid_points.csv")
    sg.add_argument("--from-mapping", action="store_true", dest="from_mapping",
                    help="reconstruct from the two per-axis mapping runs")
    sg.add_argument("--outer", type=Path, help="outer mapping run (with --from-mapping)")
    sg.add_argument("--inner", type=Path, help="inner mapping run (with --from-mapping)")
    sg.add_argument("--demo", action="store_true", help="synthesize a coupled grid")
    sg.add_argument("--runs", type=Path, default=PROJECT_DIR / "runs")
    sg.add_argument("--points", type=int, default=11, help="demo/reconstruction resolution")
    sg.add_argument("--show", action="store_true")

    for name in ("mapping", "step", "bode", "deadband"):
        _add_pertest_args(sub.add_parser(name, help=f"{name} plot"))

    args = ap.parse_args()

    if args.cmd == "surface":
        df = pd.read_csv(args.csv)
        out = args.out or args.csv.with_name(args.csv.stem + "_surface.png")
        plot_surface(df, args.x, args.y, args.z, out, title=args.title,
                     xlabel=args.xlabel, ylabel=args.ylabel, zlabel=args.zlabel,
                     cmap=plt.get_cmap(args.cmap), show=args.show,
                     resolution=args.resolution)
        return 0

    if args.cmd == "grid":
        if args.from_mapping:
            if args.outer and args.inner:
                outer_dir, inner_dir = args.outer, args.inner
            else:
                outer_dir, inner_dir = find_mapping_pair(args.runs)
            print("outer:", outer_dir, "\ninner:", inner_dir)
            df = grid_from_mapping(outer_dir, inner_dir, n=args.points)
            out_dir = PROJECT_DIR / "grid_from_mapping"
            out_dir.mkdir(parents=True, exist_ok=True)
            df.to_csv(out_dir / "grid_points.csv", index=False)
            render_grid_surfaces(df, out_dir, "Reconstructed  ", args.show)
        elif args.demo:
            df = synth_grid(n=args.points)
            out_dir = PROJECT_DIR / "grid_demo"
            out_dir.mkdir(parents=True, exist_ok=True)
            df.to_csv(out_dir / "grid_points.csv", index=False)
            render_grid_surfaces(df, out_dir, "Demo  ", args.show)
        else:
            run = args.run or newest_grid_run(args.runs)
            df = load_grid(run / "grid_points.csv")
            render_grid_surfaces(df, run, "", args.show)
        if args.show:
            plt.show()
        return 0

    plotters = {
        "mapping": (("mapping_outer_gimbal", "mapping_inner_gimbal"),
                    "mapping_points.csv", plot_mapping_one),
        "step": (("step_outer_gimbal", "step_inner_gimbal"),
                 "step_summary.csv", plot_step_one),
        "bode": (("chirp_outer_gimbal", "chirp_inner_gimbal"),
                 "chirp_response.csv", plot_bode_one),
        "deadband": (("deadband_outer_gimbal", "deadband_inner_gimbal"),
                     "deadband_points.csv", plot_deadband_one),
    }
    subdirs, required, plotter = plotters[args.cmd]
    return _run_pertest(args, subdirs, required, plotter, args.cmd)


if __name__ == "__main__":
    raise SystemExit(main())
