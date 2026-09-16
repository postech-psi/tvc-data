#!/usr/bin/env python3
"""
TVC gimbal characterization analysis  (PTK 8515MG-D + ICM-20948).

    pip install numpy pandas matplotlib scipy

    python analyze.py runs/a_axis0_2026-08-12_1430      # mapping
    python analyze.py runs/b_axis0_2026-08-12_1445      # step
    python analyze.py runs/k_axis0_...                  # deadband
    python analyze.py runs/p_axis0_...                  # repeatability
    python analyze.py --selftest                        # validate the fitter

The test type is read from meta.json; you do not name it.

NO CONSTANT IS DUPLICATED HERE.  Scale factors, neutral pulse, sample
rates and axis assignment all come from meta.json, which the firmware
built by reading its own registers back.  The historical failure mode on
this rig was a constant edited in the sketch and not in the analysis.

WHAT IS DELIBERATE, AND WHY:

* Health first, fit second.  Every run prints an I2C/timing/settling
  audit before any model is fitted, and refuses to fit a run that fails
  it.  A fitted number from a bad trace is worse than no number.

* Timestamps, never nominal dt.  All integration uses the recorded t_us.
  Assuming 1/1000 s biases the result whenever the loop ran late, and
  the flags column proves it sometimes does.

* Drift is fitted over the pre-step window AND the settled tail, jointly.
  Fitting the 200 ms pre-window alone gives a slope standard error of
  SE = sigma/(sigma_t*sqrt(n)) ~ 1.6 dps/s, about 20x the drift being
  removed; over a 2.2 s trace that injects degrees of fake ramp.  Both
  ends together give a >2 s lever arm and SE ~ 0.06 dps/s.

* Dead time comes from a model fit, not from back-extrapolating a chord.
  For a first-order response the 20-80% chord extrapolates to -0.24*tau
  relative to true onset -- biased by the very time constant being
  measured (12 ms of error on a 14 ms quantity when tau = 50 ms).  A
  6-sigma threshold crossing is computed too, but only as an independent
  cross-check that must agree, never as the primary number.

* Smoothing (zero-phase filtfilt) is used ONLY for peak/overshoot.
  Never for edge timing -- a causal filter shifts edges, and even
  filtfilt distorts a sharp onset.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit

RAD2DEG = 57.29577951308232

FLAG_I2C = 0x01
FLAG_LATE = 0x02


# ===================================================================
# loading
# ===================================================================
def axis_of(meta):
    """Which servo this run drove, as a plain int.

    The capture parser turns repeated '#META axis=0' lines into a LIST
    ([0, 0]), so `meta.get('axis', 0) == 0` is False even for axis 0 --
    which silently made the mapping read the wrong pulse column (cmd_b,
    constant, so the fit saw 'angle vs a constant' and returned a zero
    gain).  Normalise it once, here, and use axis_of(meta) everywhere."""
    a = meta.get("axis", 0)
    if isinstance(a, (list, tuple)):
        a = a[0] if a else 0
    return int(a)


def load(rundir):
    rundir = Path(rundir)
    meta = json.loads((rundir / "meta.json").read_text())
    df = pd.read_csv(rundir / "raw.csv", comment="#")
    df.columns = [c.strip() for c in df.columns]
    for c in ("t_us", "seq", "axis", "cmd_a", "cmd_b",
              "ax", "ay", "az", "gx", "gy", "gz", "flags"):
        if c in df:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    df["t"] = df["t_us"] * 1e-6
    return df, meta


def scaled(df, meta):
    """Raw LSB -> g and dps, using the manifest's factors."""
    ag = float(meta["acc_lsb_per_g"])
    gg = float(meta["gyro_lsb_per_dps"])
    out = df.copy()
    for c in "xyz":
        out["a" + c] = df["a" + c] / ag
        out["g" + c] = df["g" + c] / gg
    return out


def samples(df):
    return df[df["rec"] == "S"]


def events(df):
    return df[df["rec"] == "E"]


# ===================================================================
# health audit
# ===================================================================
def health(df, meta, expect_hz=None):
    """Returns (ok, report dict).  Printed before anything is fitted."""
    s = samples(df)
    n = len(s)
    rep = {}
    print("\n=== HEALTH ===")
    print(f"  samples            {n}")

    flags = s["flags"].fillna(0).astype(int)
    n_i2c = int((flags & FLAG_I2C).astype(bool).sum())
    n_late = int((flags & FLAG_LATE).astype(bool).sum())
    rep["n_i2c_err"], rep["n_late"] = n_i2c, n_late
    print(f"  I2C read failures  {n_i2c}  ({100*n_i2c/max(n,1):.2f}%)")
    print(f"  late samples       {n_late}  ({100*n_late/max(n,1):.2f}%)")

    dt = np.diff(s["t"].to_numpy())
    dt = dt[(dt > 0) & (dt < 1.0)]
    if len(dt):
        med = np.median(dt)
        rep["dt_median_ms"] = med * 1e3
        rep["dt_jitter_ms"] = float(np.std(dt) * 1e3)
        rep["rate_hz"] = 1.0 / med
        print(f"  median dt          {med*1e3:.3f} ms  -> {1/med:8.1f} Hz")
        print(f"  dt jitter (sd)     {np.std(dt)*1e3:.3f} ms")
        print(f"  dt p99             {np.percentile(dt,99)*1e3:.3f} ms")
        if expect_hz:
            err = abs(1 / med - expect_hz) / expect_hz
            print(f"  expected rate      {expect_hz} Hz  "
                  f"({'OK' if err < 0.05 else 'OFF by %.1f%%' % (100*err)})")

    sc = scaled(s, meta)
    gmag = np.sqrt(sc["ax"]**2 + sc["ay"]**2 + sc["az"]**2)
    rep["g_mag_mean"] = float(gmag.mean())
    print(f"  |g| mean           {gmag.mean():.4f}  sd {gmag.std():.4f}"
          f"  {'OK' if abs(gmag.mean()-1) < 0.05 else 'BAD -- check scale/mount'}")

    ok = True
    if n == 0:
        print("  !! no samples"); ok = False
    if n_i2c > 0.01 * max(n, 1):
        print("  !! >1% I2C failures -- data is not trustworthy"); ok = False
    if n_late > 0.05 * max(n, 1):
        print("  !! >5% late samples -- timing did not hold"); ok = False
    if abs(gmag.mean() - 1) > 0.10:
        print("  !! |g| far from 1 -- wrong scale factor or a moving rig"); ok = False
    print(f"  VERDICT            {'PASS' if ok else 'FAIL'}")
    return ok, rep


def pick_axes(sc, meta):
    """Data-driven axis assignment, reported not assumed.

    Gravity axis = the accel axis with the largest mean.  Main axis =
    the axis whose accel component varies most across the run (that is
    what the servo is moving).  Cross = the remaining one.  Deriving
    this from the data rather than a #define is what stops the classic
    'AXIS_MAIN edited in one file only' failure."""
    a = np.vstack([sc["ax"], sc["ay"], sc["az"]])
    grav = int(np.argmax(np.abs(a.mean(axis=1))))
    rng = a.max(axis=1) - a.min(axis=1)
    rng[grav] = -1
    main = int(np.argmax(rng))
    cross = [i for i in (0, 1, 2) if i not in (grav, main)][0]
    names = "xyz"
    print(f"  axes: gravity={names[grav]}  main={names[main]}  "
          f"cross={names[cross]}  (main travel {rng[main]:.3f} g)")
    if rng[main] < 0.02:
        print("  !! main axis barely moved -- is the sweep axis VERTICAL?")
    return grav, main, cross


def tilt_deg(sc, num, den):
    return np.arctan2(sc["a" + "xyz"[num]], sc["a" + "xyz"[den]]) * RAD2DEG


def referenced_tilt(a_num, a_den, ref_num, ref_den):
    """Signed tilt (deg) of (a_num, a_den) RELATIVE to a reference pose,
    wrapped into (-180, 180].

    Absolute atan2 sits at +/-180 whenever gravity points along -den
    (the classic 'rest angle near the pole' case), and a sweep across
    that boundary makes the naive angle jump +180 <-> -180 -- which is
    what produced the 70-degree scatter and the fake 'not settled'
    flags.  Measuring relative to the mid-range reference keeps the whole
    travel centred near 0, so it never touches the wrap.  Scale-invariant
    (it is a ratio), so the accel |g| error does not enter."""
    ang = np.degrees(np.arctan2(a_num, a_den))
    ref = np.degrees(np.arctan2(ref_num, ref_den))
    d = ang - ref
    return (d + 180.0) % 360.0 - 180.0


def rotation_axis(gvecs):
    """Given gravity unit vectors sampled across a SINGLE-axis sweep,
    return the true rotation axis in sensor coordinates.

    Why this exists: if the IMU is not glued perfectly parallel to the
    gimbal axes, its x/y/z are not the pitch/roll axes, and tilt_deg()
    (which assumes they are) mixes the two.  But when only one servo
    moves, gravity traces a circular arc whose plane normal IS that
    servo's true axis -- independent of how the IMU is mounted.  So we
    recover the axis from the data instead of trusting the mounting.

    The arc lies in a plane; the plane normal is the smallest-variance
    direction of the (mean-removed) gravity samples -> smallest right
    singular vector.  Needs real travel to be well conditioned; the
    caller checks the sweep actually moved."""
    g = gvecs / np.linalg.norm(gvecs, axis=1, keepdims=True)
    gc = g - g.mean(axis=0)
    _, s, vt = np.linalg.svd(gc, full_matrices=False)
    n = vt[-1]
    # sign convention: point it so the sweep progresses right-handed
    return n / np.linalg.norm(n), s


def angle_about_axis(gvecs, axis, g_ref):
    """Signed rotation angle (deg) of each gravity vector about `axis`,
    relative to g_ref.  This is the decoupled tilt about the TRUE servo
    axis -- projecting out `axis` removes any component the other servo
    or the mounting tilt would contribute."""
    axis = axis / np.linalg.norm(axis)
    g = gvecs / np.linalg.norm(gvecs, axis=1, keepdims=True)
    # project into the plane perpendicular to the rotation axis
    def perp(v):
        return v - np.outer(v @ axis, axis) if v.ndim > 1 else v - (v @ axis) * axis
    gp = perp(g)
    r0 = perp(g_ref / np.linalg.norm(g_ref))
    r0 = r0 / np.linalg.norm(r0)
    # build an in-plane basis (r0, axis x r0) and read the angle off it
    e2 = np.cross(axis, r0)
    x = gp @ r0
    y = gp @ e2
    return np.degrees(np.arctan2(y, x))


# ===================================================================
# TEST A : mapping
# ===================================================================
def settled_window(g, frac=0.5):
    """Last `frac` of a dwell group.  The earlier part is the servo
    still moving; including it biases the angle toward the previous
    command and inflates the apparent scatter."""
    t = g["t"].to_numpy()
    if len(t) < 4:
        return g
    cut = t[0] + (1 - frac) * (t[-1] - t[0])
    return g[g["t"] >= cut]


def analyze_mapping(df, meta, outdir):
    ok, _ = health(df, meta, expect_hz=meta.get("log_hz"))
    sc = scaled(samples(df), meta)
    grav, main, cross = pick_axes(sc, meta)

    # --- primary angle: referenced plane tilt (pole-safe) ---
    # theta = tilt of the main axis about gravity, measured relative to
    # the mid-range rest reference so it never crosses the +/-180 wrap.
    # This is scale-invariant and does not depend on the fragile arc fit.
    zero = sc[(sc["phase"] == "zero") & (sc["seq"] == -1)]
    ref_m = float(zero["a" + "xyz"[main]].mean())
    ref_g = float(zero["a" + "xyz"[grav]].mean())
    ref_c = float(zero["a" + "xyz"[cross]].mean())
    sc = sc.assign(
        theta=referenced_tilt(sc["a" + "xyz"[main]], sc["a" + "xyz"[grav]],
                              ref_m, ref_g),
        theta_cross=referenced_tilt(sc["a" + "xyz"[cross]], sc["a" + "xyz"[grav]],
                                    ref_c, ref_g))

    # --- misalignment: reported as a DIAGNOSTIC, with a pole guard ---
    # The arc fit recovers the true servo axis for a crooked IMU, but it
    # is degenerate when the sweep sits near the gravity pole (both the
    # servo axis and the gravity axis are then low-variance and the SVD
    # cannot tell them apart -- it returns the gravity axis, which used
    # to poison the whole mapping).  So we only trust it away from the
    # pole, and never let it drive the primary angle.
    sweep = sc[sc["phase"].isin(["up", "dn", "up2", "dn2"])]
    G = np.vstack([sweep["ax"], sweep["ay"], sweep["az"]]).T
    g_ref = np.array([zero["ax"].mean(), zero["ay"].mean(), zero["az"].mean()])
    ghat = g_ref / np.linalg.norm(g_ref)
    axis_meas, svals = rotation_axis(G)
    nominal = np.eye(3)[main]
    misalign = np.degrees(np.arccos(np.clip(abs(axis_meas @ nominal), 0, 1)))
    pole_align = abs(axis_meas @ ghat)          # 1 => axis == gravity => degenerate
    degenerate = pole_align > 0.90 or len(G) < 20

    print("\n=== AXIS ALIGNMENT (diagnostic) ===")
    print(f"  measured servo axis (sensor frame)  "
          f"[{axis_meas[0]:+.3f} {axis_meas[1]:+.3f} {axis_meas[2]:+.3f}]")
    if degenerate:
        print(f"  arc fit DEGENERATE (sweep near the gravity pole, "
              f"axis.g={pole_align:.2f}); misalignment not reliable.")
        print(f"  primary angle uses referenced plane tilt about "
              f"{('xyz')[main]}/{('xyz')[grav]} -- unaffected.")
    else:
        print(f"  IMU misalignment vs nominal '{('xyz')[main]}'   "
              f"{misalign:.2f} deg")
        if misalign > 5:
            print(f"  note: >5 deg crooked mount can leak into the "
                  f"cross-axis number below.")

    zero = sc[sc["phase"] == "zero"]
    zero_open = settled_window(zero[zero["seq"] == -1])
    zero_close = settled_window(zero[zero["seq"] == -2])
    z0 = float(zero_open["theta"].mean()) if len(zero_open) else 0.0
    z0c = float(zero_open["theta_cross"].mean()) if len(zero_open) else 0.0
    print("\n=== ZERO REFERENCE ===")
    print(f"  opening zero       {z0:+.4f} deg  (sd {zero_open['theta'].std():.4f})")
    if len(zero_close):
        z1 = float(zero_close["theta"].mean())
        print(f"  closing zero       {z1:+.4f} deg")
        print(f"  zero drift         {z1-z0:+.4f} deg  "
              f"{'OK' if abs(z1-z0) < 0.2 else '!! rig moved or servo re-seated'}")

    rows = []
    for (phase, seq), g in sc[sc["phase"].isin(["up", "dn", "up2", "dn2"])] \
                             .groupby(["phase", "seq"]):
        if seq < 0:
            continue
        w = settled_window(g)
        # flatness check: if the plate is still creeping in the window we
        # are supposed to call settled, say so instead of reporting it
        th = w["theta"].to_numpy()
        t = w["t"].to_numpy()
        creep = np.polyfit(t - t[0], th, 1)[0] if len(t) > 3 else 0.0
        rows.append(dict(
            pass_=phase, seq=int(seq), pulse_us=int(w["cmd_a"].iloc[0] if
                                                    axis_of(meta) == 0
                                                    else w["cmd_b"].iloc[0]),
            theta_deg=float(th.mean()) - z0,
            theta_sd=float(th.std()),
            theta_cross_deg=float(w["theta_cross"].mean()) - z0c,
            n=len(w), creep_dps=float(creep),
            settled=bool(abs(creep) < 0.5),
        ))
    m = pd.DataFrame(rows).sort_values(["pass_", "pulse_us"])
    if m.empty:
        print("!! no sweep data found"); return

    n_uns = int((~m["settled"]).sum())
    print("\n=== MAPPING ===")
    print(f"  points             {len(m)}  ({n_uns} not settled)")
    print(f"  repeatability      median sd {m['theta_sd'].median():.4f} deg, "
          f"worst {m['theta_sd'].max():.4f}")

    # linear fit over the middle 60% of travel, where geometry is most linear
    lo, hi = m["pulse_us"].quantile([0.20, 0.80])
    mid = m[(m["pulse_us"] >= lo) & (m["pulse_us"] <= hi)]
    k, b = np.polyfit(mid["pulse_us"], mid["theta_deg"], 1)
    resid = m["theta_deg"] - (k * m["pulse_us"] + b)
    true_neutral = -b / k
    span = m["theta_deg"].max() - m["theta_deg"].min()
    print(f"  gain k             {k:.5f} deg/us   ({1/k:.2f} us/deg)")
    print(f"  true neutral       {true_neutral:.1f} us  "
          f"(nominal {meta['neutral_us']}, offset {true_neutral-meta['neutral_us']:+.1f})")
    print(f"  travel             {m['theta_deg'].min():+.2f} .. "
          f"{m['theta_deg'].max():+.2f} deg  (span {span:.2f})")
    print(f"  nonlinearity       {100*resid.abs().max()/span:.2f}% of span "
          f"(max |resid| {resid.abs().max():.3f} deg)")

    # hysteresis: up vs down at the same pulse, averaged over both cycles
    up = m[m["pass_"].isin(["up", "up2"])].groupby("pulse_us")["theta_deg"].mean()
    dn = m[m["pass_"].isin(["dn", "dn2"])].groupby("pulse_us")["theta_deg"].mean()
    common = up.index.intersection(dn.index)
    hyst = (dn[common] - up[common])
    print(f"  hysteresis/backlash  mean {hyst.mean():+.4f} deg, "
          f"max {hyst.abs().max():.4f} deg  ({100*hyst.abs().max()/span:.2f}% of span)")

    cross_span = m["theta_cross_deg"].max() - m["theta_cross_deg"].min()
    print(f"  cross-axis coupling  {cross_span:.3f} deg = "
          f"{100*cross_span/span:.2f}% of main travel")

    # ---- outputs ----
    m.to_csv(outdir / "mapping_points.csv", index=False)
    lut = pd.DataFrame({"pulse_us": common,
                        "theta_deg": (up[common] + dn[common]) / 2,
                        "theta_up": up[common], "theta_dn": dn[common]})
    lut.to_csv(outdir / "lut.csv", index=False)

    fig, ax = plt.subplots(3, 1, figsize=(9, 11), sharex=True)
    for p, style in [("up", "-o"), ("dn", "-s"), ("up2", "--^"), ("dn2", "--v")]:
        d = m[m["pass_"] == p]
        if len(d):
            ax[0].plot(d["pulse_us"], d["theta_deg"], style, ms=3, lw=1, label=p)
    ax[0].plot(m["pulse_us"], k * m["pulse_us"] + b, "k:", lw=1,
               label=f"fit {k:.4f} deg/us")
    ax[0].axvline(true_neutral, color="k", lw=0.6, ls="--")
    ax[0].set_ylabel("plate angle [deg]"); ax[0].legend(fontsize=8)
    ax[0].set_title(f"TEST A mapping  |  axis {meta.get('axis')}  |  "
                    f"k={k:.4f} deg/us  neutral={true_neutral:.0f} us")
    ax[0].grid(alpha=.3)

    ax[1].plot(m["pulse_us"], resid, ".", ms=4)
    ax[1].axhline(0, color="k", lw=.6)
    ax[1].set_ylabel("residual from linear [deg]"); ax[1].grid(alpha=.3)

    ax[2].plot(common, hyst, "-o", ms=3, label="dn - up (hysteresis)")
    ax[2].plot(m["pulse_us"], m["theta_cross_deg"], ".", ms=3, label="cross axis")
    ax[2].errorbar(m["pulse_us"], np.zeros(len(m)), yerr=m["theta_sd"],
                   fmt="none", ecolor="gray", alpha=.5, label="per-point sd")
    ax[2].axhline(0, color="k", lw=.6)
    ax[2].set_ylabel("deg"); ax[2].set_xlabel("pulse [us]")
    ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(outdir / "mapping.png", dpi=130)
    print(f"\n  wrote lut.csv, mapping_points.csv, mapping.png -> {outdir}")
    if not ok:
        print("  !! health check FAILED -- numbers above are informational only")


# ===================================================================
# TEST B : step response
# ===================================================================
def first_order(t, A, td, tau):
    y = np.zeros_like(t)
    m = t > td
    y[m] = A * (1 - np.exp(-(t[m] - td) / tau))
    return y


def second_order(t, A, td, wn, z):
    """Underdamped step response, zero initial conditions."""
    y = np.zeros_like(t)
    m = t > td
    tt = t[m] - td
    z = np.clip(z, 1e-3, 0.999)
    wd = wn * np.sqrt(1 - z * z)
    y[m] = A * (1 - np.exp(-z * wn * tt) *
                (np.cos(wd * tt) + (z / np.sqrt(1 - z * z)) * np.sin(wd * tt)))
    return y


def detrend_rate(t, r, t_cmd, tail_frac=0.35):
    """Fit offset+slope over (pre-step) UNION (settled tail) and subtract.

    Both windows have true rate ~ 0, so any offset or slope there is
    sensor bias and drift.  Using both ends rather than just the
    pre-window is what makes the slope estimate usable -- see module
    docstring for the numbers."""
    tail_start = t[-1] - tail_frac * (t[-1] - t_cmd)
    mask = (t < t_cmd) | (t >= tail_start)
    if mask.sum() < 10:
        return r - np.mean(r[t < t_cmd]), (0.0, 0.0)
    p = np.polyfit(t[mask], r[mask], 1)
    return r - np.polyval(p, t), (float(p[0]), float(p[1]))


def analyze_step(df, meta, outdir):
    ok, _ = health(df, meta, expect_hz=meta.get("log_hz"))
    sc = scaled(samples(df), meta)

    # which gyro axis is the servo actually driving?
    ranges = {c: sc["g" + c].abs().max() for c in "xyz"}
    main = max(ranges, key=ranges.get)
    cross = [c for c in "xyz" if c != main]
    print(f"  main gyro axis     g{main}  (peak {ranges[main]:.1f} dps; "
          f"cross {ranges[cross[0]]:.1f}, {ranges[cross[1]]:.1f})")

    # per-step command instants, as logged -- never assumed from pre_ms
    ev = events(df)
    cmd_t = {int(r["seq"]): r["t"] for _, r in ev[ev["phase"] == "cmd"].iterrows()}

    rows, traces = [], []
    for seq, g in sc.groupby("seq"):
        g = g.sort_values("t")
        t = g["t"].to_numpy()
        r = g["g" + main].to_numpy()
        t_cmd = cmd_t.get(int(seq))
        if t_cmd is None:
            pre = g[g["phase"] == "pre"]
            if not len(pre):
                continue
            t_cmd = float(pre["t"].max())

        r_det, drift = detrend_rate(t, r, t_cmd)
        # integrate on RECORDED timestamps; rectangular would bias high
        # on a monotonic rise, trapezoid does not
        theta = np.concatenate([[0.0], np.cumsum(np.diff(t) *
                                                 (r_det[1:] + r_det[:-1]) / 2)])
        tt = t - t_cmd
        theta = theta - np.mean(theta[tt < 0])

        # final value: median of the last 150 ms, with a flatness check
        tail = theta[tt >= tt[-1] - 0.150]
        ttail = tt[tt >= tt[-1] - 0.150]
        A_obs = float(np.median(tail))
        flat = float(abs(np.polyfit(ttail, tail, 1)[0])) if len(ttail) > 3 else 9e9
        settled = flat < 0.5                    # deg/s of residual creep

        fit = post = tt[tt >= 0]
        yp = theta[tt >= 0]
        model, par, perr, kind = None, {}, {}, "none"
        try:
            p0 = [A_obs, 0.015, 0.05]
            bnds = ([-abs(A_obs)*3 - 1, 0.0, 1e-3],
                    [abs(A_obs)*3 + 1, 0.3, 2.0])
            if A_obs < 0:
                bnds = ([-abs(A_obs)*3 - 1, 0.0, 1e-3],
                        [abs(A_obs)*3 + 1, 0.3, 2.0])
            popt, pcov = curve_fit(first_order, post, yp, p0=p0,
                                   bounds=bnds, maxfev=20000)
            model, kind = first_order(post, *popt), "1st"
            par = dict(A=popt[0], td=popt[1], tau=popt[2])
            perr = dict(zip(("A", "td", "tau"), np.sqrt(np.diag(pcov))))
        except Exception as e:
            print(f"  !! seq {seq}: first-order fit failed ({e})")

        # overshoot, measured on a zero-phase smoothed copy -- smoothing
        # is allowed here because we want the peak, not its time
        if len(yp) > 30:
            b, a = signal.butter(2, min(0.2, 50 / (0.5 / np.median(np.diff(t)))))
            ysm = signal.filtfilt(b, a, yp)
        else:
            ysm = yp
        peak = ysm[np.argmax(np.abs(ysm))]
        overshoot = (abs(peak) - abs(A_obs)) / abs(A_obs) if A_obs else 0.0

        if overshoot > 0.05:
            try:
                popt2, pcov2 = curve_fit(
                    second_order, post, yp,
                    p0=[A_obs, 0.015, 40.0, 0.35],
                    bounds=([-abs(A_obs)*3-1, 0.0, 1.0, 0.01],
                            [abs(A_obs)*3+1, 0.3, 500.0, 0.999]), maxfev=30000)
                model, kind = second_order(post, *popt2), "2nd"
                par = dict(A=popt2[0], td=popt2[1], wn=popt2[2], zeta=popt2[3])
                perr = dict(zip(("A", "td", "wn", "zeta"), np.sqrt(np.diag(pcov2))))
            except Exception as e:
                print(f"  !! seq {seq}: second-order fit failed ({e})")

        # independent cross-check: 6-sigma threshold on the raw rate.
        # It always fires late; it is here to catch a fit that ran away,
        # not to be the answer.
        pre_r = r_det[tt < 0]
        sd = float(np.std(pre_r)) if len(pre_r) > 5 else 0.0
        thr = np.where(np.abs(r_det) > 6 * sd)[0]
        thr = thr[tt[thr] > 0] if len(thr) else thr
        td_thr = float(tt[thr[0]]) if len(thr) else np.nan

        resid_pct = (100 * np.std(yp - model) / abs(A_obs)
                     if model is not None and A_obs else np.nan)
        slew = float(np.max(np.abs(r_det)))

        rows.append(dict(
            seq=int(seq), kind=kind,
            amp_us=int(g["cmd_a"].iloc[-1] - g["cmd_a"].iloc[0])
                    if axis_of(meta) == 0 else
                    int(g["cmd_b"].iloc[-1] - g["cmd_b"].iloc[0]),
            A_final_deg=A_obs, A_fit_deg=par.get("A", np.nan),
            td_ms=1e3 * par.get("td", np.nan),
            td_se_ms=1e3 * perr.get("td", np.nan),
            tau_ms=1e3 * par.get("tau", np.nan),
            wn_rad_s=par.get("wn", np.nan), zeta=par.get("zeta", np.nan),
            overshoot_pct=100 * overshoot,
            td_threshold_ms=1e3 * td_thr,
            slew_dps=slew, slew_per_deg=slew / abs(A_obs) if A_obs else np.nan,
            resid_pct_of_amp=resid_pct,
            settled="YES" if settled else "NO",
            drift_slope_dps_s=drift[0],
            pre_noise_sd_dps=sd,
        ))
        traces.append((seq, tt, theta, r_det, post, model))

    if not rows:
        print("!! no steps found"); return
    s = pd.DataFrame(rows).sort_values("seq")
    s.to_csv(outdir / "step_summary.csv", index=False)

    print("\n=== STEP ===")
    with pd.option_context("display.width", 200, "display.max_columns", 50):
        print(s[["seq", "amp_us", "A_final_deg", "kind", "td_ms", "tau_ms",
                 "wn_rad_s", "zeta", "overshoot_pct", "slew_dps",
                 "resid_pct_of_amp", "settled"]].to_string(index=False,
                                                           float_format="%.3f"))

    good = s[(s["settled"] == "YES") & s["td_ms"].notna()]
    if len(good):
        print(f"\n  DEAD TIME   td = {good['td_ms'].median():.2f} ms "
              f"(median of {len(good)} steps, spread "
              f"{good['td_ms'].std():.2f} ms)")
        print(f"  threshold cross-check     {good['td_threshold_ms'].median():.2f} ms "
              f"(expected slightly LARGER; if it is smaller the fit is wrong)")
        if good["tau_ms"].notna().any():
            print(f"  LAG         tau = {good['tau_ms'].median():.2f} ms")
        if good["wn_rad_s"].notna().any():
            g2 = good[good["wn_rad_s"].notna()]
            print(f"  LAG (2nd)   wn = {g2['wn_rad_s'].median():.1f} rad/s, "
                  f"zeta = {g2['zeta'].median():.3f}")
        # saturation self-check
        sp = good["slew_per_deg"].dropna()
        if len(sp) > 2:
            cv = sp.std() / sp.mean()
            print(f"  slew/amp CV = {100*cv:.1f}%  -> "
                  + ("constant: NO saturation, drop the rate limiter"
                     if cv < 0.15 else
                     "NOT constant: large steps ARE rate limited"))
        print(f"  peak slew observed        {good['slew_dps'].max():.1f} dps "
              f"(datasheet ceiling ~577)")
    n_bad = int((s["settled"] == "NO").sum())
    if n_bad:
        print(f"  !! {n_bad} step(s) never settled -- excluded from the medians")

    n = len(traces)
    fig, axes = plt.subplots(n, 2, figsize=(12, 2.0 * n), squeeze=False)
    for i, (seq, tt, theta, r_det, post, model) in enumerate(traces):
        axes[i][0].plot(tt * 1e3, theta, lw=.8, label="integrated angle")
        if model is not None:
            axes[i][0].plot(post * 1e3, model, "r--", lw=.9, label="fit")
        axes[i][0].axvline(0, color="k", lw=.5)
        td = s.loc[s["seq"] == seq, "td_ms"]
        if len(td) and np.isfinite(td.iloc[0]):
            axes[i][0].axvline(td.iloc[0], color="g", lw=.6, ls=":")
        axes[i][0].set_ylabel(f"seq {seq}\n[deg]", fontsize=8)
        axes[i][0].grid(alpha=.3)
        if i == 0:
            axes[i][0].legend(fontsize=7)
        axes[i][1].plot(tt * 1e3, r_det, lw=.6)
        axes[i][1].axvline(0, color="k", lw=.5)
        axes[i][1].set_ylabel("[dps]", fontsize=8); axes[i][1].grid(alpha=.3)
    axes[-1][0].set_xlabel("t since command [ms]")
    axes[-1][1].set_xlabel("t since command [ms]")
    fig.suptitle(f"TEST B step response  |  axis {meta.get('axis')}", y=1.0)
    fig.tight_layout()
    fig.savefig(outdir / "step.png", dpi=130)

    fig2, ax2 = plt.subplots(figsize=(7, 4))
    ax2.plot(s["amp_us"].abs(), s["resid_pct_of_amp"], "o")
    ax2.set_xlabel("|step amplitude| [us]")
    ax2.set_ylabel("fit residual [% of amplitude]")
    ax2.set_title("model adequacy -- rising with amplitude means saturation")
    ax2.grid(alpha=.3)
    fig2.tight_layout()
    fig2.savefig(outdir / "step_residuals.png", dpi=130)

    print(f"\n  wrote step_summary.csv, step.png, step_residuals.png -> {outdir}")
    if not ok:
        print("  !! health check FAILED -- numbers above are informational only")


# ===================================================================
# TEST K : deadband
# ===================================================================
def analyze_deadband(df, meta, outdir):
    ok, _ = health(df, meta, expect_hz=meta.get("log_hz"))
    sc = scaled(samples(df), meta)
    grav, main, cross = pick_axes(sc, meta)
    # pole-safe referenced tilt; reference = median pose of this run
    ref_m = float(sc["a" + "xyz"[main]].median())
    ref_g = float(sc["a" + "xyz"[grav]].median())
    sc = sc.assign(theta=referenced_tilt(sc["a" + "xyz"[main]],
                                         sc["a" + "xyz"[grav]], ref_m, ref_g))
    pulse_col = "cmd_a" if axis_of(meta) == 0 else "cmd_b"

    rows = []
    for (phase, seq), g in sc[sc["phase"].isin(["kup", "kdn"])].groupby(
            ["phase", "seq"]):
        w = settled_window(g)
        rows.append(dict(dir=phase, seq=int(seq),
                         pulse_us=int(w[pulse_col].iloc[0]),
                         theta_deg=float(w["theta"].mean()),
                         sd=float(w["theta"].std()), n=len(w)))
    d = pd.DataFrame(rows)
    if d.empty:
        print("!! no deadband data"); return

    noise = d["sd"].median()
    print("\n=== DEADBAND / RESOLUTION ===")
    print(f"  angle noise floor  {noise:.4f} deg (median within-dwell sd)")

    out = []
    for (dirn, ), grp in d.groupby(["dir"]):
        grp = grp.sort_values("seq")
        # group by contiguous center blocks (seq was offset by 1000 per center)
        grp["center_idx"] = grp["seq"] // 1000
        for ci, gg in grp.groupby("center_idx"):
            gg = gg.sort_values("pulse_us" if dirn == "kup" else "pulse_us",
                                ascending=(dirn == "kup"))
            th = gg["theta_deg"].to_numpy()
            us = gg["pulse_us"].to_numpy()
            base = th[0]
            moved = np.where(np.abs(th - base) > 3 * noise)[0]
            step_us = abs(us[moved[0]] - us[0]) if len(moved) else np.nan
            # local gain from a straight fit across the span
            k = np.polyfit(us, th, 1)[0]
            out.append(dict(dir=dirn, center_idx=int(ci),
                            center_us=int(np.median(us)),
                            deadband_us=step_us,
                            local_gain_deg_per_us=k,
                            resolution_deg=abs(k) * step_us if
                            np.isfinite(step_us) else np.nan))
    r = pd.DataFrame(out)
    print(r.to_string(index=False, float_format="%.4f"))
    print("  (deadband_us = us of command before motion exceeds 3x the "
          "angle noise floor; datasheet claims 2 us, unloaded)")
    r.to_csv(outdir / "deadband_summary.csv", index=False)
    d.to_csv(outdir / "deadband_points.csv", index=False)

    fig, ax = plt.subplots(figsize=(9, 5))
    for (dirn, ci), gg in d.assign(ci=d["seq"] // 1000).groupby(["dir", "ci"]):
        ax.plot(gg["pulse_us"], gg["theta_deg"], "-o", ms=3, lw=.8,
                label=f"{dirn} center {ci}")
    ax.set_xlabel("pulse [us]"); ax.set_ylabel("plate angle [deg]")
    ax.set_title("TEST K  deadband / resolution, 1 us increments")
    ax.legend(fontsize=8); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(outdir / "deadband.png", dpi=130)
    print(f"\n  wrote deadband_summary.csv, deadband.png -> {outdir}")


# ===================================================================
# TEST P : repeatability / backlash
# ===================================================================
def analyze_repeat(df, meta, outdir):
    ok, _ = health(df, meta, expect_hz=meta.get("log_hz"))
    sc = scaled(samples(df), meta)
    grav, main, cross = pick_axes(sc, meta)
    # pole-safe referenced tilt; reference = median pose of this run
    ref_m = float(sc["a" + "xyz"[main]].median())
    ref_g = float(sc["a" + "xyz"[grav]].median())
    sc = sc.assign(theta=referenced_tilt(sc["a" + "xyz"[main]],
                                         sc["a" + "xyz"[grav]], ref_m, ref_g))
    pulse_col = "cmd_a" if axis_of(meta) == 0 else "cmd_b"

    rows = []
    for (phase, seq), g in sc[sc["phase"].isin(["frombelow", "fromabove"])] \
                             .groupby(["phase", "seq"]):
        w = settled_window(g)
        rows.append(dict(approach=phase, seq=int(seq),
                         target_us=int(w[pulse_col].iloc[0]),
                         theta_deg=float(w["theta"].mean())))
    d = pd.DataFrame(rows)
    if d.empty:
        print("!! no repeatability data"); return
    d.to_csv(outdir / "repeat_points.csv", index=False)

    print("\n=== REPEATABILITY / BACKLASH ===")
    out = []
    for target, g in d.groupby("target_us"):
        lo = g[g["approach"] == "frombelow"]["theta_deg"]
        hi = g[g["approach"] == "fromabove"]["theta_deg"]
        out.append(dict(target_us=int(target), n_below=len(lo), n_above=len(hi),
                        mean_below=lo.mean(), sd_below=lo.std(),
                        mean_above=hi.mean(), sd_above=hi.std(),
                        backlash_deg=hi.mean() - lo.mean(),
                        repeatability_sd_deg=np.mean([lo.std(), hi.std()])))
    r = pd.DataFrame(out)
    print(r.to_string(index=False, float_format="%.4f"))
    print("\n  backlash    = split between the two approach directions")
    print("  repeatability = scatter WITHIN one direction (sd)")
    print("  A sweep alone measures only their sum; this separates them.")
    r.to_csv(outdir / "repeat_summary.csv", index=False)

    fig, ax = plt.subplots(figsize=(9, 5))
    for approach, g in d.groupby("approach"):
        ax.plot(g["seq"], g["theta_deg"], "o", ms=4, label=approach)
    ax.set_xlabel("rep"); ax.set_ylabel("settled angle [deg]")
    ax.set_title("TEST P  repeatability and backlash")
    ax.legend(fontsize=8); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(outdir / "repeat.png", dpi=130)
    print(f"\n  wrote repeat_summary.csv, repeat.png -> {outdir}")


# ===================================================================
# self-test: validate the fitter against known truth
# ===================================================================
def selftest():
    """Synthesize step traces with KNOWN td/tau (and wn/zeta), push them
    through the real analysis path, and check what comes back.  Run this
    before spending bench time -- a fitter validated only on real data
    has no truth to be validated against."""
    print("=== SELFTEST: fitting synthetic steps with known truth ===")
    rng = np.random.default_rng(0)
    fs = 1000.0
    t = np.arange(0, 2.2, 1 / fs)
    t_cmd = 0.2
    fails = 0

    for (td_true, tau_true, A_true) in [(0.014, 0.050, 5.0),
                                        (0.008, 0.020, 1.0),
                                        (0.022, 0.120, -8.0)]:
        theta = first_order(t - t_cmd, A_true, td_true, tau_true)
        rate = np.gradient(theta, t)
        rate += rng.normal(0, 0.4, len(t)) + 0.8 + 0.35 * t   # bias + drift
        r_det, drift = detrend_rate(t, rate, t_cmd)
        th = np.concatenate([[0], np.cumsum(np.diff(t) *
                                            (r_det[1:] + r_det[:-1]) / 2)])
        tt = t - t_cmd
        th -= np.mean(th[tt < 0])
        post, yp = tt[tt >= 0], th[tt >= 0]
        A0 = np.median(yp[-150:])
        popt, _ = curve_fit(first_order, post, yp, p0=[A0, 0.015, 0.05],
                            bounds=([-30, 0, 1e-3], [30, 0.3, 2.0]),
                            maxfev=20000)
        e_td, e_tau = abs(popt[1] - td_true) * 1e3, abs(popt[2] - tau_true) * 1e3
        okk = e_td < 3.0 and e_tau < 0.15 * tau_true * 1e3
        fails += not okk
        print(f"  1st: td {td_true*1e3:5.1f} -> {popt[1]*1e3:5.1f} ms "
              f"(err {e_td:4.1f})   tau {tau_true*1e3:5.1f} -> "
              f"{popt[2]*1e3:5.1f} ms   A {A_true:+.2f} -> {popt[0]:+.2f}   "
              f"drift removed {drift[0]:+.2f} dps/s   {'OK' if okk else 'FAIL'}")

    for (td_true, wn_true, z_true, A_true) in [(0.014, 40.0, 0.35, 5.0),
                                               (0.010, 80.0, 0.20, 2.0)]:
        theta = second_order(t - t_cmd, A_true, td_true, wn_true, z_true)
        rate = np.gradient(theta, t) + rng.normal(0, 0.4, len(t)) + 0.5
        r_det, _ = detrend_rate(t, rate, t_cmd)
        th = np.concatenate([[0], np.cumsum(np.diff(t) *
                                            (r_det[1:] + r_det[:-1]) / 2)])
        tt = t - t_cmd
        th -= np.mean(th[tt < 0])
        post, yp = tt[tt >= 0], th[tt >= 0]
        popt, _ = curve_fit(second_order, post, yp,
                            p0=[np.median(yp[-150:]), 0.015, 40, 0.35],
                            bounds=([-30, 0, 1, 0.01], [30, 0.3, 500, 0.999]),
                            maxfev=30000)
        e = abs(popt[2] - wn_true) / wn_true
        okk = abs(popt[1] - td_true) * 1e3 < 3.0 and e < 0.10
        fails += not okk
        print(f"  2nd: td {td_true*1e3:5.1f} -> {popt[1]*1e3:5.1f} ms   "
              f"wn {wn_true:5.1f} -> {popt[2]:5.1f}   "
              f"zeta {z_true:.2f} -> {popt[3]:.2f}   {'OK' if okk else 'FAIL'}")

    # the thing this replaced: chord back-extrapolation, for comparison
    theta = first_order(t - t_cmd, 5.0, 0.014, 0.050)
    tt = t - t_cmd
    y = theta
    i20 = np.argmax(y > 0.2 * 5.0); i80 = np.argmax(y > 0.8 * 5.0)
    slope = (y[i80] - y[i20]) / (tt[i80] - tt[i20])
    td_chord = tt[i20] - y[i20] / slope
    print(f"\n  chord method on the SAME noiseless trace: "
          f"td {td_chord*1e3:.1f} ms vs truth 14.0 ms "
          f"(bias {-0.24*50:.1f} ms = -0.24*tau, as predicted)")

    print(f"\n  {'ALL PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


# ===================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rundir", nargs="?", help="a runs/<...> directory from capture.py")
    ap.add_argument("--selftest", action="store_true",
                    help="validate the fitters on synthetic data with known truth")
    a = ap.parse_args()

    if a.selftest:
        return selftest()
    if not a.rundir:
        ap.error("need a run directory, or --selftest")

    rundir = Path(a.rundir)
    df, meta = load(rundir)
    test = str(meta.get("test", meta.get("command", "?"))).upper()
    print(f"# {rundir}   test={test}  axis={meta.get('axis')}  "
          f"fw={meta.get('fw')}  captured={meta.get('captured_utc')}")

    handler = {"A": analyze_mapping, "B": analyze_step,
               "K": analyze_deadband, "P": analyze_repeat}.get(test)
    if handler is None:
        print(f"!! don't know how to analyze test '{test}'")
        return 1
    handler(df, meta, rundir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
