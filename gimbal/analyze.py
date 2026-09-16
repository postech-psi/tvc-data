"""Calibration and automatic analysis for the TVC system-ID project."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal
from scipy.optimize import curve_fit

# =============================================================================
# USER ANALYSIS PARAMETERS
# Edit acceptance criteria and analysis windows only in this block.
# run_experiment.py saves a copy of this file in every session directory.
# =============================================================================

# Common data-quality criteria
LATE_SAMPLE_MAX_FRACTION = 0.01
MIN_ALLOWED_LATE_SAMPLES = 1
# Single-event I2C read glitches are physically expected over long captures (a
# multi-minute run reads the IMU hundreds of thousands of times). Allow a tiny
# rate so one stray error does not discard an otherwise complete, CRC-clean run;
# a systematic wiring fault produces orders of magnitude more and still FAILs.
I2C_ERROR_MAX_FRACTION = 0.0001   # 0.01% of samples
MIN_ALLOWED_I2C_ERRORS = 2

# Experiment 1: static-health acceptance
# (Accelerometer calibration is produced separately by calibrate.py, a
# single-pose uniform-scale fit; see calibration.json "note".)
GYRO_NOISE_MAX_DPS = 0.5
# Standard tolerance. calibration.json is a single-pose uniform-scale calibration
# taken at the actual mounting orientation, so corrected |g| = 1.0 at the operating
# point and this strict gate passes honestly (no widening needed). This trims the
# accel scale so the LOCAL gain near the operating orientation is accurate -- which
# is exactly what the mapping/step relative measurements use. See calibration.json
# "note" for the rationale and limits.
HEALTH_GRAVITY_MAG_TOLERANCE_G = 0.02
HEALTH_GRAVITY_MAG_SD_MAX_G = 0.01

# Experiment 2: PWM-to-angle mapping
MAPPING_SETTLED_WINDOW_FRACTION = 0.50
MAPPING_CREEP_MAX_DPS = 0.5
MAPPING_LINEAR_FIT_QUANTILES = (0.20, 0.80)
MAPPING_MIN_TRAVEL_SPAN_DEG = 2.0
# Upper sanity bound: no physical gimbal sweeps this far, so a span above it means
# the tilt series is corrupt (branch-cut wrap, wrong axis, fixture moved) and the
# run must not be reported as PASS.
MAPPING_MAX_TRAVEL_SPAN_DEG = 90.0
MAPPING_MAX_UNSETTLED_FRACTION = 0.10
HYSTERESIS_DEADBAND_TEST_TRIGGER_DEG = 0.5

# Experiment 3: step response metrics and acceptance
RATE_DETREND_TAIL_FRACTION = 0.35
RISE_LOW_FRACTION = 0.10
RISE_HIGH_FRACTION = 0.90
SETTLING_BAND_FRACTION = 0.02
METRIC_TAIL_WINDOW_S = 0.50
FINAL_ANGLE_WINDOW_S = 0.15
SETTLED_SLOPE_WINDOW_S = 0.15
STEP_SETTLED_CREEP_MAX_DPS = 0.5
STEP_MIN_MOVE_DEG = 0.5
STEP_MAX_MODEL_RESIDUAL_PCT = 10.0
STEP_MIN_USEFUL_COUNT = 6
DIRECT_ONSET_SIGMA = 6.0
RING_MIN_SAMPLES = 32
RING_NOISE_MULTIPLIER = 2.0
RING_MIN_HZ = 2.0
RING_MAX_HZ = 200.0
RING_NYQUIST_FRACTION = 0.45
CHIRP_RESIDUAL_TRIGGER_PCT = 5.0
CHIRP_DELAY_GAP_TRIGGER_MS = 5.0
SECOND_ORDER_OVERSHOOT_TRIGGER = 0.05

# Step-model fitting search settings
FIRST_ORDER_INITIAL_DELAY_S = 0.012
FIRST_ORDER_INITIAL_TAU_S = 0.050
MODEL_AMPLITUDE_BOUND_FACTOR = 3.0
MODEL_AMPLITUDE_BOUND_MARGIN_DEG = 1.0
MODEL_DELAY_MAX_S = 0.30
FIRST_ORDER_TAU_MIN_S = 0.001
FIRST_ORDER_TAU_MAX_S = 2.0
FIRST_ORDER_MAX_EVALUATIONS = 20000
SECOND_ORDER_INITIAL_WN_RAD_S = 50.0
SECOND_ORDER_INITIAL_ZETA = 0.4
SECOND_ORDER_WN_MIN_RAD_S = 1.0
SECOND_ORDER_WN_MAX_RAD_S = 500.0
SECOND_ORDER_ZETA_MIN = 0.01
SECOND_ORDER_ZETA_MAX = 0.999
SECOND_ORDER_MAX_EVALUATIONS = 30000
SMOOTH_MAX_WINDOW_SAMPLES = 51
SMOOTH_POLY_ORDER = 3

# Experiment 4: chirp (frequency response) acceptance
CHIRP_WELCH_SEGMENT_S = 2.0
CHIRP_MIN_COHERENCE = 0.8          # band with coherence >= this is trusted
CHIRP_GAIN_REF_MAX_HZ = 2.0        # low-frequency gain reference band
CHIRP_MIN_TRUSTED_SPAN_HZ = 3.0    # need at least this much trusted band to pass

# Experiment 5: deadband / backlash acceptance
DEADBAND_MOTION_THRESHOLD_DEG = 0.10   # angle change counted as real motion
DEADBAND_MAX_ACCEPTABLE_US = 40.0      # informational: flag if wider
# Microseconds are gain-blind: 30 us is harmless at 0.01 deg/us and 1.5 deg of dead
# zone at 0.05 deg/us. What the controller actually feels is the angle, so flag on
# degrees too, matching the mapping-side HYSTERESIS_DEADBAND_TEST_TRIGGER_DEG.
DEADBAND_MAX_ACCEPTABLE_DEG = 0.5

# Experiment 6: joint 2D PWM_A x PWM_B -> angle grid map
GRID_CREEP_MAX_DPS = 0.5
GRID_MIN_CELLS = 16
GRID_MIN_TRAVEL_SPAN_DEG = 2.0
GRID_MAX_TRAVEL_SPAN_DEG = 90.0

# =============================================================================
# INTERNAL CONSTANTS -- normally do not edit below this line.
# =============================================================================

FLAG_I2C = 0x01
FLAG_LATE = 0x02
GIMBAL_NAMES = {0: "outer", 1: "inner", -1: "none"}


def last_value(value):
    return value[-1] if isinstance(value, list) and value else value


def load_run(run_dir: Path | str) -> tuple[pd.DataFrame, dict]:
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
    df = pd.read_csv(run_dir / "raw.csv")
    # Read pre-v2 captures too, but normalize all downstream code to the
    # physical outer/inner naming.
    if "axis" not in df and "motor" in df:
        df = df.rename(columns={"motor": "axis"})
    if "cmd_outer" not in df and "cmd0" in df:
        df = df.rename(columns={"cmd0": "cmd_outer"})
    if "cmd_inner" not in df and "cmd1" in df:
        df = df.rename(columns={"cmd1": "cmd_inner"})
    if "axis" not in meta and "motor" in meta:
        meta["axis"] = meta["motor"]
    axis_value = int(last_value(meta.get("axis", -1)))
    if "gimbal" not in meta:
        meta["gimbal"] = GIMBAL_NAMES.get(axis_value, "none")
    if "gimbal" not in df:
        df["gimbal"] = df["axis"].map(GIMBAL_NAMES).fillna("none")
    numeric = ["t_us", "seq", "axis", "cmd_outer", "cmd_inner",
               "ax", "ay", "az",
               "gx", "gy", "gz", "flags", "packet_seq", "jitter_us"]
    for col in numeric:
        if col in df:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    df["t"] = df["t_us"] * 1e-6
    return df, meta


def sample_rows(df: pd.DataFrame) -> pd.DataFrame:
    return df[df["rec"] == "S"].copy()


def event_rows(df: pd.DataFrame) -> pd.DataFrame:
    return df[df["rec"] == "E"].copy()


def flag_counts(samples: pd.DataFrame) -> dict:
    flags = samples["flags"].fillna(0).astype(int)
    result = {
        "i2c_errors": int(((flags & FLAG_I2C) != 0).sum()),
        "late_samples": int(((flags & FLAG_LATE) != 0).sum()),
        "samples": int(len(samples)),
    }
    if len(samples) > 1:
        periods = np.diff(samples["t_us"].to_numpy(float))
        periods = periods[(periods > 0) & (periods <= 5000)]
        if len(periods):
            result["sample_period_us_median"] = float(np.median(periods))
            result["effective_sample_rate_hz"] = float(1e6 / np.median(periods))
    if "jitter_us" in samples:
        jitter = samples["jitter_us"].dropna().to_numpy(float)
        if len(jitter):
            result["acquisition_jitter_us_p99"] = float(
                np.quantile(np.abs(jitter), 0.99))
    return result


def pass_and_reasons(checks: list[tuple[str, bool]]) -> tuple[bool, list[str]]:
    """checks: list of (failure_description, passed). Returns (all_passed,
    list of failure_description for the checks that did NOT pass), so callers
    always know exactly which gate(s) tripped instead of a blind pass=False."""
    reasons = [label for label, ok in checks if not ok]
    return not reasons, reasons


def save_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                               allow_nan=False), encoding="utf-8")


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return None if not np.isfinite(value) else float(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def load_calibration(path: Path | str) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def corrected_acc(samples: pd.DataFrame, meta: dict, calibration: dict) -> np.ndarray:
    scale = float(last_value(meta["acc_lsb_per_g"]))
    raw = samples[["ax", "ay", "az"]].to_numpy(float) / scale
    matrix = np.asarray(calibration["acc_matrix"], float)
    offset = np.asarray(calibration["acc_offset"], float)
    return raw @ matrix.T + offset


def corrected_gyro(samples: pd.DataFrame, meta: dict, calibration: dict) -> np.ndarray:
    scale = float(last_value(meta["gyro_lsb_per_dps"]))
    raw = samples[["gx", "gy", "gz"]].to_numpy(float) / scale
    rotation = np.asarray(calibration["gyro_rotation"], float)
    return raw @ rotation.T


def analyze_health(run_dir: Path, calibration_path: Path) -> dict:
    df, meta = load_run(run_dir)
    cal = load_calibration(calibration_path)
    s = sample_rows(df)
    acc = corrected_acc(s, meta, cal)
    gyro = corrected_gyro(s, meta, cal)
    counts = flag_counts(s)
    norms = np.linalg.norm(acc, axis=1)
    result = {
        "test": "HEALTH",
        **counts,
        "g_mag_mean": float(norms.mean()),
        "g_mag_sd": float(norms.std()),
        "gyro_bias_dps": gyro.mean(axis=0).tolist(),
        "gyro_noise_dps": gyro.std(axis=0).tolist(),
    }
    i2c_limit = max(MIN_ALLOWED_I2C_ERRORS, I2C_ERROR_MAX_FRACTION * len(s))
    late_limit = max(MIN_ALLOWED_LATE_SAMPLES, LATE_SAMPLE_MAX_FRACTION * len(s))
    result["pass"], result["fail_reasons"] = pass_and_reasons([
        ("calibration.json pass=False", bool(cal.get("pass"))),
        (f"i2c_errors {counts['i2c_errors']} > 허용 {i2c_limit:.1f}",
         counts["i2c_errors"] <= i2c_limit),
        (f"late_samples {counts['late_samples']} > 허용 {late_limit:.1f}",
         counts["late_samples"] <= late_limit),
        (f"g_mag_mean {result['g_mag_mean']:.4f}가 1.0±{HEALTH_GRAVITY_MAG_TOLERANCE_G} 범위 밖",
         abs(result["g_mag_mean"] - 1.0) <= HEALTH_GRAVITY_MAG_TOLERANCE_G),
        (f"g_mag_sd {result['g_mag_sd']:.4f} > 허용 {HEALTH_GRAVITY_MAG_SD_MAX_G}",
         result["g_mag_sd"] <= HEALTH_GRAVITY_MAG_SD_MAX_G),
        (f"gyro_noise_dps max {max(result['gyro_noise_dps']):.3f} >= 허용 {GYRO_NOISE_MAX_DPS}",
         max(result["gyro_noise_dps"]) < GYRO_NOISE_MAX_DPS),
    ])
    save_json(run_dir / "analysis.json", finite_json(result))
    return result


def referenced_tilt(main, gravity, ref_main, ref_gravity):
    # No np.unwrap: gimbal tilt stays well within +/-90 deg so arctan2 never wraps,
    # and skipping unwrap prevents a dropped-sample gap from injecting a spurious
    # 360 deg jump into the rest of the series (which corrupted absolute angle/
    # neutral when a long capture lost a few samples).
    #
    # The difference of two arctan2 results still needs folding into (-180, 180]:
    # when the reference orientation sits near the +/-pi branch cut (gravity
    # negative on the chosen axis, i.e. the fixture mounted inverted), the two
    # terms land on opposite sides of the cut and a ~-7 deg tilt comes out as
    # ~+353 deg. The fold is per-sample and stateless, so unlike np.unwrap it
    # cannot propagate a dropped-sample glitch into the rest of the series.
    angle = np.arctan2(main, gravity)
    delta = np.degrees(angle - np.arctan2(ref_main, ref_gravity))
    return (delta + 180.0) % 360.0 - 180.0


def settled_half(group: pd.DataFrame) -> pd.DataFrame:
    if len(group) < 4:
        return group
    cutoff = group["t"].iloc[0] + MAPPING_SETTLED_WINDOW_FRACTION * (
        group["t"].iloc[-1] - group["t"].iloc[0])
    return group[group["t"] >= cutoff]


def analyze_mapping(run_dir: Path, calibration_path: Path) -> dict:
    df, meta = load_run(run_dir)
    cal = load_calibration(calibration_path)
    s = sample_rows(df).reset_index(drop=True)
    acc = corrected_acc(s, meta, cal)
    for i, name in enumerate("xyz"):
        s["a" + name] = acc[:, i]
    counts = flag_counts(s)
    axis = int(last_value(meta["axis"]))
    gimbal = str(last_value(meta.get("gimbal", GIMBAL_NAMES[axis])))
    pulse_col = "cmd_outer" if axis == 0 else "cmd_inner"
    zero = s[s["phase"] == "zero"]
    sweep = s[s["phase"].isin(["up", "dn", "up2", "dn2"])]
    if zero.empty or sweep.empty:
        raise ValueError("mapping run has no zero/sweep samples")
    gravity = int(np.argmax(np.abs(zero[["ax", "ay", "az"]].mean()).to_numpy()))
    candidates = [i for i in range(3) if i != gravity]
    ref = zero[["ax", "ay", "az"]].mean().to_numpy()

    # Select the main sensing axis by the largest SETTLED tilt span, not by raw accel
    # peak-to-peak over the whole sweep. The fast servo transitions between dwell
    # points induce large transient swings on coupled axes (raw ptp can reach several
    # g), which otherwise fool a peak-to-peak heuristic into picking a transient-
    # dominated axis over the axis that actually carries the steady gimbal tilt.
    def _settled_tilt_span(i: int) -> float:
        s["_sel_theta"] = referenced_tilt(
            acc[:, i], acc[:, gravity], ref[i], ref[gravity])
        means = [s.loc[settled_half(g).index, "_sel_theta"].mean()
                 for _, g in s[s["phase"].isin(["up", "dn", "up2", "dn2"])]
                 .groupby(["phase", "seq"])]
        s.drop(columns="_sel_theta", inplace=True)
        return float(np.nanmax(means) - np.nanmin(means)) if means else 0.0

    main = max(candidates, key=_settled_tilt_span)
    cross = next(i for i in range(3) if i not in {gravity, main})
    s["theta"] = referenced_tilt(acc[:, main], acc[:, gravity], ref[main], ref[gravity])
    s["theta_cross"] = referenced_tilt(acc[:, cross], acc[:, gravity],
                                        ref[cross], ref[gravity])

    rows = []
    phases = ["up", "dn", "up2", "dn2"]
    for (phase, seq), group in s[s["phase"].isin(phases)].groupby(["phase", "seq"]):
        w = settled_half(group)
        t = w["t"].to_numpy()
        theta = w["theta"].to_numpy()
        creep = np.polyfit(t - t[0], theta, 1)[0] if len(w) > 3 else np.nan
        rows.append({
            "phase": phase, "seq": int(seq), "pulse_us": int(w[pulse_col].iloc[0]),
            "theta_deg": float(theta.mean()), "theta_sd_deg": float(theta.std()),
            "cross_deg": float(w["theta_cross"].mean()),
            "creep_dps": float(creep),
            "settled": bool(abs(creep) < MAPPING_CREEP_MAX_DPS),
        })
    points = pd.DataFrame(rows).sort_values(["phase", "pulse_us"])
    lo, hi = points["pulse_us"].quantile(MAPPING_LINEAR_FIT_QUANTILES)
    middle = points[(points["pulse_us"] >= lo) & (points["pulse_us"] <= hi)]
    gain, intercept = np.polyfit(middle["pulse_us"], middle["theta_deg"], 1)
    residual = points["theta_deg"] - (gain * points["pulse_us"] + intercept)
    up = points[points["phase"].isin(["up", "up2"])].groupby("pulse_us")["theta_deg"].mean()
    down = points[points["phase"].isin(["dn", "dn2"])].groupby("pulse_us")["theta_deg"].mean()
    common = up.index.intersection(down.index)
    hysteresis = down[common] - up[common]
    lut = pd.DataFrame({"pulse_us": common,
                        "theta_deg": (up[common] + down[common]) / 2,
                        "theta_up_deg": up[common], "theta_down_deg": down[common]})
    angle_min = float(points["theta_deg"].min())
    angle_max = float(points["theta_deg"].max())
    span = angle_max - angle_min
    result = {
        "test": "A", "axis": axis, "gimbal": gimbal, **counts,
        "gravity_axis": "xyz"[gravity], "main_axis": "xyz"[main],
        "cross_axis": "xyz"[cross], "gain_deg_per_us": float(gain),
        "neutral_us": float(-intercept / gain),
        "angle_min_deg": angle_min, "angle_max_deg": angle_max,
        "max_abs_angle_deg": max(abs(angle_min), abs(angle_max)),
        "travel_span_deg": span,
        "nonlinearity_pct": float(100 * residual.abs().max() / span),
        "hysteresis_mean_deg": float(hysteresis.abs().mean()),
        "hysteresis_max_deg": float(hysteresis.abs().max()),
        "cross_axis_span_deg": float(points["cross_deg"].max() - points["cross_deg"].min()),
        "unsettled_points": int((~points["settled"]).sum()),
        "point_count": len(points),
    }
    i2c_limit = max(MIN_ALLOWED_I2C_ERRORS, I2C_ERROR_MAX_FRACTION * len(s))
    late_limit = max(MIN_ALLOWED_LATE_SAMPLES, LATE_SAMPLE_MAX_FRACTION * len(s))
    unsettled_limit = max(1, MAPPING_MAX_UNSETTLED_FRACTION * len(points))
    result["pass"], result["fail_reasons"] = pass_and_reasons([
        ("calibration.json pass=False", bool(cal.get("pass"))),
        (f"i2c_errors {counts['i2c_errors']} > 허용 {i2c_limit:.1f}",
         counts["i2c_errors"] <= i2c_limit),
        (f"late_samples {counts['late_samples']} > 허용 {late_limit:.1f} "
         f"(전체 {len(s)}개 중 {LATE_SAMPLE_MAX_FRACTION*100:.1f}%, "
         f"jitter_p99={counts.get('acquisition_jitter_us_p99', float('nan')):.0f}us)",
         counts["late_samples"] <= late_limit),
        (f"travel_span_deg {span:.2f}가 허용범위 "
         f"({MAPPING_MIN_TRAVEL_SPAN_DEG}, {MAPPING_MAX_TRAVEL_SPAN_DEG}) 밖",
         MAPPING_MIN_TRAVEL_SPAN_DEG < span < MAPPING_MAX_TRAVEL_SPAN_DEG),
        (f"unsettled_points {result['unsettled_points']} > 허용 {unsettled_limit:.1f} "
         f"(전체 {len(points)}점 중 {MAPPING_MAX_UNSETTLED_FRACTION*100:.0f}%)",
         result["unsettled_points"] <= unsettled_limit),
    ])
    result["recommend_deadband_test"] = (
        result["hysteresis_max_deg"] > HYSTERESIS_DEADBAND_TEST_TRIGGER_DEG)
    points.to_csv(run_dir / "mapping_points.csv", index=False)
    lut.to_csv(run_dir / "lut.csv", index=False)
    save_json(run_dir / "analysis.json", finite_json(result))
    # Plots are produced separately by plot.py mapping.
    return result


def first_order(t, amplitude, delay, tau):
    out = np.zeros_like(t)
    active = t > delay
    out[active] = amplitude * (1.0 - np.exp(-(t[active] - delay) / tau))
    return out


def second_order(t, amplitude, delay, wn, zeta):
    out = np.zeros_like(t)
    active = t > delay
    tt = t[active] - delay
    zeta = np.clip(zeta, 1e-3, 0.999)
    wd = wn * np.sqrt(1.0 - zeta * zeta)
    out[active] = amplitude * (1.0 - np.exp(-zeta * wn * tt) *
        (np.cos(wd * tt) + zeta / np.sqrt(1.0 - zeta * zeta) * np.sin(wd * tt)))
    return out


def detrend_rate(t, rate, command_time):
    tail_start = t[-1] - RATE_DETREND_TAIL_FRACTION * (t[-1] - command_time)
    mask = (t < command_time) | (t >= tail_start)
    fit = np.polyfit(t[mask], rate[mask], 1)
    return rate - np.polyval(fit, t)


def model_bandwidth(kind: str, parameters: dict) -> float:
    if kind == "1st":
        return 1.0 / (2.0 * np.pi * parameters["tau"])
    zeta, wn = parameters["zeta"], parameters["wn"]
    b = 4.0 * zeta * zeta - 2.0
    x = (-b + np.sqrt(b * b + 4.0)) / 2.0
    return float(wn * np.sqrt(x) / (2.0 * np.pi))


def time_metrics(tt, angle, rate, final, pre_noise):
    post = tt >= 0
    tp, yp, rp = tt[post], angle[post], rate[post]
    normalized = yp / final if abs(final) > 1e-9 else np.zeros_like(yp)
    i10 = np.flatnonzero(normalized >= RISE_LOW_FRACTION)
    i90 = np.flatnonzero(normalized >= RISE_HIGH_FRACTION)
    rise = np.nan
    if len(i10) and len(i90) and i90[0] >= i10[0]:
        rise = 1000 * (tp[i90[0]] - tp[i10[0]])
    within = np.abs(yp - final) <= SETTLING_BAND_FRACTION * abs(final)
    stays = np.logical_and.accumulate(within[::-1])[::-1]
    settle_index = np.flatnonzero(stays)
    settle = 1000 * tp[settle_index[0]] if len(settle_index) else np.nan
    tail = tp >= tp[-1] - METRIC_TAIL_WINDOW_S
    tail_rate = rp[tail] - np.mean(rp[tail])
    tail_angle = yp[tail] - np.median(yp[tail])
    rate_rms = float(np.sqrt(np.mean(tail_rate ** 2)))
    frequency = np.nan
    if (len(tail_rate) >= RING_MIN_SAMPLES and
            rate_rms >= RING_NOISE_MULTIPLIER * pre_noise):
        dt = np.median(np.diff(tp[tail]))
        spectrum = np.abs(np.fft.rfft(tail_rate * np.hanning(len(tail_rate))))
        frequencies = np.fft.rfftfreq(len(tail_rate), dt)
        band = ((frequencies >= RING_MIN_HZ) &
                (frequencies <= min(RING_MAX_HZ, RING_NYQUIST_FRACTION / dt)))
        if band.any():
            idx = np.flatnonzero(band)[np.argmax(spectrum[band])]
            frequency = float(frequencies[idx])
    return rise, settle, float(np.sqrt(np.mean(tail_angle ** 2))), rate_rms, frequency


def analyze_step(run_dir: Path, calibration_path: Path) -> dict:
    df, meta = load_run(run_dir)
    cal = load_calibration(calibration_path)
    s = sample_rows(df).reset_index(drop=True)
    gyro = corrected_gyro(s, meta, cal)
    counts = flag_counts(s)
    axis = int(last_value(meta["axis"]))
    gimbal = str(last_value(meta.get("gimbal", GIMBAL_NAMES[axis])))
    pulse_col = "cmd_outer" if axis == 0 else "cmd_inner"
    main = int(np.argmax(np.ptp(gyro, axis=0)))
    events = event_rows(df)
    arm = {int(row.seq): (row.t, int(row[pulse_col]))
           for _, row in events[events["phase"] == "arm"].iterrows()}
    cmd = {int(row.seq): (row.t, int(row[pulse_col]))
           for _, row in events[events["phase"] == "cmd"].iterrows()}
    rows, traces = [], []
    for seq, group in s.groupby("seq"):
        seq = int(seq)
        if seq not in arm or seq not in cmd:
            continue
        idx = group.index.to_numpy()
        t = group["t"].to_numpy()
        command_time = cmd[seq][0]
        rate = detrend_rate(t, gyro[idx, main], command_time)
        angle = np.concatenate([[0.0], np.cumsum(np.diff(t) *
            (rate[1:] + rate[:-1]) / 2.0)])
        tt = t - command_time
        angle -= angle[tt < 0].mean()
        post_t, post_y = tt[tt >= 0], angle[tt >= 0]
        final = float(np.median(post_y[-max(
            10, int(FINAL_ANGLE_WINDOW_S / np.median(np.diff(t)))):]))
        tail = tt >= tt[-1] - SETTLED_SLOPE_WINDOW_S
        settled = (abs(np.polyfit(tt[tail], angle[tail], 1)[0]) <
                   STEP_SETTLED_CREEP_MAX_DPS)
        fit_model = None
        kind = "none"
        parameters = {}
        try:
            p1, _ = curve_fit(first_order, post_t, post_y,
                              p0=[final, FIRST_ORDER_INITIAL_DELAY_S,
                                  FIRST_ORDER_INITIAL_TAU_S],
                              bounds=([
                                  -MODEL_AMPLITUDE_BOUND_FACTOR * abs(final) -
                                  MODEL_AMPLITUDE_BOUND_MARGIN_DEG,
                                  0, FIRST_ORDER_TAU_MIN_S], [
                                  MODEL_AMPLITUDE_BOUND_FACTOR * abs(final) +
                                  MODEL_AMPLITUDE_BOUND_MARGIN_DEG,
                                  MODEL_DELAY_MAX_S, FIRST_ORDER_TAU_MAX_S]),
                              maxfev=FIRST_ORDER_MAX_EVALUATIONS)
            fit_model = first_order(post_t, *p1)
            kind = "1st"
            parameters = {"A": p1[0], "delay": p1[1], "tau": p1[2]}
        except (RuntimeError, ValueError):
            pass
        smoothed = signal.savgol_filter(
            post_y, min(SMOOTH_MAX_WINDOW_SAMPLES, len(post_y) // 2 * 2 - 1),
            SMOOTH_POLY_ORDER)
        overshoot = ((np.max(np.abs(smoothed)) - abs(final)) / abs(final)
                     if abs(final) > 1e-9 else 0.0)
        if overshoot > SECOND_ORDER_OVERSHOOT_TRIGGER:
            try:
                p2, _ = curve_fit(second_order, post_t, post_y,
                                  p0=[final, FIRST_ORDER_INITIAL_DELAY_S,
                                      SECOND_ORDER_INITIAL_WN_RAD_S,
                                      SECOND_ORDER_INITIAL_ZETA],
                                  bounds=([
                                      -MODEL_AMPLITUDE_BOUND_FACTOR * abs(final) -
                                      MODEL_AMPLITUDE_BOUND_MARGIN_DEG,
                                      0, SECOND_ORDER_WN_MIN_RAD_S,
                                      SECOND_ORDER_ZETA_MIN], [
                                      MODEL_AMPLITUDE_BOUND_FACTOR * abs(final) +
                                      MODEL_AMPLITUDE_BOUND_MARGIN_DEG,
                                      MODEL_DELAY_MAX_S,
                                      SECOND_ORDER_WN_MAX_RAD_S,
                                      SECOND_ORDER_ZETA_MAX]),
                                  maxfev=SECOND_ORDER_MAX_EVALUATIONS)
                fit_model = second_order(post_t, *p2)
                kind = "2nd"
                parameters = {"A": p2[0], "delay": p2[1],
                              "wn": p2[2], "zeta": p2[3]}
            except (RuntimeError, ValueError):
                pass
        pre = rate[tt < 0]
        noise = float(pre.std())
        threshold = np.flatnonzero(
            (tt > 0) & (np.abs(rate) > DIRECT_ONSET_SIGMA * noise))
        onset = 1000 * tt[threshold[0]] if len(threshold) else np.nan
        residual = (100 * np.std(post_y - fit_model) / abs(final)
                    if fit_model is not None and abs(final) > 1e-9 else np.nan)
        rise, settle, angle_rms, rate_rms, ring = time_metrics(
            tt, angle, rate, final, noise)
        rows.append({
            "seq": seq, "amp_us": cmd[seq][1] - arm[seq][1],
            "final_deg": final, "model": kind,
            "fitted_delay_ms": 1000 * parameters.get("delay", np.nan),
            "direct_onset_ms": onset,
            "tau_ms": 1000 * parameters.get("tau", np.nan),
            "wn_rad_s": parameters.get("wn", np.nan),
            "zeta": parameters.get("zeta", np.nan),
            "bandwidth_hz": model_bandwidth(kind, parameters) if kind != "none" else np.nan,
            "rise_10_90_ms": rise, "settling_2pct_ms": settle,
            "overshoot_pct": 100 * overshoot,
            "peak_slew_dps": float(np.max(np.abs(rate))),
            "tail_angle_rms_deg": angle_rms, "tail_rate_rms_dps": rate_rms,
            "ring_frequency_hz": ring, "residual_pct": residual,
            "settled": bool(settled), "pre_noise_dps": noise,
        })
        traces.append((seq, tt, angle, rate, post_t, fit_model))
    summary = pd.DataFrame(rows).sort_values("seq")
    if summary.empty:
        raise ValueError("step run has no complete arm/cmd sequences")
    useful = summary[
        (summary["final_deg"].abs() >= STEP_MIN_MOVE_DEG) & summary["settled"] &
        (summary["residual_pct"] <= STEP_MAX_MODEL_RESIDUAL_PCT)]
    result = {
        "test": "B", "axis": axis, "gimbal": gimbal,
        "main_gyro_axis": "xyz"[main], **counts,
        "step_count": len(summary), "useful_step_count": len(useful),
        "direct_onset_ms": float(useful["direct_onset_ms"].median()),
        "fitted_delay_ms": float(useful["fitted_delay_ms"].median()),
        "rise_10_90_ms": float(useful["rise_10_90_ms"].median()),
        "settling_2pct_ms": float(useful["settling_2pct_ms"].median()),
        "bandwidth_hz": float(useful["bandwidth_hz"].median()),
        "peak_slew_dps": float(useful["peak_slew_dps"].max()),
        "tail_rate_rms_dps": float(useful["tail_rate_rms_dps"].median()),
        "median_residual_pct": float(useful["residual_pct"].median()),
        "persistent_ring_detected": bool(useful["ring_frequency_hz"].notna().any()),
    }
    delay_gap = abs(result["fitted_delay_ms"] - result["direct_onset_ms"])
    result["delay_split_warning"] = bool(
        delay_gap > CHIRP_DELAY_GAP_TRIGGER_MS)
    result["recommend_chirp"] = bool(
        result["median_residual_pct"] > CHIRP_RESIDUAL_TRIGGER_PCT or
        result["delay_split_warning"] or
        result["persistent_ring_detected"])
    i2c_limit = max(MIN_ALLOWED_I2C_ERRORS, I2C_ERROR_MAX_FRACTION * len(s))
    late_limit = max(MIN_ALLOWED_LATE_SAMPLES, LATE_SAMPLE_MAX_FRACTION * len(s))
    result["pass"], result["fail_reasons"] = pass_and_reasons([
        ("calibration.json pass=False", bool(cal.get("pass"))),
        (f"i2c_errors {counts['i2c_errors']} > 허용 {i2c_limit:.1f}",
         counts["i2c_errors"] <= i2c_limit),
        (f"late_samples {counts['late_samples']} > 허용 {late_limit:.1f}",
         counts["late_samples"] <= late_limit),
        (f"useful_step_count {len(useful)} < 최소 {STEP_MIN_USEFUL_COUNT} "
         f"(전체 {len(summary)}개 중 min-move/settled/residual 조건을 만족하는 step 부족)",
         len(useful) >= STEP_MIN_USEFUL_COUNT),
    ])
    summary.to_csv(run_dir / "step_summary.csv", index=False)
    save_json(run_dir / "analysis.json", finite_json(result))
    # Plots are produced separately by plot.py step.
    return result


def analyze_chirp(run_dir: Path, calibration_path: Path) -> dict:
    """Frequency response (PWM -> angle) from a logarithmic chirp via Welch CSD."""
    df, meta = load_run(run_dir)
    cal = load_calibration(calibration_path)
    s = sample_rows(df).reset_index(drop=True)
    gyro = corrected_gyro(s, meta, cal)
    counts = flag_counts(s)
    axis = int(last_value(meta["axis"]))
    gimbal = str(last_value(meta.get("gimbal", GIMBAL_NAMES[axis])))
    pulse_col = "cmd_outer" if axis == 0 else "cmd_inner"
    sweep = s[s["phase"] == "chirp"]
    if len(sweep) < 1000:
        raise ValueError("chirp run has too few sweep samples")
    idx = sweep.index.to_numpy()
    t = sweep["t"].to_numpy()
    fs = 1.0 / float(np.median(np.diff(t)))
    u = sweep[pulse_col].to_numpy(float)
    u = u - u.mean()                       # commanded deviation [us]
    main = int(np.argmax(np.ptp(gyro[idx], axis=0)))
    y = gyro[idx, main]
    y = y - y.mean()                       # measured rate [deg/s]

    f0 = float(last_value(meta.get("chirp_f0_hz", 0.5)))
    f1 = float(last_value(meta.get("chirp_f1_hz", 25.0)))
    nperseg = min(int(CHIRP_WELCH_SEGMENT_S * fs), len(u))
    f, Pxx = signal.welch(u, fs=fs, nperseg=nperseg)
    _, Pyy = signal.welch(y, fs=fs, nperseg=nperseg)
    _, Pxy = signal.csd(u, y, fs=fs, nperseg=nperseg)
    with np.errstate(divide="ignore", invalid="ignore"):
        h_rate = Pxy / Pxx                 # (deg/s) per us
        coh = np.abs(Pxy) ** 2 / (Pxx * Pyy)
    keep = (f >= f0) & (f <= f1)
    f, h_rate, coh = f[keep], h_rate[keep], coh[keep]
    h_ang = h_rate / (1j * 2 * np.pi * f)  # PWM -> angle [deg/us]
    gain = np.abs(h_ang)
    phase_deg = np.degrees(np.unwrap(np.angle(h_ang)))

    trusted = coh >= CHIRP_MIN_COHERENCE
    ref = trusted & (f <= CHIRP_GAIN_REF_MAX_HZ)
    gain_ref = float(np.median(gain[ref])) if ref.any() else float("nan")
    gain_db = 20 * np.log10(gain / gain_ref) if np.isfinite(gain_ref) else np.full_like(gain, np.nan)

    bw = float("nan")
    if trusted.any():
        tf, tg = f[trusted], gain_db[trusted]
        below = np.flatnonzero(tg <= -3.0)
        if len(below):
            bw = float(tf[below[0]])
    delay_ms = float("nan")
    if trusted.sum() >= 3:
        slope = np.polyfit(f[trusted], phase_deg[trusted], 1)[0]   # deg/Hz
        delay_ms = -slope / 360.0 * 1000.0
    trusted_span = float(f[trusted].max() - f[trusted].min()) if trusted.any() else 0.0

    pd.DataFrame({"freq_hz": f, "gain_db": gain_db, "gain_deg_per_us": gain,
                  "phase_deg": phase_deg, "coherence": coh}).to_csv(
        run_dir / "chirp_response.csv", index=False)
    result = {
        "test": "C", "axis": axis, "gimbal": gimbal,
        "main_gyro_axis": "xyz"[main], **counts,
        "sample_rate_hz": float(fs),
        "coherent_band_lo_hz": float(f[trusted].min()) if trusted.any() else float("nan"),
        "coherent_band_hi_hz": float(f[trusted].max()) if trusted.any() else float("nan"),
        "trusted_span_hz": trusted_span,
        "gain_ref_deg_per_us": gain_ref,
        "bandwidth_3db_hz": bw,
        "phase_delay_ms": delay_ms,
    }
    i2c_limit = max(MIN_ALLOWED_I2C_ERRORS, I2C_ERROR_MAX_FRACTION * len(s))
    late_limit = max(MIN_ALLOWED_LATE_SAMPLES, LATE_SAMPLE_MAX_FRACTION * len(s))
    result["pass"], result["fail_reasons"] = pass_and_reasons([
        ("calibration.json pass=False", bool(cal.get("pass"))),
        (f"i2c_errors {counts['i2c_errors']} > 허용 {i2c_limit:.1f}",
         counts["i2c_errors"] <= i2c_limit),
        (f"late_samples {counts['late_samples']} > 허용 {late_limit:.1f}",
         counts["late_samples"] <= late_limit),
        (f"trusted_span_hz {trusted_span:.2f} < 최소 {CHIRP_MIN_TRUSTED_SPAN_HZ} "
         f"(coherence >= {CHIRP_MIN_COHERENCE} 대역 부족)",
         trusted_span >= CHIRP_MIN_TRUSTED_SPAN_HZ),
    ])
    save_json(run_dir / "analysis.json", finite_json(result))
    return result


def analyze_deadband(run_dir: Path, calibration_path: Path) -> dict:
    """Backlash / deadband near neutral from a fine ascending/descending staircase."""
    df, meta = load_run(run_dir)
    cal = load_calibration(calibration_path)
    s = sample_rows(df).reset_index(drop=True)
    acc = corrected_acc(s, meta, cal)
    for i, n in enumerate("xyz"):
        s["a" + n] = acc[:, i]
    counts = flag_counts(s)
    axis = int(last_value(meta["axis"]))
    gimbal = str(last_value(meta.get("gimbal", GIMBAL_NAMES[axis])))
    pulse_col = "cmd_outer" if axis == 0 else "cmd_inner"
    zero = s[s["phase"] == "zero"]
    branch = s[s["phase"].isin(["up", "dn"])]
    if zero.empty or branch.empty:
        raise ValueError("deadband run missing zero/branch samples")
    gravity = int(np.argmax(np.abs(zero[["ax", "ay", "az"]].mean()).to_numpy()))
    ref = zero[["ax", "ay", "az"]].mean().to_numpy()
    cands = [i for i in range(3) if i != gravity]

    def settled_span(i: int) -> float:
        s["_th"] = referenced_tilt(acc[:, i], acc[:, gravity], ref[i], ref[gravity])
        means = [s.loc[settled_half(g).index, "_th"].mean()
                 for _, g in branch.groupby(["phase", "seq"])]
        s.drop(columns="_th", inplace=True)
        return float(np.nanmax(means) - np.nanmin(means)) if means else 0.0

    main = max(cands, key=settled_span)
    s["theta"] = referenced_tilt(acc[:, main], acc[:, gravity], ref[main], ref[gravity])
    rows = []
    for (phase, seq), g in branch.groupby(["phase", "seq"]):
        w = settled_half(g)
        rows.append({"phase": phase, "pulse_us": int(g[pulse_col].iloc[0]),
                     "theta_deg": float(s.loc[w.index, "theta"].mean())})
    pts = pd.DataFrame(rows)
    up = pts[pts["phase"] == "up"].groupby("pulse_us")["theta_deg"].mean()
    dn = pts[pts["phase"] == "dn"].groupby("pulse_us")["theta_deg"].mean()
    common = up.index.intersection(dn.index)
    hyst = (dn[common] - up[common])
    gain = float(np.polyfit(up.index.to_numpy(float), up.to_numpy(), 1)[0])
    backlash_deg = float(np.median(np.abs(hyst))) if len(common) else float("nan")
    backlash_us = float(backlash_deg / abs(gain)) if gain else float("nan")
    span_deg = float(up.max() - up.min())

    pd.DataFrame({"pulse_us": common, "theta_up_deg": up[common].to_numpy(),
                  "theta_dn_deg": dn[common].to_numpy()}).to_csv(
        run_dir / "deadband_points.csv", index=False)
    result = {
        "test": "D", "axis": axis, "gimbal": gimbal, "main_axis": "xyz"[main],
        **counts, "local_gain_deg_per_us": gain,
        "backlash_deg": backlash_deg, "backlash_us": backlash_us,
        "travel_span_deg": span_deg,
        "backlash_exceeds_threshold": bool(
            (np.isfinite(backlash_us) and
             backlash_us > DEADBAND_MAX_ACCEPTABLE_US) or
            (np.isfinite(backlash_deg) and
             backlash_deg > DEADBAND_MAX_ACCEPTABLE_DEG)),
    }
    i2c_limit = max(MIN_ALLOWED_I2C_ERRORS, I2C_ERROR_MAX_FRACTION * len(s))
    late_limit = max(MIN_ALLOWED_LATE_SAMPLES, LATE_SAMPLE_MAX_FRACTION * len(s))
    motion_limit = DEADBAND_MOTION_THRESHOLD_DEG * 5
    result["pass"], result["fail_reasons"] = pass_and_reasons([
        ("calibration.json pass=False", bool(cal.get("pass"))),
        (f"i2c_errors {counts['i2c_errors']} > 허용 {i2c_limit:.1f}",
         counts["i2c_errors"] <= i2c_limit),
        (f"late_samples {counts['late_samples']} > 허용 {late_limit:.1f}",
         counts["late_samples"] <= late_limit),
        (f"travel_span_deg {span_deg:.3f} <= 최소 {motion_limit:.2f}",
         span_deg > motion_limit),
    ])
    save_json(run_dir / "analysis.json", finite_json(result))
    return result


def analyze_grid(run_dir: Path, calibration_path: Path) -> dict:
    """Joint PWM_A x PWM_B -> tip-angle surface from a 2D grid sweep.

    Both non-gravity accel axes are kept as two tilt components (pitch, yaw)
    referenced to the neutral 'zero' window. The component that varies with the
    OUTER command is labelled pitch, the other yaw. Writes grid_points.csv
    (cmd_outer, cmd_inner, pitch_deg, yaw_deg) that plot.py grid renders.
    """
    df, meta = load_run(run_dir)
    cal = load_calibration(calibration_path)
    s = sample_rows(df).reset_index(drop=True)
    acc = corrected_acc(s, meta, cal)
    for i, name in enumerate("xyz"):
        s["a" + name] = acc[:, i]
    counts = flag_counts(s)
    zero = s[s["phase"] == "zero"]
    grid = s[s["phase"] == "grid"]
    if zero.empty or grid.empty:
        raise ValueError("grid run has no zero/grid samples")
    gravity = int(np.argmax(np.abs(zero[["ax", "ay", "az"]].mean()).to_numpy()))
    ref = zero[["ax", "ay", "az"]].mean().to_numpy()
    axis_a, axis_b = (i for i in range(3) if i != gravity)
    tilt_a = referenced_tilt(acc[:, axis_a], acc[:, gravity], ref[axis_a], ref[gravity])
    tilt_b = referenced_tilt(acc[:, axis_b], acc[:, gravity], ref[axis_b], ref[gravity])
    s["_tilt_a"] = tilt_a
    s["_tilt_b"] = tilt_b

    rows = []
    for (co, ci), group in grid.groupby(["cmd_outer", "cmd_inner"]):
        w = settled_half(group)
        t = w["t"].to_numpy()
        ta = s.loc[w.index, "_tilt_a"].to_numpy()
        tb = s.loc[w.index, "_tilt_b"].to_numpy()
        creep_a = np.polyfit(t - t[0], ta, 1)[0] if len(w) > 3 else np.nan
        creep_b = np.polyfit(t - t[0], tb, 1)[0] if len(w) > 3 else np.nan
        rows.append({
            "cmd_outer": int(co), "cmd_inner": int(ci),
            "tilt_a_deg": float(ta.mean()), "tilt_b_deg": float(tb.mean()),
            "settled": bool(max(abs(creep_a), abs(creep_b)) < GRID_CREEP_MAX_DPS),
        })
    cells = pd.DataFrame(rows)

    # Label the component driven by the OUTER servo as pitch: pick whichever tilt
    # axis correlates more strongly with cmd_outer.
    corr_a = abs(np.corrcoef(cells["cmd_outer"], cells["tilt_a_deg"])[0, 1])
    corr_b = abs(np.corrcoef(cells["cmd_outer"], cells["tilt_b_deg"])[0, 1])
    if corr_a >= corr_b:
        cells["pitch_deg"], cells["yaw_deg"] = cells["tilt_a_deg"], cells["tilt_b_deg"]
        pitch_axis, yaw_axis = "xyz"[axis_a], "xyz"[axis_b]
    else:
        cells["pitch_deg"], cells["yaw_deg"] = cells["tilt_b_deg"], cells["tilt_a_deg"]
        pitch_axis, yaw_axis = "xyz"[axis_b], "xyz"[axis_a]

    out = cells[["cmd_outer", "cmd_inner", "pitch_deg", "yaw_deg"]].sort_values(
        ["cmd_outer", "cmd_inner"])
    out.to_csv(run_dir / "grid_points.csv", index=False)

    # planar gains and cross-coupling: [pitch;yaw] = C @ [1, dA, dB]
    neutral = float(last_value(meta.get("neutral_us", 1520)))
    design = np.column_stack([np.ones(len(out)),
                              out["cmd_outer"] - neutral,
                              out["cmd_inner"] - neutral])
    cp, *_ = np.linalg.lstsq(design, out["pitch_deg"], rcond=None)
    cy, *_ = np.linalg.lstsq(design, out["yaw_deg"], rcond=None)
    mag = np.hypot(out["pitch_deg"], out["yaw_deg"])
    span = float(max(out["pitch_deg"].max() - out["pitch_deg"].min(),
                     out["yaw_deg"].max() - out["yaw_deg"].min()))
    result = {
        "test": "E", "gimbal": "both", **counts,
        "gravity_axis": "xyz"[gravity],
        "pitch_axis": pitch_axis, "yaw_axis": yaw_axis,
        "cell_count": int(len(out)),
        "unsettled_cells": int((~cells["settled"]).sum()),
        "outer_gain_deg_per_us": float(cp[1]),
        "inner_gain_deg_per_us": float(cy[2]),
        "coupling_outer_to_yaw_pct": float(100 * abs(cy[1]) / (abs(cy[2]) + 1e-9)),
        "coupling_inner_to_pitch_pct": float(100 * abs(cp[2]) / (abs(cp[1]) + 1e-9)),
        "max_tilt_deg": float(mag.max()),
        "travel_span_deg": span,
    }
    i2c_limit = max(MIN_ALLOWED_I2C_ERRORS, I2C_ERROR_MAX_FRACTION * len(s))
    late_limit = max(MIN_ALLOWED_LATE_SAMPLES, LATE_SAMPLE_MAX_FRACTION * len(s))
    unsettled_limit = max(1, 0.10 * len(out))
    result["pass"], result["fail_reasons"] = pass_and_reasons([
        ("calibration.json pass=False", bool(cal.get("pass"))),
        (f"i2c_errors {counts['i2c_errors']} > 허용 {i2c_limit:.1f}",
         counts["i2c_errors"] <= i2c_limit),
        (f"late_samples {counts['late_samples']} > 허용 {late_limit:.1f}",
         counts["late_samples"] <= late_limit),
        (f"cell_count {len(out)} < 최소 {GRID_MIN_CELLS}",
         len(out) >= GRID_MIN_CELLS),
        (f"travel_span_deg {span:.2f}가 허용범위 "
         f"({GRID_MIN_TRAVEL_SPAN_DEG}, {GRID_MAX_TRAVEL_SPAN_DEG}) 밖",
         GRID_MIN_TRAVEL_SPAN_DEG < span < GRID_MAX_TRAVEL_SPAN_DEG),
        (f"unsettled_cells {result['unsettled_cells']} > 허용 {unsettled_limit:.1f}",
         result["unsettled_cells"] <= unsettled_limit),
    ])
    save_json(run_dir / "analysis.json", finite_json(result))
    # Plots are produced separately by plot.py grid.
    return result


def analyze_run(run_dir: Path, calibration_path: Path) -> dict:
    _, meta = load_run(run_dir)
    test = str(last_value(meta.get("test", ""))).upper()
    if test == "HEALTH":
        return analyze_health(run_dir, calibration_path)
    if test == "A":
        return analyze_mapping(run_dir, calibration_path)
    if test == "B":
        return analyze_step(run_dir, calibration_path)
    if test == "C":
        return analyze_chirp(run_dir, calibration_path)
    if test == "D":
        return analyze_deadband(run_dir, calibration_path)
    if test == "E":
        return analyze_grid(run_dir, calibration_path)
    raise ValueError(f"unsupported test type {test}")


def selftest() -> int:
    t = np.arange(0, 2.2, 0.001)
    command = 0.2
    truth = dict(amplitude=5.0, delay=0.014, tau=0.050)
    angle = first_order(t - command, truth["amplitude"], truth["delay"], truth["tau"])
    rng = np.random.default_rng(7)
    rate = np.gradient(angle, t) + 1.2 + 0.2 * t + rng.normal(0, 0.3, len(t))
    rate = detrend_rate(t, rate, command)
    integrated = np.concatenate([[0], np.cumsum(np.diff(t) *
        (rate[1:] + rate[:-1]) / 2)])
    tt = t - command
    integrated -= integrated[tt < 0].mean()
    post = tt >= 0
    fitted, _ = curve_fit(first_order, tt[post], integrated[post],
                          p0=[5, .01, .05], bounds=([-20, 0, .001], [20, .3, 2]))
    ok = abs(fitted[1] - truth["delay"]) < .003 and abs(fitted[2] - truth["tau"]) < .01
    print(f"selftest delay {1000*fitted[1]:.2f} ms, tau {1000*fitted[2]:.2f} ms: "
          f"{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def main() -> int:
    # analysis.json / fail_reasons carry Korean text; the default Windows
    # console codepage (cp1252) can't encode it and would crash the print
    # below even though the JSON file itself was already written fine.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)
    run = sub.add_parser("run")
    run.add_argument("run_dir", type=Path)
    run.add_argument("--calibration", type=Path, required=True)
    sub.add_parser("selftest")
    args = parser.parse_args()
    if args.mode == "selftest":
        return selftest()
    result = analyze_run(args.run_dir, args.calibration)
    print(json.dumps(finite_json(result), ensure_ascii=False, indent=2))
    return 0 if result.get("pass") else 2


if __name__ == "__main__":
    raise SystemExit(main())
