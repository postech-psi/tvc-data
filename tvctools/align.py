"""
Time-align load-cell force against Pixhawk PWM/current.

The two systems share no clock: the stand logs free-running MCU uptime, the Pi
logs wall-clock epoch. The filename timestamp anchors the load cell to within
about a second; cross-correlating thrust against battery current recovers the
rest.

Thrust is the only usable common observable. In every 2026-07-24 stand log the
`pwm`, `rpm` and `Current_mA` columns are dead (the motor was driven by the
Pixhawk, so the GUI never saw the command), which is why alignment keys off Fz.
"""

import numpy as np

GRID_HZ = 20.0          # match the slower stream; never upsample the Pixhawk data
MAX_LAG_S = 20.0        # stand recording is started by hand a few seconds early
MIN_CORR = 0.25         # below this the peak is noise, not a match
SMOOTH_S = 0.25         # load cell is vibration-heavy; smooth before correlating


def _moving_median(x, width):
    """Odd-width moving median; falls back to the input when too short."""
    width = max(1, int(width) | 1)
    if width <= 1 or x.size < width:
        return x
    pad = width // 2
    padded = np.pad(x, pad, mode="edge")
    windows = np.lib.stride_tricks.sliding_window_view(padded, width)
    return np.median(windows, axis=1)


def _zscore(x):
    sd = x.std()
    return (x - x.mean()) / sd if sd > 1e-12 else x - x.mean()


def thrust_signal(cols):
    """Thrust in newtons (positive) from a load-cell table."""
    return -np.asarray(cols["Fz"], dtype=float)


MIN_CMD_SPAN_US = 150.0   # a command must move at least this much to be a reference


def drive_signal(cols):
    """
    The Pixhawk-side signal to correlate thrust against.

    Commanded PWM first, when it actually moves. A staircase command correlates
    with the thrust staircase better than current does (measured: r = 0.62 vs
    0.54 on the sweeps), because it is the same shape rather than a related one.

    Constant-command runs -- the 60 s holds -- have zero command variance and
    nothing to correlate, so those fall back to measured current, which still
    fluctuates with the motor. Voltage is never used: it is dominated by slow
    battery drift and its lag estimates run to the edge of the search window.

    Note the two references do not agree exactly: command-based lags come out
    ~0.3 s more negative, which is the motor's mechanical response time. Against
    the command, that lag is absorbed into the fit, so the settled thrust lands
    in the right step -- which is what the thrust/torque map needs. Against
    current it is not, giving a truer *clock* offset but slightly worse step
    attribution. Returns the name used so run.json records which applied.
    """
    for key in ("b_cmd_us", "a_cmd_us"):
        cmd = np.asarray(cols.get(key, []), dtype=float)
        if cmd.size and np.isfinite(cmd).any():
            if np.nanmax(cmd) - np.nanmin(cmd) >= MIN_CMD_SPAN_US:
                return np.nan_to_num(cmd), key

    cur = np.asarray(cols.get("current_a", []), dtype=float)
    if cur.size and np.isfinite(cur).any() and np.nanmax(cur) > 0:
        return np.nan_to_num(cur), "current_a"

    return np.nan_to_num(np.asarray(cols["b_cmd_us"], dtype=float)), "b_cmd_us"


def estimate_lag(lc_t, lc_thrust, pm_t, pm_drive,
                 grid_hz=GRID_HZ, max_lag_s=MAX_LAG_S, min_corr=MIN_CORR):
    """
    Estimate the time offset to ADD to the load-cell clock to match the Pixhawk.

    Returns (lag_s, corr, source). corr is None when no peak clears min_corr, in
    which case lag_s is 0.0 and the filename anchor stands on its own -- better
    an honest coarse anchor than a confident wrong one.
    """
    lo = max(lc_t[0], pm_t[0]) - max_lag_s
    hi = min(lc_t[-1], pm_t[-1]) + max_lag_s
    if hi - lo < 5.0:
        return 0.0, None, "no_overlap"

    dt = 1.0 / grid_hz
    grid = np.arange(lo, hi, dt)
    if grid.size < 32:
        return 0.0, None, "too_short"

    a = np.interp(grid, lc_t, lc_thrust)
    b = np.interp(grid, pm_t, pm_drive)
    a = _zscore(_moving_median(a, SMOOTH_S * grid_hz))
    b = _zscore(b)

    max_shift = int(max_lag_s * grid_hz)
    shifts = np.arange(-max_shift, max_shift + 1)
    n = grid.size
    best_corr, best_shift = -np.inf, 0
    for s in shifts:
        # Compare load-cell sample i against Pixhawk sample i+s, i.e. positive s
        # means the load cell leads and its clock must be advanced to match.
        if s >= 0:
            x, y = a[:n - s], b[s:]
        else:
            x, y = a[-s:], b[:n + s]
        if x.size < 32:
            continue
        sx, sy = x.std(), y.std()
        if sx < 1e-12 or sy < 1e-12:
            continue
        c = float(np.dot(x - x.mean(), y - y.mean()) / (x.size * sx * sy))
        if c > best_corr:
            best_corr, best_shift = c, s

    if not np.isfinite(best_corr) or best_corr < min_corr:
        return 0.0, (None if not np.isfinite(best_corr) else round(best_corr, 3)), "weak"
    return round(best_shift / grid_hz, 3), round(best_corr, 3), "xcorr"


EDGE_WINDOW_S = 12.0      # search this far either side of the expected edge
EDGE_SMOOTH_N = 9         # samples in the derivative smoother
EDGE_MIN_JUMP_N = 3.0     # a usable edge must move thrust at least this much


def edge_lag(lc_t, lc_thrust, pm_t, pm_cmd, window_s=EDGE_WINDOW_S):
    """
    Independent lag estimate from the single largest command transition.

    Cross-correlation fits the whole record; this fits one event. It is the
    natural sanity check on that fit, because the biggest transition is by far
    the highest-SNR feature available: on these runs the final ramp-down moves
    thrust ~9 N against 0.7 N of noise (13 sigma), where an ordinary 100 us step
    moves it ~1 N (1.6 sigma) and is not individually locatable.

    Returns (lag_s, jump_N) or (None, None) when no transition is large enough.
    Note this measures command->thrust delay, so it carries the motor's
    mechanical response time exactly as a command-referenced xcorr does.
    """
    cmd = np.asarray(pm_cmd, dtype=float)
    if cmd.size < 4:
        return None, None
    d = np.abs(np.diff(cmd))
    k = int(np.argmax(d))
    if d[k] <= 0:
        return None, None
    t_cmd = float(pm_t[k + 1])
    rising = cmd[k + 1] > cmd[k]

    m = (lc_t > t_cmd - window_s) & (lc_t < t_cmd + window_s)
    if m.sum() < 20:
        return None, None
    tt, yy = lc_t[m], lc_thrust[m]

    # Compare the thrust level well before and well after: a real edge has to
    # move the mean, not just wiggle the derivative inside the noise.
    half = (tt > t_cmd - window_s) & (tt < t_cmd - 1.0)
    other = (tt > t_cmd + 1.0) & (tt < t_cmd + window_s)
    if half.sum() < 5 or other.sum() < 5:
        return None, None
    jump = abs(float(np.mean(yy[other]) - np.mean(yy[half])))
    if jump < EDGE_MIN_JUMP_N:
        return None, None

    smooth = np.convolve(yy, np.ones(EDGE_SMOOTH_N) / EDGE_SMOOTH_N, mode="same")
    slope = np.gradient(smooth, tt)
    i = int(np.argmax(slope)) if rising else int(np.argmin(slope))

    # Sign convention must match estimate_lag, which returns the offset to ADD to
    # the load-cell clock. The edge appears in load-cell time at
    # (t_cmd + tau - lag), so t_edge - t_cmd is -(lag - tau): negate it.
    # Verified against a synthetic record with a known injected shift.
    return round(-(float(tt[i]) - t_cmd), 3), round(jump, 2)


def overlap_seconds(a_start, a_end, b_start, b_end):
    """Seconds of wall-clock overlap between two intervals (0 if disjoint)."""
    return max(0.0, min(a_end, b_end) - max(a_start, b_start))
