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


def drive_signal(cols):
    """
    The Pixhawk-side signal to correlate against thrust.

    Prefers measured battery current (physically closest to thrust); falls back
    to the commanded B channel when the battery stream is missing.
    """
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


def overlap_seconds(a_start, a_end, b_start, b_end):
    """Seconds of wall-clock overlap between two intervals (0 if disjoint)."""
    return max(0.0, min(a_end, b_end) - max(a_start, b_start))
