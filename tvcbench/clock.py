"""
One clock for the whole bench.

Every sample this package records carries `t_mono`, taken from
`time.monotonic()` at the moment the host saw it. Wall-clock epoch is *derived*
from anchor pairs captured at run start and end, never read per-sample: an NTP
step in the middle of a run would otherwise shift part of the record relative to
the rest, and the old logger stamped rows with `time.time()` directly.

Arrival timestamps alone are not good enough for the load cell. USB CDC hands
over ~8-10 samples in a single read, so a burst of samples all get nearly the
same `t_mono` even though the STM32 produced them 20 ms apart. The fix is to use
each device's own counter (`t_ms` on the STM32, `time_usec` on the Pixhawk) as
the precise relative clock and fit it to the host clock:

    t_host = slope * t_dev + offset

The fit exploits the one thing known for certain about transport latency: it is
**one-sided**. A sample can arrive late, never early. So the true line lies along
the *lower* edge of the point cloud, and fitting that edge -- rather than running
least squares through the middle of it -- removes the batching jitter entirely.
This is what buys sub-millisecond relative timing with no sync wiring.
"""

import time

import numpy as np

# A pair separated by only a few samples carries almost no slope information:
# the latency noise is the same size as the real time difference. Slope precision
# scales as (latency noise / dx), so only well-separated pairs are worth using.
MIN_PAIR_SPAN_FRAC = 0.5      # pairs must straddle at least half the record
SLOPE_PAIR_SAMPLES = 20000    # sampled Theil-Sen; exact O(n^2) is pointless here

# The strict minimum residual would hand the entire fit to a single point, and a
# device clock glitch or a t_ms quantisation edge could make that point bogus.
# A low quantile is the same estimator with a fuse on it.
FLOOR_QUANTILE = 0.01

# Points within this much of the floor are the "fast" arrivals -- the ones that
# actually saw minimum latency. Polishing the slope on only these sharpens the
# fit, because the slow tail carries no timing information, only noise.
POLISH_WINDOW_S = 0.004
POLISH_ITERS = 2

# Quality gates. These are reported, not enforced -- a caller decides what to do.
MIN_FIT_SAMPLES = 50
MIN_FIT_SPAN_S = 5.0
# Consumer crystals are specified around +-50 ppm and two of them are involved
# (device and host), so beyond this something is wrong with the fit, not the part.
MAX_PLAUSIBLE_PPM = 500.0


def now():
    """Master clock reading, in seconds. Monotonic: never steps, never goes back."""
    return time.monotonic()


def now_epoch():
    """Wall-clock epoch, in seconds. Only for anchors and file naming."""
    return time.time()


class EpochAnchors:
    """
    Maps the monotonic master clock to wall-clock epoch.

    Anchors are captured at run start and run end. With two of them the mapping
    is a straight line, which also absorbs any slow NTP slew across the run; with
    one it is a constant offset. Either way, a mid-run clock *step* moves the
    anchors, not the data.
    """

    def __init__(self):
        self.pairs = []   # [(t_mono, t_epoch), ...] in capture order

    def capture(self):
        """Take an anchor now. Reads both clocks back-to-back to keep the pair tight."""
        t_mono = now()
        t_epoch = now_epoch()
        # Second monotonic read brackets the epoch call; the midpoint is the best
        # estimate of when t_epoch was actually sampled.
        t_mono = 0.5 * (t_mono + now())
        self.pairs.append((t_mono, t_epoch))
        return t_mono, t_epoch

    def to_epoch(self, t_mono):
        """Convert monotonic seconds to epoch seconds. Accepts scalars or arrays."""
        if not self.pairs:
            raise ValueError("no anchors captured")
        t_mono = np.asarray(t_mono, dtype=float)
        if len(self.pairs) == 1:
            m0, e0 = self.pairs[0]
            return t_mono + (e0 - m0)
        (m0, e0), (m1, e1) = self.pairs[0], self.pairs[-1]
        if m1 - m0 < 1e-9:
            return t_mono + (e0 - m0)
        slope = (e1 - e0) / (m1 - m0)
        return e0 + slope * (t_mono - m0)

    def drift_ppm(self):
        """Observed monotonic-vs-realtime drift over the run, in ppm.

        Large values mean the wall clock was slewed or stepped during the run --
        which is exactly the situation this class exists to survive.
        """
        if len(self.pairs) < 2:
            return None
        (m0, e0), (m1, e1) = self.pairs[0], self.pairs[-1]
        if m1 - m0 < 1e-9:
            return None
        return ((e1 - e0) / (m1 - m0) - 1.0) * 1e6

    def as_dict(self):
        return {
            "pairs": [{"t_mono": m, "t_epoch": e} for m, e in self.pairs],
            "drift_ppm": self.drift_ppm(),
        }


class ClockFit:
    """Result of fitting a device counter to the host monotonic clock."""

    def __init__(self, slope, offset, n, span_s, residuals, warnings):
        self.slope = float(slope)
        self.offset = float(offset)
        self.n = int(n)
        self.span_s = float(span_s)
        self.warnings = list(warnings)

        r = np.asarray(residuals, dtype=float)
        self.residual_p01 = float(np.quantile(r, 0.01)) if r.size else float("nan")
        self.residual_p50 = float(np.quantile(r, 0.50)) if r.size else float("nan")
        self.residual_p99 = float(np.quantile(r, 0.99)) if r.size else float("nan")
        # Spread of the arrival cloud above its floor. Under USB CDC this is
        # dominated by *batch depth*: a batch handed over together spreads its
        # members across one batch-worth of sample periods, so this reads out
        # roughly half the batch span in seconds. That makes it the single most
        # useful link-quality number -- it says how much timing information the
        # raw arrival stamps had lost before the fit put it back.
        self.floor_width_s = self.residual_p50 - self.residual_p01

    @property
    def ppm(self):
        """Device-vs-host clock rate error, in parts per million."""
        return (self.slope - 1.0) * 1e6

    @property
    def ok(self):
        return not self.warnings

    def apply(self, dev_t):
        """Device counter (seconds) -> host monotonic seconds."""
        return self.slope * np.asarray(dev_t, dtype=float) + self.offset

    def as_dict(self):
        return {
            "slope": self.slope,
            "offset": self.offset,
            "ppm": self.ppm,
            "n": self.n,
            "span_s": self.span_s,
            "residual_p01_s": self.residual_p01,
            "residual_p50_s": self.residual_p50,
            "residual_p99_s": self.residual_p99,
            "floor_width_s": self.floor_width_s,
            "ok": self.ok,
            "warnings": self.warnings,
        }

    def __repr__(self):
        return (f"ClockFit(ppm={self.ppm:+.1f}, n={self.n}, span={self.span_s:.1f}s, "
                f"floor_width={self.floor_width_s * 1e3:.2f}ms, ok={self.ok})")


def _theil_sen_slope(x, y, rng):
    """
    Robust slope from randomly sampled, well-separated point pairs.

    Only pairs straddling at least `MIN_PAIR_SPAN_FRAC` of the record are used.
    Near pairs are dominated by latency noise and would only add variance, and
    the median over pairs shrugs off the heavy right tail of USB latency that
    would drag a least-squares slope around.
    """
    n = x.size
    order = np.argsort(x)
    xs, ys = x[order], y[order]

    gap = max(1, int(n * MIN_PAIR_SPAN_FRAC))
    if gap >= n:
        return None
    i = rng.integers(0, n - gap, size=min(SLOPE_PAIR_SAMPLES, (n - gap) * 4))
    j = i + gap + rng.integers(0, n - gap - i)

    dx = xs[j] - xs[i]
    good = dx > 0
    if not np.any(good):
        return None
    return float(np.median((ys[j] - ys[i])[good] / dx[good]))


def fit_device_clock(dev_t, host_t, label=""):
    """
    Fit `host_t = slope * dev_t + offset` along the lower envelope of the cloud.

    `dev_t` and `host_t` are both in seconds -- convert the raw counter first
    (STM32 `t_ms` / 1e3, Pixhawk `time_usec` / 1e6) so the slope comes out near
    1.0 and reads directly as ppm.

    Three passes:
      1. Robust slope from well-separated pairs (immune to latency outliers).
      2. Offset at the `FLOOR_QUANTILE` of the residuals -- the arrival floor,
         not the middle of the cloud.
      3. Re-fit least squares using only the fast arrivals near that floor, which
         is where the timing information actually lives.

    What this does and does not remove: on a simulated USB CDC link (50 Hz,
    9-sample batches, 2 % stalls) raw arrival stamps are late by a median 82 ms
    with a 161 ms spread; after fitting, the spread is 0.03 ms. The *constant*
    part of the transport delay survives as a fixed offset -- roughly 1.5 ms in
    that simulation -- because no amount of fitting can distinguish a steady
    delay from a clock offset. That is harmless here: it shifts every force
    sample by the same amount, well under one sample period.

    Returns a `ClockFit`. Degenerate inputs return a fit flagged in `warnings`
    rather than raising: a bad clock fit must not abort a run that is otherwise
    recording fine, it must be visible in the manifest afterwards.
    """
    x = np.asarray(dev_t, dtype=float)
    y = np.asarray(host_t, dtype=float)
    warnings = []
    tag = f"{label}: " if label else ""

    good = np.isfinite(x) & np.isfinite(y)
    x, y = x[good], y[good]
    n = x.size

    if n < 2:
        return ClockFit(1.0, 0.0, n, 0.0, [], [f"{tag}too few samples to fit ({n})"])

    # A device counter that goes backwards means a reset or a wrap. The fit would
    # be meaningless, and silently averaging across the discontinuity is worse
    # than saying so.
    if np.any(np.diff(x) < 0):
        warnings.append(f"{tag}device counter went backwards (reset or wrap)")

    span = float(x[-1] - x[0]) if x[-1] > x[0] else float(x.max() - x.min())
    if n < MIN_FIT_SAMPLES:
        warnings.append(f"{tag}only {n} samples (want >= {MIN_FIT_SAMPLES})")
    if span < MIN_FIT_SPAN_S:
        warnings.append(f"{tag}only {span:.1f}s of data (want >= {MIN_FIT_SPAN_S}s)")

    rng = np.random.default_rng(0)   # fixed seed: the fit must be reproducible
    slope = _theil_sen_slope(x, y, rng)
    if slope is None or not np.isfinite(slope) or slope <= 0:
        # Fall back to the endpoint slope; with span == 0 fall back to 1:1.
        slope = (y[-1] - y[0]) / span if span > 0 else 1.0
        warnings.append(f"{tag}robust slope unavailable, used endpoint slope")

    offset = float(np.quantile(y - slope * x, FLOOR_QUANTILE))

    # Polish on the fast arrivals only. Each pass re-selects the window, so a
    # slightly wrong initial slope does not lock in a slanted selection.
    for _ in range(POLISH_ITERS):
        resid = y - (slope * x + offset)
        near = np.abs(resid) <= POLISH_WINDOW_S
        if np.count_nonzero(near) < max(10, MIN_FIT_SAMPLES // 5):
            break
        xf, yf = x[near], y[near]
        if xf.max() - xf.min() < 1e-9:
            break
        new_slope, new_intercept = np.polyfit(xf, yf, 1)
        if not np.isfinite(new_slope) or new_slope <= 0:
            break
        slope = float(new_slope)
        offset = float(np.quantile(y - slope * x, FLOOR_QUANTILE))

    residuals = y - (slope * x + offset)
    ppm = (slope - 1.0) * 1e6
    if abs(ppm) > MAX_PLAUSIBLE_PPM:
        warnings.append(f"{tag}implausible clock rate error {ppm:+.0f} ppm")

    return ClockFit(slope, offset, n, span, residuals, warnings)


class ClockTracker:
    """
    Accumulates (device counter, host arrival) pairs during a run and fits at the end.

    Kept deliberately cheap in the hot path -- two list appends per sample -- so
    it can sit inside the acquisition loop. `decimate` bounds memory on long
    runs: the fit is over-determined by orders of magnitude, and thinning the
    stream costs nothing because the slope comes from the record's *span*, not
    its sample count.
    """

    def __init__(self, label, dev_scale, decimate=1):
        self.label = label
        self.dev_scale = float(dev_scale)   # counter units -> seconds (1e-3, 1e-6)
        self.decimate = max(1, int(decimate))
        self._dev = []
        self._host = []
        self._seen = 0

    def add(self, dev_raw, t_mono):
        self._seen += 1
        if self._seen % self.decimate:
            return
        self._dev.append(dev_raw * self.dev_scale)
        self._host.append(t_mono)

    def fit(self):
        return fit_device_clock(self._dev, self._host, label=self.label)
