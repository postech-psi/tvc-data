"""
Deciding when a step has been held long enough.

With the load cell on the Pi, the runner can see thrust while it commands, which
turns dwell from a guess into a measurement. That matters more than it sounds:
the analysis discards the first 50 % of every step as spin-up, so a fixed dwell
spends half the battery on transients, and the configured 2.0 s on the old rig
was in reality running 4.03 s with nobody able to say why.

Three modes:

* **fixed** -- hold for a set time. What the old logger did, and where to start:
  the run's timing does not depend on the sensor, so a sensor fault cannot
  lengthen a step.
* **settle** -- hold until thrust stops moving, then hold a fixed span of settled
  signal. Spends time where the transient actually is rather than uniformly.
* **sem_target** -- hold until the *uncertainty* of the step mean reaches a
  target. This is the useful one: it makes the error bar uniform across the map
  instead of letting it vary with local noise, so points near the noisy end of
  the grid get the extra samples they need and quiet points stop early.

The uncertainty is corrected for autocorrelation using the same estimator the
analysis uses (`tvctools.analyze.integrated_autocorr_time`). Propeller vibration
has lag-1 autocorrelation near 0.5, so a naive sd/sqrt(n) understates the error
by about 2x -- and a dwell controller trusting it would stop every step about
four times too early.
"""

import math

FIXED = "fixed"
SETTLE = "settle"
SEM_TARGET = "sem_target"

#: Fraction of `min_s` discarded before statistics are computed, matching the
#: analysis's `SETTLE_FRAC`. Averaging in the spin-up would bias the mean and
#: inflate the spread, making the SEM target unreachable.
SETTLE_DISCARD_FRAC = 0.5

#: Recompute the SEM at most this often. At 50 Hz an extra 12 samples move it
#: very little, and the run loop has motors to keep alive.
SEM_INTERVAL_S = 0.25

#: Below this many settled samples the statistics are not worth acting on.
MIN_STATS_SAMPLES = 20


def _autocorr_time(values):
    """Integrated autocorrelation time, in samples. Falls back to 1.0 if unavailable."""
    try:
        import numpy as np

        from tvctools.analyze import integrated_autocorr_time

        return float(integrated_autocorr_time(np.asarray(values, dtype=float)))
    except Exception:                        # noqa: BLE001
        return 1.0


def mean_sem(values):
    """
    (mean, sem, n_eff) for a settled run of thrust samples.

    `sem` is corrected for autocorrelation: `n_eff = n / tau`. Returns
    `(mean, None, n)` when there are too few samples to say anything.
    """
    n = len(values)
    if n == 0:
        return None, None, 0
    mean = sum(values) / n
    if n < MIN_STATS_SAMPLES:
        return mean, None, n

    variance = sum((v - mean) ** 2 for v in values) / (n - 1)
    tau = _autocorr_time(values)
    n_eff = max(1.0, n / tau)
    return mean, math.sqrt(variance / n_eff), n_eff


class DwellController:
    """
    Tracks one segment and answers "can we move on yet".

    `observe` is called with every force sample; `done` is called from the run
    loop. Both are cheap -- the expensive statistics are rate-limited.
    """

    def __init__(self, segment, dwell_cfg, t_start):
        self.segment = segment
        self.cfg = dict(dwell_cfg or {})
        self.mode = self.cfg.get("mode", FIXED)
        self.t_start = t_start

        self.min_s = segment.min_dwell_s
        self.max_s = segment.max_dwell_s
        self.planned_s = segment.dwell_s

        # Non-measurement segments (idle, ramp, chirp, tare) always run to their
        # planned length: there is nothing adaptive about them.
        self.adaptive = segment.adaptive and self.mode != FIXED

        self._samples = []                   # (t_mono, thrust) after the discard
        self._window = []                    # (t_mono, thrust) for the settle test
        self._settled_at = None
        self._last_sem_at = None
        self._sem = None
        self._mean = None
        self._n_eff = 0
        self.reason = ""

    @property
    def discard_until(self):
        return self.t_start + self.min_s * SETTLE_DISCARD_FRAC

    def observe(self, t_mono, thrust_n):
        if thrust_n is None:
            return
        if self.adaptive and self.mode == SETTLE:
            self._window.append((t_mono, thrust_n))
            cutoff = t_mono - self.cfg.get("settle_window_s", 0.5)
            while self._window and self._window[0][0] < cutoff:
                self._window.pop(0)
        if t_mono >= self.discard_until:
            self._samples.append((t_mono, thrust_n))

    # --- the decision -------------------------------------------------------
    def done(self, t_mono):
        """Returns (finished, reason)."""
        elapsed = t_mono - self.t_start

        if not self.adaptive:
            if elapsed >= self.planned_s:
                return True, "planned"
            return False, ""

        if elapsed >= self.max_s:
            # The bound that stops a step that never settles -- a fouled stand,
            # a failing sensor, or simply a target that cannot be reached.
            self.reason = "max_dwell"
            return True, "max_dwell"
        if elapsed < self.min_s:
            return False, ""

        if self.mode == SETTLE:
            return self._done_settle(t_mono, elapsed)
        if self.mode == SEM_TARGET:
            return self._done_sem(t_mono)
        return elapsed >= self.planned_s, "planned"

    def _done_settle(self, t_mono, elapsed):
        rate = self._window_rate()
        if rate is None:
            return False, ""
        if abs(rate) <= self.cfg.get("settle_rate_n_per_s", 0.3):
            if self._settled_at is None:
                self._settled_at = t_mono
        else:
            self._settled_at = None          # moved again; start the hold over

        if self._settled_at is None:
            return False, ""
        if t_mono - self._settled_at >= self.cfg.get("hold_after_settle_s", 2.0):
            self.reason = "settled"
            return True, "settled"
        return False, ""

    def _window_rate(self):
        """Least-squares slope of thrust over the settle window, N/s."""
        if len(self._window) < 4:
            return None
        t0 = self._window[0][0]
        xs = [t - t0 for t, _ in self._window]
        ys = [v for _, v in self._window]
        n = len(xs)
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom <= 0:
            return None
        return sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom

    def _done_sem(self, t_mono):
        if (self._last_sem_at is not None
                and t_mono - self._last_sem_at < SEM_INTERVAL_S):
            return False, ""
        self._last_sem_at = t_mono
        self._recompute()

        target = self.cfg.get("target_thrust_sem_n", 0.05)
        if self._sem is not None and self._sem <= target:
            self.reason = "sem_target"
            return True, "sem_target"
        return False, ""

    def _recompute(self):
        self._mean, self._sem, self._n_eff = mean_sem([v for _, v in self._samples])

    # --- reporting ----------------------------------------------------------
    def summary(self):
        """Step statistics for `sequence.csv`. Computed for every mode, as live QC."""
        if self._mean is None or self.mode != SEM_TARGET:
            self._recompute()
        return {
            "thrust_mean_n": self._mean,
            "thrust_sem_n": self._sem,
            "n_settled": len(self._samples),
            "n_eff": self._n_eff,
        }
