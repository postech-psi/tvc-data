"""
Clock-fit tests.

The point of `fit_device_clock` is to recover true timing from a transport that
delivers samples in bursts. So the fixtures here reproduce that transport: a
device emitting on a steady, slightly-wrong clock, and a host that sees batches
of samples arrive together, late, with a heavy right tail.
"""

import numpy as np
import pytest

from tvcbench.clock import (
    EpochAnchors,
    fit_device_clock,
    ClockTracker,
)


def synth_stream(n=3000, rate_hz=50.0, ppm=0.0, batch=9, base_latency=0.0015,
                 jitter=0.0004, tail_frac=0.02, tail_s=0.08, quantum=None, seed=1):
    """
    A device stream as the host actually sees it.

    `ppm` skews the device counter against the host clock. `batch` samples share
    one arrival timestamp, which is the USB CDC behaviour that makes raw arrival
    stamps useless. `tail_frac` of batches are stalled by up to `tail_s` -- the
    scheduler hiccups that would drag a least-squares fit off the true line.
    `quantum` rounds the device counter (the STM32 reports integer ms).
    """
    rng = np.random.default_rng(seed)
    dev_true = np.arange(n) / rate_hz              # device's own timeline, seconds
    # The host clock runs at a slightly different rate than the device clock.
    host_true = dev_true * (1.0 + ppm * 1e-6) + 1234.5

    arrival = np.empty(n)
    for start in range(0, n, batch):
        end = min(start + batch, n)
        # The whole batch is handed over when its last sample is ready.
        lat = base_latency + abs(rng.normal(0, jitter))
        if rng.random() < tail_frac:
            lat += rng.random() * tail_s
        arrival[start:end] = host_true[end - 1] + lat

    dev_reported = dev_true.copy()
    if quantum:
        dev_reported = np.round(dev_reported / quantum) * quantum
    return dev_reported, arrival, host_true


def test_recovers_slope_and_offset_through_batching():
    """Batched arrivals must not bias the fit: the lower envelope is still the truth."""
    dev, arrival, host_true = synth_stream(ppm=0.0)
    fit = fit_device_clock(dev, arrival)

    assert fit.ok, fit.warnings
    assert abs(fit.ppm) < 20.0
    # Recovered host times land on the true timeline, not the arrival timeline.
    err = fit.apply(dev) - host_true
    assert np.abs(np.median(err)) < 2e-3
    assert np.percentile(np.abs(err - np.median(err)), 99) < 2e-3


@pytest.mark.parametrize("ppm", [-120.0, -40.0, 0.0, 40.0, 120.0])
def test_recovers_injected_drift(ppm):
    """Crystal error is what the slope is for; it must come back out."""
    dev, arrival, _ = synth_stream(ppm=ppm)
    fit = fit_device_clock(dev, arrival)
    assert fit.ok, fit.warnings
    assert abs(fit.ppm - ppm) < 15.0


def test_beats_least_squares_on_a_heavy_tail():
    """
    The reason for the lower envelope rather than OLS.

    Latency is one-sided, so least squares sits above the true line by roughly
    the mean latency -- and the heavier the stall tail, the further above.
    """
    dev, arrival, host_true = synth_stream(tail_frac=0.15, tail_s=0.25)
    fit = fit_device_clock(dev, arrival)

    ls_slope, ls_intercept = np.polyfit(dev, arrival, 1)
    ls_err = np.median(ls_slope * dev + ls_intercept - host_true)
    env_err = np.median(fit.apply(dev) - host_true)

    assert abs(env_err) < 3e-3
    assert abs(env_err) < abs(ls_err) / 3


def test_survives_millisecond_quantisation():
    """The STM32 reports integer `t_ms`; +-0.5 ms of quantisation must not matter."""
    dev, arrival, host_true = synth_stream(ppm=60.0, quantum=1e-3)
    fit = fit_device_clock(dev, arrival)
    assert fit.ok, fit.warnings
    assert abs(fit.ppm - 60.0) < 20.0
    assert abs(np.median(fit.apply(dev) - host_true)) < 3e-3


def test_floor_width_tracks_batch_depth():
    """
    `floor_width_s` is the QC readout for how much the transport batches.

    A batch handed over together spreads its members over one batch-worth of
    sample periods, so the width should scale with batch depth -- which is what
    makes it a useful "how bad were the raw stamps" number.
    """
    widths = [fit_device_clock(*synth_stream(batch=b, tail_frac=0.0)[:2]).floor_width_s
              for b in (1, 5, 20)]
    assert widths[0] < widths[1] < widths[2]
    # 20 samples at 50 Hz spread over ~0.4 s; the p50-p01 width is about half that.
    assert 0.1 < widths[2] < 0.3


def test_fit_beats_raw_arrival_stamps():
    """The headline claim: fitting recovers timing the transport had destroyed."""
    dev, arrival, host_true = synth_stream()
    fit = fit_device_clock(dev, arrival)

    def spread(e):
        return np.quantile(e, 0.99) - np.quantile(e, 0.01)

    raw_spread = spread(arrival - host_true)
    fit_spread = spread(fit.apply(dev) - host_true)

    assert raw_spread > 0.1              # raw stamps are scattered over >100 ms
    assert fit_spread < 1e-3             # fitted stamps land inside a millisecond
    assert fit_spread < raw_spread / 100


def test_degenerate_inputs_warn_rather_than_raise():
    """A bad fit must never abort a run that is otherwise recording fine."""
    assert not fit_device_clock([], []).ok
    assert not fit_device_clock([1.0], [2.0]).ok

    short = fit_device_clock(np.arange(10) / 50.0, np.arange(10) / 50.0)
    assert not short.ok
    assert any("samples" in w for w in short.warnings)


def test_counter_reset_is_flagged():
    """A device reboot mid-run makes the counter meaningless; say so."""
    dev, arrival, _ = synth_stream(n=600)
    dev = dev.copy()
    dev[300:] -= 6.0            # STM32 rebooted, t_ms restarted
    fit = fit_device_clock(dev, arrival)
    assert any("backwards" in w for w in fit.warnings)


def test_tracker_decimation_matches_full_fit():
    """Thinning the stream must not move the answer -- span carries the slope, not count."""
    dev, arrival, _ = synth_stream(n=6000, ppm=75.0)

    full = ClockTracker("x", dev_scale=1.0, decimate=1)
    thin = ClockTracker("x", dev_scale=1.0, decimate=7)
    for d, a in zip(dev, arrival):
        full.add(d, a)
        thin.add(d, a)

    assert abs(full.fit().ppm - thin.fit().ppm) < 10.0


def test_tracker_scales_raw_counter_units():
    """STM32 hands over integer ms; the tracker converts so slope reads as ppm."""
    dev, arrival, _ = synth_stream(n=2000)
    tr = ClockTracker("loadcell", dev_scale=1e-3)
    for d, a in zip(dev, arrival):
        tr.add(d * 1000.0, a)          # raw counter, in milliseconds
    assert abs(tr.fit().ppm) < 20.0


class TestEpochAnchors:
    def test_single_anchor_is_a_constant_offset(self):
        a = EpochAnchors()
        a.pairs.append((100.0, 1_700_000_000.0))
        assert a.to_epoch(150.0) == pytest.approx(1_700_000_050.0)

    def test_two_anchors_interpolate_and_absorb_slew(self):
        a = EpochAnchors()
        a.pairs.append((100.0, 1_700_000_000.0))
        a.pairs.append((200.0, 1_700_000_100.5))     # wall clock slewed +0.5 s
        assert a.to_epoch(150.0) == pytest.approx(1_700_000_050.25)
        assert a.drift_ppm() == pytest.approx(5000.0)

    def test_capture_brackets_both_clocks(self):
        a = EpochAnchors()
        t_mono, t_epoch = a.capture()
        assert t_mono > 0 and t_epoch > 1_600_000_000
        assert len(a.pairs) == 1

    def test_requires_an_anchor(self):
        with pytest.raises(ValueError):
            EpochAnchors().to_epoch(1.0)
