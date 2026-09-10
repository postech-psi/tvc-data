"""
Tests for the motor delay-vs-lag identifiability study.

Hermetic and synthetic: no bench files needed. The point is to lock in the two
facts the study rests on --

  1. at near-hover SNR the estimator has NO power (a true 44 ms lag is recovered
     as ~0 ms), which is *why* the question is unanswerable from this data;
  2. the estimator is not broken -- at high SNR it recovers the injected tau.

If a future change made (1) start "resolving" tau, that would be a false
positive and this test would catch it.
"""
import numpy as np

import identify_dynamics as ID


def _make_edges(amplitude, noise, n_edges=20, dt=0.02, pre_s=0.4, post_s=0.6):
    """Synthetic near-operating-point step windows with a given SNR."""
    t = np.arange(-pre_s, post_s + dt / 2, dt)
    edges = []
    for _ in range(n_edges):
        edges.append(ID.StepEdge(t=t.copy(), y=np.zeros_like(t),
                                 pre_level=6.0, amplitude=amplitude,
                                 noise=noise, kind="small"))
    return edges


def test_no_power_near_hover_snr():
    """SNR ~ 2.2: a true 44 ms lag must come back as ~0 ms (non-identifiable)."""
    rng = np.random.default_rng(0)
    edges = _make_edges(amplitude=1.5, noise=0.66)      # SNR ~ 2.3
    tau_grid = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10]
    onset_grid = np.arange(0.0, 0.16, 0.01)
    pc = ID.power_check(edges, [0.044], rng, 40, tau_grid, onset_grid)
    # Recovered median collapses to ~0 -- the pure-delay hypothesis "wins" a true lag.
    assert pc[0.044]["median"] <= 0.02


def test_estimator_has_power_at_high_snr():
    """SNR ~ 22: the SAME estimator recovers the injected tau, proving the null
    above is about the data, not a broken fitter."""
    rng = np.random.default_rng(1)
    edges = _make_edges(amplitude=6.0, noise=0.27)      # SNR ~ 22
    tau_grid = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10]
    onset_grid = np.arange(0.0, 0.16, 0.005)
    pc = ID.power_check(edges, [0.06], rng, 40, tau_grid, onset_grid,
                        onset_scatter_s=0.01)
    assert abs(pc[0.06]["median"] - 0.06) <= 0.02


def test_response_is_delay_in_tau_zero_limit():
    t = np.arange(-0.2, 0.6, 0.02)
    g = ID._response(t, onset=0.05, tau=0.0)
    assert np.all(g[t < 0.05] == 0.0)
    assert np.all(g[t >= 0.05] == 1.0)
