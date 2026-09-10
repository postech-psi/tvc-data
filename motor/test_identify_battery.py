"""
Tests for battery-sag identification. Fit math is tested hermetically on
synthetic data with a known slope; coulomb counting is checked for monotonicity.
"""
import numpy as np

import identify_battery as B


def test_coulomb_count_monotonic_and_scaled():
    t = np.linspace(0, 10, 101)          # 10 s
    i = np.full_like(t, 3.6)             # 3.6 A -> exactly 1 mAh per second
    mah = B.coulomb_count(t, i)
    assert np.all(np.diff(mah) >= 0)     # monotonic
    assert abs(mah[-1] - 10.0) < 1e-6    # 3.6 A for 10 s = 10 mAh


def test_fit_sag_recovers_known_slope():
    rng = np.random.default_rng(0)
    voltage = np.linspace(11.5, 9.0, 400)
    k_true, b_true = 1.4, -3.0
    thrust = k_true * voltage + b_true + rng.normal(0, 0.02, voltage.size)
    mah = (11.9 - voltage) / 0.001       # a linear discharge curve, 0.001 V/mAh
    fit = B.fit_sag(thrust, voltage, mah)
    assert abs(fit.thrust_sensitivity_n_per_v - k_true) < 0.05
    assert fit.r > 0.99
    assert fit.v_full > fit.voltage_end_v         # full-pack intercept above the end
    assert fit.v_per_mah < 0                       # voltage falls with charge drawn
