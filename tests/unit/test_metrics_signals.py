import numpy as np
import pytest

from control_lab import metrics, signals


def test_iae_ise_of_constant_error():
    t = np.linspace(0, 2, 2001)
    e = np.full_like(t, 0.5)
    assert metrics.iae(t, e) == pytest.approx(1.0, rel=1e-9)
    assert metrics.ise(t, e) == pytest.approx(0.5, rel=1e-9)


def test_itae_of_unit_error():
    t = np.linspace(0, 2, 20001)
    assert metrics.itae(t, np.ones_like(t)) == pytest.approx(2.0, rel=1e-6)  # integral t dt = 2


def test_step_metrics_first_order():
    tau = 1.0
    t = np.linspace(0, 10, 10001)
    y = 1 - np.exp(-t / tau)
    m = metrics.step_response_metrics(t, y, 1.0)
    assert m["rise_time_10_90"] == pytest.approx(tau * np.log(9), rel=1e-3)
    assert m["overshoot_percent"] == pytest.approx(0.0, abs=1e-9)
    assert m["settling_time"] == pytest.approx(-tau * np.log(0.02), rel=2e-3)


def test_step_metrics_second_order_overshoot():
    zeta, wn = 0.3, 5.0
    wd = wn * np.sqrt(1 - zeta**2)
    t = np.linspace(0, 6, 60001)
    y = 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta * wn / wd * np.sin(wd * t))
    m = metrics.step_response_metrics(t, y, 1.0)
    expected = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2))
    assert m["overshoot_percent"] == pytest.approx(expected, rel=1e-3)
    assert m["peak_time"] == pytest.approx(np.pi / wd, rel=1e-3)


def test_ripple_and_saturation():
    x = 10 + np.array([-1.0, 1.0, -1.0, 1.0])
    assert metrics.ripple_peak_to_peak_percent(x) == pytest.approx(20.0)
    u = np.array([0.0, 1.0, 1.0, 0.5])
    assert metrics.saturation_fraction(u, 0.0, 1.0) == pytest.approx(0.75)


def test_jsonable_handles_nan_and_numpy():
    out = metrics.jsonable({"a": np.float64(1.5), "b": float("nan"), "c": np.arange(3)})
    assert out == {"a": 1.5, "b": None, "c": [0, 1, 2]}


def test_noise_is_reproducible():
    a = signals.white_noise(100, 0.1, seed=3)
    b = signals.white_noise(100, 0.1, seed=3)
    assert np.array_equal(a, b)
    assert a.std() == pytest.approx(0.1, rel=0.3)


def test_ramp_profile_negative_target():
    t = np.linspace(0, 10, 101)
    v = signals.ramp_speed_profile(t, -5.0, accel=2.0)
    assert v[0] == 0.0 and v[-1] == pytest.approx(-5.0)
    assert np.all(np.diff(v) <= 1e-12)


def test_scurve_reaches_target_without_overshoot_and_limits_accel():
    t = np.linspace(0, 10, 10001)
    v = signals.scurve_speed_profile(t, 5.0, a_max=2.0, j_max=8.0)
    assert v[-1] == pytest.approx(5.0, abs=1e-2)
    assert v.max() <= 5.0 + 1e-2
    a = np.diff(v) / np.diff(t)
    assert np.abs(a).max() <= 2.0 * 1.02
