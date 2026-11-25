import numpy as np
import pytest

from control_lab import transforms as tf


def test_clarke_roundtrip_balanced():
    th = 0.7
    a, b, c = np.cos(th), np.cos(th - 2 * np.pi / 3), np.cos(th + 2 * np.pi / 3)
    al, be = tf.clarke(a, b, c)
    assert (al, be) == pytest.approx((np.cos(th), np.sin(th)), abs=1e-12)
    out = tf.inv_clarke(al, be)
    assert np.allclose(out, (a, b, c), atol=1e-12)


def test_park_roundtrip():
    d, q = tf.park(*tf.inv_park(0.3, -1.2, 1.1), 1.1)
    assert (d, q) == pytest.approx((0.3, -1.2), abs=1e-12)


def test_park_gives_dc_for_rotating_vector():
    for th in np.linspace(0, 6, 7):
        al, be = np.cos(th), np.sin(th)
        d, q = tf.park(al, be, th)
        assert (d, q) == pytest.approx((1.0, 0.0), abs=1e-12)


def test_wrap_angle_range():
    x = tf.wrap_angle(np.array([-7.0, 0.0, 3.5, 100.0]))
    assert np.all(x >= -np.pi - 1e-12) and np.all(x <= np.pi + 1e-12)


@pytest.mark.parametrize("ang", np.linspace(0, 2 * np.pi, 13))
def test_svpwm_reproduces_reference_inside_linear_range(ang):
    vdc = 48.0
    mag = 0.95 * vdc / np.sqrt(3)  # inside the hexagon incircle
    va, vb = mag * np.cos(ang), mag * np.sin(ang)
    duty, scale = tf.svpwm(va, vb, vdc)
    assert scale == pytest.approx(1.0)
    assert np.all(duty >= -1e-12) and np.all(duty <= 1 + 1e-12)
    ra, rb = tf.duties_to_alpha_beta(duty, vdc)
    assert (ra, rb) == pytest.approx((va, vb), abs=1e-9)


def test_svpwm_limits_beyond_hexagon():
    duty, scale = tf.svpwm(100.0, 0.0, 48.0)
    assert scale < 1.0
    assert np.all(duty >= -1e-12) and np.all(duty <= 1 + 1e-12)
