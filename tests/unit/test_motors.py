import numpy as np
import pytest

from control_lab.controllers.foc import DQCurrentController, current_loop_gains
from control_lab.motors import PMSM, BLDCMotor, DCMotor, DCMotorParams, PMSMParams
from control_lab.motors.bldc import BLDCParams, hall_sector, trapezoid_shape
from control_lab.transforms import clarke, park


def test_dc_motor_steady_state_matches_analytic_speed():
    p = DCMotorParams()
    m = DCMotor(p, Ts=1e-3)
    for _ in range(3000):
        m.step(12.0, 0.0)
    expected = 12.0 * p.Kt / (p.R * p.B + p.Kt * p.Ke)
    assert m.y == pytest.approx(expected, rel=2e-3)
    assert p.dc_gain == pytest.approx(p.Kt / (p.R * p.B + p.Kt * p.Ke), rel=1e-9)


def test_dc_motor_load_torque_reduces_speed():
    a, b = DCMotor(DCMotorParams()), DCMotor(DCMotorParams())
    for _ in range(3000):
        a.step(12.0, 0.0)
        b.step(12.0, 0.02)
    assert b.y < a.y


def test_trapezoid_shape_properties():
    th = np.linspace(0, 2 * np.pi, 3601)
    s = trapezoid_shape(th, 120.0)
    assert s.max() == pytest.approx(1.0, abs=1e-9) and s.min() == pytest.approx(-1.0, abs=1e-9)
    flat = np.mean(np.isclose(s, 1.0, atol=1e-9))
    assert flat == pytest.approx(120.0 / 360.0, abs=0.02)  # one 120 degree flat top per cycle


def test_hall_sector_covers_six_sectors():
    th = np.linspace(0.01, 2 * np.pi - 0.01, 600)
    assert {int(hall_sector(x, 0.0)) for x in th} == {0, 1, 2, 3, 4, 5}


def test_bldc_accelerates_with_positive_duty():
    m = BLDCMotor(BLDCParams())
    for _ in range(20000):
        m.step(0.5, 0.0)
    assert m.y > 50.0  # rad/s, well above standstill


def test_pmsm_current_loop_tracks_iq_reference():
    p = PMSMParams()
    ts = 1e-4
    motor = PMSM(p, Ts=ts)
    ctl = DQCurrentController(p.Ld, p.Lq, p.Rs, p.psi, ts, bandwidth_hz=500.0, v_max=p.v_linear_max)
    from control_lab.transforms import duties_to_alpha_beta, inv_park, svpwm

    iq_ref = 5.0
    iq = 0.0
    tl = p.kt * iq_ref  # load torque balances kt*iq so speed (and back-EMF) stays near zero
    for _ in range(300):
        ia, ib, ic = motor.phase_currents()
        al, be = clarke(ia, ib, ic)
        id_, iq = park(al, be, motor.theta_e)
        vd, vq = ctl.update(0.0, iq_ref, id_, iq, motor.omega_e)
        va, vb = inv_park(vd, vq, motor.theta_e)
        duty, _ = svpwm(va, vb, p.Vdc)
        va_r, vb_r = duties_to_alpha_beta(duty, p.Vdc)
        motor.step(va_r, vb_r, tl)
    assert iq == pytest.approx(iq_ref, rel=0.03)


def test_current_loop_gains_pole_zero_cancellation():
    kp_d, ki_d, kp_q, ki_q = current_loop_gains(1.2e-3, 1.5e-3, 0.35, 500.0)
    wc = 2 * np.pi * 500.0
    assert (kp_d, ki_d) == pytest.approx((1.2e-3 * wc, 0.35 * wc))
    assert (kp_q, ki_q) == pytest.approx((1.5e-3 * wc, 0.35 * wc))


def test_pmsm_voltage_limit_is_reached_at_high_speed_and_integrators_stay_bounded():
    p = PMSMParams()
    ts = 1e-4
    ctl = DQCurrentController(p.Ld, p.Lq, p.Rs, p.psi, ts, 500.0, p.v_linear_max)
    # command far above what 27.7 V allows at high electrical speed
    for _ in range(2000):
        vd, vq = ctl.update(0.0, 20.0, 0.0, 0.0, 3000.0)
    assert ctl.saturated
    assert np.hypot(vd, vq) <= p.v_linear_max + 1e-9
    assert abs(ctl.int_q) < 1e3  # back-calculation keeps the integrator finite


def test_cascade_limits_current_and_reaches_speed():
    import numpy as np

    from control_lab.controllers import PID, CascadeSpeedController
    from control_lab.controllers.foc import speed_loop_gains
    from control_lab.motors import DCMotor, DCMotorParams
    from control_lab.simulation import run_closed_loop

    p = DCMotorParams()
    Ts, ratio, i_max = 2e-4, 5, 3.0
    motor = DCMotor(p, Ts=Ts, substeps=4)
    kp_w, ki_w = speed_loop_gains(p.J, p.Kt, 20.0)
    wc = 2 * np.pi * 150.0
    ctl = CascadeSpeedController(
        motor,
        PID(kp_w, ki_w, Ts=ratio * Ts, u_min=-i_max, u_max=i_max, antiwindup="backcalc"),
        PID(p.L * wc, p.R * wc, Ts=Ts, u_min=-p.v_max, u_max=p.v_max, antiwindup="backcalc"),
        ratio,
    )
    out = run_closed_loop(motor, ctl, np.full(3000, 120.0), Ts, record={"i": lambda m: m.current})
    assert np.max(np.abs(out["i"])) < 1.15 * i_max
    assert abs(out["y"][-1] - 120.0) < 1.0
