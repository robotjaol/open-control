import numpy as np
import pytest

from control_lab.models import FOPDTPlant, TankLevel
from control_lab.models.lti import c2d_zoh


def test_c2d_matches_analytic_first_order():
    a, Ts = -2.0, 0.1
    Ad, Bd = c2d_zoh(np.array([[a]]), np.array([[1.0]]), Ts)
    assert Ad[0, 0] == pytest.approx(np.exp(a * Ts))
    assert Bd[0, 0] == pytest.approx((np.exp(a * Ts) - 1) / a)


def test_fopdt_step_response_is_exact_at_samples():
    K, tau, theta, Ts = 2.0, 1.5, 0.5, 0.05
    p = FOPDTPlant(K, tau, theta, Ts)
    ys = []
    for _ in range(400):
        ys.append(p.step(1.0))
    t = Ts * np.arange(1, 401)
    td = p.theta_effective
    expected = np.where(t > td, K * (1 - np.exp(-(t - td) / tau)), 0.0)
    assert np.allclose(ys, expected, atol=1e-9)


def test_fopdt_delay_is_integer_samples():
    p = FOPDTPlant(1.0, 1.0, 0.3, 0.1)
    assert p.theta_effective == pytest.approx(0.3)


def test_tank_equilibrium_is_stationary_and_linearization():
    tank = TankLevel()
    h = 1.0
    q = tank.equilibrium_inflow(h)
    for _ in range(50):
        tank.step(q)
    assert tank.y == pytest.approx(h, abs=1e-6)
    a, b, Ad, Bd = tank.linearize(h)
    assert a < 0 and b > 0
    assert Ad == pytest.approx(np.exp(a * tank.Ts), rel=1e-6)


def test_conveyor_open_loop_matches_exact_transfer_function():
    from scipy import signal

    from control_lab.models import BeltConveyor, ConveyorParams

    p = ConveyorParams(mu=0.0)
    belt = BeltConveyor(p, Ts=5e-3, m_payload=10.0)
    n = 400
    y = np.array([belt.y] + [belt.step(0.4) for _ in range(n - 1)])
    num, den = belt.linear_tf()
    t = np.arange(n) * 5e-3
    _, ye = signal.step(signal.TransferFunction(np.array(num) * 0.4, den), T=t)
    assert np.max(np.abs(y - ye)) / ye[-1] < 2e-3


def test_conveyor_steady_state_with_friction():
    from control_lab.models import BeltConveyor

    belt = BeltConveyor(m_payload=20.0)
    for _ in range(3000):
        belt.step(0.6)
    assert belt.y == pytest.approx(belt.steady_speed(0.6), rel=2e-3)
    assert belt.command_for_speed(belt.y) == pytest.approx(0.6, rel=2e-3)


def test_conveyor_drop_item_conserves_momentum_and_removal_keeps_speed():
    from control_lab.models import BeltConveyor

    belt = BeltConveyor(m_payload=5.0)
    for _ in range(2000):
        belt.step(0.5)
    v0, m0 = belt.y, belt.mass
    belt.drop_item(10.0)
    assert belt.y == pytest.approx(v0 * m0 / (m0 + 10.0), rel=1e-12)
    v1 = belt.y
    belt.drop_item(-4.0)
    assert belt.y == v1 and belt.m_payload == pytest.approx(11.0)


def test_conveyor_event_fires_once_at_its_time_and_reset_restores_it():
    from control_lab.models import BeltConveyor

    belt = BeltConveyor(Ts=0.01, payload_events=[(0.10, 7.0)])
    for _ in range(9):
        belt.step(0.0)
    assert belt.m_payload == 0.0
    for _ in range(3):
        belt.step(0.0)
    assert belt.m_payload == 7.0
    belt.reset()
    assert belt.m_payload == 0.0 and belt.t == 0.0


def test_conveyor_command_is_clipped_and_speed_never_negative():
    from control_lab.models import BeltConveyor

    belt = BeltConveyor()
    for _ in range(200):
        belt.step(5.0)
    hi = belt.y
    belt2 = BeltConveyor()
    for _ in range(200):
        belt2.step(1.0)
    assert hi == pytest.approx(belt2.y)
    belt3 = BeltConveyor()
    for _ in range(200):
        belt3.step(-3.0)
    assert belt3.y == 0.0


def test_conveyor_v_init_starts_in_steady_state_and_reset_restores_it():
    from control_lab.models import BeltConveyor

    belt = BeltConveyor(m_payload=10.0, v_init=0.5)
    u = belt.command_for_speed(0.5)
    for _ in range(400):
        belt.step(u)
    assert belt.y == pytest.approx(0.5, abs=1e-6)
    belt.reset()
    assert belt.y == 0.5
