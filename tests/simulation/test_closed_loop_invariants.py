"""Closed-loop invariants checked against independent references, on the library only."""

import numpy as np

from control_lab.controllers import PID
from control_lab.models import BeltConveyor
from control_lab.simulation import run_closed_loop
from control_lab.simulation.loop import Actuator


def _conveyor_pi(v_ref, **kw):
    plant = BeltConveyor(Ts=5e-3, m_payload=10.0)
    ctl = PID(kp=1.0, ki=4.0, Ts=5e-3, u_min=0.0, u_max=1.0)
    ref = np.full(int(4.0 / 5e-3), v_ref)
    return plant, run_closed_loop(plant, ctl, ref, 5e-3, **kw)


def test_reachable_reference_has_zero_steady_state_error():
    plant, out = _conveyor_pi(0.4)
    y = np.asarray(out["y"])
    assert abs(y[-1] - 0.4) < 1e-3


def test_command_never_leaves_the_actuator_range():
    plant, out = _conveyor_pi(10.0)  # far above the reachable speed
    u = np.asarray(out["u"])
    assert u.min() >= -1e-12 and u.max() <= 1.0 + 1e-12


def test_unreachable_reference_settles_at_the_physical_maximum():
    plant, out = _conveyor_pi(10.0)
    assert abs(np.asarray(out["y"])[-1] - plant.max_speed()) / plant.max_speed() < 0.02


def test_same_seed_gives_identical_noisy_runs():
    _, a = _conveyor_pi(0.4, noise_std=0.003, seed=7)
    _, b = _conveyor_pi(0.4, noise_std=0.003, seed=7)
    _, c = _conveyor_pi(0.4, noise_std=0.003, seed=8)
    assert np.array_equal(a["y"], b["y"]) and not np.array_equal(a["y"], c["y"])


def test_rate_limited_actuator_respects_the_slew_limit():
    plant = BeltConveyor(Ts=5e-3, m_payload=10.0)
    ctl = PID(kp=2.0, ki=6.0, Ts=5e-3, u_min=0.0, u_max=1.0)
    act = Actuator(0.0, 1.0, rate=0.05, u0=0.0)
    ref = np.full(int(2.0 / 5e-3), 0.5)
    out = run_closed_loop(plant, ctl, ref, 5e-3, actuator=act)
    assert np.max(np.abs(np.diff(out["u"]))) <= 0.05 + 1e-9
