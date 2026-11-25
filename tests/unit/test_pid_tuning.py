import numpy as np
import pytest

from control_lab.controllers import PID
from control_lab.controllers import tuning as tn
from control_lab.models import FOPDTPlant
from control_lab.simulation import run_closed_loop


def test_proportional_only_output():
    c = PID(kp=2.0, ki=0.0, kd=0.0, Ts=0.1)
    assert c.update(1.0, 0.25) == pytest.approx(1.5)


def test_integral_accumulates():
    c = PID(kp=0.0, ki=1.0, kd=0.0, Ts=0.1)
    u = [c.update(1.0, 0.0) for _ in range(10)]
    assert u[-1] == pytest.approx(1.0, rel=0.06)


def test_derivative_on_measurement_has_no_setpoint_kick():
    c = PID(kp=0.0, ki=0.0, kd=1.0, Ts=0.01, tf=0.05)
    c.update(0.0, 0.0)
    assert abs(c.update(5.0, 0.0)) < 1e-9  # reference jump, measurement flat


def test_output_saturates_and_antiwindup_recovers_faster():
    def run(mode):
        plant = FOPDTPlant(1.0, 2.0, 0.0, 0.05)
        pid = PID(kp=2.0, ki=2.0, kd=0.0, Ts=0.05, u_min=0.0, u_max=1.0, antiwindup=mode,
                  tt=0.5)
        n = 1200
        ref = np.where(np.arange(n) < 600, 5.0, 0.5)  # unreachable then reachable
        out = run_closed_loop(plant, pid, ref, 0.05)
        assert np.all(out["u"] <= 1.0 + 1e-12)
        return out["y"][600:]

    def first_below(y, level=0.6):
        idx = np.flatnonzero(y < level)
        return int(idx[0]) if idx.size else len(y)  # never recovered -> window length

    y_none, y_bc = run("none"), run("backcalc")
    # With back-calculation the loop leaves saturation sooner after the reference drops.
    assert first_below(y_bc) < first_below(y_none)


def test_zn_open_loop_table():
    r = tn.ziegler_nichols_open(K=2.0, T=10.0, L=1.0, kind="PID")
    assert r["kp"] == pytest.approx(1.2 * 10.0 / (2.0 * 1.0))
    assert r["ti"] == pytest.approx(2.0)
    assert r["td"] == pytest.approx(0.5)


def test_zn_closed_loop_table():
    r = tn.ziegler_nichols_closed(Ku=4.0, Pu=2.0, kind="PID")
    assert (r["kp"], r["ti"], r["td"]) == pytest.approx((2.4, 1.0, 0.25))


def test_tyreus_luyben_table():
    r = tn.tyreus_luyben(Ku=4.0, Pu=2.0, kind="PID")
    assert r["kp"] == pytest.approx(4.0 / 2.2)
    assert r["ti"] == pytest.approx(2.2 * 2.0)
    assert r["td"] == pytest.approx(2.0 / 6.3)


def test_ultimate_point_matches_relay_experiment():
    K, T, L = 1.0, 5.0, 1.0
    Ku, Pu = tn.ultimate_point_fopdt(K, T, L)
    plant = FOPDTPlant(K, T, L, 0.02)
    Ku_r, Pu_r, _ = tn.relay_experiment(plant, d=0.5, Ts=0.02, t_end=120.0)
    # The period is accurate. The gain is a describing-function estimate: for L/T = 0.2 it is
    # about 17-19 % below the exact value, and the gap does not shrink with Ts (checked 20 ms to
    # 1 ms), so it is a property of the method, not of the discretisation.
    assert Pu_r == pytest.approx(Pu, rel=0.03)
    assert Ku_r < Ku
    assert Ku_r == pytest.approx(Ku, rel=0.25)


def test_imc_pid_closed_loop_has_small_overshoot():
    K, T, L = 1.0, 5.0, 1.0
    g = tn.imc_pid(K, T, L, lam=2.0)
    plant = FOPDTPlant(K, T, L, 0.05)
    pid = PID.from_times(g["kp"], g["ti"], g["td"], Ts=0.05, tf=0.05)
    out = run_closed_loop(plant, pid, np.ones(1600), 0.05)
    assert out["y"].max() < 1.25
    assert out["y"][-1] == pytest.approx(1.0, abs=0.02)


def test_cohen_coon_hand_computed():
    # K=1, T=5, L=0.5 -> L/T = 0.1, values computed by hand from the published form
    g = tn.cohen_coon(1.0, 5.0, 0.5)
    assert g["kp"] == pytest.approx(10.0 * (4 / 3 + 0.025))
    assert g["ti"] == pytest.approx(0.5 * (32 + 0.6) / (13 + 0.8))
    assert g["td"] == pytest.approx(2.0 / 11.2)


def test_imc_pid_hand_computed():
    g = tn.imc_pid(1.0, 5.0, 0.5, lam=1.0)
    assert g["kp"] == pytest.approx(5.25 / 1.25)
    assert g["ti"] == pytest.approx(5.25)
    assert g["td"] == pytest.approx(5.0 * 0.5 / 10.5)


def test_simc_and_lambda_hand_computed():
    g = tn.simc_pi(2.0, 10.0, 1.0)  # tau_c = L = 1
    assert g["kp"] == pytest.approx(10.0 / (2.0 * 2.0))
    assert g["ti"] == pytest.approx(min(10.0, 8.0))
    g = tn.lambda_tuning(2.0, 10.0, 1.0, lam=4.0)
    assert g["kp"] == pytest.approx(10.0 / (2.0 * 5.0))
    assert g["ti"] == pytest.approx(10.0)
    assert g["td"] == 0.0


def test_simc_caps_ti_for_large_dead_time_ratio_only_when_needed():
    assert tn.simc_pi(1.0, 5.0, 5.0)["ti"] == pytest.approx(5.0)  # 4*(L+L)=40 > T
    assert tn.simc_pi(1.0, 5.0, 0.5)["ti"] == pytest.approx(4.0)   # 4*(2L)=4 < T
