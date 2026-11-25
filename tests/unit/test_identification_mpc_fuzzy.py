import numpy as np
import pytest

from control_lab.controllers import LinearMPC, MamdaniFLC
from control_lab.controllers.fuzzy import triangular
from control_lab.identification import fit_fopdt_least_squares, fit_fopdt_two_point
from control_lab.identification.fopdt import fopdt_step


def test_two_point_fit_recovers_parameters_from_clean_data():
    t = np.linspace(0, 60, 6001)
    y = fopdt_step(t, 2.0, 8.0, 1.5)
    f = fit_fopdt_two_point(t, y, 1.0)
    assert f["K"] == pytest.approx(2.0, rel=0.01)
    assert f["T"] == pytest.approx(8.0, rel=0.03)
    assert f["L"] == pytest.approx(1.5, rel=0.1)


def test_least_squares_fit_recovers_parameters_with_noise():
    rng = np.random.default_rng(1)
    t = np.linspace(0, 60, 1201)
    y = fopdt_step(t, 2.0, 8.0, 1.5) + rng.normal(0, 0.02, t.size)
    f = fit_fopdt_least_squares(t, y, 1.0)
    assert f["K"] == pytest.approx(2.0, rel=0.03)
    assert f["T"] == pytest.approx(8.0, rel=0.08)
    assert f["L"] == pytest.approx(1.5, abs=0.3)


def test_triangular_membership_values():
    assert triangular(0.0, -1, 0, 1) == pytest.approx(1.0)
    assert triangular(0.5, -1, 0, 1) == pytest.approx(0.5)
    assert triangular(2.0, -1, 0, 1) == pytest.approx(0.0)


def test_mamdani_is_odd_symmetric_and_zero_at_origin():
    flc = MamdaniFLC()
    assert flc.infer(0.0, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert flc.infer(0.4, 0.1) == pytest.approx(-flc.infer(-0.4, -0.1), abs=1e-9)
    assert flc.infer(0.8, 0.0) > flc.infer(0.2, 0.0) > 0


def _mpc(constrained=True):
    A = np.array([[0.9]])
    B = np.array([[0.2]])
    C = np.array([[1.0]])
    kw = dict(u_min=-1.0, u_max=1.0, du_max=0.3) if constrained else {}
    return LinearMPC(A, B, C, N=15, Q=np.array([[1.0]]), R=np.array([[0.01]]), **kw)


def test_mpc_matches_unconstrained_when_constraints_inactive():
    m = _mpc(constrained=False)
    x, r, u_prev = np.array([0.0]), 0.2, 0.0
    U = m.solve_unconstrained(x, r, u_prev)
    sol = m.solve(x, r, u_prev)
    U_c = sol[0] if isinstance(sol, tuple) else sol
    assert np.allclose(np.ravel(U)[:1], np.ravel(U_c)[:1], atol=1e-2)


def test_mpc_respects_input_and_rate_limits_and_costs_more():
    m = _mpc()
    x, r, u_prev = np.array([0.0]), 50.0, 0.0
    sol = m.solve(x, r, u_prev)
    U_c = np.ravel(sol[0] if isinstance(sol, tuple) else sol)
    assert np.all(np.abs(U_c) <= 1.0 + 1e-6)
    assert abs(U_c[0] - u_prev) <= 0.3 + 1e-6
    assert np.all(np.abs(np.diff(U_c)) <= 0.3 + 1e-6)
    U_u = np.ravel(m.solve_unconstrained(x, r, u_prev))
    assert m.cost(U_c, x, r, u_prev) >= m.cost(U_u, x, r, u_prev) - 1e-9


def test_projected_gradient_is_zero_at_box_optimum_and_positive_elsewhere():
    from control_lab.controllers.mpc import projected_gradient_norm

    lo, hi = np.array([-1.0, -1.0]), np.array([1.0, 1.0])
    # minimise 0.5 |U - c|^2 with c = (2, 0.3): optimum (1, 0.3), gradient (U - c)
    c = np.array([2.0, 0.3])
    U_opt = np.array([1.0, 0.3])
    assert projected_gradient_norm(U_opt, U_opt - c, lo, hi) == pytest.approx(0.0, abs=1e-15)
    U_bad = np.array([0.0, 0.3])
    assert projected_gradient_norm(U_bad, U_bad - c, lo, hi) == pytest.approx(1.0)
