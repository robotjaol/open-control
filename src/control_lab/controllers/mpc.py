"""Linear model predictive control with input and input-rate constraints.

Model: x+ = A x + B u, y = C x. Over a horizon of N samples the controller minimises

    sum_{k=1..N} (y_k - r)' Q (y_k - r)  +  sum_{k=0..N-1} (u_k - u_{k-1})' R (u_k - u_{k-1})

subject to u_min <= u_k <= u_max and |u_k - u_{k-1}| <= du_max. The problem is condensed into a
dense QP in the input sequence U and solved with SciPy (L-BFGS-B for pure box constraints,
SLSQP when rate limits are present).

Limits of this implementation: no terminal cost or terminal set, so closed-loop stability is not
guaranteed by construction (see Mayne et al. 2000); control horizon equals prediction horizon;
dense QP solved by a general-purpose solver, not a code-generated embedded solver.
"""

from __future__ import annotations

import time

import numpy as np
from scipy.optimize import minimize

PG_TOL = 1e-6  # box QP accepted as solved when the projected gradient is below this (cost per unit input)


def projected_gradient_norm(U, grad, lo, hi) -> float:
    """Max-norm of the projected gradient of a box-constrained problem; zero at an optimum."""
    return float(np.max(np.abs(np.asarray(U) - np.clip(np.asarray(U) - np.asarray(grad), lo, hi))))


class LinearMPC:
    def __init__(self, A, B, C, N: int, Q, R, u_min=-np.inf, u_max=np.inf, du_max=None):
        self.A = np.atleast_2d(np.asarray(A, dtype=float))
        self.B = np.atleast_2d(np.asarray(B, dtype=float))
        self.C = np.atleast_2d(np.asarray(C, dtype=float))
        self.n, self.m, self.p = self.A.shape[0], self.B.shape[1], self.C.shape[0]
        self.N = int(N)
        if self.N < 1:
            raise ValueError("N must be >= 1")
        self.Q = np.atleast_2d(np.asarray(Q, dtype=float)) * np.eye(self.p)
        self.R = np.atleast_2d(np.asarray(R, dtype=float)) * np.eye(self.m)
        self.u_min = np.broadcast_to(np.asarray(u_min, dtype=float), (self.m,)).copy()
        self.u_max = np.broadcast_to(np.asarray(u_max, dtype=float), (self.m,)).copy()
        self.du_max = None if du_max is None else np.broadcast_to(
            np.asarray(du_max, dtype=float), (self.m,)).copy()
        self._build()
        self._U_prev_solution = None

    def _build(self):
        N, n, m, p = self.N, self.n, self.m, self.p
        Phi = np.zeros((N * p, n))
        Gam = np.zeros((N * p, N * m))
        Apow = np.eye(n)
        powers = [Apow]
        for _ in range(N):
            powers.append(powers[-1] @ self.A)
        for k in range(1, N + 1):
            Phi[(k - 1) * p : k * p] = self.C @ powers[k]
            for j in range(k):
                Gam[(k - 1) * p : k * p, j * m : (j + 1) * m] = self.C @ powers[k - 1 - j] @ self.B
        Qbar = np.kron(np.eye(N), self.Q)
        Rbar = np.kron(np.eye(N), self.R)
        D = np.eye(N * m) - np.kron(np.eye(N, k=-1), np.eye(m))
        S = np.zeros((N * m, m))
        S[:m] = np.eye(m)
        self.Phi, self.Gam, self.Qbar, self.Rbar, self.D, self.S = Phi, Gam, Qbar, Rbar, D, S
        self.H = Gam.T @ Qbar @ Gam + D.T @ Rbar @ D + 1e-10 * np.eye(N * m)
        self.has_rate = self.du_max is not None
        self.has_bounds = bool(np.isfinite(self.u_min).any() or np.isfinite(self.u_max).any())

    def cost(self, U, x, r, u_prev, d=0.0) -> float:
        f = self._linear_term(x, r, u_prev, d)
        return float(0.5 * U @ self.H @ U + f @ U)

    def _linear_term(self, x, r, u_prev, d):
        x = np.asarray(x, dtype=float).reshape(self.n)
        r = np.broadcast_to(np.asarray(r, dtype=float), (self.p,))
        d = np.broadcast_to(np.asarray(d, dtype=float), (self.p,))
        u_prev = np.broadcast_to(np.asarray(u_prev, dtype=float), (self.m,))
        ref = np.tile(r - d, self.N)
        return self.Gam.T @ self.Qbar @ (self.Phi @ x - ref) - self.D.T @ self.Rbar @ self.S @ u_prev

    def solve_unconstrained(self, x, r, u_prev, d=0.0) -> np.ndarray:
        return -np.linalg.solve(self.H, self._linear_term(x, r, u_prev, d))

    def solve(self, x, r, u_prev, d=0.0):
        """Return (U, info). `U` has N*m entries, the first m are applied to the plant."""
        t0 = time.perf_counter()
        f = self._linear_term(x, r, u_prev, d)
        u_prev = np.broadcast_to(np.asarray(u_prev, dtype=float), (self.m,))
        U0 = -np.linalg.solve(self.H, f)
        lo = np.tile(self.u_min, self.N)
        hi = np.tile(self.u_max, self.N)
        if not self.has_bounds and not self.has_rate:
            return U0, {"method": "unconstrained", "success": True, "seconds": time.perf_counter() - t0}

        def fun(U):
            return 0.5 * U @ self.H @ U + f @ U

        def jac(U):
            return self.H @ U + f

        guess = np.clip(U0 if self._U_prev_solution is None else self._U_prev_solution, lo, hi)
        bounds = list(zip(np.where(np.isfinite(lo), lo, None), np.where(np.isfinite(hi), hi, None)))
        bounds = [(None if lb is None else float(lb), None if ub is None else float(ub))
                  for lb, ub in bounds]
        if not self.has_rate:
            res = minimize(fun, guess, jac=jac, bounds=bounds, method="L-BFGS-B",
                           options={"ftol": 1e-14, "gtol": 1e-10, "maxiter": 500})
            method = "L-BFGS-B"
        else:
            rate = np.tile(self.du_max, self.N)
            Su = self.S @ u_prev
            cons = [
                {"type": "ineq", "fun": lambda U: rate - (self.D @ U - Su), "jac": lambda U: -self.D},
                {"type": "ineq", "fun": lambda U: rate + (self.D @ U - Su), "jac": lambda U: self.D},
            ]
            res = minimize(fun, guess, jac=jac, bounds=bounds, constraints=cons, method="SLSQP",
                           options={"ftol": 1e-12, "maxiter": 300})
            method = "SLSQP"
        U = np.asarray(res.x)
        self._U_prev_solution = np.concatenate([U[self.m :], U[-self.m :]])
        ok = bool(res.success)
        if not ok and not self.has_rate:
            # L-BFGS-B can end with an "abnormal" line-search message at a point that is already
            # optimal (for example a warm start). For a convex box QP the first-order condition is
            # the real test: the projected gradient must vanish.
            ok = projected_gradient_norm(U, self.H @ U + f, lo, hi) < PG_TOL
        return U, {"method": method, "success": ok, "iterations": int(res.nit),
                   "seconds": time.perf_counter() - t0}


class MPCController:
    """Offset-free wrapper around LinearMPC using the controller protocol update(r, y).

    The model works in deviation variables around (y_op, u_op). An output disturbance estimate
    d = y - C x_model (the difference between the measurement and the model-only prediction) is
    held constant over the horizon, which removes steady-state offset for constant disturbances
    and for model mismatch of a stable plant.
    """

    def __init__(self, mpc: LinearMPC, y_op: float, u_op: float):
        self.mpc = mpc
        self.y_op, self.u_op = float(y_op), float(u_op)
        self.reset()

    def reset(self):
        self.x_model = np.zeros(self.mpc.n)
        self.u_prev = np.zeros(self.mpc.m)
        self.solve_times: list[float] = []
        self.failures = 0

    def update(self, r: float, y: float, u_ff: float = 0.0) -> float:
        y_dev = y - self.y_op
        d = y_dev - float((self.mpc.C @ self.x_model)[0])
        U, info = self.mpc.solve(self.x_model, r - self.y_op, self.u_prev, d)
        self.solve_times.append(info["seconds"])
        if not info["success"]:
            self.failures += 1
        u0 = U[: self.mpc.m]
        self.x_model = self.mpc.A @ self.x_model + self.mpc.B @ u0
        self.u_prev = u0
        return float(self.u_op + u0[0] + u_ff)
