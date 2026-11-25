"""Mamdani fuzzy controller and an incremental fuzzy PI built on it.

Design choices (the rule base is a design choice of this repository, not copied from a paper):
  * inputs and output are normalised to [-1, 1] and scaled by gains Ge, Gde, Gu
  * seven triangular sets NB, NM, NS, ZE, PS, PM, PB, uniformly spaced with 50 percent overlap
  * rule (i, j) -> output set clip(i + j), the usual "diagonal" rule base
  * min for AND, max for aggregation, centroid defuzzification on a sampled universe
"""

from __future__ import annotations

import numpy as np

LABELS = ("NB", "NM", "NS", "ZE", "PS", "PM", "PB")


def triangular(x, a: float, b: float, c: float):
    """Triangular membership with feet a, c and peak b."""
    x = np.asarray(x, dtype=float)
    left = np.where(b > a, (x - a) / (b - a), (x >= b).astype(float))
    right = np.where(c > b, (c - x) / (c - b), (x <= b).astype(float))
    return np.clip(np.minimum(left, right), 0.0, 1.0)


class MamdaniFLC:
    """Two-input, one-output Mamdani controller on the normalised universe [-1, 1]."""

    def __init__(self, n_sets: int = 7, universe_points: int = 201):
        if n_sets < 3 or n_sets % 2 == 0:
            raise ValueError("n_sets must be odd and at least 3")
        self.n = n_sets
        self.centers = np.linspace(-1.0, 1.0, n_sets)
        self.width = float(self.centers[1] - self.centers[0])
        self.u_axis = np.linspace(-1.0, 1.0, universe_points)
        self.out_mf = np.array(
            [triangular(self.u_axis, c - self.width, c, c + self.width) for c in self.centers]
        )
        half = n_sets // 2
        idx = np.arange(n_sets) - half
        self.rules = np.clip(idx[:, None] + idx[None, :], -half, half) + half

    def membership(self, x: float) -> np.ndarray:
        x = float(np.clip(x, -1.0, 1.0))
        return np.maximum(0.0, 1.0 - np.abs(x - self.centers) / self.width)

    def infer(self, e_norm: float, de_norm: float) -> float:
        fire = np.minimum.outer(self.membership(e_norm), self.membership(de_norm))
        strength = np.zeros(self.n)
        np.maximum.at(strength, self.rules.ravel(), fire.ravel())
        aggregated = np.max(np.minimum(strength[:, None], self.out_mf), axis=0)
        area = aggregated.sum()
        if area < 1e-12:
            return 0.0
        return float((aggregated * self.u_axis).sum() / area)


class FuzzyPI:
    """Incremental fuzzy PI: u_k = u_{k-1} + Gu * FLC(Ge e_k, Gde (e_k - e_{k-1})).

    Anti-windup: the increment is discarded when it would push a saturated output further out.
    """

    def __init__(self, ge: float, gde: float, gu: float, Ts: float,
                 u_min: float = float("-inf"), u_max: float = float("inf")):
        self.flc = MamdaniFLC()
        self.ge, self.gde, self.gu, self.Ts = float(ge), float(gde), float(gu), float(Ts)
        self.u_min, self.u_max = float(u_min), float(u_max)
        self.reset()

    def reset(self) -> None:
        self.u = 0.0
        self._e_prev = None

    def update(self, r: float, y: float, u_ff: float = 0.0) -> float:
        e = r - y
        de = 0.0 if self._e_prev is None else e - self._e_prev
        self._e_prev = e
        du = self.gu * self.flc.infer(self.ge * e, self.gde * de)
        u_new = self.u + du
        if (u_new > self.u_max and du > 0) or (u_new < self.u_min and du < 0):
            u_new = self.u
        self.u = min(max(u_new, self.u_min), self.u_max)
        return self.u + u_ff
