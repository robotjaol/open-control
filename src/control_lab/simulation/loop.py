"""Generic sampled closed loop: plant protocol + controller protocol.

Controller protocol:  u = controller.update(r, y_measured)
Plant protocol:       y_next = plant.step(u, d), attribute plant.y

Order inside one sample k: measure y_k (with optional noise and quantisation), compute u_k,
advance the plant to k+1 under u_k and disturbance d_k. The recorded `y` is the true output at
the start of sample k, so y[0] is the initial condition and metrics see the same grid as `t`.
"""

from __future__ import annotations

import numpy as np

from control_lab.signals import quantize, white_noise


def run_closed_loop(plant, controller, ref, Ts: float, disturbance=None, noise_std: float = 0.0,
                    quant_step: float = 0.0, seed: int = 0, ff=None, actuator=None,
                    record=None) -> dict:
    """Run the loop for len(ref) samples.

    ref, disturbance, ff: arrays (disturbance and ff optional, same length as ref).
    actuator: optional callable u_cmd -> u_applied (limits, rate limits, dead zone).
    record: optional dict name -> callable(plant) returning an extra signal sampled each step.
    """
    ref = np.asarray(ref, dtype=float)
    n = len(ref)
    d = np.zeros(n) if disturbance is None else np.asarray(disturbance, dtype=float)
    ff = np.zeros(n) if ff is None else np.asarray(ff, dtype=float)
    noise = white_noise(n, noise_std, seed) if noise_std > 0 else np.zeros(n)
    plant.reset()
    if hasattr(controller, "reset"):
        controller.reset()
    if actuator is not None and hasattr(actuator, "reset"):
        actuator.reset()
    y = np.empty(n)
    y_meas = np.empty(n)
    u = np.empty(n)
    extra = {k: np.empty(n) for k in (record or {})}
    for k in range(n):
        y[k] = plant.y
        y_meas[k] = quantize(plant.y + noise[k], quant_step) if quant_step > 0 else plant.y + noise[k]
        u_cmd = controller.update(ref[k], y_meas[k], ff[k]) if ff.any() else controller.update(
            ref[k], y_meas[k])
        u[k] = actuator(u_cmd) if actuator is not None else u_cmd
        for name, fn in (record or {}).items():
            extra[name][k] = fn(plant)
        plant.step(u[k], d[k])
    t = np.arange(n) * Ts
    out = {"t": t, "r": ref, "y": y, "y_meas": y_meas, "u": u, "d": d, "e": ref - y}
    out.update(extra)
    return out


class Actuator:
    """Saturation and optional rate limit, stateful. Reset by `run_closed_loop`."""

    def __init__(self, u_min=-np.inf, u_max=np.inf, rate=None, u0=0.0):
        self.u_min, self.u_max, self.rate, self.u0 = u_min, u_max, rate, u0
        self.reset()

    def reset(self):
        self.last = self.u0

    def __call__(self, u):
        if self.rate is not None:
            u = float(np.clip(u, self.last - self.rate, self.last + self.rate))
        u = float(np.clip(u, self.u_min, self.u_max))
        self.last = u
        return u
