"""Cascade speed controller: outer speed PI (slow) commands the reference of an inner current PI.

The inner loop reads the motor current directly (`motor.current`), which is how a drive with a
current sensor works. The outer loop runs every `ratio` calls. Current limiting is done by the
saturation of the outer controller output at +-i_max, with back-calculation anti-windup in both
loops. Optional back-EMF feed-forward adds Ke * speed to the voltage command.
"""

from __future__ import annotations

from control_lab.controllers.pid import PID


class CascadeSpeedController:
    def __init__(self, motor, speed_pid: PID, current_pid: PID, ratio: int, ke: float = 0.0,
                 bemf_ff: bool = False):
        if ratio < 1:
            raise ValueError("ratio must be >= 1")
        self.motor, self.speed_pid, self.current_pid = motor, speed_pid, current_pid
        self.ratio, self.ke, self.bemf_ff = int(ratio), float(ke), bool(bemf_ff)
        self.reset()

    def reset(self) -> None:
        self.speed_pid.reset()
        self.current_pid.reset()
        self._count = 0
        self.i_ref = 0.0

    def update(self, r: float, y: float, u_ff: float = 0.0) -> float:
        if self._count % self.ratio == 0:
            self.i_ref = self.speed_pid.update(r, y)
        self._count += 1
        ff = self.ke * y if self.bemf_ff else 0.0
        return self.current_pid.update(self.i_ref, self.motor.current, ff + u_ff)
