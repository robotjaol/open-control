"""PLC scan-cycle emulation and IEC 61131-3 style standard function blocks.

This is a Python emulation of how a PLC executes a program, used to study timing and logic. It is
not a PLC runtime, not certified, and not a substitute for testing the real program on the target
controller. Scan model: read all inputs into an input image, execute the program once with
dt = scan time, then write the output image. Inputs that change during a scan are seen on the
next scan.
"""

from __future__ import annotations

from collections.abc import Callable

_EPS = 1e-12


class TON:
    """On-delay timer. Q goes true once IN has been true for at least PT seconds."""

    def __init__(self, pt: float):
        self.pt, self.et, self.q = float(pt), 0.0, False

    def update(self, in_: bool, dt: float) -> bool:
        if in_:
            self.et = min(self.et + dt, self.pt)
            self.q = self.et >= self.pt - _EPS
        else:
            self.et, self.q = 0.0, False
        return self.q


class TOF:
    """Off-delay timer. Q stays true for PT seconds after IN falls."""

    def __init__(self, pt: float):
        self.pt, self.et, self.q, self._prev = float(pt), 0.0, False, False

    def update(self, in_: bool, dt: float) -> bool:
        if in_:
            self.et, self.q = 0.0, True
        else:
            if self._prev:
                self.et = 0.0
            if self.q:
                self.et = min(self.et + dt, self.pt)
                self.q = self.et < self.pt - _EPS
        self._prev = in_
        return self.q


class R_TRIG:
    """Rising edge detector (true for one scan)."""

    def __init__(self):
        self._prev = False

    def update(self, clk: bool) -> bool:
        q = bool(clk) and not self._prev
        self._prev = bool(clk)
        return q


class F_TRIG:
    """Falling edge detector (true for one scan)."""

    def __init__(self):
        self._prev = False

    def update(self, clk: bool) -> bool:
        q = (not clk) and self._prev
        self._prev = bool(clk)
        return q


class CTU:
    """Up counter. Counts rising edges of CU, Q true when CV >= PV, R resets."""

    def __init__(self, pv: int):
        self.pv, self.cv, self._edge = int(pv), 0, R_TRIG()

    def update(self, cu: bool, reset: bool = False) -> bool:
        edge = self._edge.update(cu)  # always track the input so edges are never missed
        if reset:
            self.cv = 0
        elif edge:
            self.cv += 1
        return self.cv >= self.pv


class SRLatch:
    """Set/reset latch. reset_dominant=True is the safe choice for run commands."""

    def __init__(self, reset_dominant: bool = True):
        self.q, self.reset_dominant = False, reset_dominant

    def update(self, set_: bool, reset: bool) -> bool:
        if self.reset_dominant:
            self.q = (set_ or self.q) and not reset
        else:
            self.q = set_ or (self.q and not reset)
        return self.q


class Program:
    """Base class: subclass and implement `scan(inputs, dt) -> outputs`."""

    def scan(self, inputs: dict, dt: float) -> dict:  # pragma: no cover - interface
        raise NotImplementedError


def run_scans(program: Program, input_fn: Callable[[float], dict], scan_time: float,
              t_end: float) -> list[dict]:
    """Execute `program` cyclically and return one record per scan.

    Each record has t (time at the start of the scan), the input image and the output image that
    became visible at the end of the scan.
    """
    n = int(round(t_end / scan_time))
    records = []
    for k in range(n):
        t = k * scan_time
        image = dict(input_fn(t))  # inputs latched at the start of the scan
        out = program.scan(image, scan_time)
        records.append({"t": t, "in": image, "out": dict(out)})
    return records
