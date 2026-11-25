"""Electric motor models: brushed DC, BLDC (six-step) and PMSM (dq)."""

from control_lab.motors.bldc import BLDCMotor, BLDCParams, hall_sector, trapezoid_shape
from control_lab.motors.dc_motor import DCMotor, DCMotorParams
from control_lab.motors.pmsm import PMSM, PMSMParams

__all__ = [
    "BLDCMotor",
    "BLDCParams",
    "DCMotor",
    "DCMotorParams",
    "PMSM",
    "PMSMParams",
    "hall_sector",
    "trapezoid_shape",
]
