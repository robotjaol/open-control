"""Controllers: PID, tuning rules, fuzzy, MPC, FSM, PLC scan-cycle emulation and FOC."""

from control_lab.controllers.cascade import CascadeSpeedController
from control_lab.controllers.fsm import FSM
from control_lab.controllers.fuzzy import FuzzyPI, MamdaniFLC
from control_lab.controllers.mpc import LinearMPC, MPCController
from control_lab.controllers.pid import PID

__all__ = ["CascadeSpeedController", "FSM", "FuzzyPI", "LinearMPC", "MPCController", "MamdaniFLC", "PID"]
