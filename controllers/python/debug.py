"""The controller → plant interface, and the per-run telemetry that rides with it.

Every controller's update() returns one of these, whatever its internal law.
`model/sim.py` reads `.T` to drive the plant and logs the rest; `verify.py`
asserts on the named fields; `sil/vcu_sil.py` fills it from the firmware's CAN
commands. Keeping it in its own module means the SIL adapter does not have to
import the Python s-diff just to describe its own output.

TORQUES ARE A 4-TUPLE, wheel order FL, FR, RL, RR — the same order as
WHEEL_NAMES, wheel_xy, Fz and every force list in the plant. A rear-drive
controller simply leaves the front two at zero; there is no drive-layout flag.

T_FL/T_FR/T_RL/T_RR are READ-ONLY views onto that tuple. They exist so the
named reads scattered through verify.py and sim.py keep working, but they
cannot be assigned — set `T` instead, so there is exactly one place a torque
command can come from.
"""

from dataclasses import dataclass


@dataclass
class ControllerDebug:
    # ---- the plant interface ------------------------------------------
    T: tuple = (0.0, 0.0, 0.0, 0.0)   # wheel torques [N·m], FL FR RL RR

    # ---- telemetry -----------------------------------------------------
    dw_target: float = 0.0    # target wheel-speed difference wRR-wRL [rad/s]
    dT_sdiff: float = 0.0     # s-diff torque-split contribution [N·m]
    delta_norm: float = 0.0   # |steer| / delta_max, 0..1
    f_applied: float = 1.0    # shared friction-budget multiplier (slewed)
    g_left_appl: float = 1.0  # left-wheel multiplier (slewed)
    g_right_appl: float = 1.0 # right-wheel multiplier (slewed)

    # ---- read-only named views on T ------------------------------------
    @property
    def T_FL(self) -> float:
        return self.T[0]

    @property
    def T_FR(self) -> float:
        return self.T[1]

    @property
    def T_RL(self) -> float:
        return self.T[2]

    @property
    def T_RR(self) -> float:
        return self.T[3]
