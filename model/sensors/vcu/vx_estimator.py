"""Ground-speed estimate — wheel speeds anchored by the accelerometer.

THE PROBLEM THIS EXISTS TO SOLVE. There is no ground-speed sensor on the car,
so the VCU infers vx from wheel speeds. With a REAR-drive car you can lean on
"the slowest driven wheel is closest to the truth", because a driven wheel only
ever over-reads under power. With FOUR driven wheels that argument collapses:
every wheel over-reads at once, min() is no longer a lower bound, and the
estimate follows the wheelspin it is supposed to detect. Any per-wheel slip
term computed from it — traction control, a spin guard, a torque allocator —
goes blind at exactly the moment it is needed.

THE FIX is the standard ABS/TC one: keep using the wheels, but stop trusting
them when they disagree with each other.

    v_i    = w_i·r_w + r·y_i        each wheel referred to the CG, not the hub
    v_wss  = max(v_i) braking, min(v_i) driving, over ALL FOUR
    spread = max v_i − min v_i               what CORNERING cannot explain
    trust  = clamp(1 − spread/spread_ref, 0, 1)      0 once they truly scatter
    vx    += ax·dt                                   predict
    vx    += trust·dt/(dt+tau)·(v_wss − vx)          correct, as far as trusted

The `+ r·y_i` is not a refinement, it is required. A wheel does not measure
the CG's speed, it measures its OWN contact patch's: in a corner the inner
wheels are genuinely slower by r·track/2, about 0.67 m/s at 1 rad/s on this
car. Referring every wheel back to the CG first does two jobs at once — it
removes that bias from the estimate, and it makes the remaining spread a
measure of SLIP rather than of geometry, which is what the trust gate wants.
The VCU has r from the gyro and the track widths from CAD, so this is free.

It degrades in the right direction: while the wheels agree this is the old
behaviour with a short lag; while they scatter it coasts on the accelerometer
for the 0.1–1 s a traction event lasts, then re-anchors.

Two honesty notes, because this estimator flatters itself in a simulator:

  * dvx/dt is really ax_meas + r·vy, and the VCU does not know vy. Dropping
    that term costs ~0.8 m/s² at 1.5 g and 3° of sideslip — about 0.4 m/s over
    a half-second event at 15 m/s. The wheels re-anchor it afterwards.
  * the sim's IMU models accelerometer NOISE but no accelerometer BIAS
    (imu.py applies gyro_bias only), so the integrated path here is unbiased
    in a way a real one is not. Add sensors.imu_6axis.accel_bias before
    quoting any drift number from this model.

Constants: model/sensors/vcu/params.yaml (vx_tau, vx_spread_ref).
"""

from model.config import cfg


class VxEstimator:
    """The VCU's ground-speed estimate. One per sensor suite; reset with it."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.vx = 0.0
        self._started = False

    def update(self, wheel_speeds, ax_meas: float, dt: float,
               braking: bool, r_wheel: float, yaw_rate: float = 0.0,
               wheel_y=(0.0, 0.0, 0.0, 0.0)) -> float:
        """wheel_speeds: four wheel speeds [rad/s], FL FR RL RR.
        wheel_y: each wheel's lateral offset from the CG [m], +y = LEFT."""
        # refer each wheel to the CG: its patch runs at vx − r·y, so the CG
        # runs at w·r_w + r·y
        speeds = [w * r_wheel + yaw_rate * y
                  for w, y in zip(wheel_speeds, wheel_y)]
        lo, hi = min(speeds), max(speeds)
        v_wss = hi if braking else lo

        if not self._started:              # first sample: trust the wheels
            self.vx = max(v_wss, 0.0)
            self._started = True
            return self.vx

        # the speeds are already CG-referred, so what is left of the spread is
        # slip, not geometry
        trust = 1.0 - (hi - lo) / cfg.sensors.vcu.vx_spread_ref
        trust = 0.0 if trust < 0.0 else (1.0 if trust > 1.0 else trust)

        # predict on the accelerometer, then correct toward the wheels only as
        # far as their agreement justifies. dt/(dt+tau) rather than a fixed
        # per-sample weight, so the filter behaves the same at the 100 Hz VCU
        # rate and at the 4 kHz rate verify I4 uses.
        self.vx += ax_meas * dt
        self.vx += trust * dt / (dt + cfg.sensors.vcu.vx_tau) * (v_wss - self.vx)
        if self.vx < 0.0:
            self.vx = 0.0
        return self.vx
