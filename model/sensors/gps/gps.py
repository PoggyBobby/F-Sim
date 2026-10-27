"""GPS ground-speed receiver (two fitted) — rate, latency, noise, and the
slowly wandering speed error the plant carries as a state.

    reported(t) = |v_antenna|(t − latency) + b + n        n ~ N(0, σ²)
    b_{k+1}     = b_k + w_k,  w_k ~ N(0, q² · dt)          (random walk)

The receiver updates at its own rate and the VCU holds the last value in
between; the latency is a whole number of VCU cycles. The walk is drawn by
the suite (one RNG for everything) and applied to the plant's bias state,
so the bias in the log IS the bias the reading carried.

Reported in km/h on CAN at can_scale per LSB (the DAQ frame the firmware
already parses), quantised there, handed on in m/s.

Values: model/sensors/gps/params.yaml.
"""

from model.config import cfg
from model.sensors.quantize import quant

KMH = 1.0 / 3.6


class GpsSpeed:
    def __init__(self, rng, noise=True):
        self.rng = rng
        self.noise = noise
        p = cfg.sensors.gps
        self.rate_hz = p.rate_hz
        self.latency = p.latency
        self.sigma = p.speed_noise_std if noise else 0.0
        self.quant_kmh = p.can_scale if noise else 0.0
        self.reset()

    def reset(self):
        self.buf = None
        self.hold = None
        self.n_hold = None
        self.count = 0

    def _setup(self, dt_vcu):
        n_lat = int(round(self.latency / dt_vcu))
        self.buf = [0.0] * (n_lat + 1)
        self.n_hold = max(1, int(round(1.0 / (self.rate_hz * dt_vcu))))

    def read(self, v_true, bias, dt_vcu):
        """True antenna speed [m/s] and the plant's bias state → reported
        speed [m/s]. Draws ONE noise sample per VCU cycle when noise is on."""
        if self.buf is None:
            self._setup(dt_vcu)
        self.buf.append(v_true)
        delayed = self.buf.pop(0)
        n = self.rng.normal(0.0, self.sigma) if self.noise else 0.0
        if self.hold is None or self.count % self.n_hold == 0:
            kmh = quant((delayed + bias + n) / KMH, self.quant_kmh)
            self.hold = max(kmh, 0.0) * KMH
        self.count += 1
        return self.hold
