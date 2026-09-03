"""The one quantizer every sensor uses.

A sender reports in discrete steps: `quant(v, step)` snaps a true value onto
the nearest step. `step = 0` means "no quantization" — which is how the
noise-free comparison mode in SensorSuite disables it.

Quantization is always applied in the unit the SENSOR reports in (handwheel
degrees for the SAS, motor rpm for the resolver), never in SI. The `*/LSB`
units in the params.yaml files are passthrough for exactly this reason.
"""


def quant(v, step):
    return step * round(v / step) if step > 0 else v
