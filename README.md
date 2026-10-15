## S-diff simulator
what it is? Temporary solution to try out FSAE VCU code(S-diff control algorithms) to sanity check them 
before running in the actual car.
Note that there are a few fidelity errors and data mismatches that should be tuned and handled before the controller
is actually tested on the car. 

### The controller is the real VCU firmware

There is no Python controller. Every run compares two builds of the SRE-VCU
firmware (the `S-diff` branch, pinned in `sil/SRE-VCU`), compiled for the host
and run software-in-the-loop:

    open diff   main.c with SDiff_new(FALSE) — the firmware's own s-diff switch
                off, so both motors get the same command
    s-diff      the firmware as flashed — sdiff.c cuts the inner wheel

Each VCU cycle the sim sends the firmware a sensor frame (pedal, brake,
steering, wheel speeds, IMU) and reads back the current it commands each rear
inverter. That current becomes wheel torque through Kt × gear ratio, clipped to
what the motor can deliver (peak torque, peak power at the wheel's speed).
Kt is a PLACEHOLDER. The plant drives four wheels; the firmware commands two
motors, so with `driven_wheels: 4` each rear command is mirrored onto the
same-side front — a sim-side assumption, printed with every run.

100% pedal means what it means in the firmware: MAX_MOTOR_CURRENT_MA (70 A)
through Kt and the gearbox. A maneuver's scripted torque is turned into the
pedal position that produces it, using that same map.

### VCU firmware (SIL) setup

The sim cannot run without the VCU firmware: it is the only controller.

**Where it goes.** The firmware source lives at `sil/SRE-VCU/`. That folder is
a git submodule of [spartanracingelectric/SRE-VCU](https://github.com/spartanracingelectric/SRE-VCU),
pinned to a specific commit on the `S-diff` branch (currently `5df1baf`). Do
not copy files into it and do not edit it: the build copies the sources to
`sil/build/src/` and applies the fixes in `sil/patches/` there.

**First time.** After cloning this repo:

```
git submodule update --init sil/SRE-VCU
```

If you forget, `run_sim.py` and `verify.py` try this once on their own.

**Building.** Nothing to do by hand. Every `run_sim.py` and `verify.py` run
checks that both binaries (`sil/build/vcu_sil` and `sil/build/vcu_sil_open`)
exist and are newer than their source, and rebuilds them if not. You only need
`make` and a C compiler (macOS: `xcode-select --install`; Linux:
`apt install build-essential`). To build or debug by hand:

```
make -C sil            # both binaries
make -C sil clean      # start over
```

**Testing your own firmware.** To run the sim against a different SRE-VCU
checkout, such as a branch you are working on, point at it instead of editing the
submodule:

```
export SRE_VCU_DIR=/path/to/your/SRE-VCU
.venv/bin/python run_sim.py --maneuver corner_exit
unset SRE_VCU_DIR      # back to the pinned submodule
```

Switching source trees triggers a clean rebuild automatically.

**Moving the pin to newer firmware.**

```
git -C sil/SRE-VCU fetch origin S-diff && git -C sil/SRE-VCU checkout <commit>
git add sil/SRE-VCU    # commit the new pin with your change
.venv/bin/python verify.py
```

Section F of `verify.py` checks the firmware against the constants in its own
`sdiff.c`. If the new firmware changes `MAX_MOTOR_CURRENT_MA`, update
`max_motor_current` in `model/physical/drivetrain/params.yaml` to match.

**If the build fails,** the error ends with the compiler's last lines. The usual
causes are a firmware change that broke a patch in `sil/patches/`, or `main.c`
no longer calling `SDiff_new(TRUE)` (the open-diff build flips that one call).
The inverter CAN IDs in `sil/host/sil_link.c` must also match the firmware's
`canOutput_sendDebugMessage1`.

## Commands to run

```
# first time setup, installs numpy matplotlib pyyaml scipy openpyxl
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# fetch the firmware source once (it is built automatically on each run)
git submodule update --init sil/SRE-VCU

# verify the physics and the firmware against its own sdiff.c
.venv/bin/python verify.py

# run the sim: open diff vs s-diff, both the real firmware
.venv/bin/python run_sim.py

# other flags to run with
.venv/bin/python run_sim.py --no-animate                    # skip video (~25 s/maneuver)
.venv/bin/python run_sim.py --maneuver step_steer --no-animate   # one maneuver, ~25 s
.venv/bin/python run_sim.py -l "ttc-tires" -n "first real tire fit" # label run


# Change the test point without editing files:
.venv/bin/python run_sim.py --corner-throttle 65 --step-deg 8 --slalom-hz 0.8
.venv/bin/python run_sim.py --ask          # prompts for each value, Enter keeps default


# run full workflow
./run.sh --defaults

# view results of run
cat runs/latest/summary.md      # metrics tables + what the run was
cat runs/latest/CHANGES.md      # what moved vs the previous run, and whether it was data or code
cat runs/latest/PARAMETERS.md   # every number used, with provenance
ls runs/latest/plots/ runs/latest/replay/

# run the corner matrix instead of the scripted maneuvers
.venv/bin/python run_sim.py --maneuver tracks --track 90deg --no-animate
.venv/bin/python run_sim.py --maneuver tracks --track split_mu --no-animate   # inner rear on low grip, controller not told
.venv/bin/python run_sim.py --maneuver tracks --track all --no-animate        # 30deg 45deg 90deg 120deg u_turn split_mu

# rebuild the team spreadsheet after changing any number
.venv/bin/python param_sheet.py             # writes docs/datasheets/FSAE-Sim Parameters.xlsx

# change a number, always the yaml never the py
$EDITOR model/physical/tires/params.yaml    # grip, magic formula coefficients
$EDITOR model/physical/mass/params.yaml     # car and driver mass
$EDITOR model/physical/geometry/params.yaml # wheelbase, track, cg, yaw inertia
# s-diff constants live in the firmware: sil/SRE-VCU/dev/sdiff.c

# then re verify, re run, re generate the sheet
.venv/bin/python verify.py && .venv/bin/python run_sim.py --no-animate
.venv/bin/python param_sheet.py

# read one value or its full documentation
.venv/bin/python -c "from model.config import cfg; print(cfg.tires.mu0)"
.venv/bin/python -c "import json; from model.config import cfg; print(json.dumps(cfg.meta('mass.car_no_driver'), indent=2))"

# list every number that is still a guess
.venv/bin/python -c "
from model.config import cfg
for p in cfg.params():
    if cfg.tag_of(p) == 'PLACEHOLDER': print(' ', p)"

# fit the nine magic formula coefficients from ttc data
# needs the restricted .mat files in ttc/, gitignored, member teams only
.venv/bin/python tire_fit.py --cornering ttc/*run31.mat --drivebrake ttc/*run72.mat --pressure 12 --out ttc/fit_hoosier_r20

# if make fails on incompatible pointer types your compiler is newer than the firmware
# pre existing, not a sim problem, this builds it anyway
make -C sil CC=clang CFLAGS="-std=gnu11 -O0 -g -w -Wno-error=incompatible-pointer-types -Wno-error=int-conversion -include host/host_types.h -Ihost -ISRE-VCU/inc -Ibuild/src"

```
