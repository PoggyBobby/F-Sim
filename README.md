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

## Commands to run

```
# first time setup, installs numpy matplotlib pyyaml scipy openpyxl
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# build the firmware (both configs) — needed before anything else
git submodule update --init sil/SRE-VCU
make -C sil

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
