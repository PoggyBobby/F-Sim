## S-diff simulator
what it is? Temporary solution to try out FSAE VCU code(S-diff control algorithms) to sanity check them 
before running in the actual car.
Note that there are a few fidelity errors and data mismatches that should be tuned and handled before the controller
is actually tested on the car. 

## Commands to run

```
# first time setup, installs numpy matplotlib pyyaml scipy openpyxl
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# verify if the physics are correct, unit tests per operation
.venv/bin/python verify.py

# run the actual sim, assumes the controller is in python no SIL yet
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

# bypass the sensors so the controller reads the sims true state
# splits controller error from estimation error
.venv/bin/python run_sim.py --perfect-state --no-animate

# rebuild the team spreadsheet after changing any number
.venv/bin/python param_sheet.py             # writes docs/datasheets/FSAE-Sim Parameters.xlsx

# change a number, always the yaml never the py
$EDITOR model/physical/tires/params.yaml    # grip, magic formula coefficients
$EDITOR model/physical/mass/params.yaml     # car and driver mass
$EDITOR model/physical/geometry/params.yaml # wheelbase, track, cg, yaw inertia
$EDITOR controllers/python/params.yaml      # the tuned controller gains

# then re verify, re run, re generate the sheet
.venv/bin/python verify.py && .venv/bin/python run_sim.py --no-animate
.venv/bin/python param_sheet.py

# read one value or its full documentation
.venv/bin/python -c "from model.config import cfg; print(cfg.tires.mu0)"
.venv/bin/python -c "import json; from model.config import cfg; print(json.dumps(cfg.meta('mass.car_no_driver'), indent=2))"

# list every number that is still a guess, 16 of them
.venv/bin/python -c "
from model.config import cfg
for p in cfg.params():
    if cfg.tag_of(p) == 'PLACEHOLDER': print(' ', p)"

# fit the nine magic formula coefficients from ttc data
# needs the restricted .mat files in ttc/, gitignored, member teams only
.venv/bin/python tire_fit.py --cornering ttc/*run31.mat --drivebrake ttc/*run72.mat --pressure 12 --out ttc/fit_hoosier_r20

# build the real vcu firmware and run it as a fifth config
git submodule update --init sil/SRE-VCU
make -C sil
.venv/bin/python run_sim.py --sil --maneuver step_steer --no-animate

# if make fails on incompatible pointer types your compiler is newer than the firmware
# pre existing, not a sim problem, this builds it anyway
make -C sil CC=clang CFLAGS="-std=gnu11 -O0 -g -w -Wno-error=incompatible-pointer-types -Wno-error=int-conversion -include host/host_types.h -Ihost -ISRE-VCU/inc -Ibuild/src"

```
