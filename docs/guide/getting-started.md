1.install depedencies

```
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

2. verify the test suite and model functionality(~90 seconds)
should generally be run after every commit to verify
```
.venv/bin/python verify.py
```
3. start first run
```
.venv/bin/python run_sim.py --maneuver step_steer --no-animate
```

4. view run, all run outputs are stored in runs/
can go to runs/latest for all the info along with plots and metrics along with replay videos
