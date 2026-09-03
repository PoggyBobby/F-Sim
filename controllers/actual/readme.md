## controller structure
controllers within this directory should plug into the sre vcu
bindinings and convention copied over from [here](https://github.com/spartanracingelectric/SRE-VCU)
https://github.com/spartanracingelectric/SRE-VCU
will go inside of dev/pid or controls make sure to look/read through this
repo to get a full understanding of the codebase

controllers:
s_diff.c:
- no external torque applied onto motors, only reverse current to slow down car.
- no open differential, state estimators on each wheel
