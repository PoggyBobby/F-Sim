"""Software-in-the-loop: the real VCU firmware as a controller configuration.

`make -C sil` builds sil/build/vcu_sil from the SRE-VCU sources (see
sil/Makefile). This module runs that binary as a child process and makes
it look like one more controller to sim.py: every VCU cycle the sensor
readings go down a pipe as one text line and what the firmware put on the
rear inverters' command frames comes back (protocol: sil/host/sil_link.h).

Each run boots a fresh VCU (reset() restarts the process) and first lets
it run its power-up sequence — bench check, ADC settle — by feeding idle
frames until the firmware's own clock passes VCU_SIL_BOOT_S. Only then
does the maneuver's t = 0 begin.

The firmware commands the custom inverters in duty cycle or current, not
torque, and nothing feeds the sim's sensors into it yet. So for now the
plant receives ZERO torque from this config; the raw commands are counted
and reported (summary()) so the link is visibly alive.
"""

import os
import subprocess

from model.config import cfg
from controllers.python.torque_split import ControllerDebug

# This module lives in sil/, next to the Makefile that builds the binary.
SIL_EXE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "build", "vcu_sil")
SIL_NAME = "VCU (SIL)"
MODE_NAMES = {0: "none", 1: "duty", 2: "current"}


class VcuProcess:
    """One running VCU binary and the line protocol to it."""

    def __init__(self, exe=SIL_EXE):
        if not os.path.exists(exe):
            raise FileNotFoundError(
                f"{exe} not found — build the VCU first:  make -C sil")
        self.exe = exe
        self.proc = None
        self.t_vcu = 0.0

    def start(self):
        self.stop()
        self.proc = subprocess.Popen([self.exe], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True,
                                     bufsize=1)
        self.t_vcu = 0.0
        while self.t_vcu < cfg.sil.boot_s:       # let the firmware boot
            self.exchange(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def exchange(self, t, apps_pct, bps_bar, handwheel_deg, rpm_RL, rpm_RR,
                 yaw_rate, ax, ay):
        """One VCU cycle: send the sensor frame, return
        (t_vcu, mode, cmd_RL, cmd_RR)."""
        self.proc.stdin.write(
            f"S {t:.4f} {apps_pct:.3f} {bps_bar:.3f} {handwheel_deg:.3f} "
            f"{rpm_RL:.2f} {rpm_RR:.2f} {yaw_rate:.5f} {ax:.4f} {ay:.4f}\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            err = self.proc.stderr.read()
            raise RuntimeError(f"VCU process exited (code {self.proc.poll()})"
                               + (f":\n{err}" if err else ""))
        tag, t_vcu, mode, cmd_RL, cmd_RR = line.split()
        if tag != "T":
            raise RuntimeError(f"unexpected line from VCU: {line!r}")
        self.t_vcu = float(t_vcu)
        return self.t_vcu, int(mode), int(cmd_RL), int(cmd_RR)

    def stop(self):
        if self.proc is not None:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
            self.proc = None

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass


class SilController:
    """Controller-shaped wrapper: sensors in, wheel torques out."""

    name = SIL_NAME

    def __init__(self, vp, exe=SIL_EXE):
        self.vp = vp
        self.vcu = VcuProcess(exe)
        self.plaus_cut = False          # the firmware's business, not ours
        self.t = 0.0
        self.cycles = {}                # mode -> cycles seen
        self.cmd_max = {}               # mode -> max |cmd|

    def reset(self):
        self.vcu.start()
        self.t = 0.0
        self.cycles, self.cmd_max = {}, {}

    def update(self, s, delta, T_req_total, dt):
        raise RuntimeError("the SIL controller reads sensors only — "
                           "run without --perfect-state")

    def update_from_sensors(self, sr, dt):
        self.t += dt
        _, mode, cmd_RL, cmd_RR = self.vcu.exchange(
            self.t, sr.apps_pct, sr.bps_bar, sr.handwheel_deg,
            sr.motor_rpm_RL, sr.motor_rpm_RR, sr.yaw_rate, sr.ax, sr.ay)
        self.cycles[mode] = self.cycles.get(mode, 0) + 1
        self.cmd_max[mode] = max(self.cmd_max.get(mode, 0),
                                 abs(cmd_RL), abs(cmd_RR))
        return ControllerDebug()        # T_RL = T_RR = 0: no command→torque
                                        # model yet (duty / mA, not N·m)

    def summary(self):
        parts = []
        for mode in sorted(self.cycles):
            n, m = self.cycles[mode], self.cmd_max[mode]
            unit = {1: f"max {m / 1e5:.1%} duty", 2: f"max {m / 1e3:.1f} A"}
            parts.append(f"{MODE_NAMES[mode]} x{n}"
                         + (f" ({unit[mode]})" if mode in unit else ""))
        return ("VCU command frames per cycle: " + ", ".join(parts)
                + " — torque to plant = 0 (no command→torque model yet)")
