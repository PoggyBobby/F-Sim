"""Software-in-the-loop: the real VCU firmware is the only controller.

`make -C sil` builds two binaries from the SRE-VCU sources (see sil/Makefile):
build/vcu_sil, the firmware as flashed, and build/vcu_sil_open, the same code
with main.c's SDiff_new(TRUE) flipped to FALSE. make_configs() turns them into
the two configurations every run compares:

    "open diff"  firmware, s-diff switch off — both motors get the same command
    "s-diff"     firmware as flashed — sdiff.c cuts the inner wheel

This module runs a binary as a child process and makes it look like a
controller to sim.py: every VCU cycle the sensor readings go down a pipe as
one text line and what the firmware put on the rear inverters' command frames
comes back (protocol: sil/host/sil_link.h).

Each run boots a fresh VCU (reset() restarts the process) and first lets
it run its power-up sequence — bench check, ADC settle — by feeding idle
frames until the firmware's own clock passes VCU_SIL_BOOT_S. Only then
does the maneuver's t = 0 begin.

The firmware commands the inverters in duty cycle or current, not torque, so
the sim converts: T_wheel = mA x motor_kt x gear_ratio, clipped to what the
motor can physically deliver — its peak torque, and its peak power at the
wheel's measured speed (|T| <= P_peak / |w|). That clip is motor physics, not
control: the firmware has no power limiting of its own, and nothing else in
the sim enforces the 80 kW rules cap (driven_wheels x motor_power_peak equals
it exactly, so the per-motor clip bounds the total). (motor_kt is a
PLACEHOLDER — everything put on the plant is scaled by an unmeasured number.)

FOUR WHEELS, TWO COMMANDS. The plant is AWD but this firmware is a two-motor
build: powertrainControl.c zeroes motor_fl/motor_fr at construction and nothing
ever writes them, and the CAN capture in sil/host/sil_link.c only knows
0x100 (RL) and 0x101 (RR). So the SIM mirrors each rear command onto the
same-side front — T_FL = T_RL, T_FR = T_RR. That is a SIM-SIDE assumption
about how a four-motor firmware would behave, NOT observed VCU behaviour, and
it doubles the total thrust this config puts down. summary() says so on every
run so nobody reads a four-motor SIL trace as firmware truth.

The firmware is already structurally ready for the real thing:
powertrainControl.h declares PowertrainMode { ... AWD = 3 ... } and a
_Powertrain with motor_fl/fr/rl/rr. Wiring it properly needs three things,
none of them ours: front inverter CAN IDs in sil/host/sil_link.c, rpm_FL/FR
and cmd_FL/FR added to the S and T lines in sil/host/sil_link.h, and a mode
that writes the front fields. Worth doing alongside the standing finding that
SafetyChecker_reduceTorque() computes its multiplier and discards it
(safety.c:688-691) — under AWD that failure fails to cut four motors, not two.
"""

import os
import shutil
import subprocess
from dataclasses import dataclass

from model.config import cfg

# This module lives in sil/, next to the Makefile that builds the binaries.
SIL_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_VCU_DIR = os.path.join(SIL_DIR, "SRE-VCU")     # the git submodule
_BUILD = os.path.join(SIL_DIR, "build")
SIL_EXE = os.path.join(_BUILD, "vcu_sil")            # s-diff on (as flashed)
SIL_OPEN_EXE = os.path.join(_BUILD, "vcu_sil_open")  # s-diff switch off
SIL_NAME = "s-diff"
OPEN_NAME = "open diff"
MODE_NAMES = {0: "none", 1: "duty", 2: "current"}


class FirmwareError(RuntimeError):
    """The firmware could not be found or built. The message says how to fix it."""


def vcu_source_dir():
    """Where the firmware source comes from: $SRE_VCU_DIR if set (any SRE-VCU
    checkout), otherwise the sil/SRE-VCU submodule."""
    return os.path.abspath(os.environ.get("SRE_VCU_DIR") or DEFAULT_VCU_DIR)


def ensure_firmware(log=print):
    """Make sure both firmware binaries exist AND match their source, building
    them if not. Called before every run and every audit, so an edited or
    updated firmware can never run as a stale binary.

    Raises FirmwareError, with the fix spelled out, if the source is missing
    or does not compile. See README.md, "VCU firmware (SIL) setup"."""
    vcu = vcu_source_dir()
    custom = vcu != os.path.abspath(DEFAULT_VCU_DIR)
    main_c = os.path.join(vcu, "dev", "main.c")

    # 1. the source. An empty sil/SRE-VCU means the submodule was never
    #    fetched — try once, then explain.
    if not os.path.isfile(main_c) and not custom:
        log("  firmware: sil/SRE-VCU is empty — fetching the submodule ...")
        subprocess.run(["git", "submodule", "update", "--init", "sil/SRE-VCU"],
                       cwd=os.path.dirname(SIL_DIR), capture_output=True)
    if not os.path.isfile(main_c):
        where = (f"SRE_VCU_DIR={vcu}" if custom else "sil/SRE-VCU")
        raise FirmwareError(
            f"no VCU firmware source at {where} (expected dev/main.c there).\n"
            "  Fix:  git submodule update --init sil/SRE-VCU\n"
            "  or point at your own checkout:  export SRE_VCU_DIR=/path/to/SRE-VCU\n"
            "  (README.md, 'VCU firmware (SIL) setup')")

    # 2. the toolchain
    if shutil.which("make") is None or not (shutil.which("cc") or shutil.which("clang")
                                            or shutil.which("gcc")):
        raise FirmwareError(
            "building the firmware needs make and a C compiler.\n"
            "  macOS: xcode-select --install    Linux: apt install build-essential")

    # 3. build. sil/build/.vcu_source records which source tree the build
    #    directory holds. A different (or unknown) one means a clean rebuild —
    #    make's timestamps cannot tell two checkouts apart. It is written BEFORE
    #    building and deleted on failure, so a failed build never leaves a
    #    half-built tree that the next attempt would trust.
    stamp = os.path.join(_BUILD, ".vcu_source")
    last = open(stamp).read().strip() if os.path.exists(stamp) else None
    make = ["make", "-C", SIL_DIR, "-s", f"VCU={vcu}"]
    if last == vcu and os.path.exists(SIL_EXE) and os.path.exists(SIL_OPEN_EXE) \
            and subprocess.run(make + ["-q"], capture_output=True).returncode == 0:
        return                                       # built from this source, current
    if last != vcu:
        subprocess.run(["make", "-C", SIL_DIR, "-s", "clean"], capture_output=True)
    os.makedirs(_BUILD, exist_ok=True)
    with open(stamp, "w") as fh:
        fh.write(vcu + "\n")
    log(f"  firmware: building both configs from {vcu} (make -C sil) ...")
    r = subprocess.run(make, capture_output=True, text=True)
    if r.returncode != 0 or not (os.path.exists(SIL_EXE) and os.path.exists(SIL_OPEN_EXE)):
        os.remove(stamp)
        tail = "\n".join((r.stderr or r.stdout).strip().splitlines()[-15:])
        raise FirmwareError(
            "the VCU firmware did not compile:\n" + tail + "\n"
            "  Rebuild by hand to see everything:  make -C sil\n"
            "  A firmware change may also have broken a patch in sil/patches/ "
            "or the open-diff build (it needs SDiff_new(TRUE) in main.c).")
    log("  firmware: built.")


@dataclass
class ControllerDebug:
    """What a controller hands the plant each VCU cycle: four wheel torques
    [N·m] in FL, FR, RL, RR order, plus the one diagnostic the metrics use.
    sim.py reads `.T`; verify.py's physics fixtures return one too."""
    T: tuple = (0.0, 0.0, 0.0, 0.0)
    dw_target: float = 0.0    # kinematic rear wheel-speed difference wRR-wRL [rad/s]

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


class VcuProcess:
    """One running VCU binary and the line protocol to it."""

    def __init__(self, exe=SIL_EXE):
        if not os.path.exists(exe):
            raise FirmwareError(
                f"{exe} not found. Build it with ensure_firmware() or "
                "make -C sil (README.md, 'VCU firmware (SIL) setup').")
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

    def __init__(self, vp, exe=SIL_EXE, name=SIL_NAME):
        self.vp = vp
        self.name = name
        self.vcu = VcuProcess(exe)
        self.plaus_cut = False          # the firmware's business, not ours
        self.t = 0.0
        self.cycles = {}                # mode -> cycles seen
        self.cmd_max = {}               # mode -> max |cmd|

    def reset(self):
        self.vcu.start()
        self.t = 0.0
        self.cycles, self.cmd_max = {}, {}

    def update_from_sensors(self, sr, dt):
        self.t += dt
        _, mode, cmd_RL, cmd_RR = self.vcu.exchange(
            self.t, sr.apps_pct, sr.bps_bar, sr.handwheel_deg,
            sr.motor_rpm_RL, sr.motor_rpm_RR, sr.yaw_rate, sr.ax, sr.ay)
        self.cycles[mode] = self.cycles.get(mode, 0) + 1
        self.cmd_max[mode] = max(self.cmd_max.get(mode, 0),
                                 abs(cmd_RL), abs(cmd_RR))
        k = self.vp.motor_kt * self.vp.gear_ratio / 1000.0   # N·m per mA at the wheel
        # four driven wheels: mirror each rear command onto the same-side front
        # (see the module docstring — a sim-side assumption, not firmware
        # behaviour). Two driven wheels: the fronts roll free.
        rear = (cmd_RL * k, cmd_RR * k)
        front = rear if self.vp.driven_wheels == 4 else (0.0, 0.0)
        speeds = (sr.wheel_speed_FL, sr.wheel_speed_FR,
                  sr.wheel_speed_RL, sr.wheel_speed_RR)
        T = tuple(self._motor_clip(t, w) for t, w in zip(front + rear, speeds))
        dbg = ControllerDebug(T=T)
        dbg.dw_target = sr.yaw_rate * self.vp.track_r / self.vp.r_wheel   # diagnostic only
        return dbg

    def _motor_clip(self, T, w):
        """What one motor can deliver at wheel speed w [rad/s]: its peak
        torque, and its peak power once P_peak / |w| falls below that."""
        T_max = self.vp.T_wheel_max
        if abs(w) > 0.0:
            T_max = min(T_max, self.vp.motor_P_peak / abs(w))
        return max(-T_max, min(T_max, T))

    def summary(self):
        parts = []
        for mode in sorted(self.cycles):
            n, m = self.cycles[mode], self.cmd_max[mode]
            unit = {1: f"max {m / 1e5:.1%} duty", 2: f"max {m / 1e3:.1f} A"}
            parts.append(f"{MODE_NAMES[mode]} x{n}"
                         + (f" ({unit[mode]})" if mode in unit else ""))
        return ("VCU command frames per cycle: " + ", ".join(parts)
                + f" — torque to plant = mA x Kt x gear ({self.vp.motor_kt} N·m/A"
                + f" x {self.vp.gear_ratio:g}), clipped to the motor's "
                + f"{self.vp.T_wheel_max:.0f} N·m / {self.vp.motor_P_peak / 1e3:.0f} kW"
                + ("; FRONTS MIRRORED from the rears (T_FL = T_RL, T_FR = T_RR)"
                   " — the firmware commands two motors, not four"
                   if self.vp.driven_wheels == 4 else ""))


def make_configs(vp):
    """The two configurations every run compares, both the real firmware.
    Order is fixed: verify.py indexes it."""
    return [SilController(vp, SIL_OPEN_EXE, OPEN_NAME),
            SilController(vp, SIL_EXE, SIL_NAME)]
