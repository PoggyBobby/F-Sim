"""
╔═══════════════════════════════════════════════════════════════════════════╗
║              CAR DATA LOADER  —  the numbers live in YAML                  ║
║                                                                           ║
║  Every number the simulation uses is set in a `params.yaml` next to the    ║
║  component it describes, and pulled from there by the rest of the code.    ║
║  This module finds those files, converts units, computes derived values,   ║
║  and exposes them as `cfg`.                                                ║
╚═══════════════════════════════════════════════════════════════════════════╝

WHERE THE NUMBERS ARE
─────────────────────
    model/physical/environment/params.yaml   g, air density
    model/physical/mass/params.yaml          car + driver mass
    model/physical/geometry/params.yaml      wheelbase, track, CG, yaw inertia
    model/physical/drivetrain/params.yaml    wheels, gearing, AMK motors, limits
    model/physical/aero/params.yaml          C_L, C_D, frontal area, balance
    model/physical/steering/params.yaml      Ackermann fraction
    model/physical/loads/params.yaml         lateral load-transfer split
    model/physical/numerical/params.yaml     solver guards
    model/physical/tires/params.yaml         Magic Formula coefficients
    model/sensors/<sensor>/params.yaml       one file per sensor
    controllers/python/params.yaml           the tuned controller gains
    sil/params.yaml                          software-in-the-loop timing

READING A VALUE
───────────────
    from model.config import cfg

    cfg.mass.car_no_driver          # 181.51 (kg, SI — always SI)
    cfg.sensors.imu_6axis.gyro_bias # 0.001745 (rad/s)
    cfg.tires.mu0                   # 1.737 (derived: road_scale * belt_mu_lat)

Attribute access only — a typo raises immediately instead of returning None.

    cfg.meta("mass.car_no_driver")  # the full entry: value, unit, status,
                                    # what/need/how/why, and the SI value

ENTRY SCHEMA
────────────
    car_no_driver:
      value:  400.1              # as measured, in the unit below
      unit:   lb                 # converted to SI on load; "-" = dimensionless
      symbol: m_car              # as written in the equations
      status: MEASURED 2026-08-31 (corner scales)
      what:   Complete ready-to-run vehicle, nobody in the seat.
      need:   Total car weight
      how:    Corner scales (Intercomp SW500)
      why:    With driver mass it sets EVERY inertial and grip force.

`status` is the provenance tag, and the vocabulary is fixed (see STATUS_TAGS):
FROM REPORT / DERIVED / CURRENT CAR / PLACEHOLDER / RULES VALUE / SUSPECT /
MEASURED / TTC FIT / TUNED (sim) / NUMERICAL GUARD / CONSTANT. Anything that
starts with one of those counts as that tag, so a date or a note can follow.

A derived entry carries a formula instead of a value:

    total:
      derived: mass.car_no_driver + mass.driver
      unit:   kg

The formula is evaluated against the already-loaded config, in dependency
order, so it can reference any other entry by its full dotted path. Keeping
the formula in the data file (rather than in code) is the point: the sheet and
the docs can show *how* a number is computed, not just what it came out as.

HOW TO EDIT
───────────
1. Values are entered in whatever unit you MEASURED in — set `unit` and the
   loader converts. Don't hand-convert.
2. When you replace a value, update its `status` (e.g.
   `MEASURED 2026-09-01 (corner scales)`) so we know which numbers are real.
3. After editing, just rerun:  .venv/bin/python run_sim.py
   NOTE: if vehicle/tire values change meaningfully, the controller gains in
   controllers/python/params.yaml must be retuned.
4. Regenerate the team sheet with:  .venv/bin/python param_sheet.py

(Test-maneuver settings — speeds, steer angles, throttle profiles — are
scenario definitions, not car data; those live in model/maneuvers/.)
"""

import math
import os

import yaml


# ─────────────────────────────────────────────────────────────────────────
# UNIT CONVERSIONS — an entry's `unit` picks one; the code always sees SI
# ─────────────────────────────────────────────────────────────────────────
LB = 0.45359237       # kg per pound-mass         (weight/mass)
INCH = 0.0254         # m per inch                (lengths)
FT = 0.3048           # m per foot
LBFT = 1.3558179      # N·m per lb-ft             (torque)
MPH = 0.44704         # m/s per mph               (speed)
HP = 745.699872       # W per horsepower          (power)
DEG = math.pi / 180.0  # rad per degree            (angles)
RPM = math.pi / 30.0  # rad/s per rev-per-minute  (rotational speed)

UNITS = {
    # mass
    "kg": 1.0, "lb": LB,
    # length
    "m": 1.0, "in": INCH, "ft": FT, "mm": 1e-3,
    # angle / rotation
    "rad": 1.0, "deg": DEG,
    "rad/s": 1.0, "deg/s": DEG, "rpm": RPM,
    # speed
    "m/s": 1.0, "mph": MPH, "km/h": 1.0 / 3.6,
    # force / torque / power / pressure
    "N": 1.0, "N*m": 1.0, "lbft": LBFT,
    "N*m/rad": 1.0, "N*m/(rad/s)": 1.0,
    "W": 1.0, "hp": HP,
    "bar": 1.0, "V": 1.0,
    # composite / dimensionless
    "m/s^2": 1.0, "kg/m^3": 1.0, "kg*m^2": 1.0, "m^2": 1.0,
    "Hz": 1.0, "s": 1.0, "%": 1.0, "1/rad": 1.0, "1/slip": 1.0,
    "-": 1.0,
    # Per-LSB resolutions. These are NOT converted: a sensor's quantization is
    # applied in the unit that sensor reports in (the SAS reads handwheel
    # degrees, the AMK resolver reports motor rpm), and the sensor modules
    # quantize before converting. Converting here would quantize in the wrong
    # unit and silently change every reading.
    "deg/LSB": 1.0, "rpm/LSB": 1.0, "%/LSB": 1.0, "bar/LSB": 1.0,
}

# Provenance vocabulary. Order matters: longest-first so "TUNED (sim)" is not
# swallowed by a shorter prefix. runlog.py and param_sheet.py both key off it.
STATUS_TAGS = (
    "NUMERICAL GUARD", "RULES VALUE", "CURRENT CAR", "PLACEHOLDER",
    "FROM REPORT", "TUNED (sim)", "TTC FIT", "MEASURED", "DERIVED",
    "SUSPECT", "CONSTANT",
)

# Tags meaning "confirmed for the car we are actually building" — param_sheet
# colours these green.
GREEN_TAGS = {"CURRENT CAR", "MEASURED", "TTC FIT"}

REQUIRED_FIELDS = ("unit", "what")

# Repo root = the directory containing model/. Every search path is relative
# to it, so the sim runs the same from any working directory.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SEARCH_DIRS = ("model", "controllers", "sil")


class ConfigError(Exception):
    """A params.yaml is malformed, incomplete, or contradicts another."""


class Section:
    """A dotted namespace of parameters. Attribute access, no silent None."""

    def __init__(self, path):
        self._path = path
        self._values = {}
        self._subs = {}

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        if name in self._values:
            return self._values[name]
        if name in self._subs:
            return self._subs[name]
        known = sorted(list(self._values) + list(self._subs))
        where = self._path or "cfg"
        raise AttributeError(
            f"{where} has no parameter '{name}'. Available: {', '.join(known)}"
        )

    def __dir__(self):
        return sorted(list(self._values) + list(self._subs))

    def __repr__(self):
        return f"<Section {self._path or 'cfg'}: {len(self._values)} values, " \
               f"{len(self._subs)} subsections>"

    # -- construction -----------------------------------------------------
    def _sub(self, name):
        if name not in self._subs:
            child = Section(f"{self._path}.{name}" if self._path else name)
            self._subs[name] = child
        return self._subs[name]


class Config:
    """The loaded parameter set: `cfg.mass.car_no_driver`, `cfg.meta(path)`."""

    def __init__(self):
        self._root = Section("")
        self._meta = {}          # dotted path -> the full entry dict
        self._files = {}         # dotted namespace -> source file (repo-relative)
        self._sections = {}      # dotted namespace -> {title, order, about, file}

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        return getattr(self._root, name)

    def __dir__(self):
        return dir(self._root)

    # -- reads ------------------------------------------------------------
    def get(self, dotted):
        """Value at a dotted path, e.g. get('sensors.imu_6axis.gyro_bias')."""
        node = self._root
        for part in dotted.split("."):
            node = getattr(node, part)
        return node

    def meta(self, dotted):
        """The full entry for a path: value, si, unit, status, what/need/how/why."""
        try:
            return self._meta[dotted]
        except KeyError:
            raise ConfigError(f"no parameter at '{dotted}'") from None

    def params(self):
        """Every parameter, dotted path -> entry, in file/declaration order."""
        return dict(self._meta)

    def sections(self):
        """Namespace -> source file, in load order."""
        return dict(self._files)

    def section_list(self):
        """Sections in presentation order: [(namespace, {title, order, about,
        file}), ...]. param_sheet.py walks this to lay out the team sheet, so a
        new params.yaml appears in the sheet by declaring `order:` and `title:`
        — no edit to the generator."""
        return sorted(self._sections.items(),
                      key=lambda kv: (kv[1]["order"], kv[0]))

    def section_params(self, namespace):
        """The entries of one section, in the order they appear in its file."""
        return {path: e for path, e in self._meta.items()
                if e["namespace"] == namespace}

    def tag_of(self, dotted):
        """The provenance tag of a parameter, or 'UNTAGGED'."""
        status = str(self._meta[dotted].get("status", ""))
        for tag in STATUS_TAGS:
            if status.upper().startswith(tag.upper()):
                return tag
        return "UNTAGGED"

    # -- construction -----------------------------------------------------
    def _place(self, namespace, key, value):
        node = self._root
        for part in namespace.split("."):
            node = node._sub(part)
        node._values[key] = value


# ─────────────────────────────────────────────────────────────────────────
# loading
# ─────────────────────────────────────────────────────────────────────────
def _yaml_files(root=ROOT):
    """Every params.yaml under the search dirs, in a stable sorted order."""
    found = []
    for top in SEARCH_DIRS:
        base = os.path.join(root, top)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = sorted(d for d in dirnames
                                 if d not in ("__pycache__", "build", "SRE-VCU"))
            if "params.yaml" in filenames:
                found.append(os.path.join(dirpath, "params.yaml"))
    return sorted(found)


def _to_si(value, unit, where):
    if unit not in UNITS:
        raise ConfigError(
            f"{where}: unknown unit '{unit}'. Known units: "
            f"{', '.join(sorted(UNITS))}"
        )
    factor = UNITS[unit]
    # An integer that needs no conversion stays an integer. Counts, seeds and
    # sample sizes are used where a float is wrong (numpy's default_rng rejects
    # a float seed), so don't silently widen them.
    if factor == 1.0 and isinstance(value, int) and not isinstance(value, bool):
        return value
    return value * factor


def _check_entry(entry, where):
    if not isinstance(entry, dict):
        raise ConfigError(f"{where}: entry must be a mapping, got {type(entry).__name__}")
    for field in REQUIRED_FIELDS:
        if not entry.get(field):
            raise ConfigError(f"{where}: missing required field '{field}'")
    if "value" not in entry and "derived" not in entry:
        raise ConfigError(f"{where}: needs either 'value' or 'derived'")
    if "value" in entry and "derived" in entry:
        raise ConfigError(f"{where}: has both 'value' and 'derived' — pick one")
    if "value" in entry and not isinstance(entry["value"], (int, float)):
        raise ConfigError(f"{where}: 'value' must be a number, "
                          f"got {entry['value']!r}")


def _eval_derived(expr, cfg, where):
    """Evaluate a derived formula against the config loaded so far.

    Only dotted parameter paths, numbers and arithmetic — no builtins, no
    attribute tricks. A reference to a not-yet-loaded value raises, and the
    caller retries it on the next pass.
    """
    scope = {"__builtins__": {}}
    for name in dir(cfg):
        scope[name] = getattr(cfg, name)
    try:
        return eval(expr, scope)          # noqa: S307 — restricted scope above
    except AttributeError as exc:
        raise _Unresolved(str(exc)) from None
    except Exception as exc:
        raise ConfigError(f"{where}: derived formula {expr!r} failed: {exc}") from None


class _Unresolved(Exception):
    """A derived formula referenced something not loaded yet."""


def load(root=ROOT):
    """Read every params.yaml and return the populated Config."""
    cfg = Config()
    files = _yaml_files(root)
    if not files:
        raise ConfigError(f"no params.yaml found under {root}/{{{','.join(SEARCH_DIRS)}}}")

    pending = []        # derived entries, resolved after the plain values
    for path in files:
        rel = os.path.relpath(path, root)
        with open(path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
        namespace = doc.pop("namespace", None)
        if not namespace:
            raise ConfigError(f"{rel}: missing top-level 'namespace:'")
        if namespace in cfg._files:
            raise ConfigError(f"{rel}: namespace '{namespace}' already declared "
                              f"by {cfg._files[namespace]}")
        cfg._files[namespace] = rel
        cfg._sections[namespace] = {
            "title": doc.pop("title", namespace),
            "order": doc.pop("order", 9999),
            "about": (doc.pop("about", "") or "").strip(),
            "file": rel,
        }

        for key, entry in doc.items():
            dotted = f"{namespace}.{key}"
            where = f"{rel}:{key}"
            _check_entry(entry, where)
            record = dict(entry)
            record["path"] = dotted
            record["namespace"] = namespace
            record["name"] = key
            record["file"] = rel
            cfg._meta[dotted] = record
            if "derived" in entry:
                pending.append((dotted, record, where))
            else:
                si = _to_si(entry["value"], entry["unit"], where)
                record["si"] = si
                cfg._place(namespace, key, si)

    # Derived values, resolved by repeated passes so order in the files and
    # across files does not matter. Each pass must resolve at least one.
    while pending:
        still = []
        for dotted, record, where in pending:
            try:
                si = _eval_derived(record["derived"], cfg, where)
            except _Unresolved:
                still.append((dotted, record, where))
                continue
            if not isinstance(si, (int, float)):
                raise ConfigError(f"{where}: derived formula produced "
                                  f"{type(si).__name__}, expected a number")
            record["si"] = si
            record.setdefault("status", "DERIVED")
            cfg._place(record["namespace"], record["name"], si)
        if len(still) == len(pending):
            names = ", ".join(d for d, _, _ in still)
            raise ConfigError(
                "derived parameters could not be resolved (circular reference, "
                f"or a typo in a path): {names}"
            )
        pending = still

    return cfg


# The loaded configuration. Import failures here are deliberate: a missing or
# malformed params.yaml must stop the sim at import, not surface later as a
# quietly wrong number.
cfg = load()

# Two constants are used so widely that the physics modules import them by
# name rather than through cfg.
G = cfg.environment.g
RHO_AIR = cfg.environment.rho_air
