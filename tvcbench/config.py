"""
Run plans: the file that makes a run reproducible.

The old logger took its settings from an HTML form and wrote none of them into
the data, so `dwell_s`, the grid bounds and the repeat count were gone the moment
the browser closed -- and the measured step duration turned out to be 4.03 s
against a configured 2.0 with nobody able to say why. A plan is a file, it is
checked in, and it is copied verbatim into the run manifest.

Validation reports *every* problem at once rather than stopping at the first.
Fixing a plan one error per run is a bad way to spend a battery.
"""

import copy
import json
import os

SCHEMA_VERSION = 1

# Every key the runner understands, with the value used when a plan omits it.
# Nested exactly as the plan file is, so a plan can be read as a subset of this.
DEFAULTS = {
    "schema_version": SCHEMA_VERSION,

    # Grid bounds in microseconds: [start, end, step], both ends inclusive.
    # A is the fixed (outer) rotor, B the swept (inner) one.
    "grid": {"a": [1000, 1000, 100], "b": [1000, 2000, 100]},

    "order": {
        # sequential      -- as generated; A outer, B inner
        # random          -- one shuffle, reused for every repeat
        # blocked_random  -- each repeat independently shuffled (see sequence.py)
        "mode": "blocked_random",
        "seed": 0,                    # 0 means pick one and record it
    },

    "dwell": {
        # fixed      -- hold for `fixed_s`
        # settle     -- hold until thrust stops moving, then `hold_after_settle_s`
        # sem_target -- hold until the thrust standard error reaches the target
        "mode": "fixed",
        "fixed_s": 4.0,
        "min_s": 2.0,
        "max_s": 8.0,
        "hold_after_settle_s": 2.0,
        "settle_rate_n_per_s": 0.3,   # |d(thrust)/dt| below this counts as settled
        "settle_window_s": 0.5,       # ... sustained for this long
        "target_thrust_sem_n": 0.05,
    },

    # Periodic return to a fixed point, so battery and thermal drift can be
    # fitted as a covariate instead of merely bracketed. 0 disables.
    "reference": {"a": 1500, "b": 1500, "every_n_steps": 0},

    # Both rotors at minimum with the current near zero: this voltage is the
    # pack's actual state of charge, uncontaminated by IR drop under load.
    "idle": {"pre_s": 10.0, "post_s": 10.0},

    "tare": {"seconds": 10.0},
    "warmup": {"seconds": 3.0},
    "ramp": {"steps": 8, "seconds": 0.4},

    # A short square wave at each end of the run. Not needed for Pi-internal
    # timing -- there is only one clock now -- but it is a high-SNR feature for
    # validating the .ulg against the Pi during the transition off it.
    "chirp": {"enabled": True, "high_us": 1400, "cycles": 3, "half_period_s": 0.5},

    "repeats": 1,

    "limits": {
        "min_voltage_v": 0.0,          # 0 disables; 9.9 for a 3S pack
        "max_current_a": 0.0,
        "max_thrust_n": 0.0,
        "max_torque_nm": 0.0,
        "loadcell_dropout_s": 0.5,     # no force sample for this long -> abort
        "heartbeat_timeout_s": 3.0,
    },

    "hardware": {
        "fc": {
            "device": "/dev/ttyAMA0", "baud": 921600,
            "servo_hz": 50, "battery_hz": 20, "esc_hz": 20,
            "timeout_s": 1.0, "resend_hz": 5.0,
        },
        "loadcell": {"device": "/dev/tvc-loadcell", "baud": 115200, "nominal_hz": 50.0},
    },

    "meta": {"prop": "", "battery": "", "notes": "", "operator": ""},
}

DWELL_MODES = ("fixed", "settle", "sem_target")
ORDER_MODES = ("sequential", "random", "blocked_random")

PWM_MIN_US = 1000
PWM_MAX_US = 2000


class PlanError(ValueError):
    """Raised with every validation problem found, not just the first."""

    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__("\n".join(f"  - {p}" for p in self.problems))


def _deep_merge(base, override):
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_raw(path):
    """Read a plan file. YAML if it looks like YAML, otherwise JSON."""
    ext = os.path.splitext(path)[1].lower()
    with open(path, encoding="utf-8") as f:
        text = f.read()
    if ext in (".yaml", ".yml"):
        try:
            import yaml
        except ImportError:
            raise PlanError([f"{path} is YAML but PyYAML is not installed "
                             f"(pip install pyyaml, or write the plan as .json)"])
        return yaml.safe_load(text) or {}
    return json.loads(text)


def frange_us(start, end, step):
    """Inclusive microsecond values from start to end. Integers throughout."""
    values = []
    value = int(start)
    while value <= int(end):
        values.append(value)
        value += int(step)
    return values


class Plan:
    """A validated run plan. `raw` is kept verbatim for the manifest."""

    def __init__(self, raw, source=None):
        self.raw = copy.deepcopy(raw or {})
        self.source = source
        self.data = _deep_merge(DEFAULTS, self.raw)
        self._validate()

    # --- convenience accessors ---------------------------------------------
    def __getitem__(self, key):
        return self.data[key]

    def get(self, key, default=None):
        return self.data.get(key, default)

    @property
    def a_values(self):
        return frange_us(*self.data["grid"]["a"])

    @property
    def b_values(self):
        return frange_us(*self.data["grid"]["b"])

    @property
    def n_grid_points(self):
        return len(self.a_values) * len(self.b_values)

    # --- validation ---------------------------------------------------------
    def _validate(self):
        problems = []
        d = self.data

        version = d.get("schema_version")
        if version != SCHEMA_VERSION:
            problems.append(
                f"schema_version is {version!r}, this build understands {SCHEMA_VERSION}")

        for axis in ("a", "b"):
            problems += self._check_axis(axis, d["grid"].get(axis))

        order = d["order"]
        if order.get("mode") not in ORDER_MODES:
            problems.append(f"order.mode must be one of {ORDER_MODES}, "
                            f"got {order.get('mode')!r}")

        problems += self._check_dwell(d["dwell"])

        ref = d["reference"]
        if ref.get("every_n_steps", 0):
            if ref["every_n_steps"] < 1:
                problems.append("reference.every_n_steps must be >= 1 (0 disables it)")
            for key in ("a", "b"):
                if not PWM_MIN_US <= ref.get(key, 0) <= PWM_MAX_US:
                    problems.append(f"reference.{key} must be "
                                    f"{PWM_MIN_US}-{PWM_MAX_US}us, got {ref.get(key)}")

        if d["repeats"] < 1:
            problems.append(f"repeats must be >= 1, got {d['repeats']}")

        ramp = d["ramp"]
        if ramp["steps"] < 1:
            problems.append("ramp.steps must be >= 1")
        if ramp["seconds"] <= 0:
            problems.append("ramp.seconds must be > 0")

        for name in ("tare", "warmup"):
            if d[name]["seconds"] < 0:
                problems.append(f"{name}.seconds must be >= 0")
        for name in ("pre_s", "post_s"):
            if d["idle"][name] < 0:
                problems.append(f"idle.{name} must be >= 0")

        chirp = d["chirp"]
        if chirp.get("enabled"):
            if not PWM_MIN_US <= chirp["high_us"] <= PWM_MAX_US:
                problems.append(f"chirp.high_us must be {PWM_MIN_US}-{PWM_MAX_US}us")
            if chirp["cycles"] < 1 or chirp["half_period_s"] <= 0:
                problems.append("chirp.cycles must be >= 1 and half_period_s > 0")

        problems += self._check_fc(d["hardware"]["fc"])

        limits = d["limits"]
        if limits["loadcell_dropout_s"] <= 0:
            problems.append("limits.loadcell_dropout_s must be > 0 -- a run that "
                            "cannot notice a dead force sensor is not worth taking")
        if limits["heartbeat_timeout_s"] <= 0:
            problems.append("limits.heartbeat_timeout_s must be > 0")

        if not problems and self.n_grid_points == 0:
            problems.append("the grid is empty; check the bounds and step")

        if problems:
            raise PlanError(problems)

    @staticmethod
    def _check_axis(axis, spec):
        if not isinstance(spec, (list, tuple)) or len(spec) != 3:
            return [f"grid.{axis} must be [start, end, step], got {spec!r}"]
        start, end, step = spec
        problems = []
        for name, value in (("start", start), ("end", end)):
            if not PWM_MIN_US <= value <= PWM_MAX_US:
                problems.append(f"grid.{axis} {name} must be "
                                f"{PWM_MIN_US}-{PWM_MAX_US}us, got {value}")
        if end < start:
            problems.append(f"grid.{axis} end ({end}) is below start ({start})")
        if step <= 0:
            problems.append(f"grid.{axis} step must be > 0, got {step}")
        return problems

    @staticmethod
    def _check_dwell(dwell):
        problems = []
        mode = dwell.get("mode")
        if mode not in DWELL_MODES:
            return [f"dwell.mode must be one of {DWELL_MODES}, got {mode!r}"]

        if dwell["min_s"] <= 0:
            problems.append("dwell.min_s must be > 0")
        if dwell["max_s"] < dwell["min_s"]:
            problems.append(f"dwell.max_s ({dwell['max_s']}) is below "
                            f"dwell.min_s ({dwell['min_s']})")

        if mode == "fixed" and dwell["fixed_s"] <= 0:
            problems.append("dwell.fixed_s must be > 0")
        if mode == "settle":
            if dwell["settle_rate_n_per_s"] <= 0:
                problems.append("dwell.settle_rate_n_per_s must be > 0")
            if dwell["settle_window_s"] <= 0:
                problems.append("dwell.settle_window_s must be > 0")
            if dwell["hold_after_settle_s"] <= 0:
                problems.append("dwell.hold_after_settle_s must be > 0")
        if mode == "sem_target" and dwell["target_thrust_sem_n"] <= 0:
            problems.append("dwell.target_thrust_sem_n must be > 0")

        # Adaptive dwell reads the force sensor to decide when to move on, so a
        # sensor fault would stall the run at max_s per step rather than stopping.
        # Worth flagging that max_s is doing real work in those modes.
        if mode != "fixed" and dwell["max_s"] > 60.0:
            problems.append("dwell.max_s above 60s: in an adaptive mode this is the "
                            "only bound on a step that never settles")
        return problems

    @staticmethod
    def _check_fc(fc):
        problems = []
        if fc["resend_hz"] <= 0:
            problems.append("hardware.fc.resend_hz must be > 0")
        elif fc["timeout_s"] <= 1.0 / fc["resend_hz"]:
            problems.append(
                f"hardware.fc.timeout_s ({fc['timeout_s']}) must exceed the resend "
                f"interval ({1.0 / fc['resend_hz']:.3f}s) or the motors will stutter")
        for name in ("servo_hz", "battery_hz", "esc_hz"):
            if fc[name] < 0:
                problems.append(f"hardware.fc.{name} must be >= 0")
        return problems

    def as_dict(self):
        """The fully resolved plan, for the manifest."""
        return copy.deepcopy(self.data)


def load(path):
    """Load and validate a plan file."""
    return Plan(load_raw(path), source=path)
