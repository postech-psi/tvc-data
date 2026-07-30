"""
Turning a plan into the ordered list of segments a run actually executes.

Two decisions here matter more than the mechanics.

**Segments are identified by an integer, not by a string.** The old logger wrote
`phase = "A1400_B1700"` into every row and the analysis parsed it back out. That
made two visits to the same command in one run indistinguishable -- which is
fatal the moment reference revisits or repeats exist -- and it put a regex on the
critical path. Here `seg_id` is the key, `sequence.csv` holds the commands, and
nothing downstream parses anything.

**Repeats are randomised complete blocks.** The battery drains monotonically
through a run, so a grid walked in order confounds voltage with commanded PWM
perfectly: every high-B point is also a low-voltage point. Shuffling *within*
each repeat, and reshuffling for the next, gives every command level the same
expected position in the drain -- the standard randomised-complete-block answer.
A single global shuffle (`order.mode: random`) decorrelates too, but repeats then
share one ordering and cannot separate drift from position. Adjacent jumps are
left unconstrained on purpose: settling is what dwell is for, and the periodic
reference points measure the drift directly rather than assuming it away.
"""

import random

TARE = "tare"
WARMUP = "warmup"
IDLE_PRE = "idle_pre"
IDLE_POST = "idle_post"
STEP = "step"
REFERENCE = "reference"
RAMP_DOWN = "ramp_down"
RAMP_BETWEEN = "ramp_between"
CHIRP = "chirp"
POST_STOP = "post_stop"

PWM_MIN_US = 1000

#: Kinds whose samples are written to disk. Warmup is the only one skipped: the
#: ESCs are arming and the numbers mean nothing.
RECORDED_KINDS = (TARE, IDLE_PRE, IDLE_POST, STEP, REFERENCE,
                  RAMP_DOWN, RAMP_BETWEEN, CHIRP, POST_STOP)

#: Kinds a fixed dwell applies to. Everything else has a duration set by the plan.
ADAPTIVE_KINDS = (STEP, REFERENCE)


class Segment:
    """One held command with a planned duration."""

    __slots__ = ("seg_id", "kind", "a_us", "b_us", "dwell_s",
                 "min_dwell_s", "max_dwell_s", "sweep", "record")

    def __init__(self, seg_id, kind, a_us, b_us, dwell_s,
                 min_dwell_s=None, max_dwell_s=None, sweep=0):
        self.seg_id = seg_id
        self.kind = kind
        self.a_us = int(round(a_us))
        self.b_us = int(round(b_us))
        self.dwell_s = float(dwell_s)
        self.min_dwell_s = float(dwell_s if min_dwell_s is None else min_dwell_s)
        self.max_dwell_s = float(dwell_s if max_dwell_s is None else max_dwell_s)
        self.sweep = sweep
        self.record = kind in RECORDED_KINDS

    @property
    def adaptive(self):
        """True when the runner decides this segment's length from the force signal."""
        return self.max_dwell_s > self.min_dwell_s

    def as_dict(self):
        return {
            "seg_id": self.seg_id, "kind": self.kind,
            "a_us": self.a_us, "b_us": self.b_us,
            "planned_dwell_s": self.dwell_s,
            "min_dwell_s": self.min_dwell_s, "max_dwell_s": self.max_dwell_s,
            "sweep": self.sweep, "record": self.record,
        }

    def __repr__(self):
        return (f"Segment({self.seg_id}, {self.kind}, A={self.a_us}, "
                f"B={self.b_us}, {self.dwell_s:.2f}s)")


def resolve_seed(plan):
    """The plan's seed, or a fresh one. Either way it is returned to be recorded."""
    seed = plan["order"].get("seed") or 0
    if seed:
        return int(seed)
    return random.randrange(1, 2 ** 31 - 1)


def grid_points(plan):
    """(A, B) pairs in generation order: A outer, B inner."""
    return [(a, b) for a in plan.a_values for b in plan.b_values]


def order_points(plan, seed):
    """
    One ordered point list per repeat.

    Returns a list of `repeats` lists, so `blocked_random` can give each repeat
    its own ordering while the other modes reuse one.
    """
    points = grid_points(plan)
    mode = plan["order"]["mode"]
    repeats = plan["repeats"]

    if mode == "sequential":
        return [list(points) for _ in range(repeats)]

    if mode == "random":
        rng = random.Random(seed)
        shuffled = list(points)
        rng.shuffle(shuffled)
        return [list(shuffled) for _ in range(repeats)]

    # blocked_random: each repeat is a complete replicate in its own order.
    out = []
    for r in range(repeats):
        # A distinct stream per repeat, derived from the one recorded seed so the
        # whole ordering is reproducible from the manifest alone.
        rng = random.Random(f"{seed}:{r}")
        block = list(points)
        rng.shuffle(block)
        out.append(block)
    return out


class _Builder:
    """Assigns sequential ids as segments are appended."""

    def __init__(self):
        self.segments = []

    def add(self, kind, a_us, b_us, dwell_s, min_dwell_s=None, max_dwell_s=None,
            sweep=0):
        seg = Segment(len(self.segments), kind, a_us, b_us, dwell_s,
                      min_dwell_s, max_dwell_s, sweep)
        self.segments.append(seg)
        return seg


def _dwell_bounds(plan):
    """(nominal, min, max) seconds for a measurement step under the plan's dwell mode."""
    d = plan["dwell"]
    mode = d["mode"]
    if mode == "fixed":
        return d["fixed_s"], d["fixed_s"], d["fixed_s"]
    if mode == "settle":
        # Nominal assumes the step settles about a second in -- an estimate for
        # planning only. The runner uses the measured signal.
        nominal = min(d["max_s"], max(d["min_s"], 1.0 + d["hold_after_settle_s"]))
        return nominal, d["min_s"], d["max_s"]
    # sem_target: no way to predict, so the midpoint is the honest planning number.
    return 0.5 * (d["min_s"] + d["max_s"]), d["min_s"], d["max_s"]


def _add_ramp(builder, from_a, from_b, to_a, to_b, plan, kind, sweep):
    from tvcbench.actuator import ramp_profile

    steps = plan["ramp"]["steps"]
    per_step = plan["ramp"]["seconds"] / steps
    for a, b in ramp_profile(from_a, from_b, to_a, to_b, steps):
        builder.add(kind, a, b, per_step, sweep=sweep)


def _add_chirp(builder, plan):
    """
    A square wave between minimum and `high_us`, both rotors together.

    Its only job is to be unmistakable. During the transition off the .ulg this
    is the feature that lets the Pi record and the FC log be checked against each
    other; once the ulog is gone it costs a few seconds and can be disabled.
    """
    c = plan["chirp"]
    if not c.get("enabled"):
        return
    for _ in range(c["cycles"]):
        builder.add(CHIRP, c["high_us"], c["high_us"], c["half_period_s"])
        builder.add(CHIRP, PWM_MIN_US, PWM_MIN_US, c["half_period_s"])


def expand(plan, seed=None):
    """
    Build the full segment list for a plan.

    Returns `(segments, seed)`. The seed is returned resolved so the manifest can
    record what was actually used even when the plan asked for a fresh one.
    """
    seed = resolve_seed(plan) if seed is None else int(seed)
    b = _Builder()
    nominal, min_s, max_s = _dwell_bounds(plan)
    ref = plan["reference"]
    every = ref.get("every_n_steps", 0)

    if plan["tare"]["seconds"] > 0:
        b.add(TARE, PWM_MIN_US, PWM_MIN_US, plan["tare"]["seconds"])
    if plan["warmup"]["seconds"] > 0:
        b.add(WARMUP, PWM_MIN_US, PWM_MIN_US, plan["warmup"]["seconds"])
    if plan["idle"]["pre_s"] > 0:
        b.add(IDLE_PRE, PWM_MIN_US, PWM_MIN_US, plan["idle"]["pre_s"])

    _add_chirp(b, plan)

    last_a, last_b = PWM_MIN_US, PWM_MIN_US
    orders = order_points(plan, seed)
    for sweep, points in enumerate(orders, start=1):
        for i, (a_us, b_us) in enumerate(points, start=1):
            b.add(STEP, a_us, b_us, nominal, min_s, max_s, sweep=sweep)
            last_a, last_b = a_us, b_us
            if every and i % every == 0 and i != len(points):
                b.add(REFERENCE, ref["a"], ref["b"], nominal, min_s, max_s, sweep=sweep)
                last_a, last_b = ref["a"], ref["b"]

        if sweep < len(orders):
            first_a, first_b = orders[sweep][0]
            _add_ramp(b, last_a, last_b, first_a, first_b, plan, RAMP_BETWEEN, sweep)
            last_a, last_b = first_a, first_b

    # The end-of-run ramp is recorded: it moves thrust by roughly 9 N against
    # 0.7 N of noise, which makes it the clearest single feature in the record.
    _add_ramp(b, last_a, last_b, PWM_MIN_US, PWM_MIN_US, plan, RAMP_DOWN,
              len(orders))
    _add_chirp(b, plan)

    if plan["idle"]["post_s"] > 0:
        b.add(IDLE_POST, PWM_MIN_US, PWM_MIN_US, plan["idle"]["post_s"])

    # The last segment is the one the run loop does *not* execute: it runs in the
    # shutdown path, after the motors have been stopped and with nothing
    # commanding the outputs. It is generated here anyway so that `plan show`
    # accounts for its duration and its seg_id follows on from the rest --
    # `runner` splits it back out. Its command is minimum because that is where a
    # lapsed actuator-test command leaves the outputs, not because it is sent.
    if plan["post_stop"]["seconds"] > 0:
        b.add(POST_STOP, PWM_MIN_US, PWM_MIN_US, plan["post_stop"]["seconds"])

    return b.segments, seed


def duration_s(segments):
    """(min, nominal, max) total seconds. They differ only under adaptive dwell."""
    return (sum(s.min_dwell_s for s in segments),
            sum(s.dwell_s for s in segments),
            sum(s.max_dwell_s for s in segments))


def summarise(segments):
    """Counts per kind, for `plan show`."""
    counts = {}
    for s in segments:
        counts[s.kind] = counts.get(s.kind, 0) + 1
    return counts


# --- battery budgeting ------------------------------------------------------

def current_map_from_csv(path):
    """
    Build a current lookup from a previously measured map.

    Uses `out/pwm_thrust_torque_map.csv`, which already carries measured current
    at each (A, B). Estimating a plan's battery draw from real measurements of
    this bench beats any model, and when no map exists the estimate is simply
    reported as unavailable rather than invented.
    """
    import csv

    table = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                key = (int(float(row["a_cmd_us"])), int(float(row["b_cmd_us"])))
                current = float(row["current_a"])
            except (KeyError, TypeError, ValueError):
                continue
            table.setdefault(key, []).append(current)
    if not table:
        return None
    return {k: sum(v) / len(v) for k, v in table.items()}


def lookup_current(table, a_us, b_us):
    """Nearest measured point, by squared distance in command space."""
    if not table:
        return None
    exact = table.get((a_us, b_us))
    if exact is not None:
        return exact
    return min(table.items(),
               key=lambda kv: (kv[0][0] - a_us) ** 2 + (kv[0][1] - b_us) ** 2)[1]


def estimate_charge_mah(segments, table, use="dwell_s"):
    """
    Charge the plan will draw, in mAh, or None without a current map.

    Integrates the nearest measured current over each segment's planned duration.
    Ramps and chirps are included at their endpoint command, which slightly
    overstates them -- the right direction to be wrong in when the question is
    "will this fit in the pack".
    """
    if not table:
        return None
    total_as = 0.0
    for seg in segments:
        current = lookup_current(table, seg.a_us, seg.b_us)
        if current is None:
            continue
        total_as += current * getattr(seg, use)
    return total_as / 3.6      # amp-seconds -> mAh
