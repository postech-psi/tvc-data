"""
Simulated bench, for exercising the whole stack with nothing plugged in.

**This is not a model of the bench and its output is not data.** It exists so
the runner, recorder, supervisor and dwell logic can be run end to end in tests
and rehearsed before a session. Every constant below is a plausible stand-in,
not a fitted parameter.

Two pieces are taken from the real measurements, because getting them roughly
right is what makes the exercise meaningful:

* **Power follows momentum theory.** `P = k * T^1.5` fits this bench closely --
  `out/pwm_thrust_torque_map.csv` gives 228 W at 12.37 N and 25.5 W at 3.01 N,
  implying k = 5.24 and 4.90. So the simulated pack sags about as much as the
  real one, and the low-voltage abort path gets a realistic exercise.
* **Noise is autocorrelated, not white.** Propeller vibration on this bench has
  a lag-1 autocorrelation near 0.5, which is why the analysis divides by an
  effective sample size. A simulator with white noise would make the SEM-target
  dwell logic look far better than it will be.
"""

import math
import random
import time

from tvcbench.actuator import PWM_MAX_US, PWM_MIN_US, us_to_norm
from tvcbench.clock import now
from tvcbench.sources.base import Sample, ThreadedSource
from tvcbench.sources.loadcell import FT_CHANNELS, LoadCellSource
from tvcbench.sources.mavlink import MavlinkSource

# Per-rotor static thrust at full command, newtons. Chosen so a mid-grid point
# lands in the range the real map covers.
THRUST_FULL_N = 18.75
# Coaxial derating: the lower rotor works in the upper's wake, so the pair makes
# less than the sum. Order-of-magnitude only.
COAX_FACTOR = 0.62
# Reaction torque scale. Tz is the *difference* of the rotors' drag torques, so
# it crosses zero near A = B, as the real bench does.
TORQUE_FULL_NM = 0.30

# Motor and propeller spin-up. The real command-to-thrust lag measures +0.33 to
# +0.54 s, which a first-order lag of this size reproduces well enough.
THRUST_TAU_S = 0.25

POWER_K = 5.1              # P = POWER_K * T^1.5, from the measured map
PACK_NOMINAL_V = 12.6      # 3S, charged
PACK_CAPACITY_MAH = 2200.0
PACK_INTERNAL_R = 0.045    # ohms; produces a few volts of sag at 40 A
PACK_SAG_TO_EMPTY_V = 1.8  # open-circuit fall from full to empty

THRUST_NOISE_N = 0.7       # within-step sigma when spinning, from the real bench
NOISE_AC = 0.5             # lag-1 autocorrelation
SENSOR_FLOOR_N = 0.012     # motors stopped

STAND_DEAD_LOAD_N = 9.3    # what the stand reads at rest, before taring


class SimBench:
    """Shared physical state. The sim sources read it; the sim link writes it."""

    def __init__(self, seed=0, enabled=True):
        self.rng = random.Random(seed)
        self.enabled = enabled
        self.a_us = PWM_MIN_US
        self.b_us = PWM_MIN_US

        self.thrust_n = 0.0            # settled value the lag is heading towards
        self._thrust_state = 0.0
        self._noise = 0.0
        self.torque_nm = 0.0
        self.charge_used_mah = 0.0
        self.voltage_v = PACK_NOMINAL_V
        self.current_a = 0.0
        self._t_last = None
        self.armed = False

    def set_command(self, a_us, b_us):
        self.a_us = int(a_us)
        self.b_us = int(b_us)

    def _static_thrust(self):
        na, nb = us_to_norm(self.a_us), us_to_norm(self.b_us)
        both = 1.0 if (na > 0 and nb > 0) else 0.0
        pair = THRUST_FULL_N * (na ** 2 + nb ** 2)
        return pair * (COAX_FACTOR if both else 1.0)

    def _static_torque(self):
        na, nb = us_to_norm(self.a_us), us_to_norm(self.b_us)
        return TORQUE_FULL_NM * (nb ** 2 - na ** 2)

    def advance(self, t_mono=None):
        """Integrate to `t_mono`. Idempotent enough to be called from either source."""
        t_mono = now() if t_mono is None else t_mono
        if self._t_last is None:
            self._t_last = t_mono
            return
        dt = t_mono - self._t_last
        if dt <= 0:
            return
        self._t_last = t_mono

        target = self._static_thrust() if self.enabled else 0.0
        alpha = 1.0 - math.exp(-dt / THRUST_TAU_S)
        self._thrust_state += (target - self._thrust_state) * alpha
        self.torque_nm = self._static_torque() if self.enabled else 0.0

        # AR(1) noise, scaled with thrust: a stopped stand is quiet.
        spinning = min(1.0, self._thrust_state / 5.0)
        sigma = SENSOR_FLOOR_N + THRUST_NOISE_N * spinning
        step = self.rng.gauss(0.0, sigma * math.sqrt(1.0 - NOISE_AC ** 2))
        self._noise = NOISE_AC * self._noise + step
        self.thrust_n = max(0.0, self._thrust_state) + self._noise

        self._update_pack(max(0.0, self._thrust_state), dt)

    def _update_pack(self, thrust, dt):
        power = POWER_K * thrust ** 1.5 if thrust > 0 else 0.0
        depth = min(1.0, self.charge_used_mah / PACK_CAPACITY_MAH)
        open_circuit = PACK_NOMINAL_V - PACK_SAG_TO_EMPTY_V * depth

        # P = V*I with V = Voc - I*R. Two fixed-point passes are ample here.
        current = power / open_circuit if open_circuit > 0 else 0.0
        for _ in range(2):
            voltage = max(0.1, open_circuit - current * PACK_INTERNAL_R)
            current = power / voltage if voltage > 0 else 0.0

        self.current_a = current
        self.voltage_v = max(0.1, open_circuit - current * PACK_INTERNAL_R)
        self.charge_used_mah += current * dt / 3.6

    def noload_voltage(self):
        depth = min(1.0, self.charge_used_mah / PACK_CAPACITY_MAH)
        return PACK_NOMINAL_V - PACK_SAG_TO_EMPTY_V * depth


class SimLink:
    """Duck-types `MavlinkLink` so the actuator drives the simulation unchanged."""

    def __init__(self, bench):
        self.bench = bench
        self.device = "sim"
        self.baud = 0
        self.conn = self
        self.pending = {}
        self.n_commands = 0

    def connect(self, timeout=30.0):
        return object()

    def send_actuator_test(self, func, value, timeout_s):
        self.n_commands += 1
        us = PWM_MIN_US + value * (PWM_MAX_US - PWM_MIN_US)
        self.pending[func] = us
        self.bench.set_command(self.pending.get(1, PWM_MIN_US),
                               self.pending.get(2, PWM_MIN_US))

    def set_message_interval(self, msg_id, hz):
        pass

    def request_rates(self, **_kw):
        pass

    def recv_match(self, **_kw):
        return None

    def close(self):
        pass


class _Paced(ThreadedSource):
    """Emits at a fixed rate in real time, so run timing behaves as it will live."""

    def __init__(self, name, bench, hz):
        super().__init__(name)
        self.bench = bench
        self.hz = float(hz)
        self._next = None

    def _open(self):
        self._next = now()

    def _read_once(self):
        t = now()
        if self._next is None:
            self._next = t
        wait = self._next - t
        if wait > 0:
            time.sleep(min(wait, 0.05))
            return
        self._next += 1.0 / self.hz
        self.bench.advance(t)
        self._produce(t)

    def _produce(self, t_mono):
        raise NotImplementedError


class SimLoadCellSource(_Paced):
    """Stands in for the STM32 over USB CDC."""

    streams = LoadCellSource.streams

    def __init__(self, bench, hz=50.0, name="loadcell"):
        super().__init__(name, bench, hz)
        self.tare = {ch: 0.0 for ch in FT_CHANNELS}
        self._t_ms = 100000
        self._fc = 0

    def set_tare(self, offsets):
        self.tare = {ch: float(offsets.get(ch, 0.0)) for ch in FT_CHANNELS}

    def clear_tare(self):
        self.tare = {ch: 0.0 for ch in FT_CHANNELS}

    compute_tare = staticmethod(LoadCellSource.compute_tare)

    def _produce(self, t_mono):
        b = self.bench
        # The stand reads vertical load negative and carries a dead load until
        # tared, exactly as the real one does -- so the tare path gets exercised.
        raw = {
            "Fx": b.rng.gauss(0.0, SENSOR_FLOOR_N),
            "Fy": b.rng.gauss(0.0, SENSOR_FLOOR_N),
            "Fz": -(b.thrust_n + STAND_DEAD_LOAD_N),
            "Tx": b.rng.gauss(0.0, SENSOR_FLOOR_N * 0.03),
            "Ty": b.rng.gauss(0.0, SENSOR_FLOOR_N * 0.03),
            "Tz": b.torque_nm,
        }
        fields = {"t_stm_ms": self._t_ms, "force_count": self._fc,
                  "torque_count": self._fc}
        for ch in FT_CHANNELS:
            fields[f"{ch}_raw"] = raw[ch]
            fields[ch] = raw[ch] - self.tare[ch]

        self._emit(Sample(self.name, t_mono, self._t_ms, fields))
        self._t_ms += int(round(1000.0 / self.hz))
        self._fc += 1

    def stats(self):
        base = super().stats()
        base.update({"dropped": 0, "parse_fail": 0, "overlong_lines": 0,
                     "mcu_state": "SAFE", "tare": dict(self.tare)})
        return base


class SimMavlinkSource(_Paced):
    """
    Stands in for the Pixhawk.

    Servo output is produced at the rate PX4 actually manages on this bench
    rather than the rate the plan requests, so a run rehearsed in simulation
    shows the same rate shortfall a real one will.
    """

    streams = MavlinkSource.streams
    ACHIEVED_SERVO_HZ = 19.0

    def __init__(self, bench, servo_hz=ACHIEVED_SERVO_HZ, battery_hz=10.0,
                 name="mavlink"):
        super().__init__(name, bench, servo_hz)
        self.battery_hz = float(battery_hz)
        self.armed = False
        self.last_heartbeat_t_mono = None
        self.n_ack_accepted = 0
        self.n_ack_rejected = 0
        self.last_voltage_v = None
        self.last_current_a = None
        self._t_fc_us = 0
        self._next_battery = None
        self._counts = {"fc_servo": 0, "fc_battery": 0, "fc_esc": 0}
        self._t_first = None

    def _produce(self, t_mono):
        b = self.bench
        if self._t_first is None:
            self._t_first = t_mono
        self.last_heartbeat_t_mono = t_mono
        self._t_fc_us += int(1e6 / self.hz)

        servo = {"t_fc_us": self._t_fc_us}
        for i in range(1, 9):
            servo[f"servo{i}_raw"] = (b.a_us if i == 1 else
                                      b.b_us if i == 2 else
                                      PWM_MIN_US if i <= 4 else 0)
        self._counts["fc_servo"] += 1
        self._emit(Sample("fc_servo", t_mono, self._t_fc_us, servo))

        if self._next_battery is None or t_mono >= self._next_battery:
            self._next_battery = (t_mono if self._next_battery is None
                                  else self._next_battery) + 1.0 / self.battery_hz
            self.last_voltage_v = b.voltage_v
            self.last_current_a = b.current_a
            self._counts["fc_battery"] += 1
            self._emit(Sample("fc_battery", t_mono, None, {
                "voltage_v": round(b.voltage_v, 3),
                "current_a": round(b.current_a, 3),
                "remaining_pct": max(0, int(100 - 100 * b.charge_used_mah
                                            / PACK_CAPACITY_MAH)),
            }))

    def achieved_rates(self):
        if self._t_first is None:
            return {k: 0.0 for k in self._counts}
        span = max(1e-6, (self._last_t_mono or self._t_first) - self._t_first)
        return {k: round(n / span, 1) for k, n in self._counts.items()}

    def stats(self):
        base = super().stats()
        base.update({
            "armed": self.armed,
            "last_heartbeat_t_mono": self.last_heartbeat_t_mono,
            "ack_accepted": self.n_ack_accepted, "ack_rejected": self.n_ack_rejected,
            "counts": dict(self._counts), "achieved_rates": self.achieved_rates(),
            "last_voltage_v": self.last_voltage_v,
            "last_current_a": self.last_current_a,
        })
        return base
