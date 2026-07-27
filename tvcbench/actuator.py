"""
Driving the rotors through PX4's actuator-test path.

Ported from `pwm_thrust_map.py`, which has been driving this bench for months.
The mechanism is deliberately unchanged -- it is correct, and its safety
behaviour is subtle enough to be worth restating rather than rediscovering:

* `MAV_CMD_ACTUATOR_TEST` (310) drives an output *function* directly. It is the
  command QGroundControl's actuator sliders use, and PX4 refuses it while armed,
  so the bench runs disarmed.
* Values are **normalised 0..1, not microseconds**. The mapping is linear only
  if QGC has `Minimum=1000`, `Maximum=2000`, `Disarmed=1000` on the ESC channels
  and `THR_MDL_FAC = 0`.
* Every command **expires** after `timeout_s`. Holding a value means resending
  it; and therefore *ceasing to resend is itself the stop*. That is the base
  safety layer under everything else in this package -- a crashed process, a
  severed cable and a killed thread all stop the motors by doing nothing.

The one addition is `enabled=False`, which runs the entire sequence, timing and
recording path with the commands suppressed. That makes a full dress rehearsal
of any plan possible with the battery disconnected.
"""

from tvcbench.clock import now

# Must match the QGC channel configuration, or the microsecond values recorded
# here will not be the microseconds the ESC actually saw.
PWM_MIN_US = 1000
PWM_MAX_US = 2000

# MAV_ACTUATOR_OUTPUT_FUNCTION: Motor1 = 1, Motor2 = 2. A is the fixed (outer)
# rotor, B the swept (inner) one, matching the run naming and the map columns.
MOTOR_A_FUNC = 1
MOTOR_B_FUNC = 2

# Protocol constants, spelled out so this module imports without pymavlink and
# stays unit-testable off the Pi. Resolved from the library when it is present.
ACTUATOR_TEST = 310
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

# v = 0 maps to PWM_MIN_US, which is throttle zero on a calibrated ESC.
STOP_VALUE = 0.0
# Repeated because a single frame can be lost on a serial link, and this is the
# path taken on every abort. Cheap insurance on the one command that must land.
STOP_REPEATS = 5
STOP_GAP_S = 0.05

DEFAULT_TIMEOUT_S = 1.0
DEFAULT_RESEND_HZ = 5.0


def us_to_norm(us):
    """Target microseconds -> normalised 0..1. Clamped, so a bad plan cannot overdrive."""
    norm = (us - PWM_MIN_US) / float(PWM_MAX_US - PWM_MIN_US)
    return max(0.0, min(1.0, norm))


def norm_to_us(norm):
    """Inverse of `us_to_norm`, for reporting what a normalised value meant."""
    return PWM_MIN_US + float(norm) * (PWM_MAX_US - PWM_MIN_US)


def validate_resend(timeout_s, resend_hz):
    """
    A command must be resent before it expires, or the motors stutter.

    Checked here rather than trusted from config: the failure is a partial
    dropout mid-step, which looks like noise in the map rather than an error.
    """
    if resend_hz <= 0:
        raise ValueError("resend_hz must be > 0")
    if timeout_s <= 1.0 / resend_hz:
        raise ValueError(
            f"timeout_s ({timeout_s}) must exceed the resend interval "
            f"({1.0 / resend_hz:.3f}s) or commands will lapse between sends")


def read_battery(msg):
    """
    Voltage and current from BATTERY_STATUS, by QGroundControl's exact algorithm.

    Matches `BatteryFactGroupListModel.cc`: sum `voltages[]` until the *first*
    UINT16_MAX and stop, then `voltages_ext[]` until the first 0 and stop. PX4
    fills valid cells contiguously from index 0, so stopping at the first invalid
    entry -- rather than skipping it -- is the correct reconstruction, and makes
    the recorded number identical to what the bench operator sees on screen.

    Returns `(voltage_v | None, current_a | None)`.
    """
    total_mv = None
    for v in msg.voltages:                          # cells 1-10, mV
        if v == 65535:                              # UINT16_MAX: end of valid cells
            break
        total_mv = v if total_mv is None else total_mv + v
    for v in getattr(msg, "voltages_ext", []):      # cells 11-14, absent on old pymavlink
        if v == 0:                                  # 0: unsupported, end of valid cells
            break
        total_mv = v if total_mv is None else total_mv + v

    voltage_v = total_mv / 1000.0 if total_mv is not None else None
    current_a = msg.current_battery / 100.0 if msg.current_battery != -1 else None
    return voltage_v, current_a


class Actuator:
    """
    Holds a commanded (A, B) pair alive by resending it, and stops on demand.

    The runner decides *what* to command and for how long; this decides *when to
    resend*. Splitting them keeps the expiry rule in one place -- the runner
    cannot forget to tick, because not ticking stops the motors.
    """

    def __init__(self, link, timeout_s=DEFAULT_TIMEOUT_S, resend_hz=DEFAULT_RESEND_HZ,
                 enabled=True, on_send=None):
        validate_resend(timeout_s, resend_hz)
        self.link = link
        self.timeout_s = float(timeout_s)
        self.resend_hz = float(resend_hz)
        self.enabled = bool(enabled)
        self.on_send = on_send            # called with (t_mono, a_us, b_us) per send

        self.a_us = PWM_MIN_US
        self.b_us = PWM_MIN_US
        self._next_send = 0.0
        self.n_sends = 0

    @property
    def interval_s(self):
        return 1.0 / self.resend_hz

    def set_target(self, a_us, b_us):
        """Aim at a new (A, B) in microseconds; sent on the next `tick`."""
        self.a_us = int(round(a_us))
        self.b_us = int(round(b_us))
        self._next_send = 0.0             # send immediately, do not wait out the interval

    def tick(self, t_mono=None):
        """Resend the current target if due. Returns True if a command went out."""
        t_mono = now() if t_mono is None else t_mono
        if t_mono < self._next_send:
            return False
        self._send(self.a_us, self.b_us)
        self._next_send = t_mono + self.interval_s
        if self.on_send is not None:
            self.on_send(t_mono, self.a_us, self.b_us)
        return True

    def _send(self, a_us, b_us):
        self.n_sends += 1
        if not self.enabled:
            return                         # --no-motor: everything else still runs
        self.link.send_actuator_test(MOTOR_A_FUNC, us_to_norm(a_us), self.timeout_s)
        self.link.send_actuator_test(MOTOR_B_FUNC, us_to_norm(b_us), self.timeout_s)

    def stop(self, sleep=None):
        """
        Cut both rotors to minimum immediately. No ramp -- this is the abort path.

        Used for the stop button, every supervisor limit and any error. The clean
        end-of-run ramp is the runner's business; when something is wrong, the
        priority is that the motors are off.
        """
        import time

        sleep = time.sleep if sleep is None else sleep
        self.a_us = PWM_MIN_US
        self.b_us = PWM_MIN_US
        for i in range(STOP_REPEATS):
            if self.enabled:
                self.link.send_actuator_test(MOTOR_A_FUNC, STOP_VALUE, self.timeout_s)
                self.link.send_actuator_test(MOTOR_B_FUNC, STOP_VALUE, self.timeout_s)
            self.n_sends += 1
            if i < STOP_REPEATS - 1:
                sleep(STOP_GAP_S)

    def preflight(self):
        """
        Send one stop-value command and see whether PX4 accepts it.

        `v = 0` is minimum throttle, so a calibrated ESC does not spin. A
        rejection here is almost always the vehicle being armed, which is worth
        saying in those words -- it is the single most common bench mistake.

        Returns `(ok, message)`.
        """
        if not self.enabled:
            return True, "motors disabled (--no-motor): preflight skipped"

        self.link.send_actuator_test(MOTOR_A_FUNC, STOP_VALUE, self.timeout_s)
        ack = self.link.recv_match(type="COMMAND_ACK", blocking=True, timeout=2.0)
        if ack is None or getattr(ack, "command", None) != ACTUATOR_TEST:
            # Not fatal: some setups simply do not ack this command.
            return True, "no COMMAND_ACK for ACTUATOR_TEST (continuing)"
        if ack.result == MAV_RESULT_ACCEPTED:
            return True, "preflight OK: ACTUATOR_TEST accepted"
        if ack.result == MAV_RESULT_TEMPORARILY_REJECTED:
            return False, ("rejected (TEMPORARILY_REJECTED): the vehicle is almost "
                           "certainly armed -- disarm and retry")
        return False, (f"rejected (result={ack.result}): check the Motor1/Motor2 "
                       f"assignment in QGC")


def ramp_profile(from_a, from_b, to_a, to_b, steps):
    """
    Linear (A, B) waypoints for a smooth transition, excluding the start point.

    Used both for the end-of-run ramp down and for moving between sweeps. These
    segments are *recorded*, unlike in the original script's early versions: the
    final ramp moves thrust by roughly 9 N against 0.7 N of noise, which makes it
    the highest-signal feature in the whole record and the natural cross-check
    against the ulog.
    """
    if steps < 1:
        raise ValueError("steps must be >= 1")
    out = []
    for i in range(1, steps + 1):
        frac = i / float(steps)
        out.append((from_a + (to_a - from_a) * frac,
                    from_b + (to_b - from_b) * frac))
    return out
