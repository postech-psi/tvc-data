"""
Pixhawk telemetry, over MAVLink.

Emits three streams at three different rates, kept separate on purpose. The old
logger wrote one row per `SERVO_OUTPUT_RAW` and stamped it with the last voltage
it happened to have seen, which manufactured a 20 Hz record of a 10 Hz quantity
and threw away when the voltage actually changed. Here each message type is its
own file at its own rate, and analysis decides how to combine them.

Requested rates are not achieved rates. PX4 silently caps `SET_MESSAGE_INTERVAL`
against its internal publish rate and the `MAV_*_RATE` budget -- 50 Hz asked of
`SERVO_OUTPUT_RAW` has historically delivered about 18-20 Hz on this bench. That
is fine for what the stream is (a verification echo of a command already known),
but it must be *measured* rather than assumed, so the achieved rate goes in the
manifest.
"""

from tvcbench.actuator import read_battery
from tvcbench.clock import now
from tvcbench.sources.base import Sample, ThreadedSource

# Message IDs, spelled out so this module imports without pymavlink.
MSG_SERVO_OUTPUT_RAW = 36
MSG_BATTERY_STATUS = 147
MSG_ESC_STATUS = 291
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_ACTUATOR_TEST = 310

# The Pi identifies itself as this system/component. Distinct from the FC and from
# QGC, so both can be connected at once without confusing PX4's routing.
SOURCE_SYSTEM = 191
SOURCE_COMPONENT = 191

DEFAULT_DEVICE = "/dev/ttyAMA0"
DEFAULT_BAUD = 921600

# Requested rates. Bandwidth is not the constraint: at 921600 baud these three
# together come to roughly 5 % of the link. PX4's publish rate is the constraint.
DEFAULT_SERVO_HZ = 50
DEFAULT_BATTERY_HZ = 20
DEFAULT_ESC_HZ = 20

# Cap on messages drained per pass so the loop returns often enough to notice a
# stop request, even when telemetry arrives faster than it can be handled.
MAX_DRAIN_PER_LOOP = 40
RECV_TIMEOUT_S = 0.05

N_ESC_SLOTS = 4


class MavlinkLink:
    """
    Owns the MAVLink connection and serialises access to it.

    The reader thread and the actuator both touch this object, so every send is
    taken under a lock. pymavlink makes no threading guarantees, and a torn
    command frame on the one path that stops the motors is not a risk worth
    taking to save a mutex.
    """

    def __init__(self, device=DEFAULT_DEVICE, baud=DEFAULT_BAUD, connection_factory=None):
        import threading

        self.device = device
        self.baud = baud
        self._factory = connection_factory      # injectable for tests
        self._lock = threading.Lock()
        self.conn = None

    def connect(self, timeout=30.0):
        """Open the link and wait for a heartbeat. Returns the heartbeat message."""
        if self._factory is not None:
            self.conn = self._factory()
        else:
            from pymavlink import mavutil

            self.conn = mavutil.mavlink_connection(
                self.device, baud=self.baud,
                source_system=SOURCE_SYSTEM, source_component=SOURCE_COMPONENT)

        hb = self.conn.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise RuntimeError(
                f"no HEARTBEAT from {self.device} at {self.baud} -- check the TX/RX "
                f"crossover, power, and that the baud matches the FC's SER_*_BAUD")
        return hb

    def _command_long(self, command, *params):
        padded = (list(params) + [0.0] * 7)[:7]
        with self._lock:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                command, 0, *[float(p) for p in padded])

    def send_actuator_test(self, func, value, timeout_s):
        """param1 = normalised value, param2 = expiry seconds, param5 = output function."""
        self._command_long(MAV_CMD_ACTUATOR_TEST, value, timeout_s, 0.0, 0.0, func)

    def set_message_interval(self, msg_id, hz):
        if not hz or hz <= 0:
            return
        self._command_long(MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))

    def request_rates(self, servo_hz=DEFAULT_SERVO_HZ, battery_hz=DEFAULT_BATTERY_HZ,
                      esc_hz=DEFAULT_ESC_HZ):
        """
        Raise the publish rate of the three streams recorded here.

        ESC_STATUS needs a telemetry-capable ESC and PX4 configured for it. Asking
        when there is none costs nothing -- the message simply never arrives, and
        the achieved-rate check reports it as absent.
        """
        self.set_message_interval(MSG_SERVO_OUTPUT_RAW, servo_hz)
        self.set_message_interval(MSG_BATTERY_STATUS, battery_hz)
        self.set_message_interval(MSG_ESC_STATUS, esc_hz)

    def recv_match(self, **kwargs):
        return self.conn.recv_match(**kwargs)

    def close(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:                  # noqa: BLE001
                pass
            self.conn = None


class MavlinkSource(ThreadedSource):
    """Reads the FC and fans telemetry out into three independent streams."""

    SERVO_COLUMNS = ("t_fc_us",) + tuple(f"servo{i}_raw" for i in range(1, 9))
    BATTERY_COLUMNS = ("voltage_v", "current_a", "remaining_pct")
    ESC_COLUMNS = (("t_fc_us", "esc_index")
                   + tuple(f"esc{i}_rpm" for i in range(1, N_ESC_SLOTS + 1))
                   + tuple(f"esc{i}_voltage" for i in range(1, N_ESC_SLOTS + 1))
                   + tuple(f"esc{i}_current" for i in range(1, N_ESC_SLOTS + 1)))

    streams = {
        "fc_servo": SERVO_COLUMNS,
        "fc_battery": BATTERY_COLUMNS,
        "fc_esc": ESC_COLUMNS,
    }

    def __init__(self, link, name="mavlink"):
        super().__init__(name)
        self.link = link

        self.armed = None                  # None until a heartbeat says otherwise
        self.last_heartbeat_t_mono = None
        self.n_ack_accepted = 0
        self.n_ack_rejected = 0
        self.last_voltage_v = None
        self.last_current_a = None
        self._counts = {"fc_servo": 0, "fc_battery": 0, "fc_esc": 0}
        self._t_first = None

    def _open(self):
        if self.link.conn is None:
            raise RuntimeError("link is not connected; call MavlinkLink.connect() first")

    def _close(self):
        pass                                # the link outlives the source

    def _read_once(self):
        msg = self.link.recv_match(blocking=True, timeout=RECV_TIMEOUT_S)
        drained = 0
        # Drain what has already queued behind this message. Without it, a burst
        # leaves messages sitting in the serial buffer and their arrival stamps
        # drift later and later behind reality -- the exact failure the original
        # script's comment warns about.
        while msg is not None and drained < MAX_DRAIN_PER_LOOP:
            drained += 1
            self._handle(msg, now())
            msg = self.link.recv_match(blocking=False)

    def _handle(self, msg, t_mono):
        kind = msg.get_type()
        if kind == "SERVO_OUTPUT_RAW":
            self._emit_stream("fc_servo", t_mono, getattr(msg, "time_usec", None), {
                "t_fc_us": getattr(msg, "time_usec", None),
                **{f"servo{i}_raw": getattr(msg, f"servo{i}_raw", None)
                   for i in range(1, 9)},
            })

        elif kind == "BATTERY_STATUS":
            # id 0 is the main pack; QGC reads the same one.
            if getattr(msg, "id", 0) != 0:
                return
            voltage_v, current_a = read_battery(msg)
            if voltage_v is not None:
                self.last_voltage_v = voltage_v
            if current_a is not None:
                self.last_current_a = current_a
            remaining = getattr(msg, "battery_remaining", -1)
            # BATTERY_STATUS carries no timestamp, so there is no device clock to
            # fit -- arrival time is all there is, which is ample at 10-20 Hz.
            self._emit_stream("fc_battery", t_mono, None, {
                "voltage_v": voltage_v,
                "current_a": current_a,
                "remaining_pct": None if remaining < 0 else remaining,
            })

        elif kind == "ESC_STATUS":
            base = int(getattr(msg, "index", 0))
            fields = {"t_fc_us": getattr(msg, "time_usec", None), "esc_index": base}
            for key, attr in (("rpm", "rpm"), ("voltage", "voltage"),
                              ("current", "current")):
                values = list(getattr(msg, attr, []) or [])
                for slot in range(N_ESC_SLOTS):
                    fields[f"esc{slot + 1}_{key}"] = (
                        values[slot] if slot < len(values) else None)
            self._emit_stream("fc_esc", t_mono, getattr(msg, "time_usec", None), fields)

        elif kind == "HEARTBEAT":
            self.armed = bool(getattr(msg, "base_mode", 0) & 128)   # SAFETY_ARMED
            self.last_heartbeat_t_mono = t_mono

        elif kind == "COMMAND_ACK":
            if getattr(msg, "command", None) == MAV_CMD_ACTUATOR_TEST:
                if getattr(msg, "result", None) == 0:               # ACCEPTED
                    self.n_ack_accepted += 1
                else:
                    self.n_ack_rejected += 1

    def _emit_stream(self, stream, t_mono, dev_t, fields):
        if self._t_first is None:
            self._t_first = t_mono
        self._counts[stream] += 1
        self._emit(Sample(stream, t_mono, dev_t, fields))

    def achieved_rates(self):
        """
        Measured message rates, in Hz.

        Reported next to what was requested, because PX4 caps quietly: a run
        whose servo stream came in at 18 Hz against a requested 50 is not broken,
        but it is not what the plan said either.
        """
        if self._t_first is None:
            return {k: 0.0 for k in self._counts}
        span = max(1e-6, (self._last_t_mono or self._t_first) - self._t_first)
        return {k: round(n / span, 1) for k, n in self._counts.items()}

    def stats(self):
        base = super().stats()
        base.update({
            "armed": self.armed,
            "last_heartbeat_t_mono": self.last_heartbeat_t_mono,
            "ack_accepted": self.n_ack_accepted,
            "ack_rejected": self.n_ack_rejected,
            "counts": dict(self._counts),
            "achieved_rates": self.achieved_rates(),
            "last_voltage_v": self.last_voltage_v,
            "last_current_a": self.last_current_a,
        })
        return base
