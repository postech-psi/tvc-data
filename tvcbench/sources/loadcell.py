"""
STM32 six-axis load cell, over USB CDC.

Historically this board hung off the bench laptop and `gui.py` logged it, which
is why every force sample had to be time-aligned to the Pi's commands after the
fact. Moving it onto the Pi is the reason this package exists: the same process
that commands the rotors now stamps the force samples, so there is nothing left
to align.

Wire format is the MCU's existing status line -- whitespace-separated `key=value`
pairs, one line per sample at ~50 Hz:

    st=SAFE t=3009477 pwm=1000 rpm=1724.0 Fx=-838.0 ... Tz=-4.01 fc=91823 tc=91823

Forces arrive in mN and torques in mN*m; both are scaled to SI here, once, at
the edge. `fc`/`tc` are the MCU's own per-sample counters and are the exact
dropout detector -- a jump of more than one means samples were lost on the wire,
which a timestamp gap alone could not distinguish from the MCU stalling.

Tare is **not** destructive here. `gui.py` subtracted its zero before writing and
kept no record of it, so a mis-tared run was unrecoverable. This source emits raw
and tared values side by side and the offsets go in the manifest.
"""

from tvcbench.clock import now
from tvcbench.sources.base import Sample, ThreadedSource

BAUD = 115200
NOMINAL_HZ = 50.0

# The MCU reports mN and mN*m; everything downstream of this module is SI.
MN_TO_N = 1e-3

FT_CHANNELS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")

# Read chunk size. Large enough that a 50 Hz stream is usually handed over in one
# or two reads, small enough that the timestamp is not shared across many
# samples -- and the clock fit cleans up whatever sharing remains anyway.
READ_CHUNK = 4096
READ_TIMEOUT_S = 0.05

# A line longer than this is not a status line, it is a wedged framer. Bound it
# so a stuck device cannot grow the buffer without limit.
MAX_LINE_BYTES = 4096


def parse_status_line(line):
    """
    Parse one `key=value` status line into a dict, or return None.

    Ported from `SerialWorker._parse` in gui.py, with the same conventions:
    values carrying a parenthesised annotation keep only the part before it, an
    unquoted value with a '.' is a float and otherwise an int, and anything that
    parses as neither stays a string (`st=SAFE`).
    """
    if not line or not line.startswith("t="):
        # The MCU also emits bare acknowledgements ("OK", "OK SET") and free
        # text. Only status lines carry samples.
        return None

    out = {}
    for part in line.split():
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        if "(" in value:
            value = value.split("(")[0]
        try:
            out[key] = float(value) if "." in value else int(value)
        except ValueError:
            out[key] = value
    return out or None


class LineFramer:
    """
    Turns an arbitrarily chunked byte stream into complete text lines.

    Separated from the transport so the framing can be tested against pathological
    chunk boundaries -- USB CDC will happily split a line across two reads, and a
    framer that mishandles that corrupts one sample in every batch.
    """

    def __init__(self, max_line=MAX_LINE_BYTES):
        self._buf = bytearray()
        self._max_line = max_line
        self.overlong = 0

    def feed(self, data):
        """Append bytes, yield whatever complete lines that produced."""
        self._buf += data
        lines = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                break
            raw = bytes(self._buf[:idx])
            del self._buf[:idx + 1]
            lines.append(raw.decode("utf-8", errors="replace").strip())
        if len(self._buf) > self._max_line:
            # No newline in a full line's worth of bytes: this is not our
            # protocol. Drop it rather than buffering forever.
            self.overlong += 1
            self._buf.clear()
        return lines


class LoadCellSource(ThreadedSource):
    """Reads the STM32 status stream and emits one `Sample` per status line."""

    COLUMNS = (
        ("t_stm_ms",)
        + tuple(f"{ch}_raw" for ch in FT_CHANNELS)
        + FT_CHANNELS
        + ("force_count", "torque_count")
    )
    streams = {"loadcell": COLUMNS}

    def __init__(self, port, baud=BAUD, name="loadcell", serial_factory=None):
        super().__init__(name)
        self.port = port
        self.baud = baud
        # Injectable so tests can drive the whole source from a fake byte stream.
        self._serial_factory = serial_factory
        self._ser = None
        self._framer = LineFramer()

        self.tare = {ch: 0.0 for ch in FT_CHANNELS}
        self._n_parse_fail = 0
        self._n_dropped = 0
        self._last_force_count = None
        self._last_mcu_state = None

    # --- tare ---------------------------------------------------------------
    def set_tare(self, offsets):
        """Install zero offsets, in SI units. Raw values keep being emitted regardless."""
        unknown = set(offsets) - set(FT_CHANNELS)
        if unknown:
            raise ValueError(f"unknown tare channels: {sorted(unknown)}")
        self.tare = {ch: float(offsets.get(ch, 0.0)) for ch in FT_CHANNELS}

    def clear_tare(self):
        self.tare = {ch: 0.0 for ch in FT_CHANNELS}

    @staticmethod
    def compute_tare(samples):
        """
        Mean raw value per channel over `samples` -- the zero to install.

        A plain mean is right here: the stand is at rest, so there is no signal
        to protect from outliers, and any real outlier during a tare means the
        bench was disturbed and the tare should be retaken rather than robustified.
        """
        totals = {ch: 0.0 for ch in FT_CHANNELS}
        n = 0
        for s in samples:
            if any(f"{ch}_raw" not in s.fields for ch in FT_CHANNELS):
                continue
            for ch in FT_CHANNELS:
                totals[ch] += s.fields[f"{ch}_raw"]
            n += 1
        if n == 0:
            raise ValueError("no usable samples to tare from")
        return {ch: totals[ch] / n for ch in FT_CHANNELS}, n

    # --- transport ----------------------------------------------------------
    def _open(self):
        if self._serial_factory is not None:
            self._ser = self._serial_factory()
            return
        import serial   # imported here so the module loads without pyserial

        self._ser = serial.Serial(self.port, self.baud, timeout=READ_TIMEOUT_S)

    def _close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def _read_once(self):
        waiting = getattr(self._ser, "in_waiting", 0) or 1
        data = self._ser.read(min(waiting, READ_CHUNK))
        # Stamp immediately on return, before any parsing: the clock fit can
        # remove batching, but not work this thread does after the read.
        t_mono = now()
        if not data:
            return
        for line in self._framer.feed(data):
            self._handle_line(line, t_mono)

    def _handle_line(self, line, t_mono):
        parsed = parse_status_line(line)
        if parsed is None:
            if line and not line.startswith(("OK", "!")):
                self._n_parse_fail += 1
            return
        if "st" in parsed:
            self._last_mcu_state = parsed["st"]

        fields = {"t_stm_ms": parsed.get("t")}
        for ch in FT_CHANNELS:
            raw = parsed.get(ch)
            raw = None if raw is None else float(raw) * MN_TO_N
            fields[f"{ch}_raw"] = raw
            fields[ch] = None if raw is None else raw - self.tare[ch]

        fc = parsed.get("fc")
        fields["force_count"] = fc
        fields["torque_count"] = parsed.get("tc")

        # A counter that skips means the wire lost samples. Worth knowing exactly:
        # a gap in time could equally be the MCU stalling, and the two failures
        # call for different fixes (cable/hub vs firmware/load).
        if isinstance(fc, int):
            if self._last_force_count is not None:
                step = fc - self._last_force_count
                if step > 1:
                    self._n_dropped += step - 1
            self._last_force_count = fc

        self._emit(Sample(self.name, t_mono, parsed.get("t"), fields))

    def stats(self):
        base = super().stats()
        base.update({
            "dropped": self._n_dropped,
            "parse_fail": self._n_parse_fail,
            "overlong_lines": self._framer.overlong,
            "mcu_state": self._last_mcu_state,
            "tare": dict(self.tare),
        })
        return base


def thrust_n(fields):
    """
    Thrust in newtons, positive up, from a load-cell sample's fields.

    The stand reads vertical load negative, so thrust is -Fz -- the same
    convention `tvctools.align.thrust_signal` uses. Tared, because a thrust limit
    or a settling test wants the change from rest, not the stand's dead weight.
    """
    fz = fields.get("Fz")
    return None if fz is None else -fz


def torque_nm(fields):
    """Reaction torque about the thrust axis, in N*m. Sign is rotor A minus rotor B."""
    return fields.get("Tz")
