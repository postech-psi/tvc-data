"""
The source contract.

A source owns one device, reads it on its own thread, and emits `Sample`s
stamped with the master clock. Sources never talk to each other and never join
their output -- that is the whole point. The old logger emitted a row only when
`SERVO_OUTPUT_RAW` arrived and carried the last voltage forward into it, which
invented a 20 Hz rate for a 10 Hz quantity. Here each stream is written at
whatever rate it actually has, and the join happens in analysis where the
resampling rule can be chosen deliberately.

Why a thread per source rather than one polling loop: the arrival timestamp is
only as good as the moment it is taken. A polling loop busy sending actuator
commands would stamp samples late and, worse, *unevenly* -- and uneven latency
is the one thing the clock fit cannot undo, because it looks exactly like real
device jitter. A dedicated reader thread stamps on the read itself.
"""

import threading
from collections import deque


class Sample:
    """
    One reading from one device.

    `fields` maps 1:1 onto the columns of that stream's CSV, so the recorder
    stays a dumb writer with no per-stream knowledge. `dev_t` is the device's
    own counter in its native units -- kept raw so the clock fit, not the
    source, decides how to scale it.
    """

    __slots__ = ("stream", "t_mono", "dev_t", "fields")

    def __init__(self, stream, t_mono, dev_t, fields):
        self.stream = stream
        self.t_mono = t_mono
        self.dev_t = dev_t
        self.fields = fields

    def __repr__(self):
        return f"Sample({self.stream}, t_mono={self.t_mono:.4f}, dev_t={self.dev_t})"


class Source:
    """
    Interface every source implements.

    `start`/`stop` bracket acquisition; `drain` hands over everything buffered
    since the last call. Callers must drain regularly -- sources bound their
    buffers and drop the oldest rather than growing without limit, because a
    stalled consumer must not take the process down mid-run with the motors
    spinning.
    """

    name = "source"
    #: `{stream_name: (column, ...)}` -- the CSV layout of every stream this
    #: source produces, excluding the clock columns the recorder always writes.
    #: Declared here so the recorder needs no per-device knowledge. One device
    #: may feed several streams: the Pixhawk emits servo, battery and ESC
    #: telemetry at three different rates, and merging them would be the exact
    #: mistake this design exists to avoid.
    streams = {}

    def start(self):
        raise NotImplementedError

    def drain(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def stats(self):
        """Counters for the QC report: received, dropped, errors, last arrival."""
        return {}


# One second of a 1 kHz stream. Far more headroom than the runner needs (it
# drains at least every dwell tick), and small enough that a wedged consumer
# costs bounded memory instead of the whole process.
DEFAULT_BUFFER = 1000


class ThreadedSource(Source):
    """
    Base for sources that read a blocking device on a background thread.

    Subclasses implement `_open`, `_read_once` and `_close`. `_read_once` should
    block briefly, stamp the master clock immediately on return, and push
    `Sample`s with `_emit`.
    """

    def __init__(self, name, buffer=DEFAULT_BUFFER):
        self.name = name
        self._queue = deque(maxlen=buffer)
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._error = None
        self._n_received = 0
        self._n_overflow = 0
        self._last_t_mono = None

    # --- subclass hooks -----------------------------------------------------
    def _open(self):
        pass

    def _read_once(self):
        raise NotImplementedError

    def _close(self):
        pass

    # --- machinery ----------------------------------------------------------
    def _emit(self, sample):
        with self._lock:
            if len(self._queue) == self._queue.maxlen:
                self._n_overflow += 1     # deque drops the oldest for us
            self._queue.append(sample)
            self._n_received += 1
            self._last_t_mono = sample.t_mono

    def _loop(self):
        try:
            while not self._stop.is_set():
                self._read_once()
        except Exception as exc:          # noqa: BLE001 - recorded, not swallowed
            # Do not reconnect. A device that dropped out mid-run has already
            # holed the data; the supervisor's dropout check should abort the
            # run rather than have it quietly resume with a gap in the middle.
            self._error = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                self._close()
            except Exception:             # noqa: BLE001
                pass

    def start(self):
        if self._thread is not None:
            raise RuntimeError(f"{self.name} already started")
        self._open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name=self.name, daemon=True)
        self._thread.start()

    def drain(self):
        with self._lock:
            out = list(self._queue)
            self._queue.clear()
        return out

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    @property
    def error(self):
        return self._error

    def stats(self):
        return {
            "received": self._n_received,
            "overflow": self._n_overflow,
            "last_t_mono": self._last_t_mono,
            "error": self._error,
            "alive": self.alive,
        }
