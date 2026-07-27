"""
Load-cell source tests.

The transport is faked so the framing, unit conversion, tare and dropout logic
can be exercised on a normal machine with no STM32 attached. Chunk boundaries
are deliberately pathological: USB CDC splits lines wherever it likes, and a
framer that mishandles that corrupts one sample per batch -- which is exactly
the kind of fault that survives to the map as unexplained scatter.
"""

import threading
import time

import pytest

from tvcbench.sources.loadcell import (
    FT_CHANNELS,
    LineFramer,
    LoadCellSource,
    parse_status_line,
    thrust_n,
    torque_nm,
)

# Status lines lead with `t=`; that prefix is what gui.py's proven reader keys on
# to tell a sample apart from the MCU's acknowledgements and boot chatter.
LINE = ("t=3009477 st=SAFE pwm=1000 rpm=1724.0 "
        "Fx=-838.0 Fy=-12.2 Fz=-9320.0 Tx=10.41 Ty=-65.11 Tz=-4.01 "
        "fc=91823 tc=91823")


class FakeSerial:
    """Hands over pre-programmed chunks, one per read, then goes quiet."""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.closed = False
        self.in_waiting = 4096

    def read(self, _n):
        if self.chunks:
            return self.chunks.pop(0)
        time.sleep(0.001)      # stand in for the serial read timeout
        return b""

    def close(self):
        self.closed = True


def drive(chunks):
    """Run a source over `chunks` synchronously and return the samples."""
    src = LoadCellSource("fake", serial_factory=lambda: FakeSerial(chunks))
    src._open()
    for _ in range(len(chunks)):
        src._read_once()
    return src, src.drain()


class TestParseStatusLine:
    def test_parses_a_full_status_line(self):
        d = parse_status_line(LINE)
        assert d["t"] == 3009477 and isinstance(d["t"], int)
        assert d["Fz"] == pytest.approx(-9320.0)
        assert d["st"] == "SAFE"          # unparseable as a number -> string
        assert d["fc"] == 91823

    def test_strips_parenthesised_annotations(self):
        assert parse_status_line("t=1 fc=1234(5)")["fc"] == 1234

    @pytest.mark.parametrize("line", ["", "OK", "OK SET", "boot ok", None,
                                      "st=SAFE fc=1"])
    def test_rejects_non_status_lines(self, line):
        # Only lines starting with `t=` carry a sample; everything else is chatter.
        assert parse_status_line(line) is None


class TestLineFramer:
    def test_reassembles_a_line_split_across_chunks(self):
        f = LineFramer()
        assert f.feed(b"t=1 Fz=-1.0") == []
        assert f.feed(b" fc=2\n") == ["t=1 Fz=-1.0 fc=2"]

    def test_splits_multiple_lines_in_one_chunk(self):
        assert LineFramer().feed(b"a\nb\nc\n") == ["a", "b", "c"]

    def test_strips_carriage_returns(self):
        assert LineFramer().feed(b"t=1\r\n") == ["t=1"]

    def test_byte_at_a_time_matches_one_shot(self):
        payload = (LINE + "\n").encode()
        one = LineFramer().feed(payload)
        f = LineFramer()
        many = [line for b in payload for line in f.feed(bytes([b]))]
        assert one == many == [LINE]

    def test_drops_a_wedged_buffer_rather_than_growing(self):
        f = LineFramer(max_line=64)
        f.feed(b"x" * 200)
        assert f.overlong == 1
        assert f.feed(b"t=1\n") == ["t=1"]      # recovers on the next real line


class TestUnitsAndTare:
    def test_converts_mn_to_si(self):
        _, samples = drive([(LINE + "\n").encode()])
        f = samples[0].fields
        assert f["Fz_raw"] == pytest.approx(-9.320)        # -9320 mN -> N
        assert f["Tz_raw"] == pytest.approx(-0.00401)      # -4.01 mN*m -> N*m
        assert f["t_stm_ms"] == 3009477
        assert samples[0].dev_t == 3009477

    def test_tare_is_not_destructive(self):
        """The defect this replaces: gui.py wrote tared values and lost the raw."""
        src = LoadCellSource("fake", serial_factory=lambda: FakeSerial([]))
        src._open()
        src.set_tare({"Fz": -9.0})
        src._handle_line(LINE, t_mono=1.0)
        f = src.drain()[0].fields

        assert f["Fz_raw"] == pytest.approx(-9.320)        # raw survives
        assert f["Fz"] == pytest.approx(-0.320)            # tared alongside it
        assert src.stats()["tare"]["Fz"] == -9.0           # and the offset is recorded

    def test_untared_channels_pass_through(self):
        src = LoadCellSource("fake", serial_factory=lambda: FakeSerial([]))
        src._open()
        src.set_tare({"Fz": -9.0})
        src._handle_line(LINE, t_mono=1.0)
        f = src.drain()[0].fields
        assert f["Fx"] == f["Fx_raw"]

    def test_compute_tare_averages_raw(self):
        chunks = [(f"t={i} Fx=1000.0 Fy=0.0 Fz={-9000.0 - i * 100} "
                   f"Tx=0.0 Ty=0.0 Tz=0.0\n").encode() for i in range(4)]
        _, samples = drive(chunks)
        tare, n = LoadCellSource.compute_tare(samples)
        assert n == 4
        assert tare["Fx"] == pytest.approx(1.0)
        assert tare["Fz"] == pytest.approx(-9.15)          # mean of -9.0..-9.3

    def test_compute_tare_needs_samples(self):
        with pytest.raises(ValueError):
            LoadCellSource.compute_tare([])

    def test_set_tare_rejects_unknown_channels(self):
        src = LoadCellSource("fake")
        with pytest.raises(ValueError):
            src.set_tare({"Fq": 1.0})

    def test_all_channels_present_even_when_absent_on_the_wire(self):
        _, samples = drive([b"t=5 Fz=-1000.0\n"])
        f = samples[0].fields
        for ch in FT_CHANNELS:
            assert ch in f and f"{ch}_raw" in f
        assert f["Fx"] is None                              # missing, not zero
        assert f["Fz"] == pytest.approx(-1.0)


class TestDropoutDetection:
    def test_counts_samples_lost_on_the_wire(self):
        chunks = [f"t={i} fc={c} Fz=-1000.0\n".encode()
                  for i, c in enumerate([10, 11, 15, 16])]     # 12,13,14 lost
        src, _ = drive(chunks)
        assert src.stats()["dropped"] == 3

    def test_clean_stream_reports_none(self):
        chunks = [f"t={i} fc={10 + i} Fz=-1000.0\n".encode() for i in range(5)]
        src, _ = drive(chunks)
        assert src.stats()["dropped"] == 0

    def test_survives_a_stream_without_counters(self):
        chunks = [f"t={i} Fz=-1000.0\n".encode() for i in range(3)]
        src, samples = drive(chunks)
        assert len(samples) == 3
        assert src.stats()["dropped"] == 0


class TestDerivedQuantities:
    def test_thrust_is_negative_fz(self):
        # The stand reads vertical load negative; tvctools.align uses the same sign.
        assert thrust_n({"Fz": -9.32}) == pytest.approx(9.32)
        assert thrust_n({}) is None

    def test_torque_is_tz(self):
        assert torque_nm({"Tz": 0.11}) == pytest.approx(0.11)


class TestThreadedLifecycle:
    def test_start_drain_stop(self):
        chunks = [(LINE + "\n").encode()] * 3
        src = LoadCellSource("fake", serial_factory=lambda: FakeSerial(chunks))
        src.start()
        deadline = time.time() + 2.0
        got = []
        while time.time() < deadline and len(got) < 3:
            got += src.drain()
            time.sleep(0.01)
        src.stop()

        assert len(got) == 3
        assert not src.alive
        assert src.error is None

    def test_transport_failure_is_recorded_not_retried(self):
        """A mid-run dropout has already holed the data; reconnecting would hide it."""
        class Exploding(FakeSerial):
            def read(self, _n):
                raise OSError("device disconnected")

        src = LoadCellSource("fake", serial_factory=lambda: Exploding([]))
        src.start()
        for _ in range(100):
            if not src.alive:
                break
            time.sleep(0.01)
        src.stop()

        assert "device disconnected" in src.error
        assert not src.alive

    def test_buffer_is_bounded(self):
        src = LoadCellSource("fake", serial_factory=lambda: FakeSerial([]))
        src._open()
        for i in range(src._queue.maxlen + 50):
            src._handle_line(f"t={i} Fz=-1000.0", t_mono=float(i))
        assert len(src._queue) == src._queue.maxlen
        assert src.stats()["overflow"] == 50
        # Newest samples win: a stalled consumer loses history, not the present.
        assert src.drain()[-1].fields["t_stm_ms"] == src._queue.maxlen + 49

    def test_double_start_is_refused(self):
        src = LoadCellSource("fake", serial_factory=lambda: FakeSerial([]))
        src.start()
        with pytest.raises(RuntimeError):
            src.start()
        src.stop()


def test_columns_match_emitted_fields():
    """The recorder writes the declared columns blindly, so they must match the fields."""
    _, samples = drive([(LINE + "\n").encode()])
    assert set(LoadCellSource.streams["loadcell"]) == set(samples[0].fields)
    assert samples[0].stream == "loadcell"


def test_emit_is_threadsafe():
    """Reader thread emits while the runner drains; neither may lose or duplicate."""
    src = LoadCellSource("fake", serial_factory=lambda: FakeSerial([]))
    src._open()
    n = 2000
    src._queue = type(src._queue)(maxlen=n * 2)

    def produce():
        for i in range(n):
            src._handle_line(f"t={i} Fz=-1000.0", t_mono=float(i))

    got = []
    t = threading.Thread(target=produce)
    t.start()
    while t.is_alive() or len(got) < n:
        got += src.drain()
    t.join()

    assert [s.fields["t_stm_ms"] for s in got] == list(range(n))
