"""
Selftest tests.

Each fault the selftest is supposed to catch is injected here, because a health
check that silently passes on broken hardware is worse than no health check: it
converts a caught problem into a wasted bench session.
"""

import numpy as np
import pytest

from tvcbench import selftest as st
from tvcbench.sources.loadcell import LoadCellSource


class StubSource:
    """A source that has already produced its samples."""

    def __init__(self, stats):
        self._stats = stats
        self.alive = True

    def stats(self):
        return dict(self._stats)

    def drain(self):
        return []


def make_samples(n=1000, hz=50.0, noise=0.01, ppm=0.0, seed=3,
                 drop_at=None, reset_at=None, gap_at=None, gap_s=0.0):
    """A clean 50 Hz load-cell burst, with optional faults injected."""
    rng = np.random.default_rng(seed)
    samples = []
    t_ms = 100000
    fc = 5000
    t_mono = 1000.0
    for i in range(n):
        if reset_at is not None and i == reset_at:
            t_ms, fc = 0, 0                    # MCU restarted: brownout signature
        if drop_at is not None and i == drop_at:
            fc += 4                            # three samples lost on the wire
        if gap_at is not None and i == gap_at:
            t_mono += gap_s

        fields = {"t_stm_ms": t_ms, "force_count": fc, "torque_count": fc}
        for ch in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"):
            scale = noise if ch.startswith("F") else noise * 0.03
            raw = float(rng.normal(-9.3 if ch == "Fz" else 0.0, scale))
            fields[f"{ch}_raw"] = raw
            fields[ch] = raw
        samples.append(_S("loadcell", t_mono, t_ms, fields))

        t_ms += int(round(1000.0 / hz))
        fc += 1
        t_mono += (1.0 / hz) * (1.0 + ppm * 1e-6)
    return samples


class _S:
    __slots__ = ("stream", "t_mono", "dev_t", "fields")

    def __init__(self, stream, t_mono, dev_t, fields):
        self.stream, self.t_mono, self.dev_t, self.fields = stream, t_mono, dev_t, fields


BASE_STATS = {"dropped": 0, "parse_fail": 0, "overflow": 0, "error": None, "alive": True}


def analyse(samples, **stats):
    s = dict(BASE_STATS)
    s.update(stats)
    return st.analyse(samples, StubSource(s), seconds=20.0)


class TestHealthyLink:
    def test_clean_capture_passes(self):
        report = analyse(make_samples())
        assert st.worst_verdict(report) == st.PASS
        assert report["checks"]["rate_hz"]["value"] == pytest.approx(50.0, rel=1e-3)
        assert report["checks"]["counter_resets"]["value"] == 0

    def test_reports_the_noise_floor_per_channel(self):
        report = analyse(make_samples(noise=0.02))
        chans = report["noise"]["channels"]
        assert set(chans) == {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"}
        assert chans["Fz"]["sd"] == pytest.approx(0.02, rel=0.15)
        assert report["noise"]["thrust_sd_n"] == chans["Fz"]["sd"]

    def test_fits_the_device_clock(self):
        report = analyse(make_samples(n=2000, ppm=60.0))
        # The synthetic host clock runs fast against the device counter, so the
        # fitted slope should come back near +60 ppm.
        assert report["clock"]["available"]
        assert report["clock"]["ppm"] == pytest.approx(60.0, abs=25.0)
        assert report["clock"]["ppm_verdict"] == st.PASS


class TestFaultsAreCaught:
    def test_no_data_fails_loudly(self):
        report = analyse([])
        assert st.worst_verdict(report) == st.FAIL
        assert report["checks"]["samples"]["verdict"] == st.FAIL

    def test_counter_reset_fails_and_names_power(self):
        """The brownout signature -- risk 1 of hosting the board on the Pi."""
        report = analyse(make_samples(reset_at=500))
        assert report["checks"]["counter_resets"]["value"] == 1
        assert report["checks"]["counter_resets"]["verdict"] == st.FAIL
        assert "power" in report["checks"]["counter_resets"]["note"]
        assert st.worst_verdict(report) == st.FAIL

    def test_wire_dropouts_are_counted_from_the_source(self):
        report = analyse(make_samples(), dropped=40)
        assert report["checks"]["wire_dropouts"]["value"] == 40
        assert report["checks"]["wire_dropouts"]["verdict"] == st.FAIL

    def test_a_single_dropout_warns_but_does_not_fail(self):
        report = analyse(make_samples(), dropped=1)
        assert report["checks"]["wire_dropouts"]["verdict"] == st.WARN

    def test_low_rate_is_caught(self):
        report = analyse(make_samples(hz=30.0))
        assert report["checks"]["rate_hz"]["verdict"] == st.FAIL

    def test_slightly_low_rate_only_warns(self):
        report = analyse(make_samples(hz=47.0))
        assert report["checks"]["rate_hz"]["verdict"] == st.WARN

    def test_stall_shows_as_a_gap(self):
        report = analyse(make_samples(gap_at=300, gap_s=0.6))
        assert report["checks"]["max_gap_s"]["value"] == pytest.approx(0.62, abs=0.02)
        assert report["checks"]["max_gap_s"]["verdict"] == st.FAIL

    def test_parse_failures_warn(self):
        report = analyse(make_samples(), parse_fail=12)
        assert report["checks"]["parse_failures"]["verdict"] == st.WARN

    def test_buffer_overflow_is_surfaced(self):
        report = analyse(make_samples(), overflow=7)
        assert report["checks"]["buffer_overflow"]["verdict"] == st.WARN


class TestBaselineComparison:
    """Risk 2: a ground loop raises the floor, and only a baseline reveals it."""

    def test_matching_floor_passes(self):
        base = analyse(make_samples(noise=0.01, seed=1))
        report = st.analyse(make_samples(noise=0.01, seed=2), StubSource(BASE_STATS),
                            seconds=20.0, baseline=base)
        assert report["noise"]["baseline_verdict"] == st.PASS
        assert abs(report["noise"]["baseline_delta_frac"]) < 0.1

    def test_doubled_floor_fails(self):
        base = analyse(make_samples(noise=0.01, seed=1))
        report = st.analyse(make_samples(noise=0.03, seed=2), StubSource(BASE_STATS),
                            seconds=20.0, baseline=base)
        assert report["noise"]["baseline_verdict"] == st.FAIL
        assert st.worst_verdict(report) == st.FAIL

    def test_roundtrips_through_a_file(self, tmp_path):
        path = tmp_path / "baseline.json"
        st.save_baseline(analyse(make_samples()), str(path))
        loaded = st.load_baseline(str(path))
        assert loaded["noise"]["thrust_sd_n"] > 0


class TestReportRendering:
    def test_renders_every_section(self):
        text = st.format_report(analyse(make_samples()), port="/dev/tvc-loadcell")
        for expected in ("LOAD CELL", "/dev/tvc-loadcell", "CLOCK FIT",
                         "NOISE FLOOR", "OVERALL: PASS", "rate error", "batch spread"):
            assert expected in text

    def test_renders_an_empty_capture_without_crashing(self):
        text = st.format_report(analyse([]))
        assert "OVERALL: FAIL" in text

    def test_shows_baseline_line_when_given_one(self):
        base = analyse(make_samples(noise=0.01, seed=1))
        report = st.analyse(make_samples(noise=0.01, seed=2), StubSource(BASE_STATS),
                            seconds=20.0, baseline=base)
        assert "vs baseline" in st.format_report(report)


class TestCollect:
    def test_stops_early_when_the_source_dies(self):
        class Dying:
            alive = False

            def stats(self):
                return dict(BASE_STATS)

            def drain(self):
                return []

        t0 = st.now()
        st.collect(Dying(), seconds=30.0)
        assert st.now() - t0 < 1.0        # did not sit through the full 30 s

    def test_collects_from_a_real_source(self):
        class Fake:
            def __init__(self):
                self.chunks = [b"t=1 fc=1 Fz=-9300.0\nt=21 fc=2 Fz=-9310.0\n"]
                self.in_waiting = 128

            def read(self, _n):
                return self.chunks.pop(0) if self.chunks else b""

            def close(self):
                pass

        src = LoadCellSource("fake", serial_factory=Fake)
        src.start()
        samples = st.collect(src, seconds=0.3)
        src.stop()
        assert len(samples) == 2


def test_cli_parses_both_commands():
    from tvcbench.cli import build_parser

    p = build_parser()
    assert p.parse_args(["devices", "--probe"]).probe is True
    args = p.parse_args(["selftest", "--port", "/dev/ttyACM0", "--seconds", "5"])
    assert args.port == "/dev/ttyACM0" and args.seconds == 5.0
