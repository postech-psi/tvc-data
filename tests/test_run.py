"""
End-to-end run tests, against the simulated bench.

The whole point of the sim sources is that these paths can be exercised with no
hardware. What matters most here is the safety behaviour: every abort reason
must fire, and every one of them must still leave a readable run on disk with
the motors stopped. A run that dies without a manifest is a wasted session.
"""

import json
import os

import pytest

from tvcbench import config, recorder as rec, sequence
from tvcbench.actuator import PWM_MIN_US, Actuator
from tvcbench.dwell import DwellController, mean_sem
from tvcbench.recorder import Recorder
from tvcbench.runner import Runner
from tvcbench.sources.sim import SimBench, SimLink, SimLoadCellSource, SimMavlinkSource
from tvcbench.supervisor import Abort, Supervisor

FAST_PLAN = {
    "schema_version": 1,
    "grid": {"a": [1300, 1300, 100], "b": [1000, 2000, 500]},
    "repeats": 1,
    "order": {"mode": "sequential"},
    "dwell": {"mode": "fixed", "fixed_s": 0.25},
    "idle": {"pre_s": 0.1, "post_s": 0.1},
    "post_stop": {"seconds": 0.3},
    "tare": {"seconds": 0.2},
    "warmup": {"seconds": 0.1},
    "ramp": {"steps": 2, "seconds": 0.1},
    "chirp": {"enabled": False},
    "limits": {"loadcell_dropout_s": 0.5, "heartbeat_timeout_s": 3.0},
}


def make_plan(**overrides):
    merged = json.loads(json.dumps(FAST_PLAN))
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key].update(value)
        else:
            merged[key] = value
    return config.Plan(merged)


class Harness:
    """A complete simulated run, assembled the way the CLI assembles a real one."""

    def __init__(self, tmp_path, plan=None, motors=True, seed=1, **sup_kw):
        self.plan = plan or make_plan()
        self.bench = SimBench(seed=seed, enabled=motors)
        self.link = SimLink(self.bench)
        self.loadcell = SimLoadCellSource(self.bench, hz=200.0)
        self.mavlink = SimMavlinkSource(self.bench, servo_hz=50.0, battery_hz=25.0)
        self.actuator = Actuator(self.link, enabled=motors)

        streams = dict(self.loadcell.streams)
        streams.update(self.mavlink.streams)
        self.recorder = Recorder(str(tmp_path), rec.make_run_id(), streams,
                                 plan=self.plan)
        self.supervisor = Supervisor(self.plan["limits"], **sup_kw)
        self.runner = Runner(self.plan, self.link, self.actuator, self.loadcell,
                             self.mavlink, self.recorder, self.supervisor, seed=seed)

    def run(self):
        self.loadcell.start()
        self.mavlink.start()
        try:
            return self.runner.run()
        finally:
            self.loadcell.stop()
            self.mavlink.stop()

    def read(self, name):
        path = os.path.join(self.recorder.final_dir, name)
        with open(path, encoding="utf-8") as f:
            return f.read()

    def rows(self, name):
        import csv

        with open(os.path.join(self.recorder.final_dir, name), encoding="utf-8") as f:
            return list(csv.DictReader(f))


@pytest.fixture(scope="module")
def done(tmp_path_factory):
    """One complete run, shared by the assertions below -- it takes real seconds."""
    h = Harness(tmp_path_factory.mktemp("run"))
    return h, h.run()


class TestCompleteRun:
    def test_completes(self, done):
        _, manifest = done
        assert manifest["outcome"] == "completed"
        assert manifest["abort"] is None

    def test_directory_is_renamed_out_of_part(self, done):
        h, _ = done
        assert os.path.isdir(h.recorder.final_dir)
        assert not h.recorder.final_dir.endswith(".part")
        assert not os.path.exists(h.recorder.part_dir)

    def test_writes_every_stream_and_the_metadata(self, done):
        h, _ = done
        for name in ("loadcell.csv", "fc_servo.csv", "fc_battery.csv", "fc_esc.csv",
                     "sequence.csv", "events.jsonl", "manifest.json"):
            assert os.path.exists(os.path.join(h.recorder.final_dir, name)), name

    def test_streams_are_not_joined(self, done):
        """Each at its own rate, with no value carried across streams."""
        h, _ = done
        loadcell = h.rows("loadcell.csv")
        battery = h.rows("fc_battery.csv")
        assert len(loadcell) > len(battery) * 2
        assert "voltage_v" not in loadcell[0]
        assert "Fz" not in battery[0]

    def test_battery_rows_have_no_device_clock(self, done):
        """BATTERY_STATUS carries no timestamp; inventing one would be a lie."""
        h, _ = done
        assert all(r["dev_t"] == "" for r in h.rows("fc_battery.csv"))

    def test_sequence_records_actual_against_planned(self, done):
        h, _ = done
        rows = h.rows("sequence.csv")
        assert len(rows) == len(h.runner.segments)
        for row in rows:
            actual = float(row["actual_dwell_s"])
            planned = float(row["planned_dwell_s"])
            assert actual >= planned          # never cut a segment short
            assert actual < planned + 0.5

    def test_every_segment_id_is_present_and_ordered(self, done):
        h, _ = done
        ids = [int(r["seg_id"]) for r in h.rows("sequence.csv")]
        assert ids == list(range(len(ids)))

    def test_manifest_carries_the_plan_verbatim_and_resolved(self, done):
        _, manifest = done
        assert manifest["plan_raw"]["dwell"]["fixed_s"] == 0.25
        assert manifest["plan"]["hardware"]["fc"]["baud"] == 921600   # from defaults
        assert manifest["seed"] == 1

    def test_manifest_carries_the_clock_solution(self, done):
        """Data files hold raw clocks; this is what makes them interpretable."""
        _, manifest = done
        anchors = manifest["clock"]["anchors"]["pairs"]
        assert len(anchors) == 2
        assert anchors[1]["t_mono"] > anchors[0]["t_mono"]

        fit = manifest["clock"]["device_fits"]["loadcell"]
        assert fit["n"] > 0
        assert abs(fit["ppm"]) < 500

    def test_manifest_reports_achieved_against_requested_rates(self, done):
        _, manifest = done
        assert manifest["achieved_rates"]["loadcell"] > 0
        assert manifest["requested_rates"]["fc_servo"] == 50

    def test_tare_is_recorded_and_applied(self, done):
        h, manifest = done
        offsets = manifest["tare"]["offsets"]
        assert offsets["Fz"] == pytest.approx(-9.3, abs=0.2)
        assert manifest["tare"]["n_samples"] > 0

        # Raw keeps the stand's dead load; the tared column does not.
        late = [r for r in h.rows("loadcell.csv") if int(r["seg_id"]) > 2]
        assert float(late[0]["Fz_raw"]) < -8.0
        assert abs(float(late[0]["Fz"])) < 2.0

    def test_events_cover_the_whole_run(self, done):
        h, _ = done
        kinds = [json.loads(line)["kind"]
                 for line in h.read("events.jsonl").splitlines()]
        assert kinds[0] == "run_start"
        assert "tare" in kinds
        assert kinds.count("segment_start") == kinds.count("segment_end")
        assert "motors_stopped" in kinds

    def test_motors_end_stopped(self, done):
        h, _ = done
        assert (h.actuator.a_us, h.actuator.b_us) == (PWM_MIN_US, PWM_MIN_US)

    def test_thrust_responds_to_command(self, done):
        """A sanity check that the loop really drove the simulated bench."""
        h, _ = done
        steps = {(int(r["a_us"]), int(r["b_us"])): float(r["thrust_mean_n"] or 0)
                 for r in h.rows("sequence.csv") if r["kind"] == "step"}
        assert steps[(1300, 2000)] > steps[(1300, 1000)]


class TestPostStopWindow:
    """
    The stretch of recording after the motors are stopped.

    Its value is the zero: the tare is measured once at the start and every step
    is reported against it, so measuring the same quantity again at the end is
    the only thing that says whether it held.
    """

    def _post_stop_row(self, harness):
        rows = [r for r in harness.rows("sequence.csv") if r["kind"] == "post_stop"]
        assert len(rows) == 1
        return rows[0]

    def test_recorded_as_the_final_segment(self, done):
        h, _ = done
        row = self._post_stop_row(h)
        assert int(row["seg_id"]) == len(h.runner.segments) - 1
        assert float(row["actual_dwell_s"]) >= float(row["planned_dwell_s"])

    def test_the_run_loop_never_executes_it(self, done):
        """It belongs to the shutdown path -- ticking the actuator there would
        undo the stop it just performed."""
        h, _ = done
        seg_id = int(self._post_stop_row(h)["seg_id"])
        assert seg_id not in [s.seg_id for s in h.runner.live_segments]

        events = [json.loads(line) for line in h.read("events.jsonl").splitlines()]
        assert not [e for e in events
                    if e["kind"] == "segment_start" and e.get("seg_id") == seg_id]

    def test_samples_keep_arriving_after_the_motors_stop(self, done):
        h, _ = done
        seg_id = int(self._post_stop_row(h)["seg_id"])
        after = [r for r in h.rows("loadcell.csv") if int(r["seg_id"]) == seg_id]
        assert len(after) > 5

        events = [json.loads(line) for line in h.read("events.jsonl").splitlines()]
        kinds = [e["kind"] for e in events]
        assert kinds.index("motors_stopped") < kinds.index("post_stop_start")

        # Most of the window is genuinely after the stop. Not all of it: the
        # first drain hands over whatever the source buffered while `stop()` was
        # repeating its command, and those samples predate it -- the same
        # boundary caveat that applies to every other segment.
        stopped = next(e for e in events if e["kind"] == "motors_stopped")
        assert sum(1 for r in after
                   if float(r["t_mono"]) > stopped["t_mono"]) > len(after) / 2

    def test_manifest_carries_the_zero_and_the_recovered_voltage(self, done):
        _, manifest = done
        post = manifest["post_stop"]
        assert post["reason"] == "planned"
        assert post["n_loadcell"] > 0
        assert post["zero_thrust_n"] is not None
        assert post["voltage_v"] > 0

    def test_the_zero_is_measured_once_the_rotors_have_stopped(self, tmp_path):
        """A tail long enough to settle reads the tare back, not the coast-down."""
        h = Harness(tmp_path, plan=make_plan(idle={"pre_s": 0.1, "post_s": 0.5},
                                             post_stop={"seconds": 1.5}))
        manifest = h.run()
        assert manifest["post_stop"]["zero_thrust_n"] == pytest.approx(0.0, abs=0.3)
        assert not [w for w in manifest["warnings"] if "zero" in w]

    def test_a_drifted_zero_is_reported_not_silently_absorbed(self, tmp_path):
        """The stand loaded after taring: the same signature as a drifted zero,
        and the whole map would be out by that much."""
        h = Harness(tmp_path, plan=make_plan(idle={"pre_s": 0.1, "post_s": 0.5},
                                             post_stop={"seconds": 1.5}))
        h.loadcell.start()
        h.mavlink.start()
        try:
            # The stand's dead load is -9.3 N; taring at -5.0 leaves 4.3 N of it
            # in every reading, exactly as a zero that moved mid-run would.
            h.runner._apply_tare = lambda samples: h.loadcell.set_tare({"Fz": -5.0})
            manifest = h.runner.run()
        finally:
            h.loadcell.stop()
            h.mavlink.stop()

        assert manifest["post_stop"]["zero_thrust_n"] == pytest.approx(4.3, abs=0.3)
        assert any("zero" in w for w in manifest["warnings"])

    def test_still_recorded_when_the_run_aborts(self, tmp_path):
        """An abort is when the state of the bench afterwards matters most."""
        h = Harness(tmp_path, plan=make_plan(limits={"max_thrust_n": 1.0}))
        manifest = h.run()
        assert manifest["outcome"] == "aborted"
        assert manifest["post_stop"]["n_loadcell"] > 0
        assert self._post_stop_row(h)["kind"] == "post_stop"

    def test_can_be_disabled(self, tmp_path):
        h = Harness(tmp_path, plan=make_plan(post_stop={"seconds": 0}))
        manifest = h.run()
        assert manifest["post_stop"] is None
        assert h.runner.post_stop is None
        assert not [r for r in h.rows("sequence.csv") if r["kind"] == "post_stop"]


class TestNoMotorMode:
    def test_full_rehearsal_sends_nothing(self, tmp_path):
        h = Harness(tmp_path, motors=False)
        manifest = h.run()

        assert manifest["outcome"] == "completed"
        assert manifest["motors_enabled"] is False
        assert h.link.n_commands == 0              # nothing reached the hardware
        assert manifest["n_command_sends"] > 0     # but the sequence still ran
        assert len(h.rows("sequence.csv")) == len(h.runner.segments)


class TestAbortPaths:
    """Every limit must fire, and every one must still leave a readable run."""

    def _assert_clean_abort(self, harness, manifest, reason):
        assert manifest["outcome"] == "aborted"
        assert manifest["abort"]["reason"] == reason
        assert os.path.exists(os.path.join(harness.recorder.final_dir,
                                           "manifest.json"))
        assert (harness.actuator.a_us, harness.actuator.b_us) == (PWM_MIN_US,
                                                                  PWM_MIN_US)
        kinds = [json.loads(line)["kind"]
                 for line in harness.read("events.jsonl").splitlines()]
        assert "abort" in kinds and "motors_stopped" in kinds

    def test_low_voltage(self, tmp_path):
        h = Harness(tmp_path, plan=make_plan(limits={"min_voltage_v": 12.5}))
        self._assert_clean_abort(h, h.run(), "low_voltage")

    def test_over_current(self, tmp_path):
        h = Harness(tmp_path, plan=make_plan(limits={"max_current_a": 1.0}))
        self._assert_clean_abort(h, h.run(), "over_current")

    def test_over_thrust(self, tmp_path):
        """Only possible now that the Pi can see force at all."""
        h = Harness(tmp_path, plan=make_plan(limits={"max_thrust_n": 1.0}))
        self._assert_clean_abort(h, h.run(), "over_thrust")

    def test_over_torque(self, tmp_path):
        h = Harness(tmp_path, plan=make_plan(limits={"max_torque_nm": 0.01}))
        self._assert_clean_abort(h, h.run(), "over_torque")

    def test_loadcell_dropout(self, tmp_path):
        h = Harness(tmp_path)
        h.loadcell.start()
        h.mavlink.start()
        try:
            h.loadcell.stop()          # the force sensor dies before the run begins
            manifest = h.runner.run()
        finally:
            h.mavlink.stop()
        assert manifest["outcome"] == "aborted"
        assert manifest["abort"]["reason"] in ("loadcell_silent", "loadcell_dropout")

    def test_stop_file(self, tmp_path):
        stop = tmp_path / "STOP"
        stop.write_text("")
        h = Harness(tmp_path, stop_file=str(stop))
        self._assert_clean_abort(h, h.run(), "stopped")

    def test_stop_request(self, tmp_path):
        h = Harness(tmp_path)
        h.supervisor.request_stop(source="test")
        self._assert_clean_abort(h, h.run(), "stopped")

    def test_refuses_to_run_while_armed(self, tmp_path):
        h = Harness(tmp_path)
        h.loadcell.start()
        h.mavlink.start()
        h.mavlink.armed = True
        try:
            manifest = h.runner.run()
        finally:
            h.loadcell.stop()
            h.mavlink.stop()
        assert manifest["abort"]["reason"] == "armed"
        assert "disarm" in manifest["abort"]["detail"]


class TestRecorder:
    def test_part_directory_until_finalized(self, tmp_path):
        r = Recorder(str(tmp_path), "r1", {"s": ("v",)}).open()
        assert os.path.isdir(r.part_dir)
        assert not os.path.exists(r.final_dir)
        r.finalize()
        assert os.path.isdir(r.final_dir)

    def test_never_overwrites_an_existing_run(self, tmp_path):
        Recorder(str(tmp_path), "r1", {"s": ("v",)}).open().finalize()
        second = Recorder(str(tmp_path), "r1", {"s": ("v",)}).open()
        second.finalize()
        assert second.final_dir.endswith("_2")
        assert os.path.isdir(os.path.join(str(tmp_path), "r1"))

    def test_unknown_stream_is_reported_not_dropped_silently(self, tmp_path):
        from tvcbench.sources.base import Sample

        r = Recorder(str(tmp_path), "r1", {"s": ("v",)}).open()
        r.write_samples([Sample("mystery", 1.0, None, {"v": 1})], seg_id=0)
        assert any("mystery" in w for w in r.warnings)
        r.finalize()

    def test_rounds_floats_to_bounded_precision(self, tmp_path):
        from tvcbench.sources.base import Sample

        r = Recorder(str(tmp_path), "r1", {"s": ("v",)}).open()
        r.write_samples([Sample("s", 1.0, None, {"v": 1 / 3})], seg_id=0)
        r.finalize()
        with open(os.path.join(r.final_dir, "s.csv"), encoding="utf-8") as f:
            assert "0.333333" in f.read()

    def test_event_fields_cannot_shadow_the_event_type(self, tmp_path):
        r = Recorder(str(tmp_path), "r1", {}).open()
        record = r.event("segment_start", kind="step")
        assert record["kind"] == "segment_start"
        r.finalize()


class TestSupervisorUnit:
    def test_thrust_guard_uses_a_median_not_one_sample(self, tmp_path):
        """0.7 N of vibration means a single sample cannot be trusted to abort."""
        s = Supervisor({"max_thrust_n": 10.0})
        for value in (2.0, 2.0, 40.0, 2.0, 2.0):     # one wild sample
            s.note_force(value, None)
        s.check(t_mono=1.0)                          # does not abort

        for value in (40.0, 41.0, 39.0, 42.0, 40.0):  # a real excursion
            s.note_force(value, None)
        with pytest.raises(Abort, match="over_thrust"):
            s.check(t_mono=1.0)

    def test_limits_are_skipped_before_the_motors_are_live(self, tmp_path):
        s = Supervisor({"max_thrust_n": 1.0, "min_voltage_v": 12.0})
        s.note_force(50.0, None)
        s.note_battery(11.0, 0.0)
        s.check(t_mono=1.0, motors_live=False)       # tare and warmup phases

    def test_torque_limit_is_two_sided(self, tmp_path):
        s = Supervisor({"max_torque_nm": 0.5})
        for _ in range(5):
            s.note_force(None, -0.9)
        with pytest.raises(Abort, match="over_torque"):
            s.check(t_mono=1.0)

    def test_source_errors_abort_immediately(self):
        s = Supervisor({"loadcell_dropout_s": 0.5})
        with pytest.raises(Abort, match="loadcell_error"):
            s.check(t_mono=1.0, loadcell_stats={"error": "device disconnected"})

    def test_heartbeat_loss_aborts(self):
        s = Supervisor({"heartbeat_timeout_s": 1.0})
        with pytest.raises(Abort, match="fc_heartbeat_lost"):
            s.check(t_mono=10.0, fc_stats={"last_heartbeat_t_mono": 5.0})

    def test_disabled_limits_never_fire(self):
        s = Supervisor({"max_thrust_n": 0.0, "min_voltage_v": 0.0})
        for _ in range(5):
            s.note_force(999.0, 999.0)
        s.note_battery(0.1, 999.0)
        s.check(t_mono=1.0)


class TestDwell:
    def _segment(self, min_s, max_s, nominal):
        from tvcbench.sequence import STEP, Segment

        return Segment(0, STEP, 1400, 1500, nominal, min_s, max_s, sweep=1)

    def test_fixed_holds_the_planned_time(self):
        c = DwellController(self._segment(2.0, 2.0, 2.0),
                            {"mode": "fixed"}, t_start=0.0)
        assert c.done(1.9) == (False, "")
        assert c.done(2.0)[0]

    def test_sem_target_stops_once_the_error_bar_is_small_enough(self):
        seg = self._segment(1.0, 30.0, 4.0)
        c = DwellController(seg, {"mode": "sem_target",
                                  "target_thrust_sem_n": 0.05}, t_start=0.0)
        import random

        rng = random.Random(4)
        t = 0.0
        finished = False
        for _ in range(4000):
            t += 0.02
            c.observe(t, 10.0 + rng.gauss(0, 0.7))
            finished, reason = c.done(t)
            if finished:
                assert reason == "sem_target"
                break
        assert finished
        assert 1.0 < t < 30.0

    def test_sem_target_gives_up_at_max_dwell(self):
        """The bound that stops a step that can never reach its target."""
        import random

        rng = random.Random(9)
        seg = self._segment(0.5, 2.0, 1.0)
        c = DwellController(seg, {"mode": "sem_target",
                                  "target_thrust_sem_n": 1e-6}, t_start=0.0)
        t, result = 0.0, (False, "")
        for _ in range(200):
            t += 0.02
            c.observe(t, 10.0 + rng.gauss(0, 0.7))   # unreachably noisy for the target
            result = c.done(t)
            if result[0]:
                break
        assert result == (True, "max_dwell")
        assert t == pytest.approx(seg.max_dwell_s, abs=0.05)

    def test_settle_waits_for_the_signal_to_stop_moving(self):
        seg = self._segment(0.5, 20.0, 3.0)
        c = DwellController(seg, {"mode": "settle", "settle_rate_n_per_s": 0.3,
                                  "settle_window_s": 0.5,
                                  "hold_after_settle_s": 1.0}, t_start=0.0)
        t = 0.0
        # Ramping hard: must not be declared settled.
        for _ in range(100):
            t += 0.02
            c.observe(t, 5.0 * t)
            assert not c.done(t)[0]
        # Flat from here.
        finished = False
        for _ in range(200):
            t += 0.02
            c.observe(t, 10.0)
            finished, reason = c.done(t)
            if finished:
                assert reason == "settled"
                break
        assert finished

    def test_non_measurement_segments_are_never_adaptive(self):
        from tvcbench.sequence import IDLE_PRE, Segment

        seg = Segment(0, IDLE_PRE, 1000, 1000, 5.0)
        c = DwellController(seg, {"mode": "sem_target"}, t_start=0.0)
        assert not c.adaptive
        assert c.done(5.0)[0]

    def test_mean_sem_is_corrected_for_autocorrelation(self):
        """Vibration is correlated; the naive sd/sqrt(n) would be about 2x too small."""
        import random

        rng = random.Random(2)
        white, correlated, prev = [], [], 0.0
        for _ in range(4000):
            step = rng.gauss(0, 0.7)
            white.append(step)
            prev = 0.5 * prev + step
            correlated.append(prev)

        _, sem_white, _ = mean_sem(white)
        _, sem_corr, _ = mean_sem(correlated)
        assert sem_corr > sem_white

    def test_mean_sem_declines_to_guess_from_too_few_samples(self):
        mean, sem, n = mean_sem([1.0, 2.0, 3.0])
        assert mean == pytest.approx(2.0)
        assert sem is None and n == 3

    def test_mean_sem_handles_nothing(self):
        assert mean_sem([]) == (None, None, 0)


def test_sim_pack_matches_measured_power_law():
    """
    The simulated pack should sag about as much as the real one.

    `out/pwm_thrust_torque_map.csv` gives 228.7 W at 12.37 N and 25.5 W at
    3.01 N. If the sim's power law drifted from that, the low-voltage abort path
    would be rehearsed against the wrong bench.
    """
    bench = SimBench(seed=0)
    bench.set_command(1400, 2000)
    t = 0.0
    for _ in range(400):                    # 8 s, well past the 0.25 s spin-up
        bench.advance(t)
        t += 0.02

    power = bench.voltage_v * bench.current_a
    assert 8.0 < bench.thrust_n < 20.0
    assert power == pytest.approx(5.1 * bench.thrust_n ** 1.5, rel=0.4)


def test_expand_matches_what_the_runner_executes(tmp_path):
    """`plan show` must describe the run that `run` actually performs."""
    plan = make_plan()
    segments, _ = sequence.expand(plan, seed=1)
    h = Harness(tmp_path, plan=plan, seed=1)
    assert [s.as_dict() for s in h.runner.segments] == [s.as_dict() for s in segments]
