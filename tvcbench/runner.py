"""
The run loop.

Executes a plan's segments in order, holding each command alive, draining every
source, writing to disk, feeding the supervisor and deciding when each segment
is finished. Nothing else in this package knows about the sequence as a whole.

Invariants the rest of the design depends on:

* **The actuator is ticked from this loop and nowhere else.** Commands expire, so
  a loop that stops running stops the motors. Every path out of `run()` --
  completion, abort, exception -- goes through the same stop.
* **Samples carry the `seg_id` that was executing when they were drained.** The
  loop drains every few milliseconds, so at most about one sample near a
  boundary can be attributed to the following segment. `seg_id` is therefore a
  convenience column; the authoritative boundaries are `t_mono_start` and
  `t_mono_end` in `sequence.csv`, measured on the same clock as every sample.
* **The supervisor is checked every pass**, before the dwell test, so a limit
  breach cannot be delayed by a segment that will not end.
"""

from tvcbench.clock import ClockTracker, EpochAnchors, now
from tvcbench.dwell import DwellController
from tvcbench.sequence import STEP, TARE, WARMUP, expand
from tvcbench.sources.loadcell import thrust_n, torque_nm
from tvcbench.supervisor import Abort

# How long the loop sleeps when it has nothing to do. Short enough that a stop
# request is acted on promptly and command resends stay on schedule; long enough
# that the loop is not a spin.
LOOP_SLEEP_S = 0.002

# Wait for the first force sample before commanding anything. Without it the
# dropout check has nothing to compare against and a dead sensor looks healthy.
FIRST_SAMPLE_TIMEOUT_S = 5.0

# Device counter scale factors, counter units -> seconds.
DEV_SCALE = {"loadcell": 1e-3, "fc_servo": 1e-6, "fc_esc": 1e-6}

# Keep the clock fit's memory bounded on long runs. The fit's precision comes
# from the record's span, not its sample count, so thinning costs nothing.
CLOCK_DECIMATE = {"loadcell": 5, "fc_servo": 1, "fc_esc": 1}


class Runner:
    """One run, start to finish."""

    def __init__(self, plan, link, actuator, loadcell, mavlink, recorder,
                 supervisor, seed=None, on_status=None, sleep=None):
        self.plan = plan
        self.link = link
        self.actuator = actuator
        self.loadcell = loadcell
        self.mavlink = mavlink
        self.recorder = recorder
        self.supervisor = supervisor
        self.on_status = on_status

        import time

        self._sleep = time.sleep if sleep is None else sleep

        self.segments, self.seed = expand(plan, seed=seed)
        self.anchors = EpochAnchors()
        self.clocks = {name: ClockTracker(name, scale, CLOCK_DECIMATE.get(name, 1))
                       for name, scale in DEV_SCALE.items()}

        self.outcome = "incomplete"
        self.abort_reason = None
        self.abort_detail = None
        self.tare_offsets = None
        self.tare_n = 0
        self._seg_index = 0

    # --- lifecycle ----------------------------------------------------------
    def run(self):
        """Execute the whole plan. Returns the manifest. Never leaves motors live."""
        self.supervisor.install_signal_handlers()
        self.recorder.open()
        self.anchors.capture()
        self.recorder.event("run_start", run_id=self.recorder.run_id, seed=self.seed,
                            n_segments=len(self.segments))
        try:
            self._preflight()
            self._await_first_force_sample()
            for index, segment in enumerate(self.segments):
                self._seg_index = index
                self._execute(segment)
            self.outcome = "completed"
        except Abort as abort:
            self.outcome = "aborted"
            self.abort_reason = abort.reason
            self.abort_detail = abort.detail
            self.recorder.event("abort", reason=abort.reason, detail=abort.detail)
        except KeyboardInterrupt:
            # Normally the signal handler turns this into a graceful stop, but it
            # can still land between checks. Recorded rather than swallowed: a run
            # that ends this way must say so in its manifest.
            self.outcome = "aborted"
            self.abort_reason = "interrupted"
            self.recorder.event("abort", reason="interrupted")
        except Exception as exc:                       # noqa: BLE001
            self.outcome = "error"
            self.abort_reason = type(exc).__name__
            self.abort_detail = str(exc)
            self.recorder.event("error", error=type(exc).__name__, detail=str(exc))

        # Outside the except blocks so the motors are stopped and the manifest
        # written on every path, including a failure inside shutdown itself.
        return self._shutdown()

    def _shutdown(self):
        # Order matters: motors first, always, whatever else fails.
        try:
            self.actuator.stop(sleep=self._sleep)
            self.recorder.event("motors_stopped")
        except Exception as exc:                       # noqa: BLE001
            self.recorder.warn(f"stop command failed: {exc}")

        for source in (self.loadcell, self.mavlink):
            try:
                source.stop()
            except Exception:                          # noqa: BLE001
                pass

        self.supervisor.restore_signal_handlers()
        self.anchors.capture()
        return self._finalize()

    def _finalize(self):
        fits = {name: tracker.fit().as_dict() for name, tracker in self.clocks.items()}
        for name, fit in fits.items():
            if not fit.get("n"):
                # An absent stream is a supported configuration, not a fault --
                # most notably fc_esc, which needs a telemetry-capable ESC.
                continue
            for warning in fit.get("warnings", []):
                self.recorder.warn(f"clock fit: {warning}")

        return self.recorder.finalize(
            outcome=self.outcome,
            abort={"reason": self.abort_reason, "detail": self.abort_detail}
            if self.abort_reason else None,
            seed=self.seed,
            clock={"anchors": self.anchors.as_dict(), "device_fits": fits},
            tare={"offsets": self.tare_offsets, "n_samples": self.tare_n},
            achieved_rates=self._achieved_rates(),
            requested_rates={
                "fc_servo": self.plan["hardware"]["fc"]["servo_hz"],
                "fc_battery": self.plan["hardware"]["fc"]["battery_hz"],
                "fc_esc": self.plan["hardware"]["fc"]["esc_hz"],
                "loadcell": self.plan["hardware"]["loadcell"]["nominal_hz"],
            },
            source_stats={"loadcell": _clean(self.loadcell.stats()),
                          "mavlink": _clean(self.mavlink.stats())},
            motors_enabled=self.actuator.enabled,
            n_command_sends=self.actuator.n_sends,
        )

    def _achieved_rates(self):
        rates = dict(self.mavlink.stats().get("achieved_rates") or {})
        stats = self.loadcell.stats()
        received = stats.get("received") or 0
        span = self._loadcell_span()
        rates["loadcell"] = round(received / span, 1) if span > 0 else 0.0
        return rates

    def _loadcell_span(self):
        pairs = self.anchors.pairs
        return (pairs[-1][0] - pairs[0][0]) if len(pairs) >= 2 else 0.0

    # --- setup --------------------------------------------------------------
    def _preflight(self):
        armed = self.mavlink.armed
        if armed is True:
            raise Abort("armed", "the vehicle is armed; PX4 refuses actuator tests "
                                 "while armed -- disarm and retry")
        ok, message = self.actuator.preflight()
        self.recorder.event("preflight", ok=ok, message=message)
        if not ok:
            raise Abort("preflight_failed", message)

    def _await_first_force_sample(self):
        """A run that cannot see force must not command a motor."""
        deadline = now() + FIRST_SAMPLE_TIMEOUT_S
        while now() < deadline:
            samples = self.loadcell.drain()
            if samples:
                self._ingest(samples, seg_id=-1, record=False)
                return
            if self.loadcell.error:
                raise Abort("loadcell_error", self.loadcell.error)
            self._sleep(0.02)
        raise Abort("loadcell_silent",
                    f"no force sample within {FIRST_SAMPLE_TIMEOUT_S:.0f}s")

    # --- the segment loop ---------------------------------------------------
    def _execute(self, segment):
        self.actuator.set_target(segment.a_us, segment.b_us)
        t_start = now()
        controller = DwellController(segment, self.plan["dwell"], t_start)
        motors_live = segment.kind not in (TARE, WARMUP)
        tare_samples = []
        n_force = 0

        self.recorder.event("segment_start", t_mono=t_start, seg_id=segment.seg_id,
                            seg_kind=segment.kind, a_us=segment.a_us,
                            b_us=segment.b_us, sweep=segment.sweep)

        while True:
            t_mono = now()
            self.actuator.tick(t_mono)

            force = self.loadcell.drain()
            n_force += len(force)
            if segment.kind == TARE:
                tare_samples.extend(force)
            self._ingest(force, segment.seg_id, record=segment.record,
                         controller=controller)
            self._ingest(self.mavlink.drain(), segment.seg_id, record=segment.record)

            # Checked before the dwell test: a limit breach must not wait for a
            # segment that has decided to keep going.
            self.supervisor.check(t_mono,
                                  loadcell_stats=self.loadcell.stats(),
                                  fc_stats=self.mavlink.stats(),
                                  motors_live=motors_live)

            finished, reason = controller.done(t_mono)
            if finished:
                break
            self._status(segment, t_mono - t_start)
            self._sleep(LOOP_SLEEP_S)

        t_end = now()
        if segment.kind == TARE:
            self._apply_tare(tare_samples)

        summary = controller.summary()
        self.recorder.segment_record(
            segment, t_start, t_end, n_loadcell=n_force, dwell_reason=reason,
            thrust_mean_n=summary["thrust_mean_n"], thrust_sem_n=summary["thrust_sem_n"])
        self.recorder.event("segment_end", t_mono=t_end, seg_id=segment.seg_id,
                            actual_dwell_s=round(t_end - t_start, 4), reason=reason,
                            **{k: v for k, v in summary.items() if v is not None})
        # fsync between segments, never during one: a few milliseconds of I/O
        # must not land inside a step that is being measured.
        self.recorder.flush(sync=segment.kind == STEP)

    def _ingest(self, samples, seg_id, record=True, controller=None):
        """Route samples to disk, the clock trackers, the supervisor and the dwell."""
        if not samples:
            return
        if record:
            self.recorder.write_samples(samples, seg_id)

        for sample in samples:
            tracker = self.clocks.get(sample.stream)
            if tracker is not None and sample.dev_t is not None:
                tracker.add(sample.dev_t, sample.t_mono)

            if sample.stream == "loadcell":
                thrust = thrust_n(sample.fields)
                self.supervisor.note_force(thrust, torque_nm(sample.fields))
                if controller is not None:
                    controller.observe(sample.t_mono, thrust)
            elif sample.stream == "fc_battery":
                self.supervisor.note_battery(sample.fields.get("voltage_v"),
                                             sample.fields.get("current_a"))

    def _apply_tare(self, samples):
        """
        Install the zero measured at rest -- without destroying the raw signal.

        `gui.py` subtracted its zero before writing and kept no record of it, so a
        mis-tared run could not be recovered. Here the offsets go in the manifest
        and both raw and tared values are written.
        """
        try:
            offsets, n = self.loadcell.compute_tare(samples)
        except ValueError as exc:
            self.recorder.warn(f"tare skipped: {exc}")
            return
        self.loadcell.set_tare(offsets)
        self.tare_offsets = offsets
        self.tare_n = n
        self.recorder.event("tare", offsets=offsets, n_samples=n)

    def _status(self, segment, elapsed):
        if self.on_status is None:
            return
        self.on_status({
            "seg_index": self._seg_index,
            "n_segments": len(self.segments),
            "seg_id": segment.seg_id,
            "kind": segment.kind,
            "a_us": segment.a_us,
            "b_us": segment.b_us,
            "elapsed_s": elapsed,
            "voltage_v": self.supervisor.last_voltage_v,
            "current_a": self.supervisor.last_current_a,
            "thrust_n": self.supervisor._median(self.supervisor._recent_thrust),
        })


def _clean(stats):
    """Drop unserialisable odds and ends before the stats reach the manifest."""
    return {k: v for k, v in (stats or {}).items()
            if isinstance(v, (int, float, str, bool, dict, list, type(None)))}
