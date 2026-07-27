"""
Actuator and MAVLink source tests.

The command path is the one place in this package where a bug spins a propeller,
so the safety-relevant behaviour is pinned down explicitly: the expiry/resend
invariant, the abort path, and that `--no-motor` really does suppress every send
while leaving the rest of the machinery running.
"""

import pytest

from tvcbench.actuator import (
    MOTOR_A_FUNC,
    MOTOR_B_FUNC,
    PWM_MAX_US,
    PWM_MIN_US,
    STOP_REPEATS,
    Actuator,
    norm_to_us,
    ramp_profile,
    read_battery,
    us_to_norm,
    validate_resend,
)
from tvcbench.sources.mavlink import MavlinkLink, MavlinkSource


class FakeLink:
    """Records every command instead of sending it."""

    def __init__(self, ack=None):
        self.sent = []
        self.ack = ack

    def send_actuator_test(self, func, value, timeout_s):
        self.sent.append((func, value, timeout_s))

    def recv_match(self, **_kw):
        return self.ack


class Ack:
    def __init__(self, command=310, result=0):
        self.command, self.result = command, result


class TestNormalisation:
    @pytest.mark.parametrize("us,norm", [(1000, 0.0), (1500, 0.5), (2000, 1.0)])
    def test_linear_across_the_configured_range(self, us, norm):
        assert us_to_norm(us) == pytest.approx(norm)

    def test_clamps_out_of_range_commands(self):
        # A bad plan must not be able to overdrive the ESC.
        assert us_to_norm(900) == 0.0
        assert us_to_norm(2500) == 1.0

    def test_roundtrips(self):
        for us in (1000, 1234, 1750, 2000):
            assert norm_to_us(us_to_norm(us)) == pytest.approx(us)


class TestResendInvariant:
    def test_rejects_a_timeout_shorter_than_the_resend_interval(self):
        """Commands would lapse between sends -- a partial dropout mid-step."""
        with pytest.raises(ValueError, match="lapse"):
            validate_resend(timeout_s=0.1, resend_hz=5.0)

    def test_accepts_the_shipped_defaults(self):
        validate_resend(timeout_s=1.0, resend_hz=5.0)

    def test_rejects_nonpositive_rate(self):
        with pytest.raises(ValueError):
            validate_resend(timeout_s=1.0, resend_hz=0.0)

    def test_constructor_enforces_it(self):
        with pytest.raises(ValueError):
            Actuator(FakeLink(), timeout_s=0.1, resend_hz=5.0)


class TestHoldAndResend:
    def test_sends_both_motors_on_tick(self):
        link = FakeLink()
        act = Actuator(link, resend_hz=5.0)
        act.set_target(1400, 1700)
        assert act.tick(t_mono=100.0) is True

        assert link.sent == [
            (MOTOR_A_FUNC, us_to_norm(1400), 1.0),
            (MOTOR_B_FUNC, us_to_norm(1700), 1.0),
        ]

    def test_does_not_resend_before_the_interval(self):
        link = FakeLink()
        act = Actuator(link, resend_hz=5.0)
        act.set_target(1400, 1400)
        act.tick(t_mono=100.0)
        assert act.tick(t_mono=100.1) is False      # interval is 0.2 s
        assert act.tick(t_mono=100.2) is True

    def test_a_new_target_is_sent_immediately(self):
        """Waiting out the interval would blur the step edge the analysis segments on."""
        link = FakeLink()
        act = Actuator(link, resend_hz=5.0)
        act.set_target(1400, 1400)
        act.tick(t_mono=100.0)
        act.set_target(1500, 1500)
        assert act.tick(t_mono=100.01) is True

    def test_reports_each_send(self):
        seen = []
        act = Actuator(FakeLink(), on_send=lambda t, a, b: seen.append((t, a, b)))
        act.set_target(1300, 1600)
        act.tick(t_mono=5.0)
        assert seen == [(5.0, 1300, 1600)]

    def test_rounds_targets_to_whole_microseconds(self):
        act = Actuator(FakeLink())
        act.set_target(1400.6, 1699.4)
        assert (act.a_us, act.b_us) == (1401, 1699)


class TestStop:
    def test_sends_minimum_repeatedly(self):
        link = FakeLink()
        act = Actuator(link)
        act.set_target(1800, 1800)
        act.stop(sleep=lambda _s: None)

        assert len(link.sent) == STOP_REPEATS * 2
        assert all(value == 0.0 for _func, value, _t in link.sent)
        assert (act.a_us, act.b_us) == (PWM_MIN_US, PWM_MIN_US)

    def test_ceasing_to_tick_is_itself_a_stop(self):
        """
        The base safety layer: commands expire, so a dead process stops the motors.

        Asserted as a property of the configuration rather than of any code path,
        because the guarantee comes from PX4's timeout, not from us.
        """
        act = Actuator(FakeLink(), timeout_s=1.0, resend_hz=5.0)
        assert act.timeout_s > act.interval_s
        assert act.timeout_s < 2.0            # and the motors stop promptly


class TestNoMotorMode:
    def test_suppresses_every_send_but_runs_everything_else(self):
        link = FakeLink()
        act = Actuator(link, enabled=False)
        act.set_target(1800, 1900)
        assert act.tick(t_mono=1.0) is True    # the sequence still advances
        act.stop(sleep=lambda _s: None)

        assert link.sent == []                 # but nothing reached the hardware
        assert act.n_sends > 0                 # and the run still accounts for it

    def test_preflight_is_skipped(self):
        ok, msg = Actuator(FakeLink(), enabled=False).preflight()
        assert ok and "no-motor" in msg


class TestPreflight:
    def test_accepted(self):
        ok, msg = Actuator(FakeLink(ack=Ack(result=0))).preflight()
        assert ok and "accepted" in msg

    def test_rejection_names_the_usual_cause(self):
        ok, msg = Actuator(FakeLink(ack=Ack(result=1))).preflight()
        assert not ok and "armed" in msg

    def test_other_rejection_points_at_the_motor_assignment(self):
        ok, msg = Actuator(FakeLink(ack=Ack(result=4))).preflight()
        assert not ok and "QGC" in msg

    def test_missing_ack_is_not_fatal(self):
        ok, msg = Actuator(FakeLink(ack=None)).preflight()
        assert ok and "no COMMAND_ACK" in msg


class TestRampProfile:
    def test_ends_exactly_on_target(self):
        steps = ramp_profile(1800, 1800, PWM_MIN_US, PWM_MIN_US, steps=8)
        assert len(steps) == 8
        assert steps[-1] == (PWM_MIN_US, PWM_MIN_US)

    def test_is_monotonic_and_excludes_the_start(self):
        steps = ramp_profile(1000, 1000, 2000, 2000, steps=4)
        assert steps == [(1250, 1250), (1500, 1500), (1750, 1750), (2000, 2000)]

    def test_moves_both_axes_independently(self):
        assert ramp_profile(1000, 2000, 2000, 1000, steps=2) == [(1500, 1500),
                                                                 (2000, 1000)]

    def test_rejects_zero_steps(self):
        with pytest.raises(ValueError):
            ramp_profile(1000, 1000, 2000, 2000, steps=0)


class TestReadBattery:
    """QGC's exact algorithm: stop at the first invalid cell, do not skip it."""

    def test_sums_contiguous_valid_cells(self):
        msg = type("M", (), {"voltages": [3700, 3710, 3690] + [65535] * 7,
                             "voltages_ext": [0] * 4,
                             "current_battery": 1234})()
        v, a = read_battery(msg)
        assert v == pytest.approx(11.1)
        assert a == pytest.approx(12.34)          # centiamps -> amps

    def test_stops_at_the_first_sentinel_rather_than_skipping(self):
        msg = type("M", (), {"voltages": [3700, 65535, 3690] + [65535] * 7,
                             "voltages_ext": [0] * 4,
                             "current_battery": -1})()
        v, a = read_battery(msg)
        assert v == pytest.approx(3.7)            # the 3690 after the sentinel is ignored
        assert a is None                          # -1 means not measured

    def test_extends_into_voltages_ext(self):
        msg = type("M", (), {"voltages": [3700] * 10,
                             "voltages_ext": [3700, 0, 0, 0],
                             "current_battery": 0})()
        v, _ = read_battery(msg)
        assert v == pytest.approx(40.7)           # 11 cells

    def test_missing_voltages_ext_is_tolerated(self):
        msg = type("M", (), {"voltages": [3700, 65535] + [65535] * 8,
                             "current_battery": 0})()
        v, _ = read_battery(msg)
        assert v == pytest.approx(3.7)


# --- MAVLink source ---------------------------------------------------------

class Msg:
    def __init__(self, kind, **fields):
        self._kind = kind
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind


class FakeConn:
    def __init__(self, messages):
        self.messages = list(messages)
        self.target_system = 1
        self.target_component = 1
        self.sent = []
        self.closed = False
        self.mav = self

    def command_long_send(self, *args):
        self.sent.append(args)

    def wait_heartbeat(self, timeout=None):
        return Msg("HEARTBEAT", base_mode=0)

    def recv_match(self, blocking=False, timeout=None, **_kw):
        return self.messages.pop(0) if self.messages else None

    def close(self):
        self.closed = True


def source_over(messages):
    link = MavlinkLink(connection_factory=lambda: FakeConn(messages))
    link.connect()
    src = MavlinkSource(link)
    src._open()
    src._read_once()
    return src, src.drain()


class TestMavlinkSource:
    def test_servo_and_battery_land_in_separate_streams(self):
        """The core fix: no join, no carry-forward, each at its own rate."""
        _, samples = source_over([
            Msg("SERVO_OUTPUT_RAW", time_usec=1000,
                **{f"servo{i}_raw": 1000 + i for i in range(1, 9)}),
            Msg("BATTERY_STATUS", id=0, voltages=[3700] * 3 + [65535] * 7,
                voltages_ext=[0] * 4, current_battery=500, battery_remaining=88),
        ])
        by_stream = {s.stream: s for s in samples}
        assert set(by_stream) == {"fc_servo", "fc_battery"}
        assert by_stream["fc_servo"].fields["servo2_raw"] == 1002
        assert by_stream["fc_servo"].dev_t == 1000
        assert by_stream["fc_battery"].fields["voltage_v"] == pytest.approx(11.1)
        assert by_stream["fc_battery"].fields["remaining_pct"] == 88
        # No voltage was smeared onto the servo row.
        assert "voltage_v" not in by_stream["fc_servo"].fields

    def test_secondary_battery_is_ignored(self):
        _, samples = source_over([
            Msg("BATTERY_STATUS", id=1, voltages=[3700] * 10,
                voltages_ext=[0] * 4, current_battery=0, battery_remaining=50),
        ])
        assert samples == []

    def test_unknown_remaining_becomes_none(self):
        _, samples = source_over([
            Msg("BATTERY_STATUS", id=0, voltages=[3700] + [65535] * 9,
                voltages_ext=[0] * 4, current_battery=-1, battery_remaining=-1),
        ])
        assert samples[0].fields["remaining_pct"] is None

    def test_esc_telemetry_fans_out_to_slots(self):
        _, samples = source_over([
            Msg("ESC_STATUS", time_usec=7, index=0, rpm=[100, 200, 0, 0],
                voltage=[11.1] * 4, current=[3.0] * 4),
        ])
        f = samples[0].fields
        assert samples[0].stream == "fc_esc"
        assert f["esc1_rpm"] == 100 and f["esc2_rpm"] == 200
        assert f["esc1_voltage"] == pytest.approx(11.1)

    def test_absent_esc_telemetry_yields_no_rows(self):
        """No telemetry-capable ESC is a supported configuration, not an error."""
        src, samples = source_over([Msg("HEARTBEAT", base_mode=0)])
        assert samples == []
        assert src.achieved_rates()["fc_esc"] == 0.0

    def test_tracks_armed_state_from_heartbeat(self):
        src, _ = source_over([Msg("HEARTBEAT", base_mode=128)])
        assert src.armed is True
        assert src.last_heartbeat_t_mono is not None

        src, _ = source_over([Msg("HEARTBEAT", base_mode=0)])
        assert src.armed is False

    def test_counts_command_acks(self):
        src, _ = source_over([
            Msg("COMMAND_ACK", command=310, result=0),
            Msg("COMMAND_ACK", command=310, result=1),
            Msg("COMMAND_ACK", command=511, result=1),     # not ours; ignored
        ])
        assert (src.n_ack_accepted, src.n_ack_rejected) == (1, 1)

    def test_columns_match_emitted_fields(self):
        _, samples = source_over([
            Msg("SERVO_OUTPUT_RAW", time_usec=1,
                **{f"servo{i}_raw": 1000 for i in range(1, 9)}),
            Msg("BATTERY_STATUS", id=0, voltages=[3700] + [65535] * 9,
                voltages_ext=[0] * 4, current_battery=0, battery_remaining=1),
            Msg("ESC_STATUS", time_usec=1, index=0, rpm=[0] * 4,
                voltage=[0] * 4, current=[0] * 4),
        ])
        for s in samples:
            assert set(MavlinkSource.streams[s.stream]) == set(s.fields)

    def test_drain_is_bounded_per_pass(self):
        """The loop must return often enough to notice a stop request."""
        many = [Msg("SERVO_OUTPUT_RAW", time_usec=i,
                    **{f"servo{k}_raw": 1000 for k in range(1, 9)})
                for i in range(200)]
        _, samples = source_over(many)
        assert len(samples) == 40          # MAX_DRAIN_PER_LOOP

    def test_requires_a_connected_link(self):
        src = MavlinkSource(MavlinkLink())
        with pytest.raises(RuntimeError, match="not connected"):
            src._open()


class TestMavlinkLink:
    def test_connect_raises_a_useful_message_without_a_heartbeat(self):
        class Silent(FakeConn):
            def wait_heartbeat(self, timeout=None):
                return None

        link = MavlinkLink(connection_factory=lambda: Silent([]))
        with pytest.raises(RuntimeError, match="TX/RX"):
            link.connect()

    def test_actuator_test_maps_to_the_right_command_parameters(self):
        conn = FakeConn([])
        link = MavlinkLink(connection_factory=lambda: conn)
        link.connect()
        link.send_actuator_test(func=2, value=0.4, timeout_s=1.0)

        _sys, _comp, command, _conf, p1, p2, _p3, _p4, p5, _p6, _p7 = conn.sent[0]
        assert command == 310
        assert (p1, p2, p5) == (0.4, 1.0, 2.0)

    def test_rate_request_converts_hz_to_microseconds(self):
        conn = FakeConn([])
        link = MavlinkLink(connection_factory=lambda: conn)
        link.connect()
        link.request_rates(servo_hz=50, battery_hz=20, esc_hz=0)

        intervals = [(a[4], a[5]) for a in conn.sent if a[2] == 511]
        assert (36, 20000) in intervals        # SERVO_OUTPUT_RAW at 50 Hz
        assert (147, 50000) in intervals       # BATTERY_STATUS at 20 Hz
        assert not [i for i in intervals if i[0] == 291]   # esc_hz=0 -> not requested

    def test_close_is_idempotent(self):
        conn = FakeConn([])
        link = MavlinkLink(connection_factory=lambda: conn)
        link.connect()
        link.close()
        link.close()
        assert conn.closed
