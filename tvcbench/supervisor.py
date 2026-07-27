"""
The abort authority.

One object decides when a run stops, and it is not the user interface. The old
logger's watchdog fired when the browser stopped polling, which is a sensible
rule for a browser-driven run and a meaningless one for anything headless -- and
it meant the safety system's health depended on a GUI's event loop.

Every limit here returns a distinct reason string, because the reasons call for
different responses: a low pack means charge it, a thrust spike means inspect the
propeller before doing anything else, and a load-cell dropout means the run was
already worthless from the moment it happened.

The base safety layer is not in this file. `MAV_CMD_ACTUATOR_TEST` expires, so
the motors stop whenever commands stop arriving -- including when this process
dies. Everything here is the layer above that.
"""

import os
import signal

# Thrust and torque limits are checked against a short median rather than a
# single sample. Propeller vibration is roughly 0.7 N on this bench, so a raw
# sample can be a newton off; a median over a few samples rejects that without
# meaningfully delaying a real excursion.
GUARD_MEDIAN_N = 5


class Abort(Exception):
    """Raised inside the run loop to unwind to the stop path."""

    def __init__(self, reason, detail=""):
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


class Supervisor:
    """
    Watches the run and decides when it must stop.

    `check()` is called from the run loop many times a second and must stay
    cheap; it holds no locks and does no I/O.
    """

    def __init__(self, limits, stop_file=None, on_event=None):
        self.limits = dict(limits or {})
        self.stop_file = stop_file
        self.on_event = on_event

        self.stop_requested = False
        self.stop_source = None
        self._recent_thrust = []
        self._recent_torque = []
        self._installed = []

        self.last_voltage_v = None
        self.last_current_a = None

    # --- external stop signals ---------------------------------------------
    def request_stop(self, source="user"):
        if not self.stop_requested:
            self.stop_requested = True
            self.stop_source = source
            self._emit("stop_requested", source=source)

    def install_signal_handlers(self):
        """
        Turn SIGINT/SIGTERM into a graceful stop rather than an abrupt exit.

        The motors stop either way -- commands expire -- but a graceful stop also
        closes the files and writes the manifest, so an interrupted run is still
        a readable run.
        """
        def handler(signum, _frame):
            self.request_stop(source=signal.Signals(signum).name)

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                previous = signal.signal(sig, handler)
                self._installed.append((sig, previous))
            except (ValueError, OSError):
                # Not the main thread, or unsupported on this platform.
                pass

    def restore_signal_handlers(self):
        for sig, previous in self._installed:
            try:
                signal.signal(sig, previous)
            except (ValueError, OSError):
                pass
        self._installed.clear()

    # --- observations -------------------------------------------------------
    def note_battery(self, voltage_v, current_a):
        if voltage_v is not None:
            self.last_voltage_v = voltage_v
        if current_a is not None:
            self.last_current_a = current_a

    def note_force(self, thrust_n, torque_nm):
        if thrust_n is not None:
            self._recent_thrust.append(thrust_n)
            del self._recent_thrust[:-GUARD_MEDIAN_N]
        if torque_nm is not None:
            self._recent_torque.append(torque_nm)
            del self._recent_torque[:-GUARD_MEDIAN_N]

    @staticmethod
    def _median(values):
        if not values:
            return None
        ordered = sorted(values)
        return ordered[len(ordered) // 2]

    # --- the check ----------------------------------------------------------
    def check(self, t_mono, loadcell_stats=None, fc_stats=None, motors_live=True):
        """
        Raise `Abort` if the run must stop. Returns None otherwise.

        `motors_live` is False during setup and idle phases where some limits do
        not apply -- a thrust limit is meaningless before the rotors have spun,
        and a voltage limit should not trip on a pack that is merely being
        measured at rest.
        """
        if self.stop_requested:
            raise Abort("stopped", self.stop_source or "user")

        if self.stop_file and os.path.exists(self.stop_file):
            raise Abort("stopped", f"stop file {self.stop_file}")

        self._check_loadcell(t_mono, loadcell_stats)
        self._check_fc(t_mono, fc_stats)

        if not motors_live:
            return None

        self._check_battery()
        self._check_force()
        return None

    def _check_loadcell(self, t_mono, stats):
        if not stats:
            return
        if stats.get("error"):
            raise Abort("loadcell_error", stats["error"])

        window = self.limits.get("loadcell_dropout_s", 0)
        last = stats.get("last_t_mono")
        if not window:
            return
        if last is None:
            return          # nothing yet; the run loop handles the initial wait
        if t_mono - last > window:
            # Without force there is no measurement, and the thrust and torque
            # limits are blind. Continuing would only waste the battery.
            raise Abort("loadcell_dropout",
                        f"no force sample for {t_mono - last:.2f}s")

    def _check_fc(self, t_mono, stats):
        if not stats:
            return
        if stats.get("error"):
            raise Abort("fc_error", stats["error"])

        window = self.limits.get("heartbeat_timeout_s", 0)
        last = stats.get("last_heartbeat_t_mono")
        if window and last is not None and t_mono - last > window:
            raise Abort("fc_heartbeat_lost", f"no heartbeat for {t_mono - last:.2f}s")

    def _check_battery(self):
        floor = self.limits.get("min_voltage_v", 0)
        if floor and self.last_voltage_v is not None and self.last_voltage_v < floor:
            # Stopping here loses the run. Not stopping loses the pack.
            raise Abort("low_voltage", f"{self.last_voltage_v:.2f}V < {floor:.2f}V")

        ceiling = self.limits.get("max_current_a", 0)
        if ceiling and self.last_current_a is not None and self.last_current_a > ceiling:
            raise Abort("over_current", f"{self.last_current_a:.1f}A > {ceiling:.1f}A")

    def _check_force(self):
        max_thrust = self.limits.get("max_thrust_n", 0)
        thrust = self._median(self._recent_thrust)
        if max_thrust and thrust is not None and thrust > max_thrust:
            # New with the load cell on the Pi: catches a shed blade, a slipped
            # mount or the stand fouling something, none of which the old
            # command-only logger could see at all.
            raise Abort("over_thrust", f"{thrust:.1f}N > {max_thrust:.1f}N")

        max_torque = self.limits.get("max_torque_nm", 0)
        torque = self._median(self._recent_torque)
        if max_torque and torque is not None and abs(torque) > max_torque:
            raise Abort("over_torque", f"|{torque:.2f}|N*m > {max_torque:.2f}N*m")

    def _emit(self, kind, **fields):
        if self.on_event is not None:
            self.on_event(kind, **fields)
