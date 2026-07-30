"""
Command line for the bench.

CLI first, GUI later and on top: a run defined by a checked-in plan file is
reproducible and can be exercised without hardware, whereas the old HTML form
left no record of how a run was configured. Every command here is safe to run
with the motors disconnected.
"""

import argparse
import sys

from tvcbench import SCHEMA_VERSION, config, selftest as st, sequence
from tvcbench.sources.loadcell import BAUD, LoadCellSource, NOMINAL_HZ

DEFAULT_CURRENT_MAP = "out/pwm_thrust_torque_map.csv"
# Separate from runs/, which holds the previous pipeline's derived output. This
# format is a clean break and mixing the two in one directory would only confuse.
DEFAULT_OUT_ROOT = "bench"

# Long enough for the clock fit to have real span (it wants >= 5 s) and for a
# marginal USB supply to show a brownout, short enough to actually run every time.
DEFAULT_SELFTEST_S = 20.0
PROBE_S = 1.5


def _list_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial is not installed:  pip install pyserial")
    return list(list_ports.comports())


def _probe_loadcell(port, seconds=PROBE_S):
    """Open a port briefly and report whether it is emitting load-cell samples."""
    src = LoadCellSource(port)
    try:
        src.start()
    except Exception as exc:                      # noqa: BLE001 - busy or absent
        return {"ok": False, "note": f"{type(exc).__name__}: {exc}"}
    try:
        samples = st.collect(src, seconds)
    finally:
        src.stop()
    if src.error and not samples:
        return {"ok": False, "note": src.error}
    if not samples:
        return {"ok": False, "note": "no status lines"}
    return {"ok": True, "note": f"{len(samples)} samples in {seconds:.1f}s"}


def _udev_rule(port):
    """A udev rule pinning this board to a stable name.

    Without one, `ttyACM0` and `ttyACM1` can swap between boots and a run
    silently records the wrong device -- or nothing at all.
    """
    if not (port.vid and port.pid):
        return None
    parts = [
        'SUBSYSTEM=="tty"',
        f'ATTRS{{idVendor}}=="{port.vid:04x}"',
        f'ATTRS{{idProduct}}=="{port.pid:04x}"',
    ]
    if port.serial_number:
        parts.append(f'ATTRS{{serial}}=="{port.serial_number}"')
    parts.append('SYMLINK+="tvc-loadcell"')
    return ", ".join(parts)


def cmd_devices(args):
    ports = _list_ports()
    if not ports:
        print("no serial ports found")
        return 1

    print(f"{'device':<22}{'vid:pid':<12}{'serial':<20}description")
    for p in ports:
        vidpid = f"{p.vid:04x}:{p.pid:04x}" if p.vid and p.pid else "-"
        print(f"{p.device:<22}{vidpid:<12}{(p.serial_number or '-'):<20}{p.description}")

    if not args.probe:
        print("\n--probe opens each port to identify the load cell "
              "(safe: reads only, sends nothing)")
        return 0

    print(f"\nprobing each port for {PROBE_S:.1f}s ...")
    found = []
    for p in ports:
        result = _probe_loadcell(p.device)
        mark = "LOAD CELL" if result["ok"] else "         "
        print(f"  {p.device:<22}{mark:<12}{result['note']}")
        if result["ok"]:
            found.append(p)

    for p in found:
        rule = _udev_rule(p)
        if rule:
            print(f"\nPin {p.device} to a stable name -- "
                  f"/etc/udev/rules.d/99-tvcbench.rules:\n  {rule}")
            print("  sudo udevadm control --reload-rules && sudo udevadm trigger")
    return 0 if found or not args.probe else 1


def cmd_selftest(args):
    baseline = st.load_baseline(args.baseline) if args.baseline else None

    src = LoadCellSource(args.port, baud=args.baud)
    try:
        src.start()
    except Exception as exc:                      # noqa: BLE001
        sys.exit(f"cannot open {args.port}: {exc}")

    print(f"reading {args.port} for {args.seconds:.0f}s -- "
          f"keep the motors stopped and the bench still")

    def progress(remaining, n):
        print(f"\r  {remaining:5.1f}s left, {n} samples", end="", flush=True)

    try:
        samples = st.collect(src, args.seconds, progress=progress)
    finally:
        src.stop()
    print("\r" + " " * 40 + "\r", end="")

    report = st.analyse(samples, src, args.seconds,
                        nominal_hz=args.nominal_hz, baseline=baseline)
    print(st.format_report(report, port=args.port))

    if args.save_baseline:
        st.save_baseline(report, args.save_baseline)
        print(f"\nbaseline written to {args.save_baseline}")
        print("Record one on the bench laptop, then compare the Pi against it with "
              "--baseline: a raised floor is the ground-loop signature.")

    return {st.PASS: 0, st.WARN: 0, st.FAIL: 1}[st.worst_verdict(report)]


def _fmt_hms(seconds):
    seconds = int(round(seconds))
    return f"{seconds // 60}m{seconds % 60:02d}s"


def cmd_plan_show(args):
    """Expand a plan and cost it out, before any hardware is involved."""
    import os

    try:
        plan = config.load(args.plan)
    except config.PlanError as exc:
        sys.exit(f"{args.plan} is not a valid plan:\n{exc}")
    except FileNotFoundError:
        sys.exit(f"no such plan file: {args.plan}")
    except ValueError as exc:                 # malformed JSON/YAML
        sys.exit(f"{args.plan} could not be parsed: {exc}")

    segments, seed = sequence.expand(plan, seed=args.seed)
    lo, nominal, hi = sequence.duration_s(segments)
    counts = sequence.summarise(segments)

    print(f"PLAN  {args.plan}")
    print(f"  grid              A {plan.a_values[0]}-{plan.a_values[-1]} "
          f"({len(plan.a_values)}) x B {plan.b_values[0]}-{plan.b_values[-1]} "
          f"({len(plan.b_values)}) = {plan.n_grid_points} points")
    print(f"  repeats           {plan['repeats']}")
    print(f"  order             {plan['order']['mode']}  (seed {seed})")
    dwell = plan["dwell"]
    detail = (f"{dwell['fixed_s']:g}s" if dwell["mode"] == "fixed"
              else f"{dwell['min_s']:g}-{dwell['max_s']:g}s adaptive")
    print(f"  dwell             {dwell['mode']}  {detail}")
    ref = plan["reference"]
    print(f"  reference         "
          + (f"A{ref['a']}/B{ref['b']} every {ref['every_n_steps']} steps"
             if ref["every_n_steps"] else "disabled"))

    print(f"\n  {'segments':<18}{len(segments)}")
    for kind in sorted(counts):
        print(f"  {'  ' + kind:<18}{counts[kind]}")

    if lo == hi:
        print(f"\n  {'duration':<18}{_fmt_hms(nominal)}")
    else:
        print(f"\n  {'duration':<18}{_fmt_hms(nominal)} nominal "
              f"({_fmt_hms(lo)} - {_fmt_hms(hi)} under adaptive dwell)")

    map_path = args.current_map or DEFAULT_CURRENT_MAP
    table = (sequence.current_map_from_csv(map_path)
             if os.path.exists(map_path) else None)
    mah = sequence.estimate_charge_mah(segments, table)
    if mah is None:
        print(f"  {'charge':<18}unknown -- no measured current map at {map_path}")
    else:
        worst = sequence.estimate_charge_mah(segments, table, use="max_dwell_s")
        extra = f" (up to {worst:.0f} mAh)" if worst and worst > mah * 1.01 else ""
        print(f"  {'charge':<18}~{mah:.0f} mAh{extra}   "
              f"estimated from {map_path}")

    if args.segments:
        print(f"\n  {'id':>5} {'kind':<13}{'A':>6}{'B':>6}{'dwell':>9}  sweep")
        for s in segments:
            print(f"  {s.seg_id:>5} {s.kind:<13}{s.a_us:>6}{s.b_us:>6}"
                  f"{s.dwell_s:>8.2f}s{s.sweep:>7}")
    return 0


def _build_sources(plan, args):
    """
    Assemble the link, actuator and sources for a run.

    Simulated and real paths produce the same objects, so the runner, recorder
    and supervisor below are identical either way -- which is what makes a
    rehearsal worth anything.
    """
    from tvcbench.actuator import Actuator

    fc = plan["hardware"]["fc"]
    lc = plan["hardware"]["loadcell"]
    motors = not args.no_motor

    if args.sim:
        from tvcbench.sources.sim import (SimBench, SimLink, SimLoadCellSource,
                                          SimMavlinkSource)
        bench = SimBench(seed=args.seed or 0, enabled=motors)
        link = SimLink(bench)
        loadcell = SimLoadCellSource(bench, hz=lc["nominal_hz"])
        mavlink = SimMavlinkSource(bench)
    else:
        from tvcbench.sources.loadcell import LoadCellSource
        from tvcbench.sources.mavlink import MavlinkLink, MavlinkSource

        link = MavlinkLink(fc["device"], fc["baud"])
        print(f"connecting to {fc['device']} at {fc['baud']} ...")
        link.connect()
        link.request_rates(fc["servo_hz"], fc["battery_hz"], fc["esc_hz"])
        mavlink = MavlinkSource(link)
        loadcell = LoadCellSource(lc["device"], baud=lc["baud"])

    actuator = Actuator(link, timeout_s=fc["timeout_s"], resend_hz=fc["resend_hz"],
                        enabled=motors)
    return link, actuator, loadcell, mavlink


def cmd_run(args):
    from tvcbench.recorder import Recorder, make_run_id
    from tvcbench.runner import Runner
    from tvcbench.supervisor import Supervisor

    try:
        plan = config.load(args.plan)
    except config.PlanError as exc:
        sys.exit(f"{args.plan} is not a valid plan:\n{exc}")
    except FileNotFoundError:
        sys.exit(f"no such plan file: {args.plan}")

    segments, seed = sequence.expand(plan, seed=args.seed)
    _, nominal, _ = sequence.duration_s(segments)
    print(f"{len(segments)} segments, about {_fmt_hms(nominal)}"
          f"{'  [SIMULATED]' if args.sim else ''}"
          f"{'  [--no-motor: commands suppressed]' if args.no_motor else ''}")

    try:
        link, actuator, loadcell, mavlink = _build_sources(plan, args)
    except Exception as exc:                       # noqa: BLE001
        sys.exit(f"cannot start: {exc}")

    run_id = make_run_id()
    streams = dict(loadcell.streams)
    streams.update(mavlink.streams)
    recorder = Recorder(args.out, run_id, streams, plan=plan)
    supervisor = Supervisor(plan["limits"], stop_file=args.stop_file,
                            on_event=lambda kind, **f: recorder.event(kind, **f))

    runner = Runner(plan, link, actuator, loadcell, mavlink, recorder, supervisor,
                    seed=seed)
    # Counted from the segments the loop executes: the post-stop window is not
    # one of them, and a progress line that never reaches its own total reads
    # like a run that stopped short.
    printer = _StatusPrinter(len(runner.live_segments)) if not args.quiet else None
    runner.on_status = printer.update if printer else None

    loadcell.start()
    mavlink.start()
    try:
        manifest = runner.run()
    finally:
        if printer:
            printer.clear()
        link.close()

    _print_outcome(manifest, recorder, plan)
    return 0 if manifest.get("outcome") == "completed" else 1


class _StatusPrinter:
    """Single-line bench progress. Rate-limited so it cannot slow the run loop."""

    MIN_INTERVAL_S = 0.2

    def __init__(self, n_segments):
        self.n_segments = n_segments
        self._last = 0.0

    def update(self, status):
        from tvcbench.clock import now

        t = now()
        if t - self._last < self.MIN_INTERVAL_S:
            return
        self._last = t

        def num(value, fmt):
            return format(value, fmt) if value is not None else "--"

        print(f"\r  [{status['seg_index'] + 1:>4}/{self.n_segments}] "
              f"{status['kind']:<12} A{status['a_us']} B{status['b_us']} "
              f"{status['elapsed_s']:4.1f}s  "
              f"{num(status['thrust_n'], '5.2f')} N  "
              f"{num(status['voltage_v'], '5.2f')} V  "
              f"{num(status['current_a'], '5.1f')} A   ",
              end="", flush=True)

    def clear(self):
        print("\r" + " " * 100 + "\r", end="")


def _print_outcome(manifest, recorder, plan):
    outcome = manifest.get("outcome")
    print(f"{outcome.upper()}  ->  {recorder.final_dir}")

    abort = manifest.get("abort")
    if abort and abort.get("reason"):
        print(f"  reason            {abort['reason']}: {abort.get('detail', '')}")

    rows = manifest.get("streams", {})
    print(f"  {'rows':<18}" + "  ".join(f"{name} {info['rows']}"
                                        for name, info in sorted(rows.items())))

    achieved = manifest.get("achieved_rates") or {}
    requested = manifest.get("requested_rates") or {}
    parts = []
    for name in sorted(achieved):
        want = requested.get(name)
        parts.append(f"{name} {achieved[name]:g}"
                     + (f"/{want:g}" if want else "") + " Hz")
    if parts:
        print(f"  {'rates':<18}" + "  ".join(parts))

    for name, fit in (manifest.get("clock", {}).get("device_fits") or {}).items():
        if fit.get("n"):
            print(f"  {'clock ' + name:<18}{fit['ppm']:+.1f} ppm, "
                  f"batch spread {fit['floor_width_s'] * 1e3:.1f} ms, n={fit['n']}")

    tare = manifest.get("tare") or {}
    if tare.get("offsets"):
        print(f"  {'tare':<18}Fz {tare['offsets']['Fz']:+.4f} N "
              f"from {tare['n_samples']} samples")

    post = manifest.get("post_stop") or {}
    if post.get("zero_thrust_n") is not None:
        # The zero measured again with the motors stopped: how far the tare moved
        # over the run, and therefore how far every step in it is out.
        volts = (f", pack recovered to {post['voltage_v']:.2f} V"
                 if post.get("voltage_v") is not None else "")
        print(f"  {'zero after run':<18}{post['zero_thrust_n']:+.3f} N "
              f"over {post['seconds']:.1f}s at rest{volts}")

    for warning in manifest.get("warnings", [])[:8]:
        print(f"  !                 {warning}")


def build_parser():
    ap = argparse.ArgumentParser(
        prog="tvcbench",
        description="Data acquisition for the coaxial thrust bench.")
    ap.add_argument("--version", action="version",
                    version=f"tvcbench schema v{SCHEMA_VERSION}")
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("devices", help="list serial ports and identify the load cell")
    p.add_argument("--probe", action="store_true",
                   help="open each port to see which one is the load cell")
    p.set_defaults(func=cmd_devices)

    p = sub.add_parser(
        "selftest",
        help="check the load-cell link: rate, dropouts, clock fit, noise floor",
        description="Run with the motors stopped. Answers the three hardware "
                    "risks of hosting the load cell on the Pi: USB power "
                    "(counter resets), ground loop (noise floor vs a baseline) "
                    "and link integrity (dropouts and rate).")
    p.add_argument("--port", default="/dev/tvc-loadcell", help="(default: %(default)s)")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--seconds", type=float, default=DEFAULT_SELFTEST_S)
    p.add_argument("--nominal-hz", type=float, default=NOMINAL_HZ)
    p.add_argument("--baseline", help="compare the noise floor against this file")
    p.add_argument("--save-baseline", help="write this run's report for later comparison")
    p.set_defaults(func=cmd_selftest)

    p = sub.add_parser("plan", help="expand and cost out a run plan")
    plan_sub = p.add_subparsers(dest="plan_command", required=True)
    q = plan_sub.add_parser("show", help="print the sequence, duration and battery draw")
    q.add_argument("plan", help="path to a .yaml or .json plan file")
    q.add_argument("--segments", action="store_true", help="list every segment")
    q.add_argument("--seed", type=int, help="override the plan's ordering seed")
    q.add_argument("--current-map", help=f"measured map for the battery estimate "
                                         f"(default: {DEFAULT_CURRENT_MAP})")
    q.set_defaults(func=cmd_plan_show)

    p = sub.add_parser(
        "run", help="execute a plan and record it",
        description="Records to <out>/<run_id>/. Motors stop on any exit path: "
                    "actuator-test commands expire, so ceasing to send is itself "
                    "the stop.")
    p.add_argument("plan")
    p.add_argument("--out", default=DEFAULT_OUT_ROOT,
                   help="output root (default: %(default)s)")
    p.add_argument("--no-motor", action="store_true",
                   help="run the full sequence, timing and recording with every "
                        "command suppressed -- a dress rehearsal with the battery out")
    p.add_argument("--sim", action="store_true",
                   help="use simulated hardware; output is NOT data")
    p.add_argument("--seed", type=int, help="override the plan's ordering seed")
    p.add_argument("--stop-file", help="abort the run when this path appears")
    p.add_argument("--quiet", action="store_true", help="no progress line")
    p.set_defaults(func=cmd_run)

    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
