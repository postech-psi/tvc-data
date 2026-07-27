"""
Command line for tvctools.

    python -m tvctools index              catalog every file -> runs_index.csv
    python -m tvctools build --dry-run    show the run grouping without writing
    python -m tvctools build              copy + merge into runs/
    python -m tvctools ulog <file.ulg>    standalone .ulg analysis
"""

import argparse
import json
import os
import shutil
import sys

from .discover import scan, write_index, summarize
from .schema import epoch_to_local_str
from .group import pair_runs, build_sessions, format_sessions
from .merge import (merge_run, write_merged, steady_state_by_phase,
                    estimate_file_lags)

OUT_DIR = "runs"      # organized per-run copies
ANALYSIS_DIR = "out"  # every generated table and plot lands here


def _out(root):
    """Generated output goes in one place, never scattered across the root."""
    d = os.path.join(root, ANALYSIS_DIR)
    os.makedirs(d, exist_ok=True)
    return d


def cmd_index(args):
    records = scan(args.root, verbose=args.verbose)
    csv_path, json_path = write_index(records, args.root)
    print(summarize(records))
    print("\nwrote %s\nwrote %s" % (csv_path, json_path))
    return 0


def cmd_build(args):
    from .schema import PWM_MAP, ULOG
    from .ulogtime import resolve_ulog_epochs

    records = scan(args.root)

    # Date the ulogs from the t_fc_us bridge before grouping -- their filenames
    # can be minutes wrong, which would attach them to the wrong session.
    if not args.skip_ulog_time:
        ulogs = [r for r in records if r["kind"] == ULOG]
        pms = [r for r in records if r["kind"] == PWM_MAP]
        if ulogs and pms:
            print("Dating %d ulogs via thrust_map t_fc_us..." % len(ulogs))
            resolved = resolve_ulog_epochs(ulogs, pms, args.root)
            for rec in ulogs:
                got = resolved.get(rec["rel_path"])
                if got:
                    rec["epoch_resolved"] = got
                    drift = got["start_epoch"] - (rec.get("start_epoch") or got["start_epoch"])
                    rec["start_epoch"] = got["start_epoch"]
                    rec["end_epoch"] = got["end_epoch"]
                    rec["start_local"] = epoch_to_local_str(got["start_epoch"])
                    dup = got.get("duplicate_of")
                    print("  %-32s -> %s  (filename off by %+.0f s, %d runs%s)"
                          % (rec["name"], rec["start_local"], -drift,
                             len(got.get("covers", [])),
                             ", DUPLICATE of " + dup if dup else ""))
                else:
                    rec["flags"] = list(rec.get("flags", [])) + ["undated"]
                    print("  %-32s -> could not date (no thrust_map from that boot)"
                          % rec["name"])

    runs = pair_runs(records)
    sessions = build_sessions(runs, records)

    print(summarize(records))
    print("\nPlanned layout under %s/:" % OUT_DIR)
    print(format_sessions(sessions))

    if args.dry_run:
        print("\n(dry run -- nothing written. Re-run without --dry-run to build.)")
        return 0

    out_root = os.path.join(args.root, OUT_DIR)
    os.makedirs(out_root, exist_ok=True)
    n_runs = n_merged = 0

    # One clock offset per load-cell file, shared by the runs that use it
    file_lags = estimate_file_lags(runs, args.root)
    _bat_cache = {}   # ulog rel_path -> battery/idle arrays, parsed once

    for session in sessions:
        sdir = os.path.join(out_root, session["name"])
        os.makedirs(sdir, exist_ok=True)

        # ulogs are referenced, never copied -- they are ~194 MB of binaries and
        # duplicating them would defeat the point of organizing.
        with open(os.path.join(sdir, "session.json"), "w", encoding="utf-8") as f:
            json.dump({
                "name": session["name"],
                "date": session.get("date"),
                "start_local": session["runs"][0]["start_local"],
                "n_runs": len(session["runs"]),
                "ulogs": [{"name": u["name"], "rel_path": u["rel_path"],
                           "size_mb": u["size_mb"], "start_local": u["start_local"],
                           "epoch_resolved": u.get("epoch_resolved")}
                          for u in session["ulogs"]],
                "runs": [r["name"] for r in session["runs"]],
            }, f, indent=2, ensure_ascii=False)

        for run in session["runs"]:
            rdir = os.path.join(sdir, run["name"])
            os.makedirs(rdir, exist_ok=True)
            n_runs += 1

            sources = {}
            for key, dest in (("pwm_map", "pwm.csv"), ("loadcell", "thrust.csv")):
                rec = run[key]
                if not rec:
                    continue
                src = os.path.join(args.root, rec["rel_path"])
                shutil.copy2(src, os.path.join(rdir, dest))  # originals untouched
                sources[key] = rec["rel_path"]

            rows, info = merge_run(run, args.root, file_lags=file_lags)
            noload = _noload_for_run(run, session, args.root, _bat_cache)
            steady = []
            if rows and run["pwm_map"] and run["loadcell"]:
                write_merged(rows, os.path.join(rdir, "merged.csv"))
                steady = steady_state_by_phase(rows)
                n_merged += 1

            with open(os.path.join(rdir, "run.json"), "w", encoding="utf-8") as f:
                json.dump({
                    "name": run["name"],
                    "session": session["name"],
                    "start_local": run["start_local"],
                    "duration_s": run["duration_s"],
                    "label": run["label"],
                    "flags": run["flags"],
                    "sources": sources,
                    "alignment": info,
                    "ulog": _ulog_for_run(run, session),
                    "voltage_noload_v": noload,
                    "steady_state": steady,
                }, f, indent=2, ensure_ascii=False, default=str)

    print("\nBuilt %d runs in %d sessions (%d merged) under %s/"
          % (n_runs, len(sessions), n_merged, OUT_DIR))
    print("Originals were copied, not moved -- nothing outside %s/ was changed."
          % OUT_DIR)
    return 0



def _ulog_for_run(run, session):
    """Which ulog covers this run, and where the run sits inside it.

    A single log spans a whole boot and usually holds several runs, so the run
    is located by its FC-time window rather than by the log as a whole. Skips
    logs already identified as truncated duplicates.
    """
    pm = run.get("pwm_map")
    if not pm:
        return None
    for u in session["ulogs"]:
        res = u.get("epoch_resolved")
        if not res or res.get("duplicate_of"):
            continue
        for c in res.get("covers", []):
            if c["pwm_map"] == pm["name"]:
                return {"name": u["name"], "rel_path": u["rel_path"],
                        "fc_start": c["fc_start"], "fc_end": c["fc_end"],
                        "matched_by": c["how"],
                        "fc_to_epoch_offset": res["offset"]}
    return None



def _noload_for_run(run, session, root, cache):
    """Pack voltage with the motor off, just before this run started.

    This is the state of charge. The voltage logged during a run is that minus
    an IR drop that itself varies with throttle, so it cannot be compared
    between runs.
    """
    from .battery import noload_from_pwm_map
    from .schema import load_any
    from .ulogsegment import load_session, segment, noload_windows

    # Preferred source: the Pi's own idle_pre/idle_post rows. When those exist
    # the state of charge needs no ulog at all -- one file, one clock.
    pm = run.get("pwm_map")
    if pm:
        _k, cols, _m = load_any(os.path.join(root, pm["rel_path"]))
        got = noload_from_pwm_map(cols)
        if got:
            return got

    # Otherwise the ulog, which is the only continuous record of the idle gaps
    # between runs. Segmenting it by motor activity gives the *whole* gap to
    # average over (98-545 s here) instead of a fixed guessed window, and the
    # result is self-consistent: one run's after-voltage matches the next run's
    # before-voltage.
    u = _ulog_for_run(run, session)
    if not u:
        return None
    rel = u["rel_path"]
    if rel not in cache:
        sess = load_session(os.path.join(root, rel))
        segs = segment(sess) if sess else []
        cache[rel] = (segs, noload_windows(sess, segs) if sess else {})
    segs, windows = cache[rel]
    if not segs:
        return None
    # Match this run to the ulog interval starting nearest its FC window
    best = min(segs, key=lambda r: abs(r["fc_start"] - u["fc_start"]))
    if abs(best["fc_start"] - u["fc_start"]) > 30.0:
        return None
    got = dict(windows.get(best["index"]) or {})
    if not got.get("v"):
        return None
    got["source"] = u["name"]
    got["ulog_run_index"] = best["index"]
    got["idle_before_s"] = best["idle_before_s"]
    return got


def cmd_organize(args):
    from .organize import (plan_moves, apply_moves, find_redundant_ulogs,
                           prune_empty_dirs)

    print("Inspecting ulogs for redundancy...")
    redundant = find_redundant_ulogs(args.root)
    for name, why in redundant.items():
        print("  %-32s %s" % (name, why))

    moves, skipped = plan_moves(args.root, set(redundant))
    if skipped:
        print("\nLeft in place:")
        for rel, why in skipped:
            print("  %-46s %s" % (rel[:46], why))
    if not moves:
        print("\nRaw data is already organized.")
        return 0

    print("\n%d files to move:" % len(moves))
    for src, dst in moves:
        print("  %-46s -> %s" % (src[:46], dst))

    if args.dry_run:
        print("\n(dry run -- nothing moved. Re-run without --dry-run to apply.)")
        return 0

    n = apply_moves(args.root, moves)
    gone = prune_empty_dirs(args.root, "motor test")
    print("\nMoved %d files." % n)
    for d in gone:
        print("  removed empty %s" % d)
    print("Tracked files moved with `git mv`, so history follows them.")
    return 0


def cmd_map(args):
    from .analyze import (build_map, build_sag, write_rows,
                          normalize_to_voltage, sag_exponent)
    from .plots import plot_map, plot_run_steps, plot_sag, plot_coax_grid

    runs_root = os.path.join(args.root, OUT_DIR)
    rows = build_map(runs_root)
    if not rows:
        print("No merged runs found. Run `python -m tvctools build` first.")
        return 1

    if args.normalize:
        k, src = sag_exponent(runs_root)
        rows, v_ref, k = normalize_to_voltage(rows, args.v_ref, k)
        print("Thrust normalized to %.2f V using thrust ~ V^%.2f%s" %
              (v_ref, k, " (measured from A%d_B%d)" % (src["a_cmd_us"], src["b_cmd_us"])
               if src else " (default)"))
        print("A sweep sags while it climbs, so raw points sit on a voltage "
              "gradient; thrust_N_raw keeps the uncorrected value.\n")

    map_csv = write_rows(rows, os.path.join(_out(args.root), "pwm_thrust_torque_map.csv"))
    print("%d steady-state points from %d runs"
          % (len(rows), len({r["run"] for r in rows})))
    print("  %-8s %-8s %9s %9s %11s %9s %9s"
          % ("a_us", "b_us", "thrust_N", "+/-SEM", "torque_Nm", "V", "run"))
    for r in rows:
        print("  %-8s %-8s %9.3f %9.3f %11s %9s %9s"
              % (r["a_cmd_us"], r["b_cmd_us"], r["thrust_N"], r["thrust_sem"],
                 r["torque_Nm"], r["voltage_v"], r["run"]))
    print("\nwrote %s" % map_csv)

    audit = build_sag(runs_root, include_rejected=True)
    rejected = [g for g in audit if g.get("rejected")]
    sag = [g for g in audit if not g.get("rejected")]
    if rejected and args.show_rejected:
        print("\nConstant-PWM groups rejected for sag analysis:")
        for g in rejected:
            print("  A%d_B%d: %s" % (g["a_cmd_us"], g["b_cmd_us"], g["rejected"]))
    elif rejected:
        print("\n%d constant-PWM groups rejected as confounded "
              "(--show-rejected for why)." % len(rejected))

    if sag:
        sag_csv = write_rows(sag, os.path.join(_out(args.root), "voltage_sag.csv"))
        print("\nThrust vs voltage at constant command (battery drain only):")
        for g in sag:
            print("  A%d_B%d over %d runs: %.2f-%.2f V -> %.2f-%.2f N"
                  % (g["a_cmd_us"], g["b_cmd_us"], g["n_runs"],
                     g["voltage_min"], g["voltage_max"],
                     g["thrust_min"], g["thrust_max"]))
            print("      dThrust/dV = %.3f N/V   thrust ~ V^%.2f   r = %.3f   "
                  "%.1f%% thrust lost"
                  % (g["dthrust_dV_N_per_V"], g["exponent_k"], g["r"],
                     g["pct_thrust_loss"]))
        print("\nwrote %s" % sag_csv)

    if not args.no_plots:
        p = plot_map(rows, os.path.join(_out(args.root), "pwm_thrust_torque_map.png"))
        if p:
            print("wrote %s" % p)
        p = plot_coax_grid(rows, os.path.join(_out(args.root), "coax_grid.png"))
        if p:
            print("wrote %s" % p)
        if sag:
            p = plot_sag(rows, sag, os.path.join(_out(args.root), "voltage_sag.png"))
            if p:
                print("wrote %s" % p)
        n = 0
        for run_dir in sorted({r["_dir"] for r in _run_dirs(runs_root)}):
            if plot_run_steps(run_dir):
                n += 1
        print("wrote %d per-run steps.png" % n)
    return 0


def _run_dirs(runs_root):
    from .analyze import load_runs
    return load_runs(runs_root)


def cmd_ulog(args):
    if args.segment:
        from .ulogsegment import describe
        for f in args.files:
            print(describe(f))
            print()
        return 0
    from .ulog import main as ulog_main
    argv = list(args.files)
    if args.outdir:
        argv += ["-o", args.outdir]
    return ulog_main(argv)


def build_parser():
    ap = argparse.ArgumentParser(prog="tvctools",
                                 description="Organize TVC test-stand data")
    ap.add_argument("--root", default=".", help="data root (default: cwd)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("index", help="catalog every data file")
    p.add_argument("-v", "--verbose", action="store_true")
    p.set_defaults(func=cmd_index)

    p = sub.add_parser("build", help="group into runs/sessions and merge")
    p.add_argument("--dry-run", action="store_true",
                   help="print the planned layout without writing anything")
    p.add_argument("--skip-ulog-time", action="store_true",
                   help="skip parsing ulogs to date them (faster, less accurate)")
    p.set_defaults(func=cmd_build)

    p = sub.add_parser("organize", help="move raw data into raw/<date>/<source>/")
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(func=cmd_organize)

    p = sub.add_parser("map", help="thrust/torque vs PWM map + voltage-sag analysis")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--show-rejected", action="store_true",
                   help="explain why constant-PWM groups were excluded from sag")
    p.add_argument("--normalize", action="store_true",
                   help="correct thrust to a common voltage (thrust ~ V^k)")
    p.add_argument("--v-ref", type=float,
                   help="reference voltage for --normalize (default: median)")
    p.set_defaults(func=cmd_map)

    p = sub.add_parser("ulog", help="analyze .ulg file(s)")
    p.add_argument("files", nargs="+")
    p.add_argument("-o", "--outdir")
    p.add_argument("--segment", action="store_true",
                   help="list the runs inside each log with their no-load voltages")
    p.set_defaults(func=cmd_ulog)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
