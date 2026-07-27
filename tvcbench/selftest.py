"""
Pre-session health check for the load-cell link.

This is the gate the whole package sits behind. Moving the STM32 from the bench
laptop onto the Pi is what removes post-hoc time alignment, but it introduces
three hardware risks, and all three are silent -- they corrupt data rather than
stopping it. `selftest` makes each one visible in twenty seconds:

1. **Power.** The board draws its own supply and the bridge excitation over the
   same cable. A Pi 5 gives ~1.6 A across its USB-A ports with the 27 W supply
   but ~600 mA with a weaker one. A brownout does not announce itself -- the MCU
   resets, so watch for its counters jumping backwards.
2. **Ground loop.** The Pi now shares ground with the STM32 and the Pixhawk
   around a frame carrying motor current. That shows up as a raised noise floor,
   so record one on the laptop as a baseline and compare the Pi against it.
3. **Link integrity.** Dropouts, parse failures and rate shortfalls, each
   counted separately because they have different causes and different fixes.

Everything reported here is measured with the motors stopped, so the numbers are
the instrument's own floor, not the bench's vibration.
"""

import json

import numpy as np

from tvcbench.clock import fit_device_clock, now
from tvcbench.sources.loadcell import FT_CHANNELS, NOMINAL_HZ

PASS, WARN, FAIL, INFO = "PASS", "WARN", "FAIL", "INFO"

# Achieved rate below this fraction of nominal means samples are being lost
# somewhere the counters cannot see -- usually the host, not the wire.
RATE_WARN_FRAC = 0.95
RATE_FAIL_FRAC = 0.80

# Any wire dropout at all is worth flagging; a run's worth of them is a fault.
DROP_WARN_FRAC = 0.0
DROP_FAIL_FRAC = 0.001

# A gap of several sample periods is a stall. At 50 Hz one period is 20 ms.
GAP_WARN_S = 0.10
GAP_FAIL_S = 0.50

# Two consumer crystals against each other; beyond this something is wrong with
# the fit or the part, not the tolerance.
PPM_WARN = 150.0
PPM_FAIL = 500.0

# How much the noise floor may rise against a baseline before a ground loop is
# the likely explanation. Sensor noise is stable run-to-run, so a real increase
# stands out well before this.
NOISE_WARN_FRAC = 0.25
NOISE_FAIL_FRAC = 1.00


def collect(source, seconds, progress=None):
    """Run a started source for `seconds` and return everything it emitted."""
    samples = []
    t_end = now() + seconds
    last_report = 0.0
    while now() < t_end:
        samples.extend(source.drain())
        if progress is not None:
            remaining = t_end - now()
            if last_report - remaining > 1.0 or last_report == 0.0:
                last_report = remaining
                progress(max(0.0, remaining), len(samples))
        if not source.alive:
            break
        # Short enough to keep the drain loop responsive, long enough not to spin.
        _sleep(0.02)
    samples.extend(source.drain())
    return samples


def _sleep(dt):
    import time

    time.sleep(dt)


def _verdict(value, warn, fail, higher_is_worse=True):
    if higher_is_worse:
        return FAIL if value >= fail else WARN if value > warn else PASS
    return FAIL if value <= fail else WARN if value < warn else PASS


def analyse(samples, source, seconds, nominal_hz=NOMINAL_HZ, baseline=None):
    """
    Turn a collected burst into a report dict.

    Structured as `{section: {check: {value, verdict, note}}}` so the same data
    can be printed at the bench and diffed against a saved baseline later.
    """
    stats = source.stats()
    report = {
        "nominal_hz": nominal_hz,
        "seconds": seconds,
        "n_samples": len(samples),
        "checks": {},
        "noise": {},
        "clock": {},
        "source_stats": {k: v for k, v in stats.items() if k != "tare"},
    }
    checks = report["checks"]

    if not samples:
        checks["samples"] = {"value": 0, "verdict": FAIL,
                             "note": stats.get("error") or "no data from device"}
        return report

    t_mono = np.array([s.t_mono for s in samples], dtype=float)
    span = float(t_mono[-1] - t_mono[0])
    rate = (len(samples) - 1) / span if span > 0 else 0.0

    checks["rate_hz"] = {
        "value": rate,
        "verdict": _verdict(rate, nominal_hz * RATE_WARN_FRAC,
                            nominal_hz * RATE_FAIL_FRAC, higher_is_worse=False),
        "note": f"nominal {nominal_hz:g} Hz",
    }

    dropped = stats.get("dropped", 0)
    drop_frac = dropped / max(1, len(samples) + dropped)
    checks["wire_dropouts"] = {
        "value": dropped,
        "verdict": _verdict(drop_frac, DROP_WARN_FRAC, DROP_FAIL_FRAC),
        "note": "samples lost between MCU and host (from the fc counter)",
    }

    checks["parse_failures"] = {
        "value": stats.get("parse_fail", 0),
        "verdict": PASS if not stats.get("parse_fail") else WARN,
        "note": "unrecognised lines -- a firmware format change looks like this",
    }

    # Counters running backwards is the brownout signature. The MCU restarts, its
    # uptime and sample counters reset, and nothing else in the stream says so.
    resets = _count_resets(samples)
    checks["counter_resets"] = {
        "value": resets,
        "verdict": PASS if resets == 0 else FAIL,
        "note": "MCU restarted mid-capture -- suspect USB power (use a powered hub)",
    }

    gaps = np.diff(t_mono)
    max_gap = float(gaps.max()) if gaps.size else 0.0
    checks["max_gap_s"] = {
        "value": max_gap,
        "verdict": _verdict(max_gap, GAP_WARN_S, GAP_FAIL_S),
        "note": f"largest silence between arrivals ({1 / nominal_hz * 1e3:.0f} ms expected)",
    }

    if stats.get("overflow"):
        checks["buffer_overflow"] = {
            "value": stats["overflow"], "verdict": WARN,
            "note": "consumer fell behind the reader thread",
        }

    report["clock"] = _clock_section(samples)
    report["noise"] = _noise_section(samples, baseline)
    return report


def _count_resets(samples):
    """Times the device counter jumped backwards -- one per MCU restart."""
    resets = 0
    prev = None
    for s in samples:
        dev = s.dev_t
        if dev is None:
            continue
        if prev is not None and dev < prev:
            resets += 1
        prev = dev
    return resets


def _clock_section(samples):
    dev = [s.dev_t for s in samples if s.dev_t is not None]
    host = [s.t_mono for s in samples if s.dev_t is not None]
    if len(dev) < 2:
        return {"available": False}

    fit = fit_device_clock(np.asarray(dev) * 1e-3, np.asarray(host), label="loadcell")
    out = fit.as_dict()
    out["available"] = True
    out["ppm_verdict"] = _verdict(abs(fit.ppm), PPM_WARN, PPM_FAIL)
    # Batch spread is informational: it says how much timing the raw arrival
    # stamps had lost, all of which the fit puts back. It is not a fault.
    out["batch_spread_ms"] = fit.floor_width_s * 1e3
    return out


def _noise_section(samples, baseline=None):
    """
    Standard deviation of each channel with the motors stopped.

    This is the instrument floor and the ground-loop detector. It is deliberately
    *not* compared against the ~0.7 N within-step figure from the running bench:
    that number is propeller vibration, a real force, and has nothing to do with
    whether the cable arrangement is sound.
    """
    out = {"channels": {}}
    for ch in FT_CHANNELS:
        vals = np.array([s.fields.get(f"{ch}_raw") for s in samples
                         if s.fields.get(f"{ch}_raw") is not None], dtype=float)
        if vals.size < 2:
            continue
        out["channels"][ch] = {"sd": float(vals.std(ddof=1)),
                               "mean": float(vals.mean()),
                               "p2p": float(vals.max() - vals.min())}

    fz = out["channels"].get("Fz")
    if fz:
        out["thrust_sd_n"] = fz["sd"]

    base_fz = (baseline or {}).get("noise", {}).get("thrust_sd_n")
    if base_fz and fz:
        frac = (fz["sd"] - base_fz) / base_fz
        out["baseline_thrust_sd_n"] = base_fz
        out["baseline_delta_frac"] = frac
        out["baseline_verdict"] = _verdict(frac, NOISE_WARN_FRAC, NOISE_FAIL_FRAC)
    return out


def worst_verdict(report):
    """Overall result: the worst of everything checked."""
    seen = [c["verdict"] for c in report["checks"].values()]
    if report.get("clock", {}).get("available"):
        seen.append(report["clock"]["ppm_verdict"])
    if "baseline_verdict" in report.get("noise", {}):
        seen.append(report["noise"]["baseline_verdict"])
    for level in (FAIL, WARN):
        if level in seen:
            return level
    return PASS if seen else FAIL


_FORMATS = {
    "rate_hz": "{:.2f} Hz",
    "max_gap_s": "{:.3f} s",
}


def _row(label, value, verdict, note=""):
    return f"  {label:<18}{value:<16}{verdict:<6}{note}"


def format_report(report, port=""):
    """Render the report for a terminal at the bench."""
    lines = []
    head = f"LOAD CELL  {port}" if port else "LOAD CELL"
    lines.append(head)
    lines.append(f"  {'samples':<18}{report['n_samples']} in {report['seconds']:.1f} s")

    for name, check in report["checks"].items():
        value = _FORMATS.get(name, "{}").format(check["value"])
        lines.append(_row(name, value, check["verdict"], check["note"]))

    clock = report.get("clock", {})
    if clock.get("available"):
        lines.append("")
        lines.append("CLOCK FIT  (device counter -> host monotonic)")
        lines.append(_row("rate error", f"{clock['ppm']:+.1f} ppm",
                          clock["ppm_verdict"], "device vs host crystal"))
        lines.append(_row("batch spread", f"{clock['batch_spread_ms']:.1f} ms", INFO,
                          "timing the raw stamps lost, and the fit restored"))
        lines.append(_row("residual p99", f"{clock['residual_p99_s'] * 1e3:.1f} ms",
                          INFO, "worst arrival lateness"))
        for w in clock.get("warnings", []):
            lines.append(f"  {'!':<18}{w}")

    noise = report.get("noise", {})
    if noise.get("channels"):
        lines.append("")
        lines.append("NOISE FLOOR  (motors stopped -- instrument, not vibration)")
        forces = " ".join(f"{ch} {noise['channels'][ch]['sd']:.4f}"
                          for ch in ("Fx", "Fy", "Fz") if ch in noise["channels"])
        torques = " ".join(f"{ch} {noise['channels'][ch]['sd']:.5f}"
                           for ch in ("Tx", "Ty", "Tz") if ch in noise["channels"])
        lines.append(f"  {'sd, N':<18}{forces}")
        lines.append(f"  {'sd, N*m':<18}{torques}")
        if "baseline_delta_frac" in noise:
            lines.append(_row("vs baseline",
                              f"{noise['baseline_delta_frac'] * 100:+.1f} %",
                              noise["baseline_verdict"],
                              f"baseline {noise['baseline_thrust_sd_n']:.4f} N"))

    lines.append("")
    lines.append(f"OVERALL: {worst_verdict(report)}")
    return "\n".join(lines)


def save_baseline(report, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)


def load_baseline(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)
