"""
PX4 .ulg analysis: PWM output against battery voltage, current and power.

The flight controller records what it commanded and what the battery did, at a
much longer horizon than the bench scripts -- it covers the idle time before and
after each sweep, which is where the no-load voltage recovery shows up.

Standalone:
    python -m tvctools.ulog "motor test/ulog/log_3_2026-7-24-00-27-40.ulg"
"""

import argparse
import csv
import os
import sys

import numpy as np

# actuator_outputs has 16 slots but only the first few are ever wired; a channel
# counts as live only if it actually moved.
MAX_CHANNELS = 16
STEP_US = 100  # command staircase resolution, for the per-step summary


def load_ulog(path):
    """
    Extract the PWM and battery streams from a .ulg.

    Returns a dict of arrays with a shared t0 (seconds from log start).
    """
    try:
        from pyulog import ULog
    except ImportError:
        raise SystemExit(
            "pyulog is required to read .ulg files.\n"
            "    pip install pyulog")

    ulog = ULog(path)
    t0 = ulog.start_timestamp

    ao = [d for d in ulog.data_list
          if d.name == "actuator_outputs" and d.multi_id == 0]
    bat = [d for d in ulog.data_list if d.name == "battery_status"]
    if not ao:
        raise SystemExit("%s has no actuator_outputs data" % os.path.basename(path))

    ao = ao[0]
    out = {
        "name": os.path.basename(path),
        "pwm_t": (ao.data["timestamp"] - t0) / 1e6,
        "channels": {},
        "duration_s": (ulog.last_timestamp - t0) / 1e6,
    }
    for i in range(MAX_CHANNELS):
        key = "output[%d]" % i
        if key not in ao.data:
            continue
        v = np.asarray(ao.data[key], dtype=float)
        # Constant channels are disarmed idle or unused padding, not signals
        if v.std() > 1e-6:
            out["channels"][i] = v

    if bat:
        b = bat[0]
        out["bat_t"] = (b.data["timestamp"] - t0) / 1e6
        for src, dst in (("voltage_v", "voltage"), ("current_a", "current"),
                         ("ocv_estimate_filtered", "ocv"),
                         ("discharged_mah", "discharged_mah")):
            if src in b.data:
                out[dst] = np.asarray(b.data[src], dtype=float)
        if "voltage" in out and "current" in out:
            out["power"] = out["voltage"] * out["current"]
        if "ocv" in out and "voltage" in out:
            out["sag"] = out["ocv"] - out["voltage"]
    return out


def throttle_channel(data):
    """
    The channel driving the motor: the one with the widest travel.

    On this bench that is consistently output[1] (full 1000-2000 sweep), with
    output[0] a narrower gimbal/enable channel.
    """
    if not data["channels"]:
        return None, None
    idx = max(data["channels"], key=lambda i: np.ptp(data["channels"][i]))
    return idx, data["channels"][idx]


def step_table(data, step_us=STEP_US):
    """Mean voltage/current/power at each commanded throttle level."""
    idx, thr = throttle_channel(data)
    if thr is None or "bat_t" not in data:
        return []
    v = np.interp(data["pwm_t"], data["bat_t"], data["voltage"])
    i = np.interp(data["pwm_t"], data["bat_t"], data["current"])
    levels = np.round(thr / step_us) * step_us

    rows = []
    for lvl in sorted(np.unique(levels)):
        m = levels == lvl
        # Skip idle and any level with too few samples to average meaningfully
        if lvl <= thr.min() or m.sum() < 5:
            continue
        rows.append({
            "pwm_us": int(lvl), "n": int(m.sum()),
            "voltage_v": round(float(v[m].mean()), 3),
            "current_a": round(float(i[m].mean()), 3),
            "power_w": round(float((v[m] * i[m]).mean()), 1),
        })
    return rows


def correlations(data):
    """Correlation of each live PWM channel against voltage and current."""
    if "bat_t" not in data:
        return {}
    v = np.interp(data["pwm_t"], data["bat_t"], data["voltage"])
    i = np.interp(data["pwm_t"], data["bat_t"], data["current"])
    out = {}
    for ch, sig in data["channels"].items():
        out[ch] = {
            "voltage": round(float(np.corrcoef(sig, v)[0, 1]), 3),
            "current": round(float(np.corrcoef(sig, i)[0, 1]), 3),
        }
    return out


def summarize(data):
    """One-screen text report."""
    lines = ["%s  (%.1f s)" % (data["name"], data["duration_s"])]
    idx, thr = throttle_channel(data)
    lines.append("  live PWM channels: %s (throttle = ch%s)"
                 % (sorted(data["channels"]) or "none", idx))
    if "voltage" in data:
        lines.append("  voltage %.2f-%.2f V   peak current %.1f A   peak power %.0f W"
                     % (data["voltage"].min(), data["voltage"].max(),
                        data["current"].max(), data["power"].max()))
        if "discharged_mah" in data:
            lines.append("  discharged %.0f mAh" % data["discharged_mah"].max())
        if "sag" in data:
            lines.append("  max sag %.2f V" % np.nanmax(data["sag"]))
    for ch, c in correlations(data).items():
        lines.append("  corr(ch%d, voltage) = %+.3f   corr(ch%d, current) = %+.3f"
                     % (ch, c["voltage"], ch, c["current"]))

    rows = step_table(data)
    if rows:
        lines.append("")
        lines.append("  %8s %8s %8s %9s" % ("PWM[us]", "V", "I[A]", "P[W]"))
        for r in rows:
            lines.append("  %8d %8.2f %8.1f %9.0f"
                         % (r["pwm_us"], r["voltage_v"], r["current_a"], r["power_w"]))
    return "\n".join(lines)


def write_csv(data, path):
    """Flatten the battery stream (with held PWM) to CSV for quick plotting."""
    if "bat_t" not in data:
        return None
    idx, thr = throttle_channel(data)
    fields = ["t_s", "pwm_us", "voltage_v", "current_a", "power_w", "ocv_v",
              "sag_v", "discharged_mah"]
    pwm_on_bat = (np.interp(data["bat_t"], data["pwm_t"], thr)
                  if thr is not None else None)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for k in range(len(data["bat_t"])):
            w.writerow({
                "t_s": round(float(data["bat_t"][k]), 3),
                "pwm_us": int(round(pwm_on_bat[k])) if pwm_on_bat is not None else "",
                "voltage_v": round(float(data["voltage"][k]), 3),
                "current_a": round(float(data["current"][k]), 3),
                "power_w": round(float(data["power"][k]), 2),
                "ocv_v": round(float(data["ocv"][k]), 3) if "ocv" in data else "",
                "sag_v": round(float(data["sag"][k]), 3) if "sag" in data else "",
                "discharged_mah": (round(float(data["discharged_mah"][k]), 1)
                                   if "discharged_mah" in data else ""),
            })
    return path


def plot(data, path):
    """Three stacked panels: PWM, voltage (loaded vs OCV), current and power."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)

    ax = axes[0]
    for ch, sig in sorted(data["channels"].items()):
        ax.plot(data["pwm_t"], sig, lw=0.8, label="PWM ch%d" % ch)
    ax.set_ylabel("PWM [us]")
    ax.set_ylim(950, 2050)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("PWM vs battery voltage / current / power - %s" % data["name"])

    ax = axes[1]
    if "voltage" in data:
        ax.plot(data["bat_t"], data["voltage"], color="#d62728", lw=1.4,
                label="voltage (loaded)")
        if "ocv" in data:
            ax.plot(data["bat_t"], data["ocv"], color="#ff7f0e", lw=1.0, ls="--",
                    label="OCV (no-load est.)")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_ylabel("voltage [V]")
    ax.grid(alpha=0.3)

    ax = axes[2]
    if "current" in data:
        ax.plot(data["bat_t"], data["current"], color="#9467bd", lw=1.0,
                label="current [A]")
        ax.set_ylabel("current [A]", color="#9467bd")
        ax.tick_params(axis="y", labelcolor="#9467bd")
        axp = ax.twinx()
        axp.plot(data["bat_t"], data["power"], color="#8c564b", lw=1.0, alpha=0.7,
                 label="power [W]")
        axp.set_ylabel("power [W]", color="#8c564b")
        axp.tick_params(axis="y", labelcolor="#8c564b")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = axp.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8)
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def analyze(path, outdir=None, make_plot=True, make_csv=True):
    """Run the full analysis, writing <name>_pwm_power.png / _pwm_voltage.csv."""
    data = load_ulog(path)
    print(summarize(data))
    outdir = outdir or os.path.dirname(os.path.abspath(path))
    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(path))[0]
    written = []
    if make_csv:
        p = write_csv(data, os.path.join(outdir, stem + "_pwm_voltage.csv"))
        if p:
            written.append(p)
    if make_plot:
        written.append(plot(data, os.path.join(outdir, stem + "_pwm_power.png")))
    for p in written:
        print("  wrote %s" % p)
    return data, written


def main(argv=None):
    ap = argparse.ArgumentParser(description="Analyze PX4 .ulg PWM vs battery data")
    ap.add_argument("ulog", nargs="+", help="path(s) to .ulg file(s)")
    ap.add_argument("-o", "--outdir", help="where to write outputs (default: alongside)")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--no-csv", action="store_true")
    args = ap.parse_args(argv)
    for path in args.ulog:
        analyze(path, args.outdir, not args.no_plot, not args.no_csv)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
