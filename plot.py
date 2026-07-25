#!/usr/bin/env python3
"""
Thrust Stand Data Viewer
------------------------
Zero-friction viewer for motor/prop thrust-stand logs.

Drop this file in the SAME FOLDER as your data_*.csv files and run:
    python thrust_viewer.py

- Auto-lists every CSV in the folder in a checkable list.
- Select one or more files to instantly overlay:
    (1) PWM vs Thrust
    (2) RPM  vs Thrust
- Per file: max thrust (N & gf) with PWM/RPM/current/efficiency, max RPM
  with its PWM/thrust, and the tested PWM range.
- Built for iterative testing: run once, keep it open, hit "Reload folder"
  after each new bench run to pick up freshly-saved CSVs.

Expected columns (header row required):
    t_ms, pwm, rpm, Fx, Fy, Fz, Tx, Ty, Tz, Current_mA, ADC_Current_mA
Thrust is taken as Fz (vertical load-cell axis), signed, logged in Newtons --
the same convention used by the other plotting scripts in this project.
Toggle the unit selector between "Newtons" (raw Fz) and "grams-force"
(Fz / 9.80665 * 1000). The max-thrust summary always shows both.

Only dependencies: matplotlib, numpy  (pip install matplotlib numpy)
tkinter ships with standard CPython.
"""

import os
import csv
import sys
import glob
import zlib

import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    _GUI_OK = True
except Exception as _e:  # pragma: no cover
    _GUI_OK = False
    _GUI_ERR = _e


# ----------------------------- data handling ------------------------------- #

REQUIRED = ["pwm", "rpm", "Fz"]
G = 9.80665  # m/s^2, for N -> gram-force conversion
GF_PER_N = 1000.0 / G  # multiply Newtons by this to get grams-force

# Fixed 8-hue categorical palette (validated for CVD-safe adjacency), assigned
# to files by a stable hash of their filename -- not by selection order --
# so a given file always plots in the same color.
PALETTE = [
    "#2a78d6",  # blue
    "#1baf7a",  # aqua
    "#eda100",  # yellow
    "#008300",  # green
    "#4a3aa7",  # violet
    "#e34948",  # red
    "#e87ba4",  # magenta
    "#eb6834",  # orange
]
MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*"]


def color_for(name):
    """Deterministic palette color for a filename (stable across reloads/selections)."""
    return PALETTE[zlib.crc32(name.encode("utf-8")) % len(PALETTE)]


def trend_line(x, y, degree=2, n=100):
    """Fit a smooth trend curve through (x, y), dropping NaNs. Returns (xs, ys) or None."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    finite = np.isfinite(x) & np.isfinite(y)
    x, y = x[finite], y[finite]
    if len(x) < 3 or np.ptp(x) == 0:
        return None
    deg = min(degree, len(x) - 1)
    coeffs = np.polyfit(x, y, deg)
    xs = np.linspace(x.min(), x.max(), n)
    ys = np.polyval(coeffs, xs)
    return xs, ys


def script_dir():
    """Folder the script lives in (works when run as a file)."""
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return os.getcwd()


def _is_loadcell(path):
    """True if the header carries the load-cell schema (t_ms + Fz)."""
    try:
        with open(path, "r", newline="") as f:
            header = next(csv.reader(f), []) or []
    except OSError:
        return False
    names = {h.strip() for h in header}
    return "t_ms" in names and "Fz" in names


def load_csv(path):
    """
    Read a thrust log. Returns dict of numpy arrays keyed by column name.
    Robust to extra/missing trailing columns and blank cells.
    """
    cols = {}
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        header = [h.strip() for h in header]
        idx = {name: i for i, name in enumerate(header)}
        for name in REQUIRED:
            if name not in idx:
                raise ValueError(
                    f"'{os.path.basename(path)}' is missing required column '{name}'. "
                    f"Found columns: {header}"
                )
        data = {name: [] for name in header}
        for row in reader:
            if not row:
                continue
            for name, i in idx.items():
                if i < len(row) and row[i] != "":
                    try:
                        data[name].append(float(row[i]))
                    except ValueError:
                        data[name].append(np.nan)
                else:
                    data[name].append(np.nan)
    for name in data:
        cols[name] = np.asarray(data[name], dtype=float)
    return cols


def mad_mask(x, k=3.5):
    """Boolean mask (True = keep) rejecting samples > k robust-sigmas from the median."""
    finite = np.isfinite(x)
    if finite.sum() < 4:
        return finite
    med = np.nanmedian(x[finite])
    mad = np.nanmedian(np.abs(x[finite] - med)) * 1.4826  # normal-consistent MAD
    if mad == 0:
        return finite
    keep = np.abs(x - med) <= k * mad
    return finite & keep


def hampel_filter(values, window=5, k=3.5):
    """Boolean mask (True = keep) dropping points whose value deviates from
    the local (windowed) median by more than k robust-sigmas -- catches
    isolated aggregated PWM points that don't fit the surrounding trend."""
    n = len(values)
    keep = np.ones(n, dtype=bool)
    half = window // 2
    for idx in range(n):
        lo, hi = max(0, idx - half), min(n, idx + half + 1)
        local = values[lo:hi]
        med = np.median(local)
        mad = np.median(np.abs(local - med)) * 1.4826
        if mad == 0:
            continue
        if abs(values[idx] - med) > k * mad:
            keep[idx] = False
    return keep


def aggregate_by_pwm(cols, thrust_mode="raw", settle_frac=0.5, min_samples=5,
                      filter_outliers=True):
    """
    Group samples by PWM level and compute steady-state means.

    For each distinct PWM value:
      - take the samples at that PWM,
      - discard the first `settle_frac` fraction (throttle transient),
      - reject per-sample outliers (sensor glitches / dropouts) via MAD,
      - average the remaining steady portion for thrust, RPM and current.

    Then, across the whole PWM sweep, drop any aggregated point that is
    itself a strange outlier relative to its neighbors (Hampel filter on
    thrust) -- e.g. a whole PWM step contaminated by a vibration event.

    thrust_mode:
      "raw"   -> thrust = Fz exactly as logged (signed, no absolute value)
      "grams" -> thrust = Fz / g * 1000  (treats Fz as Newtons -> grams-force)

    Returns arrays sorted by PWM: pwm[], thrust[], rpm[], thrust_std[], current_mA[]
    current_mA is all-NaN if the CSV has no Current_mA column.
    """
    pwm = cols["pwm"]
    rpm = cols["rpm"]
    fz = cols["Fz"]
    current = cols.get("Current_mA", np.full_like(pwm, np.nan))

    thrust_raw = fz
    if thrust_mode == "grams":
        thrust_raw = thrust_raw / G * 1000.0

    out_pwm, out_thr, out_rpm, out_std, out_cur = [], [], [], [], []
    # Preserve encounter order but aggregate per unique level
    for level in sorted(set(pwm[~np.isnan(pwm)])):
        mask = pwm == level
        t_seg = thrust_raw[mask]
        r_seg = rpm[mask]
        c_seg = current[mask]
        n = len(t_seg)
        if n < min_samples:
            continue
        start = int(n * settle_frac)
        t_steady = t_seg[start:]
        r_steady = r_seg[start:]
        c_steady = c_seg[start:]
        # guard against all-nan
        if np.all(np.isnan(t_steady)):
            continue

        if filter_outliers:
            t_keep = mad_mask(t_steady)
            r_keep = mad_mask(r_steady)
            c_keep = mad_mask(c_steady)
        else:
            all_true = np.ones(len(t_steady), dtype=bool)
            t_keep = r_keep = c_keep = all_true

        out_pwm.append(level)
        out_thr.append(np.nanmean(t_steady[t_keep]) if t_keep.any() else np.nanmean(t_steady))
        out_rpm.append(np.nanmean(r_steady[r_keep]) if r_keep.any() else np.nanmean(r_steady))
        out_std.append(np.nanstd(t_steady[t_keep]) if t_keep.any() else np.nanstd(t_steady))
        if c_keep.any() and not np.all(np.isnan(c_steady[c_keep])):
            out_cur.append(np.nanmean(c_steady[c_keep]))
        else:
            out_cur.append(np.nan)

    order = np.argsort(out_pwm)
    out_pwm = np.asarray(out_pwm)[order]
    out_thr = np.asarray(out_thr)[order]
    out_rpm = np.asarray(out_rpm)[order]
    out_std = np.asarray(out_std)[order]
    out_cur = np.asarray(out_cur)[order]

    if filter_outliers and len(out_thr) >= 5:
        keep = hampel_filter(out_thr)
        out_pwm, out_thr = out_pwm[keep], out_thr[keep]
        out_rpm, out_std, out_cur = out_rpm[keep], out_std[keep], out_cur[keep]

    return out_pwm, out_thr, out_rpm, out_std, out_cur


TORQUE_COL = "Tz"  # vertical torque axis, same convention as Fz


def trim_to_active(t_ms, thrust, torque, threshold_frac=0.05, pad=5):
    """
    Cut the leading/trailing stretch of a time series where there's no real
    thrust (idle/noise before the run starts and after it ends).

    A sample counts as "active" once |thrust| exceeds `threshold_frac` of
    the run's peak |thrust|. Keeps `pad` extra samples on each side of the
    active stretch so the ramp up/down isn't clipped. Falls back to the
    full series if nothing looks active (e.g. an all-idle log).
    """
    thrust = np.asarray(thrust, dtype=float)
    finite = np.isfinite(thrust)
    if not finite.any():
        return t_ms, thrust, torque
    peak = np.nanmax(np.abs(thrust[finite]))
    if peak <= 0:
        return t_ms, thrust, torque
    active = np.abs(thrust) > threshold_frac * peak
    if not active.any():
        return t_ms, thrust, torque
    idx = np.flatnonzero(active)
    lo = max(0, idx[0] - pad)
    hi = min(len(t_ms), idx[-1] + pad + 1)
    return t_ms[lo:hi], thrust[lo:hi], torque[lo:hi]


# ------------------------------- GUI --------------------------------------- #

class ThrustViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Thrust Stand Data Viewer")
        self.geometry("1200x760")
        self.minsize(900, 560)

        self.folder = script_dir()
        self.thrust_mode = tk.StringVar(value="raw")
        self.settle = tk.DoubleVar(value=0.5)
        self.filter_outliers = tk.BooleanVar(value=True)

        self._build_layout()
        self.reload_folder()

    # ---- layout ---- #
    def _build_layout(self):
        left = ttk.Frame(self, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(left, text="CSV files in folder", font=("", 10, "bold")).pack(anchor="w")
        ttk.Label(left, text=self.folder, foreground="#888",
                  wraplength=240, font=("", 8)).pack(anchor="w", pady=(0, 6))

        # multi-select list
        self.listbox = tk.Listbox(left, selectmode=tk.EXTENDED, width=34, height=22,
                                  exportselection=False, activestyle="dotbox")
        self.listbox.pack(fill=tk.BOTH, expand=True)
        self.listbox.bind("<<ListboxSelect>>", lambda e: self.refresh_plots())

        btns = ttk.Frame(left)
        btns.pack(fill=tk.X, pady=6)
        ttk.Button(btns, text="Reload folder", command=self.reload_folder).pack(side=tk.LEFT)
        ttk.Button(btns, text="Select latest", command=self.select_latest).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Clear", command=self.clear_sel).pack(side=tk.LEFT)

        # options
        opt = ttk.LabelFrame(left, text="Thrust unit", padding=6)
        opt.pack(fill=tk.X, pady=(8, 4))
        ttk.Radiobutton(opt, text="Newtons (raw Fz)", value="raw",
                        variable=self.thrust_mode, command=self.refresh_plots).pack(anchor="w")
        ttk.Radiobutton(opt, text="grams-force (Fz ÷ 9.80665 × 1000)", value="grams",
                        variable=self.thrust_mode, command=self.refresh_plots).pack(anchor="w")

        stl = ttk.LabelFrame(left, text="Steady-state window", padding=6)
        stl.pack(fill=tk.X, pady=4)
        ttk.Label(stl, text="discard first fraction of each PWM step:",
                  wraplength=220, font=("", 8)).pack(anchor="w")
        s = ttk.Scale(stl, from_=0.0, to=0.9, variable=self.settle,
                      command=lambda e: self.refresh_plots())
        s.pack(fill=tk.X)
        self.settle_lbl = ttk.Label(stl, text="0.50")
        self.settle_lbl.pack(anchor="e")
        ttk.Checkbutton(stl, text="Remove outlier points", variable=self.filter_outliers,
                        command=self.refresh_plots).pack(anchor="w", pady=(4, 0))

        # summary box
        sumf = ttk.LabelFrame(left, text="Summary", padding=6)
        sumf.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        sum_scroll = ttk.Scrollbar(sumf, orient="vertical")
        self.summary = tk.Text(sumf, width=34, height=10, wrap="word", font=("Courier", 9),
                                yscrollcommand=sum_scroll.set)
        sum_scroll.config(command=self.summary.yview)
        sum_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.summary.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # right: plots, in tabs
        right = ttk.Frame(self, padding=4)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)

        steady_tab = ttk.Frame(nb)
        nb.add(steady_tab, text="Steady-state (PWM / RPM)")
        self.fig = Figure(figsize=(8, 7), dpi=100, constrained_layout=True)
        self.fig.set_facecolor("#fcfcfb")
        self.ax_pwm = self.fig.add_subplot(211)
        self.ax_rpm = self.fig.add_subplot(212)
        self.canvas = FigureCanvasTkAgg(self.fig, master=steady_tab)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, steady_tab)

        time_tab = ttk.Frame(nb)
        nb.add(time_tab, text="Time series (Thrust / Torque)")
        self.fig_time = Figure(figsize=(8, 7), dpi=100, constrained_layout=True)
        self.fig_time.set_facecolor("#fcfcfb")
        self.ax_time_thrust = self.fig_time.add_subplot(211)
        self.ax_time_torque = self.fig_time.add_subplot(212)
        self.canvas_time = FigureCanvasTkAgg(self.fig_time, master=time_tab)
        self.canvas_time.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas_time, time_tab)

    # ---- file list ops ---- #
    def reload_folder(self):
        # Load-cell logs live under raw/<date>/loadcell/ now, so search there as
        # well as the folder itself. Only files carrying the load-cell schema are
        # kept -- the pwm/ folder holds a different layout that load_csv rejects.
        found = glob.glob(os.path.join(self.folder, "*.csv"))
        found += glob.glob(os.path.join(self.folder, "raw", "*", "loadcell", "*.csv"))
        self.files = sorted(f for f in found if _is_loadcell(f))
        self.listbox.delete(0, tk.END)
        for f in self.files:
            self.listbox.insert(tk.END, os.path.basename(f))
        if not self.files:
            self.summary.delete("1.0", tk.END)
            self.summary.insert(tk.END, "No CSV files found in this folder.")

    def select_latest(self):
        if not self.files:
            return
        # latest by modification time
        latest = max(range(len(self.files)), key=lambda i: os.path.getmtime(self.files[i]))
        self.listbox.selection_clear(0, tk.END)
        self.listbox.selection_set(latest)
        self.listbox.see(latest)
        self.refresh_plots()

    def clear_sel(self):
        self.listbox.selection_clear(0, tk.END)
        self.refresh_plots()

    # ---- plotting ---- #
    def refresh_plots(self):
        self.settle_lbl.config(text=f"{self.settle.get():.2f}")
        sel = self.listbox.curselection()
        self.ax_pwm.clear()
        self.ax_rpm.clear()
        self.summary.delete("1.0", tk.END)

        unit = "gf" if self.thrust_mode.get() == "grams" else "N"

        if not sel:
            for ax in (self.ax_pwm, self.ax_rpm):
                ax.text(0.5, 0.5, "Select one or more files on the left",
                        ha="center", va="center", transform=ax.transAxes, color="#999")
            self.canvas.draw()
            self.refresh_timeseries(sel, unit)
            return

        def to_both_units(val):
            """val is in the currently selected unit; return (N, gf)."""
            if unit == "gf":
                return val / GF_PER_N, val
            return val, val * GF_PER_N

        summary_lines = []

        for k, i in enumerate(sel):
            path = self.files[i]
            name = os.path.basename(path)
            color = color_for(name)
            marker = MARKERS[k % len(MARKERS)]
            try:
                cols = load_csv(path)
                pwm, thr, rpm, std, cur = aggregate_by_pwm(
                    cols, thrust_mode=self.thrust_mode.get(),
                    settle_frac=self.settle.get(),
                    filter_outliers=self.filter_outliers.get())
            except Exception as e:
                summary_lines.append(f"{name}\n  ERROR: {e}\n")
                continue

            if len(pwm) == 0:
                summary_lines.append(f"{name}\n  no usable PWM steps\n")
                continue

            label = name.replace("data_", "").replace(".csv", "")

            # dots only (no connecting line) + a separate fitted trend curve
            self.ax_pwm.errorbar(pwm, thr, yerr=std, marker=marker, ms=6, linestyle="none",
                                 capsize=3, color=color, label=label,
                                 markeredgecolor="white", markeredgewidth=1.1)
            self.ax_rpm.plot(rpm, thr, marker=marker, ms=6, linestyle="none", color=color,
                              label=label, markeredgecolor="white", markeredgewidth=1.1)
            fit_pwm = trend_line(pwm, thr)
            if fit_pwm is not None:
                self.ax_pwm.plot(*fit_pwm, color=color, lw=1.6, linestyle="--", alpha=0.85)
            fit_rpm = trend_line(rpm, thr)
            if fit_rpm is not None:
                self.ax_rpm.plot(*fit_rpm, color=color, lw=1.6, linestyle="--", alpha=0.85)

            # locate the strongest thrust point by magnitude (sign may be
            # negative depending on load-cell mounting), but report the
            # signed value itself -- no absolute value taken on the number.
            jmax_t = int(np.argmax(np.abs(thr)))
            jmax_r = int(np.argmax(rpm))
            n_t, gf_t = to_both_units(thr[jmax_t])
            n_r, gf_r = to_both_units(thr[jmax_r])
            cur_t = cur[jmax_t]
            eff_t = (gf_t / (cur_t / 1000.0)) if np.isfinite(cur_t) and cur_t > 0 else np.nan

            lines = [
                f"{label}",
                f"  max thrust : {n_t:.2f} N   ({gf_t:.1f} gf)",
                f"    @ PWM {int(pwm[jmax_t])} us   RPM {rpm[jmax_t]:.0f}"
                + (f"   I {cur_t:.0f} mA" if np.isfinite(cur_t) else ""),
            ]
            if np.isfinite(eff_t):
                lines.append(f"    efficiency  {eff_t:.1f} gf/A")
            lines.append(f"  max RPM    : {rpm[jmax_r]:.0f}")
            lines.append(
                f"    @ PWM {int(pwm[jmax_r])} us   thrust {n_r:.2f} N ({gf_r:.1f} gf)"
            )
            lines.append(
                f"  PWM range  : {int(pwm.min())}-{int(pwm.max())} us  ({len(pwm)} steps)"
            )
            summary_lines.append("\n".join(lines) + "\n")

        n_series = len(sel)
        self.ax_pwm.set_xlabel("PWM (us)")
        self.ax_pwm.set_ylabel(f"Thrust ({unit})")
        self.ax_pwm.set_title("PWM vs Thrust")
        self._style_axis(self.ax_pwm, legend=n_series > 1)

        self.ax_rpm.set_xlabel("RPM")
        self.ax_rpm.set_ylabel(f"Thrust ({unit})")
        self.ax_rpm.set_title("RPM vs Thrust")
        self._style_axis(self.ax_rpm, legend=n_series > 1)

        self.summary.insert(tk.END, "\n".join(summary_lines))
        self.canvas.draw()
        self.refresh_timeseries(sel, unit)

    def refresh_timeseries(self, sel, unit):
        self.ax_time_thrust.clear()
        self.ax_time_torque.clear()

        if not sel:
            for ax in (self.ax_time_thrust, self.ax_time_torque):
                ax.text(0.5, 0.5, "Select one or more files on the left",
                        ha="center", va="center", transform=ax.transAxes, color="#999")
            self.canvas_time.draw()
            return

        for i in sel:
            path = self.files[i]
            name = os.path.basename(path)
            color = color_for(name)
            label = name.replace("data_", "").replace(".csv", "")
            try:
                cols = load_csv(path)
            except Exception:
                continue
            if "t_ms" not in cols:
                continue

            t_ms = cols["t_ms"]
            fz = cols["Fz"]
            if unit == "gf":
                fz = fz * GF_PER_N
            tz = cols.get(TORQUE_COL, np.full_like(fz, np.nan))

            t_ms, fz, tz = trim_to_active(t_ms, fz, tz)
            if len(t_ms) == 0:
                continue
            t_sec = (t_ms - t_ms[0]) / 1000.0

            self.ax_time_thrust.plot(t_sec, fz, lw=1.3, color=color, label=label)
            self.ax_time_torque.plot(t_sec, tz, lw=1.3, color=color, label=label)

        n_series = len(sel)
        self.ax_time_thrust.set_xlabel("time (s)")
        self.ax_time_thrust.set_ylabel(f"Thrust ({unit})")
        self.ax_time_thrust.set_title("Time vs Thrust")
        self._style_axis(self.ax_time_thrust, legend=n_series > 1)

        self.ax_time_torque.set_xlabel("time (s)")
        self.ax_time_torque.set_ylabel(f"Torque ({TORQUE_COL}, Nm)")
        self.ax_time_torque.set_title("Time vs Torque")
        self._style_axis(self.ax_time_torque, legend=n_series > 1)

        self.canvas_time.draw()

    @staticmethod
    def _style_axis(ax, legend):
        """Recessive chrome: hairline grid, muted spines, legend only for 2+ series."""
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color="#e1e0d9", linewidth=0.8, linestyle="-")
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color("#c3c2b7")
        ax.tick_params(colors="#52514e")
        if legend:
            ax.legend(fontsize=8, frameon=False)


if __name__ == "__main__":
    if not _GUI_OK:
        sys.stderr.write(
            "Could not start the GUI. This tool needs tkinter + matplotlib.\n"
            f"Details: {_GUI_ERR}\n\n"
            "Fix:\n"
            "  pip install matplotlib numpy\n"
            "  (tkinter ships with python.org builds; on Linux: sudo apt install python3-tk)\n"
        )
        sys.exit(1)
    app = ThrustViewer()
    app.mainloop()