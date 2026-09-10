"""
Battery-sag identification from the sustained-load bench runs.
================================================================================
    python identify_battery.py     # -> out/battery_sag.json, and a YAML comparison

The thrust/torque surface is a FRESH-PACK fit. Over a real flight the pack sags
and thrust falls with it. This script measures that fade from the seven
back-to-back A1850/B1850 60 s runs (2026-07-31), which drain the pack from
~11.5 V to ~9.0 V, and produces the two numbers a sim model needs:

  thrust_sensitivity_n_per_v   how many newtons are lost per volt of sag, at a
                               fixed command (slope of thrust vs voltage).
  voltage vs drawn charge      the discharge curve, via COULOMB COUNTING
                               (integrating the measured current), so the sim can
                               map state-of-charge -> voltage -> thrust derate.

Provenance: this REPLACES trusting vehicle_params.yaml's stored 1.46 N/V; the
script prints both so the difference is visible. Nothing here is written back
automatically -- the number crosses into the YAML by hand, tagged measured.
"""
import csv
import glob
import json
import os
from dataclasses import asdict, dataclass

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_TESTS = os.path.join(_HERE, "raw", "2026-07-31", "A1850_B1850_TESTS")
_OUT = os.path.join(_HERE, "out")

_POWERED = "A1850_B1850"


@dataclass
class BatterySagFit:
    thrust_sensitivity_n_per_v: float   # N per V, slope of thrust vs voltage
    thrust_v_intercept_n: float
    r: float                            # correlation of that fit
    v_full: float                       # V at 0 mAh drawn (discharge-curve intercept)
    v_per_mah: float                    # V lost per mAh (negative)
    capacity_to_cutoff_mah: float       # mAh from full to a 9.0 V cutoff, from the fit
    thrust_start_n: float
    thrust_end_n: float
    voltage_start_v: float
    voltage_end_v: float
    n_samples: int


def coulomb_count(t, current_a):
    """Cumulative charge drawn, in mAh, by trapezoidal integration of current.

    1 mAh = 3.6 A*s. Monotonic non-decreasing for non-negative current.
    """
    t = np.asarray(t, dtype=float)
    i = np.asarray(current_a, dtype=float)
    dq_as = np.concatenate([[0.0], 0.5 * (i[1:] + i[:-1]) * np.diff(t)])
    return np.cumsum(dq_as) / 3.6


def fit_sag(thrust, voltage, mah, cutoff_v=9.0):
    """Least-squares thrust~voltage and voltage~charge from paired samples."""
    thrust = np.asarray(thrust, float)
    voltage = np.asarray(voltage, float)
    mah = np.asarray(mah, float)

    k, b = np.polyfit(voltage, thrust, 1)                 # thrust = k*V + b
    r = float(np.corrcoef(voltage, thrust)[0, 1])

    vslope, vfull = np.polyfit(mah, voltage, 1)           # V = vslope*mAh + vfull
    cap = (cutoff_v - vfull) / vslope if vslope != 0 else float("nan")

    order = np.argsort(mah)
    return BatterySagFit(
        thrust_sensitivity_n_per_v=float(k),
        thrust_v_intercept_n=float(b),
        r=r,
        v_full=float(vfull),
        v_per_mah=float(vslope),
        capacity_to_cutoff_mah=float(cap),
        thrust_start_n=float(thrust[order][:50].mean()),
        thrust_end_n=float(thrust[order][-50:].mean()),
        voltage_start_v=float(voltage.max()),
        voltage_end_v=float(voltage.min()),
        n_samples=int(thrust.size),
    )


def _read(path, cols):
    out = {c: [] for c in cols + ["t_epoch", "phase"]}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            out["t_epoch"].append(float(row["t_epoch"]))
            out["phase"].append(row["phase"])
            for c in cols:
                out[c].append(float(row[c]))
    return {k: (np.asarray(v) if k != "phase" else np.asarray(v, dtype=object))
            for k, v in out.items()}


def load_sustained(tests_dir=_TESTS, settle_s=5.0):
    """Concatenate the seven runs (chronological): powered, settled thrust paired
    with interpolated voltage and cumulative mAh across the whole sequence."""
    runs = sorted(glob.glob(os.path.join(tests_dir, "A1850_B1850_*")))
    thrust_all, volt_all, mah_all = [], [], []
    mah_offset = 0.0
    for run in runs:
        if not os.path.isdir(run):
            continue
        bat = _read(os.path.join(run, "battery.csv"), ["voltage_v", "current_a"])
        lc = _read(os.path.join(run, "loadcell.csv"), ["Fz"])
        # Global charge from ALL battery samples in this run (idle current ~ 0).
        mah = mah_offset + coulomb_count(bat["t_epoch"], bat["current_a"])
        mah_offset = float(mah[-1])

        powered = lc["phase"] == _POWERED
        if not powered.any():
            continue
        t0 = lc["t_epoch"][powered].min()
        keep = powered & (lc["t_epoch"] > t0 + settle_s)
        t_lc = lc["t_epoch"][keep]
        thrust = -lc["Fz"][keep]
        volt = np.interp(t_lc, bat["t_epoch"], bat["voltage_v"])
        mah_lc = np.interp(t_lc, bat["t_epoch"], mah)
        thrust_all.append(thrust)
        volt_all.append(volt)
        mah_all.append(mah_lc)

    return (np.concatenate(thrust_all), np.concatenate(volt_all),
            np.concatenate(mah_all))


def main():
    os.makedirs(_OUT, exist_ok=True)
    thrust, voltage, mah = load_sustained()
    fit = fit_sag(thrust, voltage, mah)

    out_path = os.path.join(_OUT, "battery_sag.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(asdict(fit), f, indent=2)

    print("wrote", out_path)
    print("  thrust_sensitivity_n_per_v = %.3f  (r = %.3f)"
          % (fit.thrust_sensitivity_n_per_v, fit.r))
    print("  vehicle_params.yaml currently stores 1.46 -- re-derived above")
    print("  v_full = %.2f V,  v_per_mah = %.5f V/mAh,  capacity_to_9V ~ %.0f mAh"
          % (fit.v_full, fit.v_per_mah, fit.capacity_to_cutoff_mah))
    print("  thrust %.2f -> %.2f N over %.2f -> %.2f V"
          % (fit.thrust_start_n, fit.thrust_end_n,
             fit.voltage_start_v, fit.voltage_end_v))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
