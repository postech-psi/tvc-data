"""
Scattered interpolation over the coax (a_cmd_us, b_cmd_us) thrust/torque map,
so gaps in the measured grid (currently 36 of 72 cells -- the off-diagonal
region at A in {1700, 1800, 1850} has no measurements at all) don't block
downstream consumers such as a Gazebo thruster plugin or a PX4 SITL lookup
table.

Points are first normalized to one reference voltage (analyze.normalize_to_voltage)
before interpolating -- otherwise the interpolant conflates PWM effect with
whatever the pack voltage happened to be at each measurement, which is exactly
the confound docs/DATA_INVENTORY.md warns about for the raw sweep data.

Linear (Delaunay) interpolation is used inside the convex hull of measured
points; nearest-neighbor outside it. `extrapolated` in the query result says
which one you got -- an extrapolated point is a guess, not a measurement, and
callers should treat it with less confidence.
"""
import csv

import numpy as np


def load_map_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    num_fields = ("a_cmd_us", "b_cmd_us", "n", "n_eff", "thrust_N", "thrust_sd",
                  "thrust_sem", "torque_Nm", "torque_sem", "voltage_v",
                  "current_a", "power_w", "efficiency_N_per_W", "voltage_noload_v")
    for r in rows:
        for k in num_fields:
            v = r.get(k)
            r[k] = float(v) if v not in (None, "") else None
    return rows


def _aggregate(rows):
    """Effective-sample-weighted mean thrust/torque per (a, b) cell.

    Multiple runs can visit the same commanded point (different sweeps,
    different battery states) -- normalize_to_voltage already corrected for
    voltage, so what's left is measurement noise, which weighted averaging
    reduces rather than papers over.
    """
    groups = {}
    for r in rows:
        a, b = r.get("a_cmd_us"), r.get("b_cmd_us")
        if a is None or b is None or r.get("thrust_N") is None:
            continue
        groups.setdefault((a, b), []).append(r)

    pts, thrust, torque = [], [], []
    for (a, b), grp in sorted(groups.items()):
        w = np.array([g.get("n_eff") or 1.0 for g in grp])
        t = np.array([g["thrust_N"] for g in grp])
        pts.append((a, b))
        thrust.append(float(np.average(t, weights=w)))

        tq_grp = [(g["torque_Nm"], g.get("n_eff") or 1.0)
                  for g in grp if g.get("torque_Nm") is not None]
        if tq_grp:
            tq, tw = zip(*tq_grp)
            torque.append(float(np.average(tq, weights=tw)))
        else:
            torque.append(np.nan)

    return np.array(pts, dtype=float), np.array(thrust), np.array(torque)


class ThrustTorqueMap:
    """Query thrust(a, b) / torque(a, b) anywhere in PWM space."""

    def __init__(self, csv_path, v_ref=None, exponent=None):
        from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
        from .analyze import normalize_to_voltage

        rows = load_map_csv(csv_path)
        rows, self.v_ref, self.exponent = normalize_to_voltage(rows, v_ref, exponent)
        self.points, thrust, torque = _aggregate(rows)
        if len(self.points) < 3:
            raise ValueError(
                "need at least 3 measured (a, b) cells to interpolate; got %d "
                "from %s" % (len(self.points), csv_path))

        self._thrust_lin = LinearNDInterpolator(self.points, thrust)
        self._thrust_near = NearestNDInterpolator(self.points, thrust)

        have_torque = ~np.isnan(torque)
        if have_torque.sum() >= 3:
            self._torque_lin = LinearNDInterpolator(self.points[have_torque], torque[have_torque])
            self._torque_near = NearestNDInterpolator(self.points[have_torque], torque[have_torque])
        else:
            self._torque_lin = self._torque_near = None

    def query(self, a_us, b_us):
        a_us, b_us = float(a_us), float(b_us)
        t_lin = float(self._thrust_lin(a_us, b_us))
        extrapolated = bool(np.isnan(t_lin))
        thrust = float(self._thrust_near(a_us, b_us)) if extrapolated else t_lin

        if self._torque_lin is not None:
            tq_lin = float(self._torque_lin(a_us, b_us))
            torque_extrap = bool(np.isnan(tq_lin))
            torque = float(self._torque_near(a_us, b_us)) if torque_extrap else tq_lin
        else:
            torque, torque_extrap = None, True

        return {
            "a_cmd_us": a_us, "b_cmd_us": b_us,
            "thrust_N": round(thrust, 4),
            "torque_Nm": round(torque, 5) if torque is not None else None,
            "extrapolated": extrapolated or torque_extrap,
            "v_ref": self.v_ref,
        }

    def grid(self, a_values, b_values):
        return [self.query(a, b) for a in a_values for b in b_values]
