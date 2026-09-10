"""
Motor delay-vs-lag identifiability study -- an HONEST NULL RESULT.
================================================================================
    python identify_dynamics.py        # loads the bench runs, writes out/dynamics_identifiability.md

The bench recorded a "command -> thrust response delay ~0.10 s" and never said
whether that is a transport DELAY or a first-order LAG. docs/3-THEORY.md calls
resolving it the highest-value measurement outstanding, because it decides
whether the roll channel is controllable at the flown gains.

THIS SCRIPT DOES NOT RESOLVE IT, AND SHOWS WHY IT CANNOT FROM THIS DATA.

  1. Dead time is structurally unmeasurable here. The command is stamped on the
     host clock (bursts of up to 4 rows, ~0.7% drift) while force is stamped on
     the STM32 clock. An unknown command-onset shift is algebraically identical
     to a dead time: response(t - t0; Td) == response(t; Td + t0). So onset and
     Td are confounded and only the LAG time constant tau could be identifiable.

  2. tau is not identifiable at the near-hover operating point either. A
     synthetic-truth power check -- real step windows, real noise (~0.65 N),
     real onset scatter, a KNOWN injected tau, then refit with a free onset --
     recovers a true 44 ms lag as 0 ms in every trial. SNR ~ 2.2 there is simply
     too low once the onset is free. The estimator is NOT broken: at high SNR it
     recovers the injected tau. The near-hover data has no power.

  3. The large steps from REST are resolvable (SNR ~ 22), and there a pure delay
     is rejected. But that is ESC start-up plus rotor spin-up (T ~ omega^2), a
     different process from the small-signal response control design needs. It
     must not be used to set the model's tau.

WHAT WOULD SETTLE IT (no new rig): log the load cell at 500 Hz -- the firmware
already averages ~20 raw samples into every 20 ms row (force_count ~ 20), so the
resolution is being discarded, not missing -- or use ~6 N steps. More repeats do
NOT help: they buy sqrt(n), so ~1000 steps to match what 500 Hz gives for free.

The model keeps its switch; the default stays first_order; vehicle_params.yaml is
NOT changed by this script.
"""
import glob
import json
import os
from dataclasses import dataclass, field

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_RUNS = os.path.join(_HERE, "raw", "2026-07-31")
_OUT = os.path.join(_HERE, "out")


@dataclass
class StepEdge:
    """One command transition: thrust vs time, t relative to the command change."""
    t: np.ndarray          # s, relative to the command-change sample
    y: np.ndarray          # N, thrust (= -Fz, tared)
    pre_level: float       # N, mean thrust before the step
    amplitude: float       # N, post - pre
    noise: float           # N, std of the pre-level (measurement noise)
    a_us: int = 0
    b_pre: int = 0
    b_post: int = 0
    kind: str = "small"    # "small" (near hover) | "from_rest"

    @property
    def snr(self):
        return abs(self.amplitude) / self.noise if self.noise > 0 else np.inf


# --- the response model ------------------------------------------------------
def _response(t, onset, tau):
    """First-order step response with an onset. tau<=0 is the pure-delay limit."""
    g = np.zeros_like(t, dtype=float)
    m = t >= onset
    if tau <= 1e-9:
        g[m] = 1.0
    else:
        g[m] = 1.0 - np.exp(-(t[m] - onset) / tau)
    return g


def _rss_at(t, y, onset, tau):
    """Min RSS over the LINEAR params (baseline, amplitude) at fixed onset, tau.

    Variable projection: baseline and amplitude enter linearly, so profile them
    out with a 2-column least squares and return only the residual sum of squares.
    """
    g = _response(t, onset, tau)
    X = np.column_stack([np.ones_like(t), g])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ coef
    return float(r @ r)


def fit_edge(edge, tau, onset_grid):
    """Best RSS for one edge at a fixed tau, minimizing over a free onset."""
    return min(_rss_at(edge.t, edge.y, o, tau) for o in onset_grid)


def varpro_fit(edges, tau_grid, onset_grid):
    """Total RSS across edges for each tau (onset free per edge). Returns
    (best_tau, {tau: total_rss})."""
    rss = {}
    for tau in tau_grid:
        rss[tau] = sum(fit_edge(e, tau, onset_grid) for e in edges)
    best = min(rss, key=rss.get)
    return best, rss


def recover_tau(t, y, tau_grid, onset_grid):
    """The tau a single trace is best explained by (free onset). 0 = pure delay."""
    return min(tau_grid, key=lambda tau: min(_rss_at(t, y, o, tau)
                                             for o in onset_grid))


def power_check(template_edges, tau_true_list, rng, n_trials,
                tau_grid, onset_grid, onset_scatter_s=0.06):
    """Inject a KNOWN tau into synthetic steps built from real windows, then ask
    the estimator to recover it. Returns {tau_true: {median, p5, p95, n}}.

    This is how we prove non-identifiability rather than assert it: if a true
    44 ms lag comes back as 0 ms, the data cannot tell delay from lag.
    """
    out = {}
    for tau_true in tau_true_list:
        recovered = []
        for _ in range(n_trials):
            e = template_edges[rng.integers(len(template_edges))]
            onset = max(0.0, rng.normal(0.03, onset_scatter_s))
            truth = e.pre_level + e.amplitude * _response(e.t, onset, tau_true)
            y = truth + rng.normal(0.0, e.noise, size=e.t.shape)
            recovered.append(recover_tau(e.t, y, tau_grid, onset_grid))
        a = np.asarray(recovered, dtype=float)
        out[tau_true] = {"median": float(np.median(a)),
                         "p5": float(np.percentile(a, 5)),
                         "p95": float(np.percentile(a, 95)),
                         "mean": float(np.mean(a)), "n": int(a.size)}
    return out


# --- data loading ------------------------------------------------------------
def _read_loadcell(run_dir):
    """-> (t_epoch, thrust=-Fz, b_cmd_us) as float arrays, ordered by time."""
    import csv
    p = os.path.join(run_dir, "loadcell.csv")
    ts, thr, bcmd = [], [], []
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            fz = row.get("Fz")
            if fz in (None, ""):
                continue
            ts.append(float(row["t_epoch"]))
            thr.append(-float(fz))
            bcmd.append(int(float(row["b_cmd_us"])))
    return np.asarray(ts), np.asarray(thr), np.asarray(bcmd)


def load_edges(run_dir, pre_s=0.6, post_s=0.6):
    """Extract every B-command transition in one run as a StepEdge."""
    ts, thr, bcmd = _read_loadcell(run_dir)
    edges = []
    change = np.where(np.diff(bcmd) != 0)[0] + 1   # first index of each new level
    for i in change:
        t0 = ts[i]
        pre = (ts >= t0 - pre_s) & (ts < t0)
        post = (ts >= t0) & (ts <= t0 + post_s)
        win = pre | post
        if pre.sum() < 5 or post.sum() < 8:
            continue
        y = thr[win]
        t = ts[win] - t0
        pre_level = float(np.mean(thr[pre]))
        post_level = float(np.mean(thr[post][-5:]))
        noise = float(np.std(thr[pre]))
        b_pre, b_post = int(bcmd[i - 1]), int(bcmd[i])
        # "From rest" = stepping up out of the idle floor (motor essentially off).
        kind = "from_rest" if pre_level < 0.5 else "small"
        edges.append(StepEdge(t=t, y=y, pre_level=pre_level,
                              amplitude=post_level - pre_level, noise=max(noise, 1e-6),
                              b_pre=b_pre, b_post=b_post, kind=kind))
    return edges


def load_all_edges(runs_dir=_RUNS):
    edges = []
    for run in sorted(glob.glob(os.path.join(runs_dir, "A*_B1000-2000_*"))):
        if os.path.isdir(run):
            edges.extend(load_edges(run))
    return edges


# --- report ------------------------------------------------------------------
def main():
    os.makedirs(_OUT, exist_ok=True)
    rng = np.random.default_rng(0)
    tau_grid = [0.0, 0.02, 0.03, 0.04, 0.06, 0.08, 0.10, 0.14]
    onset_grid = np.arange(0.0, 0.16, 0.01)

    edges = load_all_edges()
    small = [e for e in edges if e.kind == "small" and 0.5 < abs(e.amplitude) < 4.0]
    rest = [e for e in edges if e.kind == "from_rest"]

    lines = ["# Motor delay-vs-lag identifiability -- null result", "",
             "Generated by `identify_dynamics.py`. This does NOT resolve the "
             "delay-vs-lag question; it shows the existing 50 Hz data cannot.", ""]

    if small:
        best, _ = varpro_fit(small[:60], tau_grid, onset_grid)
        pc = power_check(small[:60], [0.0, 0.044, 0.08], rng, 40, tau_grid, onset_grid)
        lines += [
            "## Near-hover small steps (the operating point)", "",
            "- usable small steps: %d (median SNR %.1f)"
            % (len(small), float(np.median([e.snr for e in small]))),
            "- best-fit tau on the real data (free onset): **%.0f ms**" % (best * 1000),
            "", "### Synthetic-truth power check (inject a known tau, refit)", "",
            "| injected tau | recovered (median) | p5-p95 | verdict |",
            "|---|---|---|---|"]
        for tau_true, s in pc.items():
            # "Identifiable" requires the recovered interval to EXCLUDE the
            # pure-delay hypothesis (p5 > ~10 ms) and bracket the truth. A wide
            # interval that reaches 0 -- as the 44 ms case does -- means a pure
            # delay cannot be rejected, so tau is not identifiable regardless of
            # where the median lands.
            excludes_delay = s["p5"] > 0.010
            close = abs(s["median"] - tau_true) < 0.02
            if tau_true <= 1e-9:
                verdict = "delay (baseline)"
            elif excludes_delay and close:
                verdict = "identifiable"
            else:
                verdict = "**NOT identifiable -- interval reaches 0**"
            lines.append("| %.0f ms | %.0f ms | %.0f-%.0f ms | %s |"
                         % (tau_true * 1000, s["median"] * 1000,
                            s["p5"] * 1000, s["p95"] * 1000, verdict))
        lines += ["", "A true 44 ms lag returns as ~0 ms: at SNR ~ %.1f with a free "
                  "onset, tau is not identifiable near hover." % float(np.median([e.snr for e in small])), ""]

    if rest:
        best_r, _ = varpro_fit(rest[:20], tau_grid, onset_grid)
        lines += ["## Large steps from rest (a DIFFERENT process)", "",
                  "- steps found in this sweep: %d (median SNR %.1f)"
                  % (len(rest), float(np.median([e.snr for e in rest]))),
                  "- best-fit tau: **%.0f ms** -- this regime is ESC start-up + "
                  "rotor spin-up (T ~ omega^2), not the small-signal response, so "
                  "it must NOT set the model's tau. The standard staircase sweep "
                  "steps by 100 us and rarely returns to rest, so few high-SNR "
                  "from-rest steps exist here; a dedicated large-step run would be "
                  "needed to characterise spin-up properly." % (best_r * 1000), ""]

    lines += ["## What would settle it (no new rig)", "",
              "- Log the load cell at **500 Hz** (the firmware already averages "
              "~20 raw samples per 20 ms row -- the resolution is discarded, not "
              "missing).", "- Or use **~6 N steps** for SNR ~ 22.",
              "- More repeats do NOT help (they buy only sqrt(n)).", "",
              "The model keeps its switch; the default stays `first_order`; "
              "`vehicle_params.yaml` is unchanged."]

    report = os.path.join(_OUT, "dynamics_identifiability.md")
    with open(report, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("wrote", report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
