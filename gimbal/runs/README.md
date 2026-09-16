# runs/ — data index

Layout: one folder per **test type**, each containing `session_<timestamp>/` captures.
Sessions that failed QC (`analyze.py` `pass: false`, or firmware-reported bad health)
are kept but moved into a `failed/` subfolder — nothing was deleted, only sorted out
of the way so the good data is easy to find.

```
runs/
  mapping_inner_gimbal/   <- gimbal angle-vs-PWM mapping (see below)
  mapping_outer_gimbal/
  chirp_*_gimbal/         frequency-sweep captures (bandwidth ID)
  deadband_*_gimbal/      servo deadband captures
  step_*_gimbal/          step-response captures
  grid_both_gimbals/      combined 2-axis grid sweep (only failed attempt so far)
  health/                 pre-flight IMU/servo health checks
  health_after_calibration/
  _sessions/              per-session REPORT.md / session_results.json / calibration_source.txt
                          (one entry per `run_experiment.py` run, ties the test-type
                          folders above back together by timestamp)
```

Each session folder has: `raw.csv` (raw capture), `meta.json`, `session.log`,
`analysis.json` (from `analyze.py`), and per-test outputs (`lut.csv`,
`mapping_points.csv`, `mapping_pretty.png`, `step_summary.csv`, etc).

## Gimbal mapping data (angle ↔ PWM LUT)

All sessions below passed QC (`pass: 1`). Numbers are nonlinearity / hysteresis
(lower is better) and acquisition cleanliness (i2c_errors, late_samples out of
~280-300k samples).

**Inner gimbal** — `mapping_inner_gimbal/`
| session | nonlinearity | hysteresis_max | i2c_errors | late_samples | note |
|---|---|---|---|---|---|
| `session_2026-08-12_224338` | 16.2% | 1.06 deg | 1 | 4 | first pass |
| `session_2026-08-13_004259` | 13.1% | 1.25 deg | 1 | 19 | cleanest acquisition |
| **`session_2026-08-13_042629`** | **9.1%** | **0.74 deg** | 9 | 974 | **best fit quality — recommended** |

**Outer gimbal** — `mapping_outer_gimbal/`
| session | nonlinearity | hysteresis_max | i2c_errors | late_samples | note |
|---|---|---|---|---|---|
| `session_2026-08-12_224338` | 12.6% | 0.43 deg | 0 | 0 | cleanest acquisition |
| **`session_2026-08-13_055016`** | **10.7%** | 0.57 deg | 3 | 441 | **best fit quality, latest — recommended** |

For processing (fitting a final LUT, feeding the PID/vehicle model), start from the
**recommended** session's `lut.csv` / `analysis.json` in each folder above. The other
passing sessions are kept as repeat measurements if you want to compare or average.

`mapping_inner_gimbal/failed/` and `mapping_outer_gimbal/*` also contain two inner-gimbal
attempts (`session_2026-08-13_055016`, `session_2026-08-13_061200`) that failed the
`late_samples` acquisition-jitter threshold by a small margin (~1.0-1.3% vs 1.0% allowed).
The fitted angle/gain numbers in those look plausible and close to the passing runs, so
they were archived rather than deleted — treat them as backup/comparison data, not
primary, since QC flagged the timing as marginal.

## Other archived failures (`*/failed/`)

- `health/failed/` — 4 aborted health captures where firmware reported bad IMU bias
  (`health_pass=0`); rig was still settling. Not usable, kept for reference only.
- `chirp_inner_gimbal/failed/` — one aborted chirp capture, superseded by
  `chirp_inner_gimbal/session_2026-08-13_053707` (passed).
- `grid_both_gimbals/failed/` — the only 2-axis grid attempt so far; failed on
  i2c errors and unsettled cells well past threshold (not a borderline case). No
  passing grid data exists yet.
