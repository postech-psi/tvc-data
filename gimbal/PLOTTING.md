# Plotting guide — which data to point `plot.py` at

All plots come from one CLI: `plot.py <cmd> --run <folder>`, where `<cmd>` is
`mapping`, `step`, `bode`, `deadband`, or `grid` and `<folder>` is one run folder
(e.g. `runs/mapping_inner_gimbal/session_2026-08-13_042629`). This file lists,
per command, which sessions are **usable** (passed QC — use these) and
which are **possibly usable** (failed QC by a small margin, or are raw/unanalyzed —
inspect before trusting). Full QC detail lives in `runs/README.md`.

Run all commands from the project root with the venv active:
`.venv\Scripts\python.exe plot.py <cmd> --run <folder>`

## Mapping (angle ↔ PWM LUT) — `plot.py mapping`

**Usable (recommended):**
```
python plot.py mapping --run runs/mapping_inner_gimbal/session_2026-08-13_042629
python plot.py mapping --run runs/mapping_outer_gimbal/session_2026-08-13_055016
```

**Usable (other passing repeats, for comparison):**
```
python plot.py mapping --run runs/mapping_inner_gimbal/session_2026-08-12_224338
python plot.py mapping --run runs/mapping_inner_gimbal/session_2026-08-13_004259
python plot.py mapping --run runs/mapping_outer_gimbal/session_2026-08-12_224338
```

**Possibly usable (failed QC on late_samples by ~1.0-1.4%, values look plausible):**
```
python plot.py mapping --run runs/mapping_inner_gimbal/failed/session_2026-08-13_055016
python plot.py mapping --run runs/mapping_inner_gimbal/failed/session_2026-08-13_061200
python plot.py mapping --run runs/mapping_inner_gimbal/session_2026-08-13_070621/mapping_inner_gimbal
```
The third one is a nested `run_experiment.py` session (point `--run` at the
`mapping_inner_gimbal` subfolder, not the session root) — failed on
late_samples 3856/281192 (1.37%), same borderline pattern as the other two.

## Grid (combined outer+inner PWM → angle surface) — `plot.py grid`

No real 2-axis grid capture has passed QC yet (`runs/grid_both_gimbals/failed/` only,
failed hard on i2c errors — not usable). Until a real grid capture passes, reconstruct
an approximate surface from the recommended mapping sessions above:
```
python plot.py grid --from-mapping --outer runs/mapping_outer_gimbal/session_2026-08-13_055016 --inner runs/mapping_inner_gimbal/session_2026-08-13_042629
```
This is a separable estimate (first-order coupling only) — treat it as a stand-in,
not a substitute for a real grid sweep.

Each grid run (`--run`, `--from-mapping`, or `--demo`) saves two clean single-surface
plots next to `grid_points.csv` — `grid_surface_pitch.png` and `grid_surface_yaw.png` —
each a smooth least-squares cubic fit through the measured points with the raw points
overlaid (`--show` opens an interactive window).

`--demo` and `--from-mapping` write to `grid_demo/` and `grid_from_mapping/` in the
project root (already generated — both present). A real `--run` grid capture writes
next to its own `grid_points.csv` under `runs/...` instead.

To plot any single surface (including the motor thrust map) from an arbitrary CSV,
use the generic `surface` command with explicit columns:
```
python plot.py surface grid_from_mapping/grid_points.csv --x cmd_outer --y cmd_inner --z pitch_deg --zlabel "Pitch [deg]"
```

## Step response — `plot.py step`

**Usable:**
```
python plot.py step --run runs/step_inner_gimbal/session_2026-08-12_232648
python plot.py step --run runs/step_outer_gimbal/session_2026-08-12_232648
python plot.py step --run runs/step_inner_gimbal/session_2026-08-13_072208/step_inner_gimbal
python plot.py step --run runs/step_outer_gimbal/session_2026-08-13_072533/step_outer_gimbal
```
The last two are nested `run_experiment.py` sessions — point `--run` at the
`step_*_gimbal` subfolder, not the session root. Both PASSed QC clean (0% sample
loss, 20/20 events); bandwidth 13.9 Hz inner / 9.8 Hz outer, both `recommend_chirp=True`.

**Not directly usable yet:** `runs/step_outer_gimbal/session_2026-08-13_045022` has
raw capture data (firmware reported pass) but was never analyzed — no
`step_summary.csv`/`analysis.json`. Run `analyze.py` on it first if you want it.

## Chirp (frequency sweep / bandwidth ID) — `plot.py bode`

**Usable:**
```
python plot.py bode --run runs/chirp_inner_gimbal/session_2026-08-13_053707
python plot.py bode --run runs/chirp_outer_gimbal/session_2026-08-13_053707
```

**Not usable:** `runs/chirp_inner_gimbal/failed/session_2026-08-13_051019` — failed QC,
superseded by the passing run above.

## Deadband — `plot.py deadband`

**Usable (2 passing repeats each, both fine to plot/compare):**
```
python plot.py deadband --run runs/deadband_inner_gimbal/session_2026-08-13_010241
python plot.py deadband --run runs/deadband_inner_gimbal/session_2026-08-13_053707
python plot.py deadband --run runs/deadband_outer_gimbal/session_2026-08-13_010241
python plot.py deadband --run runs/deadband_outer_gimbal/session_2026-08-13_053707
```
