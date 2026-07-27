# tvc-data

Coaxial motor thrust/torque bench data for the TVC model-rocket project, plus
the pipeline that turns raw logs into a PWM → thrust/torque map.

## Layout

```
raw/                        raw acquisition files, never edited
  2026-07-24/
    pwm/       thrust_map_<epoch>.csv    commanded PWM + voltage/current   (Pi)
    loadcell/  data_<date>_<time>.csv    thrust (Fz) + torque (Tz)         (stand)
    ulog/      log_*.ulg                 flight-controller log             (Pixhawk)
      _redundant/                        duplicates / no-motor logs, kept but ignored

runs/                       organized per-run copies + merged data  (generated)
  2026-07-24_s1/
    A1400_B1000-2000_0051_1300/
      pwm.csv        byte-exact copy of the source thrust_map file
      thrust.csv     byte-exact copy of the source load-cell file
      merged.csv     the two, time-aligned on one 50 Hz grid
      run.json       sources, alignment quality, steady-state table
      steps.png      raw thrust vs command-segmented step means

out/                        analysis products                      (generated)
  runs_index.csv/.json      catalog of every raw file
  pwm_thrust_torque_map.csv steady-state points, both rotor commands
  pwm_thrust_torque_map.png thrust/torque/efficiency vs B, one curve per A
  coax_grid.png             thrust and torque over the (A, B) plane
  voltage_sag.csv/.png      thrust vs battery state of charge

bench/                      new-format acquisition runs             (generated)
  <YYYY-MM-DD_HHMMSS>/      one directory per run -- see docs/ACQUISITION.md

docs/SETUP.md               what measures what -- read this first
docs/PREPROCESSING.md       every step from raw file to map point
docs/ACQUISITION.md         the tvcbench rewrite: one clock, one logger
docs/DATA_INVENTORY.md      what exists, by date, with voltage status
tvctools/                   the analysis pipeline
tvcbench/                   acquisition (Raspberry Pi)
plans/                      run plans -- a run is a checked-in file
tests/                      runs without hardware, via simulated sources
gui.py                      load-cell acquisition (bench laptop, superseded)
pwm_thrust_map.py           sweep runner + web GUI (Raspberry Pi, superseded)
pwm_map_gui.html            its control panel
plot.py                     standalone load-cell viewer
```

## Two generations

`pwm_thrust_map.py` + `gui.py` produced everything under `raw/` and `runs/`, with
the load cell on the bench laptop and the commands on the Pi — three clocks, and
an entire post-hoc alignment stage to reconcile them.

`tvcbench` replaces both. The load cell moves onto the Pi, so force and command
share one clock and no alignment is needed. Output goes to `bench/` in a new
format; the old pipeline and its data are untouched.
See [docs/ACQUISITION.md](docs/ACQUISITION.md).

```bash
python -m tvcbench selftest                       # hardware gate, before every session
python -m tvcbench plan show plans/coax_grid.yaml
python -m tvcbench run plans/coax_grid.yaml
```

Run folder names say what the test was: `A1400_B1000-2000_0051` means rotor A
held at 1400 µs, rotor B swept 1000→2000 µs, started 00:51. `A1850_B1850_1630`
means both rotors held at 1850 µs. Both commands appear because this is a
**coaxial** rig — thrust and especially reaction torque depend on the pair.

## Pipeline

```bash
pip install -r requirements.txt

python -m tvctools organize --dry-run   # file new raw data under raw/<date>/
python -m tvctools organize

python -m tvctools index                # catalog -> out/runs_index.csv
python -m tvctools build                # group + align + merge -> runs/
python -m tvctools map                  # thrust/torque map + plots -> out/
```

`organize` and `build` copy or move; they never edit a raw file. Every step is
idempotent, so re-running after adding data is safe.

To analyze one flight-controller log on its own:

```bash
python -m tvctools ulog raw/2026-07-24/ulog/log_3_2026-7-24-00-27-40.ulg
```

## Adding new data

1. Drop the files anywhere in the repo (or straight into `raw/<date>/<source>/`).
2. `python -m tvctools organize` — dates them from their contents and files them.
3. `python -m tvctools build && python -m tvctools map`.

`organize` reads the measurement date from inside each file where possible, so a
git checkout rewriting mtimes cannot misfile anything.

## What measures what

| Quantity | Source | Do **not** use |
|---|---|---|
| Commanded PWM (both rotors) | `pwm/` — `a_cmd_us`, `b_cmd_us`, `phase` | `pwm` in load-cell files |
| Voltage / current | `pwm/` — `voltage_v`, `current_a` | `Current_mA` in load-cell files |
| **Thrust** | `loadcell/` — `Fz`, newtons | — |
| **Torque** | `loadcell/` — `Tz`, N·m | — |

The load cell's own `pwm`, `rpm` and `Current_mA` columns are dead by design:
the Pi drives the motor through the Pixhawk, so the stand never sees the
throttle. `index` reports these under "By design", not as faults. Full detail in
[docs/SETUP.md](docs/SETUP.md).

## How alignment works

Full detail of every processing step is in
[docs/PREPROCESSING.md](docs/PREPROCESSING.md).

The stand logs free-running MCU uptime, the Pi logs UTC epoch. The load-cell
filename anchors it to ~1 s, then cross-correlation recovers the rest: thrust
(`−Fz`) against the **commanded PWM** on sweeps, falling back to **current** on
constant holds where the command has no variance to correlate. One lag per
load-cell file, recorded in `run.json` as `lag_s` / `lag_corr` / `drive_signal`;
a peak too weak to trust is reported as `weak` with lag 0 rather than a confident
wrong number.

Flight-controller logs have no usable absolute time — `time_ref_utc` is 0 and
the filenames are minutes wrong — so they are dated from the `t_fc_us` /
`t_epoch` pair in the `pwm/` files. One log spans many runs.

## Resolving PWM steps

Within-step thrust noise is σ ≈ 0.7 N while a 100 µs step changes thrust by
0.65–1.5 N, so a single sample cannot separate adjacent steps. Segmenting by the
*command* (which the Pi logged) and averaging the settled portion gives a
standard error of ≈ 0.11 N — a 6–14 σ separation. Error bars use the effective
sample size, since the vibration noise is autocorrelated. See each run's
`steps.png`.
