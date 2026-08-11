# Coaxial motor mapping bench

PWM → thrust/torque mapping for a coaxial rotor pair, measured on a Raspberry Pi 5
that drives a Pixhawk and reads a 6-axis load cell in the same process.

Everything in this repo serves one script: [`pwm_thrust_map.py`](pwm_thrust_map.py).
It runs on the Pi, serves a browser GUI, walks a 2-D grid of PWM commands as a
staircase sweep, and writes the raw CSVs in [`motor_raw/`](motor_raw).

```
pwm_thrust_map.py   the sweep logger + local web server  (runs on the Pi)
pwm_map_gui.html    the browser UI it serves             (must sit beside it)
motor_raw/          raw acquisition output — never edited
out/                derived maps and plots
plans/coax_grid.yaml  a sweep plan kept for reference
requirements.txt
"pi connection manual.md"  how to reach the Pi over Ethernet / Wi-Fi / hotspot
```

---

## 1. Why the load cell is on the Pi

Force used to be logged on the bench laptop while commands were sent from the Pi.
Two machines, two clocks, no shared timebase — so every run had to be
time-aligned afterwards by cross-correlation, good to about ±0.1 s and failing
outright on runs with little command variance.

`pwm_thrust_map.py` moves the load cell onto the Pi. The same process that sends
the command also stamps the force, both on `time.time()`. There is nothing left
to align. Data recorded this way lives in `motor_raw/2026-07-31/`; the older
two-machine data is in `motor_raw/Archive/` and still carries the alignment
problem (see §7).

One measured detail that matters: the STM32 load cell drops to ~10 Hz unless it
receives an `ARM <value>` heartbeat at 50 Hz. The script sends that heartbeat at
a fixed idle value of 1000 — it does not drive anything, it only keeps the load
cell sampling at its full 50 Hz.

## 2. Before the first run

Set these in QGroundControl. The script's µs ↔ normalized conversion assumes
them, and a mismatch makes every recorded command wrong:

- On the ESC channels: **Minimum = 1000, Maximum = 2000, Disarmed = 1000**
- **`THR_MDL_FAC = 0`** — turns off thrust-curve correction, so normalized ↔ PWM
  stays exactly linear

The Pixhawk must be **disarmed**. PX4 rejects `MAV_CMD_ACTUATOR_TEST` while armed.

Wiring defaults: Pixhawk on `/dev/ttyAMA0` at 921600 (Pi 5 GPIO UART, pins 8/10),
load cell on `/dev/ttyACM0` at 115200. Motor A is output function `Motor1`
(MAIN1), motor B is `Motor2` (MAIN2).

## 3. Running a sweep

```bash
python3 pwm_thrust_map.py
```

Then open `http://localhost:8000` on the Pi and configure the run in the GUI.
The server binds `127.0.0.1` only — connect to the Pi's desktop over RDP, or
tunnel with `ssh -L 8000:localhost:8000 ugrp@<pi-address>`.

Stop with `Ctrl+C` in the terminal: a run in progress stops the motors, ramps
down and closes the CSVs before exiting.

**Dry runs.** Set `no_motor` to exercise the entire timing, logging and load-cell
path with nothing sent to the Pixhawk. For logic-only testing off the bench, put
`udpin:0.0.0.0:14550` in the GUI's device field and point it at SITL or QGC.

### The sweep design

A is the fixed (outer) axis, B the swept (inner) one. Give each a start/end/step
and the grid is the product; setting start = end gives a single level. Each
`(A, B)` point is held for `dwell_s` and then the command steps straight to the
next point — no stop, no settling gap. Dwell is fixed-length only.

`repeats` walks the whole grid N times. With `randomize`, each repeat gets its
own fresh shuffle (a randomized complete block design) so that battery drain
doesn't correlate with command level — walked in order, every high-B point would
also be a low-voltage point and the two could never be separated.

`idle_pre_s` / `idle_post_s` record both rotors at 1000 µs, where current ≈ 0, so
the voltage there is the pack's state of charge free of IR sag. `idle_post` is
longer by default (30 s vs 5 s) because voltage relaxation after load takes
30–60 s.

The only safety cutoff is `min_voltage_v` — set it to 9.9 for a 3S pack; the
default of 0.0 disables it. Beneath that sits the real protection: an
`ACTUATOR_TEST` command expires after `timeout_s`, so **ceasing to send is itself
the stop**. A crash, a pulled cable, or a browser that stops polling (2 s
watchdog) all stop the motors by doing nothing.

## 4. Output

One folder per run, named `A<range>_B<range>_<date>_<time>/`:

```
motor_raw/2026-07-31/A1000_B1000-2000_2026-07-31_015904/
```

Each stream writes its own file at its own true rate — no carrying a 10 Hz value
forward into a 20 Hz row to fake resolution. All four CSVs repeat
`t_epoch, phase, sweep_idx, a_cmd_us, b_cmd_us, a_cmd_norm, b_cmd_norm` on every
line, so each one is analyzable on its own without a join.

| File | Written on | Rate | Payload beyond the common columns |
|---|---|---|---|
| `loadcell.csv` | each load-cell status line | ~50 Hz | `t_stm_ms`, `Fx_raw…Tz_raw`, `Fx…Tz` (tared), `force_count`, `torque_count` |
| `servo.csv` | each `SERVO_OUTPUT_RAW` | ~18–20 Hz | `t_fc_us`, `servo1_raw…servo8_raw` — **measured** µs, the ground truth |
| `battery.csv` | each `BATTERY_STATUS` | ~10–50 Hz | `voltage_v`, `current_a` |
| `esc.csv` | each `ESC_STATUS` | — | `esc1_rpm…esc4_rpm`; header-only without ESC telemetry |
| `run.json` | at completion | — | full config, grid, tare offsets, achieved Hz per stream, outcome |

Units: the load cell reports mN and mN·m; the script converts to N and N·m.
The stand reads vertical load negative, so **thrust = −Fz**. `Tz` is reaction
torque and changes sign as the coaxial pair balances.

Tare is non-destructive — both `Fz_raw` and tared `Fz` are written, and the
offsets are recorded in `run.json`, so a mis-tared run is still recoverable.

Check `achieved_rates_hz` in `run.json` after every session. If a stream came in
below its requested rate, it hit a PX4 publishing limit and the request needs
lowering rather than the data trusting.

## 5. Derived output

`out/` holds the maps built from the raw runs:

- `pwm_thrust_torque_map.csv` / `.png` — the main deliverable, thrust and torque
  over the `(A, B)` grid
- `sweep_run_summary_20260731.csv`, `torque_zero_crossing_20260731.csv`
- `voltage_sag*.csv` / `.png` — sag and recovery against the idle windows
- assorted `coax_grid` and identification plots

The scripts that generated these (`tvctools/`, `plot.py`) are no longer in this
repo — they were removed in the 2026-08-12 cleanup and are recoverable from git
history with `git restore tvctools plot.py`.

## 6. Data on disk

| Path | What | Format |
|---|---|---|
| `motor_raw/2026-07-31/` | Current runs, load cell on the Pi | per-run folders, §4 |
| `motor_raw/2026-07-31/A1850_B1850_TESTS/` | Repeat runs at a single point | same |
| `motor_raw/Archive/2026-07-20/` | Earliest thrust logs | `thrust_*.csv` |
| `motor_raw/Archive/2026-07-24_s1`, `_s2` | Two-machine sessions | `pwm.csv`, `thrust.csv`, `merged.csv`, `steps.png`, `run.json`, `session.json` |

Nothing under `motor_raw/` is ever edited. Everything else is rebuildable from it.

Date folders are the *measurement* date read from inside the files, not mtime —
a git checkout rewrites mtime.

## 7. Reading the Archive data

`motor_raw/Archive/` predates the integrated logger and does **not** share a
clock between force and command. Its `merged.csv` files are the output of the old
alignment pipeline, not raw measurement, and their accuracy varies by run — some
carry a `lag_method: "weak"` marker in `run.json` meaning alignment effectively
failed. Treat those runs as indicative, not quantitative, and prefer the
2026-07-31 data for anything that needs real timing.

The `.ulg` files from these sessions are gitignored (they run to ~180 MB, and
GitHub rejects single files over 100 MB) and stay on local disk.


## 8. Dependencies

`pwm_thrust_map.py` needs only **`pymavlink`** and **`pyserial`**. The web
server, CSV writing, threading and JSON are all standard library.

```bash
pip install -r requirements.txt
```
