# Bench setup and data provenance

Authoritative description of what measures what. Read this before analyzing
anything — several columns in the raw files are vestigial, and treating them as
real is the easiest way to get a wrong answer.

## Architecture

```
   ┌──────────────────┐   MAVLink (UART, 921600)   ┌──────────────────┐
   │  Raspberry Pi 5  │ ─────────────────────────► │     Pixhawk      │
   │ pwm_thrust_map.py│ ◄───────────────────────── │   (PX4, FMU_V6C) │
   └──────────────────┘   SERVO_OUTPUT_RAW 20 Hz   └──────────────────┘
        │  commands            BATTERY_STATUS 10 Hz     │  PWM out
        │  the PWM                                      ▼
        │                                          ┌──────────┐
        │  writes                                  │ ESC+motor│
        ▼                                          └──────────┘
   thrust_map_<epoch>.csv                               │ thrust
                                                        ▼
   ┌──────────────────┐   USB serial 115200    ┌──────────────────┐
   │  bench laptop    │ ◄───────────────────── │  STM32 + load    │
   │     gui.py       │                        │  cell / torque   │
   └──────────────────┘                        └──────────────────┘
        │  writes
        ▼
   data_<date>_<time>.csv
```

The Pixhawk also writes its own `.ulg` to SD continuously across a whole
session.

## Who is authoritative for what

| Quantity | Authoritative source | Do **not** use |
|---|---|---|
| Commanded PWM | `thrust_map_*.csv` (`a_cmd_us`, `b_cmd_us`, `phase`) | `pwm` in `data_*.csv` |
| Measured PWM output | `thrust_map_*.csv` (`servo1_raw`, `servo2_raw`) or the `.ulg` | — |
| Battery voltage / current | `thrust_map_*.csv` (`voltage_v`, `current_a`) or the `.ulg` | `Current_mA`, `ADC_Current_mA` in `data_*.csv` |
| **Thrust** | `data_*.csv` — `Fz`, in newtons | — |
| **Torque** | `data_*.csv` — `Tz`, in N·m | — |
| RPM | *not currently measured* | `rpm` in `data_*.csv` |

### Why the stand's own columns are dead

`gui.py` was originally written to drive the ESC itself and to read RPM and
current from the STM32. In the current architecture the **Pi commands the motor
through the Pixhawk**, so:

- `pwm` sits at 1000 forever — the STM32 never sees the throttle command.
- `rpm` is 0 or garbage — no RPM sensor is wired.
- `Current_mA` is 0 and `ADC_Current_mA` is empty — the Pixhawk's battery
  monitor is the current/voltage source.

**This is expected, not a fault.** `tvctools index` reports these as
`gui_pwm_unused` / `gui_rpm_unused` / `gui_current_unused` under a "By design"
heading, separate from real problems.

The practical consequence: `plot.py`'s `aggregate_by_pwm()` groups by the `pwm`
column and **cannot be used on any 2026-07-24 file** — every sample would land
in a single bin. PWM must come from the paired `thrust_map` file. Only
`raw/2026-07-20/loadcell/data_20260720_183744.csv`, recorded before the Pi took over, has a real
PWM sweep in it.

## The two problems this creates

### 1. No shared clock

| Stream | Clock | Rate |
|---|---|---|
| `thrust_map_*.csv` | `t_epoch` — Pi wall clock, **UTC** | 20 Hz |
| `data_*.csv` | `t_ms` — free-running STM32 uptime, **not** epoch | 50 Hz |
| `.ulg` | PX4 boot time, µs | 5–10 Hz |

Nothing links them. The load cell's filename is written in **bench-local time
(KST, UTC+9)** while the Pixhawk log's epoch is UTC — a 9-hour offset that must
be applied before anything lines up. See "How alignment works" in the top-level
[README](../README.md).

### 2. Thrust noise exceeds the step size

Measured on this bench, within a held command step:

| | value |
|---|---|
| Within-step thrust noise (σ) | ≈ 0.7 N |
| Thrust change per 100 µs step | 0.65 – 1.5 N |
| Separation between adjacent steps | **1.1 – 2.1 σ** |

So a single sample genuinely cannot tell one PWM step from the next, and looking
for step edges *in the thrust signal* does not work.

It is also unnecessary: the Pi commanded those steps and logged them with
timestamps, so **segment by the command, not by the thrust**. Averaging the
settled portion of each step gives a standard error of ≈ 0.11 N against
increments of 0.65–1.5 N — a 6–14 σ separation.

The noise is propeller/motor vibration, not white measurement error: lag-1
autocorrelation is ≈ 0.5 and the integrated autocorrelation time is ≈ 4 samples.
Naive σ/√n therefore understates the error by about 2×, so `tvctools` divides by
the *effective* sample size instead. Error bars are wider and honest.

Verify visually with the per-run `steps.png`: raw trace in grey, command
staircase in blue, step means with error bars in red. On `r04_005108_1300` the
same step repeats to within ~1 % across four sweeps (2.85 / 2.84 / 2.83 / 2.98 N).

## Do the Pi CSV and the ulog agree on voltage/current?

Yes. Both come from the same `BATTERY_STATUS` field but via different paths --
the Pi reads it live over the MAVLink telemetry link, PX4 logs its internal
`battery_status` uORB topic to SD independently -- so they are never
bit-identical, but they measure the same signal and track it closely.

Verified with `python -m tvctools verify`, matching samples by `t_fc_us` (the
shared flight-controller clock) within 30 ms, across all 14 runs with a ulog:

- **Voltage: no systematic offset.** Per-run mean difference is -0.016 to
  +0.010 V, straddling zero.
- **Current: no systematic offset**, mean differences -0.08 to +0.15 A.
- **Per-sample scatter is real but expected**: dV up to ~0.3 V, dI up to ~7 A.
  This is not disagreement -- it is two independent samples of a signal that is
  itself moving (during a throttle step, current swings several amps in
  fractions of a second, so even a well-matched 10-30 ms offset lands on
  different points of the transient).
- **Per-phase (steady-state) means agree to ~0.02 V**, confirmed directly by
  averaging both sources within each held command and comparing.

Conclusion: use either source for voltage/current with confidence. The pipeline
prefers the Pi CSV (`thrust_map_*.csv`) because it is higher rate (up to 50 Hz
vs the ulog's 20 Hz battery_status) and it is what step segmentation is keyed
to; the ulog is not a second, disagreeing measurement.

## Coaxial rig: both rotor commands matter

This is a **coaxial counter-rotating** setup. `a_cmd_us` (rotor A) and
`b_cmd_us` (rotor B) are independent inputs, and the outputs depend on both:

- **Thrust** rises with either rotor.
- **Reaction torque `Tz` changes sign.** It is the *difference* between the two
  rotors' drag torques, so it crosses zero when they balance. Measured: at
  A = 1300 µs, Tz runs −0.03 → +0.15 N·m as B goes 1000 → 2000, crossing zero
  near B ≈ 1300. The zero-torque line sits close to A ≈ B, as expected.

Because of this, **never collapse the map onto a single PWM axis** — a
thrust-vs-PWM curve averaged over A hides the torque behaviour entirely.
`tvctools map` therefore emits:

- `pwm_thrust_torque_map.csv` — every steady-state point with both `a_cmd_us`
  and `b_cmd_us`
- `pwm_thrust_torque_map.png` — thrust/torque/efficiency vs B, one curve per A
- `coax_grid.png` — thrust and torque as 2-D maps over the (A, B) plane

Current coverage is 36 of 72 (A, B) cells: A ∈ {1300, 1400, 1500, 1700, 1800,
1850}, B ∈ 12 values. The A = 1700/1800/1850 points are diagonal-only (A = B).
Filling the off-diagonal cells is what would make the torque map complete.

## Sampling rates

| Stream | Raw rate | Notes |
|---|---|---|
| Load cell (`data_*.csv`) | **50 Hz** | uniform across all 16 files; `t_ms` steps of exactly 20 ms |
| `thrust_map_*.csv` | 20 Hz servo, 10 Hz battery | |
| `.ulg` `actuator_outputs` | 10 Hz | |
| `.ulg` `battery_status` | 5 Hz | |

The merge grid is **50 Hz**, matching the load cell. Thrust is the noisy channel
that benefits most from every available sample; downsampling to 20 Hz threw away
60 % of the force data. Commands are zero-order held onto the finer grid (exact,
since they are staircases) and voltage/current interpolated, so carrying the
slower Pixhawk streams onto the faster grid invents nothing.

Note the ulog is the **lowest**-rate source of PWM and voltage, at half the
thrust_map rate.

## Why `thrust_map_*.csv` cannot be replaced by the ulog

The PWM and voltage *values* are indeed duplicated in the ulog, but the ulog has
no usable absolute time:

- `time_ref_utc` is 0 in all six logs — no GPS time reference.
- **The filename is wrong**, by 6 s to 54 min. `log_3_2026-7-24-00-27-40.ulg`
  actually starts at 00:21:42; `log_3_2026-7-24-17-11-46.ulg` starts at 16:17:56.
- The `/fs/microsd/log/...` message records the first log of the boot, not the
  file it appears in.

`thrust_map_*.csv` carries `t_fc_us` (the FC boot clock) **and** `t_epoch` (the
Pi's wall clock) on every row. Their median difference is the boot-to-epoch
offset, and it is the only thing that dates a ulog. Verified agreement between
independent sweeps in the same log: **0.000–0.030 s**.

Delete `thrust_map_*.csv` and the ulogs become un-timestampable, so they can
never be aligned with the load cell. Keep them. The ulog remains valuable as an
independent cross-check and for the idle/recovery periods outside each run.

### One ulog holds many runs

A log spans an entire boot session, so it contains several runs.
`log_3_2026-7-24-17-11-46.ulg` covers all ten runs of session 2 across 54
minutes. `tvctools build` records, per run, which log covers it and the FC-time
window to slice (`ulog` block in `run.json`).

Runs are located inside a log two ways: a swept command is matched by
correlating its PWM trace (corr ≥ 0.9), and a **constant hold** — which has no
shape to correlate — is located by epoch containment once a sweep has fixed the
offset. Matching a constant hold on PWM *level* was tried and rejected: an idle
1000 µs matches every quiet stretch of every log.

### Duplicate logs

`log_1_2026-7-24-01-07-24.ulg` is a byte-identical **prefix** of
`log_1_2026-7-24-01-16-18.ulg` — the same boot downloaded twice. The shorter
copy is flagged `duplicate_of` and excluded from per-run attachment. Two logs
(`log_2`, `log_4`) contain **no motor activity at all** and cannot be dated.

## Test campaigns on record

| Session | Local time | What it is |
|---|---|---|
| `2026-07-20_s1`, `_s2` | 16:47, 18:34 | Early stand-alone load-cell runs, before the Pi drove the motor. `raw/2026-07-20/loadcell/data_20260720_183744.csv` has a genuine PWM sweep. |
| `2026-07-23_s1` | 18:33 | `1000_F.csv` — A axis fixed at 1000. Pixhawk data only, no load cell. |
| `2026-07-24_s1` | 00:05 – 01:16 | **PWM → thrust/torque sweeps.** A fixed at 1300/1400/1500, B swept 1000→2000 in 100 µs steps, repeated 3–4×. |
| `2026-07-24_s2` | 16:20 – 17:10 | **Voltage-sag test.** Constant command, repeated as the pack drained. Seven holds at `A1850_B1850`, 11.32 V → 10.46 V. |

## Reading the voltage-sag result

`tvctools map` reports thrust vs voltage at constant PWM, but only where the
voltage differences are real. Two confounds are excluded automatically:

1. **Repeated sweeps inside one run** are the same battery state seconds apart.
   Counting them separately fakes a big sample across a tiny voltage span, so
   points are averaged per run first.
2. **Within a sweep, voltage is low *because* thrust is high** — the motor's own
   current causes the sag. Correlating the two measures reverse causation and
   produces a negative slope, which is physically impossible for a draining
   pack. Groups spanning < 0.4 V, or fitting a negative slope, are rejected.

Only `A1850_B1850` survives, which is exactly the intended sag test:

```
10.46 - 11.32 V  ->  12.68 - 14.64 N
dThrust/dV = 2.15 N/V     thrust ~ V^1.71     r = 0.972
13.4 % of thrust lost across the discharge
```

The exponent near 2 is the expected result: momentum theory gives thrust ∝ RPM²,
and RPM ∝ voltage for a fixed-pitch prop on a constant duty command.

Run `python -m tvctools map --show-rejected` to see why each other group was
excluded.

## Pixhawk configuration required

From the header of `pwm_thrust_map.py`:

- QGC actuator outputs: `Minimum = 1000`, `Maximum = 2000`, `Disarmed = 1000`
- `THR_MDL_FAC = 0` (no thrust-model linearization — the map must measure the
  raw relationship)
- The FC must be **disarmed**; motion is driven by `MAV_CMD_ACTUATOR_TEST` (310)
- Telemetry on `/dev/ttyAMA0 @ 921600` from the Pi 5

## Recording new runs

Set the output folder and the free-text metadata in the web GUI
(`pwm_map_gui.html`) so runs no longer land unlabelled in the working directory.
Settings that never appear in the CSV — `dwell_s`, `repeats`, grid range, prop,
battery, notes — are written beside it as `<name>.run.json`, which `tvctools`
reads back into the catalog.

For `gui.py`, set `CSV_OUT_DIR` in the file or `TVC_CSV_DIR` in the environment.

Recommended step timing, from the settling analysis: **4 s per step** (≈1.5 s
discarded as transient + ≈2.5 s averaged). The current 2.4 s works but leaves
little margin.
