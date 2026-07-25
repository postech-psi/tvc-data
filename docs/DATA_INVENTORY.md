# Data inventory and voltage status

Generated from `runs/` by `python -m tvctools build` + `map`. Times are
bench-local (KST). Regenerate after adding new runs.

## Alignment: the three links

Nothing shares a clock, so three separate links are chained. Each is verified,
and the verification numbers are stored in `run.json` / `session.json`.

```
 load cell (STM32 uptime)          Pi (UTC epoch)            Pixhawk (boot us)
        │                                │                          │
        │  LINK 1                        │  LINK 2                  │  LINK 3
        │  filename (KST) + elapsed t_ms │  t_epoch & t_fc_us in    │  same boot
        │  then xcorr thrust vs current  │  the SAME thrust_map row  │  clock
        └───────────────►────────────────┴────────────►─────────────┘
```

**Link 1 — load cell → wall clock.** The stand logs only free-running MCU
uptime. Its filename gives the local start time, so
`epoch ≈ filename_epoch + (t_ms − t_ms[0])/1000`, good to about a second. That
is refined by cross-correlating thrust (`−Fz`) against the Pixhawk's `current_a`
— the only observable both systems see, since the stand's own pwm/rpm/current
columns are dead. One lag is fitted **per load-cell file** (the offset belongs to
the recording, not the sweep), taken from the longest overlapping sweep.
*Result: lags −0.85 s … +0.95 s, correlations 0.37–0.66.*

**Link 2 — wall clock → FC clock.** Every `thrust_map` row carries `t_epoch`
(Pi wall clock) **and** `t_fc_us` (Pixhawk boot clock). Their median difference
is the boot-to-epoch offset. This is the only bridge to the flight controller's
time base. *Result: independent sweeps in one log agree to 0.000–0.030 s.*

**Link 3 — FC clock → ulog.** The ulog uses that same boot clock, so link 2
dates it. Because the clock restarts near zero every boot, containment alone is
not proof; the match is confirmed by correlating the ulog's `actuator_outputs`
against the thrust_map's `servo*_raw` (≥ 0.9). Constant holds have no shape to
correlate and are located by epoch containment once a sweep has fixed the offset.

Accuracy overall is limited by link 1, i.e. **roughly ±0.1 s** — far finer than
the 2.4 s command steps, so step attribution is unambiguous.

## What exists, by date and time

`pwm` = commanded PWM + voltage/current. `thr` = thrust + torque.

### 2026-07-20 — load cell only, before the Pi drove the motor

| Time | Run | Dur | pwm | thr | Voltage | Note |
|---|---|---|---|---|---|---|
| 16:47:17 | `r01_164717` | 37.5 s | ✗ | ✓ | — | no Pixhawk data |
| 18:34:15 | `r01_183415` | 49.3 s | ✗ | ✓ | — | no Pixhawk data |
| 18:37:44 | `r02_183744` | 66.2 s | ✗ | ✓ | — | **only file with a real `pwm` column sweep** |

Unusable for mapping — no voltage record at all.

### 2026-07-23 — Pixhawk only

| Time | Run | Dur | pwm | thr | Voltage |
|---|---|---|---|---|---|
| 18:33:06 | `r01_183306` (`1000_F.csv`) | 88.0 s | ✓ | ✗ | — |

No thrust. Usable only as a PWM/voltage reference.

### 2026-07-24 session 1 (00:05–01:16) — the PWM sweeps

| Time | Run | Dur | pwm | thr | V high → low | Sag | ulog |
|---|---|---|---|---|---|---|---|
| 00:05:35 | `r01_000535_1200f` | 91.2 s | ✓ | ✗ | — | — | — |
| 00:23:15 | `r02_002315` | 160.5 s | ✓ | ✓ | 11.95 → 11.12 | **0.83 V** | log_3_..00-27-40 |
| 00:24:13 | `r03_002413` | 160.5 s | ✓ | ✓ | 11.68 → 11.04 | **0.64 V** | log_3_..00-27-40 |
| 00:51:08 | `r04_005108_1300` | 201.8 s | ✓ | ✓ | 11.44 → 10.84 | **0.60 V** | — |
| 01:04:14 | `r05_010414_1400` | 210.3 s | ✓ | ✓ | 11.27 → 10.72 | **0.55 V** | log_1_..01-16-18 |
| 01:13:50 | `r06_011350_1500_3s` | 142.6 s | ✓ | ✓ | 11.13 → 10.58 | **0.55 V** | log_1_..01-16-18 |

`r02` and `r03` share one continuous load-cell recording.

### 2026-07-24 session 2 (16:20–17:10) — the voltage-sag test

Each run is a single constant command held ~60 s, so voltage is **flat within a
run** and steps down between runs as the pack drains.

| Time | Run | Command | Dur | thr | Voltage |
|---|---|---|---|---|---|
| 16:20:57 | `r01_162057` | A1700_B1700 | 52.4 s | ✓ | 11.95 |
| 16:25:19 | `r02_162519` | A1800_B1800 | 27.8 s | ✓ | 11.67 |
| 16:30:20 | `r03_163020` | A1850_B1850 | 83.2 s | ✓ | 11.32 |
| 16:38:08 | `r04_163808` | A1850_B1850 | 80.8 s | ✓ | 11.12 |
| 16:41:54 | `r05_164154` | A1850_B1850 | 87.2 s | ✓ | 10.96 |
| 16:44:32 | `r06_164432` | A1850_B1850 | 80.7 s | ✓ | 10.84 |
| 16:47:30 | `r07_164730` | A1850_B1850 | 60.0 s | ✗ | — (no thrust) |
| 16:57:36 | `r08_165736` | A1850_B1850 | 78.1 s | ✓ | 10.63 |
| 17:01:59 | `r09_170159` | A1850_B1850 | 76.3 s | ✓ | 10.57 |
| 17:09:43 | `r10_170943` | A1850_B1850 | 82.2 s | ✓ | 10.46 |

All ten sit inside one log, `log_3_2026-7-24-17-11-46.ulg` (16:17:56–17:11:50).

## Voltage: state of charge vs transient sag

**These are two different things and must not be mixed.**

- **No-load voltage** — the pack with the motor off. This is the state of
  charge, and the only number that compares runs meaningfully.
- **Loaded voltage** — no-load minus an `I*R` drop under 20-25 A. The drop is
  *not* constant: measured across session 2 it ranged **0.66 V to 1.18 V**,
  because it scales with whatever throttle was applied.

Measured on session 2 (`log_3_2026-7-24-17-11-46.ulg`, 54 min):

| | |
|---|---|
| No-load voltage, start → end | **12.61 → 11.06 V** |
| Genuine pack depletion | **1.55 V** |
| Mean IR sag while running | 1.04 V |
| Loaded voltage range (all runs) | 10.29 – 12.58 V |

So the loaded-voltage spread is dominated by transient sag, not depletion.

No-load voltage is read from the ulog — the only source covering the idle
stretches between runs — as the median of battery samples with current < 1 A and
throttle at idle, in the 40 s before each run. It is stored per run as
`voltage_noload_v` in `run.json`.

### Per-run state of charge

| Session | Run | Time | **V no-load** | V loaded | IR sag |
|---|---|---|---|---|---|
| s1 | `r02_002315` | 00:23 | **12.02** | 11.95 | 0.07 |
| s1 | `r03_002413` | 00:24 | **11.90** | 11.65 | 0.25 |
| s1 | `r04_005108_1300` | 00:51 | *no ulog* | 11.38 | — |
| s1 | `r05_010414_1400` | 01:04 | **11.60** | 11.23 | 0.38 |
| s1 | `r06_011350_1500_3s` | 01:14 | **11.44** | 11.10 | 0.34 |
| s2 | `r01_162057` | 16:21 | **12.62** | 11.95 | 0.67 |
| s2 | `r02_162519` | 16:25 | **12.56** | 11.67 | 0.89 |
| s2 | `r03_163020` | 16:30 | **12.50** | 11.32 | 1.18 |
| s2 | `r04_163808` | 16:38 | **12.20** | 11.12 | 1.08 |
| s2 | `r05_164154` | 16:42 | **11.88** | 10.96 | 0.92 |
| s2 | `r06_164432` | 16:44 | **11.65** | 10.84 | 0.81 |
| s2 | `r07_164730` | 16:48 | **11.50** | — | — |
| s2 | `r08_165736` | 16:57 | **11.40** | 10.63 | 0.77 |
| s2 | `r09_170159` | 17:02 | **11.30** | 10.57 | 0.74 |
| s2 | `r10_170943` | 17:10 | **11.25** | 10.46 | 0.78 |

`r04_005108_1300` has no ulog covering it, so its state of charge is unknown —
it can only be placed by interpolation between `r03` (11.90 V) and `r05`
(11.60 V).

### The sag result, on the correct axis

Against **no-load** voltage the fit is tighter than against loaded voltage
(r = 0.980 vs 0.971), which is what you would expect from removing a confound:

```
A1850_B1850, 7 runs:  11.25 - 12.50 V no-load  ->  12.65 - 14.64 N
dThrust/dV = 1.435 N/V     thrust ~ V^1.24     r = 0.980
13.6 % of thrust lost across the discharge
```

## Why no existing run gives a clean thrust map

The two sessions fail in opposite ways:

- **Session 1** sweeps all of B inside one run, but the pack sags *while the
  sweep climbs*, so low-throttle points sit at high voltage and high-throttle
  points at low voltage. The curve lies along a voltage gradient rather than at
  one operating point.
- **Session 2** holds a steady state of charge within each run, but each run is
  a single (A, B) point at a different charge level — ideal for calibrating the
  sag, useless as a map on its own.

Session 2 calibrates the exponent; session 1 supplies the shape. Combine with:

```bash
python -m tvctools map --normalize --v-ref 11.5
```

`thrust_N` becomes the corrected value, `thrust_N_raw` keeps the original, and
`v_correction` records the factor. This is a first-order correction, **not** a
substitute for testing at constant voltage: it assumes one exponent everywhere
and cannot undo thermal drift.

## Protocol for the next thrust tests (no bench PSU)

Without a PSU the pack will drain, so the goal shifts from *preventing* drift to
*measuring* it and *decorrelating* it from PWM. All four options below are now
implemented in `pwm_thrust_map.py` and exposed in the web GUI.

1. **`무부하 기록(s)` = 10 s** (`idle_s`). Records `idle_pre` and `idle_post`
   rows with **both rotors at 1000 µs**, so the pack is genuinely unloaded. The
   voltage there is the state of charge, and `idle_pre − idle_post` is what that
   run actually consumed. This also removes the need for a ulog to get SoC.
2. **`계단 순서 섞기`** (`randomize`). Drift then adds scatter instead of a
   systematic slope, because voltage decay is no longer aligned with rising PWM.
   Set `seed` to a fixed number for a reproducible order.
3. **`첫 조합을 끝에 반복`** (`bracket`). The opening and closing measurement of
   the same command differ by exactly the drift accumulated during the sweep —
   a direct measurement rather than a model.
4. **Hold the state of charge in a band.** Compare runs only when their
   `idle_pre` voltages are close; recharge between sweeps rather than chaining
   them. This is the substitute for a constant-voltage supply.

Also: **4 s per step** (≈1.5 s settle + ≈2.5 s average), and **fill the
off-diagonal (A, B) cells** — coverage is 36 of 72, with A = 1700/1800/1850
existing only where A = B, so the coaxial torque map is incomplete exactly where
the balance point is most interesting.

Still unresolved: **no RPM sensor.** Without one, "voltage changes thrust" and
"voltage changes RPM which changes thrust" cannot be separated.

## After the change: one-step alignment

The three-link chain exists almost entirely to accommodate the ulog:

| Link | Needed because | Still needed without the ulog? |
|---|---|---|
| 1. load cell → wall clock | the stand logs only MCU uptime | **yes** — unavoidable |
| 2. wall clock → FC clock | to date the ulog | no |
| 3. FC clock → ulog | to date the ulog | no |

Drop the ulog and alignment collapses to link 1 alone: anchor on the load-cell
filename, refine by cross-correlating thrust against the Pi's current. That is
the only irreducible step, because the load-cell CSV carries no absolute time.

The one thing the ulog uniquely supplied was voltage during the idle stretches
between runs — and `idle_s` now puts that in the Pi's own log. `tvctools` reads
the Pi's idle phases first and falls back to the ulog only for older runs.
