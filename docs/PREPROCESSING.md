# Preprocessing: every step from raw file to map point

What happens between a raw CSV and a row in `out/pwm_thrust_torque_map.csv`, and
why each step exists.

```
raw load-cell CSV ──┐
                    ├─► 1 time base ─► 2 align ─► 3 resample ─► 4 segment
raw thrust_map CSV ─┘                                               │
                                                                    ▼
                                        7 map point ◄─ 6 average ◄─ 5 trim
```

---

## 1. Put both files on a common time base

The two systems share no clock.

| File | Its clock | Fix |
|---|---|---|
| `pwm/thrust_map_*.csv` | `t_epoch` — Pi wall clock, UTC | already absolute; used as-is |
| `loadcell/data_*.csv` | `t_ms` — free-running STM32 uptime | anchored on the filename |

For the load cell:

```
epoch[i] = filename_epoch + (t_ms[i] − t_ms[0]) / 1000
```

The filename is bench-local (KST, UTC+9), so a 9-hour shift is applied. This is
good to roughly a second — the delay between the GUI opening the file and the
first sample arriving.

## 2. Refine the anchor by cross-correlation

One second is too coarse when steps are 2.4 s, so the residual offset is fitted.
Cross-correlation needs one signal from **each** system:

- **Load cell → thrust (`−Fz`).** The only usable signal there; `pwm`, `rpm` and
  `Current_mA` are dead by design.
- **Pixhawk → commanded PWM (`b_cmd_us`), or current when the command is
  constant.** Selected in `align.drive_signal`.

Why commanded PWM first: it is the same *shape* as the thrust response, and it
correlates better (r = 0.62 vs 0.54 on the sweeps). Why current as fallback: a
60 s constant hold has zero command variance, so there is nothing to correlate —
current still fluctuates with the motor. Voltage is never used; it follows slow
battery drift and its lag estimates run to the edge of the search window.

The procedure (`align.estimate_lag`):

1. Resample both onto a common 20 Hz grid spanning their overlap ±20 s.
2. Median-filter the thrust over 0.25 s. **This is the one place filtering is
   used**, and only to sharpen the correlation peak — it never touches the values
   that get averaged later.
3. Z-score both signals, so the fit responds to shape rather than magnitude.
4. Scan the lag from −20 s to +20 s and keep the highest Pearson correlation.
5. If the best peak is below r = 0.25, return lag 0 and method `weak`: the coarse
   filename anchor stands rather than a confident wrong number.

One lag is fitted **per load-cell file**, not per run — the offset is a property
of the recording. It is taken from the longest overlapping sweep, because a short
sweep cross-correlated against a long recording containing several similar
staircases can lock onto the wrong one.

**Known systematic:** command-based and current-based lags disagree by ~0.3 s,
which is the motor's mechanical response time. Correlating against the command
absorbs that lag into the fit, so settled thrust lands in the correct step — good
for the map. Correlating against current gives a truer *clock* offset. Neither is
"wrong"; `run.json` records which was used in `drive_signal`.

### Cross-check: the single largest edge

Cross-correlation fits the whole record, which is what makes it robust to noise
but also what lets it lock onto the wrong cycle of a repetitive staircase (a
27.5 s sweep matched against a 160 s recording once produced a confident 19 s
error). So a second, independent estimate is taken from the **single biggest
command transition** and stored alongside.

Why one edge is worth anything: it is by far the highest-SNR feature in the
record. The final ramp-down moves thrust ~9 N against 0.7 N of noise — a 13-sigma
event — whereas an ordinary 100 us step moves it ~1 N and is only 1.6 sigma, not
individually locatable. Edges smaller than 3 N are rejected.

Both estimates use the same sign convention (offset to ADD to the load-cell
clock), verified against a synthetic record with a known injected shift: xcorr and
edge agreed to 0.03 s at +0.7, -0.4 and +1.3 s.

`run.json` records `edge_lag_s`, `edge_jump_n` and `edge_vs_xcorr_s`. On real
sweeps the difference is **+0.33 to +0.54 s**, which is the motor's mechanical
response time -- thrust genuinely lags the command, and the edge measures
command-to-thrust while the global fit averages over the whole staircase. A
difference beyond 1.5 s raises `edge_disagrees`, and on the current data exactly
one run trips it: the short sweep whose edge is also the weakest (3.3 N).

### Why not align on the first step alone?

Anchoring on one event and then walking forward by the known step durations is
attractive, and clock drift is genuinely negligible at these run lengths
(a 100 ppm crystal error is 0.02 s over 200 s), so one anchor is in principle
enough. Two things stop it being sufficient here:

1. **There is no large edge at the start.** These sweeps begin at B = 1000 with A
   already holding, so the first transition (1000 -> 1100) moves thrust 1.12 N --
   1.6 sigma, indistinguishable from noise. The big edge is at the *end*.
2. **Step durations must not be assumed.** The configured `dwell_s` is not what
   happens: on one run the actual mean step was 4.03 s with 0.097 s of jitter, and
   assuming the 2.0 s default would have accumulated 87 s of error over 43 steps.
   The Pi timestamps every sample, so the boundaries are already known exactly --
   they never need to be inferred from a duration.

Hence: use the Pi's logged timestamps for segmentation (which is what step 4
does), fit the offset globally, and keep the big end edge as the sanity check.

## 3. Resample onto one grid

Everything is interpolated onto a **50 Hz** grid — the load cell's native rate,
because thrust is the noisy channel and every sample counts. Downsampling to the
Pixhawk's 20 Hz would discard 60 % of the force data.

Two different rules, and the distinction matters:

| Columns | Rule | Why |
|---|---|---|
| `a_cmd_us`, `b_cmd_us`, `sweep_idx`, `phase` | **zero-order hold** | Commands are staircases. A commanded 1000 followed by a commanded 1100 was never 1043 in between; interpolating would invent values that were never sent. |
| `voltage_v`, `current_a`, `Fx…Tz`, `servo*_raw` | **linear interpolation** | These are continuous physical quantities that really did pass through intermediate values. |

Carrying the slower Pixhawk stream onto the faster grid adds no information, but
it invents none either.

Derived here: `thrust_N = −Fz` (the stand logs vertical load negative),
`torque_Nm = Tz`, `power_w = voltage_v × current_a`.

Output: `runs/<date>/<run>/merged.csv`.

## 4. Segment into steps — by the command, never by the thrust

Rows are grouped by the `phase` string (`A1400_B1700`); a new group starts
whenever it changes.

This is the central design decision. Within-step thrust noise is σ ≈ 0.7 N while
a 100 µs step changes thrust by only 0.65–1.5 N, so adjacent steps overlap at
1.1–2.1 σ and **step edges cannot be found in the thrust signal**. They do not
need to be: the Pi commanded those steps and logged exactly when. The boundaries
are known, not inferred.

## 5. Trim each step at both ends

| Trim | Amount | Reason |
|---|---|---|
| Leading | 50 % (`SETTLE_FRAC`) | motor and prop spin-up; the stand also rings |
| Trailing | 12 % (`TAIL_GUARD_FRAC`) | guards against alignment-lag error letting the *next* step bleed in |

The tail guard was added after measuring the effect: a ±0.3 s lag error biased
step means by up to 0.144 N without it, and 0.058 N with it — now safely under
the ~0.11 N standard error.

## 6. Average, robustly

1. **MAD outlier rejection** (`plot.mad_mask`, k = 3.5) — drops sensor glitches
   and dropouts, not vibration. Uses the median absolute deviation, so a few wild
   samples cannot drag the threshold out with them.
2. **Mean** of what survives.
3. **Uncertainty**, corrected for autocorrelation:

```
tau   = 1 + 2·Σ ρ(k)        integrated autocorrelation time
n_eff = n / tau
SEM   = sd / sqrt(n_eff)
```

The noise is propeller vibration, not white measurement error: lag-1
autocorrelation is ≈ 0.5 and tau ≈ 3–4 samples. Naive `sd/√n` therefore
understates the uncertainty by about 2×. Reporting `n_eff` keeps the error bar
honest.

### Why the thrust is *not* low-pass filtered

A moving average and a mean are both linear, so filtering before averaging is
very nearly a no-op. Measured across a sweep, filtering changed the step mean by
**0.0005–0.024 N** — against a 0.11 N standard error and 0.65–1.5 N step
increments. It does not recover a truer value because there is no extra
information to recover; the mean was already doing the smoothing.

What it *does* do is make the naive error bar look about 5× smaller, by inducing
correlation between neighbouring samples. That is false confidence, and undoing
it properly requires an autocorrelation window at least as long as the filter —
delicate to get right, and pointless when the mean is unchanged.

Filtering is therefore used only for **alignment** (step 2) and **plotting**,
never for the numbers.

The distribution is also close to symmetric (skew −0.6 … +0.4), so the mean is
sound and the median would only add noise.

## 7. Emit the map point

One row per step in `out/pwm_thrust_torque_map.csv`, carrying **both** rotor
commands — thrust and especially reaction torque depend on the coaxial pair, so a
single-PWM row would not identify the test point:

```
run, session, phase, a_cmd_us, b_cmd_us, n, n_eff,
thrust_N, thrust_sd, thrust_sem, torque_Nm, torque_sem,
voltage_v, voltage_noload_v, current_a, power_w, efficiency_N_per_W
```

Points are **not** averaged across runs: the same command at a different battery
state gives a different thrust, and collapsing runs would hide exactly the effect
`voltage_sag` measures.

## Tuning

All in `tvctools/analyze.py` and `tvctools/align.py`:

| Constant | Default | Effect |
|---|---|---|
| `SETTLE_FRAC` | 0.50 | more → cleaner steady state, fewer samples |
| `TAIL_GUARD_FRAC` | 0.12 | more → safer against lag error, fewer samples |
| `MIN_SAMPLES` | 8 | a step below this is dropped |
| `MAX_AC_LAG` | 8 | lags summed for tau |
| `GRID_HZ` (merge) | 50 | match the load cell |
| `MIN_CORR` | 0.25 | below this the lag fit is rejected |
| `MIN_CMD_SPAN_US` | 150 | command must move this much to be the reference |

With 4 s steps the trims leave ~1.5 s of settled data per step (≈ 75 samples at
50 Hz), which is comfortable.
