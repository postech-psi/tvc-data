# TVC Servo & Actuator Characterization

Bench characterization of the thrust-vector-control gimbal actuator.
**Status (Aug 2026):** hardware verified, test code written and validated,
measurement data not yet collected.

The output of this phase is the actuator model that feeds the TVC simulator
and sets the ceiling on attitude-loop gain.

---

## Scope

Two measurements, deliberately:

1. **Mapping** — servo pulse width (µs) ↔ actual TVC plate angle (deg)
2. **Delay** — dead time and lag, from step response

Cut as not worth the schedule cost right now: dedicated sine-sweep frequency
response (the step test gives the same parameters to ~30 %, enough to pick a
control bandwidth), slew-rate characterization as a goal in itself (falls out
of the step data for free, and likely never binds within the gimbal's travel),
durability cycling, thrust-load effects, gyroscopic coupling (all deferred to
static fire).

Dead time is the one thing worth keeping from the "full" battery: it sets a
hard ceiling on achievable control bandwidth and no amount of gain tuning
recovers it.

---

## Hardware

| Item | Part | Notes |
|---|---|---|
| MCU | ESP32-WROOM-32D DevKit (KS0413) | servo A **GPIO 18**, servo B **GPIO 19**, I2C **SDA 21 / SCL 22** |
| Servo | 2× PTK 8515MG-D | 500–2500 µs, neutral **1520 µs**, 333 Hz, deadband 2 µs |
| Sensor | Adafruit ICM-20948 | I2C **0x69**, mounted on the motor plate |
| Power | separate BEC 7.4 V, 3 A+ | 1000 µF across servo V+/GND, **common GND mandatory** |

Servo at 7.4 V: 0.104 s/60° → **≈ 577 °/s** slew ceiling, 13.5 kg-cm stall.
Neutral is 1520 µs (Futaba convention), not 1500. Run the servo at **333 Hz** —
it cuts PWM frame delay from 10 ms (50 Hz) to 3 ms.

### Rig requirements

- **Real motor (or equal dummy mass) mounted.** `ω_n ∝ 1/√J` and `ζ ∝ 1/√J`,
  so the load makes the servo slower *and* less damped at the same time.
  Characterizing without it measures a servo you will never fly.
- **Sweep axis horizontal.** The accelerometer reads tilt from gravity; a
  vertical rotation axis reads nothing and Test A returns garbage.
- IMU bolted rigidly, axes parallel to the gimbal axes. Hot glue only as thin
  corner fillets with the board seated on a rigid face — never a thick bond
  line, and not for flight (softens ~60–80 °C).
- Cables routed with slack near the pivot. A stiff cable is a torsion spring
  and shows up as fake backlash.

### Why the IMU sits on the motor plate

The plate measures the **actual thrust vector angle** end-to-end — servo
nonlinearity, linkage geometry, and backlash all folded into the one
`pulse → θ_TVC` curve the controller needs. Motor and plate are one rigid
body, so mounting on the motor adds nothing and costs heat, vibration, and
lever arm. Position on the plate barely matters; center is mildly preferable.

---

## Sensor strategy

| Test | Sensor | Why |
|---|---|---|
| Mapping (static) | **accelerometer** | tilt from gravity, plate at rest |
| Step (dynamic) | **gyro** | rate measured directly, no differentiation noise |

During a step the accelerometer is unusable — tangential and centripetal
acceleration from the lever arm contaminate the gravity vector.

Raw registers only. Gyro ±2000 dps, DLPF cfg 7 (BW 361 Hz, group delay
0.17 ms), ODR 1125 Hz; accel ±2 g, DLPF cfg 7, ODR 1125 Hz.
**Fusion / quaternion output is forbidden** — its internal filter adds
20–50 ms of delay, the same magnitude as the quantity being measured.

---

## Files

| File | Purpose |
|---|---|
| [char.ino](char.ino) | everything — jog, IMU health, and tests A / B / K / P |
| [capture.py](capture.py) | serial → run directory (pyserial, 460800 baud) |
| [analyze.py](analyze.py) | health audit, model fitting, plots (needs numpy/pandas/scipy/matplotlib) |
| [imu test.ino](imu%20test.ino) | standalone IMU bench, for debugging the sensor with no servo attached |

**Design rule: the firmware measures and dumps, the PC decides.** Nothing is
averaged, filtered, bias-corrected, or fitted on the ESP32. Every sample
leaves the chip raw, in LSB, with its own timestamp and health flags. That is
what makes a run reanalyzable without repeating it — the mapping test emits
every sample of every dwell rather than one mean per point, so the scatter
within a dwell *is* the repeatability number and the settling is visible
instead of assumed.

Nothing is written to flash on the ESP32. Test B samples at 1 kHz into a RAM
buffer (printing inside the loop would wreck the timing), then dumps CSV over
serial after each step; the PC captures it to disk.

### Tests

| Cmd | Test | What it gets you |
|---|---|---|
| `a` | **A — mapping** | pulse ↔ angle, 4-pass sweep (up/dn/up2/dn2): gain, true neutral, nonlinearity, hysteresis, cross-axis |
| `b` | **B — step** | dead time, lag, slew, saturation check |
| `k` | **K — deadband** | 1 µs increments at 3 centers: real command resolution with the linkage loaded |
| `p` | **P — repeatability** | same target approached from both sides, 20×: separates backlash from repeatability |
| `l` | L — limit suggestion | *advisory only* — feels for the stops and prints suggested limits; never applies them |
| `c` | IMU health | noise floor, |g|, throughput, per-axis pass/fail |
| `t` | tap test | mount rigidity — a rigid IMU rings down in ~0.2 s |
| `z` | live readout | rig orientation check |

Tests K and P are new relative to the original plan. They exist because a
sweep alone cannot separate backlash from repeatability — it only ever sees
their sum — and because the datasheet's 2 µs deadband is an unloaded number.

### Travel limits are yours to set, and they are enforced

A servo commanded past its mechanical stop does not fault — it holds full
torque against the stop for as long as the command stands (500 ms per point,
56 points, 4 passes in Test A). That strips gears, browns out the BEC, and
logs *plausible* data, because a plate held against a stop reports a
beautifully stable angle. So the limits are not optional and not guessed:

- **You set them.** Edit `LIMIT_A_LO/HI`, `LIMIT_B_LO/HI` and set
  `LIMITS_PRESET 1`, then re-flash — the values live in source, in git, with
  the rig. Or type `w <lo> <hi>` at the prompt (saved to NVS, survives reset).
  Enter the *usable* range, i.e. the stop minus a 30–50 µs margin.
- **They are enforced by abort, never by clamp.** Every test checks its whole
  intended pulse range against your limits up front and refuses to run if it
  doesn't fit — printing what it wanted and what the travel is. It never
  silently clamps, because a clamped point is a stalled servo logged as good
  data. Until you set limits, the firmware allows only a deliberately tiny
  ±80 µs window, enough to jog and sanity-check, not enough to break anything.
- `l` (Test L) is a **second opinion**: it creeps toward each stop watching
  the plate stop responding, and *prints* a suggestion. It never writes.
  Cross-check it against your hand-measured numbers; if they disagree, trust
  the rig, not the search.

Steps in Test B and the centres in K/P are sized from *your* measured
half-travel about the mechanical centre — which need not be 1520 if the
travel is asymmetric — so the 100 % step lands just inside the stop instead
of slamming into it.

### IMU health runs automatically

At boot the firmware checks the IMU with no command from you: is it alive
(WHO_AM_I = 0xEA), is the read throughput > 1 kHz, and — held still — is the
per-axis gyro noise, accel noise, and |g| within spec. It reports PASS/FAIL
and, on failure, points at the usual cause (a non-rigid mount). The gyro
**bias is measured but never subtracted** — the analysis recovers bias and
drift from each trace's own windows, which a startup calibration can't see —
but it travels in every run's `#META` as an independent cross-check. Re-run
it any time with `c`, and `t` for the tap test.

### Output format

One schema for every test, so one parser:

```
#META key=value ...      run manifest
#COLS rec,t_us,phase,seq,axis,cmd_a,cmd_b,ax,ay,az,gx,gy,gz,flags
E,...                    a command was written, at this exact micros()
S,...                    a sample
```

Two things make this robust:

- **`#META` is self-describing.** Servo config, scale factors, and the IMU
  config registers *read back from the chip* all travel with the data.
  `analyze.py` duplicates no constant from the sketch, so the two cannot
  drift apart — which is exactly how `AXIS_MAIN` got out of sync before.
- **Command instants are logged, not assumed.** Every `writeMicroseconds`
  emits an `E` row carrying the `micros()` value of the write. Dead time is
  measured against that, never against a nominal `B_PRE_MS`.

`flags` carries bit0 = I2C read failed, bit1 = sample loop missed its
deadline, per sample — so a bad read is visible rather than silently
zero-filled.

Axis assignment is **derived from the data**, not from a `#define`: gravity
axis = largest mean accel component, main axis = the one that varies most
across the run. `analyze.py` prints what it picked, and warns if the main
axis barely moved (the signature of a vertical sweep axis).

**The IMU does not need to be mounted parallel to the gimbal axes.** Because
each Test A sweep drives one servo alone, the gravity vector traces a circular
arc whose plane normal *is* that servo's true rotation axis in sensor
coordinates — recovered from the data, whatever angle the IMU is glued at.
`analyze.py` extracts that axis, reports the misalignment in degrees, and
measures the plate angle as rotation *about the measured axis* rather than
`atan2` of two assumed-parallel sensor axes. Cross-axis coupling is then the
rotation about the in-plane perpendicular — the tilt the driven servo should
*not* produce. Validated on synthetic data: a 15°/20° crooked mount recovers
the true 16.5° travel span to within 0.2° and drops apparent coupling to
0.06%. (If a sweep is too small to define a plane, it falls back to the naive
two-axis method and says so.)

### analyze.py preprocessing order

1. **Timing audit** — verify the 1 kHz loop held (median dt, jitter, late count).
   All integration uses recorded timestamps, never a nominal dt.
2. **Bias + drift removal** — linear fit over pre-step ∪ settled tail
3. **Trapezoidal integration** on recorded timestamps
4. **Final value** — median over the last 150 ms, with a flatness check that
   flags `settled = NO` rather than silently reporting garbage
5. **Dead time** — model fit (primary) + 6σ threshold (independent cross-check)
6. **Smoothing** — zero-phase `filtfilt`, for peak/overshoot **only**, never
   for edge timing

Outputs per test: `lut.csv` + `mapping_points.csv` + `mapping.png` (A),
`step_summary.csv` + `step.png` + `step_residuals.png` (B),
`deadband_summary.csv` + `deadband.png` (K),
`repeat_summary.csv` + `repeat.png` (P).

**Health first, fit second.** Every run prints an I2C / timing / settling
audit before any model is fitted, and marks its numbers informational if the
audit fails. A fitted number from a bad trace is worse than no number.

---

## Procedure

### 0. Validate the analysis before spending bench time

```bash
python analyze.py --selftest
```

Fits synthetic traces with known truth. Currently recovers t_d = 13.96 ms
from 14.0 and τ = 50.13 ms from 50.0, ω_n = 40.0 from 40.0, ζ = 0.35 from
0.35 — and reproduces the −12 ms chord-method bias for comparison. A fitter
validated only against real data has no truth to be validated against.

### 1. Pre-checks

```bash
python capture.py COM7 c
```

Pass = all three gyro axes within ±1 dps, gyro sd < 0.5 dps, accel sd
< 0.01 g, |g| ≈ 1.00, 12-byte read > 1000 Hz. Then `z` (or `--shell`, then
`z`) to confirm the sweep axis is horizontal — jog the servo and watch the
tilt number actually move.

### 2. Find travel limits, with the linkage attached

```bash
python capture.py COM7 --shell
```

Then `x0` to pick the servo, `s5` for a 5 µs step, and `+` / `-` to creep up
on each stop. **The moment the servo strains or buzzes, press `u`** — it goes
limp instantly. Then `m` prints a `LIMIT,...` line with the 30 µs margin
already subtracted. Put those two numbers into `A_START` / `A_END` and
re-flash. Holding a servo against a hard stop cooks the gearbox and browns
out the BEC.

### 3. Run the tests

```bash
python capture.py COM7 a --axis 0 && python analyze.py runs/a_axis0_*
```

```bash
python capture.py COM7 b --axis 0 && python analyze.py runs/b_axis0_*
```

Each run lands in `runs/<test>_axis<n>_<timestamp>/` as `raw.csv` +
`meta.json` + `session.log`. Repeat with `--axis 1` for the other servo.
Test A ~5 min, B ~1 min, K ~2 min, P ~4 min, per axis.

**Step amplitudes** are computed by the sketch from `A_START`/`A_END` as
fractions of half-travel: 5 %, 10 %, 25 %, 50 %, 100 %, both directions —
so they follow automatically once step 2 sets the limits. Self-check: if
`slew_dps / amplitude` is constant across steps, nothing saturated and the
rate limiter can be dropped from the model. `analyze.py` reports that CV and
says which conclusion it supports.

**Validation.** Digital angle gauge at 5 points (neutral, ±max, ±half). An
independent method through a different code path catches scale and axis errors
that a second identical IMU would not. Phone slow-mo (240 fps) for a slew
sanity check.

---

## Deliverables

1. `lut.csv` — θ_TVC ↔ pulse_µs command map
2. Linear gain k (deg/µs) and **true neutral pulse** (will differ from 1520)
3. **Dead time** τ_d (ms)
4. **Lag** — τ, or {ω_n, ζ} if underdamped
5. Backlash (deg), repeatability (sd), cross-axis coupling (%)
6. Slew ceiling — only if reachable within travel

Items 3 and 4 stay separate. Colloquially both are "delay", but dead time
costs phase linearly with frequency and **nothing compensates it** — no
derivative action, no lead compensator. Lag is partially recoverable. 15 ms
dead time + 50 ms lag is far more forgiving than 65 ms of pure delay, even
though "time to converge" reads the same.

---

## Bugs already found and fixed — do not reintroduce

**ICM-20948 register layout ≠ MPU6050.** Bank 0 is `0x2D..0x32` accel,
`0x33..0x38` gyro, `0x39..0x3A` temp — temperature comes *after* the gyro,
where the MPU6050 puts it between. A burst read that skips two bytes after the
accel block yields `g[0]=gyroY, g[1]=gyroZ, g[2]=TEMP`. Symptom was a constant
~171–195 dps on gyro Z that drifted upward as the board warmed
(195 × 16.4 = 3200 raw → 3200/333.87 + 21 = 30.6 °C). Fix: read **12 bytes**,
no temp skip.

**Byte-order evaluation UB.** `(Wire.read() << 8) | Wire.read()` — operand
evaluation order is unspecified in C++. Use an explicit `be16()` helper that
reads hi then lo.

**Drift removal from the pre-step window alone.** Fitting offset + slope to a
90 ms window with σ ≈ 0.4 dps gives a slope standard error of
`SE = σ/(σ_t·√n) = 0.4/(0.026·9.5) ≈ 1.6 dps/s`, ~20× the drift being removed;
extrapolated over 1.1 s that is ~1.8° of fake ramp. Fix: fit over the pre-step
window **and the settled tail together** — both have true rate ≈ 0, the lever
arm becomes 1.1 s, SE drops to ~0.06 dps/s.

**Dead time by chord back-extrapolation.** For a first-order response,
extrapolating the 20–80 % chord back to baseline lands at −0.24 τ relative to
true onset — biased by the very time constant being measured (12 ms error on a
14 ms quantity when τ = 50 ms). Fix: direct least-squares fit of
`y = A(1 − e^{−(t−t_d)/τ})` over the whole trace; validated on synthetic data
(recovered t_d = 13.7 ms from 14.0, τ = 52 ms from 50). If overshoot > 5 %,
refit second order (recovered ω_n = 40.0, ζ = 0.35 from truth 40 / 0.35).

---

## Theory notes worth retaining

**Why a servo is low-pass.** The pulse is a *position setpoint* to a closed
loop already inside the case. Motor speed is first order
(`Ω/V = K_m/(τ_m s + 1)`, back-EMF `K_tK_e/R` dominating the damping);
position adds a free integrator; closing a proportional loop gives

```
T(s) = ω_n² / (s² + 2ζω_n s + ω_n²)
ω_n = √(K_p K_m / τ_m)      ζ = 1 / (2√(K_p K_m τ_m))
```

Low-pass is structural, not a design choice — relative degree 2 forces
−40 dB/decade. Physically: tracking `A sin ωt` needs peak torque `J A ω²` and
torque is bounded, so `A ≤ τ_max/(Jω²) ∝ 1/ω²`. The Bode roll-off *is* the
statement "finite torque, finite inertia". A second, separate ceiling comes
from back-EMF capping speed → `A ∝ 1/ω`, the slew-limited region at −20 dB/dec.

**Loop rate ≠ bandwidth.** Running the control loop at 333 Hz does not make
the servo respond at 333 Hz. At 577 °/s the servo moves at most 1.73° per 3 ms
frame; a 60° move takes 35 frames. What 333 Hz buys is **reduced latency**
(1.5 ms average vs 5 ms at 100 Hz), not speed. Achievable closed-loop bandwidth
is roughly servo bandwidth / 3–5.

**Bandwidth naming.** From radio: a "band" is a frequency range and its width
is the bandwidth. A servo is low-pass, so the band runs 0 → f_c and the width
equals the upper edge — hence one number. The −3 dB convention is the
half-power point (0.707² = 0.5).

---

## Planned flight control architecture

- **Control loop 333 Hz**, synchronized to the servo PWM frame (avoids
  mid-frame duty changes truncating a pulse)
- IMU polled at 1 kHz, 3-sample average into the 333 Hz loop (√3 noise
  reduction, free)
- Core 1: control task only, high priority. Core 0: telemetry, logging, WiFi
- `WiFi.mode(WIFI_OFF); btStop();` — the WiFi stack causes ms-scale jitter
- **`float`, never `double`** — the ESP32 FPU is single-precision only, so
  `double` is software-emulated. `sqrtf`, `atan2f`, `0.5f` literals
- D-term from the **gyro directly**, never a numerical derivative of angle
- Integrator freeze when the command saturates (anti-windup)
- No `Serial.print` inside the control loop — ring buffer, drained on core 0

Delay budget estimate: IMU filter 0.2 ms + I2C 0.3 ms + loop 3 ms + PWM frame
3 ms + servo mechanical ~10 ms ≈ **17 ms**, dominated by the servo. MCU-side
optimization cannot help. Test B confirms whether this budget is real.

Flight logging will need SD (SPI) or LittleFS — serial is bench-only.
LittleFS is simpler and sufficient for short flights (~1 MB for
200 Hz × 20 ch × 60 s).

---

## Next actions

1. `python analyze.py --selftest` — confirm the fitters still pass
2. Flash `char.ino`, run `c`, confirm all gyro axes ≈ 0 and |g| ≈ 1
3. `--shell` → `z` — confirm the sweep axis is horizontal and the tilt moves
4. `--shell` → jog both axes to their limits, `m` to record, set
   `A_START` / `A_END`, re-flash
5. Test A → B → K → P on axis 0, then repeat on axis 1
6. Cross-check the mapping with an angle gauge at 5 points
7. Feed τ_d, τ (or ω_n, ζ), backlash into the simulator's actuator model;
   overlay simulated vs measured step response to validate
8. Then: attitude controller design, with the measured phase lag setting the
   gain ceiling
