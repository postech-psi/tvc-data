# Acquisition: `tvcbench`

The Pi-side rewrite. `tvcbench` records; [`tvctools`](../tvctools) analyses.

## Why it exists

The old arrangement had three clocks and no link between them:

| Stream | Owner | Clock | Rate |
|---|---|---|---|
| `thrust_map_*.csv` | Pi | `t_epoch`, UTC wall clock | ~18–20 Hz |
| `data_*.csv` (Fx…Tz) | **bench laptop** | `t_ms`, STM32 uptime | 50 Hz |
| `*.ulg` | Pixhawk SD | PX4 boot µs | 5–10 Hz |

Because force lived on a different machine from the commands, the entire timing
chain was reconstructed afterwards — filename anchoring good to ~1 s, then
cross-correlation, then a largest-edge cross-check, landing at about ±0.1 s and
giving up entirely (`lag_method: "weak"`) on runs with too little command
variance. See [PREPROCESSING.md](PREPROCESSING.md) for that machinery.

Moving the load cell onto the Pi removes the problem rather than improving it.
One process stamps force and command on one clock, so there is nothing to align.

Three further defects are fixed along the way:

- **No fabricated rates.** The old logger emitted a row only when
  `SERVO_OUTPUT_RAW` arrived and carried the last voltage forward into it,
  inventing a 20 Hz record of a 10 Hz quantity. Each stream is now its own file
  at its own rate, joined only in analysis.
- **No string keys.** `phase = "A1400_B1700"` could not distinguish two visits to
  the same command. `seg_id` is an integer; `sequence.csv` holds the commands.
- **The run is reproducible.** Settings came from an HTML form and were written
  nowhere — the configured `dwell_s` was 2.0 while the measured step was 4.03 s,
  with no record to explain it. A run is now a checked-in plan file, copied
  verbatim into the manifest.

## Commands

```bash
python -m tvcbench devices --probe                  # find the load cell, get a udev rule
python -m tvcbench selftest --seconds 20            # the hardware gate; run before every session
python -m tvcbench plan show plans/coax_grid.yaml   # expand, time and cost out a plan
python -m tvcbench run plans/coax_grid.yaml --sim   # full dress rehearsal, no hardware
python -m tvcbench run plans/coax_grid.yaml --no-motor
python -m tvcbench run plans/coax_grid.yaml
```

## Before the first real run

Hosting the STM32 on the Pi introduces three risks, all of which corrupt data
rather than stopping it. `selftest` reports each one.

1. **USB power.** The board powers itself *and* the bridge excitation over one
   cable. A Pi 5 supplies ~1.6 A across its USB-A ports with the official 27 W
   supply, but ~600 mA with a weaker one. A brownout resets the MCU and nothing
   else says so — `selftest` watches the sample counters for a backwards jump.
   Prefer a powered hub.
2. **Ground loop.** The Pi now shares ground with the STM32 and the Pixhawk
   around a frame carrying motor current. Record a baseline on the laptop, then
   compare:
   ```bash
   python -m tvcbench selftest --save-baseline laptop.json   # on the laptop
   python -m tvcbench selftest --baseline laptop.json        # on the Pi
   ```
   A materially raised noise floor means a USB isolator is needed.
3. **Device naming.** `ttyACM0` and `ttyACM1` can swap between boots.
   `devices --probe` prints the udev rule that pins the board to
   `/dev/tvc-loadcell`.

Connect the board to a **USB-A port** with an A-to-C cable — the Pi 5's USB-C is
the power input, and the Pi must be the host.

## Output

```
bench/<run_id>/
  manifest.json   plan (verbatim + resolved), seed, clock anchors and fits,
                  tare offsets, achieved vs requested rates, outcome, warnings
  sequence.csv    one row per segment: planned command, actual start/end,
                  measured thrust mean and SEM
  events.jsonl    every transition, tare, warning and abort
  loadcell.csv    t_mono, dev_t, seg_id, Fx..Tz raw *and* tared
  fc_servo.csv    t_mono, dev_t, seg_id, servo1..8
  fc_battery.csv  t_mono, seg_id, voltage, current
  fc_esc.csv      t_mono, dev_t, seg_id, rpm/voltage/current (empty without ESC telemetry)
```

`bench/` is separate from `runs/`, which holds the previous pipeline's derived
output. The formats are unrelated and mixing them would only confuse.

### Clocks

Data files carry **raw** clock values and no derived epoch column, so they stay
append-only: a run killed mid-way is still valid up to the point it stopped. The
interpretation lives in `manifest.json`.

Every sample has `t_mono` from `time.monotonic()`. Epoch is derived from anchor
pairs captured at the start and end, so an NTP step mid-run moves the anchors
rather than part of the data.

`dev_t` is the device's own counter — `t_ms` on the STM32, `time_usec` on the
Pixhawk. Arrival timestamps alone are not good enough: USB CDC hands over 8–10
samples in one read, so a burst all share an arrival time. Since transport
latency is strictly one-sided (a sample can arrive late, never early), the true
relation lies along the **lower edge** of the point cloud, and
`clock.fit_device_clock` fits that edge rather than running least squares through
the middle of it.

Measured on a simulated 50 Hz link with 9-sample batches and 2 % stalls: raw
arrival stamps are late by a median 82 ms with a 161 ms spread; after fitting,
the spread is 0.03 ms. A constant transport delay (~1.5 ms) survives as a fixed
offset, since no fit can tell a steady delay from a clock offset — harmless, as
it shifts every force sample equally.

### Tare

Non-destructive. `gui.py` subtracted its zero before writing and kept no record,
so a mis-tared run was unrecoverable. Here `Fz_raw` and `Fz` are both written and
the offsets go in the manifest.

## Plans

See [`plans/coax_grid.yaml`](../plans/coax_grid.yaml). Two choices are worth
understanding.

**Ordering.** The pack drains monotonically, so a grid walked in order makes
every high-B point a low-voltage point and the two can never be separated.
`blocked_random` gives each repeat its own shuffled complete pass over the grid —
a randomised complete block design — so every command level sees the same
expected position in the drain. Adjacent jumps are left unconstrained: settling
is what dwell is for.

**Dwell.** Three modes:

| Mode | Behaviour |
|---|---|
| `fixed` | Hold for a set time. Start here — the run's timing cannot depend on the sensor. |
| `settle` | Hold until thrust stops moving, then hold a fixed span of settled signal. |
| `sem_target` | Hold until the thrust standard error reaches a target. |

`sem_target` is the interesting one: it makes the error bar **uniform across the
map** instead of letting it vary with local noise. The uncertainty is corrected
for autocorrelation using the same estimator the analysis uses
(`tvctools.analyze.integrated_autocorr_time`) — propeller vibration has lag-1
autocorrelation near 0.5, so a naive `sd/√n` would stop every step about four
times too early.

Move to an adaptive mode only once fixed-dwell runs are trusted; `max_s` is then
the only bound on a step that never settles.

**Reference revisits** return to a fixed point every N steps, so drift can be
fitted as a covariate rather than merely bracketed at the ends.

## Safety

The base layer is not in any of this code: `MAV_CMD_ACTUATOR_TEST` expires after
`timeout_s`, so **ceasing to send is itself the stop** — a crashed process, a
severed cable and a killed thread all stop the motors by doing nothing. The
runner is the only thing that ticks the actuator.

Above that, `supervisor.py` is the sole abort authority — not the user interface.
The old watchdog fired when the browser stopped polling, which is meaningless
headless. Limits: minimum voltage, maximum current, **maximum thrust and
torque** (new, and only possible now the Pi can see force — catches a shed blade
or a slipped mount), load-cell dropout, FC heartbeat loss, plus SIGINT/SIGTERM
and a `--stop-file`.

Every abort path still stops the motors, closes the files and writes a manifest.
A run that ends badly is still a readable run.

`--no-motor` executes the whole sequence, timing and recording path with every
command suppressed — a full rehearsal with the battery disconnected.

## Retiring the `.ulg`

Everything the ulog supplies is now recorded by the Pi, including the no-load
voltage that the `idle_pre`/`idle_post` segments measure directly. Two honest
caveats: `SERVO_OUTPUT_RAW` achieves ~18–20 Hz against a requested 50 (it is a
verification echo of a command already known, so this is ample), and
`BATTERY_STATUS` at 10–20 Hz is the real resolution limit for power integration.

Keep the ulog for a handful of sessions and check it against the Pi over the
**sync chirps** — a short square wave at each end of every run, retained purely
as a high-SNR feature for that comparison. Drop the ulog once they agree.

## Testing

```bash
python -m pytest tests/ -q
```

The simulated sources exercise the runner, recorder, supervisor and dwell logic
with nothing plugged in, including every abort path. `sources/sim.py` output is
**not data** — but its power law (`P = 5.1·T^1.5`) and its autocorrelated noise
are taken from the real measurements, so the sag and dwell paths are rehearsed
against something close to this bench.
