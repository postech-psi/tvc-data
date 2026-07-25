# raw/

Raw acquisition files, filed by measurement date and by which system wrote them.
**Nothing in here is ever edited.** Everything else in the repo is rebuildable
from this directory.

```
raw/<YYYY-MM-DD>/pwm/       thrust_map_<epoch>.csv
                 loadcell/  data_<YYYYMMDD>_<HHMMSS>.csv
                 ulog/      log_<n>_<date>.ulg
                 ulog/_redundant/
```

The date folder is the *measurement* date, read from inside the file where
possible — not the file's mtime, which a git checkout rewrites.

## pwm/ — Raspberry Pi, `pwm_thrust_map.py`, ~20-50 Hz

What was commanded, and what the battery did. 22 columns:

| Column | Meaning |
|---|---|
| `t_epoch` | Pi wall clock, **UTC**. The merge key against the load cell. |
| `t_fc_us` | Pixhawk boot clock. Dates the `.ulg` files. |
| `phase` | `A<us>_B<us>`, or `idle_pre` / `idle_post` for the no-load windows |
| `a_cmd_us`, `b_cmd_us` | commanded µs, rotor A and rotor B |
| `a_cmd_norm`, `b_cmd_norm` | the same as normalized 0–1 |
| `servo1_raw` … `servo8_raw` | **measured** output µs per channel (1 = A, 2 = B) |
| `voltage_v`, `current_a` | pack voltage and current, QGC-identical calculation |
| `sweep_idx` | 1-based repeat number |
| `esc1_rpm` … `esc4_rpm` | ESC telemetry RPM; blank when unsupported |

`idle_pre` / `idle_post` rows have **both rotors at 1000 µs**, so their voltage
is the battery's state of charge, free of the IR sag that loaded readings carry.
Files recorded before those phases existed take the state of charge from the
`.ulg` instead.

## loadcell/ — bench laptop, `gui.py`, 50 Hz

Force and torque. 11 columns, of which **only these four matter**:

| Column | Meaning |
|---|---|
| `t_ms` | free-running STM32 uptime, **not** epoch — hence the alignment step |
| `Fz` | vertical force, newtons. **Thrust = −Fz** (logged negative). |
| `Tz` | reaction torque, N·m. Changes sign as the coaxial pair balances. |
| `Fx`, `Fy`, `Tx`, `Ty` | off-axis components |

`pwm`, `rpm`, `Current_mA`, `ADC_Current_mA` are **dead by design** — the Pi
drives the motor through the Pixhawk, so the stand never sees the throttle,
there is no RPM sensor, and current comes from the Pixhawk. Ignore them; take
PWM and voltage from the matching `pwm/` file.

## ulog/ — Pixhawk SD card

PX4 logs, covering whole sessions including the idle gaps between runs. Lower
rate than the Pi (10 Hz outputs, 5 Hz battery) and carrying no usable absolute
time, so they are a cross-check and a source of no-load voltage for older runs,
not the primary record. Gitignored — they are large and stay local.

`_redundant/` holds logs with no unique content: truncated second downloads of
the same boot, and logs recorded with the motor never running. Kept rather than
deleted, but skipped by the pipeline. Safe to delete if you need the space.
