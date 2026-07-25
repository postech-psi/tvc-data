"""
tvctools -- organize scattered TVC test-stand data.

The bench produces three unrelated data streams that never shared a clock:

    pwm_map   thrust_map_<epoch>.csv    PX4 commands + voltage/current   20 Hz, UTC epoch
    loadcell  data_<date>_<time>.csv    Fx..Fz / Tx..Tz from the stand   50 Hz, MCU uptime
    ulog      log_*.ulg                 PX4 SD-card log                  FC boot clock

This package catalogs those files, groups them into runs and sessions, and
time-aligns force/torque against the PWM/voltage that produced it.

Entry points:
    python -m tvctools index          # catalog everything -> runs_index.csv
    python -m tvctools build          # group + copy + merge -> runs/
    python -m tvctools ulog <file>    # standalone .ulg analysis
"""

__version__ = "0.1.0"

LOCAL_TZ_OFFSET_H = 9.0  # bench is in KST (UTC+9); load-cell and ulog filenames are local
