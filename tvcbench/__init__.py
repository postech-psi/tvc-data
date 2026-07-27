"""
tvcbench -- data acquisition for the coaxial thrust bench.

Companion to `tvctools`, which analyses what this package records. The split is
deliberate: `tvcbench` runs on the Raspberry Pi and owns the hardware, the
command sequence and the clock; `tvctools` runs anywhere and owns the maths.

The design premise is that the Pi is the *sole* acquisition node. It commands
the rotors through the Pixhawk and reads the load cell over USB CDC, so force
and command share one clock and no post-hoc time alignment is required. That is
the whole reason this package exists.
"""

SCHEMA_VERSION = 1
