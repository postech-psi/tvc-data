#!/usr/bin/env python3
"""
Serial capture for the TVC gimbal characterization sketch (char.ino).

    pip install pyserial

    python capture.py --list                  # find your port
    python capture.py COM7 a                  # run TEST A (mapping)
    python capture.py COM7 b --axis 1         # TEST B on servo B
    python capture.py COM7 c                  # IMU health check
    python capture.py COM7 --shell            # interactive jog

Each run writes a self-contained directory:

    runs/<test>_axis<n>_<YYYY-MM-DD_HHMMSS>/
        raw.csv       every S/E row, verbatim, nothing dropped
        meta.json     the #META manifest, parsed into key/value
        session.log   the complete serial stream including comments

raw.csv is the ONLY thing the analysis reads for data, and meta.json is
the only thing it reads for constants -- no constant is duplicated in
the analysis source, so the sketch and the analysis cannot drift apart.
"""
import argparse
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

BAUD = 460800

# command -> (end marker, rough expected duration for the progress line)
TESTS = {
    "a": ("# TEST_A_END", "mapping sweep, ~5 min"),
    "b": ("# TEST_B_END", "step response, ~1 min"),
    "k": ("# TEST_K_END", "deadband, ~2 min"),
    "p": ("# TEST_P_END", "repeatability, ~4 min"),
    "c": ("# HEALTH_END", "IMU health, ~5 s"),
    "z": ("# LIVE_END",   "live readout, 20 s"),
}

COLS = None  # filled from the #COLS line


def parse_meta_line(line, meta):
    """#META k=v k=v ... -> dict.  Repeated keys become lists, because
    TEST B emits one '#META step ...' block per step."""
    body = line[len("#META"):].strip()
    # bare words (like 'step' or 'deadband') are recorded as flags
    pairs = re.findall(r"(\w+)=([^\s]+)", body)
    flags = [w for w in body.split() if "=" not in w]
    for f in flags:
        meta.setdefault("_flags", [])
        if f not in meta["_flags"]:
            meta["_flags"].append(f)
    for k, v in pairs:
        try:
            v = int(v, 0) if re.fullmatch(r"[-+]?(0x[0-9a-fA-F]+|\d+)", v) else float(v)
        except ValueError:
            pass
        if k in meta:
            if not isinstance(meta[k], list):
                meta[k] = [meta[k]]
            meta[k].append(v)
        else:
            meta[k] = v


def open_port(port):
    ser = serial.Serial(port, BAUD, timeout=1.0)
    time.sleep(2.0)          # ESP32 resets on DTR; wait out the boot
    ser.reset_input_buffer()
    return ser


def run_test(ser, cmd, axis, outdir, timeout):
    end_marker, blurb = TESTS[cmd]

    if axis is not None:
        ser.write(f"x{axis}\n".encode())
        time.sleep(0.2)
    ser.write((cmd + "\n").encode())

    outdir.mkdir(parents=True, exist_ok=True)
    raw = (outdir / "raw.csv").open("w", newline="")
    log = (outdir / "session.log").open("w")

    meta = {"captured_utc": datetime.utcnow().isoformat() + "Z",
            "command": cmd, "axis": axis, "baud": BAUD}
    cols = None
    n_s = n_e = 0
    last = time.time()
    t0 = time.time()

    print(f"# running '{cmd}' ({blurb}) -> {outdir}")
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
            if not line:
                if time.time() - last > timeout:
                    print(f"\n!! {timeout:.0f} s of silence, giving up")
                    meta["incomplete"] = True
                    break
                continue
            last = time.time()
            log.write(line + "\n")

            if line.startswith("#META"):
                parse_meta_line(line, meta)
            elif line.startswith("#COLS"):
                cols = line[len("#COLS"):].strip()
                raw.write(cols + "\n")
                meta["columns"] = cols.split(",")
            elif line.startswith(("S,", "E,")):
                if cols is None:
                    print("!! data before #COLS -- sketch/capture mismatch")
                raw.write(line + "\n")
                if line[0] == "S":
                    n_s += 1
                else:
                    n_e += 1
                if n_s % 500 == 0:
                    el = time.time() - t0
                    print(f"\r  {n_s:7d} samples  {n_e:4d} events  {el:6.1f}s",
                          end="", flush=True)
            elif line.startswith("!!"):
                print(f"\n{line}")
            else:
                print(f"\n{line}")

            if line.startswith(end_marker):
                break
    except KeyboardInterrupt:
        print("\n!! interrupted -- sending neutral")
        ser.write(b"n\n")
        meta["incomplete"] = True

    meta["n_samples"] = n_s
    meta["n_events"] = n_e
    meta["duration_s"] = round(time.time() - t0, 2)
    raw.close()
    log.close()
    (outdir / "meta.json").write_text(json.dumps(meta, indent=2))

    print(f"\n# {n_s} samples, {n_e} events, {meta['duration_s']} s")

    ok = True
    if n_s == 0:
        print("!! no samples captured"); ok = False
    if cols is None:
        print("!! no #COLS header -- raw.csv has no column names"); ok = False
    if meta.get("incomplete"):
        print("!! run did not reach its end marker; treat as suspect"); ok = False

    err0 = meta.get("i2c_err_at_start")
    err1 = meta.get("i2c_err_at_end")
    if isinstance(err0, (int, float)) and isinstance(err1, (int, float)):
        d = err1 - err0
        print(f"# I2C errors during run: {d}")
        if d > 0:
            print("!! I2C errors occurred -- check the flags column")
    return ok


def shell(ser):
    """Interactive jog. Everything typed goes to the sketch verbatim."""
    import threading

    print("# interactive. type sketch commands ('h' for help), Ctrl-C to exit.")
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            line = ser.readline().decode("utf-8", errors="replace").rstrip()
            if line:
                print(line)

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    try:
        for line in sys.stdin:
            ser.write(line.rstrip("\n").encode() + b"\n")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.write(b"n\n")     # leave the rig at neutral
        print("\n# neutral, exiting")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?")
    ap.add_argument("cmd", nargs="?", choices=sorted(TESTS))
    ap.add_argument("--axis", type=int, choices=(0, 1),
                    help="select servo A(0) or B(1) before running")
    ap.add_argument("--outdir", default="runs", help="root output directory")
    ap.add_argument("--tag", default="", help="suffix for the run directory")
    ap.add_argument("--timeout", type=float, default=20.0,
                    help="seconds of silence before giving up")
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--shell", action="store_true", help="interactive jog mode")
    a = ap.parse_args()

    if a.list or not a.port:
        for p in list_ports.comports():
            print(f"  {p.device:20s} {p.description}")
        return 0

    ser = open_port(a.port)

    if a.shell:
        shell(ser)
        return 0

    if not a.cmd:
        ap.error("need a test command, or --shell")

    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    name = f"{a.cmd}_axis{a.axis if a.axis is not None else 'X'}_{stamp}"
    if a.tag:
        name += "_" + a.tag
    outdir = Path(a.outdir) / name

    ok = run_test(ser, a.cmd, a.axis, outdir, a.timeout)
    ser.write(b"n\n")
    ser.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
