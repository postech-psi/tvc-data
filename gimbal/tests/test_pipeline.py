"""Hardware-free contract and end-to-end tests."""
from __future__ import annotations

import binascii
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from analyze import (analyze_health, analyze_mapping, analyze_step,
                     first_order)
from sid_capture import (BINARY_FORMAT, BINARY_MAGIC, BINARY_RECORD,
                         CSV_COLUMNS, Device, RECORD_SAMPLE)

COLUMNS = ["rec", "t_us", "phase", "seq", "axis", "gimbal",
           "cmd_outer", "cmd_inner", "ax", "ay", "az", "gx", "gy", "gz",
           "flags"]


def write_run(path: Path, test: str, rows: list[list], axis: int = -1) -> None:
    path.mkdir(parents=True)
    pd.DataFrame(rows, columns=COLUMNS).to_csv(path / "raw.csv", index=False)
    meta = {
        "test": test, "axis": axis,
        "gimbal": {0: "outer", 1: "inner"}.get(axis, "none"),
        "acc_lsb_per_g": 16384.0,
        "gyro_lsb_per_dps": 16.4, "health_pass": 1,
        "end_marker_seen": True, "malformed_rows": 0,
    }
    (path / "meta.json").write_text(json.dumps(meta), encoding="utf-8")


class FakeSerial:
    def __init__(self, payload: bytes):
        self.payload = bytearray(payload)
        self.writes = []
        self.is_open = True

    def reset_input_buffer(self): pass
    def flush(self): pass
    def close(self): self.is_open = False
    def write(self, data): self.writes.append(data)
    def read(self, size):
        if not self.payload:
            return b""
        count = min(size, 17, len(self.payload))
        result = bytes(self.payload[:count])
        del self.payload[:count]
        return result
    def readline(self):
        if not self.payload:
            return b""
        newline = self.payload.find(b"\n")
        count = len(self.payload) if newline < 0 else newline + 1
        result = bytes(self.payload[:count])
        del self.payload[:count]
        return result


def binary_sample(packet_seq: int = 0, t_us: int = 0) -> bytes:
    values = [
        BINARY_MAGIC, RECORD_SAMPLE, packet_seq, t_us, 0, 0, -1, 1520, 1520,
        0, 0, 16384, 0, 0, 0, 0, 3, 0,
    ]
    packet = BINARY_RECORD.pack(*values)
    values[-1] = binascii.crc_hqx(packet[2:-2], 0xFFFF)
    return BINARY_RECORD.pack(*values)


def test_capture(root: Path) -> None:
    device = Device.__new__(Device)
    device.port = "FAKE"
    prefix = (
        "#META fw=tvc_sid_2.1_gimbal_bin1k test=HEALTH axis=-1 "
        "gimbal=none gimbal_count=2 health_pass=1 "
        "stream_hz=1000\n"
        "#META expected_samples=1000 expected_events=0\n"
        "#COLS " + ",".join(CSV_COLUMNS) + "\n"
        f"#BINARY_BEGIN format={BINARY_FORMAT} "
        f"record_bytes={BINARY_RECORD.size} stream_hz=1000\n"
    ).encode()
    suffix = (
        "#BINARY_END packets=1000 serial_short_writes=0\n"
        "#META health_pass=1\n# HEALTH_END\n"
    ).encode()
    samples = b"".join(binary_sample(i, i * 1000) for i in range(1000))
    device.ser = FakeSerial(prefix + samples + suffix)
    meta = device.capture("HEALTH", root / "capture", silence_timeout=.01)
    assert meta["capture_ok"] and meta["n_samples"] == 1000
    assert meta["effective_sample_rate_hz"] == 1000.0
    assert meta["packet_gaps"] == 0 and meta["binary_crc_errors"] == 0
    assert device.ser.writes == [b"HEALTH\n"]


def build_synthetic_calibration(root: Path) -> Path:
    """Single-pose uniform-scale calibration, mirroring calibrate.py: fit one
    scale so corrected |g| = 1.0 at the (Z-up) mounting orientation."""
    rng = np.random.default_rng(2)
    accel = np.array([0.0, 0.0, 1.0]) + rng.normal(0, 0.001, (400, 3))
    gyro = np.array([0.7, -0.4, 0.2]) + rng.normal(0, 0.03, (400, 3))
    scale = 1.0 / float(np.linalg.norm(accel, axis=1).mean())
    cal = {
        "format": "tvc_sid_calibration_v1",
        "pass": True,
        "acc_matrix": [[scale, 0, 0], [0, scale, 0], [0, 0, scale]],
        "acc_offset": [0.0, 0.0, 0.0],
        "gyro_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "gyro_bias_dps": gyro.mean(axis=0).tolist(),
        "gyro_noise_dps": gyro.std(axis=0).tolist(),
    }
    path = root / "calibration.json"
    path.write_text(json.dumps(cal, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def test_health(root: Path, calibration: Path) -> None:
    rng = np.random.default_rng(3)
    rows = []
    for i in range(800):
        accel = np.array([0, 0, 1.0]) + rng.normal(0, .001, 3)
        gyro = np.array([.7, -.4, .2]) + rng.normal(0, .03, 3)
        rows.append(["S", i * 5000, "health", i, -1, "none", 1520, 1520,
                     *(accel * 16384).round().astype(int),
                     *(gyro * 16.4).round().astype(int), 0])
    path = root / "health"
    write_run(path, "HEALTH", rows)
    result = analyze_health(path, calibration)
    assert result["pass"], result


def test_mapping(root: Path, calibration: Path) -> None:
    rows = []
    timestamp = 0
    for i in range(80):
        rows.append(["S", timestamp, "zero", -1, 0, "outer", 1520, 1520,
                     0, 0, 16384, 0, 0, 0, 0])
        timestamp += 10000
    orders = {
        "up": range(1420, 1630, 10), "dn": range(1620, 1410, -10),
        "up2": range(1420, 1630, 10), "dn2": range(1620, 1410, -10),
    }
    for phase, pulses in orders.items():
        for pulse in pulses:
            angle = np.radians(0.05 * (pulse - 1520) +
                               (0.08 if phase.startswith("dn") else -0.08))
            accel = np.array([np.sin(angle), 0, np.cos(angle)])
            for _ in range(30):
                rows.append(["S", timestamp, phase, pulse, 0, "outer", pulse, 1520,
                             *(accel * 16384).round().astype(int), 0, 0, 0, 0])
                timestamp += 10000
    path = root / "mapping"
    write_run(path, "A", rows, axis=0)
    result = analyze_mapping(path, calibration)
    assert result["pass"], result
    assert result["gimbal"] == "outer"
    assert abs(result["gain_deg_per_us"] - 0.05) < 0.002


def test_step(root: Path, calibration: Path) -> None:
    rng = np.random.default_rng(4)
    rows = []
    start = 0.0
    amplitudes_us = [8, -8, 16, -16, 35, -35, 70, -70, 130, -130]
    fs = 1000
    local = np.arange(0, 2.2, 1 / fs)
    for seq, amp_us in enumerate(amplitudes_us):
        command = start + 0.2
        rows.append(["E", int(start * 1e6), "arm", seq, 0, "outer", 1520, 1520,
                     "", "", "", "", "", "", 0])
        rows.append(["E", int(command * 1e6), "cmd", seq, 0, "outer",
                     1520 + amp_us, 1520,
                     "", "", "", "", "", "", 0])
        final = 0.05 * amp_us
        theta = first_order(local - .2, final, .014, .050)
        rate = np.gradient(theta, local) + np.array([.7, -.4, .2])[0]
        rate += rng.normal(0, .03, len(rate))
        for i, lt in enumerate(local):
            pulse = 1520 if lt < .2 else 1520 + amp_us
            gyro = np.array([rate[i], -.4, .2])
            rows.append(["S", int((start + lt) * 1e6),
                         "pre" if lt < .2 else "post", seq, 0, "outer",
                         pulse, 1520,
                         0, 0, 16384, *(gyro * 16.4).round().astype(int), 0])
        start += 2.5
    path = root / "step"
    write_run(path, "B", rows, axis=0)
    result = analyze_step(path, calibration)
    assert result["pass"], result
    assert abs(result["direct_onset_ms"] - 14) < 10
    assert abs(result["bandwidth_hz"] - 1 / (2 * np.pi * .05)) < 2


def main() -> int:
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        test_capture(root)
        calibration = build_synthetic_calibration(root)
        test_health(root, calibration)
        test_mapping(root, calibration)
        test_step(root, calibration)
    print("pipeline tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
