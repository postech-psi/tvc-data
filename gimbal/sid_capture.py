"""Loss-detecting serial capture for the TVC system-ID firmware."""
from __future__ import annotations

import binascii
import csv
import json
import re
import shutil
import statistics
import struct
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
from serial.tools import list_ports

# =============================================================================
# USER SERIAL-LOGGING PARAMETERS
# Keep BAUD equal to SERIAL_BAUD at the top of src/main.cpp.
# =============================================================================
BAUD = 921600
READY_TIMEOUT_S = 12.0
SERIAL_READ_TIMEOUT_S = 0.05
SERIAL_WRITE_TIMEOUT_S = 2.0
COMMAND_RESPONSE_TIMEOUT_S = 3.0
CAPTURE_SILENCE_TIMEOUT_S = 30.0
PROGRESS_FLUSH_EVERY_SAMPLES = 1000
EXPECTED_STREAM_RATE_HZ = 1000
# OS serial RX buffer. The default (~4 KB on Windows ≈ 0.1 s of stream) overflows
# and drops samples (packet_gaps) if PC-side processing stalls briefly during the
# long multi-minute captures. 1 MB ≈ 28 s of headroom absorbs any realistic stall.
SERIAL_RX_BUFFER_BYTES = 1 << 20
# Tolerated stray-byte budget before a capture fails on framing loss of sync.
# Surviving records are CRC-validated regardless; this only bounds resync noise.
# Scaled by stream size (like GLITCH_MAX_FRACTION below) rather than a fixed
# count: a fixed 72 B budget is fine for a short HEALTH capture but is only
# ~2 record-lengths of headroom on a multi-minute, multi-MB mapping stream,
# where a couple of harmless USB-bridge glitches can exceed it even though the
# actual data loss (see usability verdict) is negligible.
FRAMING_BYTES_DISCARD_MIN = 72  # floor for short captures (< 2 record lengths)
FRAMING_BYTES_DISCARD_MAX_FRACTION = 0.0001  # 0.01% of total stream bytes
# First bytes that mark a text line rather than a binary record. "#!SE" are the
# firmware's own prefixes; "[" catches ESP32 core log output ("[ 12345][E][Wire.cpp
# :499] ..."), which shares this UART with the binary stream. Without "[", each log
# line silently burned its 9-byte "[ timestamp][" prefix as framing discards and
# then matched on the "E" of "[E]", so a healthy capture could fail on
# FRAMING_BYTES_DISCARD_MAX purely from IMU log chatter. Build with
# -D CORE_DEBUG_LEVEL=0 so these are not emitted in the first place; this is the
# belt-and-braces half.
LINE_START_BYTES = b"#!SE["
# Tolerated single-event record loss/corruption over a long stream. Gap budget is
# max(GLITCH_MIN_COUNT, GLITCH_MAX_FRACTION * n_samples); CRC errors up to
# GLITCH_MIN_COUNT. Systematic loss produces far more and still fails.
GLITCH_MAX_FRACTION = 0.0005   # 0.05% of samples (~149 of 297k)
GLITCH_MIN_COUNT = 5

# Fixed 36-byte little-endian record emitted by src/main.cpp.
BINARY_MAGIC = b"\xA5\x5A"
BINARY_FORMAT = "tvc_sid_gimbal_bin_v2"
BINARY_RECORD = struct.Struct("<2sBIIBhbHHhhhhhhBhH")
RECORD_SAMPLE = 0x01
RECORD_EVENT = 0x02
GIMBAL_NAMES = {0: "outer", 1: "inner", -1: "none"}
# Index-for-index mirror of the firmware PhaseCode enum (src/main.cpp); the
# binary record carries the phase as its index into this list.
PHASES = [
    "health", "zero",
    "up", "dn", "up2", "dn2", "pre", "post", "arm", "cmd", "chirp", "grid",
]
CSV_COLUMNS = [
    "rec", "t_us", "phase", "seq", "axis", "gimbal",
    "cmd_outer", "cmd_inner", "ax", "ay", "az", "gx", "gy", "gz",
    "flags", "packet_seq", "jitter_us",
]

# Internal protocol markers -- change only if the firmware protocol changes.
END_MARKERS = {
    "HEALTH": "# HEALTH_END",
    "A": "# TEST_A_END",
    "B": "# TEST_B_END",
    "C": "# TEST_C_END",
    "D": "# TEST_D_END",
    "E": "# TEST_E_END",
    "GRID": "# TEST_E_END",
}


def available_ports() -> list[tuple[str, str]]:
    return [(p.device, p.description) for p in list_ports.comports()]


def last_value(value):
    return value[-1] if isinstance(value, list) and value else value


def as_bool(value):
    value = last_value(value)
    if isinstance(value, str):
        return value.lower() not in {"0", "false", "fail", "no"}
    return bool(value)


def parse_meta(line: str, meta: dict) -> None:
    body = line[len("#META"):].strip()
    for key, value in re.findall(r"(\w+)=([^\s]+)", body):
        try:
            if re.fullmatch(r"[-+]?(?:0x[0-9a-fA-F]+|\d+)", value):
                value = int(value, 0)
            else:
                value = float(value)
        except ValueError:
            pass
        if key in meta:
            if not isinstance(meta[key], list):
                meta[key] = [meta[key]]
            meta[key].append(value)
        else:
            meta[key] = value


def decode_binary_record(packet: bytes) -> dict:
    """Validate and decode one tvc_sid_gimbal_bin_v2 fixed-size record."""
    if len(packet) != BINARY_RECORD.size or packet[:2] != BINARY_MAGIC:
        raise ValueError("invalid binary record framing")
    values = BINARY_RECORD.unpack(packet)
    expected_crc = binascii.crc_hqx(packet[2:-2], 0xFFFF)
    if values[-1] != expected_crc:
        raise ValueError("binary record CRC mismatch")
    (magic, record_type, packet_seq, t_us, phase_code, seq, axis,
     cmd_outer, cmd_inner, ax, ay, az, gx, gy, gz, flags,
     jitter_us, crc) = values
    if record_type not in {RECORD_SAMPLE, RECORD_EVENT}:
        raise ValueError(f"unknown binary record type {record_type}")
    phase = PHASES[phase_code] if phase_code < len(PHASES) else f"unknown_{phase_code}"
    return {
        "rec": "S" if record_type == RECORD_SAMPLE else "E",
        "t_us": t_us, "phase": phase, "seq": seq, "axis": axis,
        "gimbal": GIMBAL_NAMES.get(axis, f"unknown_{axis}"),
        "cmd_outer": cmd_outer, "cmd_inner": cmd_inner,
        "ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz,
        "flags": flags, "packet_seq": packet_seq, "jitter_us": jitter_us,
    }


def build_usability(meta: dict, gap_budget: int, framing_budget: int,
                    total_bytes: int) -> dict:
    """Judge whether a capture is usable for downstream analysis, independent
    of the hard pass/fail gate below. The gate exists to catch systematic
    protocol faults; this looks at the actual measured impact (how much data
    is missing / corrupted, out of how much) so a human doesn't have to
    re-derive it by hand from meta.json + raw.csv after the fact."""
    expected_samples = last_value(meta.get("expected_samples"))
    expected_events = last_value(meta.get("expected_events"))
    n_samples = meta["n_samples"]
    n_events = meta["n_events"]
    crc = meta["binary_crc_errors"]
    framing = meta["framing_bytes_discarded"]

    sample_loss_pct = None
    shortfall = 0
    if isinstance(expected_samples, (int, float)) and expected_samples:
        shortfall = max(0, int(expected_samples) - n_samples)
        sample_loss_pct = 100.0 * shortfall / int(expected_samples)

    framing_pct = 100.0 * framing / max(1, total_bytes)
    events_complete = (not isinstance(expected_events, (int, float))
                        or n_events == int(expected_events))

    problems: list[str] = []
    notes: list[str] = []

    if sample_loss_pct is not None:
        notes.append(f"샘플 손실: {shortfall}/{int(expected_samples)} "
                      f"({sample_loss_pct:.4f}%)")
        if sample_loss_pct > 1.0:
            problems.append("severe_sample_loss")
        elif shortfall > gap_budget:
            problems.append("sample_loss_over_budget")

    notes.append(f"CRC 오류: {crc}건")
    if crc > max(GLITCH_MIN_COUNT, gap_budget):
        problems.append("high_crc_errors")

    notes.append(f"프레이밍 바이트 손실: {framing}B / {total_bytes}B "
                 f"({framing_pct:.5f}%, budget {framing_budget}B)")
    if framing > framing_budget:
        problems.append("framing_over_budget")

    if isinstance(expected_events, (int, float)):
        notes.append(f"이벤트: {n_events}/{int(expected_events)}"
                      + (" (완전)" if events_complete else " (불완전)"))
    if not events_complete:
        problems.append("missing_events")

    if meta.get("packet_order_errors"):
        problems.append("packet_reorder")
        notes.append(f"패킷 순서 오류: {meta['packet_order_errors']}건")
    if meta.get("malformed_rows"):
        problems.append("malformed_rows")
        notes.append(f"손상된 행: {meta['malformed_rows']}건")
    if meta.get("incomplete"):
        problems.append("capture_incomplete")
        notes.append(f"캡처 미완료: {meta.get('incomplete_reason')}")

    fatal = {"severe_sample_loss", "missing_events", "packet_reorder",
             "malformed_rows", "capture_incomplete"}
    if any(p in fatal for p in problems):
        verdict = "UNUSABLE"
    elif problems:
        verdict = "MARGINAL"
    else:
        verdict = "USABLE"

    return {
        "verdict": verdict,
        "problems": problems,
        "notes": notes,
        "sample_loss_pct": sample_loss_pct,
        "framing_loss_pct": framing_pct,
        "crc_errors": crc,
    }


def find_pio() -> str:
    found = shutil.which("pio") or shutil.which("platformio")
    if found:
        return found
    candidate = Path.home() / ".platformio" / "penv" / "Scripts" / "pio.exe"
    if candidate.exists():
        return str(candidate)
    raise FileNotFoundError("PlatformIO CLI를 찾지 못했습니다. 먼저 PlatformIO를 설치하세요.")


def upload_firmware(project_dir: Path, port: str) -> None:
    command = [find_pio(), "run", "-t", "upload", "--upload-port", port]
    print("# firmware build/upload:", " ".join(command))
    subprocess.run(command, cwd=project_dir, check=True)


class Device:
    def __init__(self, port: str, ready_timeout: float = READY_TIMEOUT_S):
        self.port = port
        self.ser = serial.Serial(
            port, BAUD, timeout=SERIAL_READ_TIMEOUT_S,
            write_timeout=SERIAL_WRITE_TIMEOUT_S)
        # Enlarge the OS RX buffer (Windows-only API) to survive brief PC stalls
        # during long captures without dropping samples. No-op / harmless elsewhere.
        try:
            self.ser.set_buffer_size(rx_size=SERIAL_RX_BUFFER_BYTES)
        except Exception:
            pass
        deadline = time.monotonic() + ready_timeout
        boot_lines: list[str] = []
        next_ping = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_ping:
                # Some USB-UART bridges do not reset the ESP32 when the port
                # opens, so the one-shot boot READY line may already be gone.
                self.ser.write(b"PING\n")
                self.ser.flush()
                next_ping = now + 0.5
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="backslashreplace").rstrip("\r\n")
            boot_lines.append(line)
            print(line)
            if (line.startswith("# READY") or
                    (line.startswith("# PONG") and "ready=1" in line)):
                break
        else:
            self.ser.close()
            raise TimeoutError("ESP32의 '# READY'를 받지 못했습니다. 포트/펌웨어를 확인하세요.")
        self.boot_lines = boot_lines
        self.ser.reset_input_buffer()

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def send(self, command: str) -> None:
        self.ser.write((command.strip() + "\n").encode("ascii"))
        self.ser.flush()

    def request_line(self, command: str, prefix: str,
                     timeout: float = COMMAND_RESPONSE_TIMEOUT_S) -> str:
        self.ser.reset_input_buffer()
        self.send(command)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.ser.readline().decode(
                "utf-8", errors="backslashreplace").rstrip()
            if line:
                print(line)
            if line.startswith(prefix):
                return line
        raise TimeoutError(f"'{command}' 응답에서 '{prefix}'를 받지 못했습니다.")

    def capture(self, command: str, outdir: Path,
                silence_timeout: float = CAPTURE_SILENCE_TIMEOUT_S) -> dict:
        family = command.split()[0].upper()
        if family not in END_MARKERS:
            raise ValueError(f"capture할 수 없는 명령: {command}")
        end_marker = END_MARKERS[family]
        outdir.mkdir(parents=True, exist_ok=False)
        meta = {
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "port": self.port,
            "baud": BAUD,
            "command": command,
            "end_marker_seen": False,
            "binary_begin_seen": False,
            "binary_end_seen": False,
            "binary_crc_errors": 0,
            "framing_bytes_discarded": 0,
            "bytes_decoded": 0,
            "packet_gaps": 0,
            "packet_order_errors": 0,
            "malformed_rows": 0,
            "n_samples": 0,
            "n_events": 0,
        }
        columns: list[str] | None = None
        raw_path = outdir / "raw.csv"
        log_path = outdir / "session.log"
        last_rx = time.monotonic()
        started = time.monotonic()
        rx_buffer = bytearray()
        last_packet_seq: int | None = None
        last_sample_t: int | None = None
        sample_periods_us: list[int] = []

        self.ser.reset_input_buffer()
        self.send(command)
        print(f"\n# {command} -> {outdir}")
        try:
            with raw_path.open("w", encoding="utf-8", newline="") as raw_file, \
                    log_path.open("w", encoding="utf-8", newline="") as log_file:
                csv_writer = csv.writer(raw_file, lineterminator="\n")

                def next_item():
                    """Return ('line', str), ('record', dict), or None on timeout."""
                    nonlocal last_rx
                    while True:
                        if rx_buffer.startswith(BINARY_MAGIC):
                            if len(rx_buffer) >= BINARY_RECORD.size:
                                packet = bytes(rx_buffer[:BINARY_RECORD.size])
                                try:
                                    record = decode_binary_record(packet)
                                except ValueError:
                                    meta["binary_crc_errors"] += 1
                                    del rx_buffer[0]
                                    continue
                                del rx_buffer[:BINARY_RECORD.size]
                                meta["bytes_decoded"] += BINARY_RECORD.size
                                return "record", record
                        elif (len(rx_buffer) == 1 and
                              rx_buffer[0] == BINARY_MAGIC[0]):
                            # A serial read may split the two-byte magic word.
                            # Keep the first byte until the next chunk arrives.
                            pass
                        elif rx_buffer and rx_buffer[0] in LINE_START_BYTES:
                            newline = rx_buffer.find(b"\n")
                            if newline >= 0:
                                payload = bytes(rx_buffer[:newline + 1])
                                del rx_buffer[:newline + 1]
                                return "line", payload.decode(
                                    "utf-8", errors="backslashreplace").rstrip("\r\n")
                        elif rx_buffer:
                            del rx_buffer[0]
                            meta["framing_bytes_discarded"] += 1
                            continue

                        payload = self.ser.read(4096)
                        if not payload:
                            return None
                        last_rx = time.monotonic()
                        rx_buffer.extend(payload)

                while True:
                    item = next_item()
                    if item is None:
                        if time.monotonic() - last_rx > silence_timeout:
                            meta["incomplete"] = True
                            meta["incomplete_reason"] = "serial_silence_timeout"
                            break
                        continue
                    kind, value = item
                    if kind == "line":
                        line = value
                        log_file.write(line + "\n")
                        if line.startswith("#META"):
                            parse_meta(line, meta)
                        elif line.startswith("#COLS"):
                            header = line[len("#COLS"):].strip()
                            columns = header.split(",")
                            meta["columns"] = columns
                            csv_writer.writerow(columns)
                        elif line.startswith("#BINARY_BEGIN"):
                            meta["binary_begin_seen"] = True
                            parse_meta("#META " + line[len("#BINARY_BEGIN"):], meta)
                        elif line.startswith("#BINARY_END"):
                            meta["binary_end_seen"] = True
                            parse_meta("#META " + line[len("#BINARY_END"):], meta)
                        elif line.startswith(("S,", "E,")):
                            fields = line.split(",")
                            if columns is None or len(fields) != len(columns):
                                meta["malformed_rows"] += 1
                                print(f"!! malformed row: {line[:160]}")
                                continue
                            csv_writer.writerow(fields)
                            meta["n_samples" if fields[0] == "S" else "n_events"] += 1
                        elif line.startswith("!!"):
                            print("\n" + line)
                        if line.startswith(end_marker):
                            meta["end_marker_seen"] = True
                            break
                        continue

                    record = value
                    if columns is None:
                        meta["malformed_rows"] += 1
                        continue
                    csv_writer.writerow([
                        record.get(column, "") if not (
                            record["rec"] == "E" and column in {
                                "ax", "ay", "az", "gx", "gy", "gz"}) else ""
                        for column in columns
                    ])
                    packet_seq = record["packet_seq"]
                    if last_packet_seq is not None:
                        expected = (last_packet_seq + 1) & 0xFFFFFFFF
                        if packet_seq != expected:
                            gap = (packet_seq - expected) & 0xFFFFFFFF
                            if gap < 0x80000000:
                                meta["packet_gaps"] += gap
                            else:
                                meta["packet_order_errors"] += 1
                    last_packet_seq = packet_seq
                    if record["rec"] == "S":
                        meta["n_samples"] += 1
                        if last_sample_t is not None:
                            dt = record["t_us"] - last_sample_t
                            if 0 < dt <= 5 * round(1_000_000 / EXPECTED_STREAM_RATE_HZ):
                                sample_periods_us.append(dt)
                        last_sample_t = record["t_us"]
                    else:
                        meta["n_events"] += 1
                    if (meta["n_samples"] and
                            meta["n_samples"] % PROGRESS_FLUSH_EVERY_SAMPLES == 0):
                        raw_file.flush()
                        log_file.flush()
                        print(f"\r  samples={meta['n_samples']:7d} "
                              f"events={meta['n_events']:3d}", end="", flush=True)
        except KeyboardInterrupt:
            self.send("!")
            meta["incomplete"] = True
            meta["incomplete_reason"] = "keyboard_interrupt"
            raise
        except (OSError, serial.SerialException) as exc:
            try:
                self.send("!")
            except Exception:
                pass
            meta["incomplete"] = True
            meta["incomplete_reason"] = type(exc).__name__
            meta["capture_error"] = str(exc)
        finally:
            meta["duration_s"] = round(time.monotonic() - started, 3)
            if sample_periods_us:
                target_period = 1_000_000 / EXPECTED_STREAM_RATE_HZ
                median_period = statistics.median(sample_periods_us)
                period_errors = sorted(abs(x - target_period)
                                       for x in sample_periods_us)
                p99_index = min(len(period_errors) - 1,
                                int(0.99 * len(period_errors)))
                meta["sample_period_us_median"] = median_period
                meta["sample_period_error_us_p99"] = period_errors[p99_index]
                meta["effective_sample_rate_hz"] = round(
                    1_000_000 / median_period, 6)
            (outdir / "meta.json").write_text(
                json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

        print(f"\n# samples={meta['n_samples']} events={meta['n_events']} "
              f"duration={meta['duration_s']:.1f}s")
        errors = []
        if not meta["end_marker_seen"]:
            errors.append(f"missing {end_marker}")
        if not meta["binary_begin_seen"] or not meta["binary_end_seen"]:
            errors.append("binary stream marker missing")
        # Over a multi-minute continuous stream, a handful of records can be lost
        # or corrupted by USB-bridge hiccups. The measurements average many samples
        # per dwell / step, so a tiny fraction of loss is harmless; the analysis is
        # also gap-robust (no unwrap). Allow a small budget, still catching a real
        # systematic fault (which produces orders of magnitude more). packet_order
        # errors stay strict -- reordering means a genuine protocol problem.
        gap_budget = max(GLITCH_MIN_COUNT,
                         int(GLITCH_MAX_FRACTION * max(meta["n_samples"], 1)))
        total_bytes = meta["bytes_decoded"] + meta["framing_bytes_discarded"]
        framing_budget = max(FRAMING_BYTES_DISCARD_MIN,
                             int(FRAMING_BYTES_DISCARD_MAX_FRACTION * total_bytes))
        if meta["binary_crc_errors"] > GLITCH_MIN_COUNT:
            errors.append(f"binary_crc_errors={meta['binary_crc_errors']}")
        if meta["framing_bytes_discarded"] > framing_budget:
            errors.append(f"framing_bytes_discarded={meta['framing_bytes_discarded']} "
                          f"(budget {framing_budget})")
        if meta["packet_gaps"] > gap_budget or meta["packet_order_errors"]:
            errors.append(
                f"packet_gaps={meta['packet_gaps']} (budget {gap_budget}) "
                f"packet_order_errors={meta['packet_order_errors']}")
        if meta["malformed_rows"]:
            errors.append(f"malformed_rows={meta['malformed_rows']}")
        if meta.get("incomplete"):
            errors.append(str(meta.get("incomplete_reason", "incomplete")))
        if columns is None or meta["n_samples"] == 0:
            errors.append("no sample table")
        if last_value(meta.get("format")) != BINARY_FORMAT:
            errors.append(f"unexpected binary format={last_value(meta.get('format'))}")
        if last_value(meta.get("record_bytes")) != BINARY_RECORD.size:
            errors.append(
                f"record_bytes={last_value(meta.get('record_bytes'))} "
                f"expected={BINARY_RECORD.size}")
        if last_value(meta.get("stream_hz")) != EXPECTED_STREAM_RATE_HZ:
            errors.append(
                f"stream_hz={last_value(meta.get('stream_hz'))} "
                f"expected={EXPECTED_STREAM_RATE_HZ}")
        if last_value(meta.get("serial_short_writes", 0)) != 0:
            errors.append(
                f"serial_short_writes={last_value(meta.get('serial_short_writes'))}")
        expected_samples = last_value(meta.get("expected_samples"))
        expected_events = last_value(meta.get("expected_events"))
        # Share gap_budget with the packet_gaps check above: a strict equality here
        # would fire on any single lost record and make that budget unreachable, so
        # captures well inside the documented tolerance still aborted. A surplus is
        # never tolerated -- more samples than the firmware announced means a
        # protocol fault, not a dropped record.
        if isinstance(expected_samples, (int, float)):
            shortfall = int(expected_samples) - meta["n_samples"]
            if shortfall < 0 or shortfall > gap_budget:
                errors.append(f"samples={meta['n_samples']} "
                              f"expected={int(expected_samples)} (budget {gap_budget})")
        if isinstance(expected_events, (int, float)) and meta["n_events"] != int(expected_events):
            errors.append(f"events={meta['n_events']} expected={int(expected_events)}")
        if family == "HEALTH" and not as_bool(meta.get("health_pass")):
            errors.append("firmware health FAIL")
        if family in {"A", "B"} and not as_bool(meta.get("firmware_test_pass")):
            errors.append("firmware test FAIL")
        if family in {"A", "B"}:
            if not as_bool(meta.get("health_pass")):
                errors.append("motion run started without health PASS")
        meta["capture_ok"] = not errors
        meta["errors"] = errors
        meta["usability"] = build_usability(meta, gap_budget, framing_budget, total_bytes)
        print(f"\n# 데이터 사용 가능성 판정: {meta['usability']['verdict']}")
        for note in meta["usability"]["notes"]:
            print(f"  - {note}")
        (outdir / "meta.json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        if errors:
            raise RuntimeError(f"{command} capture 실패: " + ", ".join(errors))
        return meta
