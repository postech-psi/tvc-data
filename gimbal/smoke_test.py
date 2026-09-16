"""Upload optionally, then run the low-amplitude outer/inner servo smoke test."""
from __future__ import annotations

import argparse
import re
from pathlib import Path

from sid_capture import Device, upload_firmware

PROJECT_DIR = Path(__file__).resolve().parent
SMOKE_TIMEOUT_S = 20.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 serial port, e.g. COM14")
    parser.add_argument("--upload", action="store_true",
                        help="run PlatformIO build/upload before the smoke test")
    parser.add_argument("--mode", choices=("slow", "fast", "both"),
                        default="slow",
                        help="motion mode; both runs slow first, then fast")
    args = parser.parse_args()

    if args.upload:
        upload_firmware(PROJECT_DIR, args.port)

    device: Device | None = None
    try:
        device = Device(args.port)
        modes = ("slow", "fast") if args.mode == "both" else (args.mode,)
        for mode in modes:
            line = device.request_line(
                f"SMOKE {mode.upper()}", "# SMOKE_END",
                timeout=SMOKE_TIMEOUT_S)
            passed = re.search(r"\bpass=1\b", line) is not None
            neutral = "outer_us=1520" in line and "inner_us=1520" in line
            detached = "attached=0" in line
            reported_mode = f"mode={mode}" in line
            if not (passed and neutral and detached and reported_mode):
                raise RuntimeError(f"{mode} smoke test 종료 상태 이상: {line}")
            print(f"# {mode.upper()} SMOKE PASS: 두 축 중립 복귀 및 detach 확인")
        return 0
    except KeyboardInterrupt:
        if device:
            try:
                device.send("!")
            except Exception:
                pass
        print("!! 사용자 중단: emergency detach 요청")
        return 130
    finally:
        if device:
            device.close()


if __name__ == "__main__":
    raise SystemExit(main())
