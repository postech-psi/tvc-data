"""Guided calibration followed by outer/inner TVC gimbal tests."""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

from analyze import analyze_run, finite_json
from sid_capture import Device, available_ports, upload_firmware

PROJECT_DIR = Path(__file__).resolve().parent

# =============================================================================
# USER PC-WORKFLOW PARAMETERS
# Firmware motion/sample parameters are at the top of src/main.cpp.
# Analysis acceptance parameters are at the top of analyze.py.
# =============================================================================
EXPECTED_IMU_COUNT = 1
EXPECTED_GIMBAL_COUNT = 2
MOTION_CAPTURE_SILENCE_TIMEOUT_S = 45.0
# Settle time after ATTACH+NEUTRAL before the post-calibration static health capture.
NEUTRAL_SETTLE_S = 3.0
# Default sequence (mapping + step). Chirp (C) and deadband (D) are optional
# follow-ups; run them by name with --tests, e.g. --tests "C OUTER,C INNER".
TEST_SCHEDULE = [
    ("A OUTER", "mapping_outer_gimbal"),
    ("A INNER", "mapping_inner_gimbal"),
    ("B OUTER", "step_outer_gimbal"),
    ("B INNER", "step_inner_gimbal"),
]
ALL_TESTS = TEST_SCHEDULE + [
    ("C OUTER", "chirp_outer_gimbal"),
    ("C INNER", "chirp_inner_gimbal"),
    ("D OUTER", "deadband_outer_gimbal"),
    ("D INNER", "deadband_inner_gimbal"),
    ("E", "grid_both_gimbals"),
]

def prevent_sleep(enable: bool) -> None:
    """Keep Windows awake during a long capture. Laptop sleep / USB selective
    suspend stalls the serial reader and drops ~100 ms+ of samples (packet_gaps).
    No-op on non-Windows."""
    if sys.platform != "win32":
        return
    import ctypes
    ES_CONTINUOUS = 0x80000000
    ES_SYSTEM_REQUIRED = 0x00000001
    ES_DISPLAY_REQUIRED = 0x00000002
    flags = ES_CONTINUOUS
    if enable:
        flags |= ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED
    try:
        ctypes.windll.kernel32.SetThreadExecutionState(flags)
    except Exception:
        pass


def require_enter(message: str, assume_yes: bool = False) -> None:
    print("\n" + message)
    if assume_yes:
        print("# --yes: 확인 입력을 생략합니다.")
        return
    answer = input("준비됐으면 Enter, 중단하려면 q: ").strip().lower()
    if answer == "q":
        raise KeyboardInterrupt


def verify_identity(device: Device) -> None:
    line = device.request_line("PING", "# PONG")
    imu = re.search(r"imu_count=(\d+)", line)
    gimbals = re.search(r"gimbal_count=(\d+)", line)
    ready = re.search(r"ready=(\d+)", line)
    if not imu or int(imu.group(1)) != EXPECTED_IMU_COUNT:
        raise RuntimeError(
            f"펌웨어가 IMU {EXPECTED_IMU_COUNT}개를 확인하지 못했습니다.")
    if not gimbals or int(gimbals.group(1)) != EXPECTED_GIMBAL_COUNT:
        raise RuntimeError(
            f"펌웨어 gimbal_count가 {EXPECTED_GIMBAL_COUNT}가 아닙니다.")
    if not ready or int(ready.group(1)) != 1:
        raise RuntimeError("ICM-20948 초기화 실패: 주소 0x69와 배선을 확인하세요.")


def write_session_report(session: Path, results: dict, status: str) -> None:
    report = [
        "# TVC system-identification session report", "",
        f"- Status: **{status}**",
        f"- Session: `{session.name}`", "",
        "## Results", "",
    ]
    for name, result in results.items():
        report.append(f"### {name}")
        report.append("")
        report.append(f"- PASS: `{result.get('pass')}`")
        if result.get("error"):
            report.append(f"- error: `{result['error']}`")
        if result.get("fail_reasons"):
            report.append("- fail_reasons:")
            for reason in result["fail_reasons"]:
                report.append(f"    - {reason}")
        usability = result.get("usability")
        if usability:
            report.append(f"- data usability: **{usability['verdict']}**")
            for note in usability.get("notes", []):
                report.append(f"    - {note}")
        for key in ("gimbal", "g_mag_mean", "gain_deg_per_us", "neutral_us",
                    "angle_min_deg", "angle_max_deg", "max_abs_angle_deg",
                    "travel_span_deg", "hysteresis_max_deg", "direct_onset_ms",
                    "rise_10_90_ms", "settling_2pct_ms", "bandwidth_hz",
                    "peak_slew_dps", "tail_rate_rms_dps", "recommend_chirp",
                    "bandwidth_3db_hz", "phase_delay_ms", "coherent_band_hi_hz",
                    "backlash_deg", "backlash_us", "local_gain_deg_per_us"):
            if key in result:
                report.append(f"- {key}: `{result[key]}`")
        report.append("")
    report.extend([
        "## Interpretation", "",
        "- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.",
        "- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.",
        "- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.",
        "- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.",
    ])
    (session / "REPORT.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (session / "session_results.json").write_text(
        json.dumps(finite_json({"status": status, "results": results}),
                   ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    # fail_reasons and other status text are Korean; the default Windows
    # console codepage (cp1252) can't encode it and would crash print().
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="ESP32 serial port, e.g. COM7")
    parser.add_argument("--list", action="store_true", help="serial port 목록")
    parser.add_argument("--upload", action="store_true",
                        help="실험 전 PlatformIO build/upload 실행")
    parser.add_argument("--calibration", type=Path, default=PROJECT_DIR / "calibration.json",
                        help="calibration.json 경로 (기본: 프로젝트 루트). calibrate.py로 생성.")
    parser.add_argument("--runs", type=Path, default=PROJECT_DIR / "runs")
    parser.add_argument("--yes", action="store_true",
                        help="기존 calibration 사용 시 안전 확인 Enter 생략")
    parser.add_argument(
        "--tests", help="실행할 테스트만 콤마로 지정 (예: \"C OUTER,C INNER\"). "
        "생략 시 기본 A/B 4개. 선택지: " + ", ".join(c for c, _ in ALL_TESTS))
    parser.add_argument("--keep-going", action="store_true",
                        help="한 테스트가 실패해도 중단하지 않고 다음 테스트를 계속")
    args = parser.parse_args()

    active_schedule = TEST_SCHEDULE
    if args.tests:
        wanted = [t.strip().upper() for t in args.tests.split(",") if t.strip()]
        by_cmd = {c.upper(): (c, n) for c, n in ALL_TESTS}
        unknown = [w for w in wanted if w not in by_cmd]
        if unknown:
            parser.error(f"알 수 없는 테스트: {unknown}. 선택지: {list(by_cmd)}")
        active_schedule = [by_cmd[w] for w in wanted]

    if args.list or not args.port:
        for port, description in available_ports():
            print(f"{port:16s} {description}")
        return 0 if args.list else 2
    if not args.calibration.exists():
        parser.error(f"calibration.json이 없습니다: {args.calibration}\n"
                     f"먼저 calibrate.py로 보정을 만드세요.")

    if args.upload:
        upload_firmware(PROJECT_DIR, args.port)

    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    session = args.runs / f"session_{stamp}"
    session.mkdir(parents=True, exist_ok=False)
    results: dict[str, dict] = {}
    device: Device | None = None
    prevent_sleep(True)
    try:
        device = Device(args.port)
        verify_identity(device)
        device.request_line("DETACH", "# DETACHED")

        # Calibration is produced separately by calibrate.py; here we only load it.
        calibration_path = args.calibration.resolve()
        calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
        if not calibration.get("pass"):
            raise RuntimeError("지정한 calibration.json이 PASS가 아닙니다.")
        (session / "calibration_source.txt").write_text(
            str(calibration_path), encoding="utf-8")

        require_enter(
            "[준비] 모션 시험 전 최종 확인\n"
            "- 실제 모터/노즐 또는 동일 질량 더미 장착\n"
            "- 베이스 고정, 케이블 장력 없음, 두 축 이동 경로에 사람/물체 없음\n"
            "- 서보 전원 7.4 V, 3 A 이상 BEC와 공통 GND\n"
            "- 뒷판 IMU 단단히 고정, 두 회전축이 중력에 수평\n"
            "확인 후 attach → health → outer/inner mapping → outer/inner step 자동 실행.",
            args.yes)
        device.request_line("ATTACH", "# ATTACHED")
        device.request_line("NEUTRAL", "# NEUTRAL")
        # Let the servos/structure settle after attach+neutral before the static
        # health capture; otherwise the ~1-2 s attach transient inflates gyro/accel
        # noise and fails firmware health even though the rig is fine once settled.
        print(f"# 서보 정착 대기 {NEUTRAL_SETTLE_S:.0f}s...")
        time.sleep(NEUTRAL_SETTLE_S)

        health_dir = session / "health"
        health_capture_meta = device.capture("HEALTH", health_dir)
        results["health"] = analyze_run(health_dir, calibration_path)
        results["health"]["usability"] = health_capture_meta.get("usability")
        if not results["health"]["pass"]:
            reasons = results["health"].get("fail_reasons") or ["(사유 미상)"]
            for reason in reasons:
                print(f"  - FAIL: {reason}", file=sys.stderr)
            raise RuntimeError("health 분석 FAIL: " + "; ".join(reasons))

        for command, name in active_schedule:
            run_dir = session / name
            try:
                capture_meta = device.capture(
                    command, run_dir,
                    silence_timeout=MOTION_CAPTURE_SILENCE_TIMEOUT_S)
                results[name] = analyze_run(run_dir, calibration_path)
                results[name]["usability"] = capture_meta.get("usability")
                print(json.dumps(finite_json(results[name]),
                                 ensure_ascii=False, indent=2))
                if not results[name]["pass"]:
                    reasons = results[name].get("fail_reasons") or ["(사유 미상)"]
                    for reason in reasons:
                        print(f"  - FAIL: {reason}", file=sys.stderr)
                    raise RuntimeError(
                        f"{name} 분석 FAIL: " + "; ".join(reasons))
            except Exception as exc:
                # capture() writes meta.json (with its usability verdict) before
                # raising, so a capture-stage failure still leaves that judgment
                # on disk -- recover it instead of losing it behind a bare error
                # string. Preserve whatever analyze_run already produced too
                # (fail_reasons, metrics).
                usability = results.get(name, {}).get("usability")
                if usability is None:
                    meta_path = run_dir / "meta.json"
                    if meta_path.exists():
                        try:
                            usability = json.loads(
                                meta_path.read_text(encoding="utf-8")).get("usability")
                        except Exception:
                            usability = None
                results[name] = {**results.get(name, {}),
                                  "pass": False, "error": str(exc),
                                  "usability": usability}
                print(f"\n!! {name} 실패: {exc}", file=sys.stderr)
                if usability:
                    print(f"  # 데이터 사용 가능성 판정(참고): {usability['verdict']}",
                          file=sys.stderr)
                    for note in usability.get("notes", []):
                        print(f"    - {note}", file=sys.stderr)
                # Recover to a safe neutral before the next test.
                try:
                    device.send("!")
                    device.request_line("NEUTRAL", "# NEUTRAL", timeout=3.0)
                except Exception:
                    pass
                if not args.keep_going:
                    reasons = results[name].get("fail_reasons")
                    reason_text = (" (" + "; ".join(reasons) + ")") if reasons else ""
                    raise RuntimeError(
                        f"{name} 실패로 중단{reason_text}. 이 테스트를 건너뛰고 계속하려면 "
                        f"--keep-going, 특정 테스트만 하려면 --tests 를 쓰세요.")
                continue
            device.request_line("NEUTRAL", "# NEUTRAL")

        device.request_line("DETACH", "# DETACHED")
        all_pass = bool(results) and all(r.get("pass") for r in results.values())
        status = "PASS" if all_pass else "PARTIAL"
        write_session_report(session, results, status)
        print(f"\n# 실험 종료 ({status}): {session / 'REPORT.md'}")
        return 0
    except KeyboardInterrupt:
        print("\n!! 사용자 중단")
        if device:
            try: device.send("!")
            except Exception: pass
        write_session_report(session, results, "INTERRUPTED")
        return 130
    except Exception as exc:
        print(f"\n!! 실험 중단: {exc}", file=sys.stderr)
        if device:
            try:
                device.send("!")
                device.request_line("DETACH", "# DETACHED", timeout=2.0)
            except Exception:
                pass
        write_session_report(session, results, f"FAIL: {exc}")
        return 1
    finally:
        prevent_sleep(False)
        if device:
            device.close()


if __name__ == "__main__":
    raise SystemExit(main())
