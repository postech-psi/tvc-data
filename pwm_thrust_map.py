#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLDC 추력 맵핑용 2D PWM 격자 '계단식' 스윕 + 로깅 — HTML GUI 버전 (측정 로직 + 로컬 웹서버 한 파일)

[이 파일이 하는 일]
  1) 파이썬 표준 라이브러리만으로 '작은 로컬 웹서버'를 띄운다.
  2) 브라우저에서 http://localhost:8000 을 열면 옆에 있는 pwm_map_gui.html 을 배달한다.
  3) GUI에서 [측정 시작]을 누르면, 그 설정대로 픽스호크에 PWM(액추에이터 테스트)을 보내며
     실제 PWM(µs)·전압·전류를 CSV로 기록한다. (pwm_thrust_grid.py 와 같은 방식/같은 컬럼)
  4) 측정 진행상황을 GUI가 0.5초마다 물어보면(GET /status) 알려준다(진행바·현재값·남은시간).

  ※ 이 파일은 pwm_thrust_grid.py 를 import 하지 않는다. 공통으로 쓰는 함수는
    (요청대로) 복사해서 이 파일 안에 독립적으로 두었다. pymavlink 만 외부 의존성.

[측정 구조 — 팀과 합의한 계획 (계단식)]
  - 격자 범위 입력: A(고정축) 시작/끝/스텝, B(스윕축) 시작/끝/스텝 → (A,B) 조합 목록 자동 생성.
      · A·B의 시작=끝 으로 넣으면 조합이 1개 → "배터리 아껴 한 조합만" 케이스도 이걸로 커버.
  - '계단식': 각 조합을 dwell초 동안 유지·기록한 뒤, 멈추거나 쉬지 않고 곧바로 다음 조합 PWM으로
    올라간다(정지/안정화/간격 단계 없음).
  - 스윕 반복(N): 격자 전체를 한 바퀴 도는 것을 1스윕이라 하면, 이를 N번 반복한다(반복 사이도 안 쉼).
  - 안전을 위해 '맨 마지막 종료 시에만' 서서히 정지(램프다운)한다.

[동작 원리 — PX4 공식 경로]  (pwm_thrust_grid.py 와 동일)
  * MAV_CMD_ACTUATOR_TEST(310): '출력 기능(Motor1/Motor2)' 단위로 값을 직접 구동.
    QGroundControl Actuators 화면의 테스트 슬라이더가 쓰는 바로 그 명령.
  * 명령값은 '정규화값'(µs가 아님). 모터 정의역 [0,1]: 0→최소(PWM_MIN_US), 1→최대(PWM_MAX_US).
    선형:  µs = MIN + value*(MAX-MIN),  value = (µs-MIN)/(MAX-MIN).
  * 이 명령엔 타임아웃(초)이 있어 그 시간 뒤 기본값으로 복귀 → 값을 '유지'하려면 계속 재전송.
    재전송을 멈추면 자동으로 꺼진다(안전상 무한 명령 불가).
  * PX4는 '시동(armed)' 상태에서는 이 명령을 거부 → 반드시 '시동 해제(disarmed)'로 실행.
  * 실제 나가는 µs는 SERVO_OUTPUT_RAW(#36)로 '실측'해서 기록(계산값이 아니라 실측이 진리값).

[사전 설정 — QGC에서 반드시 먼저 맞출 것]
  * ESC가 물린 채널에서 Minimum=1000, Maximum=2000, Disarmed=1000
  * THR_MDL_FAC = 0  (추력곡선 보정 끄기 → 정규화↔PWM 이 완전 선형)


[실행]
    python3 pwm_thrust_map.py          # 서버 시작 → 브라우저에서 localhost:8000
    (개발/검증) 브라우저 GUI의 device 칸에 udpin:0.0.0.0:14550 을 넣으면 SITL/QGC로 로직만 확인
    종료: 이 터미널에서 Ctrl+C (측정 중이면 모터 정지+CSV 저장 후 종료)

의존성: pymavlink + 파이썬 표준 라이브러리(http.server, threading, json, csv ...) 뿐.
"""

import argparse
import csv
import json
import os
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pymavlink import mavutil   # 유일한 외부 라이브러리


# ============================================================================
# 고정 상수 — 하드웨어/프로토콜 관련. 웬만하면 안 건드림(대부분 grid 파일과 동일).
#   (실험마다 바뀌는 값들은 GUI에서 넘어오는 cfg 로 받는다 — 아래 DEFAULT_CFG 참고)
# ============================================================================
# QGC에서 맞춰둔 값과 '반드시' 일치시킬 것 (아래 계산이 실제 출력과 같아지려면)
PWM_MIN_US = 1000         # 정규화 0.0 이 되는 µs (= QGC Minimum)
PWM_MAX_US = 2000         # 정규화 1.0 이 되는 µs (= QGC Maximum)

# 출력 기능 번호 (MAV_ACTUATOR_OUTPUT_FUNCTION 규약): Motor1=1, Motor2=2, ...
MOTOR_A_FUNC = 1          # 고정(바깥 루프) 모터 — 화면상 MAIN1 = Motor1
MOTOR_B_FUNC = 2          # 스윕(안쪽 루프) 모터 — 화면상 MAIN2 = Motor2

# MAV_CMD_ACTUATOR_TEST = 310. 구버전 pymavlink엔 상수가 없을 수 있어 정수로 폴백.
ACTUATOR_TEST = getattr(mavutil.mavlink, "MAV_CMD_ACTUATOR_TEST", 310)

# 모터 정지값: v=0 → MIN(=1000µs) = 스로틀 0 = 정지(ESC 캘리브레이션 전제, disarmed=1000과 동일).
STOP_VALUE = 0.0

# --- 안전장치 관련 숫자 ---
WATCHDOG_TIMEOUT_S = 2.0  # GUI(브라우저)가 이 시간 동안 status를 안 물어보면 '끊김'으로 보고 즉시 정지
RAMP_STEPS = 8            # '서서히 정지(램프다운)' 단계 수 (맨 마지막 종료에만 사용)
RAMP_S = 0.4              # 램프다운 전체 소요 시간(초)
# 수신 버퍼를 한 번에 비울 최대 메시지 수. 이 상한이 있어야 메시지가 아무리 빨리
# 들어와도 주기적으로 바깥 루프(=deadline·STOP·watchdog 확인)로 돌아온다.
MAX_DRAIN_PER_LOOP = 40

# 서버 기본 포트/호스트. 호스트는 localhost(127.0.0.1)만 열어 라즈베리파이 자기 자신에서만 접속(안전).
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8000

OUT_PREFIX = "thrust_map"   # 출력 CSV 파일명 앞부분

# GUI에서 값이 일부만 오거나 이상해도 안전하게 돌도록 하는 '기본 설정'.
# (GUI가 보낸 값으로 덮어쓴 뒤, validate_cfg 로 최종 검증한다)
DEFAULT_CFG = {
    "device": "/dev/ttyAMA0",   # Pi5 GPIO UART(핀 8·10). 개발용은 "udpin:0.0.0.0:14550"
    "baud": 921600,             # 픽스호크 TELEM2 권장 속도
    "a_start": 1000, "a_end": 1000, "a_step": 100,   # A(고정축) 격자 범위 µs
    "b_start": 1000, "b_end": 2000, "b_step": 100,   # B(스윕축) 격자 범위 µs
    "repeats": 1,               # 격자 전체 스윕을 몇 번 반복할지 (1이면 한 바퀴)
    "dwell_s": 2.0,             # 각 조합에서 '유지+기록' 시간 (계단 한 칸의 길이)
    "resend_hz": 5.0,           # ACTUATOR_TEST 재전송 주기(타임아웃으로 값이 풀리지 않게)
    "timeout_s": 1.0,           # 각 명령의 타임아웃(초). 재전송 간격(1/resend_hz)보다 커야 함
    "warmup_s": 3.0,            # 시작 시 ESC arming 을 위해 최소값(0)으로 잠깐 대기
    # --- 측정 설계(교란 제거용) ---
    "idle_s": 10.0,             # 스윕 전/후 '무부하' 구간을 기록(양 채널 1000µs, 전류≈0).
                                # 여기서 읽은 전압이 배터리의 '실제 잔량'(SoC)이다.
                                # 부하 중 전압은 IR 강하가 섞여 있어 런끼리 비교 불가.
    "servo_hz": 50,             # SERVO_OUTPUT_RAW 요청 주기. CSV 한 줄이 이 메시지마다 나온다.
    "bat_hz": 20,               # BATTERY_STATUS 요청 주기
    "esc_hz": 20,               # ESC_STATUS(RPM) 요청 주기. 미지원 ESC면 그냥 안 온다.
    "min_voltage_v": 0.0,       # 이 전압 아래로 내려가면 즉시 중단(리포 보호). 3S면 9.9 권장.
    "randomize": False,         # 계단 순서 섞기 → 전압 드리프트와 PWM의 상관을 끊는다
    "bracket": False,           # 첫 조합을 맨 뒤에 반복 → 스윕 중 드리프트를 직접 측정
    "seed": 0,                  # randomize 재현용 시드(0이면 매번 다름)
    # --- 아래는 데이터 정리용(측정 자체에는 영향 없음) ---
    "out_dir": "",              # CSV 저장 폴더(빈 값이면 현재 폴더). 예: "raw/2026-07-25"
    "notes": "",                # 이 런에 대한 자유 메모
    "prop": "",                 # 프로펠러 사양
    "battery": "",              # 배터리 사양(셀 수/용량 등)
}

# CSV 컬럼(헤더). pwm_thrust_grid.py 와 '동일' + 맨 뒤에 sweep_idx(몇 번째 스윕인지) 한 개만 추가.
#   servo1~8_raw = 각 물리 출력 채널의 '실제' PWM(µs) 실측값.
#   → 로드셀 로그와 t_epoch(에폭 시각)로 병합할 때 기존 파이프라인과 그대로 호환된다.
#   esc1~4_rpm   = ESC 텔레메트리(ESC_STATUS #291)의 실측 RPM. 지원 ESC가 없으면 빈 칸.
#     → RPM 이 있으면 '전압이 추력을 바꾼다'와 '전압이 RPM 을, RPM 이 추력을 바꾼다'를
#       분리할 수 있다. 지금 데이터로는 이 둘이 섞여 있어 구분이 불가능하다.
CSV_HEADER = (
    ["t_epoch", "t_fc_us", "phase",
     "a_cmd_us", "b_cmd_us", "a_cmd_norm", "b_cmd_norm"]
    + [f"servo{i}_raw" for i in range(1, 9)]
    + ["voltage_v", "current_a", "sweep_idx"]
    + [f"esc{i}_rpm" for i in range(1, 5)]
)


# ============================================================================
# 예외 — 측정 도중 '중단'을 위쪽으로 던지는 신호(즉시 정지로 이어짐)
# ============================================================================
class AbortMeasurement(Exception):
    """STOP 버튼 또는 watchdog(브라우저 끊김)으로 측정을 즉시 중단할 때 던지는 예외."""
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


# ============================================================================
# MAVLink 헬퍼 (모두 이 파일 안에서 정의 — grid 파일에서 복사, import 없음)
# ============================================================================
def us_to_norm(us):
    """목표 PWM(µs)을 액추에이터 테스트로 보낼 정규화값(0~1)으로 변환.

    모터 정의역 [0,1] 선형 매핑:  norm = (us - MIN) / (MAX - MIN)
    (QGC의 Minimum/Maximum 과 PWM_MIN_US/PWM_MAX_US 가 같아야 실제 출력과 일치한다)
    범위를 벗어난 µs가 들어와도 0~1로 잘라(clamp) 폭주를 막는다.
    """
    norm = (us - PWM_MIN_US) / float(PWM_MAX_US - PWM_MIN_US)
    return max(0.0, min(1.0, norm))


def send_actuator_test(conn, func, value, timeout_s):
    """출력 기능(func)을 정규화값 value 로 timeout_s초 동안 구동하라고 1회 명령한다.

    command_long 파라미터 매핑:
        param1=value(0~1), param2=timeout_s(이 시간 뒤 기본값 복귀),
        param3,4=예약(0), param5=func(Motor1=1, Motor2=2 ...), param6,7=미사용(0)
    """
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        ACTUATOR_TEST,
        0,                 # confirmation
        float(value),      # param1
        float(timeout_s),  # param2
        0.0, 0.0,          # param3, param4 (예약)
        float(func),       # param5
        0.0, 0.0,          # param6, param7
    )


def check_armed(conn, timeout=3.0):
    """HEARTBEAT의 base_mode로 시동 여부 확인. True=시동, False=시동해제, None=판단불가."""
    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if hb is None:
        return None
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def request_rates(conn, servo_hz=50, bat_hz=20, esc_hz=20):
    """필요한 메시지의 송신 주기를 SET_MESSAGE_INTERVAL로 올린다(공식 권장 방식).

    - SERVO_OUTPUT_RAW: 실제 출력 PWM(µs)  → 우리가 기록할 '진리값'
    - BATTERY_STATUS  : 배터리 전압/전류. QGroundControl이 화면에 쓰는 바로 그 메시지라
                        여기서 읽으면 QGC 표시값과 동일해진다(캘리브레이션은 FC에서 적용됨).
    - ESC_STATUS      : ESC 텔레메트리(RPM). DShot/BLHeli 등 텔레메트리 지원 ESC + PX4
                        설정(예: DSHOT_TEL_CFG)이 있어야 나온다. 없으면 그냥 안 올 뿐이라
                        요청해도 손해는 없다.
    interval(µs) = 1e6 / rate_hz

    대역폭: 921600 baud ≈ 92 kB/s. 위 세 메시지를 50/20/20 Hz 로 받아도
    (49B*50 + 66B*20 + 46B*20) ≈ 4.7 kB/s 로 약 5% 에 불과하다. 즉 상한은
    시리얼 대역폭이 아니라 PX4 내부 발행 주기와 MAV_x_RATE 설정이다.
    """
    targets = [
        (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, servo_hz),
        (mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, bat_hz),
    ]
    esc_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_ESC_STATUS", 291)
    targets.append((esc_id, esc_hz))

    for msg_id, hz in targets:
        if not hz or hz <= 0:
            continue
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,            # param1: 메시지 ID
            int(1e6 / hz),     # param2: 주기(µs)
            0, 0, 0, 0, 0,
        )


def read_battery(msg):
    """BATTERY_STATUS(#147)에서 QGroundControl과 '완전히 동일한 알고리즘'으로 (전압V, 전류A)를 계산.

    QGC 소스(src/Vehicle/FactGroups/BatteryFactGroupListModel.cc)의 전압 계산과 일치시킨다:
      · voltages[0..9]를 순서대로 더하되, UINT16_MAX(65535)를 '처음' 만나면 즉시 멈춘다(break).
      · 이어서 voltages_ext[0..3]를 더하되, 0(미지원)을 '처음' 만나면 즉시 멈춘다(break).
      · current_battery: -1이면 미측정(None), 아니면 cA(10mA단위)→A.
    MAVLink 스펙상 PX4는 유효 셀을 index 0부터 '연속'으로 채우고 나머지를 UINT16_MAX로 두므로,
    '건너뛰기'가 아니라 '처음 무효값에서 멈춤'이 정확한 재구성이며 QGC 표시값과 값이 완전 일치한다.
    (셀 정보가 없으면 총전압이 voltages[0]에 통째로 담긴다. 전압 divider 캘리브레이션은 QGC가
     아니라 픽스호크(PX4) 펌웨어에서 이미 적용되어 전송된다.)
    반환: (voltage_v 또는 None, current_a 또는 None)
    """
    total_mv = None
    for v in msg.voltages:                        # 셀 1~10 (mV)
        if v == 65535:                            # UINT16_MAX = 유효 셀의 끝 → 멈춤
            break
        total_mv = v if total_mv is None else total_mv + v
    for v in getattr(msg, "voltages_ext", []):    # 셀 11~14 (구버전 pymavlink엔 없을 수 있음)
        if v == 0:                                # 0 = 미지원 → 멈춤
            break
        total_mv = v if total_mv is None else total_mv + v
    voltage_v = total_mv / 1000.0 if total_mv is not None else None
    current_a = msg.current_battery / 100.0 if msg.current_battery != -1 else None
    return voltage_v, current_a


def preflight_ack(conn, timeout_s):
    """정지값(v=0)을 모터 A에 한 번 보내고 COMMAND_ACK로 픽스호크가 수락하는지 확인한다.

    반환: (ok: bool, 사람이 읽을 메시지 문자열). 여기서 실패해도 치명은 아님(진행 가능).
    v=0 은 최소(1000µs)=정지로 매핑되므로, 캘리브레이션된 ESC라면 모터가 돌지 않는다.
    """
    send_actuator_test(conn, MOTOR_A_FUNC, STOP_VALUE, timeout_s)
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=2.0)
    if ack is None or ack.command != ACTUATOR_TEST:
        return True, "ACTUATOR_TEST 에 대한 COMMAND_ACK 미수신(그래도 진행 가능)"
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        return True, "프리플라이트 OK: ACTUATOR_TEST 수락됨(ACCEPTED)"
    if ack.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
        return False, "거부(TEMPORARILY_REJECTED): 보통 '시동(armed)' 상태입니다. 시동 해제 후 재시도"
    return False, f"거부됨(result={ack.result}). QGC에서 Motor1/2 배정을 확인하세요"


def stop_motors_immediate(conn, timeout_s):
    """[긴급/최종] 두 모터를 정지값(v=0 → 1000µs)으로 즉시 몇 번 보내 끈다(램프 없음).

    STOP 버튼·watchdog·오류·종료 시 사용. 재전송을 멈추면 타임아웃으로도 정지된다.
    """
    for _ in range(5):
        send_actuator_test(conn, MOTOR_A_FUNC, STOP_VALUE, timeout_s)
        send_actuator_test(conn, MOTOR_B_FUNC, STOP_VALUE, timeout_s)
        time.sleep(0.05)


def _ramp_step(conn, a, b, timeout_s, log=None, phase="ramp"):
    """램프 한 칸: 명령을 보내고, 로깅 컨텍스트가 있으면 그 구간도 '기록'한다.

    램프 구간을 기록해야 하는 이유 — 여기가 추력이 가장 크게 변하는 지점이다.
    스윕 종료 램프다운은 추력을 ~9N 움직이는데(노이즈 0.7N 대비 13σ), 예전에는
    time.sleep 만 하고 아무것도 안 남겨서 이 '가장 뚜렷한 에지'가 CSV에 없었다.
    로드셀 로그와 시각을 맞출 때 가장 쓸모 있는 특징이 바로 이 구간이다.
    """
    if log is not None:
        # hold_and_log 이 명령 전송 + 수신 기록을 함께 처리한다
        hold_and_log(log["ctl"], conn, log["writer"], log["latest"], log["cfg"],
                     int(round(a)), int(round(b)), phase,
                     RAMP_S / RAMP_STEPS, record=True, sweep_idx=log["sweep_idx"])
    else:
        send_actuator_test(conn, MOTOR_A_FUNC, us_to_norm(a), timeout_s)
        send_actuator_test(conn, MOTOR_B_FUNC, us_to_norm(b), timeout_s)
        time.sleep(RAMP_S / RAMP_STEPS)


def ramp_down(conn, a_us, b_us, timeout_s, log=None):
    """[정상 종료] 현재 (a_us,b_us)에서 최소값(1000µs)까지 여러 단계로 '서서히' 내린다.

    측정을 정상적으로 다 마쳤을 때 딱 한 번 사용 — 급격한 전류 컷·기계적 충격 완화용.
    (긴급 상황에는 쓰지 않는다. 긴급은 stop_motors_immediate 로 즉시 끈다)

    log 을 주면 이 구간을 phase="ramp_down" 으로 기록한다. 중단(abort) 경로에서는
    writer 가 이미 닫혔을 수 있으므로 log=None 으로 호출해 기록을 건너뛴다.
    """
    for i in range(1, RAMP_STEPS + 1):
        frac = 1.0 - i / float(RAMP_STEPS)        # 1 → 0 으로 감소
        a = PWM_MIN_US + (a_us - PWM_MIN_US) * frac
        b = PWM_MIN_US + (b_us - PWM_MIN_US) * frac
        _ramp_step(conn, a, b, timeout_s, log, "ramp_down")


def ramp_to(conn, from_a, from_b, to_a, to_b, timeout_s, log=None):
    """[정상 전환] (from_a,from_b) → (to_a,to_b) 로 여러 단계에 걸쳐 '서서히' 이동한다.

    한 세트(스윕)가 끝나고 다음 세트를 시작할 때, PWM을 한 번에 확 줄이지 않고
    부드럽게 내려서(또는 올려서) 새 세트를 시작하기 위한 용도.
    RAMP_S초 동안 RAMP_STEPS 단계로 선형 보간한다.
    log 을 주면 phase="ramp_between" 으로 기록해 스윕 사이에 공백이 남지 않게 한다.
    """
    for i in range(1, RAMP_STEPS + 1):
        frac = i / float(RAMP_STEPS)              # 0 → 1
        a = from_a + (to_a - from_a) * frac
        b = from_b + (to_b - from_b) * frac
        _ramp_step(conn, a, b, timeout_s, log, "ramp_between")


def frange_us(start, end, step):
    """start~end(포함)까지 step 간격의 µs 리스트를 만든다(정수 µs)."""
    values = []
    v = start
    while v <= end:
        values.append(v)
        v += step
    return values


def write_run_sidecar(out_path, cfg, t_start):
    """측정 설정을 CSV 옆에 <이름>.run.json 으로 남긴다.

    CSV 에는 결과(phase 열)만 들어가고 dwell_s·repeats·격자 범위 같은 '어떻게 측정했는지'는
    사라진다. 나중에 런을 재현하거나 비교하려면 이 정보가 꼭 필요하므로 따로 저장한다.
    device·baud 같은 접속 정보는 데이터 분석과 무관하므로 제외.
    """
    meta = {k: cfg.get(k) for k in (
        "a_start", "a_end", "a_step", "b_start", "b_end", "b_step",
        "repeats", "dwell_s", "warmup_s", "resend_hz",
        "idle_s", "randomize", "bracket", "seed",
        "servo_hz", "bat_hz", "esc_hz", "min_voltage_v",
        "notes", "prop", "battery")}
    meta["t_start_epoch"] = t_start
    meta["csv"] = os.path.basename(out_path)
    try:
        with open(os.path.splitext(out_path)[0] + ".run.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
    except OSError:
        pass   # 메타 저장 실패가 측정을 막으면 안 된다


def build_combos(cfg):
    """격자 범위(cfg)로 (A,B) 조합 목록을 만든다. A가 바깥, B가 안쪽(= grid 파일과 동일 순서).

    A·B 시작=끝 이면 각 축이 1개 → 조합 1개("한 조합만" 케이스).
    이 목록 순서가 곧 '계단'을 올라가는 순서다.

    randomize=True 면 순서를 섞는다. 배터리는 스윕 도중 계속 소모되므로, 순서대로
    올라가면 '전압 하락'과 'PWM 상승'이 완전히 겹쳐서 둘을 구분할 수 없다(교란).
    순서를 섞으면 전압 드리프트가 PWM과 무상관이 되어 계통 오차가 아니라 산포로 바뀐다.

    bracket=True 면 첫 조합을 맨 뒤에 한 번 더 넣는다. 처음과 마지막의 같은 조합을
    비교하면 그 스윕 동안 일어난 드리프트(배터리·발열)를 직접 측정할 수 있다.
    """
    a_vals = frange_us(cfg["a_start"], cfg["a_end"], cfg["a_step"])
    b_vals = frange_us(cfg["b_start"], cfg["b_end"], cfg["b_step"])
    combos = [(a, b) for a in a_vals for b in b_vals]

    if cfg.get("randomize"):
        seed = cfg.get("seed")
        rng = random.Random(seed if seed else None)
        rng.shuffle(combos)

    if cfg.get("bracket") and len(combos) > 1:
        combos = combos + [combos[0]]

    return combos


# ============================================================================
# 설정 검증 — GUI가 보낸 값을 기본값과 합치고, 물리적으로 말이 되는지 확인
# ============================================================================
def build_cfg(raw):
    """GUI가 보낸 JSON(raw)을 DEFAULT_CFG 위에 덮어써서 완전한 cfg 를 만든다(타입 보정 포함)."""
    cfg = dict(DEFAULT_CFG)
    for k, v in (raw or {}).items():
        if k not in cfg:
            continue  # 모르는 키는 무시(안전)
        if isinstance(DEFAULT_CFG[k], bool):
            cfg[k] = bool(v)
        elif isinstance(DEFAULT_CFG[k], int):
            cfg[k] = int(v)
        elif isinstance(DEFAULT_CFG[k], float):
            cfg[k] = float(v)
        else:
            cfg[k] = v
    return cfg


def validate_cfg(cfg):
    """말이 안 되는 설정이면 ValueError(사람이 읽을 메시지)를 던진다. 문제 없으면 조용히 통과."""
    for name in ("a_start", "a_end", "b_start", "b_end"):
        if not (PWM_MIN_US <= cfg[name] <= PWM_MAX_US):
            raise ValueError(f"{name} 는 {PWM_MIN_US}~{PWM_MAX_US}µs 범위여야 합니다 (현재 {cfg[name]})")
    if cfg["a_step"] <= 0 or cfg["b_step"] <= 0:
        raise ValueError("스텝(step)은 0보다 커야 합니다")
    if cfg["a_end"] < cfg["a_start"] or cfg["b_end"] < cfg["b_start"]:
        raise ValueError("끝 값은 시작 값보다 크거나 같아야 합니다")
    if cfg["repeats"] < 1:
        raise ValueError("스윕 반복 횟수는 1 이상이어야 합니다")
    if cfg["dwell_s"] <= 0:
        raise ValueError("dwell 은 0보다 커야 합니다")
    if cfg["resend_hz"] <= 0:
        raise ValueError("재전송 Hz는 0보다 커야 합니다")
    # 타임아웃이 재전송 간격보다 짧으면 모터 명령이 중간에 끊긴다(값 유지 실패).
    if cfg["timeout_s"] <= 1.0 / cfg["resend_hz"]:
        raise ValueError("timeout_s 가 재전송 간격(1/resend_hz)보다 커야 값이 유지됩니다")
    if not build_combos(cfg):
        raise ValueError("격자 조합이 0개입니다. 범위/스텝을 확인하세요")


# ============================================================================
# 컨트롤러 — 측정 스레드의 상태를 안전하게(스레드 락) 담고, GUI에 상태를 넘겨준다
# ============================================================================
class Controller:
    """측정 실행 상태의 '단일 진실 공급원'. HTTP 스레드와 측정 스레드가 공유한다.

    - status  : GUI에 보여줄 현재 상태(진행바·현재값 등). 락으로 보호.
    - stop_requested : STOP 버튼이 눌리면 True → 측정 루프가 AbortMeasurement 를 던짐.
    - last_ping      : GUI가 마지막으로 /status 를 물어본 시각(watchdog 기준).
    - watchdog_enabled : 모터가 움직일 수 있는 구간에서만 True(연결 대기 중엔 오작동 방지 위해 False).
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.stop_requested = False
        self.last_ping = 0.0
        self.watchdog_enabled = False
        self.start_time = None
        # 실제로 도달한 수신 주기를 세어 둔다. 요청한 Hz 가 그대로 나온다는 보장이
        # 없기 때문(PX4 내부 발행 주기·MAV_x_RATE 상한에 걸리면 조용히 낮아진다).
        self.rx_counts = {}
        self.rx_since = time.time()
        self.status = self._blank_status()

    @staticmethod
    def _blank_status():
        return {
            "state": "idle",          # idle/connecting/running/done/stopped/error
            "message": "대기 중",
            "connected": False,
            "combo_idx": 0, "combo_total": 0,
            "sweep_idx": 0, "sweep_total": 0,
            "a_us": 0, "b_us": 0,
            "servo_raw": [0] * 8,
            "voltage_v": None, "current_a": None,
            "point_done": 0, "point_total": 0,
            "elapsed_s": 0.0, "eta_s": 0.0,
            "out_path": None,
        }

    def is_running(self):
        return self.thread is not None and self.thread.is_alive()

    def note_rx(self, kind):
        """메시지 수신 1건 기록(실측 주기 계산용). 락 없이 쓰기엔 단순 증가라 안전."""
        self.rx_counts[kind] = self.rx_counts.get(kind, 0) + 1

    def achieved_rates(self):
        """지금까지 실제로 받은 평균 주기(Hz). 요청값과 비교해 경고하는 데 쓴다."""
        dt = max(1e-6, time.time() - self.rx_since)
        return {k: round(n / dt, 1) for k, n in self.rx_counts.items()}

    def set_status(self, **kw):
        with self.lock:
            self.status.update(kw)

    def get_status(self):
        """GUI에 줄 상태 사본. 진행 중이면 경과/남은 시간을 실시간 계산해 붙인다."""
        with self.lock:
            s = dict(self.status)
        if self.start_time and s["state"] in ("connecting", "running"):
            s["elapsed_s"] = round(time.time() - self.start_time, 1)
            if s["point_total"] > 0 and s["point_done"] > 0:
                avg = s["elapsed_s"] / s["point_done"]
                s["eta_s"] = round(avg * (s["point_total"] - s["point_done"]), 1)
        return s

    def start(self, cfg):
        """측정 스레드를 시작한다(이미 실행 중이면 호출하지 않는다 — 서버가 먼저 막는다)."""
        with self.lock:
            self.status = self._blank_status()
        self.stop_requested = False
        self.last_ping = time.time()      # 시작 순간 살아있음으로 간주(첫 폴링 전까지 여유)
        self.start_time = time.time()
        self.thread = threading.Thread(target=run_measurement, args=(self, cfg), daemon=True)
        self.thread.start()

    def request_stop(self):
        """STOP 버튼/종료에서 호출. 측정 루프가 다음 점검 때 AbortMeasurement 를 던진다."""
        self.stop_requested = True

    def check_abort(self):
        """측정 루프 안에서 자주 호출. 중단 조건이면 AbortMeasurement 를 던진다."""
        if self.stop_requested:
            raise AbortMeasurement("stop-button")
        if self.watchdog_enabled and (time.time() - self.last_ping) > WATCHDOG_TIMEOUT_S:
            # GUI가 일정 시간 응답을 안 물어봄 = 브라우저 닫힘/새로고침/네트워크 끊김 → 즉시 정지
            raise AbortMeasurement("watchdog(브라우저 끊김)")


# ============================================================================
# 측정 핵심 — 값을 유지하며 기록. grid 파일 hold_and_log 를 복사 + 안전 점검/상태갱신 추가
# ============================================================================
def hold_and_log(ctl, conn, writer, latest, cfg, a_us, b_us, phase, duration, record, sweep_idx):
    """(A,B)를 각 목표 µs로 duration초 '유지'하며, record=True면 수신값을 CSV로 기록한다.

    - 명령은 타임아웃이 있으므로 duration 동안 '계속 재전송'해야 값이 유지된다.
    - 매 반복마다 ctl.check_abort() 로 STOP/watchdog 을 점검(즉시 중단 가능).
    - 최신 전압/전류/실측 PWM 은 GUI 표시용으로 ctl.status 에도 갱신한다.
    """
    a_norm = us_to_norm(a_us)
    b_norm = us_to_norm(b_us)
    deadline = time.time() + duration
    next_send = 0.0   # 다음 재전송 예정 시각(0이면 즉시)

    while time.time() < deadline:
        ctl.check_abort()             # (1) STOP/watchdog 점검 — 걸리면 AbortMeasurement
        now = time.time()

        # (2) 재전송 주기가 되면 두 모터 명령을 다시 보낸다(값 유지)
        if now >= next_send:
            send_actuator_test(conn, MOTOR_A_FUNC, a_norm, cfg["timeout_s"])
            send_actuator_test(conn, MOTOR_B_FUNC, b_norm, cfg["timeout_s"])
            next_send = now + 1.0 / cfg["resend_hz"]

        # (3) 들어오는 메시지를 '버퍼가 빌 때까지' 처리한다.
        # 한 번에 한 개만 꺼내면 요청 주기를 올렸을 때 수신이 생성 속도를 못 따라가
        # 시리얼 버퍼에 밀리고, t_epoch 이 실제 수신 시각보다 점점 뒤처진다.
        # 먼저 블로킹으로 하나 기다린 뒤, 남아 있는 것을 논블로킹으로 모두 비운다.
        # 한 번에 비우는 개수에 상한을 둔다. 상한이 없으면 메시지가 처리 속도보다
        # 빨리 들어올 때 이 루프에서 빠져나오지 못하고, 그 동안 deadline 과
        # check_abort()(STOP 버튼·watchdog)를 확인하지 못한다. 모터가 도는 중이므로
        # 안전상 반드시 주기적으로 바깥 루프로 돌아와야 한다.
        msg = conn.recv_match(blocking=True, timeout=0.05)
        drained = 0
        while msg is not None and drained < MAX_DRAIN_PER_LOOP:
            drained += 1
            mtype = msg.get_type()

            if mtype == "BATTERY_STATUS" and getattr(msg, "id", 0) == 0:
                # QGC와 동일한 소스(주 배터리 id=0의 BATTERY_STATUS)에서 전압/전류 계산
                v, c = read_battery(msg)
                if v is not None:
                    latest["voltage_v"] = v
                if c is not None:
                    latest["current_a"] = c
                ctl.set_status(voltage_v=latest["voltage_v"], current_a=latest["current_a"])
                ctl.note_rx("battery")
                # 저전압 컷오프 — 리포 보호. 반복 방전 시험에서 셀당 3.3V 아래로
                # 내려가면 팩이 상한다. 여기서 멈추면 '측정 실패'지만 배터리는 산다.
                floor = cfg.get("min_voltage_v") or 0
                if floor and latest["voltage_v"] and latest["voltage_v"] < floor:
                    raise AbortMeasurement(
                        f"저전압 컷오프: {latest['voltage_v']:.2f}V < {floor:.2f}V")

            elif mtype == "ESC_STATUS":
                # index 는 이 메시지가 담고 있는 첫 ESC 번호(0,4,8...). rpm 은 4개씩 온다.
                base = int(getattr(msg, "index", 0))
                for k, rpm in enumerate(getattr(msg, "rpm", [])[:4]):
                    slot = base + k
                    if 0 <= slot < 4:
                        latest["esc_rpm"][slot] = rpm
                ctl.set_status(esc_rpm=list(latest["esc_rpm"]))
                ctl.note_rx("esc")

            elif mtype == "SERVO_OUTPUT_RAW":
                servo = [getattr(msg, f"servo{i}_raw") for i in range(1, 9)]   # 실측 PWM 8채널
                ctl.set_status(servo_raw=servo)   # GUI 실시간 표시용
                ctl.note_rx("servo")
                if record:
                    # 실제 출력 PWM(µs) + 명령값(µs/정규화) + 최근 전압/전류 + 스윕번호를 한 줄로 기록
                    row = [
                        time.time(),        # t_epoch: 라즈베리파이 수신 시각(로드셀 로그와 병합 기준)
                        msg.time_usec,      # t_fc_us: 픽스호크 측 시각
                        phase,              # 어느 조합인지 식별 문자열(예: A1000_B1200)
                        a_us, b_us,         # 명령 µs
                        round(a_norm, 4), round(b_norm, 4),   # 명령 정규화값
                    ]
                    row += servo
                    row += [latest["voltage_v"], latest["current_a"], sweep_idx]
                    row += ["" if r is None else r for r in latest["esc_rpm"]]
                    writer.writerow(row)

            msg = conn.recv_match(blocking=False)   # 남은 것 비우기


# ============================================================================
# 측정 메인 흐름 — 별도 스레드에서 돈다. 예외는 모두 잡아 상태로 남기고, 끝에 반드시 정지.
# ============================================================================
def run_measurement(ctl, cfg):
    conn = None
    fp = None
    try:
        # 1) 연결 + 하트비트
        ctl.set_status(state="connecting", message=f"연결 시도: {cfg['device']} @ {cfg['baud']}")
        conn = mavutil.mavlink_connection(
            cfg["device"], baud=cfg["baud"], source_system=191, source_component=191)
        hb = conn.wait_heartbeat(timeout=30)
        if hb is None:
            raise RuntimeError("HEARTBEAT 없음 — 배선(TX/RX 교차)·전원·baud(921600)를 확인하세요")
        ctl.set_status(connected=True,
                       message=f"연결됨: system={conn.target_system}, comp={conn.target_component}")

        # 2) 메시지 주기 상향 + 시동 여부 확인(시동 상태면 거부)
        request_rates(conn, cfg.get("servo_hz", 50),
                      cfg.get("bat_hz", 20), cfg.get("esc_hz", 20))
        armed = check_armed(conn)
        if armed is True:
            raise RuntimeError("지금 '시동(armed)' 상태입니다. 시동 해제 후 실행하세요")
        # armed is None 이면 판단 불가 — 경고만 남기고 진행(모터는 안 돌린 상태)
        ok, ack_msg = preflight_ack(conn, cfg["timeout_s"])
        ctl.set_status(message=f"프리플라이트: {ack_msg}")
        if not ok:
            raise RuntimeError(ack_msg)

        # 3) 격자/조합 준비 + CSV 열기
        combos = build_combos(cfg)
        point_total = len(combos) * cfg["repeats"]
        t_start = int(time.time())
        out_dir = (cfg.get("out_dir") or "").strip()
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f"{OUT_PREFIX}_{t_start}.csv")
        ctl.set_status(combo_total=len(combos), sweep_total=cfg["repeats"],
                       point_total=point_total, out_path=out_path)
        latest = {"voltage_v": None, "current_a": None,
                  "esc_rpm": [None, None, None, None]}

        # 설정값(dwell/repeats/격자 범위 등)은 CSV 안에 남지 않는다 → 옆에 사이드카로 저장.
        # 나중에 tvctools 가 이 파일을 읽어 런 정보를 복원한다.
        write_run_sidecar(out_path, cfg, t_start)

        fp = open(out_path, "w", newline="")
        writer = csv.writer(fp)
        writer.writerow(CSV_HEADER)

        # 여기서부터 모터가 움직일 수 있음 → watchdog 켬(브라우저 끊기면 즉시 정지)
        ctl.watchdog_enabled = True
        ctl.last_ping = time.time()

        # 4-1) ESC 워밍업(최소값 유지, 기록 안 함) — 계단 시작 전 딱 한 번
        ctl.set_status(state="running", message=f"ESC 워밍업 {cfg['warmup_s']}s (최소값 유지)")
        hold_and_log(ctl, conn, writer, latest, cfg,
                     PWM_MIN_US, PWM_MIN_US, "warmup", cfg["warmup_s"], record=False, sweep_idx=0)

        # 4-1b) 무부하 구간을 '기록'한다(phase="idle_pre").
        # 양 채널 1000µs·전류≈0 이므로 여기 전압이 곧 배터리 실제 잔량(SoC)이다.
        # 예전에는 이 구간이 기록되지 않아 SoC를 ulog 에서만 얻을 수 있었다.
        if cfg.get("idle_s", 0) > 0:
            ctl.set_status(message=f"무부하 기준 전압 측정 {cfg['idle_s']}s")
            hold_and_log(ctl, conn, writer, latest, cfg,
                         PWM_MIN_US, PWM_MIN_US, "idle_pre", cfg["idle_s"],
                         record=True, sweep_idx=0)

        # 주기 측정은 여기서부터(연결·워밍업 구간을 빼야 실제 스윕 주기가 나온다)
        ctl.rx_counts = {}
        ctl.rx_since = time.time()

        # 4-2) 계단식 스윕: 격자 전체를 repeats 번 반복. 스텝 사이에 정지/안정화/간격 없음.
        point_done = 0
        last_a, last_b = PWM_MIN_US, PWM_MIN_US
        for r in range(1, cfg["repeats"] + 1):
            for ci, (a_us, b_us) in enumerate(combos, start=1):
                phase = f"A{a_us}_B{b_us}"
                ctl.set_status(state="running",
                               message=f"스윕 {r}/{cfg['repeats']} · 조합 {ci}/{len(combos)} "
                                       f"(A={a_us}µs, B={b_us}µs)",
                               sweep_idx=r, combo_idx=ci, a_us=a_us, b_us=b_us)
                # 바로 dwell 동안 유지+기록 → 끝나면 즉시 다음 조합으로(계단 한 칸)
                hold_and_log(ctl, conn, writer, latest, cfg,
                             a_us, b_us, phase, cfg["dwell_s"], record=True, sweep_idx=r)
                point_done += 1
                last_a, last_b = a_us, b_us
                ctl.set_status(point_done=point_done)
            fp.flush()   # 스윕 한 바퀴 끝날 때마다 디스크에 안전 저장

            # 다음 스윕(세트)이 남았으면: 마지막 조합에서 다음 스윕 첫 조합으로 '서서히' 이동.
            # (한 번에 PWM을 확 줄이지 않도록 — 세트 사이 부드러운 전환으로 새 세트를 시작)
            if r < cfg["repeats"]:
                first_a, first_b = combos[0]
                ctl.set_status(message=f"스윕 {r} 종료 → 다음 스윕 준비(서서히 감속)")
                ctl.check_abort()
                ramp_to(conn, last_a, last_b, first_a, first_b, cfg["timeout_s"],
                        log={"ctl": ctl, "writer": writer, "latest": latest,
                             "cfg": cfg, "sweep_idx": r})
                last_a, last_b = first_a, first_b

        # 4-3) 정상 종료: 마지막 값에서 서서히 정지.
        # 이 구간을 기록해야 추력이 크게 떨어지는 '가장 뚜렷한 에지'가 CSV에 남는다.
        ramp_down(conn, last_a, last_b, cfg["timeout_s"],
                  log={"ctl": ctl, "writer": writer, "latest": latest,
                       "cfg": cfg, "sweep_idx": cfg["repeats"]})

        # 4-4) 스윕 후 무부하 전압(phase="idle_post").
        # idle_pre 와의 차이가 이 런에서 실제로 소모된 배터리 양이다.
        # 주의: 팩은 부하 직후 수십 초에 걸쳐 회복하므로, 바로 뒤 값은 완전히
        # 쉰 OCV 보다 조금 낮게 나온다.
        if cfg.get("idle_s", 0) > 0:
            ctl.set_status(message=f"무부하 종료 전압 측정 {cfg['idle_s']}s")
            hold_and_log(ctl, conn, writer, latest, cfg,
                         PWM_MIN_US, PWM_MIN_US, "idle_post", cfg["idle_s"],
                         record=True, sweep_idx=0)

        # 실제로 받은 주기를 보고한다. 요청한 Hz 가 그대로 나오는 경우는 오히려 드물다
        # (PX4 내부 토픽 발행 주기, MAV_x_RATE 대역 상한에 걸리면 조용히 낮아진다).
        rates = ctl.achieved_rates()
        want = (cfg.get("servo_hz", 50), cfg.get("bat_hz", 20), cfg.get("esc_hz", 20))
        got = (rates.get("servo", 0), rates.get("battery", 0), rates.get("esc", 0))
        rate_msg = ("실측 주기 servo %.0f/%d Hz, battery %.0f/%d Hz, esc %.0f/%d Hz"
                    % (got[0], want[0], got[1], want[1], got[2], want[2]))
        if got[2] == 0:
            rate_msg += " — ESC 텔레메트리 없음(RPM 미기록)"
        if got[0] < 0.7 * want[0]:
            rate_msg += " — servo 주기가 요청보다 낮음: PX4 발행 주기/MAV_x_RATE 확인"
        ctl.set_status(achieved_rates=rates)
        ctl.set_status(state="done", message=f"완료 — 저장: {out_path} · {rate_msg}")

    except AbortMeasurement as ab:
        # STOP 버튼 또는 watchdog: 사용자/안전 중단
        ctl.set_status(state="stopped", message=f"중단됨: {ab.reason}")
    except Exception as e:
        # 그 외 오류(연결 실패, 시동 상태 등)
        ctl.set_status(state="error", message=f"오류: {e}")
    finally:
        # 어떤 경우든 마지막엔 반드시 모터 정지 + 파일 저장
        ctl.watchdog_enabled = False
        if conn is not None:
            try:
                stop_motors_immediate(conn, cfg["timeout_s"])
            except Exception:
                pass
        if fp is not None:
            try:
                fp.flush()
                fp.close()
            except Exception:
                pass


# ============================================================================
# HTTP 서버 — GUI(html) 배달 + 시작/정지/상태 API
# ============================================================================
HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pwm_map_gui.html")


class Handler(BaseHTTPRequestHandler):
    """작은 웹서버 핸들러.

    GET  /            → pwm_map_gui.html 배달
    GET  /status      → 현재 진행상황 JSON (호출될 때마다 watchdog 'ping' 갱신)
    POST /start       → 설정(JSON)으로 측정 시작
    POST /stop        → 즉시 정지 요청
    """
    controller = None   # main() 에서 클래스 속성으로 주입

    # --- 응답 헬퍼 ---
    def _send_json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self):
        try:
            with open(HTML_PATH, "rb") as f:
                body = f.read()
        except FileNotFoundError:
            self._send_json(500, {"error": f"GUI 파일을 못 찾음: {HTML_PATH}"})
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return None   # 파싱 실패 표시

    # --- 라우팅 ---
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send_html()
        elif path == "/status":
            # status 를 물어보는 것 자체가 GUI가 '살아있다'는 신호 → watchdog ping 갱신
            self.controller.last_ping = time.time()
            self._send_json(200, self.controller.get_status())
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/start":
            raw = self._read_json_body()
            if raw is None:
                self._send_json(400, {"error": "JSON 파싱 실패"})
                return
            if self.controller.is_running():
                self._send_json(409, {"error": "이미 측정이 실행 중입니다"})
                return
            cfg = build_cfg(raw)
            try:
                validate_cfg(cfg)
            except ValueError as e:
                self._send_json(400, {"error": str(e)})
                return
            self.controller.start(cfg)
            self._send_json(200, {"ok": True, "message": "측정 시작"})
        elif path == "/stop":
            self.controller.request_stop()
            self._send_json(200, {"ok": True, "message": "정지 요청됨"})
        else:
            self._send_json(404, {"error": "not found"})

    def log_message(self, *args):
        # 서버 접근 로그를 터미널에 도배하지 않도록 조용히(디버그가 필요하면 print 로 바꾸면 됨)
        pass


def main():
    ap = argparse.ArgumentParser(description="BLDC 2D PWM 계단식 스윕/로거 — HTML GUI 서버")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="바인드 주소 (기본 %(default)s = 라즈베리파이 자기 자신만). "
                         "다른 노트북에서 접속하려면 0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="포트 (기본 %(default)s)")
    args = ap.parse_args()

    controller = Controller()
    Handler.controller = controller

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    shown_host = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    print("=" * 64)
    print(f"  로컬 서버 시작됨 → 브라우저에서  http://{shown_host}:{args.port}  여세요")
    print("  종료: 이 터미널에서 Ctrl+C  (측정 중이면 모터 정지+CSV 저장 후 종료)")
    print("=" * 64)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nCtrl+C — 종료 중...")
    finally:
        # 종료 시 측정 중이면 정지시키고 스레드가 마무리(정지+저장)할 시간을 준다
        controller.request_stop()
        if controller.is_running():
            controller.thread.join(timeout=5.0)
        server.shutdown()
        print("서버 종료 완료.")


if __name__ == "__main__":
    main()
