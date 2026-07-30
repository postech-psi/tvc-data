#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLDC 추력 맵핑용 2D PWM 격자 '계단식' 스윕 + 로드셀 통합 로깅 — HTML GUI 버전
(측정 로직 + 로컬 웹서버 + STM32 로드셀 리더 한 파일)

[이 파일이 하는 일]
  1) 파이썬 표준 라이브러리 + pymavlink + pyserial만으로 '작은 로컬 웹서버'를 띄운다.
  2) 브라우저에서 http://localhost:8000 을 열면 옆에 있는 pwm_map_gui.html 을 배달한다.
  3) GUI에서 [측정 시작]을 누르면, 그 설정대로 픽스호크에 PWM(액추에이터 테스트)을 보내며
     실제 PWM(µs)·전압·전류·RPM 을 기록하고, 동시에 STM32 로드셀(Fx..Tz, USB 시리얼)을
     같은 프로세스·같은 시계(time.time())로 함께 기록한다 — 별도 노트북/별도 시계로 인한
     사후 시간정렬이 필요 없다.
  4) 측정 진행상황을 GUI가 0.5초마다 물어보면(GET /status) 알려준다(진행바·현재값·남은시간).

[로드셀이 왜 여기 있나]
  예전에는 로드셀이 벤치 노트북(gui.py, superseded)에 물려 있어 커맨드(Pi)와 힘(노트북)이
  서로 다른 시계를 썼다. 이 파일은 로드셀을 Pi에 직접 연결해 커맨드를 보내는 바로 그
  프로세스가 힘도 같이 찍는다 — 시간정렬이 필요 없어진다. tvcbench(별도 CLI/plan 기반
  재작성)가 이미 이 통합을 하지만, 이 파일은 그 로직을 (기존 관행대로) import 없이
  독립적으로 복사해 두어, 웹 GUI 하나로 붙여 쓰는 단순한 운영 방식을 유지한다.

  ※ 실측으로 확인된 중요한 사실: STM32 로드셀은 'ARM <값>'을 50Hz로 계속 보내주지 않으면
    자체적으로 ~10Hz로만 상태줄을 내보낸다(계속 보내면 50Hz). tvcbench의 로드셀 코드는
    이 하트비트를 전혀 보내지 않아 실제로는 항상 ~10Hz로만 기록되고 있었다 — 이 파일은
    그 하트비트를 구현해 실제 50Hz 로 기록되도록 고쳤다. 보내는 값은 항상 1000(idle)
    고정이며, 모터를 구동하지 않는다(모터 구동은 Pixhawk가 별도로 함) — 로드셀 자신의
    샘플링 주기를 유지하기 위한 용도일 뿐이다.

[측정 구조 — 팀과 합의한 계획 (계단식, 단순 유지)]
  - 격자 범위 입력: A(고정축) 시작/끝/스텝, B(스윕축) 시작/끝/스텝 → (A,B) 조합 목록 자동 생성.
      · A·B의 시작=끝 으로 넣으면 조합이 1개 → "배터리 아껴 한 조합만" 케이스도 이걸로 커버.
  - '계단식': 각 조합을 dwell초 동안 유지·기록한 뒤, 멈추거나 쉬지 않고 곧바로 다음 조합 PWM으로
    올라간다(정지/안정화/간격 단계 없음). dwell 은 고정값만 지원한다(적응형 모드 없음 — 단순 유지).
  - 스윕 반복(N): 격자 전체를 한 바퀴 도는 것을 1스윕이라 하면, 이를 N번 반복한다(반복 사이도 안 쉼).
    randomize=True 면 반복마다 새로 셔플한다(blocked_random — 배터리 드리프트와 PWM의 상관을 끊음).
  - 안전을 위해 '맨 마지막 종료 시에만' 서서히 정지(램프다운)한다.
  - 안전 한계는 기존과 동일하게 min_voltage_v 컷오프 하나만 사용한다(로드셀이 생겨도
    추력/토크 기반 추가 중단 조건은 넣지 않음 — 단순함 우선).

[CSV 출력 — 스트림별 별도 파일, 가짜 rate 없음]
  예전 버전은 SERVO_OUTPUT_RAW 메시지가 올 때마다 한 줄을 쓰고 그 순간의 최신 전압/전류를
  끼워 넣었다 — 10Hz 짜리 값을 20Hz 로 '위조'하는 셈이었다(tvcbench 문서가 지적한 결함).
  이 버전은 각 스트림이 자기 고유 주기로 자기 파일에만 쓴다:
    A<a범위>_B<b범위>_<날짜>_<시각>/   예: A1000_B1000-1200_2026-07-31_143022/
      servo.csv      각 SERVO_OUTPUT_RAW 수신마다   (실측 PWM, ~18-20Hz)
      battery.csv    각 BATTERY_STATUS 수신마다      (전압/전류, ~10-20Hz)
      esc.csv        각 ESC_STATUS 수신마다          (RPM, 지원 ESC 없으면 파일은 헤더만)
      loadcell.csv   각 로드셀 상태줄 수신마다        (Fx..Tz, ARM 하트비트로 ~50Hz)
      run.json       설정값 + tare 오프셋 + 스트림별 실측 Hz + 격자/스텝 요약
  네 파일 모두 phase(예: "A1400_B1700")·sweep_idx·a_cmd_us/b_cmd_us 를 매 줄에 직접 싣는다
  (별도 시퀀스 테이블과 join 하지 않아도 각 CSV 만으로 바로 분석 가능하게).

[동작 원리 — PX4 공식 경로]
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

[no_motor — 드라이런]
  cfg["no_motor"]=True 면 타이밍/재전송/CSV 기록/로드셀/ARM 하트비트는 전부 평소와 동일하게
  돌아가되, 실제 Pixhawk 로 나가는 send_actuator_test 호출만 생략한다. 모터를 돌리지 않고
  이 파일의 배관(로깅 파이프라인)을 검증할 때 쓴다.

[실행]
    python3 pwm_thrust_map.py          # 서버 시작 → 브라우저에서 localhost:8000
    (개발/검증) 브라우저 GUI의 device 칸에 udpin:0.0.0.0:14550 을 넣으면 SITL/QGC로 로직만 확인
    종료: 이 터미널에서 Ctrl+C (측정 중이면 모터 정지+CSV 저장 후 종료)

의존성: pymavlink + pyserial + 파이썬 표준 라이브러리(http.server, threading, json, csv ...) 뿐.
"""

import argparse
import csv
import json
import os
import random
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pymavlink import mavutil   # 픽스호크용 외부 라이브러리
import serial                   # 로드셀(STM32, USB CDC)용 외부 라이브러리


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


# --- 로드셀(STM32) 관련 상수 ---
# 실측: 이 값을 50Hz로 계속 보내지 않으면 로드셀은 자체적으로 ~10Hz 로만 상태줄을 낸다.
LOADCELL_ARM_HZ = 50.0
LOADCELL_ARM_VALUE = 1000          # 항상 idle 고정 — 모터를 구동하지 않는다(하트비트 전용)
LOADCELL_READ_TIMEOUT_S = 0.05
LOADCELL_MAX_LINE_BYTES = 4096     # 이보다 긴 '줄'은 프로토콜이 아니라 끼인 프레이머 — 버림
FT_CHANNELS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
MN_TO_N = 1e-3                      # 로드셀은 mN/mN*m 단위로 보고 → SI(N/N*m)로 변환

# GUI에서 값이 일부만 오거나 이상해도 안전하게 돌도록 하는 '기본 설정'.
# (GUI가 보낸 값으로 덮어쓴 뒤, validate_cfg 로 최종 검증한다)
DEFAULT_CFG = {
    "device": "/dev/ttyAMA0",   # Pi5 GPIO UART(핀 8·10, 픽스호크). 개발용은 "udpin:0.0.0.0:14550"
    "baud": 921600,             # 픽스호크 TELEM2 권장 속도
    "a_start": 1000, "a_end": 1000, "a_step": 100,   # A(고정축) 격자 범위 µs
    "b_start": 1000, "b_end": 2000, "b_step": 100,   # B(스윕축) 격자 범위 µs
    "repeats": 1,               # 격자 전체 스윕을 몇 번 반복할지 (1이면 한 바퀴)
    "dwell_s": 3.0,             # 각 조합에서 '유지+기록' 시간 (계단 한 칸의 길이, 고정값만 지원)
    "resend_hz": 5.0,           # ACTUATOR_TEST 재전송 주기(타임아웃으로 값이 풀리지 않게)
    "timeout_s": 1.0,           # 각 명령의 타임아웃(초). 재전송 간격(1/resend_hz)보다 커야 함
    "warmup_s": 3.0,            # 시작 시 ESC arming 을 위해 최소값(0)으로 잠깐 대기
    # --- 측정 설계(교란 제거용) ---
    "idle_pre_s": 5.0,          # 스윕 '전' 무부하 구간(양 채널 1000µs, 전류≈0) — 기록 시간(초).
    "idle_post_s": 30.0,        # 스윕 '후' 무부하 구간 — 기록 시간(초). 전압 회복(relaxation)이
                                # 부하 직후 30~60초에 걸쳐 일어나므로 pre 보다 길게 두는 게 보통 낫다.
                                # 여기서 읽은 전압이 배터리의 '실제 잔량'(SoC)에 가장 가깝다.
    "servo_hz": 50,             # SERVO_OUTPUT_RAW 요청 주기. servo.csv 한 줄이 이 메시지마다 나온다.
    "bat_hz": 50,               # BATTERY_STATUS 요청 주기. 예전엔 20으로 뒀지만 이 기체에서
                                # 실측으로 50Hz 가 그대로 나오는 걸 확인해서 상향(PX4 내부 발행
                                # 상한에 걸리면 achieved_rates 가 낮게 찍혀서 바로 드러난다).
    "esc_hz": 50,               # ESC_STATUS(RPM) 요청 주기. 미지원 ESC면 그냥 안 온다(비용 없음).
    "min_voltage_v": 0.0,       # 이 전압 아래로 내려가면 즉시 중단(리포 보호). 3S면 9.9 권장.
    "randomize": False,        # True 면 '반복마다 새로 셔플'(blocked_random) — 배터리 드리프트와
                                # PWM의 상관을 끊는다. 예전처럼 한 번만 섞어 반복마다 재사용하지 않는다.
    "bracket": False,           # 첫 조합을 맨 뒤에 반복 → 스윕 중 드리프트를 직접 측정
    "seed": 0,                  # randomize 재현용 시드(0이면 매번 새로 고름)
    # --- 로드셀(STM32) ---
    "loadcell_device": "/dev/ttyACM0",   # 비워두면(""), 로드셀 없이 PWM/전압/전류만 기록
    "loadcell_baud": 115200,
    "tare_s": 5.0,              # 측정 시작 전 이 시간(초) 동안의 평균을 0점으로 뺀다(비파괴적 tare)
    # --- 스윕 설계 옵션(선택, 기본은 꺼짐 → 안 쓰면 예전과 동일) ---
    "ref_a": 0, "ref_b": 0,     # 주기적으로 되돌아갈 기준점(µs). ref_every_n_steps=0 이면 무시.
    "ref_every_n_steps": 0,     # N 스텝마다 기준점을 한 번 방문 → 드리프트를 covariate 로 보정 가능
    # --- 검증/드라이런 ---
    "no_motor": False,          # True 면 실제 액추에이터 명령만 생략(그 외 로직은 전부 동일)
    # --- 아래는 데이터 정리용(측정 자체에는 영향 없음) ---
    "out_dir": "raw/pwm",       # CSV 저장 폴더. 기본이 repo 루트에 바로 쌓이지 않도록
                                # raw/ 밑에 둔다(다른 데이터 전부 이 관례를 따름 — README 참고).
                                # 나중에 tvctools organize 가 raw/<날짜>/pwm/ 로 다시 정리한다.
    "notes": "",                # 이 런에 대한 자유 메모
    "prop": "",                 # 프로펠러 사양
    "battery": "",              # 배터리 사양(셀 수/용량 등)
}

# CSV 컬럼 — 스트림마다 별도 파일. 모든 파일에 공통 접두 컬럼을 그대로 싣는다
# (별도 시퀀스 테이블과 join 하지 않아도 각 CSV 파일 하나만으로 바로 분석 가능하게).
COMMON_PREFIX = ["t_epoch", "phase", "sweep_idx", "a_cmd_us", "b_cmd_us", "a_cmd_norm", "b_cmd_norm"]

SERVO_COLUMNS = COMMON_PREFIX + ["t_fc_us"] + [f"servo{i}_raw" for i in range(1, 9)]
BATTERY_COLUMNS = COMMON_PREFIX + ["voltage_v", "current_a"]
ESC_COLUMNS = COMMON_PREFIX + [f"esc{i}_rpm" for i in range(1, 5)]
LOADCELL_COLUMNS = (
    COMMON_PREFIX + ["t_stm_ms"]
    + [f"{ch}_raw" for ch in FT_CHANNELS] + list(FT_CHANNELS)
    + ["force_count", "torque_count"]
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


def send_actuator_test(conn, func, value, timeout_s, enabled=True):
    """출력 기능(func)을 정규화값 value 로 timeout_s초 동안 구동하라고 1회 명령한다.

    enabled=False(no_motor 드라이런) 면 아무것도 보내지 않고 조용히 리턴한다 — 호출부의
    타이밍/재전송 로직은 그대로 두고 실제 하드웨어 액추에이션만 끄기 위한 단일 관문.

    command_long 파라미터 매핑:
        param1=value(0~1), param2=timeout_s(이 시간 뒤 기본값 복귀),
        param3,4=예약(0), param5=func(Motor1=1, Motor2=2 ...), param6,7=미사용(0)
    """
    if not enabled:
        return
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

    voltages[0..9]를 순서대로 더하되 UINT16_MAX(65535)를 '처음' 만나면 멈추고, 이어서
    voltages_ext[0..3]를 0을 '처음' 만나면 멈추고 더한다(PX4는 유효 셀을 index 0부터
    연속으로 채우므로 '처음 무효값에서 멈춤'이 정확한 재구성). current_battery: -1이면
    미측정(None), 아니면 cA(10mA단위)→A.
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


def preflight_ack(conn, timeout_s, enabled=True):
    """정지값(v=0)을 모터 A에 한 번 보내고 COMMAND_ACK로 픽스호크가 수락하는지 확인한다.

    enabled=False(no_motor) 면 실제로는 아무것도 보내지 않고 그렇다고 명시한 메시지만 반환한다.
    반환: (ok: bool, 사람이 읽을 메시지 문자열). 여기서 실패해도 치명은 아님(진행 가능).
    """
    if not enabled:
        return True, "no_motor: 액추에이터 프리플라이트 생략(실제 명령 전송 안 함)"
    send_actuator_test(conn, MOTOR_A_FUNC, STOP_VALUE, timeout_s)
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=2.0)
    if ack is None or ack.command != ACTUATOR_TEST:
        return True, "ACTUATOR_TEST 에 대한 COMMAND_ACK 미수신(그래도 진행 가능)"
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        return True, "프리플라이트 OK: ACTUATOR_TEST 수락됨(ACCEPTED)"
    if ack.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
        return False, "거부(TEMPORARILY_REJECTED): 보통 '시동(armed)' 상태입니다. 시동 해제 후 재시도"
    return False, f"거부됨(result={ack.result}). QGC에서 Motor1/2 배정을 확인하세요"


def stop_motors_immediate(conn, timeout_s, enabled=True):
    """[긴급/최종] 두 모터를 정지값(v=0 → 1000µs)으로 즉시 몇 번 보내 끈다(램프 없음).

    STOP 버튼·watchdog·오류·종료 시 사용. 재전송을 멈추면 타임아웃으로도 정지된다.
    """
    for _ in range(5):
        send_actuator_test(conn, MOTOR_A_FUNC, STOP_VALUE, timeout_s, enabled=enabled)
        send_actuator_test(conn, MOTOR_B_FUNC, STOP_VALUE, timeout_s, enabled=enabled)
        time.sleep(0.05)


def _ramp_step(conn, a, b, timeout_s, log=None, phase="ramp", enabled=True):
    """램프 한 칸: 명령을 보내고, 로깅 컨텍스트가 있으면 그 구간도 '기록'한다.

    램프 구간을 기록해야 하는 이유 — 여기가 추력이 가장 크게 변하는 지점이다.
    """
    if log is not None:
        # hold_and_log 이 명령 전송 + 수신 기록을 함께 처리한다
        hold_and_log(log["ctl"], conn, log["writers"], log["latest"], log["cfg"],
                     int(round(a)), int(round(b)), phase,
                     RAMP_S / RAMP_STEPS, record=True, sweep_idx=log["sweep_idx"],
                     loadcell=log.get("loadcell"), enabled=enabled)
    else:
        send_actuator_test(conn, MOTOR_A_FUNC, us_to_norm(a), timeout_s, enabled=enabled)
        send_actuator_test(conn, MOTOR_B_FUNC, us_to_norm(b), timeout_s, enabled=enabled)
        time.sleep(RAMP_S / RAMP_STEPS)


def ramp_down(conn, a_us, b_us, timeout_s, log=None, enabled=True):
    """[정상 종료] 현재 (a_us,b_us)에서 최소값(1000µs)까지 여러 단계로 '서서히' 내린다."""
    for i in range(1, RAMP_STEPS + 1):
        frac = 1.0 - i / float(RAMP_STEPS)        # 1 → 0 으로 감소
        a = PWM_MIN_US + (a_us - PWM_MIN_US) * frac
        b = PWM_MIN_US + (b_us - PWM_MIN_US) * frac
        _ramp_step(conn, a, b, timeout_s, log, "ramp_down", enabled=enabled)


def ramp_to(conn, from_a, from_b, to_a, to_b, timeout_s, log=None, enabled=True):
    """[정상 전환] (from_a,from_b) → (to_a,to_b) 로 여러 단계에 걸쳐 '서서히' 이동한다."""
    for i in range(1, RAMP_STEPS + 1):
        frac = i / float(RAMP_STEPS)              # 0 → 1
        a = from_a + (to_a - from_a) * frac
        b = from_b + (to_b - from_b) * frac
        _ramp_step(conn, a, b, timeout_s, log, "ramp_between", enabled=enabled)


def frange_us(start, end, step):
    """start~end(포함)까지 step 간격의 µs 리스트를 만든다(정수 µs)."""
    values = []
    v = start
    while v <= end:
        values.append(v)
        v += step
    return values


def describe_grid(cfg):
    """격자 범위를 'A1400_B1000-2000' 처럼 사람이 읽는 문자열로 — runs/ 폴더 관례와 동일.

    시작=끝이면 값 하나만(A1400), 아니면 범위(B1000-2000)로 표기한다. 출력 폴더 이름에
    바로 써서, 폴더 목록만 보고도 무슨 스윕이었는지(어떤 축을 고정했고 뭘 스윕했는지)
    파일을 열어보지 않고 바로 식별 가능하게 한다.
    """
    def axis(start, end):
        return f"{start}" if start == end else f"{start}-{end}"
    return f"A{axis(cfg['a_start'], cfg['a_end'])}_B{axis(cfg['b_start'], cfg['b_end'])}"


def write_run_json(out_dir, updates):
    """run.json 을 읽고(있으면) 병합해서 다시 쓴다.

    측정 시작 직전에 cfg 로 한 번 써 두면(크래시 나도 설정은 남는다), 끝날 때 실측 Hz·tare
    오프셋·outcome 을 같은 파일에 덮어써 완성한다.
    """
    path = os.path.join(out_dir, "run.json")
    data = {}
    if os.path.exists(path):
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            data = {}
    data.update(updates)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except OSError:
        pass   # 메타 저장 실패가 측정을 막으면 안 된다


def build_orders(cfg):
    """격자 범위(cfg)로 '반복(repeat)마다 하나씩'인 (A,B) 방문 순서 리스트들을 만든다.

    반환값은 길이 repeats 인 리스트이며, 각 원소가 그 반복에서 방문할 (a,b) 튜플 리스트다.
    A가 바깥, B가 안쪽(격자 생성 순서는 grid 파일과 동일).

    randomize=True 면 '반복마다 새로 셔플'한다(blocked_random) — 예전처럼 한 번만 섞어
    반복마다 그대로 재사용하면, 배터리는 스윕 내내 계속 소모되므로 '전압 하락'과 'PWM
    상승'이 완전히 겹쳐 버려 구분이 불가능하다(교란). 반복마다 독립적으로 다시 섞으면
    각 PWM 레벨이 매 반복 배터리 잔량 구간에 고르게 걸치게 되어 이 교란이 사라진다.
    시드는 f"{seed}:{repeat}" 로 구성해 하나의 시드값만 기록해도 재현 가능하다.

    bracket=True 면 각 반복의 맨 뒤에 그 반복의 첫 조합을 한 번 더 넣는다 — 처음과 마지막의
    같은 조합을 비교하면 그 스윕 동안 일어난 드리프트(배터리·발열)를 직접 측정할 수 있다.

    ref_every_n_steps>0 이면 N 스텝마다 고정 기준점(ref_a,ref_b)을 한 번 방문한다(반복의
    마지막 스텝 뒤에는 넣지 않음 — 어차피 곧 다음 반복이나 ramp_down 으로 이어지므로).
    기본값은 0(꺼짐)이라 안 쓰면 예전과 완전히 동일하게 동작한다.
    """
    a_vals = frange_us(cfg["a_start"], cfg["a_end"], cfg["a_step"])
    b_vals = frange_us(cfg["b_start"], cfg["b_end"], cfg["b_step"])
    points = [(a, b) for a in a_vals for b in b_vals]
    repeats = cfg["repeats"]
    seed = cfg.get("seed") or 0

    if cfg.get("randomize"):
        orders = []
        for r in range(repeats):
            rng = random.Random(f"{seed}:{r}")
            shuffled = list(points)
            rng.shuffle(shuffled)
            orders.append(shuffled)
    else:
        orders = [list(points) for _ in range(repeats)]

    if cfg.get("bracket") and len(points) > 1:
        for order in orders:
            order.append(order[0])

    ref_every = int(cfg.get("ref_every_n_steps") or 0)
    if ref_every > 0:
        ref_point = (int(cfg["ref_a"]), int(cfg["ref_b"]))
        spliced_orders = []
        for order in orders:
            spliced = []
            for i, pt in enumerate(order, start=1):
                spliced.append(pt)
                if i % ref_every == 0 and i != len(order):
                    spliced.append(ref_point)
            spliced_orders.append(spliced)
        orders = spliced_orders

    return orders


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
    if cfg["tare_s"] < 0:
        raise ValueError("tare_s 는 0 이상이어야 합니다")
    if cfg.get("loadcell_device") and cfg["loadcell_baud"] <= 0:
        raise ValueError("loadcell_baud 는 0보다 커야 합니다")
    ref_every = int(cfg.get("ref_every_n_steps") or 0)
    if ref_every < 0:
        raise ValueError("ref_every_n_steps 는 0 이상이어야 합니다")
    if ref_every > 0:
        for name in ("ref_a", "ref_b"):
            if not (PWM_MIN_US <= cfg[name] <= PWM_MAX_US):
                raise ValueError(f"{name} 는 {PWM_MIN_US}~{PWM_MAX_US}µs 범위여야 합니다 (현재 {cfg[name]})")
    if not any(build_orders(cfg)):
        raise ValueError("격자 조합이 0개입니다. 범위/스텝을 확인하세요")


# ============================================================================
# 로드셀(STM32) — 백그라운드 스레드로 읽고, 별도 스레드로 ARM 하트비트를 계속 보낸다.
#   (tvcbench/sources/loadcell.py 의 로직을 이 파일 관행대로 import 없이 복사)
# ============================================================================
def parse_status_line(line):
    """MCU 상태줄(`key=value` 공백 구분, 예: "t=3009477 st=SAFE ... Fx=-838.0 ...")을 파싱.

    값에 괄호 주석이 붙어 있으면(`123(note)`) 그 앞부분만 취하고, '.'이 있으면 float,
    아니면 int로 파싱을 시도한다. 파싱 실패 값은 문자열로 남긴다(`st=SAFE`).
    """
    if not line or not line.startswith("t="):
        # MCU는 빈 확인응답("OK", "OK SET")이나 자유 텍스트도 보낸다. 상태줄만 샘플로 취급.
        return None
    out = {}
    for part in line.split():
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        if "(" in value:
            value = value.split("(")[0]
        try:
            out[key] = float(value) if "." in value else int(value)
        except ValueError:
            out[key] = value
    return out or None


class LineFramer:
    """임의로 쪼개져 들어오는 바이트 스트림을 완전한 텍스트 줄로 만든다.

    USB CDC는 한 줄을 두 번의 read 에 걸쳐 나눠 줄 수 있어, 이 프레이밍이 틀리면
    배치마다 샘플 하나가 깨진다.
    """
    def __init__(self, max_line=LOADCELL_MAX_LINE_BYTES):
        self._buf = bytearray()
        self._max_line = max_line
        self.overlong = 0

    def feed(self, data):
        """바이트를 추가하고, 그 결과로 완성된 줄들을 리스트로 반환."""
        self._buf += data
        lines = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                break
            raw = bytes(self._buf[:idx])
            del self._buf[:idx + 1]
            lines.append(raw.decode("utf-8", errors="replace").strip())
        if len(self._buf) > self._max_line:
            # 완전한 한 줄 분량 안에 개행이 없다 — 우리 프로토콜이 아니다. 무한정 버퍼링 대신 버림.
            self.overlong += 1
            self._buf.clear()
        return lines


class LoadCellReader:
    """STM32 로드셀 시리얼을 읽는 백그라운드 스레드 + ARM 하트비트를 보내는 스레드.

    - 읽기 스레드: 상태줄을 파싱해 (t_epoch, fields) 를 스레드 세이프 큐에 쌓는다.
    - 하트비트 스레드: 'ARM 1000\\r\\n' 을 LOADCELL_ARM_HZ 로 계속 보낸다 — 이걸 안 보내면
      로드셀이 자체적으로 ~10Hz 로만 상태줄을 낸다(실측으로 확인). 항상 idle(1000) 값만
      보내며, 어떤 모터도 구동하지 않는다(모터는 Pixhawk 가 별도로 구동).
    - tare: 비파괴적 — raw 값은 항상 그대로 유지하고, tare 된 값을 별도 필드로 함께 낸다.
    """
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self._ser = None
        self._framer = LineFramer()
        self._lock = threading.Lock()
        self._queue = deque()
        self._stop = threading.Event()
        self._read_thread = None
        self._arm_thread = None
        self.tare = {ch: 0.0 for ch in FT_CHANNELS}
        self.error = None
        self.n_received = 0
        self.n_parse_fail = 0
        self.last_state = None
        self.t_first = None
        self.t_last = None

    def start(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=LOADCELL_READ_TIMEOUT_S)
        self._stop.clear()
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        self._arm_thread = threading.Thread(target=self._arm_loop, daemon=True)
        self._arm_thread.start()

    def _arm_loop(self):
        interval = 1.0 / LOADCELL_ARM_HZ
        cmd = f"ARM {LOADCELL_ARM_VALUE}\r\n".encode()
        while not self._stop.is_set():
            try:
                self._ser.write(cmd)
            except Exception:
                pass
            time.sleep(interval)

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(4096)
                if not data:
                    continue
                for line in self._framer.feed(data):
                    self._handle_line(line)
            except Exception as e:
                self.error = f"{type(e).__name__}: {e}"
                break

    def _handle_line(self, line):
        parsed = parse_status_line(line)
        if parsed is None:
            if line and not line.startswith(("OK", "!")):
                self.n_parse_fail += 1
            return
        if "st" in parsed:
            self.last_state = parsed["st"]

        t_epoch = time.time()
        fields = {"t_stm_ms": parsed.get("t")}
        for ch in FT_CHANNELS:
            raw = parsed.get(ch)
            raw = None if raw is None else float(raw) * MN_TO_N
            fields[f"{ch}_raw"] = raw
            fields[ch] = None if raw is None else raw - self.tare[ch]
        fields["force_count"] = parsed.get("fc")
        fields["torque_count"] = parsed.get("tc")

        with self._lock:
            self._queue.append((t_epoch, fields))
            self.n_received += 1
            if self.t_first is None:
                self.t_first = t_epoch
            self.t_last = t_epoch

    def drain(self):
        """지금까지 쌓인 샘플을 모두 꺼내고 큐를 비운다."""
        with self._lock:
            items = list(self._queue)
            self._queue.clear()
        return items

    def wait_armed(self, timeout_s):
        """MCU 상태줄이 st=ARMED 로 바뀔 때까지 대기(하트비트가 몇 번 왕복해야 반영된다).

        tare 는 이게 True 를 반환한 '뒤'에 시작해야 한다 — ARM 하트비트를 보내기 시작한
        직후에는 아직 예전 ~10Hz/DISARMED 샘플이 큐에 남아 있을 수 있어, 그 상태로 tare를
        하면 저rate 구간이 0점 평균에 섞여 들어간다. 타임아웃되면 False(그래도 진행은 가능).
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.last_state == "ARMED":
                return True
            time.sleep(0.02)
        return False

    def compute_tare(self, seconds):
        """`seconds`초 동안 raw 샘플을 모아 채널별 평균을 새 tare 오프셋으로 설정한다.

        측정 시작 전, 프로펠러/로드셀이 정지해 있을 때 호출한다. 반환: (offsets|None, n).
        """
        if seconds <= 0:
            return None, 0
        end = time.time() + seconds
        totals = {ch: 0.0 for ch in FT_CHANNELS}
        n = 0
        while time.time() < end:
            for _t_epoch, fields in self.drain():
                if any(fields.get(f"{ch}_raw") is None for ch in FT_CHANNELS):
                    continue
                for ch in FT_CHANNELS:
                    totals[ch] += fields[f"{ch}_raw"]
                n += 1
            time.sleep(0.02)
        if n == 0:
            return None, 0
        offsets = {ch: totals[ch] / n for ch in FT_CHANNELS}
        self.tare = offsets
        return offsets, n

    def stats(self):
        with self._lock:
            n = self.n_received
        span = (self.t_last - self.t_first) if (self.t_first and self.t_last) else 0.0
        hz = round(n / span, 1) if span > 0 else 0.0
        return {"received": n, "parse_fail": self.n_parse_fail, "hz": hz,
                "state": self.last_state, "error": self.error}

    def stop(self):
        self._stop.set()
        try:
            if self._ser:
                self._ser.write(b"DISARM\r\n")
                time.sleep(0.05)
        except Exception:
            pass
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        if self._arm_thread:
            self._arm_thread.join(timeout=1.0)
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass


def thrust_n(fields):
    """추력(N), 위쪽이 양수. 스탠드는 수직 하중을 음수로 읽으므로 -Fz. tare 된 값 사용."""
    fz = fields.get("Fz")
    return None if fz is None else -fz


def torque_nm(fields):
    """추력축 기준 반작용 토크(N·m). 부호는 로터 A 기준 - 로터 B."""
    return fields.get("Tz")


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
        # 실제로 도달한 수신 주기를 세어 둔다(스트림별). 요청한 Hz 가 그대로 나온다는 보장이
        # 없기 때문(PX4 내부 발행 주기·MAV_x_RATE 상한, 로드셀 ARM 상태에 걸리면 조용히 낮아진다).
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
            "thrust_n": None, "torque_nm": None,
            "loadcell_hz": 0.0, "loadcell_state": None,
            "point_done": 0, "point_total": 0,
            "elapsed_s": 0.0, "eta_s": 0.0,
            "out_path": None,
            "achieved_rates": {},
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
def hold_and_log(ctl, conn, writers, latest, cfg, a_us, b_us, phase, duration, record,
                  sweep_idx, loadcell=None, enabled=True):
    """(A,B)를 각 목표 µs로 duration초 '유지'하며, record=True면 수신값을 스트림별 CSV로 기록한다.

    - 명령은 타임아웃이 있으므로 duration 동안 '계속 재전송'해야 값이 유지된다(enabled=False
      면 재전송 로직/타이밍은 그대로 돌되 실제 전송만 생략 — no_motor 드라이런).
    - 매 반복마다 ctl.check_abort() 로 STOP/watchdog 을 점검(즉시 중단 가능).
    - 스트림마다(servo/battery/esc/loadcell) 자기 메시지가 도착했을 때만 그 파일에 한 줄
      쓴다 — 다른 스트림의 최신값을 끼워 넣어 가짜 rate 를 만들지 않는다.
    - 최신 전압/전류/실측 PWM 은 GUI 표시용으로 ctl.status 에도 갱신한다.
    """
    a_norm = us_to_norm(a_us)
    b_norm = us_to_norm(b_us)
    prefix = [phase, sweep_idx, a_us, b_us, round(a_norm, 4), round(b_norm, 4)]
    deadline = time.time() + duration
    next_send = 0.0   # 다음 재전송 예정 시각(0이면 즉시)

    while time.time() < deadline:
        ctl.check_abort()             # (1) STOP/watchdog 점검 — 걸리면 AbortMeasurement
        now = time.time()

        # (2) 재전송 주기가 되면 두 모터 명령을 다시 보낸다(값 유지). no_motor 면 생략.
        if now >= next_send:
            send_actuator_test(conn, MOTOR_A_FUNC, a_norm, cfg["timeout_s"], enabled=enabled)
            send_actuator_test(conn, MOTOR_B_FUNC, b_norm, cfg["timeout_s"], enabled=enabled)
            next_send = now + 1.0 / cfg["resend_hz"]

        # (3) MAVLink 수신 메시지를 '버퍼가 빌 때까지' 처리한다. 상한(MAX_DRAIN_PER_LOOP)을
        # 두어야 메시지가 아무리 빨리 들어와도 주기적으로 deadline/check_abort 로 돌아온다.
        msg = conn.recv_match(blocking=True, timeout=0.05)
        drained = 0
        while msg is not None and drained < MAX_DRAIN_PER_LOOP:
            drained += 1
            mtype = msg.get_type()
            t_epoch = time.time()

            if mtype == "BATTERY_STATUS" and getattr(msg, "id", 0) == 0:
                v, c = read_battery(msg)
                if v is not None:
                    latest["voltage_v"] = v
                if c is not None:
                    latest["current_a"] = c
                ctl.set_status(voltage_v=latest["voltage_v"], current_a=latest["current_a"])
                ctl.note_rx("battery")
                if record:
                    writers.battery.writerow([t_epoch] + prefix + [latest["voltage_v"], latest["current_a"]])
                # 저전압 컷오프 — 리포 보호.
                floor = cfg.get("min_voltage_v") or 0
                if floor and latest["voltage_v"] and latest["voltage_v"] < floor:
                    raise AbortMeasurement(
                        f"저전압 컷오프: {latest['voltage_v']:.2f}V < {floor:.2f}V")

            elif mtype == "ESC_STATUS":
                base = int(getattr(msg, "index", 0))
                for k, rpm in enumerate(getattr(msg, "rpm", [])[:4]):
                    slot = base + k
                    if 0 <= slot < 4:
                        latest["esc_rpm"][slot] = rpm
                ctl.set_status(esc_rpm=list(latest["esc_rpm"]))
                ctl.note_rx("esc")
                if record:
                    row = [t_epoch] + prefix + ["" if r is None else r for r in latest["esc_rpm"]]
                    writers.esc.writerow(row)

            elif mtype == "SERVO_OUTPUT_RAW":
                servo = [getattr(msg, f"servo{i}_raw") for i in range(1, 9)]   # 실측 PWM 8채널
                ctl.set_status(servo_raw=servo)   # GUI 실시간 표시용
                ctl.note_rx("servo")
                if record:
                    row = [t_epoch] + prefix + [msg.time_usec] + servo
                    writers.servo.writerow(row)

            msg = conn.recv_match(blocking=False)   # 남은 것 비우기

        # (4) 로드셀 큐를 비우고, 자기 파일에 자기 rate 로 기록한다.
        if loadcell is not None:
            for t_epoch, fields in loadcell.drain():
                ctl.note_rx("loadcell")
                thrust = thrust_n(fields)
                torque = torque_nm(fields)
                ctl.set_status(thrust_n=thrust, torque_nm=torque,
                               loadcell_state=fields.get("t_stm_ms") and loadcell.last_state)
                if record:
                    row = ([t_epoch] + prefix + [fields["t_stm_ms"]]
                           + [fields[f"{ch}_raw"] for ch in FT_CHANNELS]
                           + [fields[ch] for ch in FT_CHANNELS]
                           + [fields["force_count"], fields["torque_count"]])
                    writers.loadcell.writerow(row)
            ctl.set_status(loadcell_hz=loadcell.stats()["hz"])


class Writers:
    """네 스트림 CSV 파일의 csv.writer 를 담아 두는 단순 컨테이너. loadcell 은 로드셀 미설정 시 None."""
    __slots__ = ("servo", "battery", "esc", "loadcell")

    def __init__(self, servo, battery, esc, loadcell):
        self.servo = servo
        self.battery = battery
        self.esc = esc
        self.loadcell = loadcell


# ============================================================================
# 측정 메인 흐름 — 별도 스레드에서 돈다. 예외는 모두 잡아 상태로 남기고, 끝에 반드시 정지.
# ============================================================================
def run_measurement(ctl, cfg):
    conn = None
    files = []
    loadcell = None
    motor_enabled = not cfg.get("no_motor")
    out_dir = None
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
        ok, ack_msg = preflight_ack(conn, cfg["timeout_s"], enabled=motor_enabled)
        ctl.set_status(message=f"프리플라이트: {ack_msg}")
        if not ok:
            raise RuntimeError(ack_msg)

        # 2b) 로드셀 연결 + tare (loadcell_device 가 비어 있으면 완전히 건너뜀 — 선택 하드웨어)
        if cfg.get("loadcell_device"):
            ctl.set_status(message=f"로드셀 연결 시도: {cfg['loadcell_device']}")
            loadcell = LoadCellReader(cfg["loadcell_device"], cfg["loadcell_baud"])
            try:
                loadcell.start()
            except Exception as e:
                raise RuntimeError(f"로드셀 연결 실패({cfg['loadcell_device']}): {e}")
            # ARM 하트비트를 보내기 시작한 직후엔 아직 예전 ~10Hz/DISARMED 샘플이 큐에 남아
            # 있을 수 있다 — st=ARMED 로 전환될 때까지 기다린 뒤(그 사이 쌓인 것은 버리고)
            # 50Hz 로 안정된 상태에서만 tare 창을 연다.
            ctl.set_status(message="로드셀 ARM 대기 중 (50Hz 전환 확인)")
            armed_ok = loadcell.wait_armed(timeout_s=3.0)
            loadcell.drain()   # ARM 전/전환 중 쌓인 샘플은 버림 — tare 는 여기서부터 깨끗하게
            if not armed_ok:
                ctl.set_status(message="경고: 로드셀이 ARMED 상태로 전환 확인 안 됨(그래도 tare 진행)")
            if cfg.get("tare_s", 0) > 0:
                ctl.set_status(message=f"로드셀 tare 중 ({cfg['tare_s']}s, ARMED/50Hz 상태)")
                offsets, n = loadcell.compute_tare(cfg["tare_s"])
                if offsets is None:
                    ctl.set_status(message="경고: tare 샘플을 못 받음 — raw=tare 로 진행")
                else:
                    ctl.set_status(message=f"tare 완료 (n={n})")

        # 3) 격자/조합 준비 + 출력 디렉터리/CSV 열기
        orders = build_orders(cfg)
        point_total = sum(len(o) for o in orders)
        t_start = int(time.time())
        run_stamp = time.strftime("%Y-%m-%d_%H%M%S", time.localtime(t_start))
        base_dir = (cfg.get("out_dir") or "").strip()
        out_dir = os.path.join(base_dir, f"{describe_grid(cfg)}_{run_stamp}")
        os.makedirs(out_dir, exist_ok=True)
        ctl.set_status(combo_total=len(orders[0]) if orders else 0, sweep_total=cfg["repeats"],
                       point_total=point_total, out_path=out_dir)
        latest = {"voltage_v": None, "current_a": None,
                  "esc_rpm": [None, None, None, None]}

        # 설정값(dwell/repeats/격자 범위 등)은 개별 CSV 안에 남지 않는다 → run.json 에 저장.
        write_run_json(out_dir, {
            "cfg": {k: v for k, v in cfg.items() if k not in ("device", "baud")},
            "t_start_epoch": t_start,
            "grid": {"a_values": frange_us(cfg["a_start"], cfg["a_end"], cfg["a_step"]),
                     "b_values": frange_us(cfg["b_start"], cfg["b_end"], cfg["b_step"]),
                     "n_grid_points": len(orders[0]) if orders else 0,
                     "repeats": cfg["repeats"], "point_total": point_total},
            "idle_pre_s": cfg.get("idle_pre_s", 0.0),
            "idle_post_s": cfg.get("idle_post_s", 0.0),
            "outcome": "in_progress",
        })

        servo_f = open(os.path.join(out_dir, "servo.csv"), "w", newline="", encoding="utf-8")
        battery_f = open(os.path.join(out_dir, "battery.csv"), "w", newline="", encoding="utf-8")
        esc_f = open(os.path.join(out_dir, "esc.csv"), "w", newline="", encoding="utf-8")
        files.extend([servo_f, battery_f, esc_f])
        servo_w, battery_w, esc_w = csv.writer(servo_f), csv.writer(battery_f), csv.writer(esc_f)
        servo_w.writerow(SERVO_COLUMNS)
        battery_w.writerow(BATTERY_COLUMNS)
        esc_w.writerow(ESC_COLUMNS)
        loadcell_w = None
        if loadcell is not None:
            loadcell_f = open(os.path.join(out_dir, "loadcell.csv"), "w", newline="", encoding="utf-8")
            files.append(loadcell_f)
            loadcell_w = csv.writer(loadcell_f)
            loadcell_w.writerow(LOADCELL_COLUMNS)
        writers = Writers(servo_w, battery_w, esc_w, loadcell_w)

        # 여기서부터 모터가 움직일 수 있음 → watchdog 켬(브라우저 끊기면 즉시 정지)
        ctl.watchdog_enabled = True
        ctl.last_ping = time.time()

        # 4-1) ESC 워밍업(최소값 유지, 기록 안 함) — 계단 시작 전 딱 한 번
        ctl.set_status(state="running", message=f"ESC 워밍업 {cfg['warmup_s']}s (최소값 유지)")
        hold_and_log(ctl, conn, writers, latest, cfg,
                     PWM_MIN_US, PWM_MIN_US, "warmup", cfg["warmup_s"], record=False,
                     sweep_idx=0, loadcell=loadcell, enabled=motor_enabled)

        # 4-1b) 무부하 구간을 '기록'한다(phase="idle_pre"). 양 채널 1000µs·전류≈0 이므로
        # 여기 전압이 곧 배터리 실제 잔량(SoC)이다.
        if cfg.get("idle_pre_s", 0) > 0:
            ctl.set_status(message=f"무부하 기준 전압 측정 {cfg['idle_pre_s']}s")
            hold_and_log(ctl, conn, writers, latest, cfg,
                         PWM_MIN_US, PWM_MIN_US, "idle_pre", cfg["idle_pre_s"],
                         record=True, sweep_idx=0, loadcell=loadcell, enabled=motor_enabled)

        # 주기 측정은 여기서부터(연결·워밍업 구간을 빼야 실제 스윕 주기가 나온다)
        ctl.rx_counts = {}
        ctl.rx_since = time.time()

        # 4-2) 계단식 스윕: 격자를 repeats 번(반복마다 독립적인 방문 순서) 반복.
        # 스텝 사이에 정지/안정화/간격 없음.
        point_done = 0
        last_a, last_b = PWM_MIN_US, PWM_MIN_US
        for r, points in enumerate(orders, start=1):
            for ci, (a_us, b_us) in enumerate(points, start=1):
                phase = f"A{a_us}_B{b_us}"
                ctl.set_status(state="running",
                               message=f"스윕 {r}/{cfg['repeats']} · 조합 {ci}/{len(points)} "
                                       f"(A={a_us}µs, B={b_us}µs)",
                               sweep_idx=r, combo_idx=ci, combo_total=len(points),
                               a_us=a_us, b_us=b_us)
                # 바로 dwell 동안 유지+기록 → 끝나면 즉시 다음 조합으로(계단 한 칸)
                hold_and_log(ctl, conn, writers, latest, cfg,
                             a_us, b_us, phase, cfg["dwell_s"], record=True, sweep_idx=r,
                             loadcell=loadcell, enabled=motor_enabled)
                point_done += 1
                last_a, last_b = a_us, b_us
                ctl.set_status(point_done=point_done)
            for f in files:
                f.flush()   # 스윕 한 바퀴 끝날 때마다 디스크에 안전 저장

            # 다음 스윕(세트)이 남았으면: 마지막 조합에서 다음 스윕 첫 조합으로 '서서히' 이동.
            if r < cfg["repeats"]:
                first_a, first_b = orders[r][0]
                ctl.set_status(message=f"스윕 {r} 종료 → 다음 스윕 준비(서서히 감속)")
                ctl.check_abort()
                ramp_to(conn, last_a, last_b, first_a, first_b, cfg["timeout_s"],
                        log={"ctl": ctl, "writers": writers, "latest": latest,
                             "cfg": cfg, "sweep_idx": r, "loadcell": loadcell},
                        enabled=motor_enabled)
                last_a, last_b = first_a, first_b

        # 4-3) 정상 종료: 마지막 값에서 서서히 정지.
        # 이 구간을 기록해야 추력이 크게 떨어지는 '가장 뚜렷한 에지'가 CSV에 남는다.
        ramp_down(conn, last_a, last_b, cfg["timeout_s"],
                  log={"ctl": ctl, "writers": writers, "latest": latest,
                       "cfg": cfg, "sweep_idx": cfg["repeats"], "loadcell": loadcell},
                  enabled=motor_enabled)

        # 4-4) 스윕 후 무부하 전압(phase="idle_post").
        if cfg.get("idle_post_s", 0) > 0:
            ctl.set_status(message=f"무부하 종료 전압 측정 {cfg['idle_post_s']}s")
            hold_and_log(ctl, conn, writers, latest, cfg,
                         PWM_MIN_US, PWM_MIN_US, "idle_post", cfg["idle_post_s"],
                         record=True, sweep_idx=0, loadcell=loadcell, enabled=motor_enabled)

        # 실제로 받은 주기를 보고한다. 요청한 Hz 가 그대로 나오는 경우는 오히려 드물다.
        rates = ctl.achieved_rates()
        want = (cfg.get("servo_hz", 50), cfg.get("bat_hz", 20), cfg.get("esc_hz", 20), LOADCELL_ARM_HZ)
        got = (rates.get("servo", 0), rates.get("battery", 0), rates.get("esc", 0), rates.get("loadcell", 0))
        rate_msg = ("실측 주기 servo %.0f/%d Hz, battery %.0f/%d Hz, esc %.0f/%d Hz, loadcell %.0f/%d Hz"
                    % (got[0], want[0], got[1], want[1], got[2], want[2], got[3], want[3]))
        if got[2] == 0:
            rate_msg += " — ESC 텔레메트리 없음(RPM 미기록)"
        if got[0] < 0.7 * want[0]:
            rate_msg += " — servo 주기가 요청보다 낮음: PX4 발행 주기/MAV_x_RATE 확인"
        if loadcell is not None and got[3] < 0.7 * want[3]:
            rate_msg += " — loadcell 주기가 낮음: ARM 하트비트/시리얼 연결 확인"
        ctl.set_status(achieved_rates=rates)
        ctl.set_status(state="done", message=f"완료 — 저장: {out_dir} · {rate_msg}")

        write_run_json(out_dir, {
            "outcome": "completed",
            "achieved_rates_hz": rates,
            "tare_offsets": dict(loadcell.tare) if loadcell is not None else None,
            "loadcell_stats": loadcell.stats() if loadcell is not None else None,
        })

    except AbortMeasurement as ab:
        # STOP 버튼 또는 watchdog: 사용자/안전 중단
        ctl.set_status(state="stopped", message=f"중단됨: {ab.reason}")
        if out_dir:
            write_run_json(out_dir, {"outcome": "aborted", "abort_reason": ab.reason})
    except Exception as e:
        # 그 외 오류(연결 실패, 시동 상태 등)
        ctl.set_status(state="error", message=f"오류: {e}")
        if out_dir:
            write_run_json(out_dir, {"outcome": "error", "error": str(e)})
    finally:
        # 어떤 경우든 마지막엔 반드시 모터 정지 + 로드셀 정지(DISARM) + 파일 저장
        ctl.watchdog_enabled = False
        if conn is not None:
            try:
                stop_motors_immediate(conn, cfg["timeout_s"], enabled=motor_enabled)
            except Exception:
                pass
        if loadcell is not None:
            try:
                loadcell.stop()
            except Exception:
                pass
        for f in files:
            try:
                f.flush()
                f.close()
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
    ap = argparse.ArgumentParser(description="BLDC 2D PWM 계단식 스윕/로드셀 로거 — HTML GUI 서버")
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
