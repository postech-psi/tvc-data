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
}

# CSV 컬럼(헤더). pwm_thrust_grid.py 와 '동일' + 맨 뒤에 sweep_idx(몇 번째 스윕인지) 한 개만 추가.
#   servo1~8_raw = 각 물리 출력 채널의 '실제' PWM(µs) 실측값.
#   → 로드셀 로그와 t_epoch(에폭 시각)로 병합할 때 기존 파이프라인과 그대로 호환된다.
CSV_HEADER = (
    ["t_epoch", "t_fc_us", "phase",
     "a_cmd_us", "b_cmd_us", "a_cmd_norm", "b_cmd_norm"]
    + [f"servo{i}_raw" for i in range(1, 9)]
    + ["voltage_v", "current_a", "sweep_idx"]
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


def request_rates(conn, servo_hz=20, sys_hz=10):
    """필요한 메시지의 송신 주기를 SET_MESSAGE_INTERVAL로 올린다(공식 권장 방식).

    - SERVO_OUTPUT_RAW: 실제 출력 PWM(µs)  → 우리가 기록할 '진리값'
    - SYS_STATUS      : 배터리 전압/전류
    interval(µs) = 1e6 / rate_hz
    """
    for msg_id, hz in (
        (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, servo_hz),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, sys_hz),
    ):
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,            # param1: 메시지 ID
            int(1e6 / hz),     # param2: 주기(µs)
            0, 0, 0, 0, 0,
        )


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


def ramp_down(conn, a_us, b_us, timeout_s):
    """[정상 종료] 현재 (a_us,b_us)에서 최소값(1000µs)까지 여러 단계로 '서서히' 내린다.

    측정을 정상적으로 다 마쳤을 때 딱 한 번 사용 — 급격한 전류 컷·기계적 충격 완화용.
    (긴급 상황에는 쓰지 않는다. 긴급은 stop_motors_immediate 로 즉시 끈다)
    """
    for i in range(1, RAMP_STEPS + 1):
        frac = 1.0 - i / float(RAMP_STEPS)        # 1 → 0 으로 감소
        a = PWM_MIN_US + (a_us - PWM_MIN_US) * frac
        b = PWM_MIN_US + (b_us - PWM_MIN_US) * frac
        send_actuator_test(conn, MOTOR_A_FUNC, us_to_norm(a), timeout_s)
        send_actuator_test(conn, MOTOR_B_FUNC, us_to_norm(b), timeout_s)
        time.sleep(RAMP_S / RAMP_STEPS)


def frange_us(start, end, step):
    """start~end(포함)까지 step 간격의 µs 리스트를 만든다(정수 µs)."""
    values = []
    v = start
    while v <= end:
        values.append(v)
        v += step
    return values


def build_combos(cfg):
    """격자 범위(cfg)로 (A,B) 조합 목록을 만든다. A가 바깥, B가 안쪽(= grid 파일과 동일 순서).

    A·B 시작=끝 이면 각 축이 1개 → 조합 1개("한 조합만" 케이스).
    이 목록 순서가 곧 '계단'을 올라가는 순서다.
    """
    a_vals = frange_us(cfg["a_start"], cfg["a_end"], cfg["a_step"])
    b_vals = frange_us(cfg["b_start"], cfg["b_end"], cfg["b_step"])
    return [(a, b) for a in a_vals for b in b_vals]


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

        # (3) 들어오는 메시지를 짧게 받아 처리
        msg = conn.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue
        mtype = msg.get_type()

        if mtype == "SYS_STATUS":
            # voltage_battery: mV→V,  current_battery: cA(10mA단위)→A
            latest["voltage_v"] = msg.voltage_battery / 1000.0
            latest["current_a"] = msg.current_battery / 100.0
            ctl.set_status(voltage_v=latest["voltage_v"], current_a=latest["current_a"])

        elif mtype == "SERVO_OUTPUT_RAW":
            servo = [getattr(msg, f"servo{i}_raw") for i in range(1, 9)]   # 실측 PWM 8채널
            ctl.set_status(servo_raw=servo)   # GUI 실시간 표시용
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
                writer.writerow(row)


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
        request_rates(conn)
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
        out_path = f"{OUT_PREFIX}_{int(time.time())}.csv"
        ctl.set_status(combo_total=len(combos), sweep_total=cfg["repeats"],
                       point_total=point_total, out_path=out_path)
        latest = {"voltage_v": None, "current_a": None}

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

        # 4-3) 정상 종료: 마지막 값에서 서서히 정지
        ramp_down(conn, last_a, last_b, cfg["timeout_s"])
        ctl.set_status(state="done", message=f"완료 — 저장: {out_path}")

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
