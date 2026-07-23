import csv
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

# 스페이스바 감지(Linux/macOS)
try:
    import select
    import termios
    import tty
    _HAS_TTY = True
except ImportError:
    _HAS_TTY = False

# ===================== 설정 (여기만 고치면 됨) =====================
DEVICE = "/dev/ttyAMA0"       # 라즈베리파이 5 GPIO UART (핀 8·10)
BAUD = 921600
OUTDIR_BASE = "logs"          # 저장 상위 폴더 (실행마다 하위에 타임스탬프 폴더 생성)
REQUEST_HZ = 50               # 화이트리스트 메시지를 이 주기로 능동 요청(PX4 확실)

# 기록할 메시지 종류만. 필요하면 추가/삭제 (공식명: common.html)
MESSAGES = {
    "ATTITUDE",             # [제어] 실제 자세 roll/pitch/yaw(오일러) + 각속도 - 사람이 읽기 쉬움
    "ATTITUDE_QUATERNION",  # [제어] 실제 자세 쿼터니언 - 짐벌락 없음, 셋포인트와 직접 비교용
    "ATTITUDE_TARGET",      # [제어] 제어기 자세 셋포인트(쿼터니언 q + 추력·바디각속도)
    "SERVO_OUTPUT_RAW",     # [제어] 실제 서보/모터 PWM 출력
    "GLOBAL_POSITION_INT",  # [GPS ] 융합 위치/고도/속도
    "GPS_RAW_INT",          # [GPS ] 원시 GPS(위성수·fix 상태)
}
# =================================================================

stop_event = threading.Event()


def key_listener():
    """스페이스바가 눌리면 stop_event를 세팅한다."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            if select.select([sys.stdin], [], [], 0.2)[0]:
                if sys.stdin.read(1) == " ":
                    stop_event.set()
                    return
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    conn = mavutil.mavlink_connection(DEVICE, baud=BAUD, source_system=191)
    print(f"연결 시도: {DEVICE} @ {BAUD} — 하트비트 대기...")
    conn.wait_heartbeat()
    print(f"연결됨 sys={conn.target_system}")

    # 화이트리스트 각 메시지를 SET_MESSAGE_INTERVAL로 능동 요청.
    # (PX4는 REQUEST_DATA_STREAM을 무시 → 기본 스트림에 없는 메시지,
    #  예: SERVO_OUTPUT_RAW 를 확실히 받으려면 이 방식이 필요)
    for name in MESSAGES:
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
        if msg_id is None:
            print(f"  (경고: 알 수 없는 메시지명 '{name}' — 건너뜀)")
            continue
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / REQUEST_HZ), 0, 0, 0, 0, 0)

    outdir = Path(OUTDIR_BASE) / f"log_{int(time.time())}"
    outdir.mkdir(parents=True, exist_ok=True)

    if _HAS_TTY and sys.stdin.isatty():
        threading.Thread(target=key_listener, daemon=True).start()
        hint = "[스페이스바 또는 Ctrl+C = 저장 후 종료]"
    else:
        hint = "[Ctrl+C = 저장 후 종료]"
    print(f"기록 시작 → {outdir}/   {hint}")
    print(f"대상 메시지: {', '.join(sorted(MESSAGES))}")

    writers = {}
    last_flush = time.time()
    count = 0
    try:
        while not stop_event.is_set():
            msg = conn.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype not in MESSAGES:       # 화이트리스트 외 무시
                continue

            if mtype not in writers:
                fields = msg.get_fieldnames()
                fp = open(outdir / f"{mtype}.csv", "w", newline="")
                w = csv.writer(fp)
                w.writerow(["t_rpi"] + list(fields))
                writers[mtype] = (fp, w, fields)

            fp, w, fields = writers[mtype]
            w.writerow([time.time()] + [getattr(msg, fn) for fn in fields])
            count += 1

            now = time.time()
            if now - last_flush >= 1.0:
                for fp, _, _ in writers.values():
                    fp.flush()
                last_flush = now
                print(f"\r기록 중... {len(writers)}종류 / {count}행", end="")
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for fp, _, _ in writers.values():
            fp.close()
        print(f"\n저장 완료: {outdir.resolve()}/   ({len(writers)}종류, {count}행)")


if __name__ == "__main__":
    main()