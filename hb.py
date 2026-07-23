import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, source_system=191)

print('하트비트 대기...')
if m.wait_heartbeat(timeout=30) is None:
    print('❌ 수신 없음 — 배선(Pix TX→RPi RX)·baud 확인'); raise SystemExit
print(f'✅ 수신 OK sys={m.target_system} comp={m.target_component}')

# 우리도 하트비트 송신 → 픽스호크 rx 증가 (QGC에서 확인 가능)
i=0
while i<50:
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    print('heartbeat sent →')
    time.sleep(1)
    i+=1



