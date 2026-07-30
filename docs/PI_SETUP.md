# 라즈베리파이 셋업 (pwm_thrust_map.py 기준)

`pwm_thrust_map.py` 하나로 픽스호크(모터 명령/전압/전류)와 STM32 로드셀(추력/토크)을
같은 프로세스·같은 시계로 함께 기록한다. 이 문서는 새 라즈베리파이에서 이 스크립트를
돌리기까지 필요한 최소 설정만 간략히 정리한 것.

## 1. 하드웨어 연결

| 장치 | 연결 | 비고 |
|---|---|---|
| 픽스호크 (PX4) | Pi5 GPIO UART(TX/RX, 핀 8·10) → `/dev/ttyAMA0` | 921600 baud. TX/RX 교차 주의 |
| STM32 로드셀 | USB-A 포트 → `/dev/ttyACM0` | 115200 baud. **Pi5의 USB-C는 전원 입력이라 반드시 USB-A에** |

`ls /dev/ttyACM* /dev/ttyAMA*` 로 두 장치가 보이는지 먼저 확인.
(장치명이 재부팅마다 바뀌면 `python -m tvcbench devices --probe` 로 udev 규칙을 뽑아
`/dev/tvc-loadcell` 처럼 고정할 수 있음 — `pwm_thrust_map.py`는 필수는 아님, 그냥 GUI의
`loadcell_device` 칸에 실제 경로를 넣으면 됨.)

## 2. 네트워크 / SSH 접속

- Pi가 WiFi(예: 학교/공용 AP)에 연결되어 있으면 그걸로 충분 — 유선(LAN)이 따로 필요 없다.
  `hostname -I` 로 현재 IP들을 확인하고, 그중 원격 PC와 같은 서브넷에 있는 IP로 접속.
- 비밀번호 대신 **SSH 키 인증**을 쓸 것: 원격 PC의 `~/.ssh/id_ed25519.pub` 내용을 Pi의
  `~/.ssh/authorized_keys` 에 추가.
  ```bash
  mkdir -p ~/.ssh && chmod 700 ~/.ssh
  echo "ssh-ed25519 AAAA...공개키..." >> ~/.ssh/authorized_keys
  chmod 600 ~/.ssh/authorized_keys
  ```
- 접속 확인: `ssh <user>@<pi-ip> hostname`

## 3. 파이썬 환경

```bash
cd ~/CODE/tvc-data     # 저장소 경로
python3 -m venv .venv
.venv/bin/python -m pip install pymavlink pyserial
```

`pwm_thrust_map.py` 자체가 필요로 하는 외부 라이브러리는 이 둘뿐이다(`requirements.txt`
전체를 다 설치할 필요 없음 — 그건 `tvctools`/`tvcbench`/`gui.py` 등 다른 도구까지 포함한
목록). `python -m py_compile pwm_thrust_map.py` 로 문법 확인만 해도 됨.

⚠️ **흔한 함정**: PyPI에 `pyserial`과 이름이 비슷한 `serial`(완전히 다른, 시리얼포트와
무관한 패키지)이 따로 있다. `pip install serial`을 잘못 실행하면 `import serial`은
성공하지만 `serial.Serial(...)`/`serial.tools.list_ports`가 없어 로드셀 연결이 조용히
실패한다. 증상: GUI 포트 드롭다운이 "No serial ports found"만 뜸. 확인/수정:
```bash
.venv/bin/python -m pip show serial      # Summary가 시리얼포트와 무관하면 잘못 설치된 것
.venv/bin/python -m pip uninstall serial -y
.venv/bin/python -m pip install pyserial
```

## 4. 픽스호크(QGC) 사전 설정 — 반드시 먼저 맞출 것

- Actuator 출력: `Minimum = 1000`, `Maximum = 2000`, `Disarmed = 1000`
- `THR_MDL_FAC = 0` (추력곡선 보정 끄기 → 정규화↔PWM 선형)
- 측정 중 픽스호크는 **시동 해제(disarmed)** 상태여야 함(`MAV_CMD_ACTUATOR_TEST`가 armed
  상태에서 거부됨)

## 5. 실행

```bash
.venv/bin/python pwm_thrust_map.py            # 기본: 127.0.0.1:8000 (Pi 자기 자신만)
.venv/bin/python pwm_thrust_map.py --host 0.0.0.0   # 다른 기기 브라우저에서 접속하려면
```

브라우저에서 `http://<pi-ip>:8000` 접속 → GUI에서 격자/dwell/로드셀 장치 설정 →
[측정 시작]. 진행 중 로드셀은 서버가 자동으로 `ARM 1000` 하트비트(50Hz)를 계속 보내
50Hz 로 샘플링되도록 유지하고, tare는 로드셀이 `ARMED` 상태로 전환된 뒤에만 시작한다 —
둘 다 코드가 알아서 처리하므로 운영자가 신경 쓸 부분 아님.

각 런은 `thrust_map_<epoch>/` 폴더에 `servo.csv`, `battery.csv`, `esc.csv`,
`loadcell.csv`, `run.json`(설정값·tare 오프셋·스트림별 실측 Hz)을 남긴다.

## 6. 안전

- `no_motor` 체크박스를 켜면 모터 명령만 생략하고 나머지(타이밍·로깅·로드셀·ARM
  하트비트)는 그대로 돌아간다 — 배선/로깅만 검증할 때 사용.
- 브라우저 탭을 닫으면 0.5초 폴링이 끊기고, 서버가 2초 안에 감지해 자동으로 모터를
  정지한다(watchdog). STOP 버튼으로도 즉시 정지 가능.
