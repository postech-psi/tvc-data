# TVC 시스템 식별

ESP32 + ICM-20948 IMU 1개 + 짐벌 서보 2개(Outer/Inner)로 **PWM→각도 매핑**과
**step 응답**을 측정하는 프로젝트다. IMU를 1,000 Hz로 읽어 서보의 정적/동적
특성을 뽑아 제어기 설계 입력으로 쓴다.

## 1. 코드 구성

| 파일 | 역할 |
|---|---|
| `calibrate.py` | **보정** — 장착 자세에서 `calibration.json` 생성 (1회) |
| `run_experiment.py` | **실행** — health → mapping(A) → step(B), 세션 저장 |
| `plot.py` | **모든 플롯 하나로.** 서브커맨드: `surface`(범용 3D 큐빅 표면·모터 추력에도 사용), `grid`(pitch/yaw 표면), `mapping`, `step`, `bode`(chirp), `deadband`. 각 `--show`로 인터랙티브 |
| `analyze.py` | 분석 엔진 (보정·health·mapping·step 판정). `run_experiment`가 호출 |
| `sid_capture.py` | 펌웨어 업로드 + serial 로거 |
| `smoke_test.py` | (선택) 배선·축 확인용 저진폭 이동 |
| `src/main.cpp` | ESP32 펌웨어. 맨 위에 핀·PWM·샘플링 파라미터 |
| `tests/` | 하드웨어 없이 도는 합성 테스트 |

파라미터 위치: 펌웨어(핀/PWM/dwell/step)는 `src/main.cpp` 상단, 분석 판정 기준은
`analyze.py` 상단, 통신은 `sid_capture.py` 상단, 워크플로는 `run_experiment.py` 상단.

## 2. 하드웨어

| 장치 | 연결 |
|---|---|
| ESP32-WROOM-32D | USB serial 921600 baud |
| ICM-20948 | I2C **200 kHz**, 주소 **0x69**, SDA=21, SCL=22 |
| 서보 Outer(A)/Inner(B) | GPIO 18 / 19, 333 Hz |
| 서보 전원 | 별도 **7.4 V BEC, 3 A↑**, ESP32·BEC·서보 **GND 공통** |

usable PWM 범위(명령 제한, `src/main.cpp` 상단): Outer 1370–1690, Inner 1310–2040 µs,
중립 1520 µs. 링크/horn 재조립 시 반드시 수동 재확인 후 값 수정.

> I2C는 200 kHz. 400 kHz는 이 배선의 약한 풀업 여유로 간헐 실패해 200 kHz로 고정했다
> (1,000 Hz 데이터 레이트에는 충분).

## 3. 실행 순서

```powershell
# 0) 의존성
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe run_experiment.py --list      # 포트 확인

# 1) (선택) 배선·축 확인
.\.venv\Scripts\python.exe smoke_test.py --port COM21 --upload --mode slow

# 2) 보정 — IMU를 시험 자세로 고정·정지시키고 1회
.\.venv\Scripts\python.exe calibrate.py --port COM21

# 3) 실험 — 펌웨어 업로드 포함
.\.venv\Scripts\python.exe run_experiment.py --port COM21 --upload --yes

# 4) 플롯
.\.venv\Scripts\python.exe plot.py mapping --show
.\.venv\Scripts\python.exe plot.py step --show
```

`run_experiment.py` 옵션:
- `--calibration PATH` : 보정 파일 (기본 `calibration.json`)
- `--yes` : 모션 시험 전 안전 확인 Enter 생략
- `--tests "..."` : 지정한 테스트만 실행. 생략 시 기본 A/B 4개.
  선택지: `A OUTER, A INNER, B OUTER, B INNER, C OUTER, C INNER, D OUTER, D INNER`
- `--keep-going` : 한 테스트가 실패해도 다음 테스트 계속

추가 테스트(선택) 예:
```powershell
# chirp (주파수 응답)
.\.venv\Scripts\python.exe run_experiment.py --port COM21 --tests "C OUTER,C INNER" --yes
.\.venv\Scripts\python.exe plot.py bode --show
# deadband (백래시)
.\.venv\Scripts\python.exe run_experiment.py --port COM21 --tests "D OUTER,D INNER" --yes
.\.venv\Scripts\python.exe plot.py deadband --show
```

실행 중 자동으로 노트북 **절전/USB 서스펜드를 차단**한다(긴 캡처 중 샘플 유실 방지).

## 4. 보정과 health의 구분

- **보정(`calibrate.py` → `calibration.json`)**: 장착 자세에서 균일 스케일 하나를 잡아
  보정 후 `|g|=1.0`으로 맞춘다. 각도는 `atan2` 비율이라 **스케일에 불변**이므로, 이
  보정은 매핑/step 각도를 바꾸지 않고 크기만 정규화한다(health 게이트 통과용). 각도는
  raw 방향 + 중립 기준(referencing)으로 계산되고, 상수 바이어스는 referencing에서
  상쇄된다. 남는 오차는 cross-axis(데이터시트 ±2%) 수준으로 작다.
- **health(실험 내 1회)**: 서보 attach·중립·정착 후 한 번 측정. 보정 후 `|g|≈1`,
  gyro 노이즈 < 0.5 dps, I2C 에러, late sample을 확인하는 **모션 진입 게이트**다.

## 5. 실험 내용

- **Test A — 매핑**: PWM을 10 µs 간격으로 왕복(up/down×2), 각 지점 **1,000 ms dwell**
  후반 50%로 정적 각도. 결과: `angle↔PWM LUT`, `deg/us gain`, 가동범위, hysteresis,
  비선형성, 반대축 coupling.
- **Test B — step**: 중립에서 ±5·10·25·50·90% travel step. 결과: 직접 onset 지연,
  10–90% rise, ±2% settling, overshoot, peak slew, 모델 bandwidth, 정착 진동.
- **Test C — chirp (선택)**: 중립 부근 소진폭 로그 주파수 스윕(0.5–25 Hz, 20 s). 입력 PWM과
  gyro rate의 교차스펙트럼(Welch)으로 **PWM→각도 주파수 응답**을 직접 측정. 결과:
  gain/phase 곡선, coherence, **-3 dB bandwidth**, phase delay. step의 delay 모호성을 해소.
  `recommend_chirp=true`일 때 돌린다.
- **Test D — deadband (선택)**: 중립 부근 ±60 µs를 3 µs 간격으로 상승/하강. 미세
  hysteresis 루프에서 **백래시(µs·deg)** 와 국소 gain을 측정. mapping hysteresis가 크면
  (`recommend_deadband_test`) 돌린다.

판정은 `analyze.py` 상단 기준으로 자동. 단발 물리 글리치는 소량 허용(I2C·framing),
실제 유실(packet gap)·CRC는 엄격.

## 6. 저장 데이터

세션 `runs/session_YYYY-MM-DD_HHMMSS/`:

```
calibration_source.txt
health/{raw.csv, meta.json, analysis.json, session.log}
mapping_outer_gimbal/{raw.csv, meta.json, analysis.json, lut.csv,
                      mapping_points.csv, mapping_pretty.png, session.log}
mapping_inner_gimbal/{...}
step_outer_gimbal/{raw.csv, meta.json, analysis.json, step_summary.csv,
                   step_pretty.png, session.log}
step_inner_gimbal/{...}
session_results.json, REPORT.md
```

`raw.csv` = CRC 통과 레코드를 해석한 분석용 표. `meta.json`의
`effective_sample_rate_hz`, `sample_period_error_us_p99`, `packet_gaps`,
`binary_crc_errors`로 각 run이 1,000 Hz 조건을 만족했는지 판정한다.

## 7. 안전

- 서보 전류는 **별도 BEC**에서. ESP32/USB에서 뽑지 말 것. GND 공통 필수.
- BEC를 즉시 끌 수 있는 **물리 스위치**를 손 닿는 곳에.
- 시험 축은 **중력에 수평**(가속도계로 각도 측정 조건). 수직이면 span 부족으로 FAIL.
- 전원 후 3분 워밍업, 진동원(팬 등) 제거.

## 8. 제어기 반영 / 다음 단계

Test A의 역 LUT로 각도→PWM 변환, Test B의 onset·응답 모델·bandwidth를 vehicle
simulator에 넣는다. step 분석이 `recommend_chirp=true`(onset과 fitted delay 괴리 등)를
남기면 별도 chirp(주파수 스윕) 시험을 추가로 설계한다. 최종 비행 PID는 추력·질량·
CG·관성 모델과 합쳐 확정한다.
