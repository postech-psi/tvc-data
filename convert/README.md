# convert/ — .ulg 를 CSV 한 장으로

Pixhawk 로그(`.ulg`)를 CSV로 푸는 곳. `tvctools` 파이프라인과는 독립이며
`raw/`, `runs/`, `out/` 어느 것도 건드리지 않는다.

```
convert/
  ulog/       .ulg 원본을 여기 넣는다  (하위 폴더 만들어도 됨)
  csv/        변환 결과가 같은 폴더 구조로 여기 나온다
  convert.py  변환 스크립트
```

## 사용법

`.ulg` 파일을 `convert/ulog/` 에 넣고:

```bash
pip install pyulog          # 처음 한 번
python convert/convert.py
```

`convert/ulog/` 아래 모든 `.ulg`가 `convert/csv/` 에 **로그 하나당 CSV 하나**로
나온다. 하위 폴더 구조는 그대로 따라간다:

```
convert/ulog/2026-07-24/log_3_2026-7-24-00-27-40.ulg
  -> convert/csv/2026-07-24/log_3_2026-7-24-00-27-40.csv
```

이미 변환된 로그는 건너뛴다 (CSV가 `.ulg`보다 최신이면). 다시 변환하려면
`--force`.

| 플래그 | 하는 일 |
|---|---|
| `--list` | 변환하지 않고 로그별 토픽 목록과 메시지 수만 출력 |
| `--topics a,b,c` | 지정한 토픽만 변환 (기본: 전부) |
| `--dry-run` | 어디에 쓸지만 출력 |
| `--force` | 최신이어도 다시 변환 |
| `--in` / `--out` | 입출력 폴더 변경 (기본 `convert/ulog`, `convert/csv`) |

## 출력 형식

`runs/*/*/merged.csv` 와 같은 넓은 표를, 로그의 **모든 토픽**으로 확장한 것.

```
t_s,timestamp,actuator_outputs.output[0],actuator_outputs.output[1],battery_status.voltage_v,vehicle_attitude.q[0]
0.0,1784878912001170,1000,1000,,
0.0021,1784878912003270,,,,0.99981
0.01,1784878912011170,,,16.72,
0.0201,1784878912021270,1000,1450,,
```

- `t_s` — 로그 시작부터의 초. `timestamp` — 원본 마이크로초.
- 그 뒤로 로그에 있는 **모든 토픽의 모든 필드**가 `<토픽>.<필드>` 컬럼으로.
  같은 토픽의 두 번째 인스턴스만 `<토픽>_1.<필드>`로 구분한다.
- 한 행 = 기록된 메시지 하나. 그 메시지의 컬럼만 채워지고 나머지는 빈칸.
  같은 시각의 서로 다른 토픽은 한 행으로 합쳐진다.

### 무손실

리샘플·보간·반올림·forward-fill 을 하지 않는다. 원본 메시지의 모든 필드 값이
정확히 한 번씩 나타나므로 CSV만 있으면 `.ulg` 없이 같은 분석을 할 수 있다.

- **빈칸** = 그 시각에 그 토픽의 샘플이 없었다 (값 미상이 아님)
- **`nan`** = 로그에 실제로 NaN이 기록돼 있었다 — 빈칸과 다른 의미
- 숫자는 원래 값으로 정확히 되돌아가는 가장 짧은 표기로 쓴다
  (float32 `16.72`는 `16.719999313354492`가 아니라 `16.72`로)

### 용량 주의

무손실 + 한 장을 지키는 대가로 표가 넓고 대부분이 빈칸이다. 컬럼 수백 개, 행
수백만 개가 되며 **120 MB 로그가 수 GB CSV**가 될 수 있다. 행 수의 대부분은
고빈도 토픽(`sensor_combined`, `sensor_accel`, `sensor_gyro`, `vehicle_imu` 등)이
차지하므로, 먼저 `--list`로 뭐가 얼마나 있는지 보고 필요한 것만 뽑는 편이 낫다:

```bash
python convert/convert.py --list
python convert/convert.py --topics actuator_outputs,battery_status
```

이 벤치에서 쓰는 토픽은 `actuator_outputs`(지령 PWM)와
`battery_status`(전압/전류)다 — `tvctools/ulog.py` 참고.

## git

이 폴더의 `.ulg`와 CSV는 커밋하지 않는 게 좋다. `.ulg`는 이미 최상위
`.gitignore`의 `*.ulg`로 걸러지지만 CSV는 아니므로, 필요하면 직접 추가할 것:

```
convert/csv/*
!convert/csv/.gitkeep
```
