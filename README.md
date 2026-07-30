# tvc-data

TVC 모델 로켓 프로젝트를 위한 동축(coaxial) 모터 추력/토크 벤치 데이터, 그리고
원시 로그를 PWM → 추력/토크 맵으로 변환하는 파이프라인.

## 구성

```
raw/                        원시 취득 파일, 절대 수정하지 않음
  2026-07-24/
    pwm/       thrust_map_<epoch>.csv    지령 PWM + 전압/전류        (Pi)
    loadcell/  data_<date>_<time>.csv    추력 (Fz) + 토크 (Tz)       (스탠드)
    ulog/      log_*.ulg                 비행 컨트롤러 로그           (Pixhawk)
      _redundant/                        중복 / 모터 없는 로그, 보관하되 무시

runs/                       실행별 정리된 사본 + 병합 데이터  (생성됨)
  2026-07-24_s1/
    A1400_B1000-2000_0051_1300/
      pwm.csv        원본 thrust_map 파일의 바이트 단위 정확한 사본
      thrust.csv     원본 로드셀 파일의 바이트 단위 정확한 사본
      merged.csv     위 두 개를 하나의 50 Hz 격자에서 시간 정렬한 것
      run.json       출처, 정렬 품질, 정상상태 테이블
      steps.png      원시 추력 vs 지령 구간별 계단 평균

out/                        분석 산출물                        (생성됨)
  runs_index.csv/.json      모든 원시 파일의 카탈로그
  pwm_thrust_torque_map.csv 정상상태 지점, 두 로터 지령 모두
  pwm_thrust_torque_map.png 추력/토크/효율 vs B, A마다 곡선 하나
  coax_grid.png             (A, B) 평면 위의 추력과 토크
  voltage_sag.csv/.png      배터리 충전 상태 vs 추력

bench/                      새 포맷 취득 실행                  (생성됨)
  <YYYY-MM-DD_HHMMSS>/      실행당 디렉터리 하나 -- docs/ACQUISITION.md 참고

docs/SETUP.md               무엇이 무엇을 측정하는가 -- 먼저 읽을 것
docs/PREPROCESSING.md       원시 파일에서 맵 지점까지의 모든 단계
docs/ACQUISITION.md         tvcbench 재작성: 하나의 클록, 하나의 로거
docs/DATA_INVENTORY.md      무엇이 존재하는가, 날짜별, 전압 상태 포함
tvctools/                   분석 파이프라인
tvcbench/                   취득 (Raspberry Pi)
plans/                      실행 계획 -- 실행은 곧 체크인된 파일
tests/                      하드웨어 없이, 시뮬레이션 소스로 실행
gui.py                      로드셀 취득 (벤치 노트북, 대체됨)
pwm_thrust_map.py           스윕 러너 + 웹 GUI (Raspberry Pi, 대체됨)
pwm_map_gui.html            그 제어판
plot.py                     독립형 로드셀 뷰어
```

## 두 세대

`pwm_thrust_map.py` + `gui.py`가 `raw/`와 `runs/` 아래의 모든 것을 만들어 냈으며,
로드셀은 벤치 노트북에, 지령은 Pi에 있었다 — 클록이 세 개였고,
이들을 조화시키기 위한 사후(post-hoc) 정렬 단계 전체가 필요했다.

`tvcbench`가 이 둘을 대체한다. 로드셀이 Pi로 옮겨져 힘과 지령이
하나의 클록을 공유하므로 정렬이 필요 없다. 출력은 새 포맷으로 `bench/`에
저장되며, 기존 파이프라인과 그 데이터는 손대지 않는다.
[docs/ACQUISITION.md](docs/ACQUISITION.md)를 참고할 것.

```bash
python -m tvcbench selftest                       # 하드웨어 게이트, 매 세션 전에
python -m tvcbench plan show plans/coax_grid.yaml
python -m tvcbench run plans/coax_grid.yaml
```

실행 폴더 이름은 어떤 테스트였는지를 말해 준다: `A1400_B1000-2000_0051`은 로터 A를
1400 µs로 고정하고, 로터 B를 1000→2000 µs로 스윕했으며, 00:51에 시작했다는 뜻이다.
`A1850_B1850_1630`은 두 로터를 모두 1850 µs로 고정했다는 뜻이다. 이것이
**동축** 장비이기 때문에 두 지령이 모두 나타난다 — 추력, 특히 반작용 토크는
로터 쌍에 의존한다.

## 파이프라인

```bash
pip install -r requirements.txt

python -m tvctools organize --dry-run   # 새 원시 데이터를 raw/<date>/ 아래로 정리
python -m tvctools organize

python -m tvctools index                # 카탈로그 -> out/runs_index.csv
python -m tvctools build                # 그룹화 + 정렬 + 병합 -> runs/
python -m tvctools map                  # 추력/토크 맵 + 플롯 -> out/
```

`organize`와 `build`는 복사하거나 이동만 할 뿐, 원시 파일을 절대 수정하지 않는다.
모든 단계가 멱등(idempotent)이므로 데이터를 추가한 뒤 다시 실행해도 안전하다.

비행 컨트롤러 로그 하나만 따로 분석하려면:

```bash
python -m tvctools ulog raw/2026-07-24/ulog/log_3_2026-7-24-00-27-40.ulg
```

## 새 데이터 추가하기

1. 파일을 저장소 어디에든 넣는다 (또는 곧바로 `raw/<date>/<source>/`에).
2. `python -m tvctools organize` — 내용으로부터 날짜를 매기고 정리한다.
3. `python -m tvctools build && python -m tvctools map`.

`organize`는 가능한 경우 각 파일 내부에서 측정 날짜를 읽어오므로, git 체크아웃이
mtime을 다시 쓰더라도 어떤 것도 잘못 분류될 수 없다.

## 무엇이 무엇을 측정하는가

| 물리량 | 출처 | 사용하지 **말 것** |
|---|---|---|
| 지령 PWM (두 로터 모두) | `pwm/` — `a_cmd_us`, `b_cmd_us`, `phase` | 로드셀 파일의 `pwm` |
| 전압 / 전류 | `pwm/` — `voltage_v`, `current_a` | 로드셀 파일의 `Current_mA` |
| **추력** | `loadcell/` — `Fz`, 뉴턴 | — |
| **토크** | `loadcell/` — `Tz`, N·m | — |

로드셀 자체의 `pwm`, `rpm`, `Current_mA` 열은 설계상 죽어 있다:
Pi가 Pixhawk를 통해 모터를 구동하므로 스탠드는 스로틀을 결코 보지 못한다.
`index`는 이들을 결함이 아니라 "By design(설계상)" 항목으로 보고한다. 자세한 내용은
[docs/SETUP.md](docs/SETUP.md)에 있다.

## 정렬이 작동하는 방식

모든 처리 단계의 상세 내용은
[docs/PREPROCESSING.md](docs/PREPROCESSING.md)에 있다.

스탠드는 자유 진행(free-running) MCU 가동 시간을 기록하고, Pi는 UTC epoch을
기록한다. 로드셀 파일명이 이것을 약 1초 정밀도로 고정한 뒤, 상호상관(cross-correlation)이
나머지를 복원한다: 스윕에서는 추력(`−Fz`)을 **지령 PWM**과 대조하고, 지령에 상관시킬
분산이 없는 정상 홀드(constant hold)에서는 **전류**로 대체한다. 로드셀 파일당 하나의
지연(lag)이 `run.json`에 `lag_s` / `lag_corr` / `drive_signal`로 기록되며,
신뢰하기에 너무 약한 피크는 확신에 찬 틀린 숫자 대신 지연 0의 `weak`로 보고된다.

비행 컨트롤러 로그에는 사용 가능한 절대 시간이 없다 — `time_ref_utc`는 0이고
파일명은 몇 분씩 틀리다 — 그래서 `pwm/` 파일의 `t_fc_us` / `t_epoch` 쌍으로부터
날짜를 매긴다. 로그 하나가 여러 실행에 걸쳐 있다.

## PWM 계단 분해하기

계단 내 추력 잡음은 σ ≈ 0.7 N인데 100 µs 계단은 추력을 0.65–1.5 N 변화시키므로,
단일 샘플로는 인접한 계단을 구분할 수 없다. (Pi가 기록한) *지령*으로 구간을 나누고
안정된 부분을 평균하면 표준오차 ≈ 0.11 N를 얻는다 — 6–14 σ 분리이다. 진동 잡음이
자기상관되어 있으므로 오차 막대는 유효 표본 크기를 사용한다. 각 실행의
`steps.png`를 참고할 것.
