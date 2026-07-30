# raw/

원시 취득 파일, 측정 날짜별 및 어느 시스템이 기록했는지에 따라 분류됨.
**여기 있는 어떤 것도 결코 수정하지 않는다.** 저장소의 다른 모든 것은 이
디렉터리로부터 재구축 가능하다.

```
raw/<YYYY-MM-DD>/pwm/       thrust_map_<epoch>.csv
                 loadcell/  data_<YYYYMMDD>_<HHMMSS>.csv
                 ulog/      log_<n>_<date>.ulg
                 ulog/_redundant/
```

날짜 폴더는 *측정* 날짜이며, 가능한 경우 파일 내부에서 읽는다 — git 체크아웃이
다시 쓰는 파일의 mtime이 아니다.

## pwm/ — Raspberry Pi, `pwm_thrust_map.py`, ~20-50 Hz

무엇이 지령되었고 배터리가 무엇을 했는가. 22개 열:

| 열 | 의미 |
|---|---|
| `t_epoch` | Pi 벽시계, **UTC**. 로드셀에 대한 병합 키. |
| `t_fc_us` | Pixhawk 부팅 클록. `.ulg` 파일에 날짜를 매긴다. |
| `phase` | `A<us>_B<us>`, 또는 무부하 창에 대해 `idle_pre` / `idle_post` |
| `a_cmd_us`, `b_cmd_us` | 지령 µs, 로터 A와 로터 B |
| `a_cmd_norm`, `b_cmd_norm` | 위와 같은 값을 0–1로 정규화 |
| `servo1_raw` … `servo8_raw` | 채널당 **측정된** 출력 µs (1 = A, 2 = B) |
| `voltage_v`, `current_a` | 팩 전압과 전류, QGC와 동일한 계산 |
| `sweep_idx` | 1부터 시작하는 반복 번호 |
| `esc1_rpm` … `esc4_rpm` | ESC 텔레메트리 RPM; 미지원 시 비어 있음 |

`idle_pre` / `idle_post` 행은 **두 로터 모두 1000 µs**이므로, 그 전압은 부하
판독값이 지니는 IR 새그가 없는 배터리의 충전 상태이다. 그 위상들이 존재하기 전에
기록된 파일은 대신 `.ulg`에서 충전 상태를 취한다.

## loadcell/ — 벤치 노트북, `gui.py`, 50 Hz

힘과 토크. 11개 열이며, 그중 **오직 이 네 개만 중요하다**:

| 열 | 의미 |
|---|---|
| `t_ms` | 자유 진행 STM32 가동 시간, epoch **아님** — 그래서 정렬 단계가 필요하다 |
| `Fz` | 수직 힘, 뉴턴. **추력 = −Fz** (음수로 기록됨). |
| `Tz` | 반작용 토크, N·m. 동축 쌍이 균형을 이룰 때 부호가 바뀐다. |
| `Fx`, `Fy`, `Tx`, `Ty` | 축 외(off-axis) 성분 |

`pwm`, `rpm`, `Current_mA`, `ADC_Current_mA`는 **설계상 죽어 있다** — Pi가 Pixhawk를
통해 모터를 구동하므로 스탠드는 스로틀을 결코 보지 못하고, RPM 센서가 없으며,
전류는 Pixhawk에서 온다. 이들은 무시할 것; PWM과 전압은 짝을 이루는 `pwm/`
파일에서 취할 것.

## ulog/ — Pixhawk SD 카드

PX4 로그로, 실행 사이의 유휴 간격을 포함해 세션 전체를 커버한다. Pi보다 낮은
속도(출력 10 Hz, 배터리 5 Hz)이고 사용 가능한 절대 시간을 담지 않으므로, 주요
기록이 아니라 교차 확인이자 더 오래된 실행을 위한 무부하 전압 출처이다.
Gitignore됨 — 크기가 크고 로컬에 남는다.

`_redundant/`는 고유한 내용이 없는 로그를 담는다: 같은 부팅의 잘린 두 번째
다운로드와, 모터가 결코 돌지 않은 채 기록된 로그. 삭제하지 않고 보관하되,
파이프라인은 건너뛴다. 공간이 필요하면 삭제해도 안전하다.
