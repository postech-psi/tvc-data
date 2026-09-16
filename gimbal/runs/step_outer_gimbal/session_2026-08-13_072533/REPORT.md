# TVC system-identification session report

- Status: **PASS**
- Session: `session_2026-08-13_072533`

## Results

### health

- PASS: `True`
- data usability: **USABLE**
    - 샘플 손실: 0/4000 (0.0000%)
    - CRC 오류: 0건
    - 프레이밍 바이트 손실: 0B / 144000B (0.00000%, budget 72B)
    - 이벤트: 0/0 (완전)
- g_mag_mean: `0.9990452492100226`

### step_outer_gimbal

- PASS: `True`
- data usability: **USABLE**
    - 샘플 손실: 0/22000 (0.0000%)
    - CRC 오류: 0건
    - 프레이밍 바이트 손실: 0B / 792720B (0.00000%, budget 79B)
    - 이벤트: 20/20 (완전)
- gimbal: `outer`
- direct_onset_ms: `9.961999999999804`
- rise_10_90_ms: `42.00000000000159`
- settling_2pct_ms: `280.46249999999964`
- bandwidth_hz: `9.8080794373844`
- peak_slew_dps: `234.76479516625955`
- tail_rate_rms_dps: `0.1810690612205531`
- recommend_chirp: `True`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
