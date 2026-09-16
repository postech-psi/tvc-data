# TVC system-identification session report

- Status: **PASS**
- Session: `session_2026-08-13_072208`

## Results

### health

- PASS: `True`
- data usability: **USABLE**
    - 샘플 손실: 0/4000 (0.0000%)
    - CRC 오류: 0건
    - 프레이밍 바이트 손실: 0B / 144000B (0.00000%, budget 72B)
    - 이벤트: 0/0 (완전)
- g_mag_mean: `0.9995526334810324`

### step_inner_gimbal

- PASS: `True`
- data usability: **USABLE**
    - 샘플 손실: 0/22000 (0.0000%)
    - CRC 오류: 0건
    - 프레이밍 바이트 손실: 0B / 792720B (0.00000%, budget 79B)
    - 이벤트: 20/20 (완전)
- gimbal: `inner`
- direct_onset_ms: `9.96200000000158`
- rise_10_90_ms: `32.00000000000003`
- settling_2pct_ms: `337.9620000000001`
- bandwidth_hz: `13.943020249725393`
- peak_slew_dps: `402.78405522356366`
- tail_rate_rms_dps: `0.2012946851520176`
- recommend_chirp: `True`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
