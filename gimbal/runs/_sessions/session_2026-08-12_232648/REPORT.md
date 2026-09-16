# TVC system-identification session report

- Status: **PASS**
- Session: `session_2026-08-12_232648`

## Results

### health

- PASS: `True`
- g_mag_mean: `0.9998813277335187`

### step_outer_gimbal

- PASS: `True`
- gimbal: `outer`
- direct_onset_ms: `9.97049999999966`
- rise_10_90_ms: `41.499999999998984`
- settling_2pct_ms: `563.4705000000001`
- bandwidth_hz: `10.387698840984356`
- peak_slew_dps: `258.7649435988726`
- tail_rate_rms_dps: `0.1885091937884214`
- recommend_chirp: `True`

### step_inner_gimbal

- PASS: `True`
- gimbal: `inner`
- direct_onset_ms: `8.969999999997924`
- rise_10_90_ms: `34.00099999999995`
- settling_2pct_ms: `363.9700000000019`
- bandwidth_hz: `12.743646854193242`
- peak_slew_dps: `401.23812417854236`
- tail_rate_rms_dps: `0.19308760702575845`
- recommend_chirp: `True`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
