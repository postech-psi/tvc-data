# TVC system-identification session report

- Status: **PASS**
- Session: `session_2026-08-13_053707`

## Results

### health

- PASS: `True`
- g_mag_mean: `0.9988813837685738`

### chirp_outer_gimbal

- PASS: `True`
- gimbal: `outer`
- bandwidth_3db_hz: `9.00003600015152`
- phase_delay_ms: `73.52543458007824`
- coherent_band_hi_hz: `11.500046000193608`

### chirp_inner_gimbal

- PASS: `True`
- gimbal: `inner`
- bandwidth_3db_hz: `13.000052000218862`
- phase_delay_ms: `55.0431025311813`
- coherent_band_hi_hz: `17.50007000029462`

### deadband_outer_gimbal

- PASS: `True`
- gimbal: `outer`
- travel_span_deg: `6.841827010350968`
- backlash_deg: `0.5459166253938288`
- backlash_us: `9.445097444504547`
- local_gain_deg_per_us: `0.057798940519291325`

### deadband_inner_gimbal

- PASS: `True`
- gimbal: `inner`
- travel_span_deg: `4.796979364554945`
- backlash_deg: `0.4072943465905965`
- backlash_us: `10.325443002313834`
- local_gain_deg_per_us: `0.03944570189379048`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
