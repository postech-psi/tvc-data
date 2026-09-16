# TVC system-identification session report

- Status: **PASS**
- Session: `session_2026-08-13_004259`

## Results

### health

- PASS: `True`
- g_mag_mean: `1.0006325043771056`

### mapping_inner_gimbal

- PASS: `True`
- gimbal: `inner`
- gain_deg_per_us: `0.03325883661114552`
- neutral_us: `1499.6852004449177`
- angle_min_deg: `-3.478416809200969`
- angle_max_deg: `354.8717891578463`
- max_abs_angle_deg: `354.8717891578463`
- travel_span_deg: `358.35020596704726`
- hysteresis_max_deg: `323.8923144281323`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
