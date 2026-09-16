# TVC system-identification session report

- Status: **FAIL: A INNER capture 실패: framing_bytes_discarded=9**
- Session: `session_2026-08-12_224338`

## Results

### health

- PASS: `True`
- g_mag_mean: `1.0003127905204061`

### mapping_outer_gimbal

- PASS: `True`
- gimbal: `outer`
- gain_deg_per_us: `0.0603803543010005`
- neutral_us: `1523.578014989178`
- angle_min_deg: `-9.318912581159404`
- angle_max_deg: `7.926599314750538`
- max_abs_angle_deg: `9.318912581159404`
- travel_span_deg: `17.245511895909942`
- hysteresis_max_deg: `0.4310631312104558`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
