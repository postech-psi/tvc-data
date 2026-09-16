# TVC system-identification session report

- Status: **FAIL: mapping_inner_gimbal 실패로 중단. 이 테스트를 건너뛰고 계속하려면 --keep-going, 특정 테스트만 하려면 --tests 를 쓰세요.**
- Session: `session_2026-08-13_055016`

## Results

### health

- PASS: `True`
- g_mag_mean: `0.9991456600557175`

### mapping_outer_gimbal

- PASS: `True`
- gimbal: `outer`
- gain_deg_per_us: `0.06026207930871887`
- neutral_us: `1524.4445573987216`
- angle_min_deg: `-9.07009780947585`
- angle_max_deg: `8.229583529741427`
- max_abs_angle_deg: `9.07009780947585`
- travel_span_deg: `17.29968133921728`
- hysteresis_max_deg: `0.5723868308103399`

### mapping_inner_gimbal

- PASS: `False`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
