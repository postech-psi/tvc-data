# TVC system-identification session report

- Status: **FAIL: mapping_inner_gimbal 실패로 중단 (late_samples 3747 > 허용 2811.0 (전체 281099개 중 1.0%, jitter_p99=32767us)). 이 테스트를 건너뛰고 계속하려면 --keep-going, 특정 테스트만 하려면 --tests 를 쓰세요.**
- Session: `session_2026-08-13_061200`

## Results

### health

- PASS: `True`
- g_mag_mean: `0.9996614372594436`

### mapping_inner_gimbal

- PASS: `False`
- error: `mapping_inner_gimbal 분석 FAIL: late_samples 3747 > 허용 2811.0 (전체 281099개 중 1.0%, jitter_p99=32767us)`
- fail_reasons:
    - late_samples 3747 > 허용 2811.0 (전체 281099개 중 1.0%, jitter_p99=32767us)
- gimbal: `inner`
- gain_deg_per_us: `0.0336102854447024`
- neutral_us: `1513.7112078141113`
- angle_min_deg: `-6.500563599297155`
- angle_max_deg: `15.730660085933115`
- max_abs_angle_deg: `15.730660085933115`
- travel_span_deg: `22.23122368523027`
- hysteresis_max_deg: `0.6156605421746093`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
