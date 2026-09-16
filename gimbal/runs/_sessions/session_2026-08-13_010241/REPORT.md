# TVC system-identification session report

- Status: **PASS**
- Session: `session_2026-08-13_010241`

## Results

### health

- PASS: `True`
- g_mag_mean: `1.0003450968354428`

### deadband_outer_gimbal

- PASS: `True`
- gimbal: `outer`
- travel_span_deg: `358.56568969694007`
- backlash_deg: `1.7637705877237266`
- backlash_us: `0.48737939077898274`
- local_gain_deg_per_us: `-3.6188862744168904`

### deadband_inner_gimbal

- PASS: `True`
- gimbal: `inner`
- travel_span_deg: `3.686409505960574`
- backlash_deg: `0.6622258881537684`
- backlash_us: `20.201768185056057`
- local_gain_deg_per_us: `0.032780590396222824`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
