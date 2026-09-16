# TVC system-identification session report

- Status: **FAIL: grid_both_gimbals 실패로 중단. 이 테스트를 건너뛰고 계속하려면 --keep-going, 특정 테스트만 하려면 --tests 를 쓰세요.**
- Session: `session_2026-08-13_060322`

## Results

### health

- PASS: `True`
- g_mag_mean: `0.998807780503413`

### grid_both_gimbals

- PASS: `False`

## Interpretation

- A 결과의 LUT를 각도 명령을 PWM으로 변환할 때 사용한다.
- B의 direct onset, 전체 step 파형과 bandwidth를 vehicle simulator에 넣는다.
- `recommend_chirp=true`일 때만 추가 chirp를 설계한다.
- 최종 비행 PID는 이 결과만으로 정하지 않고 추력·질량·CG·관성 모델과 합친다.
