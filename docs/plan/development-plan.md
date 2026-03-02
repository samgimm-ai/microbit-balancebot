# Self-Balancing Robot 개발 계획

## 프로젝트 개요

Micro:bit + Keyes KE0136 모터 드라이버 확장 보드 + MPU6050 센서를 사용하여
2륜 self-balancing robot을 구현한다. MicroPython으로 개발하며,
상보 필터(Complementary Filter)와 PID 제어 알고리즘을 적용한다.

## 하드웨어 구성

| 부품 | 역할 | 비고 |
|------|------|------|
| BBC micro:bit V2 | 메인 컨트롤러 | |
| Keyes KE0136 | TB6612FNG 모터 드라이버 확장 보드 | 70x58mm, 34.2g |
| MPU6050 (GY-521) | 6축 자이로/가속도 센서 (I2C) | 3PIN 인터페이스로 연결 |
| DC 기어 모터 x 2 | 구동 | 1:220 기어비, 단자: A1/A2, B1/B2 |
| micro:bit 전용 배터리 팩 | micro:bit 전원 | AAA x2 (3V), JST 커넥터 |
| 18650 Battery Shield | 모터 전원 | 18650 x1, 충전 기능, 5V 출력 → KE0136 |

## 소프트웨어 구조

```
src/
├── main.py       # 메인 제어 루프 (200Hz)
├── mpu6050.py    # MPU6050 I2C 드라이버 + 상보 필터
├── motor.py      # TB6612FNG 모터 제어 (좌/우 독립)
└── pid.py        # PID 컨트롤러 (Anti-windup)
```

## 제어 알고리즘

```
[MPU6050] → 가속도+자이로 raw 데이터
    ↓
[상보 필터] → 기울기 각도 (degree)
    angle = 0.98 * (angle + gyro * dt) + 0.02 * accel_angle
    ↓
[PID 컨트롤러] → 모터 출력값 (-1023 ~ +1023)
    output = Kp*error + Ki*integral + Kd*derivative
    ↓
[TB6612FNG] → 모터 정회전/역회전 + 속도
    ↓
[로봇 균형 유지]
```

## PID 초기 튜닝값

| 파라미터 | 값 | 설명 |
|---------|------|------|
| Kp | 100 | 비례 게인 |
| Ki | 76 | 적분 게인 |
| Kd | 52 | 미분 게인 |

버튼 A/B로 실시간 조정 가능:
- Button A: Kp → Ki → Kd 파라미터 선택 순환
- Button B: 선택된 파라미터 값 증가

## 안전 기능

- **넘어짐 감지**: ±45° 이상 기울어지면 모터 자동 정지
- **복구 감지**: 다시 ±35° 이내로 돌아오면 제어 재개
- **LED 상태 표시**: 기울기 방향, 넘어짐 상태 시각적 표시

## 검증 단계

1. **MPU6050 테스트**: I2C 연결 확인 → raw 데이터 출력 → 기울기 각도 확인
2. **모터 테스트**: 각 모터 정/역회전 및 속도 제어 확인
3. **PID 테스트**: 로봇을 손으로 기울이며 모터 반응 확인
4. **통합 테스트**: 실제 밸런싱 동작 확인 및 PID 파라미터 튜닝
