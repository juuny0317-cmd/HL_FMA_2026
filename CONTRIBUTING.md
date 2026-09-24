# HL_KU 기여 가이드

이 저장소는 1/5 차량의 조향·구동기를 직접 제어합니다. 변경 사항은 기본 정지 상태와 실차 승인 절차를 유지해야 합니다.

## 작업 순서

1. `main`에서 목적이 드러나는 짧은 브랜치를 만듭니다.
2. 한 PR에는 하나의 검증 가능한 변경만 넣습니다.
3. 하드웨어 없이 실행 가능한 단위 테스트와 설정 검사를 먼저 수행합니다.
4. 실차 시험이 필요하면 차륜을 띄운 시험, 저속 지상주행 순서로 범위를 넓힙니다.
5. 시험 조건, 센서 연결 상태와 확인하지 못한 항목을 PR 본문에 적습니다.

## 안전 조건

- `drive_enabled: false`와 `route_calibrated: false`를 공개 기본값으로 유지합니다.
- 비상정지, watchdog, 명령 freshness 검사를 우회하지 않습니다.
- 보정하지 않은 조향각·PWM·GNSS datum 값을 임의의 합격값으로 기록하지 않습니다.
- 포트와 토픽 이름을 바꾸면 launch, 설정, 운용 문서를 함께 갱신합니다.
- 실차 동작을 바꾸는 PR에는 실패 시 정지 조건과 되돌리는 절차를 포함합니다.

## 공개 자료

계정 정보, NTRIP 자격 증명, 정확한 개인 위치, 원본 rosbag과 얼굴이 식별되는 자료는 커밋하지 않습니다. 재현에 필요한 결과는 익명화한 CSV, 짧은 로그 요약, 체크섬 또는 메타데이터를 제거한 이미지로 남깁니다.

## 기본 검사

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

현재 차량 펌웨어 변경은 `firmware/nucleo_h723zg/`의 통합 빌드와 ROS bridge가
사용하는 명령 범위를 함께 확인합니다. `firmware/nucleo_h743zi2/`는 이전 보드
이식 자료입니다. 실제 차량 시험 절차는 [COMMISSIONING.md](docs/COMMISSIONING.md),
인지 장치 시험은 [PERCEPTION_SETUP.md](docs/PERCEPTION_SETUP.md)를 따릅니다.

## PR 체크리스트

- [ ] 기본 상태에서 모터 출력이 비활성화되어 있다.
- [ ] 단위·범위·좌표계와 토픽 계약을 확인했다.
- [ ] 자동 검사를 실행하고 결과를 기록했다.
- [ ] 실차에서 확인하지 않은 내용을 명확히 구분했다.
- [ ] 민감 정보와 대용량 원본이 포함되지 않았다.
