# HL_KU 시스템 설계

## 제어 책임 분리

Linux 주 컴퓨터는 위치추정, 인지, 경로추종, 미션 상태기계와 엔코더 없는
구동모터의 외부 속도 제어를 담당한다. NUCLEO는 조향각 PID, PWM 출력 제한,
통신 watchdog과 하드웨어 안전 입력을 담당한다.

주 컴퓨터가 NUCLEO에 보내는 값은 목표 속도가 아니라 최종 `drive_duty`,
`steering_angle_rad`, `brake`, `enable`이다. 구동모터 속도 피드백이 NUCLEO에
없기 때문에 속도 PID를 두 장치에 나누지 않는다.

## 좌표계

- `map`: 경기장 고정 ENU 좌표계. 원점은 매 실행마다 바꾸지 않는다.
- `base_link`: 차량 제어 기준점. 기본값은 후륜축 중심이다.
- `gnss_main`: UM982 ANT1 위상 중심.
- `lidar_link`, `camera_link`: 실측 정적 변환이 필요하다.

UM982의 진북 기준 시계방향 heading은 ROS ENU yaw로 변환한다.

```text
yaw_ros = normalize(pi/2 - heading_true + heading_mount_offset)
```

경로추종 기하 계산에서는 차량 왼쪽이 양의 곡률이지만, 설치된 T870/NUCLEO
조향 명령 규약은 왼쪽이 음수이고 오른쪽이 양수이다. 따라서 Pure Pursuit와
Stanley 모두 최종 조향 출력 경계에서 부호를 한 번 반전한다.

ANT1 위치에서 `base_link` 위치를 구할 때는 차량에서 안테나까지의 lever arm을
회전시킨 뒤 빼준다.

## Stanley 전진 경로 추종

GNSS ENU 전진 경로의 실제 조향 명령은 직선과 곡선 모두 Stanley 100%를 사용한다.
Pure Pursuit 결과와 경로 곡률은 Foxglove 및 현장 분석용 진단 토픽으로만 발행하며
`/planning/path_command` 조향에는 섞지 않는다. 전진 중
`/planning/tracker_mode`는 `STANLEY_100`, `/planning/pure_pursuit_weight`는 `0.0`이다.

Stanley 기준점은 후륜축 `base_link`에서 wheelbase 0.58 m 앞이고, RTK로 기록한
폴리라인의 짧은 구간 잡음을 그대로 조향에 넣지 않도록 1.0 m 기준선으로 경로
헤딩을 계산한다. 현재 만도 TUI의 직선·곡선·S 전환·S 탈출+직선 경로는 전진 구간만 허용한다.
주차용 후진 구간은 역방향 Stanley 동역학을 별도로 검증하기 전까지 기존 Pure
Pursuit를 유지한다.

## 무엔코더 구동 제어

중고속에서는 UM982 Doppler ground speed를 사용하고, 저속/후진/주차에서는
LiDAR odometry 또는 정적 물체까지의 상대 이동을 우선한다. 둘 다 없으면
위치를 추측하며 계속 주행하지 않고 감속 정지한다.

```text
drive_duty = feedforward(target_speed, battery_voltage)
           + PI(target_speed - measured_speed)
           + hill_hold_term
```

PWM feed-forward 표는 전진/후진과 배터리 전압별로 실측한다. PI 적분기는 제동,
센서 불량, 출력 포화 시 동결한다.

## 미션 구역

Course 07 원본 미션과 여덟 개 경로 조합은
[`course_07_mission.csv`](../ros2_ws/src/hl_ku_core/routes/course_07_mission.csv)에
함께 저장한다. `Route.load_csv()`는 선택한 `variant_id`만 읽고, 각 점의 `fsm_zone`,
이벤트, 방향을 런타임 경로 데이터로 보존한다. 경로 추종기가 현재 미션 구역과
가장 최근 이벤트를 `/planning/route_zone`, `/planning/route_event`로 발행한다.
GNSS 구역은 어떤 인지기를 믿을지
선택하고, 실제 동작은 카메라/LiDAR 관측으로 확정한다.

- `NORMAL`: GNSS 경로 중심, 카메라 차선 약결합
- `HILL`: CSV의 `HILL_STOP` 위치에서 멈추고 CSV `fsm_hold_sec`만큼 대기
- `S_OBSTACLE`: LiDAR 장애물을 진행방향 뒤로 1.3 m 연장해 로컬 우회 경로를
  만들고, 장애물 후단까지 횡오프셋을 유지한 뒤 전역경로로 재합류
- `TRAFFIC`: 정지선 1m 이내 정렬, 적색/UNKNOWN 정지
- `PERP_PARK`, `PARALLEL_PARK`: 보정된 전진/후진 maneuver 경로
- `DUMMY`: 앞범퍼 기준 3 m 이내의 LiDAR 경로 장애물에 회피 없이
  즉시 제동하고, 3.2초 완전정지 후 최신 스캔에서 경로가 비어야 재출발
- `END_LANE`: 카메라가 허용한 차선으로 제한된 lateral offset 생성

미션 구역/이벤트가 CSV에 있다는 것만으로 센서 조건이 충족되는 것은 아니다.
교통 신호와 장애물 상태는 각각 카메라/LiDAR 인지 결과가 결정한다. TUI의 짧은
건대 시험경로는 같은 CSV의 일부 거리 구간이며, 마지막 점은 정지점으로 변환한다.
FSM 런타임 구조와 데이터 컬럼 설명은
[`COURSE_07_MISSION_CSV.md`](COURSE_07_MISSION_CSV.md)를 따른다.

## 안전 우선순위

1. 물리 비상정지 및 NUCLEO fault
2. MCU 통신 watchdog
3. 실측 보정기록·경로 preflight
4. LiDAR 긴급거리
5. RTK/heading/pose/velocity freshness
6. 미션 정지 요청
7. 정상 경로 명령

`preflight_check`는 시스템 설정, 보정 기록, 실제 경로를 서로 대조하여
`/system/preflight_ok`와 상세 보고서를 주기적으로 발행한다. 안전 감독은 이 토픽이
false이거나 오래되면 자율 명령을 차단한다. 수동 차륜상승 보정 모드는 이 게이트를
우회하지만 duty/조향 제한, MCU feedback, watchdog은 그대로 적용된다.

프로세스 내부 freshness/deadman은 ROS 메시지 stamp가 아니라 단조 시계를 사용한다.
따라서 NTP 시각 보정이나 시스템 시간 역행이 오래된 명령을 다시 유효하게 만들지
않는다.

신규 미션 감점보다 도로 이탈 위험이 큰 경우 신규 미션을 보수적으로 포기한다.
단, 1분 무동작 DNF를 피하기 위해 각 복구 상태에는 제한시간을 둔다.
