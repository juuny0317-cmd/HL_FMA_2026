# 현재 차량 전역경로 저속 시험

이 절차는 기존 차량의 조향 위치제어 설정을 바꾸지 않고 GNSS 전역경로를 처음
따라가기 위한 순서다. 설정은 다음과 같이 고정되어 있다.

| 항목 | 시험값 |
|---|---:|
| 조향 부호 | 음수=좌, 양수=우 |
| 좌/중앙/우 ADC | 58744 / 31974 / 5111 |
| ROS 조향 범위 | -0.48 ~ +0.48 rad |
| NUCLEO 구동 명령 | 전진 8~12, 후진 금지 |
| GPS 경로 목표속도 상한 | 1.00 m/s |
| 속도 PI | 첫 시험에서는 `kp=ki=0` |
| 카메라 차선 보정 | 사용 안 함 |
| LiDAR 정지 | 반드시 사용, 기본 0.75 m |

`route_test.yaml`은 기존 `track_drive/nucleo_motor_bridge.py`의 UART 포트와 조향
ADC를 그대로 옮긴 프로필이다. UART ACK는 실제 조향각·배터리 전압 계측값이
아니므로, NUCLEO 내부의 ADC 폐루프와 watchdog은 아래 절차에서 따로 확인해야 한다.

## 0. 빌드 및 터미널 준비

모든 터미널에서 다음 순서로 overlay를 읽는다.

```bash
source /opt/ros/humble/setup.bash
source /path/to/planning_ws/install/setup.bash
cd $HOME/HL_KU/ros2_ws
source install/setup.bash
```

코드를 바꾼 뒤 한 번 빌드한다.

```bash
cd $HOME/HL_KU/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

기존 `track_drive`의 `nucleo_motor_bridge`와 HL_KU의
`nucleo_serial_bridge`를 동시에 실행하면 안 된다. 두 프로세스가 같은 UART를
열거나 번갈아 명령을 보내지 않는지 확인한다.
GNSS도 기존 `gnss_path_recorder`와 HL_KU의 `um982_serial`을 동시에 실행하지 않는다.

## 1. 전원 차단 상태에서 장치와 비상정지 확인

가장 먼저 구동모터 전원을 끄고 차륜을 지면에서 띄운다. 물리 비상정지 담당자도
차량 옆에 둔다.

```bash
ls -l /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_004700233434511834313937-if02
ls -l /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
groups | grep dialout
```

비상정지가 Linux나 NUCLEO 상태와 무관하게 모터 전원을 직접 끊는지 확인한다.
배터리 완충전압, 퓨즈 정격, 차량 ID, 시험자와 날짜를
`config/calibration_record.yaml`에 기록한다. 확인하지 않은 항목은 `true`로 바꾸지
않는다.

## 2. UART 통신만 확인 (`drive_enabled=false`)

```bash
ros2 launch hl_ku_core nucleo_manual_test.launch.py drive_enabled:=false
```

다른 터미널에서 확인한다.

```bash
ros2 topic hz /vehicle/feedback
ros2 topic echo --once /vehicle/feedback
ros2 topic echo /safety/status
```

`/vehicle/feedback`이 약 20 Hz이고 `fault_flags: 0`이어야 한다. 이 단계에서는
`/safety/status`에 `drive_enabled=false` 또는 `raw_stale`이 보여도 정상이며,
NUCLEO에는 `CMD:0,0`만 전송된다.

## 3. 차륜을 띄운 채 조향과 저출력 구동 확인

2단계 launch를 종료한 후 다음처럼 다시 실행한다.

```bash
ros2 launch hl_ku_core nucleo_manual_test.launch.py drive_enabled:=true
```

두 번째 터미널에서 키보드 노드를 실행한다.

```bash
ros2 run hl_ku_core keyboard_teleop --ros-args \
  --params-file src/hl_ku_core/config/system.yaml \
  --params-file src/hl_ku_core/config/route_test.yaml
```

시험 순서는 `C` 중앙 → `A` 한 번 좌측 → `C` → `D` 한 번 우측 → `C`다.
`A`는 음수 조향/큰 ADC 방향, `D`는 양수 조향/작은 ADC 방향이어야 한다. 한 번에
끝까지 보내지 말고 링크 간섭과 타이어 방향을 눈으로 확인한다. 그 다음 `W`를
짧게 눌러 명령 8에서 바퀴가 전진 방향으로 도는지만 확인한다. 첫 전역경로
시험에서는 `S`를 사용하지 않는다. `Space`는 정지, `X`는 재시작 전까지 풀리지
않는 키보드 비상정지다.

합격 후 `steering_limit_test`, `wheels_up_test`만 `true`로 기록한다.

## 4. 통신 watchdog과 물리 비상정지 시험

차륜을 계속 띄운 상태에서 `W`로 바퀴를 돌린 뒤 UART USB를 분리한다. 펌웨어의
0.30초 watchdog 이내에 구동과 조향 PWM이 0이 되어야 한다. 다음에는 같은 조건에서
물리 비상정지를 눌러 모터 전원이 즉시 끊기는지 확인한다.

둘 다 직접 확인한 경우에만 `serial_watchdog_test`, `estop_test`,
`physical_estop_verified`를 `true`로 기록한다. UART protocol/watchdog fault는
의도적으로 latch되므로 시험 후 bridge를 재시작해야 한다.

## 5. LiDAR 방향과 정지 시험

LiDAR driver를 먼저 실행한다.

```bash
ros2 launch xycar_lidar xycar_lidar.launch.py
ros2 topic hz /scan
```

그 상태에서 3단계의 `nucleo_manual_test.launch.py`와 키보드 노드도 실행한다. 수동
launch에는 LiDAR 인지/집계 노드가 포함되어 있어 저속 구동 중에도 비상거리 gate가
동작한다.

차량 정면에 표적을 두었을 때 LaserScan 0 rad인지, 왼쪽 표적의 각도가 양수인지
확인한다. 차체 폭과 LiDAR에서 앞 범퍼까지의 거리를 실측해 `system.yaml`과
`calibration_record.yaml`에 같은 값으로 넣는다. 저속 직진 중 0.75 m보다 앞에서
표적을 넣었을 때 `/safety/status`가 `lidar_emergency_distance`가 되고 정지하는지
확인한 뒤에만 LiDAR 방향 항목과 `lidar_emergency_test`를 승인한다.

## 6. 평지 직진·제동거리 시험

사람과 장애물이 없는 넓은 평지에서 명령 8부터 짧은 직진을 한다. 좌우로 흐르면
조향 ADC를 다시 보정하는 대신 먼저 차륜 정렬, 타이어 압력, 중앙 ADC 재현성을
확인한다. 명령 8, 10, 12에서 GNSS 평균속도를 각각 기록한다.

명령 12에서 여러 번 정지시켜 최악의 제동거리를 잰다. 그 거리가 0.75 m보다
작아야 현재 비상거리 설정을 사용할 수 있다. 합격 후 다음을 기록한다.

```yaml
traction:
  route_test_drive_command_limit: 12
  low_speed_braking_distance_m: <실측 최악값>
acceptance:
  low_speed_straight_test: true
```

실측 속도가 `route_test.yaml`의 0.10/0.25/0.45 m/s와 다르면
`forward_speeds_mps`만 실제 명령 8/10/12에 대응하도록 수정한다. 첫 경로시험 전에
PI gain은 계속 0으로 둔다.

## 7. RTK와 ENU datum 확정

`/gnss/status`에서 RTK FIXED, heading valid, 위성 15개 이상, HDOP 1.5 이하가
유지되는지 먼저 확인한다. NTRIP이 필요하면 계정이 든 별도 비공개 설정파일을
사용한다.

고정된 물리 마킹에 ANT1을 놓고 2분 측량한다.

```bash
ros2 launch hl_ku_core datum_survey.launch.py \
  duration_sec:=120.0 minimum_samples:=100
```

출력된 datum을 `system.yaml`과 `calibration_record.yaml` 양쪽에 동일하게 넣고
`datum_configured: true`로 바꾼다. 안테나 기준선 길이, 후륜축 중심에서 ANT1까지의
x/y 거리, heading mount offset도 실측해서 두 파일에 동일하게 기록한다. 차량을
0/90/180/270도 방향으로 놓아 ENU yaw 부호를 확인한다.

## 8. 전역경로 기록

경로 파일 위치를 정하고 기록 launch를 실행한다. 이 launch에는 모터 bridge가
없으므로 차량을 손으로 밀거나, 별도 터미널의 3단계 저속 수동시험으로 움직인다.

```bash
ros2 launch hl_ku_core route_record.launch.py \
  output_file:=$HOME/HL_KU/ros2_ws/src/hl_ku_core/routes/course_measured.csv
```

```bash
ros2 service call /route_recorder/start std_srvs/srv/Trigger '{}'
# 경로를 한 방향으로 끝까지 이동
ros2 service call /route_recorder/stop_and_save std_srvs/srv/Trigger '{}'
```

마지막 점은 자동으로 `FINISH`, 목표속도 0, 전진 방향으로 저장된다. 첫 시험
경로에는 `NORMAL`과 마지막 `FINISH`만 사용하고 후진점이나 미션 태그를 넣지 않는다.
RViz의 `/planning/reference_path`와 CSV를 확인해 경로가 벽·연석과 충분히 떨어져
있는지 확인한다. `calibration_record.yaml`에 파일명을 적고 검토가 끝난 뒤에만
`course.route_calibrated: true`로 바꾼다.

Enter로 성기게 기록한 경로는 측정된 선분을 바꾸지 않고 최대 0.5 m 간격으로
나눠 주행용 파일을 만든다.

```bash
ros2 run hl_ku_core route_prepare \
  $HOME/HL_KU/data/course_01_route.csv \
  $HOME/HL_KU/ros2_ws/src/hl_ku_core/routes/course_01_vehicle.csv \
  --maximum-spacing-m 0.50 \
  --base-to-antenna-x-m 0.802 \
  --base-to-antenna-y-m 0.0
```

입력 CSV는 손으로 기록한 ANT1 위상중심 궤적이다. 위 명령은 경로 접선 방향으로
0.802 m 뒤에 있는 후륜축 중심 궤적으로 변환한 다음 최대 0.5 m 간격으로 나눈다.
`FINISH`는 마지막 점이 가장 가까운 웨이포인트가 되었을 때가 아니라 실제 마지막
좌표의 `finish_tolerance_m` 반경 안에 들어왔을 때만 전달된다. 현재 course_01 시험
반경은 0.50 m다.

### 8.1 course_01 듀얼 안테나 시험

전방 주 위치 안테나를 ANT1, 뒤쪽 heading 안테나를 ANT2에 연결하는 것이 기본이다.
현재 실차 배선에서는 UM982가 보고한 헤딩이 차량 전방과 정확히 반대인 것이 RViz와
경로 시작 방향 비교로 확인되어 `heading_mount_offset_deg: 180.0`을 적용한다.
`/gnss/status`에서 `heading_valid: true`가 아니면 이 프로필은 주행을 허용하지 않는다.
실제 두 안테나 위상중심 사이 거리를 mm 단위로 측정해 `baseline_m`에 기록하고,
필요하면 수신기의 fixed-length heading 설정에도 같은 길이를 cm 단위로 넣는다.

손으로 기록한 궤적은 ANT1의 궤적이다. 현재 시험 프로필에는
후륜축 중심에서 ANT1까지 전방 0.802 m, 좌측 0 m를 적용한다. 좌우 오프셋이 실제로
있다면 `base_to_ant1_y_m`을 다시 실측해 설정과 기록 양쪽에 반영한 뒤 차량용 경로를
재생성한다.

## 9. 구동 금지 상태로 전체 경로 stack 확인

GPS-only 시험에서는 NUCLEO를 열지 않는 GNSS/경로 preview부터 시작한다.

현재 활성 경로는 `course_04_vehicle.csv`다. 22개 RTK FIX 원점을 후륜축 기준
124점으로 변환했으며 길이 54.765 m, 최대 점 간격 0.499 m다. 시작 후륜축 위치는
`(0.1072, 0.7948)`, 시작 진북 헤딩은 약 `187.69 deg`다.

```bash
ros2 launch hl_ku_core gps_route_test.launch.py \
  rviz:=true actuator_bridge_enabled:=false \
  route_calibrated:=false drive_enabled:=false
```

RViz의 고정 좌표계는 `map`이다. 노란 선은 주행 경로, 초록 화살표는 경로 시작
위치와 방향, 빨간 화살표는 종료 위치와 방향, 파란 화살표는 ANT1 레버암을 제거한
현재 후륜축 중심과 차량 방향이다. 첫 정렬에서는 파란 화살표를 초록 화살표에
겹치고 두 화살표 방향도 같게 맞춘다. 현재 수신기의 RMC 출력은 약 1 Hz이므로
GPS-only 프로필은 `velocity_timeout_sec: 1.5`를 사용한다.

`/gnss/status`의 `fix_type: 4`, `heading_valid: true`, `position_valid: true`를 먼저
확인한다. 출발점에 차를 놓았을 때 `/localization/gnss_pose`가 차량용 경로의 첫 점
`(0.1072, 0.7948)` 부근이고 `/planning/path_command`의 속도가 0이어야 한다.

LiDAR를 사용하는 일반 전역경로 시험은 LiDAR driver를 실행한 상태에서 구동을
잠근 채 시작한다.

```bash
ros2 launch hl_ku_core route_test.launch.py \
  route_file:=$HOME/HL_KU/ros2_ws/src/hl_ku_core/routes/course_measured.csv \
  route_calibrated:=false drive_enabled:=false
```

다른 터미널에서 다음을 모두 확인한다.

```bash
ros2 topic echo --once /system/preflight_report
ros2 topic echo /safety/status
ros2 topic echo /gnss/status
ros2 topic echo /planning/path_command
ros2 topic echo /vehicle/actuator_command_safe
```

`preflight_report`가 `OK`가 되기 전에는 주행을 시작하지 않는다. 보고서가 말하는
남은 실측·시험 항목만 채운다. safe 명령은 계속 `drive_duty: 0`, `brake: true`여야
한다.

이 상태에서 NTRIP 보정만 잠시 차단해 RTK FIXED가 해제되도록 한다. 수신기가 FIXED를
해제하면 `/localization/gnss_pose`가 끊겨야 하고, correction age가 2초를 넘거나 fix가
풀리면 `/safety/status`에 `rtk_correction_stale`, `rtk_not_fixed` 또는 `pose_stale`이
나타나야 한다. safe 명령이 계속 brake인지 확인한 후 보정을 복구한다. 이 시험까지
성공했을 때만 `acceptance.rtk_dropout_test: true`로 기록한다.

## 10. 최종 저속 전역경로 시험

차량을 경로 시작점에 같은 방향으로 놓고 비상정지 담당자와 안전거리를 확보한다.
그 다음에만 launch의 두 잠금을 풀고 미션을 arm한다.

```bash
ros2 launch hl_ku_core route_test.launch.py \
  route_file:=$HOME/HL_KU/ros2_ws/src/hl_ku_core/routes/course_measured.csv \
  route_calibrated:=true drive_enabled:=true
```

```bash
ros2 topic echo --once /system/preflight_report
ros2 service call /mission/arm std_srvs/srv/Trigger '{}'
```

arm 전까지는 움직이지 않아야 한다. 첫 주행은 5~10 m 직선에 가까운 짧은 경로로
하고, 횡오차·조향 포화·RTK dropout·LiDAR 정지를 확인한 뒤 길이와 곡률을
늘린다. 이상 시 물리 비상정지를 누른 뒤 launch를 종료한다. `Ctrl+C`만을 유일한
정지수단으로 사용하지 않는다.

### 10.1 T870 감독하 현장 시험

물리 비상정지를 실제로 확인했지만 정식 계측 기록이 아직 완성되지 않은 경우에는
정규 preflight를 거짓 값으로 채우지 않고 아래처럼 명시적인 감독하 시험을 사용한다.
이 모드는 정적 기록 검사만 생략하며 RTK FIX, GNSS heading/position/velocity,
보정정보 나이, NUCLEO feedback, 명령 timeout과 mission 상태 검사는 유지한다.
자동 arm하지 않는다. 현재 T870의 실측 정지마찰 때문에 명령 10에서는 바퀴가
돌지 않으므로, 감독하 시험에서 전진 명령이 있을 때만 기존 20/100의 1.5배인
구동 명령 30/100을 사용한다. 기본 경로 목표속도는 0.30 m/s다. 이 설정은
PWM 명령만 1.5배로 올리는 것이며 실제 차량 속도가 정확히 1.5배가 된다는 뜻은 아니다.

빌드가 끝난 상태라면 `ros2_ws`에서 아래 한 명령으로 GNSS, 경로 추종, 제어,
NUCLEO bridge와 RViz를 모두 실행한다. 시작 직후에는 정지 상태다.

```bash
./run_gnss_drive.sh
```

실행하면 같은 터미널에서 아래 두 값을 차례로 입력한다. 대괄호 값은 아무것도
입력하지 않고 Enter를 눌렀을 때 사용할 기본값이다.

```text
목표속도 m/s (0.01~1.00) [0.3]:
구동 PWM (1~100) [30]:
```

입력이 끝나면 전체 노드와 RViz가 실행되며, 이때도 차량은 정지 상태다. GNSS와
차량 방향을 확인한 다음 `Space`를 눌러 출발한다.

같은 터미널에는 약 2초마다 RTK 상태가 한국어 한 줄로 표시된다.

```text
NTRIP 보정 서버 연결 성공
RTK 연결: 정상 · 주행 가능 | 측위=RTK 고정(FIXED) | 위성=35개 | HDOP=0.50 | 보정나이=0.7초 | 위치=정상 | 헤딩=정상
```

`수렴 중`은 FLOAT 상태, `보정정보 지연`은 RTCM 보정나이 2초 초과,
`GNSS 메시지 끊김`은 수신기 토픽이 2초 이상 들어오지 않는 상태다. 출발 전에는
`정상 · 주행 가능`인지 확인한다.

- `Space`: 첫 입력은 출발, 다음 입력은 일시정지, 다시 입력하면 이어서 출발
- `X`: 래치 비상정지. 해제하려면 세션을 종료하고 다시 실행
- `Q`: 래치 정지를 건 뒤 전체 세션 종료
- `Ctrl+C`: 래치 정지를 요청하고 전체 세션 종료

다른 경로를 지정할 때는
`./run_gnss_drive.sh --route-file /절대/경로/route_vehicle.csv`를 사용한다.
RViz가 필요 없으면 `--no-rviz`를 덧붙인다. 물리 비상정지는 별도로 즉시 사용할
수 있어야 하며 키 입력을 유일한 비상정지 수단으로 간주하지 않는다.

자동 실행이나 반복 시험에서 입력 질문을 생략하려면 `--mission-speed`와
`--drive-command`를 명령에 직접 지정할 수도 있다. 예를 들어 목표 0.40 m/s,
PWM 40/100은 다음과 같다.

```bash
./run_gnss_drive.sh --mission-speed 0.40 --drive-command 40
```

`--mission-speed`는 CSV의 양수 순항속도를 런타임에 덮어쓰지만 FINISH의 속도 0은
유지한다. 현재 단일 실행기는 차량제어기 설정에 맞춰 1.00 m/s까지 받는다.
`--drive-command`는 NUCLEO 프로토콜 범위 1~100을 받으며, 같은 값으로 제어기 출력
상·하한과 NUCLEO 최종 명령 상한을 함께 설정한다.

현재 GPS 주행 프로필에는 다음 절대 제한이 적용된다.

- 미션 기본 목표속도: 0.30 m/s
- 차량제어기 전진 속도 상한: 1.00 m/s
- 기본 구동 출력: 전진 시 30/100, `--drive-command`로 1~100 지정 가능
- 조향각: -0.48~+0.48 rad(약 -27.5~+27.5도)
- PC 측 속도/조향 변화율(slew-rate) 제한: 없음

즉 기본 양수 속도 명령은 정지마찰 보상을 위해 30/100으로 바로 출력되고,
조향 목표는 20 Hz로 전송된다. 조향이 천천히 움직인다면 NUCLEO 내부 위치제어와
실제 기구 응답을 별도로 확인한다. 기계 안전범위를 확인하지 않은 채 조향 절대
한계를 높이지 않는다. PWM 30 초과는 이전 자율주행 기준보다 높은 출력이므로
차륜을 띄운 시험과 물리 비상정지 확인 후 단계적으로 올린다.

단일 실행기의 기본값과 같은 수동 launch 명령은 다음과 같다.

```bash
ros2 launch hl_ku_core gps_route_test.launch.py \
  rviz:=true actuator_bridge_enabled:=true \
  route_calibrated:=true drive_enabled:=true \
  require_preflight:=false mission_speed_mps:=0.30 \
  maximum_drive_duty:=0.30 minimum_forward_duty:=0.30 \
  maximum_drive_command:=30
```

수동 launch에서 목표 0.40 m/s와 PWM 40을 지정하려면 위 명령의 마지막 네 인자를
다음처럼 바꾼다. `mission_speed_mps`는 경로 추종기의 CSV 순항속도와 미션 상한에
동시에 적용된다.

```bash
require_preflight:=false mission_speed_mps:=0.40 \
maximum_drive_duty:=0.40 minimum_forward_duty:=0.40 \
maximum_drive_command:=40
```

`/safety/status`가 `mission_init`만 표시하고 차량이 경로 시작점에서 같은 방향인지
RViz로 확인한 뒤에만 `Space`를 누른다. 수동 launch를 사용했다면 기존처럼
`/mission/arm`을 호출할 수 있다. 다른 차단 사유가 하나라도 있으면 출발하지 않는다.
물리 비상정지 담당자는 시험 내내 차량과 안전거리를 유지한다.
UM982의 약 1 Hz RMC 주기 사이에서 구동이 끊기지 않도록 이 프로필은 serial과
velocity selector의 GNSS timeout을 모두 1.5초로 사용한다.

경로 추종기의 실제 제어기와 비교용 출력을 별도 터미널에서 확인한다.
전진 경로에서는 직선과 곡선 모두 `STANLEY_100`과 `0.0`이 보여야 한다.
Pure Pursuit 조향은 비교용으로만 발행되고 실제 명령에는 반영되지 않는다.
조향 토픽은 T870 규약인 음수=좌, 양수=우이다.

```bash
ros2 topic echo /planning/tracker_mode
ros2 topic echo /planning/pure_pursuit_weight
ros2 topic echo /planning/path_curvature_per_m
ros2 topic echo /planning/pure_pursuit_steering_rad
ros2 topic echo /planning/stanley_steering_rad
ros2 topic echo /planning/cross_track_error_m
ros2 topic echo /planning/heading_error_rad
```
