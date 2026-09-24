# LiDAR 스캔 수신 후 이어받기 (새 Ubuntu PC / Codex)

이 문서를 새 PC의 Codex에 보여 주고 **1단계부터 진행**한다. 사용자는 RPLIDAR A3로
파악된 라이다의 배선을 끝내고 스캔 데이터 수신에 성공했다고 보고했다. 이 저장소를
작성한 PC에서는 새 PC의 ROS `/scan` 토픽, 장착 방향, 거리 정확도를 직접 확인하지
못했다. 따라서 배선이나 NUCLEO 펌웨어부터 다시 작업하지 말고, 현재 수신 상태를
먼저 확인한다.

## Codex에 맡길 작업과 안전 경계

> 이 저장소와 새 PC의 실제 ROS 상태를 확인한 뒤, LiDAR 스캔을 검증하고 전역경로
> 위의 장애물을 RViz에서 볼 수 있도록 연결해 줘. 기존 모터·GNSS·카메라 기능은
> 보존하고, 기본값 `drive_enabled=false`를 유지해. 실차를 움직이거나 기존
> NUCLEO 펌웨어를 덮어쓰지 마. 확인되지 않은 보정값을 `true`로 바꾸지 말고,
> 확인된 사실·수정한 파일·테스트 결과·남은 실차 검증을 구분해서 보고해 줘.

- 차량: HL Mando 1/5, UM982 듀얼 안테나 RTK GNSS, 전방 카메라, 앞범퍼 2D LiDAR,
  NUCLEO-H723ZG, MDD20A 구동, MD10C 조향. 구동 엔코더는 없고 조향각 피드백은
  있다. 앞범퍼 LiDAR만으로는 후방을 볼 수 없으므로 후진 회피를 허용하지 않는다.
- LiDAR 우회 코드는 2026-09-17에 GitHub `main`에 통합됐다. 다른 PC에 이미
  복제본이 있다면 `git status --short --branch`와 `git log -1`부터 확인하고 로컬
  변경을 보존한다. 새 복제본이면 다음을 사용한다.

  ```bash
  cd ~
  git clone https://github.com/juuny0317-cmd/HL_FMA_2026.git
  cd HL_FMA_2026
  ```

- 권장 환경은 Ubuntu 22.04 / ROS 2 Humble이다. 설치가 안 된 PC에서는
  [`SETUP_NEW_PC.md`](SETUP_NEW_PC.md)의 설치 스크립트가 시스템 패키지를
  변경한다는 점을 확인하고 실행한다. 이미 설치했다면 재설치하지 않는다.
- NTRIP 비밀번호, 원시 rosbag, 실차 보정 개인 파일은 GitHub에 올리지 않는다.

## 현재 코드 상태 — 연결과 지도 투영까지 구현됨

- `lidar_perception_node.py`는 `/scan`을 읽고
  `/perception/obstacle_distance_m`, `/perception/obstacle_in_path`,
  `/perception/avoidance_steering_rad`를 출력한다. 현재 코드는 스캔의 0도 방향이
  **차량 정면**과 일치한다고 가정하며 장착 yaw/좌표 변환을 적용하지 않는다.
- `local_detour_node.py`에는 가상 장애물 우회 미리보기와 실시간 후보 경로의
  틀이 있다. 실시간 입력은 `/planning/route_s` 및
  `/planning/obstacles_map` (`geometry_msgs/PoseArray`, `frame_id=map`)이고, 출력은
  `/planning/live/local_detour_candidate`와 `/planning/live/local_detour_status`다.
  `PoseArray`는 장애물 중심만 담고 반지름은 공통 파라미터
  `live_obstacle_radius_m`를 사용한다.
- `scan_obstacle_mapper_node.py`가 유효한 전방 `/scan` 군집을 GNSS pose 시각과
  검사한 뒤 `map` 좌표의 `/planning/obstacles_map`으로 바꾼다. 유효 스캔에
  장애물이 없을 때만 빈 배열을 발행하며, 스캔/pose/frame/timestamp가 잘못되면
  새 빈 배열을 발행하지 않는다. 상태는 `/planning/obstacle_mapper_status`다.
- 현재 차량은 NUCLEO ST-LINK VCP 하나를 `nucleo_serial_mux`로 나눠
  `/tmp/nucleo-control`은 조향·구동, `/tmp/nucleo-lidar`는 RPLIDAR A3에 쓴다.
  `run_mando_tui.sh`가 mux와 `rplidar_ros/rplidar_node`를 자동 실행한다.
- 장착 실측 전에는 `scan_obstacle_mapper.geometry_verified=false`,
  `local_detour.live_geometry_verified=false`,
  `mission_manager.enable_local_detour_steering=false`가 기본이다. 따라서 연결과
  상태는 볼 수 있지만 검증되지 않은 회피 조향은 차량 명령에 섞이지 않는다.
  자세한 경로 계약은 [`LOCAL_DETOUR_PREVIEW.md`](LOCAL_DETOUR_PREVIEW.md)에 있다.
- `system.yaml`의 전방 감지 8 m, 처리 범위 12 m, 약 5 m 앞 우회 시작,
  `corridor_half_width_m=1.70` 등은 실차 확인값이 아니다. 후보가 만들어졌다는
  사실만으로 도로 경계·차폭·제동거리 안전성이 입증되지 않는다.
- `course_07_vehicle.csv`는 미리보기·검토에 쓰인다. 실제 주행용 경로와 GNSS datum은
  현장에서 확인해야 하며 `course_template.csv`로 실차를 구동하지 않는다.
- TUI의 3번 `S자 전체`와 7번 `M01 S자 전체`는 현재 위치에 재배치할 때 주행점에
  `S_OBSTACLE` 미션을 붙이고 마지막 점만 `FINISH`로 유지한다. 따라서 아래 장착
  검증을 마치고 회피 조향을 켠 뒤에는 해당 두 시나리오에서만 S자 회피 게이트가
  열린다.

## 1. 새 PC에서 `/scan`을 확인

사용자가 확인한 것이 제조사 프로그램의 스캔인지 ROS `LaserScan` 토픽인지 먼저
구분한다. 다음 명령은 **구동 노드를 실행하지 않는다**.

```bash
source /opt/ros/humble/setup.bash
test -f ~/HL_KU/ros2_ws/install/setup.bash && source ~/HL_KU/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic info /scan -v
ros2 topic hz /scan
ros2 topic echo --once /scan
```

현재 차량에서는 먼저 아래 명령으로 mux와 A3만 실행한다. 이 명령은 액추에이터
bridge를 시작하지 않는다. 이미 mux가 실행 중이면 중복 실행하지 않는다.

```bash
source ~/HL_KU/operations/setup_env.bash
ros2 run hl_ku_core nucleo_serial_mux &
ros2 launch hl_ku_core rplidar_rviz.launch.py rviz:=false
```

기본 물리 포트는 현재 NUCLEO의 고정 by-id이고, 새 보드라면 mux의 `--port`로
새 by-id를 지정한다. A3 통신은 `/tmp/nucleo-lidar` 256000 bps, `/scan`,
`frame_id=laser`다. 유효한 `header.stamp`, 측정 거리, 실제 Hz와 끊김을 기록한다.
현재 NUCLEO에 이미 multiplex firmware가 올라가 있다는 전제이며 이 저장소의
호스트 설정 때문에 펌웨어를 다시 쓰지 않는다.

## 2. 모터 전원 OFF: 거리·방향·장착 위치 보정

RViz에서 `/scan`을 `LaserScan`으로 표시한다. 처음에는 Fixed Frame을 스캔 메시지의
`frame_id`와 같게 해도 된다. 넓은 판을 차량 정면 1/2/4/6 m, 좌·우에 두고 다음을
기록한다. `LaserScan`의 0도는 **센서 프레임 +x**이며, 그것이 차량 정면인지는
설치 상태로 검증해야 한다.

| 항목 | 새 PC에서 기록할 값 |
|---|---|
| ROS `/scan` 발행 명령·장치 경로·frame_id·실측 Hz | 2026-09-17 현재 PC: `nucleo_serial_mux` → `/tmp/nucleo-lidar` → `rplidar_ros/rplidar_node`, `laser`, 약 22.5 Hz, 1800 samples/scan |
| 정면 표적 1/2/4/6 m의 스캔 거리·오차 | 미측정 |
| 정면 0도 오차와 좌·우 각도 부호 | 미확인 |
| `base_link → lidar_link` x/y/z 및 roll/pitch/yaw | 미측정 |
| 라이다 원점 → 앞범퍼 거리, 차폭, 전방 가림·사각지대 | 미측정 |
| GNSS `map → base_link` 품질·timestamp | 미확인 |

2026-09-17 통신 확인에서는 A3 serial, firmware 1.27, hardware rev 6, health OK를
읽었다. 빈 공간 여부를 통제하지 않은 한 스캔에서 차량 전방 기준 후보가
`0.228 m / -7 deg`와 `0.612 m / +5 deg` 부근에 있었고 기존 corridor 판정은
앞범퍼 보정 후 약 `0.077 m`, obstacle=true를 출력했다. 이것이 차체 자체 반사인지
주변 물체인지는 아직 확정하지 않았다. 넓은 빈 공간에서 같은 점이 남는지 먼저
확인하고, 남으면 차체 마스크 또는 센서 위치 보정을 적용해야 한다. 이 확인 전에는
장애물 회피 조향을 켜지 않는다.

실측한 값만 `ros2_ws/src/hl_ku_core/config/calibration_record.yaml`과
`ros2_ws/src/hl_ku_core/config/system.yaml`에 반영한다.
센서 장착 yaw가 0이 아니면 TF 기록뿐 아니라 `lidar_perception_node.py`의
차량 전방 corridor 판정도 바로잡아야 한다. 카메라와의 정합·기록 순서는
[`PERCEPTION_SETUP.md`](PERCEPTION_SETUP.md)의 6~8절을 따른다.

## 3. 기존 LiDAR 장애물 판정 시험

드라이버가 `/scan`을 발행하는 별도 터미널을 유지한다. 아래는 LiDAR 인지 노드만
실행하며 액추에이터를 시작하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/HL_KU/ros2_ws/install/setup.bash
ros2 run hl_ku_core lidar_perception --ros-args \
  --params-file ~/HL_KU/ros2_ws/src/hl_ku_core/config/system.yaml
```

다른 터미널에서 아래 결과를 관찰한다.

```bash
ros2 topic echo /perception/obstacle_distance_m
ros2 topic echo /perception/obstacle_in_path
ros2 topic echo /perception/avoidance_steering_rad
```

넓은 장애물을 정면·좌·우·차량 경로 밖에 두고 거리와 참/거짓이 예상대로 변하는지
기록한다. 장애물 제거, 드라이버 중단, 손실·빈/잘못된 스캔도 시험한다. 토픽
끊김을 “장애물이 없음”으로 취급하면 안 된다. 이후 GNSS 위치와 `/scan` 및 판단
토픽을 함께 rosbag에 남긴다. 센서 손실 시 정지하는 전체 안전 동작은 구동 전원
없이 별도로 확인한다.

## 4. 다음 현장 작업: 지도 장애물과 우회 후보 검증

`map → base_link → laser`의 유효한 실측값과 동일한 GNSS datum을 준비한다.
`scan_obstacle_mapper`는 스캔 **취득 시각**과 GNSS pose timestamp 차이를 검사하고,
전방 점을 군집화해 `map` 장애물 중심으로 투영한다. 다음 순서로 확인한다.

1. 유효한 새 스캔에 장애물이 없을 때만 `frame_id=map`인 **빈**
   `/planning/obstacles_map` `PoseArray`를 발행한다. 스캔/TF/GNSS가 끊겼을 때
   신선한 빈 배열을 계속 보내 안전장치를 속이지 않는다.
2. 스캔 timestamp, TF 지연, `map` 위치 품질, 군집 크기·거리와 입력 freshness를
   검사한다. 변환 불가·오래된 입력이면 장애물 입력을 stale 상태로 두고 후보
   경로가 비워지는지 확인한다.
3. 기존 공통 장애물 반지름 계약의 한계를 문서화하고 실측 물체 크기·차폭·여유를
   반영한다. 미관측 영역이나 차선/도로 경계 밖을 “빈 공간”이라고 단정하지 않는다.
4. 액추에이터와 분리된 상태에서 RViz/Foxglove에 전역경로, 현재 pose,
   `/planning/obstacles_map`, `/planning/live/local_detour_candidate`를 함께 띄운다. 기존
   `drive_enabled=false`, `live_geometry_verified=false`,
   `enable_local_detour_steering=false`를 검증 없이 변경하지 않는다.
5. 테스트에 정상·빈 스캔, 좌우 배치, 각도 경계, 장애물 제거, TF/GNSS/스캔 손실,
   지도 좌표 투영, 경로 밖 장애물, 오래된 데이터가 포함되도록 한다.

실측값을 `system.yaml`의 `scan_obstacle_mapper.lidar_x_m`, `lidar_y_m`,
`lidar_yaw_deg`에 기록한 뒤 모터 전원 OFF 상태에서만 `geometry_verified=true`와
`live_geometry_verified=true`로 바꿔 후보를 확인한다. GNSS 오차, 제동거리,
차선/경기장 경계, 조향 한계, 장애물 크기와 전방 가림을 검증한 마지막 단계에서
`operations/mando_scenarios.yaml`의
`common.enable_local_detour_steering: true`를 별도로 켠다. 안전한 후보가 없으면
회피가 아니라 정지한다.

## 완료 보고 형식

새 PC의 Codex는 먼저 **확인된 사실 / 미확인 사항 / 지금 수정할 코드**를 나눠
짧게 보고한다. 이후 `/scan` Hz·frame·장치 포트, 거리/방향 보정표, 변경 파일,
`colcon build`·테스트 결과, RViz 또는 rosbag 확인 결과, 남은 실차 검증 항목을
요약한다. 실차 구동이나 GitHub `main` 병합은 이 문서만으로 승인되지 않는다.

참고: [Slamtec A3 사양](https://www.slamtec.com/en/lidar/a3spec),
[ROS 2 Humble `LaserScan` 정의](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg).
