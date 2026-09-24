# HL_KU 차량 운용 명령어

> 운영 폴더: `$HOME/HL_KU/operations`<br>
> 실제 저장소: `$HOME/HL_KU`<br>
> 원본 RTK 데이터: `$HOME/HL_KU/data`

---

## 0. 새 터미널 공통 환경설정

직접 `ros2` 명령을 실행하는 터미널마다 한 번 실행한다.

```bash
cd $HOME/HL_KU/operations
source ./setup_env.bash
```

| 경로 변수 | 내용 |
| --- | --- |
| `$HLKU_ROOT` | HL_KU 저장소 |
| `$HLKU_WS` | ROS 2 워크스페이스 |
| `$HLKU_NODES` | Python 노드 |
| `$HLKU_LAUNCH` | launch 파일 |
| `$HLKU_CONFIG` | YAML 설정 |
| `$HLKU_ROUTES` | 차량용 경로 CSV |
| `$HLKU_RVIZ` | RViz 설정 |
| `$HLKU_DATA` | 원본 RTK·datum·bag 데이터 |

---

## 1. 빌드

```bash
cd $HOME/HL_KU/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 2. WASD 키보드 주행

> 차륜을 지면에서 띄우고 시작한다. GNSS 주행과 동시에 실행하지 않는다.

```bash
cd $HOME/HL_KU/operations
./run_finger_drive.sh
```

| 키 | 기능 |
| --- | --- |
| `W` / `S` | 기본 10 PWM씩 누적해 전진·정지·후진 이동 |
| `A` / `D` | 0.096 rad(약 5.5°)씩 좌/우 조향하고 현재 각도 고정 |
| `C` | 조향 중앙 |
| `V` | TUI에서 키당 PWM 증감값 변경 |
| `Space` | 정지 |
| `X` | 비상정지 고정 |
| `Q` | 종료 |

---

## 3. GNSS/RTK 연결 확인

### 터미널 1: UM982와 NTRIP

```bash
cd $HOME/HL_KU/operations
./rtk_check.sh
```

### 터미널 2: GNSS 상태

```bash
cd $HOME/HL_KU/operations
source ./setup_env.bash
ros2 topic echo /gnss/status hl_ku_interfaces/msg/GnssStatus \
  | grep --line-buffered -E '^fix_type:|^heading_valid:|^position_valid:|^velocity_valid:|^satellites:|^hdop:|^correction_age_sec:|^nmea_checksum_valid:'
```

정상 주행 조건:

```text
fix_type: 4
heading_valid: true
position_valid: true
velocity_valid: true
satellites: 15 이상 권장
hdop: 1.5 이하 권장
correction_age_sec: 0~2초
nmea_checksum_valid: true
```

주기와 USB 확인:

```bash
ros2 topic hz /gnss/status
ros2 topic hz /gnss/rtcm
ls -l /dev/serial/by-id
```

---

## 4. GNSS 웨이포인트 주행

> `rtk_check.sh`, 수동주행, 기록기 등 GNSS 또는 NUCLEO 포트를 사용하는
> 프로그램을 먼저 종료한다. 차량 후륜축을 경로 시작점에 정렬한다.

### course_06

```bash
cd $HOME/HL_KU/operations
./run_gnss_drive.sh \
  --route-file $HOME/HL_KU/operations/routes/course_06_vehicle.csv \
  --mission-speed 0.30 \
  --drive-command 30
```

실행 후 `Space`는 출발/일시정지/재출발, `X`는 비상정지, `Q`는 종료다.

### course_07

> 실행 전에 `gps_only_route.yaml`의 datum이 반드시 `course_07_datum.yaml`과
> 같은지 확인한다. CSV만 변경하고 course_06 datum을 사용하면 안 된다.

```bash
cd $HOME/HL_KU/operations
./run_gnss_drive.sh \
  --route-file $HOME/HL_KU/operations/routes/course_07_vehicle.csv \
  --mission-speed 0.30 \
  --drive-command 30
```

상태 확인:

```bash
ros2 topic echo --once /mission/status hl_ku_interfaces/msg/MissionStatus
ros2 topic echo --once /safety/status std_msgs/msg/String
ros2 node list | grep nucleo_serial_bridge
ros2 topic hz /vehicle/feedback
```

---

## 5. 웨이포인트 기록

아래 예시는 `course_07`을 기록한다.

### 터미널 1: GNSS/RTK

```bash
cd $HOME/HL_KU/operations
./rtk_check.sh
```

### 터미널 2: 기록기

```bash
cd $HOME/HL_KU/operations
./record_waypoints.sh course_07
```

기록기 키: `Enter` 저장, `U`/`Backspace` 마지막 점 취소, `P` 상태,
`Q` 저장 후 종료.

### 터미널 3: 기록용 RViz

```bash
cd $HOME/HL_KU/operations
./rviz_waypoint_recording.sh
```

### 터미널 4: 차량 이동

```bash
cd $HOME/HL_KU/operations
./run_finger_drive.sh
```

기록 결과:

```text
$HOME/HL_KU/data/course_07_wgs84.csv
$HOME/HL_KU/data/course_07_route.csv
$HOME/HL_KU/data/course_07_datum.yaml
```

---

## 6. ANT1 경로를 차량 후륜축 경로로 변환

> 현재 main의 `route_prepare`는 경로를 직선 분할하며 smoothing하지 않는다.
> 기존 출력 파일을 보존하려면 출력 이름을 새로 지정한다.

```bash
cd $HOME/HL_KU/operations
source ./setup_env.bash
ros2 run hl_ku_core route_prepare \
  "$HLKU_DATA/course_07_route.csv" \
  "$HLKU_ROUTES/course_07_vehicle.csv" \
  --maximum-spacing-m 0.30 \
  --base-to-antenna-x-m 0.802 \
  --base-to-antenna-y-m 0.0
```

변환 후 `course_07_datum.yaml` 값을 `gps_only_route.yaml`에 반영하고 RViz에서
현재 차량 위치, 시작점, 종료점을 확인한다.

---

## 7. rosbag 전체 기록

### 기록

```bash
cd $HOME/HL_KU/operations
./record_all_topics.sh
```

이름 지정:

```bash
./record_all_topics.sh course_07_test_01
```

### bag 확인

```bash
cd $HOME/HL_KU/operations
source ./setup_env.bash
ros2 bag info "$HLKU_DATA/bags/저장된_bag_폴더명"
```

### bag 재생

> 저장된 제어 명령이 다시 발행될 수 있으므로 모터 전원을 끄고 NUCLEO USB를
> 분리한 상태에서 실행한다.

```bash
cd $HOME/HL_KU/operations
source ./setup_env.bash
ros2 bag play "$HLKU_DATA/bags/저장된_bag_폴더명" --clock 100 --start-paused
```

---

## 8. RViz와 제어 그래프

경로 정합 RViz:

```bash
cd $HOME/HL_KU/operations
./rviz_route_alignment.sh
```

추종 오차와 조향 출력:

```bash
cd $HOME/HL_KU/operations
source ./setup_env.bash
ros2 run rqt_plot rqt_plot \
  /planning/cross_track_error_m/data \
  /planning/heading_error_rad/data \
  /planning/stanley_steering_rad/data \
  /planning/pure_pursuit_steering_rad/data
```

---

## 9. YOLO 학습 영상 녹화

`01` 부분만 촬영 번호에 맞게 변경한다.

```bash
mkdir -p $HOME/HL_KU/data/yolo_dataset/videos
ffmpeg -f v4l2 -thread_queue_size 1024 \
  -input_format mjpeg -video_size 800x600 -framerate 30 \
  -i /dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0 \
  -an -c:v copy \
  "$HOME/HL_KU/data/yolo_dataset/videos/c270_recording_01_$(date +%Y%m%d_%H%M%S).mkv"
```

결과 확인:

```bash
ls -lh $HOME/HL_KU/data/yolo_dataset/videos
```
