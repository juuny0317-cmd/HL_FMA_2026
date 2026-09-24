# HL_KU 최초 설정·정합·검증 절차

이 문서는 새 차량 한 대를 처음 조립한 시점부터 자율주행 허가까지의 순서다.
`system.yaml`은 실행값, `calibration_record.yaml`은 측정 근거의 원본 기록으로
사용한다. 숫자를 추정해서 채우지 말고 측정일과 담당자를 같이 기록한다.

기본 코드는 다음 네 잠금이 풀리지 않으면 움직이지 않는다.

1. GNSS datum이 설정되어 RTK FIXED 위치와 듀얼안테나 heading이 출력될 것
2. `calibration_record.yaml`의 필수 실측·안전시험이 preflight를 통과할 것
3. 실제 경로 검증 후 `route_calibrated:=true`를 줄 것
4. 마지막으로 `drive_enabled:=true`를 줄 것

## 1. 전원과 기계 안전부터 확정

### 1.1 대회 제한 및 전원

- 차량 전체 크기와 센서 포함 높이를 실측해 규정 제한과 비교한다.
- 구동 전압은 규정의 24 V 이하 조건을 만족해야 한다. 6S LiPo는 완충 시
  25.2 V이므로 그대로 “24 V”라고 간주하지 않는다. 대회 측에 허용 기준을
  서면 확인하거나, 허용 범위가 분명한 전원 구성을 사용한다.
- 배터리 바로 뒤에 정격 퓨즈, 주전원 스위치, 물리 비상정지 contactor를 둔다.
  비상정지는 Nucleo 프로그램이 멈춰도 모터 전원을 직접 끊어야 한다.
- 컴퓨터/센서/Nucleo 전원은 모터 전원과 분리된 DC/DC를 권장한다. 신호 GND는
  한 점에서 공통화하고, 모터선과 GNSS 안테나선을 떨어뜨린다.
- Nucleo ADC에는 3.3 V를 넘는 신호를 직접 넣지 않는다. 배터리 전압은 분압과
  입력 보호 후 측정한다.

### 1.2 구동기 역할

- MDD20A: 구동모터 PWM/DIR. PWM Low는 동적 제동이지만, 정지 경사에서
  지속적인 정지토크를 보장하는 기계식 브레이크가 아니다.
- MD10C: 조향모터 PWM/DIR. 조향 포텐셔미터/센서 피드백은 Nucleo ADC로만
  폐루프 제어한다.
- 구동모터에는 엔코더가 없으므로 Nucleo에서 속도 PID를 만들지 않는다.
  Linux 컴퓨터가 GNSS 속도와 저속 LiDAR odometry로 외부 속도제어를 한다.

처음에는 구동 퓨즈를 빼고 조향만, 그 다음 차륜을 지면에서 띄우고 구동을
시험한다. 좌/우 및 전/후진 극성이 모두 확인되기 전에는 지상 시험을 하지 않는다.

## 2. PC, USB, 네트워크 설정

ROS 2 Humble PC에서 빌드한다.

```bash
cd HL_FMA_2026/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

UM982 USB 포트를 연결한 뒤 장치와 속성을 확인한다.

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
udevadm info --attribute-walk --name=/dev/ttyUSB0
```

보드의 실제 `idVendor`, `idProduct`, 가능하면 고유 serial을 사용해 udev 규칙으로
`/dev/ttyGNSS` 심볼릭 링크를 만든다. 여러 센서를 단순히 `ttyUSB0` 번호로 구별하지
않는다. 사용자를 `dialout` 그룹에 추가한 뒤 다시 로그인한다.

```bash
sudo usermod -aG dialout "$USER"
```

PC Ethernet은 `192.168.10.10/24`, Nucleo는 `192.168.10.20/24`를 예시로 쓴다.
일반 인터넷/NTRIP 망과 모터제어 망은 가능하면 NIC를 분리한다. 다음을 확인한다.

```bash
ping 192.168.10.20
ros2 topic hz /vehicle/feedback
ros2 topic echo /safety/status
```

MCU 명령은 50 Hz, watchdog은 100 ms를 권장한다. 케이블을 뽑았을 때 100 ms 안에
구동 PWM=0/브레이크가 되는지 차륜을 띄운 상태에서 검증한다.

## 3. UM982 듀얼안테나 설치

### 3.1 안테나 기계 배치

- ANT1은 주 위치 안테나, ANT2는 heading용 보조 안테나다.
- UM982 heading은 진북에서 **ANT2에서 ANT1으로 향하는 기준선**까지 시계방향
  각도다. 따라서 ANT1을 차량 앞, ANT2를 뒤에 두면 차량 전방과 자연스럽게 맞는다.
- 두 안테나는 같은 모델, 같은 RF 케이블 길이, 같은 높이와 수평면에 단단히
  고정하고 금속 접지판 조건도 같게 한다.
- 기준선은 차체 안에서 가능한 길게 한다. 제조사 사양은 1 m 기준 약 0.1도이며,
  더 짧아지면 방위 오차가 커진다. 실제 ANT1/ANT2 위상중심 사이를 mm 단위로
  재서 `baseline_m`에 기록한다.
- 안테나 위 하늘을 카메라, LiDAR, 컴퓨터, 차체 구조물이 가리지 않게 한다.
  전원 변환기와 모터 케이블에서 최대한 멀리 둔다.

### 3.2 수신기 포트와 출력 설정

판매 보드의 USB-UART가 UM982의 COM1/2/3 중 어디에 연결되는지는 보드마다 다를
수 있다. uPrecise나 serial terminal에서 `VERSION`, `CONFIG`, `MODE`를 보내 먼저
확인한다. 아래의 `COM3`은 확인된 실제 포트명으로 바꾼다.

```text
UNLOG COM3
MODE ROVER AUTOMOTIVE
CONFIG HEADING FIXLENGTH
CONFIG HEADING LENGTH <실측_baseline_cm> <허용오차_cm>
CONFIG HEADING RELIABILITY 3
CONFIG COM3 115200 8 N 1
GPGGA COM3 0.05
GPRMC COM3 0.05
GPTHS COM3 0.05
SAVECONFIG
```

`0.05`는 20 Hz다. 현재 parser는 실제 출력되는 `GNGGA`, `GNRMC`, `GNTHS`를
talker ID와 무관하게 읽는다. `MODE HEADING2`는 별도 수신기 사이 moving-baseline
모드이므로 이 단일 UM982 듀얼안테나 차량에는 사용하지 않는다. baudrate를 바꾸면
현재 serial 연결도 같은 속도로 다시 열어야 한다.

자동 명령 전송은 기본적으로 꺼져 있다. 명령 응답과 재부팅 후 출력을 터미널에서
검증한 후에만 `startup_commands`를 사용한다. 근거 문서는 제조사의
[UM982 제품 페이지](https://en.unicorecomm.com/products/um982/),
[UM982 User Manual R1.7](https://en.unicorecomm.com/uploads/file/UM982_User%20Manual_EN_R1.7.pdf),
N4 High Precision Products Commands Reference다.

### 3.3 RTK/NTRIP 설정

RTK는 rover인 UM982가 가까운 기준국의 RTCM 3.x 보정 데이터를 계속 받아야 한다.
인터넷이 된다고 자동으로 RTK가 되는 것은 아니다. NTRIP 공급자에게서 다음 값을
받는다.

- caster host/domain, port, mountpoint
- ID/password와 TLS 사용 여부
- mountpoint의 기준국 위치/지원 위성·신호/RTCM 형식

`system.yaml`의 `ntrip_client`에 입력하고 `enabled: true`로 바꾼다. 계정이 들어간
파일은 공개 저장소에 commit하지 않는다. 데이터 흐름은 다음과 같다.

```text
NTRIP caster -> PC ntrip_client -> /gnss/rtcm -> UM982 serial RX
UM982 GGA -> PC -> NTRIP caster의 rover 위치 갱신
```

UM982는 RTCM 형식을 자동 인식한다. 개활지에서 다음 합격 기준을 로그로 확인한다.

- GGA quality 4 / `/gnss/status.fix_type == 4`가 대부분 유지
- `heading_valid == true`, GNTHS mode가 `V`가 아님
- correction age가 보통 2초 이하, HDOP 1.5 이하를 목표
- 정지 2분 동안 큰 점프가 없고, 안테나를 가리지 않았을 때 재고정이 반복 가능
- NTRIP 단절 시 `/localization/gnss_pose`가 중단되고 안전 감독이 즉시 구동 금지

실내, 건물 바로 옆, 나무 아래에서는 FIXED가 잠깐 보이더라도 다중경로 오차를
신뢰하지 않는다.

## 4. ENU 기준점, heading, 레버암 정합

### 4.1 고정 datum 측량

매 실행 때 첫 GNSS 값을 원점으로 삼으면 전역경로가 움직인다. 경기장에 재현 가능한
물리 마킹을 하나 정하고 ANT1을 그 점에 정확히 둔 뒤, 완전 개활지 RTK FIXED를
2분 동안 평균한다.

```bash
ros2 run hl_ku_core gnss_survey --ros-args \
  -p duration_sec:=120.0 -p minimum_samples:=100
```

출력된 위도/경도/고도를 `system.yaml`의 `datum_*`에 넣고
`datum_configured: true`로 변경한다. 다른 날 같은 마킹에서 ENU가 허용 오차 안에
재현되는지 확인한다.

### 4.2 좌표축과 레버암

`base_link`는 후륜축 중심, x 전방, y 좌측, z 위로 정의한다. 다음을 줄자와 수직추로
측정한다.

- `base_to_ant1_x_m`: 후륜축 중심에서 ANT1까지 전방 거리
- `base_to_ant1_y_m`: 좌측이면 양수
- ANT1 높이는 기록하되 현재 평면 경로제어는 x/y 보정이 핵심

차를 정확히 직선 기준선에 놓고 UM982 heading과 기준선의 진북 방위를 비교한다.
ROS에서 계산한 raw yaw는 `pi/2 - heading_true`다. 원하는 차량 yaw와 raw yaw의
차이를 `heading_mount_offset_deg`로 넣는다. 0도/90도/180도/270도 방향을 모두
시험해 부호와 180도 뒤집힘을 잡는다. 원시 수신기 heading을 바꾸는 것보다 ROS의
mount offset에 기록을 남기는 편이 추적하기 쉽다.

## 5. 차량 기하와 조향 피드백 보정

1. 후륜축/전륜축 중심 사이 `wheelbase_m`, 차폭, 앞 범퍼 위치를 측정한다.
2. 차륜을 띄우고 조향 링크를 기계 중앙에 고정한다. ADC 값을 100회 평균해
   `adc_center`로 기록한다.
3. 기계 간섭 2~3도 전에서 좌/우 software limit를 잡고 `adc_left/right`와 실제
   평균 전륜각을 각도계로 측정한다.
4. 센서선을 뽑거나 ADC가 범위를 벗어나면 `FAULT_STEERING_SENSOR`가 생기고 PWM이
   즉시 0이 되는지 시험한다.
5. P부터 올려 목표각의 90%에 빠르게 접근시키고, 작은 정상상태 오차만 I로 없앤다.
   D는 노이즈가 충분히 낮을 때만 소량 사용한다.
6. `-max, -중간, 0, +중간, +max` 다섯 점에서 목표각과 실측각 오차를 기록한다.
   piecewise linear ADC 변환 오차가 크면 펌웨어를 다점 LUT로 바꾼다.

조향 PID는 Nucleo에만 둔다. PC는 목표 조향각만 보내며, 통신이 끊기면 Nucleo가
watchdog으로 조향 PWM과 구동 PWM을 끈다.

## 6. 카메라 내부·외부 정합

### 6.1 내부 파라미터

실제 해상도와 focus를 고정한 상태에서 checkerboard를 영상 전체 위치와 각도로
촬영한다. 예시는 내부 코너 9x6, 한 칸 25 mm일 때다.

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
```

재투영 오차와 calibration 파일을 기록한다. autofocus, 해상도, 렌즈를 바꾸면 다시
한다. exposure/white balance는 자동 변동이 신호등 색 분류를 깨뜨리지 않도록
현장 밝기와 우천에서 고정값 또는 제한 범위를 정한다.

### 6.2 외부 파라미터 및 ROI

카메라 광학중심의 `base_link` 기준 x/y/z와 roll/pitch/yaw를 실측하고, 바닥에
1 m 격자와 차선 폭 마킹을 둬 영상의 지면점과 실제 좌표를 비교한다.

- `lane_roi_top_ratio`: 지평선/차체를 제외하면서 3~6 m 차선이 남는 최소값
- `lane_meters_per_pixel`: 영상 하단의 실제 차선 폭 / 픽셀 차선 폭으로 시작
- `traffic_roi`: 정지선 접근 중 신호등이 들어오는 정규화 좌표 `[x0,y0,x1,y1]`
- 정지선 pixel-row→거리 변환은 평면 homography로 별도 검증해야 한다. 현재
  classical node의 6 m 선형값은 안전한 최종 캘리브레이션이 아니다.

흰색/노란색 차선, 역광, 그늘, 젖은 노면 각각에서 lane offset의 부호가 “차량이
차선 중심보다 오른쪽이면 양/음 중 무엇인지” 기록하고 steering correction 부호를
확인한다. 차선 confidence가 낮을 때는 GNSS 경로만 추종하도록 제한한다.

## 7. LiDAR 정합과 저속 속도

- `base_link`에서 LiDAR 원점까지 x/y/z/yaw를 측정한다. LaserScan 0 rad가 차량
  전방, +각이 좌측인지 벽과 표적으로 확인한다.
- 평평한 벽을 1/2/4/6 m에 두고 거리 bias와 최소/최대 유효거리를 기록한다.
- `vehicle_half_width_m`은 차체 반폭, `corridor_margin_m`은 위치/조향 오차 여유,
  `lidar_to_front_bumper_m`은 LiDAR 원점에서 앞 범퍼까지 x 거리다.
- 어린이 더미와 T870 차체는 각 거리와 좌우 위치에서 점군 수가
  `minimum_cluster_points` 이상인지 확인한다. 사람 다리 하나나 빗방울 한 점에
  오작동하지 않도록 군집 조건을 실제 로그로 정한다.
- GNSS Doppler는 정지 부근과 짧은 후진에 약하다. 검증한 2D scan-matching odometry를
  `/lidar/odometry`로 넣으면 `velocity_selector`가 저속에서 선택한다. 벽이 없는
  환경이나 scan match 품질 저하 시에는 저속을 추측하지 않고 정지한다.

카메라-LiDAR 정합은 같은 수직판/반사 표적의 모서리를 영상과 점군에서 동시에
취득해 투영 오차를 최소화한다. 최소한 좌/중/우와 2/4/6 m 표적에서 LiDAR 장애물
방향과 카메라 픽셀 방향이 일치해야 한다.

## 8. 엔코더 없는 구동모터 보정

평지 직선, 일정 타이어 공기압, 차량 완성 중량으로 진행한다. 처음에는 PI의
`kp=ki=0`으로 두고 feed-forward 표부터 만든다.

1. 배터리 전압을 기록한다.
2. 전진 duty를 작은 값부터 0.03~0.05씩 올리고 각 점에서 5초 이상 일정 속도를
   유지해 GNSS 평균속도를 기록한다.
3. 출발하지 않는 duty, 안정적으로 굴러가기 시작하는 duty, 최대 허용속도 duty를
   찾는다. 후진도 별도로 반복한다.
4. 배터리 전압이 낮을 때 다시 측정해 전압 보정이 과도하지 않은지 확인한다.
5. 표를 `forward/reverse_speeds_mps`와 `duties`에 넣은 후 작은 P, 그 다음 I를
   추가한다. 급격한 duty 왕복이나 적분 누적이 생기면 즉시 낮춘다.
6. 0.3/0.6/1.0 m/s에서 건조·젖은 노면 제동거리와 지연을 각각 10회 측정한다.
   `emergency_distance_m`은 최악 제동거리, 인지/통신 지연, 여유거리를 합쳐 정한다.

경사로에서는 MDD20A brake만으로 밀리지 않는다고 가정하지 않는다. 차량 하중을
올린 실경사에서 유지 duty를 조금씩 찾아야 하며, 3초 동안 이동이 허용 오차를
넘으면 `hill_hold_enabled`를 켜지 않는다. open-loop 유지 duty는 전압·하중·노면에
민감하므로 기계식 브레이크가 가장 확실하다.

## 9. 경로 기록과 미션 구역

datum과 lever arm을 확정한 뒤 수동 저속 주행 또는 차량을 밀어서 경로를 기록한다.
이 조작은 경기 시작 전 지도 제작 과정이다.

```bash
ros2 run hl_ku_core route_recorder --ros-args \
  -p output_file:=$PWD/src/hl_ku_core/routes/course_measured.csv
ros2 service call /route_recorder/start std_srvs/srv/Trigger '{}'
ros2 topic pub --once /route_recorder/mission std_msgs/msg/String '{data: HILL}'
ros2 topic pub --once /route_recorder/speed std_msgs/msg/Float32 '{data: 0.4}'
ros2 topic pub --once /route_recorder/direction std_msgs/msg/Int8 '{data: -1}'
ros2 service call /route_recorder/stop_and_save std_srvs/srv/Trigger '{}'
```

미션 태그는 `NORMAL`, `HILL`, `S_OBSTACLE`, `TRAFFIC`, `PERP_PARK`, `DUMMY`,
`PARALLEL_PARK`, `END_LANE`, `FINISH`다. 주차는 전진/후진 지점이 겹치므로 단순
nearest-point가 잘못 건너뛰지 않는지 별도 검증하고, 확인선 접촉 판정용 센서/경로를
현장에서 조정한다. 제공된 `course_template.csv`는 형식 예시일 뿐 실차용이 아니다.

경로를 RViz 위에 놓고 다음을 확인한다.

- 모든 점이 실제 주행 가능 영역 안이고 curb/차선을 침범하지 않음
- 미션 시작 태그가 인지 가능한 거리보다 앞에 있음
- 경사 정지점 `hill_stop_s_m`이 규정의 허용 정지구역 안임
- 신호 정지 목표가 정지선 전 1 m 이내이면서 범퍼가 넘지 않음
- 주차 확인선과 전/후진 방향 전환점이 실측 차량 치수와 맞음
- END_LANE offset 후 두 차선 중 허용 차선 중심에 들어감

## 10. 실행 전 단계별 합격 게이트

### A. 모터 전원 OFF

- ROS 빌드, 모든 센서 topic rate와 frame 확인
- 2분 datum, RTK FIXED/heading 유효율 기록
- camera intrinsic, LiDAR 거리/각도, 모든 static transform 확인

### B. 차륜을 띄움

- 비상정지, UDP 단절, PC node kill, 잘못된 CRC에서 PWM=0 확인
- 조향 좌/우 극성, ADC 단선 fault, software limit 확인
- 구동 전/후진 극성과 duty slew 확인

### C. 통제된 평지 0.3 m/s 이하

- GNSS 경로 오차, camera correction 부호, LiDAR 긴급정지 확인
- RTK 또는 camera/LiDAR 단절 시 `/safety/status` 원인과 정지 확인
- feed-forward와 제동거리 측정

### D. 미션별 단독 시험

- 경사: 지정구역 완전정지 3.2초, 50 cm 미만 밀림, 30초 내 통과
- 신호: RED/UNKNOWN 정지, GREEN debounce 후 출발, 정지선 미통과
- 더미: LiDAR 경로 장애물이 앞범퍼 기준 3 m 이내이면 회피 없이
  즉시 제동, 3.2초 완전정지 후 최신 스캔에서 경로가 비어야 재출발
- S 장애물: 좌/우 랜덤 조합, 차선/curb 여유 유지
- 직각/평행주차: 확인선, 후진/전진 전환, 출차까지 각각 검증
- 종점: ↓ 차선 선택, X 차선 회피, UNKNOWN이면 보수 정지

### E. 전체 코스와 우천

- 먼저 `drive_enabled:=false`로 rosbag/replay 검증
- 안전요원이 물리 E-stop을 든 폐쇄코스에서 저속 전체 주행
- 최소 10회 연속 성공 전에는 속도를 올리지 않음
- 젖은 노면은 더 낮은 속도 profile과 실측 wet braking distance 사용
- 경기용 launch에는 teleop, 무선 키보드, 수동 개입 node를 포함하지 않음

최종 실행은 모든 기록이 채워진 뒤에만 한다.

```bash
ros2 launch hl_ku_core bringup.launch.py \
  config_file:=$PWD/src/hl_ku_core/config/system.yaml \
  calibration_file:=$PWD/src/hl_ku_core/config/calibration_record.yaml \
  route_file:=$PWD/src/hl_ku_core/routes/course_measured.csv \
  route_calibrated:=true drive_enabled:=true
ros2 service call /mission/arm std_srvs/srv/Trigger '{}'
```

먼저 `/system/preflight_report`가 `OK`인지 확인하고 `/safety/status`가 `OK`가 아닌데
임의로 조건을 우회하지 않는다. preflight의 `dry_full_course_test`, `wet_test`는 첫
폐쇄코스 시험 전에는 경고로 남지만, 나머지 필수 항목은 구동 차단 사유다. 실제 대회 경로,
센서 모델, pin map, 배터리와 NTRIP 계정은 현재 제공되지 않았으므로 이 문서의
빈 항목을 실측으로 채우는 과정이 반드시 남아 있다.
