# 인지 센서 장착·정합·검증 순서

이 문서는 전방 카메라와 2D LiDAR를 차량에 장착한 뒤 해야 할 일을 순서대로
정리한다. GNSS는 전역 경로의 주 센서이고, 카메라는 차선·정지선·신호를, LiDAR는
장애물과 근거리 안전을 담당한다. 현재 코드는 실차 데이터로 보정하기 전의
보수적인 기초 구현이며 완성된 대회용 인지기가 아니다.

## 1. 현재 코드의 범위

| 기능 | 현재 입력과 출력 | 현재 상태 | 실차에서 남은 작업 |
|---|---|---|---|
| 차선 | `/camera/image_raw` → offset/heading/confidence | 흰색·노란색 HSV 픽셀 기반 | 원근변환, 좌·우 차선 분리, 도로경계, 경사·우천 검증 |
| 정지선 | 영상의 넓은 흰색 행 → 거리 | 6 m 선형 환산 기초판 | homography 기반 앞범퍼 거리와 오검출 제거 |
| 신호등 | 고정 ROI의 적/녹색 비율 | 적/녹/UNKNOWN 기초판 | 신호등 검출·추적, 황색 정책, 역광/야간 데이터 |
| 일반 장애물 | `/scan` 전방 corridor → 거리/존재 | 연속 스캔점 군집과 최소거리 기반 | 차체 팽창, 물체 추적, 속도별 제동거리 |
| S코스 T870 | 가장 빈 각도 → 제한된 회피각 | gap steering 기초판 | 두 장애물 추적, 연석/도로경계와 함께 경로 생성 |
| 어린이 더미 | 일반 장애물로만 정지 | 전용 분류·추적 없음 | 좌/우 이동 검출, 도로 중앙 정지 확인, 3초 정지 시험 |
| 종점 `↓/X` | 메시지 계약과 상태기계만 있음 | **영상 검출기 없음** | 표지 검출, 차선별 신호 연결, UNKNOWN 안전정지 |

현재 `camera_perception_node.py`의 색 임계값, 고정 ROI와 정지선 거리식은 연결
확인용이다. 그대로 대회 성능을 가정하면 안 된다. `lidar_perception_node.py`도
도로 바깥의 큰 빈 공간을 안전한 회피 경로로 오인할 수 있으므로 S코스에서
road-boundary guard가 완성되기 전에는 속도를 높이지 않는다.

## 2. 센서 선정 전에 확인할 정보

카메라와 LiDAR 모델이 정해지면 아래를 `config/calibration_record.yaml`에 기록한다.

- 카메라: 모델/시리얼, 렌즈 수평·수직 FOV, 해상도, 실제 FPS, USB 방식,
  global/rolling shutter, 수동 exposure와 white balance 지원 여부
- LiDAR: 모델/시리얼, 측정 범위, scan rate, 각도 해상도, 최소거리, 실외·우천
  사용 조건, ROS 2 driver와 출력 메시지 형식
- 노트북: 각 장치가 필요한 USB 대역폭과 전력, 포트별 장치 경로, CPU/GPU 부하
- 전원: 센서와 노트북 전원은 모터 전원 노이즈의 영향을 줄이고, 케이블 빠짐과
  순간 전압강하가 없도록 고정한다.

제조사 driver는 하드웨어마다 달라 저장소에 포함하지 않았다. driver가 카메라는
`sensor_msgs/Image`, 2D LiDAR는 `sensor_msgs/LaserScan`을 내보내야 한다.

## 3. 차량 장착

ROS 차량 좌표는 `base_link` 기준 `+x` 전방, `+y` 좌측, `+z` 위쪽으로 통일한다.

### 카메라

1. 차체 중앙선 가까이, 진동이나 조향에 따라 움직이지 않는 브래킷에 고정한다.
2. 범퍼나 차체가 영상 하단을 과도하게 가리지 않으면서 가까운 정지선과 앞쪽
   차선이 함께 보이게 한다.
3. focus, 해상도, FPS를 확정한 뒤 테이프/나사고정 표시를 남긴다. 하나라도 바꾸면
   내부·외부 보정을 다시 한다.
4. 렌즈에 직사광선, 물방울, 오염이 생기는 위치를 피하고 차양과 투명 방수창이
   영상 왜곡을 만들지 확인한다.

### 2D LiDAR

1. 스캔 평면이 지면과 평행하고, 전방 시야를 타이어·범퍼·케이블이 가리지 않게
   중앙에 단단히 고정한다.
2. LiDAR 원점에서 앞범퍼까지의 x 거리를 실측한다. 장애물 거리는 센서가 아니라
   앞범퍼 기준이어야 한다.
3. 차체 반폭뿐 아니라 조향 시 앞 모서리가 휘두르는 영역까지 안전 corridor에
   포함한다.
4. 투명판, 검은 표면, 비·안개와 햇빛에서 누락/허위점이 얼마나 생기는지 실제
   센서 사양과 로그로 확인한다.

### 배선과 장치 이름

USB 허브보다 노트북의 독립 포트를 우선하고 커넥터에 strain relief를 둔다.
`/dev/video0`, `/dev/ttyUSB0` 같은 번호는 재부팅 때 바뀔 수 있으므로 카메라
serial과 USB VID/PID/path를 이용해 영구 이름을 만든다. 모든 센서 메시지의
`header.stamp`가 노트북 ROS clock과 같은 기준인지 확인한다.

## 4. 모터를 끈 연결 시험

모터 전원은 끈 상태에서 sensor driver를 먼저 실행한다. 그 다음 다음 항목을
확인한다.

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /scan
ros2 topic echo --once /scan
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

현재 freshness timeout은 카메라/LiDAR 각각 0.30초다. 순간 끊김을 포함해 항상 이
안에 새 메시지가 들어와야 하며, 시작 목표는 두 입력 모두 안정적인 10 Hz 이상이다.
카메라 해상도 때문에 노트북 처리 지연이 커지면 해상도/FPS를 함께 조정하고 원본
메시지 stamp부터 `/perception/state`까지 지연을 측정한다.

빈/형식오류 `LaserScan`은 freshness를 갱신하지 않으며, aggregator는 카메라와
LiDAR가 각각 약속한 모든 결과 필드가 최신일 때만 source를 fresh로 표시한다.
driver가 정상이라면 구동 노드를 전혀 띄우지 않는 인지 전용 launch를 사용한다.

```bash
cd HL_FMA_2026/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hl_ku_core perception_test.launch.py
```

```bash
ros2 topic echo /perception/state
ros2 topic hz /perception/state
```

이 launch에는 MCU bridge, controller, keyboard와 모터 명령 노드가 없다.

## 5. 카메라 정합

### 내부 보정

실사용 해상도와 고정 focus로 checkerboard를 화면 중앙·네 모서리·가까이·멀리에서
다양한 각도로 촬영한다. 내부 코너가 9x6이고 한 칸이 25 mm인 예시는 다음과 같다.

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
```

왜곡 보정 영상의 직선이 휘지 않는지 확인하고 calibration 파일, 재투영 오차,
해상도와 렌즈 설정을 기록한다.

### 외부 보정과 지면 변환

1. 후륜축 중심인 `base_link`에서 카메라 광학중심까지 x/y/z와 roll/pitch/yaw를
   실측한다.
2. 평지에 1 m 격자, 실제 차선 폭, 정지선을 만들고 대응점을 취득한다.
3. undistort 후 영상 픽셀을 차량 바닥좌표로 보내는 homography/IPM을 구한다.
4. 좌/중/우와 1/2/4/6 m 표적의 변환 오차를 재고, 앞범퍼 기준 정지선 거리를
   별도로 검증한다.
5. 카메라 기준 offset 부호와 실제 조향 보정 부호를 저속에서 확인한다.

오르막 진입에서는 노면 pitch가 변하므로 평지 homography가 틀어진다. IMU 성능이
낮은 현재 구성에서는 GNSS 경로를 주 제어로 유지하고 카메라 correction을 제한한
채, 평지·경사 진입·경사 상단 데이터로 구간별 ROI/homography 또는 학습형 차선
모델을 검증한다. 한 개의 고정 `lane_meters_per_pixel` 값만 믿지 않는다.

### 신호 데이터

실제 설치 높이와 접근 경로에서 적색·황색·녹색·꺼짐을 각각 촬영한다. 거리,
좌회전 접근각, 역광, 그늘, 흐림, 우천을 포함한다. 최종 구현은 전체 영상에서
신호등을 찾고 여러 frame 동안 추적한 뒤 색을 판정해야 한다. 검출 실패나 상충
판정은 `LIGHT_UNKNOWN`으로 보내 정지하도록 유지한다.

종점 `↓/X`는 좌우 각 차선의 표시가 어느 차선에 속하는지까지 출력해야 한다.
단순히 기호 하나를 찾는 것으로 끝나지 않으며 `/perception/end_lane_signal`,
`/perception/allowed_lane`, `/perception/end_lane_confidence`를 발행할 별도 검출기가
필요하다.

## 6. LiDAR 정합

1. 벽을 전방 1/2/4/6 m에 놓아 range bias, 최소 유효거리와 각도 0의 방향을 잰다.
2. 좌우 같은 거리에 표적을 놓아 `LaserScan`의 +각도가 차량 좌측인지 확인한다.
3. `base_link`에서 센서 원점까지 x/y/z/yaw와 `lidar_to_front_bumper_m`을 기록한다.
4. 실제 차폭, localization 오차와 조향 sweep를 합쳐 `vehicle_half_width_m`과
   `corridor_margin_m`을 정한다.
5. T870 차체와 어린이 더미를 좌/중/우, 가까이/멀리 배치해 점 개수, 연속성,
   최소거리 오차를 rosbag으로 측정한다.
6. 계산한 정지 trigger가 최악의 인지 지연과 실측 제동거리를 포함하는지 검증한다.

S코스는 전방 최소거리만으로 풀지 않는다. 스캔을 연속 군집으로 만들고 두 T870의
위치·크기를 추적한 뒤, 차체 footprint를 팽창시킨 local path를 생성해야 한다.
카메라 차선/연석 경계를 함께 사용해 회피점이 규정 도로 안에 있는지 검사한다.

어린이 더미 미션은 회피가 아니라 완전정지다. `DUMMY` 구간에서는 전방
주행 corridor 안의 LiDAR 장애물 거리가 앞범퍼 기준 3 m 이내가 되면 즉시
제동한다. 완전정지를 3.2초 이상 유지한 뒤에도 최신 LiDAR 스캔이어야 하며,
경로 corridor에서 장애물이 사라졌을 때만 재출발한다. 스캔이 끊기거나
stale이면 출발하지 않는다.

## 7. 카메라-LiDAR 시간·공간 정합

동일한 수직판 또는 반사표적을 좌/중/우와 2/4/6 m에 두고 영상 픽셀과 LiDAR
좌표가 같은 물체를 가리키는지 확인한다. 고정 TF만 기록하는 것으로 끝내지 말고
투영 오차를 거리별로 표로 남긴다.

차량을 천천히 움직이며 camera와 scan stamp 차이, driver buffering과 처리시간을
측정한다. 현재 aggregator는 결과 토픽의 최신성만 묶으며 엄밀한 timestamp 동기
fusion을 하지 않는다. 고속화 전에 message_filters 또는 timestamp 기준 결합과
동기 오차 제한을 추가해야 한다.

## 8. rosbag 데이터 수집

인지 개선 전에는 원본 센서와 현재 결과를 함께 저장한다.

```bash
ros2 bag record -o perception_$(date +%Y%m%d_%H%M%S) \
  /camera/image_raw /camera/camera_info /scan \
  /perception/state /gnss/fix /gnss/status /localization/gnss_pose
```

다음 장면을 서로 다른 bag으로 구분하고, 정답표에는 물체/차선/신호, 거리,
성공·실패와 날씨를 기록한다.

- 평지 직선·곡선·좌회전, 흰색/노란색 차선과 마모된 차선
- 오르막 진입·중간·정상부와 내리막, 햇빛 방향별 장면
- 적/황/녹/꺼짐 신호와 정지선, 여러 접근거리와 각도
- T870 두 대의 좌→우/우→좌 배치, 도로 가장자리 여유
- 어린이 더미 좌/우 출현, 이동 중·중앙 정지·제거 후
- 종점 두 차선의 `↓/X` 모든 조합
- 맑음·그늘·역광·우천, 깨끗한 렌즈와 물방울/오염 상태

같은 bag으로 알고리즘 변경 전후를 replay해 비교하고, 좋은 장면만 골라 임계값을
맞추지 않는다.

## 9. 단계별 합격 기준

### 책상/집

- 30분 동안 USB 재연결, frame 누락과 driver crash가 없음
- 카메라·LiDAR frame 방향, timestamp와 topic rate가 일정함
- 녹화 bag replay에서 같은 입력에 같은 결과가 나옴
- sensor topic을 끊으면 0.30초 안에 freshness가 false가 됨

### 차량 고정, 모터 OFF

- 실측 TF와 앞범퍼 거리 보정 완료
- 좌/중/우 표적에서 camera와 LiDAR 방향이 일치
- 차체 자체가 장애물로 보이지 않고 가까운 표적 누락이 없음
- 카메라를 가리거나 LiDAR를 끊으면 `/safety/status`가 주행 불가 원인을 냄

### 폐쇄코스 저속

- GNSS 경로 위에서 차선 보정 부호가 항상 올바르고 correction 포화가 없음
- 적색/UNKNOWN에서 정지선을 넘지 않고 녹색은 debounce 후 출발
- 더미 좌·우 출현 모두 3 m 이내에서 회피하지 않고 즉시 제동,
  3.2초 완전정지 후 최신 LiDAR 스캔에서 경로가 비어야 재출발
- T870 모든 랜덤 배치에서 충돌과 도로이탈 없이 통과
- 경사 진입과 정상부에서도 차선 confidence 실패가 위험한 조향으로 이어지지 않음
- 종점 모든 `↓/X` 조합에서 허용 차선 또는 UNKNOWN 안전정지가 재현됨

각 항목을 최소 10회 연속 통과하고 실패 bag을 검토하기 전에는 속도를 올리지
않는다. 건조 결과로 우천 성능을 대신하지 않는다.

## 10. 구현 우선순위

1. 센서 모델 확정, driver 설치와 안정적인 원본 topic
2. 카메라 intrinsic, 카메라/LiDAR TF, 앞범퍼와 차량 footprint 실측
3. rosbag 수집과 시각화 도구, 지면 homography 기반 차선·정지선 거리
4. LiDAR 연속 군집과 속도별 emergency stop
5. 신호등 detector/tracker와 보수적인 UNKNOWN 처리
6. 어린이 더미 추적·3초 정지, S코스 local planner + road boundary
7. 종점 `↓/X` detector와 차선별 신호 association
8. 경사·우천·역광 전체 회귀시험 후에만 자율주행과 결합

센서 모델, 설치 높이, 렌즈, LiDAR scan 방향과 실제 경기장 치수는 아직 제공되지
않았다. 따라서 수치 파라미터를 임의 확정하지 말고 측정 결과를
`calibration_record.yaml`과 rosbag 이름에 함께 남긴다.
