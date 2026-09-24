# HL 만도 건국대 시험 TUI

`run_mando_tui.sh`는 RTK 확인, 시나리오 선택, 목표값 입력, MCAP 기록,
주행 준비와 정지를 한 터미널에서 관리한다. TeamKAI Decision `hwj` 브랜치의
번호 선택/속도 입력/실행/정지 흐름을 참고했고, HL_KU의 기존
`gps_route_test.launch.py`, `safety_supervisor`, `mission_manager`를 그대로
사용한다.

## 1. 최초 1회 빌드

```bash
cd $HOME/HL_KU/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hl_ku_interfaces hl_ku_core hl_ku_foxglove
```

빌드 후 설정 파일을 검사한다.

```bash
cd $HOME/HL_KU/operations
./run_mando_tui.sh --check-config
./run_mando_tui.sh --print-commands
```

일반 실행은 Foxglove Studio를 열고 `ws://localhost:8765`에 자동 연결한다.
이미 앱이 열려 있으면 같은 앱에 live 연결을 연다. Foxglove를 열지 않을 때만
`./run_mando_tui.sh --no-open-foxglove`를 사용한다.

차량에서는 같은 스크립트가 NUCLEO ST-LINK 921600 bps multiplex를 먼저 실행해
`/tmp/nucleo-control`과 `/tmp/nucleo-lidar`를 만든다. 경로를 준비하면 A3 드라이버,
`lidar_perception`, `scan_obstacle_mapper`, `local_detour`도 함께 시작되고 TUI의
`LiDAR` 행에 `/scan`, 지도 투영, 후보 상태가 나온다. NUCLEO 포트가 없거나
mux가 시작되지 않아도 TUI 화면은 열린다. 이 경우 보드를 연결한 뒤 TUI를
종료하고 다시 실행해야 mux가 시작된다. 장치 없이 설정만 볼 때는
`./run_mando_tui.sh --no-start-mux --no-auto-rtk`도 사용할 수 있다. NUCLEO
보드를 교체했다면 `HLKU_NUCLEO_PORT=/dev/serial/by-id/... ./run_mando_tui.sh`로
새 by-id를 지정한다.

자동 배치 경로가 아직 없으면 `--check-config`는 `NOT_READY`를
표시한다. TUI에서 `G`로 RTK를 시작한 뒤 FIXED와 ENU 위치·방향이 표시된
상태에서 시나리오 번호와 `R`을 누르면 경로와 calibration 파일을 자동으로
만들고 주행 스택 준비까지 계속한다.

## 2. 통합 미션 CSV와 형상 선택

`course_07_mission.csv` 하나에 Course 07 전역 경로, 8개 P/T/F 형상,
런타임 미션 상태와 CSV 이벤트를 합쳤다. 모든 TUI 경로 시나리오는 이 파일에서
`source_variant_id`와 `s` 구간을 선택한다. `S_CURVE`는 런타임 `S_OBSTACLE`로
변환되고, 후진/전진은 `direction` 값으로 실행된다. 원본 경로는 축소하지 않는다.

1번은 직선 `s=44~52 m`, 2번은 약 53도 좌곡선 `s=77.7~85.7 m`를 사용한다.
3번은 통합 미션 시나리오다. 처음 8개 선택지는 전체 Course 07의 P/T/F 조합이고,
그 다음에 S자/T주차 전체 구간을 고른다.

- `S-FULL`: S_CURVE 시작 `s=191.3864 m`부터 종료 `s=229.0598 m`까지
  37.67 m 전체
- `T1-FULL`: T자 주차 1번 44.53 m 전체, 전진 18.28 m → 후진 10.25 m →
  전진 15.80 m
- `T2-FULL`: T자 주차 2번 46.71 m 전체, 전진 19.22 m → 후진 13.19 m →
  전진 14.10 m

전체 Course 07 선택지는 약 695~699 m이므로 닫힌 시험 코스에서만 사용한다.
건대에서는 6~9번의 길이 짧은 원본 조각으로 나눠 시험한다.

학교 주차 시험은 번호만 누르면 자동선택 구간이 첫 항목으로 선택된다.

- `7`: `T-AUTO`, T자 주차 전체 46.83 m. T1/T2를 라이다로 판단한다.
- `9`: `P-AUTO`, 평행주차 전체 45.21 m. P1/P2를 라이다로 판단한다.

두 구간은 현재 RTK 위치와 차량 방향에 회전·평행이동만 하며 크기를 줄이지
않는다. 다른 조각을 보다가 자동선택 구간으로 돌아갈 때는 해당 번호를 다시
누르거나 `[`/`]`로 `T-AUTO` 또는 `P-AUTO`를 선택한다.

T1/T2는 방향 전환점에서 차량이 멈춘 것을 확인하고 0.75초 정지한 다음 다음
방향으로 넘어간다. 구간의 미션 태그는 마지막 정지점을 제외하고 원본 그대로
유지한다. HILL의 정지 위치와 hold 시간도 CSV 이벤트에서 읽는다. 6~9번은
같은 통합 CSV에서 각각 전반, 핵심 중반, 후반 연결, 주차/종료 조각을 고른다.
8개 조합이 필요한 전체 코스 외 캠퍼스 조각의 기본 형상은 P1/T1/F1이다.
`R`을 누를 때마다 선택 구간을 현재 차량의 RTK ENU 위치와 헤딩에
새로 회전·평행이동한다. 점 사이 거리와 곡률의 스케일은 `1.0`으로 유지된다.

3번 `S-FULL`과 7번 `M01 S자 전체`만 실시간 LiDAR 회피가 켜진다. 두 구간은
`/scan`에서 실제로 관측한 장애물 군집을 GNSS 기준 `map`으로 옮기고, 군집의
보이는 폭을 반영한 짧은 회피 경로를 만든다. 장애물이 없으면 원본 S자 경로를
Stanley로 따라간다. 장애물이 있으면 원본 경로를 따라 접근한 뒤 회피 경로도
Stanley로 추종하고 다시 원본에 합류한다. 가상 장애물 미리보기 토픽은 주행
조향에 연결되지 않는다. T주차와 다른 구간은 회피 조향이 꺼져 있다.

처음 실행할 때의 순서는 다음과 같다.

1. 차량을 실제 출발 위치에 놓고 앞바퀴와 차체 방향을 주행 방향에 맞춘다.
2. `G`를 누르고 화면의 RTK가 `FIXED`, heading이 `OK`가 될 때까지 기다린다.
3. 직선은 `1`, 곡선은 `2`를 누른다. 통합 미션은 `3`을 누르고 `[`/`]`로
   전체 코스 형상 또는 S/T 전체 동작을 고른다. 건대의 분할 시험 구간은 `6`~`9`에서
   고르고, TUI에 표시된 원본 형상 ID와 `s` 범위를 확인한 뒤 `R`을 누른다.
4. TUI가 현재 `x`, `y`, `heading`에 선택한 원본 구간을 배치하고
   선택한 `routes/mando/seg_*.csv`와 `config/mando/seg_*_calibration.yaml`을
   저장한다.
5. RTK와 차량 준비 항목이 `[✓]`가 되면 `SPACE`로 출발한다. 기록이 필요하면
   출발 전이나 주행 중 `B`를 눌러 MCAP을 시작한다.

`G`를 누르지 않고 바로 시나리오 번호와 `R`을 누르면 TUI가 RTK 스택을 먼저 시작한다.
RTK와 ENU 위치·방향이 표시된 뒤 `R`을 한 번 더 누르면 자동 배치가 진행된다.
이미 생성한 구간도 `R`을 누를 때 현재 위치와 방향으로 다시 배치한다.
Foxglove의 왼쪽 지도에는 건대 기준경로와 흰색 실제 궤적이 보이고, 오른쪽
아래 `시험장 원본 위치`에는 원본 690.37 m 경로의 선택 범위와 현재 `s`가
동시에 표시된다. FSM 카드는 원본 위치에서 예상되는 상태를 강조하며 실제
`/mission/status`도 바로 아래에 따로 표시한다.

TUI의 시나리오 표 아래 `현재 선택` 상자에는 구간 코드, 정확한 원본 파일과
`s` 범위, 길이, FSM, 전진/후진 순서, 필요한 풋프린트가 표시된다. 긴 설명이
한 줄에서 잘려도 이 상자에서 선택한 구간을 모두 확인할 수 있다.

수동으로 다른 구간을 만들 때는 `prepare_mando_route.sh`를 사용할 수 있다.
아래 명령은 원본 구간을 건국대 ENU `(0, 0)`, 진행방향 동쪽 `0 deg`에 놓는
예시다.

```bash
cd $HOME/HL_KU/operations
source ./setup_env.bash

./prepare_mando_route.sh \
  "$HLKU_ROUTES/course_07_vehicle.csv" \
  "$HLKU_ROUTES/mando/konkuk_straight.csv" \
  --start-s-m 44.0 \
  --length-m 8.0 \
  --target-x-m 0.0 \
  --target-y-m 0.0 \
  --target-heading-deg 0.0 \
  --speed-mps 0.30 \
  --gps-config "$HLKU_CONFIG/gps_only_route.yaml" \
  --calibration-output "$HLKU_CONFIG/mando/konkuk_straight_calibration.yaml"
```

곡선과 S자 구간도 같은 명령에서 원본 `--start-s-m`, 길이, 출력 파일을
바꿔 만든다. 출력 파일은
`mando_scenarios.yaml`의 `route_file`/`calibration_file`과 맞춰야 한다.
이미 파일이 있으면 덮어쓰지 않으며, 의도적으로 다시 만들 때만 `--force`를
추가한다.

수동 명령으로 생성된 calibration 초안은 `course.route_calibrated: false`다.
TUI가 RTK pose로 자동 배치한 파일은 `true`와 당시 `x`, `y`, `heading`,
`scale: 1.0`을 함께 기록한다. relaxed 모드에서는 calibration 정합 여부가
출발을 차단하지 않는다. 다음 값은 화면과 Foxglove에서 경로를 이해하기 위한
확인 항목이다.

1. `gps_only_route.yaml`의 datum이 이번 건국대 고정 ENU 원점과 같다.
2. Foxglove/RViz에서 경로 시작점, 차량 위치와 차량 진행방향이 겹친다.
3. 생성된 calibration의 `course.route_file`이 선택한 CSV 이름과 같다.
4. calibration의 datum, 안테나 lever arm, heading offset이
   `gps_only_route.yaml`과 같다.

기본 시나리오의 `preflight: relaxed`는 초기 주행 확인용이다. calibration과
시작 정렬은 정보로 표시하고 RTK FIXED는 모든 경로 시나리오에서 출발 조건으로
확인한다. 카메라/GNSS MCAP은 `B`로 선택한 경우에만 기록 상태를 표시한다.

경로 CSV는 다음 조건을 지켜야 한다.

- 열: `s_m,x_m,y_m,target_speed_mps,mission,direction`
- `s_m`은 0부터 엄격히 증가하고 좌표 단위는 m
- 초기 시험에는 `direction=1`, `NORMAL/FINISH` 사용을 권장
- 마지막 점은 `mission=FINISH`, `target_speed_mps=0`

## 3. TUI 실행과 시험 순서

Foxglove rosbag 재생이 켜져 있으면 먼저 그 재생 터미널에서 `Ctrl+C`로
끝낸다. 과거 bag의 `/gnss/status`와 실차 GNSS가 섞이는 것을 막기 위해 TUI는
`rosbag2_player`를 감지하면 RTK/주행 준비를 차단한다. 첨부 화면처럼 상단에
`ROSBAG REPLAY`가 보이면 재생을 끝낸 상태에서 TUI를 실행한다.

```bash
cd $HOME/HL_KU/operations
./run_mando_tui.sh
```

카메라는 현재 연결된 C270의 고정 장치 경로를 사용한다. 카메라를 바꾸면
`mando_scenarios.yaml`의 `common.camera_device`만 해당 `/dev/v4l/by-id/...`로
수정한다.

이 PC에는 수정된 Foxglove 확장을 설치해 두었다. Foxglove 앱을 한 번 다시
시작한다. 다른 PC에서는 아래 `.foxe` 파일을 더블클릭해 설치한다. 기존에 가져온
`HL_KU_FMA_Replay.json` 레이아웃은 그대로 쓰며, live 입력에서는 상단이
`CURRENT RUN · LIVE MONITOR`와 `LIVE DRIVE`로 바뀌고 하단 재생 바가 숨겨진다.
만도 개요 화면의 왼쪽 위는 8경로 그림 대신 현재 시험 기준경로, 차량 위치,
실제 궤적을 표시한다. 제목 아래에는 원본 `course_07`의 `s` 구간과 배치된
경로 전체 길이가 나오며 초록 점은 시작, 빨간 점은 종료다.

```text
$HOME/HL_KU/ros2_ws/src/hl_ku_foxglove/foxglove-extension/hlku.hl-ku-fma-dashboard-0.1.2.foxe
```

TUI는 액추에이터 없이 RTK, C270 카메라와 Foxglove live bridge부터 자동으로
시작한다. Foxglove가 `ws://localhost:8765`에 연결돼 있으면 같은 레이아웃에서
현재 ENU 위치와 `/camera/image_raw`가 표시된다. 권장 시험 순서는 다음과 같다.

화면 표시는 `[✓]`가 해당 조건 완료, `[대기]`가 아직 입력을 기다리는 상태,
`[차단]`이 출발할 수 없는 상태다.

1. 경로·datum 시험은 GNSS 행의 `[✓] RTK: FIXED`를 확인한다. 최신 RTK FIXED와
   유효 위치·헤딩만 출발 조건이며, 위성 수, HDOP와 RTCM 보정나이는 판단용으로
   계속 표시한다. 손가락 주행은 GNSS 없이 바로 실행할 수 있다.
2. `1` 직선, `2` 곡선, `3` S자·굴절, `4` 손가락 주행, `5` 건대 기준좌표,
   `6` 전반, `7` T주차 자동선택·중반, `8` 후반,
   `9` 평행주차 자동선택·종료 중 하나를 고른다.
   여러 구간이 있는 번호는 `[`/`]` 또는 좌우 방향키로 조각을 고른다.
3. `V`를 누르고 경로 주행은 구동 PWM(0~100), 손가락 주행은 키당 PWM 증감값(1~100)을 숫자로
   입력한다. 선택한 값은 TUI에 `30/100 PWM` 또는 `키당 ±10/100`처럼 표시된다. 상단 표의
   `마지막 키`, `최근 숫자`, `선택 번호`, `현재 설정`에서 방금 누른 키와 입력값을
   계속 확인할 수 있다.
4. `R`은 선택한 원본 조각을 현재 RTK pose에 다시 배치하고 주행 또는 datum
   스택만 준비하며 MCAP을 시작하지 않는다.
   손가락 주행은 GNSS·RTK·카메라·MCAP 준비 여부로 실행을 막지 않는다. 경로
   시나리오는 RTK 확인 프로세스를 종료하고 같은 GNSS 포트를 주행 스택이
   인계하며, C270 카메라와 Foxglove live bridge도 함께 다시 연결한다.
5. 기록이 필요할 때만 `B`를 누른다. `ros2 bag record -a`를 사용하므로 원시
   LiDAR `/scan`, 장애물 군집, 회피 후보를 포함해 실행 중인 모든 ROS topic을
   MCAP에 저장한다. `기록 확인` 행의 `[✓] MCAP`, `[✓] GNSS`, `[✓] 카메라`,
   `[✓] LiDAR /scan`은 실제 파일 생성과 recorder 구독을 뜻한다. 기록하지 않을 때는 네 항목이 `[선택]`으로 표시되며 출발을
   막지 않는다. 주행 중 `B`를 다시 누르면 그 시점에서 파일을 마무리한다.
   이미 다른 rosbag recorder가 있으면 중복 녹화를 시작하지 않고 오류를 표시한다.
6. `[✓] 출발 조건: 완료`를 확인한다. TUI에 현재 ENU x/y, 시작점 거리, 방향 오차,
   차량 feedback과 LiDAR `/scan` 수신 상태가 함께 표시된다. S자 회피 시험에서는
   `회피조향=ON`, `map=OK:0`(장애물 없음) 또는 `OK:n`(실제 장애물 n개)을 확인한다.
   T/P 자동 주차에서는 다음 줄에 `T1/T2` 또는 `P1/P2`의
   `UNKNOWN/CLEAR/BLOCKED`, 선택 경로와 `관찰 중/선택 고정`이 표시된다.
   두 공간이 모두 판정되면 경로가 가까운 접근 초반에 즉시 선택하며, 한쪽이
   미확인이면 지정된 전진→후진 전환 직전까지 관찰한 뒤 기본 규칙으로 고정한다.
   RViz의 노란 경로는 실제 추종 중인 `/planning/reference_path`이므로 선택이
   고정되면 선택된 주차 경로로 바뀐다.
   주행용 RViz의 청록색 점은 원시 `/scan`, 빨간 원기둥은 주차 판단에 전달되는
   지도 좌표계 군집이다. `선택 고정` 이후에 놓은 라바콘은 해당 주행의 선택을
   바꾸지 않으므로 출발 전에 주차 공간을 구성한다.
   장애물을 놓았을 때 `detour=CANDIDATE`면 후보가 만들어진 것이다. `NO_OBSTACLE`은
   장애물이 없어서 원본 경로를 따르는 정상 상태다. 다른 구간의 `회피조향=OFF`는
   정상이다. `GEOMETRY_UNVERIFIED`, `POSE_STALE`, `NO_SAFE_DETOUR`이면 회피 후보가
   없으므로 실제 장애물 옆에서 출발하지 않는다.
7. `Space`로 출발한다. 주행 중 같은 키는 일시정지/재출발이다.
8. `X`는 즉시 mission fault를 latch한다. `S`는 fault를 보낸 뒤 ROS 주행
   스택을 종료한다. 다시 확인하려면 `G`로 RTK 확인을 시작한다.
9. `B`로 기록을 끝내고 `Q`로 TUI를 종료한다. `Q`는 주행 스택과 기록기도
   함께 정리한다.

전체 키는 다음과 같다.

| 키 | 기능 |
|---|---|
| `1`~`9` | 기준시험/손가락/datum/course 07 전반·중반·후반·종료 그룹 선택 |
| `[` / `]` | 선택 그룹 안의 원본 구간 이동 |
| `V` | 경로 구동 PWM(0~100), 손가락 PWM 증감값 또는 datum 측량시간 입력 |
| `R` | 선택 경로를 현 위치에 재배치하고 주행 준비, 손가락 진입 또는 datum 시작 |
| `A` | 완료된 건대 datum을 `gps_only_route.yaml`에 적용 |
| `G` | 현재 스택 종료 후 RTK 확인 시작 |
| `B` | 전체 ROS 토픽 MCAP 선택 기록 시작/종료 |
| `Space` | 경로 출발/일시정지/재출발 |
| `X` | 비상정지 및 fault latch |
| `S` | 안전정지 후 현재 스택 종료 |
| `Q` | 전체 종료 |

## 4. PWM 입력의 의미

경로 시나리오 1~3과 6~9의 `V`는 `0~100/100` 구동 PWM이다. 예를 들어
`V`에 `20`을 입력하면 전진 구간은 `+20/100`, 후진 구간은 `-20/100`을
요청한다. 전진·후진별 별도 PWM 표와 기존 `30/100` 시험 상한은 TUI 경로 주행에
적용되지 않는다. `0`은 구동 PWM 0이다. 방향 전환점, CSV 정지점, 미션 정지,
일시정지·비상정지는 계속 0을 명령한다. PWM은 실제 m/s가 아니며 배터리 상태,
노면과 차량 하중에 따라 속도가 달라진다. 실행 중에는 `V`로 PWM을 바꿀 수 없고
`S`로 현재 스택을 종료한 뒤 바꾸고 다시 `R`로 준비한다.
이 직접 PWM 방식은 만도 TUI로 시작한 경로 시나리오에 적용된다. 경로 계산과
Foxglove 목표 속도 표시에 쓰이는 내부 m/s 값은 실제 구동 PWM 크기를 결정하지 않는다.

시나리오 4에는 차량 속도 폐루프가 없으므로 m/s를 설정할 수 없다. 여기서
`V`는 `1~100 PWM` 범위의 키당 증감값이다. 기본값 10에서는 `W`를 누를 때마다
`0 → +10 → +20 → +30`처럼 전진 PWM이 커지고, `S`를 누를 때마다 10씩 내려
`+20 → +10 → 0 → -10 → -20`처럼 정지를 거쳐 후진한다. `A`는 누를 때마다
좌조향을 `0.096 rad`씩, `D`는 우조향을 `0.096 rad`씩 움직인다. 중앙에서 조향
끝까지 5번 입력하며, 각 입력 뒤에는
그 조향각을 계속 유지하며 반대 키를 누르면 한 단계씩 중앙 방향으로 돌아온다.
조향 범위는 `-0.48~+0.48 rad`이고 `C`는 즉시 중앙으로 맞춘다. `Space`는 구동을
0으로 만들고 브레이크를 건다.
`R`을 누르면 GNSS 없이 손가락 주행 화면으로 바로 넘어간다. 화면 하단에는
마지막 입력 키, 실제 구동 명령, 조향 방향·각도, PWM 증감 단위, 브레이크/FAULT 상태가
계속 갱신된다. 손가락 화면의 `Q`는 TUI로 복귀하고, TUI에서 `Q`를 다시 누르면
전체 프로세스와 MCAP을 종료한다. 손가락 주행을 기록하려면 손가락 화면으로
들어가기 전에 TUI에서 `B`를 누른 다음 `R`을 누른다.

`100/100`은 NUCLEO 메시지에서 표현 가능한 전체 PWM 범위이고 `±0.48 rad`는 이
차량 설정에 기록된 조향 양 끝점이다. 손가락 주행 설정은 이 범위를 모두 쓴다.

시나리오 5에서 `V`는 속도가 아니라 datum 측량시간 `10~3600초`다. 기본값은
120초이고 최소 100개 샘플을 사용한다. `R`을 누르면 RTK·카메라·Foxglove live 연결과 datum 측량을 시작하고
RTK FIXED 위치 샘플만 평균한다. 안테나 ANT1을 건대에서 계속 사용할 고정 마킹에
움직이지 않게 둔다. 측량 파일이 생성되면 화면에 `[✓] 건대 datum 측량 완료`가
나타난다. 이때 `A`를 누르면 위도·경도·고도만 `gps_only_route.yaml`에 적용하고
기존 파일은 같은 폴더의 `.bak_날짜_시간` 파일로 보관한다. `G` 또는 다음 `R`로
GNSS 스택을 다시 시작해야 새 datum이 현재 ENU 위치에 반영된다.

## 5. 직선 시험에서 비교할 값

5~10 m 직선에서는 한 번에 한 PWM 값만 사용하고 같은 구간을 반복한다. TUI와
bag에서 아래 토픽을 비교한다.

| 항목 | 토픽/필드 |
|---|---|
| 실제 ENU 위치 | `/localization/gnss_pose` x, y |
| 기준 경로 | `/planning/reference_path` |
| 진행 거리 | `/mission/status.route_s_m` |
| 횡오차 | `/planning/cross_track_error_m` |
| 헤딩 오차 | `/planning/heading_error_rad` |
| 요구 조향 | `/planning/path_command.steering_angle_rad` |
| 비교용 Stanley/Pure Pursuit | `/planning/stanley_steering_rad`, `/planning/pure_pursuit_steering_rad` |
| 실제 제어기 확인 | `/planning/tracker_mode=STANLEY_100`, `/planning/pure_pursuit_weight=0.0` |
| 실제/목표 조향 | `/vehicle/feedback.steering_angle_rad`, `steering_target_rad` |
| 안전 차단 원인 | `/safety/status` |

첫 주행은 낮은 PWM부터 시작해 시작점 0.5 m 이내, 방향 오차 10 deg
이내로 정렬한다. 직선에서 CTE 부호가 한쪽으로 계속 커지는지, 요구 조향과
실제 조향의 부호·지연이 맞는지 먼저 본 뒤 곡선과 다른 구간으로 넘어간다.
만도 TUI 경로는 CSV의 `direction=1/-1`에 따라 전진·후진하며 실제 조향에
Stanley만 사용한다. Pure Pursuit 값은 MCAP에서 두 알고리즘을 비교하기 위한
진단값이며 실제 조향 명령에 섞이지 않는다.

`B`를 눌러 선택 기록한 bag은 MCAP 형식으로 아래에 저장된다.

```text
$HOME/HL_KU/data/bags/mando_sNN_YYYYMMDD_HHMMSS/*_0.mcap
```

TUI가 실행한 ROS 프로세스의 stdout/stderr 로그는 다음 폴더에 남는다.

```text
$HOME/HL_KU/logs/mando_tui
```

## 6. 개별 명령

TUI 밖에서 RTK만 확인:

```bash
./rtk_check.sh
```

TUI 밖에서 전체 PWM 범위를 열고 키당 10 PWM씩 증감하는 손가락 주행:

```bash
./run_finger_drive.sh --max-pwm 100 --initial-pwm 10 --step-pwm 10
```

실시간 상태 확인:

```bash
ros2 topic echo --once /gnss/status hl_ku_interfaces/msg/GnssStatus
ros2 topic echo --once /mission/status hl_ku_interfaces/msg/MissionStatus
ros2 topic echo --once /safety/status std_msgs/msg/String
ros2 topic hz /vehicle/feedback
```

기존 Foxglove bag 재생은 실차 시험을 끝낸 후 실행한다. 실시간 시험에서는
TUI가 같은 `ws://localhost:8765` 연결을 열기 때문에 별도 명령이 필요 없다.

```bash
./replay_foxglove.sh "$HLKU_DATA/bags/mando_YYYYMMDD_HHMMSS" 1.0
```

## 7. relaxed 모드에서 남아 있는 조건

`R`을 누르면 ROS 주행 노드가 올라오고 미션은 INIT 상태로 기다린다. relaxed
모드는 다음 조건만 출발에 사용한다.

- CSV를 읽을 수 있고 마지막 waypoint가 `FINISH`, 속도 0임
- Foxglove rosbag 재생기나 별도 GNSS/미션/NUCLEO 프로세스와 충돌하지 않음
- 최신 RTK FIXED와 유효 위치·헤딩
- 경로 계산에 필요한 ENU pose가 들어옴
- 차량 feedback, mission, path command, safety 메시지가 들어옴
- NUCLEO fault가 없고 명령값이 유한하며 설정된 PWM·조향 범위 안에 있음

calibration, `alignment_confirmed`, 시작점 거리와 방향 오차는 출발을 막지
않는다. GNSS 위치나 헤딩이 없어 ENU pose 자체가 생성되지 않는 경우에는
대기한다. MCAP·카메라 기록 상태는 출발 및 주행 유지 조건에 포함하지 않는다.

주행 중 pose, 차량 feedback, path command 또는 safety 메시지가 끊기면 마지막
명령을 계속 보내지 않도록 브레이크와 mission fault가 걸린다. `X`, `S`, `Q`도
같은 정지 절차를 사용한다.
