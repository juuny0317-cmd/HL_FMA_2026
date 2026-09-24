# 안테나 + GNSS RTK 확인 및 Enter 웨이포인트 기록

이 절차는 차량, NUCLEO, 구동모터, 조향모터, 카메라, LiDAR를 전혀 실행하지
않는다. 노트북에 UM982 USB와 인터넷만 연결해 야외에서 RTK 수신을 확인하고,
안테나를 원하는 지점에 놓은 뒤 Enter로 점을 저장하는 절차다.

## 1. 현장에 가져갈 것

- GNSS 수신기와 안테나(들)
- GNSS USB 케이블과 노트북
- NTRIP caster에 접속할 인터넷(휴대전화 핫스팟 등)
- 실제 caster host, port, mountpoint, username, password

안테나는 하늘이 넓게 보이는 위치에 두고 건물 벽, 나무, 차량 바로 옆처럼
다중경로가 큰 곳은 피한다. 듀얼 안테나 방향은 이번 위치 웨이포인트 기록의
필수 조건이 아니다. 위치용 주 안테나는 반드시 연결한다.

## 2. 최초 한 번: NTRIP 계정 입력과 빌드

아래 파일은 Git에 올라가지 않는 로컬 전용 파일이다.

```bash
cd $HOME/HL_KU/ros2_ws
nano src/hl_ku_core/config/ntrip_private.yaml
```

공급자가 준 값을 그대로 입력하고 `enabled`를 `true`로 바꾼다.

```yaml
ntrip_client:
  ros__parameters:
    enabled: true
    host: "공급자_CASTER_HOST"
    port: 2101
    mountpoint: "공급자_MOUNTPOINT"
    username: "공급자_ID"
    password: "공급자_PASSWORD"
    tls: false
    gga_period_sec: 5.0
    reconnect_sec: 3.0
```

공급자가 TLS 포트를 지정한 경우에만 해당 포트와 `tls: true`를 사용한다.
설정을 마친 뒤 빌드한다. 현재 PC에서는 이미 빌드되어 있으며 YAML만 수정할
때는 symlink install이므로 다시 빌드하지 않아도 된다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hl_ku_interfaces hl_ku_core
source install/setup.bash
```

## 3. 현장 터미널 1: GNSS + NTRIP만 실행

```bash
cd $HOME/HL_KU/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hl_ku_core rtk_check.launch.py
```

정상이면 이 터미널에 `NTRIP caster connected`가 출력된다. 아래 메시지는 각각
원인이 명확하다.

- `NTRIP is disabled`: `ntrip_private.yaml`의 `enabled`가 아직 `false`
- `401` 또는 `403`: 계정, 비밀번호 또는 사용 권한 문제
- `404` 또는 sourcetable 응답: mountpoint 문제
- 연결 timeout: 인터넷, host, port 또는 방화벽 문제
- serial open 오류: USB 포트 또는 다른 GNSS 프로세스와의 포트 충돌

이 터미널은 웨이포인트 기록이 끝날 때까지 켜 둔다. 같은 GNSS serial 포트를
여는 다른 launch는 동시에 실행하지 않는다.

## 4. 선택 확인: RTCM 데이터가 들어오는지 보기

새 터미널에서 다음을 실행한다.

```bash
cd $HOME/HL_KU/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /gnss/rtcm
```

주기가 계속 표시되면 caster에서 RTCM byte가 들어오는 것이다. 이것만으로 FIX가
보장되지는 않으므로 최종 판정은 다음 기록기의 `GNSS READY`로 한다. 확인을 마치면
Ctrl-C로 이 `topic hz`만 종료한다.

## 5. 현장 터미널 2: RTK 상태 확인 및 Enter 기록

`course_01`은 그날 사용할 고유한 이름으로 바꾼다. 같은 이름의 결과가 있으면
기록기는 덮어쓰지 않고 종료한다.

```bash
cd $HOME/HL_KU/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p $HOME/HL_KU/data
ros2 run hl_ku_core rtk_waypoint_recorder --ros-args \
  --params-file src/hl_ku_core/config/rtk_field_test.yaml \
  -p output_prefix:=$HOME/HL_KU/data/course_01
```

기록기는 1초마다 아래 상태를 출력한다.

```text
GNSS READY: fix=RTK_FIXED, sats=..., HDOP=..., corr_age=..., msg_age=..., saved=...
```

`READY`의 기본 합격 조건은 다음과 같다.

- `fix=RTK_FIXED` (`fix_type=4`)
- 유효한 NMEA checksum과 위치
- 위성 15개 이상
- HDOP 1.5 이하
- correction age 2.0초 이하
- 최신 위치 메시지의 나이 0.5초 이하

## 6. 점을 따는 방법

1. `GNSS READY`가 연속해서 표시될 때까지 기다린다.
2. 안테나 기준점을 원하는 웨이포인트에 놓고 최소 1초간 움직이지 않는다.
3. Enter를 한 번 누른다.
4. `WAYPOINT N SAVED`와 위치 분산(`spread`)을 확인한다.
5. 다음 지점으로 이동해 2~4를 반복한다.
6. 모든 점을 기록한 뒤 `Q`를 눌러 종료한다.

키 동작은 다음과 같다.

- `Enter`: 최근 1초의 합격한 RTK FIX 샘플을 평균내어 점 저장. 최신 표본이
  없으면 시간제한 없이 새 RTK FIX 표본을 기다린 뒤 자동 저장
- `U` 또는 Backspace: 마지막 점 취소
- `P`: 즉시 상태 출력
- `Q`: 저장 상태를 유지하고 종료

RTK FLOAT, 오래된 위치, 부족한 위성 수, 큰 HDOP/보정 나이에서 Enter를 누르면
`WAYPOINT WAITING`을 표시하고 최신 합격 표본을 시간제한 없이 기다린다.
RTK FIXED가 회복되면 자동 저장한다. 포인트 사이
최대 거리 제한은 없으며, 가까운 점도 허용한다. 완전히 같은 좌표만 0길이 경로
방지를 위해 무시한다. 대기 중 `U` 또는 Backspace를 누르면 저장 요청을 취소한다.
caster 연결은 유지되지만
RTCM payload가 3초 동안 오지 않으면 NTRIP client가 자동으로 다시 접속한다.

## 7. 생성되는 파일

위 예시는 아래 세 파일을 매번 Enter 즉시 갱신한다. 갑자기 프로그램이 종료돼도
마지막으로 합격한 점까지 남는다.

- `$HOME/HL_KU/data/course_01_wgs84.csv`: 원본 위도, 경도, 고도와 품질
- `$HOME/HL_KU/data/course_01_route.csv`: 첫 점을 `(0, 0)`으로 한 ENU 경로
- `$HOME/HL_KU/data/course_01_datum.yaml`: 첫 점 기준 datum

최소 두 점이 있어야 `_route.csv`를 경로로 사용할 수 있다. 마지막 점은 자동으로
`FINISH`, 목표속도 `0.0`이 된다. 원본 `_wgs84.csv`는 수정하지 말고 보관한다.

이렇게 손으로 딴 점은 안테나 위치의 경로다. 나중에 차량에 장착해 주행할 때는
차량 기준점에서 안테나까지의 lever arm을 실측하고, 급한 곡선은 점 간격을 더
촘촘하게 기록해야 한다. 차량 구동 전에는 별도의 저속 경로 검증 절차를 거친다.

## 8. 종료

기록기 터미널에서 `Q`, GNSS launch 터미널에서 Ctrl-C 순서로 종료한다. 종료 후
파일을 확인한다.

```bash
column -s, -t $HOME/HL_KU/data/course_01_wgs84.csv
column -s, -t $HOME/HL_KU/data/course_01_route.csv
cat $HOME/HL_KU/data/course_01_datum.yaml
```
