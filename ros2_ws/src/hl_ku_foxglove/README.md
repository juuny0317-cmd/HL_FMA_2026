# HL KU Foxglove live / replay

이 패키지는 `fma_full_20260905_152721`처럼 수동으로 기록한 전체 코스
rosbag을 실차 출력 없이 재생하고 전용 `HL KU FMA Dashboard` 패널에서
다음 항목을 한 화면에 표시한다.

- 카메라 원본과 추후 YOLO 오버레이
- 기록 웨이포인트, 차량 기준 경로, GNSS 실제 이동 궤적
- 현재 위치와 경로 최근접점, 진행 거리 `s`, 횡오차(CTE)
- 경로 CSV의 미션 태그와 실제 `/mission/status`를 구분한 9개 FSM 카드
- GNSS 품질, 조향·구동 피드백, 브레이크 상태
- 분리된 속도·조향·CTE 그래프
- ±10초, 드래그 탐색, 재생/일시정지, 0.5/1/2배속 제어 바

현재 rosbag 정합·CTE 기준은 기존 `course_07_vehicle.csv`를 유지한다. 실제
`/mission/status`가 없는 구간의 FSM은 현재 GNSS 위치를 8개 통합 경로에
투영하고 가장 가까운 경로의 `fsm_zone` waypoint 태그를 사용해
`INFERRED_FROM_WAYPOINT`로 표시한다. 실제 상태가 들어오면 그 값이 우선한다.

만도 TUI 실행 중에는 같은 레이아웃이 `LIVE DRIVE` 모드로 전환된다. TUI가
C270의 `/camera/image_raw`, GNSS의 `/localization/gnss_pose`와
`ws://localhost:8765` Foxglove bridge를 함께 시작하므로 현재 영상과 위치를
표시한다. `run_mando_tui.sh`를 실행하면 Foxglove 앱도 같은 주소로 자동
연결된다. live 모드에서는 rosbag 탐색 바를 숨긴다.

현재 건대 경로와 실제 궤적 옆에는 `시험장 원본 위치` 지도가 함께 표시된다.
청록색은 선택한 원본 조각, 노란 점은 `course_07` 690.37 m 중 현재 위치다.
FSM 카드는 이 원본 `s`에 해당하는 예상 FSM을 강조하고, 실제
`/mission/status`는 별도로 표시하므로 두 상태를 혼동하지 않는다. YOLO
overlay가 없을 때 두 번째 영상은 `RAW FALLBACK (YOLO 미연결)`로 명시해
현재 카메라 연결 자체는 계속 확인할 수 있다.

`운행 · FSM` 탭의 기존 Local/Overview 두 패널 영역은 하나의 8경로 통합
지도로 사용한다. 지도 위 `01`~`08` 버튼으로 강조할 경로를 바꿀 수 있다.

## 8개 경로 연결

`course_07_parking_overlay/course_07_p*_t*_f*_parallel_csv.csv` 8개가
`launch/course_07_replay.launch.py`에서 다음 토픽에 연결된다.
기본 활성 전체 경로도 새 P1/T1/F1 실행 경로 `course_07_full.csv`를 사용한다.

```text
/visualization/route_variant_01
...
/visualization/route_variant_08
```

각 CSV의 `fsm_zone`, `mission`, `direction`, `fsm_event_ids`를 연속 waypoint
구간으로 묶은 메타데이터는 아래 transient-local 토픽으로 함께 발행한다.

```text
/visualization/route_variant_metadata
```

`8경로 통합 점검` 탭에서 경로 카드를 선택하면 선택 경로는 미션 구간별 색으로
표시되고, 오른쪽 목록에서 CSV waypoint index, 누적 거리 `s`, 전진/후진과
경계 이벤트를 확인할 수 있다. `HILL_STOP` 위치는 지도 위 빨간 표적과
`HILL STOP · 3.2 s` 라벨로 표시된다. 다른 7개 경로는 옅은 비교선으로 남는다.

운행 중 선택된 단일 경로는 기존 알고리즘 토픽
`/planning/reference_path`를 사용한다. Foxglove 레이아웃에는 이 토픽도 미리
등록돼 있다.

8개 파일은 형상과 FSM 앵커 확인용이며 `target_speed_mps=0`이다. 실차 주행용
경로로 사용하지 않는다.

## 실행

```bash
export HLKU_DATA=/absolute/path/to/HL_KU_RTK
cd HL_FMA_2026
./operations/replay_foxglove.sh \
  "$HLKU_DATA/bags/fma_full_20260905_152721" 1.0
```

Foxglove에서 최초 한 번 `Layouts` → `Import from file...`로 아래 파일을
불러온다.

```text
HL_KU/ros2_ws/src/hl_ku_foxglove/config/HL_KU_FMA_Replay.json
```

전용 패널이 보이지 않으면 데스크톱 Foxglove용 확장을 설치한 뒤 앱을 다시
시작한다.

```bash
cd HL_FMA_2026/ros2_ws/src/hl_ku_foxglove/foxglove-extension
pnpm install --frozen-lockfile
npm run local-install
```

배포용 확장 파일은 `foxglove-extension/hlku.hl-ku-fma-dashboard-0.1.2.foxe`다.
하단 탐색 바는 `/rosbag2_player/seek`, `toggle_paused`, `set_rate` 서비스를
호출하므로 `replay_foxglove.sh`로 실행했을 때 사용할 수 있다.

재생 스크립트는 기록된 `/vehicle/actuator_command_raw`와
`/vehicle/actuator_command_safe`를 명시적으로 제외한다. 실차 구동 노드도
시작하지 않는다.
