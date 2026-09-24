# Course 07 통합 미션 CSV

`ros2_ws/src/hl_ku_core/routes/course_07_mission.csv`가 Course 07 전체 코스와
건국대 구간 시험이 함께 참조하는 원본이다. 8개 주차/종료 차선 조합을
`variant_id`로 구분하고, 경로 좌표·진행 방향·FSM 구역·이벤트를 한 표에 둔다.

| 열 | 런타임에서의 의미 |
|---|---|
| `variant_id` | P1/P2, T1/T2, F1/F2 경로 조합 선택 키 |
| `s_m,x_m,y_m` | 원본 전역 ENU 경로와 누적 거리 |
| `mission` | 차량 FSM이 사용하는 상태 구역 |
| `direction` | `1` 전진, `-1` 후진. 주차 cusp에서 정지 후 방향 전환 |
| `fsm_zone` | 코스 제작 단계의 구역 이름 |
| `fsm_event_ids`, `fsm_actions`, `fsm_conditions`, `fsm_hold_sec` | 위치에 도달할 때 활성화되는 이벤트 정보 |
| `direction_source` | 진행 방향의 출처/작성 상태 추적 |

기존 `S_CURVE`는 런타임의 LiDAR 회피 미션인 `S_OBSTACLE`로 변환한다. 다른
구역은 `ROUTE/BEND → NORMAL`, `T_PARK → PERP_PARK`를 포함해 동일 매핑을
`route.py`에서 관리한다. 경로 추종기는 현재 위치에 해당하는 미션 구역과 가장
최근 CSV 이벤트를 발행하고, 미션 관리자는 이를 FSM 동작에 사용한다. `HILL_STOP`
은 CSV의 위치와 hold 시간을 사용해 정지/대기/재출발한다. 신호등, 장애물,
차선 선택은 기존처럼 카메라/LiDAR 관측이 조건을 판정한다. 방향 전환 주차는
CSV의 `direction` 순서를 따른다.

TUI의 시나리오 1–3, 6–9는 모두 이 CSV에서 `source_variant_id`와 원본 `s_m`
구간을 선택한다. TUI는 선택 경로를 GNSS 현재 위치·방향에 회전/평행이동만 해
배치하고, 시나리오 속도를 적용해 실행용 CSV를 만든다. 축척은 바꾸지 않는다.
시나리오 3에는 8개 전체 코스 조합과 S자/T주차 전체 동작이 있고, 6–9에는
짧은 캠퍼스 시험 구간이 있다. 시나리오 6–9의 기본 형상은 P1/T1/F1이며,
T2 전체 주차는 해당 전용 형상(P1/T2/F1)을 고른다.

통합 CSV는 미리보기 경로 라이브러리이므로 `target_speed_mps`는 0으로 저장한다.
TUI가 실행 경로를 만들 때 설정한 시험 속도를 넣고, 구간 끝은 정지점으로 만든다.
직접 실행용 경로로 사용하지 않는다. 경로 형상·주차·FSM 경계의 실차 검증이
끝나기 전에는 전체 코스 주행 가능으로 간주하지 않는다.

평행주차 구간은 `parallel_parking_source.csv`의 `P_entry` 49번 점부터 시작한다.
`to_forward3`와 앞쪽 `P_entry`는 제외하고, P1·P2의 후진/출차 경로를 각각
원본 CSV의 `P1_reverse/P1_exit`, `P2_reverse/P2_exit`로 교체했다. 진입점과
출차점은 기존 Course 07 경로에 짧게 연결한다. 생성된 8개
`course_07_*_parallel_csv.csv`를 통합본이 읽으며, 평행주차 시작·전진→후진·
후진→전진·종료 FSM 이벤트와 `direction`도 새 경로 위치에 있다. 같은 생성
단계에서 HILL 정지 이벤트를 기존 위치에서 출발지 방향으로 경로상 1.5 m
옮겼다. P1/T1/F1 기준 `HILL_STOP`은 `s=34.1405 m`에서 `s=32.6405 m`로
바뀌었고, 정지 유지 시간 3초는 같다.

소스 CSV에서 통합본과 본선 실행 경로를 재생성하고 검증할 수 있다.

```bash
python3 operations/build_course_07_csv_parking.py
python3 operations/build_course_07_csv_parking.py --check
python3 operations/build_course_07_mission_csv.py
python3 operations/build_course_07_mission_csv.py --check
python3 operations/generate_mando_competition_routes.py --force
./operations/run_mando_competition_tui.sh --check-config
```

본선 시나리오 1은 생성된 `course_07_full.csv`를 사용하고, 주차 후보 8개는
통합 CSV에서 읽는다. 실행 전 파일 검사는 오래된 생성 경로가 남으면 중단한다.
현재 P1/T1/F1 전체 길이는 약 697.80 m다. 경로 연결과 FSM 전환은 소프트웨어로
검증했으며 주차 구획 여유 폭과 실제 차량 주행은 현장에서 검증해야 한다.

전체 FSM 구역 거리와 현장 구간별 시험 계획은
[MANDO_SEGMENT_TEST_PLAN.md](MANDO_SEGMENT_TEST_PLAN.md)에 있다.
