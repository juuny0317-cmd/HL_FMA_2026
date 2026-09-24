# HL_KU 운영 폴더

실차 운용에 필요한 스크립트와 주요 ROS 파일 바로가기를 한곳에 모은 폴더다.
ROS 2 Python 노드의 실제 위치는 패키지 규칙을 유지하기 위해 옮기지 않았다.

```text
operations/
├── setup_new_pc.sh            새 Ubuntu PC 의존성 설치·빌드·확장 설치
├── check_mando_pc.sh          소프트웨어·설정·USB 장치 읽기 전용 점검
├── apt-packages.txt           재현 가능한 Ubuntu/ROS 의존성 목록
├── setup_env.bash              공통 ROS/워크스페이스 환경설정
├── run_finger_drive.sh         WASD 수동주행
├── run_gnss_drive.sh           GNSS 웨이포인트 주행
├── rtk_check.sh                UM982/NTRIP 연결 확인
├── record_waypoints.sh         코스 이름을 받아 웨이포인트 기록
├── rviz_waypoint_recording.sh  기록용 RViz
├── rviz_route_alignment.sh     경로 정합용 RViz
├── record_all_topics.sh        전체 rosbag 기록
├── replay_foxglove.sh          안전 토픽만 재생하고 Foxglove 대시보드 실행
├── run_mando_tui.sh            RTK/시나리오/속도/기록 통합 TUI
├── mando_scenarios.yaml        만도 TUI 번호/구간/경로 설정
├── prepare_mando_route.sh      원본 축척을 유지한 구간 절단·ENU 배치
├── build_course_07_csv_parking.py  평행주차 CSV 결합·HILL_STOP 1.5m 앞당김
├── build_course_07_mission_csv.py  8개 형상/FSM 이벤트를 통합 CSV로 생성
├── apply_course_07_mission_tags.sh 미리보기 FSM 구간을 실차 경로 mission 태그로 병합
├── MANDO_TUI.md                만도 TUI 준비·실험·명령어 안내
├── NOTION_COMMANDS.md          복사·붙여넣기용 명령어 문서
├── nodes -> ...                실제 Python 노드 폴더 바로가기
├── launch -> ...               launch 파일 바로가기
├── config -> ...               YAML 설정 바로가기
├── routes -> ...               차량용 CSV 경로 바로가기
└── rviz -> ...                 RViz 설정 바로가기
```

다른 PC에 처음 설치할 때는 저장소 루트에서 다음을 실행한다.

```bash
./operations/setup_new_pc.sh
```

전체 순서는 [새 PC 설치 문서](../docs/SETUP_NEW_PC.md)를 따른다.
course 07의 FSM별 전체 구간, 정확한 원본 `s`, 진행방향과 풋프린트는
[구간별 시험 계획](../docs/MANDO_SEGMENT_TEST_PLAN.md)에 정리되어 있다.
전체 코스와 건대 분할 시험이 공유하는 CSV 구조는
[통합 미션 CSV 안내](../docs/COURSE_07_MISSION_CSV.md)에 있다.

새 터미널에서 직접 `ros2` 명령을 사용할 때만 다음을 실행한다.

```bash
cd HL_FMA_2026/operations
source ./setup_env.bash
```

나머지 실행 스크립트는 환경설정을 내부에서 수행한다. 기존
`ros2_ws/run_finger_drive.sh`, `ros2_ws/run_gnss_drive.sh`도 호환용으로 남긴다.

Foxglove 재생 데이터가 저장소 밖에 있다면 실행 전에 위치를 지정한다.

```bash
export HLKU_DATA=/absolute/path/to/HL_KU_RTK
./replay_foxglove.sh "$HLKU_DATA/bags/fma_full_20260905_152721" 1.0
```
