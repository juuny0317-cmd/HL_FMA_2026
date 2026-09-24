# HL FMA 2026 규정 추적표

기준 문서: `HL FMA{1_5] 경기규정.pdf` (문서 표기 2026-08-14). 이 표는
“기초 구현”, “외부 인지기/현장 보정 필요”, “추가 구현 필요”를 구분한다.

| 규정/미션 | 현재 코드 상태 | 실차 완료 조건 |
|---|---|---|
| 출발 후 원격조작 금지 | 경기용 launch에 teleop 없음, 수동 arm은 출발 전에만 수행 | launch/process 목록 현장 확인 |
| 차선/중앙선/가장자리 접촉 | GNSS Stanley 100% 전진 추종 + 비교용 Pure Pursuit 진단 | 실제 차선 segmentation, road-boundary guard, 반복주행 |
| 경사 지정구역 완전정지 3초 | `HILL_APPROACH/HOLD/CLIMB`, 기본 3.2초 구현 | 정지점 s, 완전정지 임계값 실측 |
| 경사 50 cm 이상 밀림 방지 | active hold duty 인터페이스만 구현 | rollback 위치 감시 추가, 유지 duty 또는 기계식 브레이크 검증 |
| 경사 30초 내 통과 | 아직 별도 timeout 없음 | hill entry/crest timer와 복구 전략 추가 |
| S코스 T870 두 대 회피 | LiDAR corridor/gap steering 기초 구현 | 물체 군집/도로경계 결합, 모든 좌우 배치 시험 |
| 적색 정지선 준수 | RED/UNKNOWN 정지와 GREEN debounce 구현 | perspective 거리 보정, 좌회전 route 현장 검증 |
| 교차로 시간 감점 | 별도 20/30초 diagnostic 없음 | mission timer/log 추가 |
| 직각주차 | 전·후진 route 형식과 저속 제어 인터페이스만 구현 | 방향전환 정지, 확인선 판정, 현장 maneuver planner 필요 |
| 어린이 더미 앞 3초 정지 | `DUMMY_BRAKE/HOLD`, LiDAR 경로 장애물 3 m trigger, 3.2초 정지 후 fresh-clear 재출발 | 실차 제동거리·corridor 폭·좌우 출현 반복시험 필요 |
| 평행주차 | 전·후진 route 형식과 저속 제어 인터페이스만 구현 | 확인선 판정과 현장 maneuver planner 필요 |
| 종점 ↓/X 차선 선택 | perception topic/상태기계/safety gate 구현 | ↓/X camera model은 별도 학습·검증 필요 |
| 기능코스 1분 무동작 DNF | 아직 no-progress diagnostic 없음 | 자동 복구와 55초 경고 timer 추가 |
| 전체 지정시간 8분 | 아직 elapsed-time diagnostic 없음 | 전체 mission timer/log 추가 |
| 2026 신규 3종 감점, 도로이탈 위험 | 신규 상태는 보수 저속/정지로 설계 | road-boundary guard가 완성되기 전 속도 상향 금지 |
| 우천 대응 | speed/PWM/긴급거리 parameter 구조만 있음 | wet profile, 방수, 실제 제동거리 시험 필요 |

따라서 이 저장소는 배선·통신·좌표·상태기계·안전 게이트를 시작할 수 있는
기초 스택이지, 현재 상태 그대로 대회에 투입하는 완성본은 아니다. 특히 종점 신호
모델, 주차 확인선과 maneuver, road-boundary guard, 경사 rollback/timeouts는 실제
센서와 경기장 측정값을 받은 뒤 구현해야 한다.
