# 노트북 키보드 수동시험

이 기능은 조향 방향, 모터 방향, PWM, Nucleo 통신을 확인하는 **정비·보정 전용**이다.
HL FMA 경기 출발 후에는 키보드·원격조작을 사용할 수 없으므로 자율주행
`bringup.launch.py`에는 포함하지 않았다.

현재 차량의 ST-LINK UART와 기존 조향 ADC 값으로 시험할 때는 이 문서의 UDP
`manual_test.launch.py` 대신 [GLOBAL_ROUTE_TEST.md](GLOBAL_ROUTE_TEST.md)의
`nucleo_manual_test.launch.py` 절차를 사용한다.

## 시험 전 조건

1. 물리 비상정지 담당자가 차량 옆에 있어야 한다.
2. 최초 시험은 구동륜을 지면에서 띄우고 진행한다.
3. MDD20A/MD10C 방향, 퓨즈, 조향 ADC와 Nucleo watchdog을 먼저 확인한다.
4. 노트북 Ethernet을 Nucleo 망에 연결하고 `ping 192.168.10.20`이 되어야 한다.
5. 전체범위 명령은 `finger_drive.yaml`에서 `1.0`으로 열리며 NUCLEO 값 100에
   해당한다.

## 실행

전체범위 손가락 주행은 `ros2_ws`에서 다음 한 명령으로 bridge, 안전감독과 키보드
노드를 같은 터미널에 실행한다. 키보드 노드는 terminal 표준입력을 직접 받으며,
`Q` 또는 `Ctrl+C`로 끝내면 함께 실행한 launch 노드도 종료된다.

```bash
./run_finger_drive.sh
```

이 스크립트는 `finger_drive.yaml`을 사용한다. 기본값은 첫 입력과 키당 증감 모두
10/100이다. `--step-pwm 20`처럼 키당 증감값을 바꿀 수 있으며 전체 구동범위는
-100~100이다.

아래 두 터미널 방식은 각 노드의 출력을 나누어 진단해야 할 때만 사용한다.

첫 번째 터미널에서 MCU bridge와 수동시험 안전 감독을 실행한다.

```bash
cd HL_FMA_2026/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hl_ku_core manual_test.launch.py drive_enabled:=true
```

두 번째 터미널에서 키보드 노드를 직접 실행한다. `ros2 launch` 아래에서 실행하면
표준입력 terminal을 받지 못할 수 있으므로 반드시 `ros2 run`을 사용한다.

```bash
cd HL_FMA_2026/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hl_ku_core keyboard_teleop \
  --ros-args \
  --params-file src/hl_ku_core/config/system.yaml \
  --params-file src/hl_ku_core/config/finger_drive.yaml
```

키 기능은 다음과 같다.

| 키 | 동작 |
|---|---|
| `W` | 현재 구동 PWM에 +10 누적 |
| `S` | 현재 구동 PWM에 -10 누적 |
| `A`, `D` | 좌/우로 0.048 rad씩 이동하고 현재 각도 고정 |
| `C` | 조향을 즉시 중앙으로 복귀 |
| `Space` | 즉시 제동 |
| `X` | 수동 비상정지 latch, 재시작 전 해제 불가 |
| `Q` | 제동 명령 후 종료 |

현재 입력 키, 실제 구동 명령, 실제 목표 조향각, PWM 증감 단위와 브레이크 상태는
터미널 한 줄에 계속 표시된다. 반대 방향 키를 누르면 매번 10 PWM씩 0에 가까워지고,
0을 지난 다음부터 반대 방향 PWM이 10씩 커진다. `Space`는 즉시 0과 브레이크를
보낸다. `A/D`는 누를 때마다 조향을 한 단계씩 누적하고 그 목표각을 유지한다.
`100/100`과 `±0.48 rad`는 각각 NUCLEO protocol과 차량 조향 설정의 전체 범위다.

시험 중 다음을 별도 터미널에서 확인한다.

```bash
ros2 topic echo /safety/status
ros2 topic echo /vehicle/feedback
```

종료 후에는 `manual_test.launch.py`도 중단하고 모터 전원을 끈다. 경기용 실행에는
반드시 `bringup.launch.py`만 사용한다.
