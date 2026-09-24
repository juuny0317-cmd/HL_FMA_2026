# 새 PC에서 HL Mando 환경 설치

## 준비할 PC

- Ubuntu 22.04 x86_64
- 인터넷 연결과 `sudo` 권한
- HL 차량에서 쓰던 UM982, NUCLEO-H723ZG, 카메라

설치 스크립트는 ROS 2 Humble, rosdep/colcon, Foxglove Bridge, USB 카메라,
MCAP 저장소, Python 모듈을 설치하고 워크스페이스를 빌드한다. 저장소에 포함된
Foxglove 확장도 사용자 폴더에 설치한다.

## 처음 한 번 실행

```bash
cd ~
git clone https://github.com/juuny0317-cmd/HL_FMA_2026.git
cd HL_FMA_2026
./operations/setup_new_pc.sh
```

Foxglove Desktop 패키지 저장소가 PC에 등록되어 있으면 앱도 자동 설치한다. 앱이
없고 apt에서 찾을 수 없으면 [Foxglove 공식 다운로드](https://foxglove.dev/download)에서
설치한다. ROS 2 연결과 MCAP 기록은 앱 설치 여부와 관계없이 준비된다.

설치 중 생성된 아래 파일에 NTRIP 공급자 계정을 입력한다.

```bash
nano ros2_ws/src/hl_ku_core/config/ntrip_private.yaml
```

이 파일은 Git에서 항상 제외된다. `host`, `mountpoint`, `username`, `password` 네
항목을 채우고 비밀번호를 채팅이나 GitHub에 올리지 않는다.

장치를 모두 연결한 다음 점검한다.

```bash
./operations/check_mando_pc.sh
```

소프트웨어 항목은 `[✓]`가 되어야 한다. GNSS, NUCLEO, 카메라는 연결되어 있을
때 `[✓]`가 된다. 설치 스크립트가 `dialout`, `video` 그룹을 처음 추가했다면 한
번 로그아웃한 뒤 다시 로그인한다.

## 장치 식별자가 다를 때

같은 실제 수신기, 보드, 카메라를 옮기면 현재 `/dev/.../by-id` 설정을 그대로
사용한다. 다른 장비를 쓰면 다음 명령으로 새 식별자를 확인한다.

```bash
ls -l /dev/serial/by-id/
ls -l /dev/v4l/by-id/
```

GNSS와 NUCLEO 경로는
`ros2_ws/src/hl_ku_core/config/gps_only_route.yaml`, 카메라 경로는
`operations/mando_scenarios.yaml`의 `camera_device`에서 바꾼다. `/dev/ttyUSB0`
같이 연결 순서에 따라 달라지는 이름보다 `by-id` 경로를 사용한다.

## Foxglove 처음 한 번 설정

확장은 자동으로 설치된다. Foxglove를 다시 시작한 다음 `Layouts`에서 아래
파일을 한 번 가져온다.

```text
ros2_ws/src/hl_ku_foxglove/config/HL_KU_FMA_Replay.json
```

LIVE 연결 주소는 `ws://localhost:8765`이다. TUI를 켜면 브리지가 함께 준비된다.

## 실험 시작

```bash
cd ~/HL_KU
./operations/run_mando_tui.sh
```

현재 건대 datum, 직선 경로, 곡선 경로와 Stanley 100% 설정은 저장소에 포함된다.
다른 위치에서 datum을 다시 잡으면 TUI의 `A` 적용 후 경로 시나리오에서 `R`을
누를 때 보정 파일의 datum 불일치를 감지하고 현재 RTK pose에 원본 경로 조각을
다시 배치한다. 축척은 항상 1.0이다. `3`번의 `S-FULL`, `T1-FULL`,
`T2-FULL`도 첫 실행 시 현재 pose에 맞춰 생성된다. T자 주차는 원본의
전진→후진→전진 방향을 유지한다.

MCAP은 자동으로 시작하지 않는다. TUI에서 `B`를 눌렀을 때만 `data/bags` 아래에
기록하고, 다시 `B`를 누르면 종료한다. `*.mcap`, NTRIP 개인 설정, 빌드 결과와
로그는 GitHub에 올라가지 않는다.

## 설치 없이 상태만 다시 확인

```bash
./operations/setup_new_pc.sh --check-only
```

이미 ROS와 의존성이 설치된 PC에서는 다음처럼 시스템 패키지 단계를 생략할 수
있다.

```bash
./operations/setup_new_pc.sh --skip-system-packages
```

검증 기준 환경은 Ubuntu 22.04, ROS 2 Humble, Python 3.10, Foxglove Desktop
3.1.x이다. ROS 및 Ubuntu 패키지는 보안 수정이 반영된 같은 배포판의 최신 버전을
설치한다.
