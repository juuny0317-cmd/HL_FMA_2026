# NUCLEO-H723ZG 조향 + RPLIDAR A3 통합 펌웨어

현재 HL Mando 차량의 NUCLEO-H723ZG에 기록된 펌웨어와 같은 복구 자료다. 정상
차량에서는 보드에 이미 펌웨어가 있으므로 새 노트북을 연결할 때 다시 플래시할
필요가 없다. PC 쪽 TUI가 `nucleo_serial_mux`를 실행해 ST-LINK VCP 한 포트를
`/tmp/nucleo-control`과 `/tmp/nucleo-lidar`로 나눈다.

## 포함 파일

- `nucleo_h723zg_steering_lidar_mux.bin`: 보드에 검증된 통합 바이너리
- `nucleo_h723zg_steering_lidar_mux_source.tar.gz`: STM32CubeIDE 원본 프로젝트
- `nucleo_h723zg_cn9_rplidar_uart_marked.png`: CN9 라이다 UART 배선 그림
- `SHA256SUMS`: 위 파일의 무결성 확인값

무결성은 다음처럼 확인한다.

```bash
cd firmware/nucleo_h723zg
sha256sum -c SHA256SUMS
```

## 통신 구성

- USART2: RPLIDAR A3, 256000 baud, 8N1, 3.3 V TTL
- USART3 / ST-LINK VCP: PC multiplex 통신, 921600 baud
- multiplex channel 1: 조향·구동 제어
- multiplex channel 2: 라이다 데이터
- 프레임 무결성: CRC16

라이다 배선은 다음과 같다.

- A3 TX 노란색 → CN9-4 / D52 / PD6 / USART2_RX
- A3 RX 초록색 → CN9-6 / D53 / PD5 / USART2_TX
- A3 GND 검은색 → NUCLEO 및 라이다 전원과 공통 GND
- 라이다 모터와 전원은 별도 USB-TTL에서 공급

## 보드 복구

보드 교체 또는 펌웨어 손상 때만 STM32CubeProgrammer에서 바이너리를 flash 주소
`0x08000000`에 기록한다. 기존 보드가 정상 통신 중이면 덮어쓰지 않는다. 기록 후
저장소 루트에서 다음 순서로 확인한다.

```bash
source operations/setup_env.bash
ros2 run hl_ku_core nucleo_serial_mux
```

다른 터미널에서:

```bash
python3 - <<'PY'
import serial
with serial.Serial('/tmp/nucleo-control', 115200, timeout=0.5) as port:
    port.write(b'PING\n')
    print(port.readline().decode().strip())
PY
```

정상 응답은 `PONG`이다. `/dev/ttyACM0`을 다른 프로그램에서 동시에 직접 열면
multiplex 통신과 충돌하므로 조향은 `/tmp/nucleo-control`, 라이다는
`/tmp/nucleo-lidar`만 사용한다.
