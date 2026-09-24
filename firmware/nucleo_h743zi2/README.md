# NUCLEO-H743ZI2 펌웨어 이식

이 폴더는 CubeMX가 생성한 전체 프로젝트가 아니라, 그 프로젝트의 `Core/Inc`와
`Core/Src`에 넣는 하드웨어 독립 제어 모듈이다. 실제 핀은 아직 정해지지 않았기
때문에 임의로 지정하지 않았다.

## CubeMX 설정

1. 보드 `NUCLEO-H743ZI2`, H743 코어의 캐시/MPU와 400 MHz 클록을 설정한다.
2. Ethernet + LwIP UDP를 활성화하고 MCU 고정 IP를 `192.168.10.20/24`로 둔다.
3. MDD20A 구동용 PWM 타이머 1채널(권장 20 kHz)과 DIR GPIO를 정한다.
4. MD10C 조향용 PWM 타이머 1채널과 DIR GPIO를 정한다.
5. 조향 포텐셔미터와 배터리 분압용 ADC를 활성화한다. ADC 입력은 절대로
   3.3 V를 넘지 않게 한다.
6. 물리 비상정지는 소프트웨어 GPIO뿐 아니라 모터 전원 contactor도 직접
   차단하도록 구성한다.
7. 1 kHz 타이머에서 `hlku_app_tick_1khz()`를 호출하고, UDP 15000 수신 payload를
   `hlku_app_on_udp_receive()`에 전달한다. UDP 피드백은 PC의 15001번으로 보낸다.
8. `hlku_board_port.h`의 함수들을 실제 HAL/LwIP 코드로 구현한다.

`hlku_control_config_t`의 ADC 세 점, 좌/우 조향각, PID, duty 제한, 저전압 기준은
실측 전부가 끝나기 전까지 보수적으로 설정한다. watchdog 기본 권장값은 100 ms다.

MDD20A의 PWM=0은 동적 제동이지만 정지 경사에서 지속 유지토크를 보장하지 않는다.
경사 유지 duty는 실차 하중으로 측정하며, 신뢰할 수 없다면 별도 기계식 브레이크가
필요하다.

하드웨어 독립 제어부는 PC에서 다음 명령으로 watchdog, 명령 플래그, 저전압,
비상정지 latch를 회귀시험할 수 있다.

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -ICore/Inc Core/Src/hlku_protocol.c Core/Src/hlku_control.c \
  test/test_control.c -lm -o /tmp/hlku_control_test
/tmp/hlku_control_test
```
