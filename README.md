# UWB-ShortRange-Follower
UWB-based precise short-range tracking system for autonomous agricultural robots using 2:2 multi-node communication and IMU data fusion. (2:2 다중 노드 통신 및 IMU 데이터 융합을 이용한 자율 주행 농업 로봇용 UWB 기반 단거리 정밀 추종 시스템)

# 🚜 UWB 기반 2:2 다중 노드 측위 및 센서 융합 추종 제어 시스템 (STM32)

![Project Status](https://img.shields.io/badge/Status-Completed-success)
![Platform](https://img.shields.io/badge/Platform-STM32-blue)
![Language](https://img.shields.io/badge/Language-C-orange)
![Tech](https://img.shields.io/badge/Tech-UWB%20%7C%20DS--TWR-lightgrey)

## 📌 프로젝트 개요
본 프로젝트는 **(주)두루기계**의 기업 연계 과제로, 대파 수확기(선행기)와 운반기(추종기) 간 **1m 간격의 단거리 자율 추종**을 위한 핵심 인지 시스템 구축을 목표로 함. 흙먼지와 진동이 심한 가혹한 농업 환경에서도 두 기체가 안정적으로 간격을 유지할 수 있도록, **다중 노드(2:2) UWB 통신망을 구축하여 물리적 거리와 앞차의 기동 상태(IMU)를 실시간으로 공유하는 견고한 아키텍처**를 설계 및 구현.


* **담당 역할:** UWB 센서 제어 및 통신 아키텍처 설계 (Firmware Engineer)

---

## 🎯 핵심 기여 및 문제 해결 (My Contributions)

단순한 거리 측정을 넘어, 제한된 하드웨어 리소스와 전파 간섭 환경에서 **'통신 충돌 방지'**와 **'실시간 센서 융합'**을 달성하는 데 집중.

### 1. 📡 실시간 거리 측정 및 센서 융합 전송 (DS-TWR & IMU Piggybacking)
* **문제:** 뒷차의 정밀한 추종 제어를 위해서는 UWB의 거리 데이터뿐만 아니라 앞차의 기울기/주행 상태(IMU 데이터)가 동시에 지연 없이 필요
* **해결:** 초정밀 하드웨어 타이머를 활용한 **DS-TWR(Double-Sided Two-Way Ranging)** 기법을 적용하여 거리 오차를 최소화. 또한, 통신의 첫 단계인 `POLL` 메시지의 페이로드 공간에 앞차의 **최신 IMU 데이터 12바이트를 탑재(Piggybacking)**하는 방식을 고안. 별도의 통신 채널 없이 하나의 UWB 패킷으로 거리 계산과 자세 데이터 전달을 동시에 처리하여 전체 시스템의 응답성을 극대화.

### 2. 🔄 다중 노드 전파 간섭 해소 (Token-Passing 알고리즘)
* **문제:** 좁은 구역에 4개의 UWB 노드(앞차 Init 1/2, 뒷차 Res 1/2)가 밀집해 있어, 송신 시점이 겹칠 경우 패킷 충돌(Collision)로 인한 치명적인 제어 오차가 우려되었음.
* **해결:** 마스터 노드(Init_1, Init_2) 간에 주도권을 교대로 넘기는 **토큰 패싱(Token-Passing)** 구조를 하드웨어 인터럽트 레벨에서 설계. 마스터가 자신의 통신 사이클(Res_1 → Res_2)을 마치면 즉각 `TURN` 메시지를 브로드캐스트하여 파트너에게 송신 권한을 이관함으로써, 병목 현상과 데이터 유실을 원천 차단.

### 3. 🛡️ 가혹 환경 대비 예외 처리 및 자가 복구 (Fail-Safe & Watchdog)
* **문제:** 농기계의 극심한 진동이나 순간적인 전원 불안정으로 파트너 노드가 먹통이 될 경우, TURN 메시지가 돌아오지 않아 전체 통신망이 무한 대기(Deadlock)에 빠질 위험이 있었음.
* **해결:** 대기 모드 진입 시 작동하는 **50ms의 소프트웨어 타임아웃 워치독(Watchdog)**을 구현. 타임아웃 발생 시 메인 마스터가 즉각 통신 주도권을 강제로 회수하고 통신 사이클을 스스로 재시작(`start_ranging`)하는 **자가 복구(Self-restart) 로직**을 통해 중단 없는 펌웨어 가용성(High Availability)을 확보.

---

## ⚙️ 시스템 동작 흐름 (System Architecture)


*(설명: 2:2 노드 배치 및 DS-TWR 통신 시퀀스 다이어그램)*

1. **[Init_1]** Res_1, Res_2를 향해 순차적으로 POLL(IMU 탑재) 발사 및 RESP, FINAL 수신 (DS-TWR 완료)
2. **[Init_1]** 1사이클 완료 후 `TURN` 메시지 브로드캐스트 → 수신 대기 모드 진입
3. **[Init_2]** `TURN` 수신 확인 후, 자신의 차례 시작 (Res_1, Res_2와 DS-TWR)
4. **[Init_2]** 완료 후 다시 Init_1에게 `TURN` 전달 (무한 루프)
5. **[Fail-Safe]** TURN 대기 중 50ms 초과 시 강제 주도권 회수 및 통신 재시작

---

## 🛠 기술 스택 (Tech Stack)
* **MCU:** STM32F4xx / STM32 시리즈
* **Language:** C (Embedded)
* **Sensor/Hardware:** UWB (DW3000), IMU, I2C/SPI Interface
* **Environment:** STM32CubeIDE

---

## 📂 주요 코드 구조 (Repository Structure)

```text
📁 src
 ┣ 📜 ds_twr_initiator_irq1.c  # Init_1 (Token-Passing Master) 메인 로직 및 IRQ 콜백
 ┣ 📜 ds_twr_initiator_irq2.c  # Init_2 (Token-Passing Slave) 제어 로직
 ┣ 📜 ds_twr_responder_irq.c   # Res_1, Res_2 (Responder) 제어 로직 및 IMU 데이터 수집부
 ┣ 📜 deca_spi.c / port.c      # UWB 칩셋 SPI 통신 및 하드웨어 포팅 레이어
 ┗ 📜 main.c                   # 시스템 초기화 및 타이머, 센서 인터럽트 구동


