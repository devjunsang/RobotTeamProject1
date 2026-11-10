# aris3_real_bridge

실물 E0509 를 `/aris3/real/*` 인터페이스로 내놓는 노드. PyQt GUI(`aris3_gui`)와 C++ Qt GUI(ARIS, `aris3_gui_cpp`)가
같은 노드·같은 안전 코드를 쓰도록 GUI 밖으로 뺐다. 로봇 쪽 코드는 `e0509_manual` 의 `RobotLink`·`bringup` 그대로다.

## 실행

```bash
ros2 launch aris3_real_bridge real_bridge.launch.py               # 기본 110.120.1.66:12345, 최저 높이 50 mm
ros2 launch aris3_real_bridge real_bridge.launch.py host:=192.168.0.10 z_min_mm:=80.0
```

로봇 접속(`dsr_bringup2`)은 `connect` 를 받았을 때 시작하고, 노드를 끄면 이 노드가 시작한 접속을 끊는다.

## 가상 모드 (실물 비슷한 시뮬레이션, 2026-11-10)

```bash
ros2 launch aris3_bringup aris3_real_sim.launch.py                                   # 월드 먼저
ros2 launch aris3_real_bridge real_bridge.launch.py mode:=virtual gripper:=service   # (GUI --mode real --sim 이 대신 띄움)
```

- `mode:=virtual`: 「접속」 이 실물 대신 두산 가상 컨트롤러(Docker)를 띄운다 (`virtual.VirtualBringupManager`). 주소 127.0.0.1.
- `gripper:=service`: 그리퍼를 DRL 대신 `/aris3/gripper/command` 로 (`virtual.ServiceGripper`). **실물 모드에서는 쓸 수 없다** (노드가 거부).
- 보정 파일은 `~/.config/aris3/sim_camera_calib.json` - 가상 모드에서 실물 보정 파일 경로를 주면 노드가 시작을 거부한다.
- RobotLink 의 안전 검사·도착 확인·정지·복구 코드는 실물과 같다. 가상 컨트롤러가 재현하지 못하는 것은 충돌 감지·외력 안전 정지·
  그리퍼 DRL·토크 - 실물 재시험 필요.

## 인터페이스

| 이름 | 타입 | 설명 |
|---|---|---|
| `/aris3/real/state` | `aris3_interfaces/RealRobotState` (10 Hz) | 접속·상태·모드·조작 허용·관절·TCP·그리퍼·집기 순서·보정 |
| `/aris3/real/connect`, `disconnect` | `std_srvs/Trigger` | 로봇 접속·해제 |
| `/aris3/real/stop` | `std_srvs/Trigger` | 정지. 조작 허용과 상관없이 언제나 받는다 |
| `/aris3/real/set_armed` | `std_srvs/SetBool` | 조작 허용. 120 s 명령이 없으면 자동으로 잠김 |
| `/aris3/real/set_hand_guide` | `std_srvs/SetBool` | 손으로 움직이기(수동 모드). 켜면 조작 허용도 잠김 |
| `/aris3/real/calibration` | `aris3_interfaces/RealCalibration` | 카메라 보정 ① 기억 / ② 기록 / 삭제 / 저장 / 불러오기 |
| `/aris3/real/motion` | `aris3_interfaces/RealMotion` (액션) | 조그·홈·복구 후 홈·그리퍼. 취소 = 정지 |
| `/aris3/real/pick_place` | `aris3_interfaces/RealPickPlace` (액션) | 큐브 위로·집기·고른 자리에 놓기·집어서 옮기기·여기 내려놓기. 취소 = 정지 |

## 명령줄 시험

```bash
ros2 topic echo /aris3/real/state --once
ros2 service call /aris3/real/connect std_srvs/srv/Trigger
ros2 service call /aris3/real/set_armed std_srvs/srv/SetBool "{data: true}"
ros2 action send_goal --feedback /aris3/real/motion aris3_interfaces/action/RealMotion \
  "{kind: 1, index: 2, delta: 10.0, speed_percent: 30.0}"            # TCP +Z 10 mm
ros2 service call /aris3/real/stop std_srvs/srv/Trigger
```

## 안전

- 이동 전 검사(관절 한계, 최저 높이, 도달 범위 150~750 mm, 한 번 이동 400 mm), 도착 확인, 그리퍼 토크 확인,
  복구 순서는 `RobotLink` 가 한다. 이 노드는 그 위에 조작 허용(armed)과 동작 하나씩만 받기를 더한다.
- 확인창(사람 확인)은 GUI 가 띄운다. 명령줄로 직접 부르면 확인창이 없으니 로봇 옆에서만 쓴다.
- 화면·명령줄의 정지는 인증된 비상정지 장치를 대신하지 않는다.
