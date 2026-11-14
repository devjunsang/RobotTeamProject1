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
| `/aris3/real/pick_place` | `aris3_interfaces/RealPickPlace` (액션) | 큐브 위로·집기·고른 자리에 놓기·집어서 옮기기·여기 내려놓기·이어서 하기·쌓기(받침 큐브 위, 놓는 높이 + 큐브 한 개 + 여유 3 mm, 받침이 안 보이면 거부. 큐브 높이 = 파라미터 `cube_mm`, 0 이면 실물 40 / 가상 50 mm). 취소 = 정지 |

## 집기 보정 (2026-11-13)

| 기능 | 하는 일 | 끄기 |
|---|---|---|
| 그리퍼 각도 맞추기 | 큐브 위 접근 높이에서 J6 만 돌려 그리퍼 면을 큐브 면에 맞춘 뒤 집는다. 놓을 때는 큐브 면이 팔 방향(베이스 -> 놓을 자리)과 평행·수직이 되게, 쌓을 때는 받침 큐브와 같은 각도로 돌린다. 90° 대칭이라 최대 45°, 한 번 10° 씩 (`RobotLink.jog_joint`). 각도는 색 검출 `pose.orientation` (영상) -> 카메라 보정으로 로봇 각도 | `align_yaw:=false` |
| 쌓인 큐브 위치 보정 | 보정이 1층 윗면 높이 기준이라 2층 이상은 원근으로 밀려 보인다. 색 검출이 깊이로 윗면을 1층 높이로 내려 다시 투영한 픽셀(`grasp_u`, `grasp_v`)로 로봇 좌표를 계산 | (항상 - 0 이면 화면 위치) |
| 큐브 중심 맞추기 | 각도를 맞춘 뒤 접근 높이에서 새 검출 3 장의 중앙값으로 큐브 중심을 다시 재서 그 바로 위로 옮긴 다음 내려간다. 1.5 mm 미만이면 그대로, 25 mm 넘으면 큐브가 움직인 것으로 보고 멈춤. 넓이가 30 % 넘게 다른 장(그리퍼에 일부 가려짐)은 버리고, 못 재면 처음 좌표로 집는다 | `center_grasp:=false` |

전제: 그리퍼 중심이 J6 축 위에 있다 (손목을 돌려도 손끝 XY 가 그대로). 실물에서 처음 쓸 때는 「큐브 위로」(HOVER) 로
각도·중심만 맞춰 보고 눈으로 확인한다.

## 탁자 블록 마커 보정 (2026-11-14)

블록 윗면에 ArUco 마커(`DICT_4X4_50`, ID 0 - 파라미터 `marker_id`)를 천장을 보게 붙여 보정 ① ② 를 한다.

1. 블록을 탁자 위 한 자리에 두고 ARIS 「① 마커·큐브 픽셀 기억」 (클릭 필요 없음). 브리지가 새 영상 5 장에서
   마커 중심(대각선 교점, 0.1 px 단위)과 D455 깊이로 마커 3D 점·블록 높이(둘레 탁자 평면까지)를 잰다.
   마커가 여럿이면 화면에서 마지막으로 클릭한 자리에 가까운 것. 마커가 안 보이면 예전처럼 고른 큐브(60 px 안).
2. 조그·손으로 움직이기로 그리퍼를 그 블록을 집는 자세까지 내리고 「② 로봇 위치 기록」.
3. 자리를 바꿔 6~8 번 (영상 안에서 넓게 퍼지게), 「보정 저장」.

**블록 자동 기록 (로봇이 잡아서, 권장)**: 사람은 블록 자리만 옮긴다. ARIS 「블록 자동 기록 (로봇이 잡아서)」
(`RealCalibration AUTO_RECORD`) 을 누르면 브리지가

1. 마커 중심·각도를 재서 **지금 보정**으로 블록 위치를 계산 (오차 몇 mm - 손가락이 약 106 mm 벌어져 충분)
2. 블록 위로 -> 손목을 블록 각도에 맞춤 -> 열기 -> 하강 -> 닫기(한 축 가운데로) -> 열기 -> 올려 손목 90° -> 하강 ->
   닫기(다른 축도 가운데로) -> 잡았는지 확인 -> **그때 TCP 기록 = 블록 중심** -> 열기 -> 올리기 -> 홈
3. 홈에서 마커를 다시 재서(가운데 맞추며 블록이 밀렸을 수 있음) 자동 보정점 하나 추가

자동 보정점은 지금 보정과 따로 모은다 (지금 보정은 블록을 찾아가는 데 계속 씀). 블록 자리를 바꿔 6~8 번 한 뒤
「보정 저장」 = 자동 보정점으로 **새 보정** (예전 파일은 `.bak-시각`). 「마지막 점 삭제」 는 자동 보정점부터 지운다.
그리퍼가 무언가를 잡고 있거나, 마커가 안 보이거나, 블록이 도달 범위(150~750 mm) 밖이면 시작하지 않는다.
두 번째로 닫아도 빈손이면 기록하지 않고 멈춘다. 전제: 그리퍼 중심이 J6 축 위 (집기와 같은 가정).

잡은 상태(`holding`, 2026-11-14): 닫은 뒤 그리퍼 위치·토크로 판단한다 (`real_pick.judge_hold` - 100 ~ 700 에서 멈추면 잡음,
열려 있거나 끝까지 닫히면 빈손). 순서 안에서든 버튼으로든 그리퍼를 열면 빈손으로 바뀌고, 닫기 버튼도 판단을 갱신한다.
그리퍼 상태는 명령을 보냈을 때만 읽으므로, 잡은 뒤 큐브를 손으로 빼면 다음 그리퍼 명령까지는 「잡음」 으로 남는다.

블록 높이가 큐브(40 mm)와 5 mm 넘게 다르면 픽셀은 큐브 높이 윗면 중심으로, 집는 높이 Z 는 TCP z - (블록 - 40 mm) 로
고쳐 기록한다. 마커 3D 점(`cam`)이 4 점 이상 있으면 보정 요약에 「마커 3D 일관성 최대 … mm」 가 붙는다 -
한 점만 크면 그 점을 잘못 기록한 것(블록을 잡는 자세가 어긋남)이니 「마지막 점 삭제」 후 다시.

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
