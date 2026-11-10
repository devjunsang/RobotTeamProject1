# ARIS3 실행·테스트 가이드 (시뮬레이션, 2026-11-09)

GUI 하나(ARIS3 Control Station)로 Gazebo 시뮬레이션을 띄우고 Pick & Place 를 시험하는 순서.
최종 목표(손짓으로 물체를 가리켜 옮기기)의 1단계인 **「물체 이름 + 좌표에 놓기」** 까지 포함한다.

---

## 0. 처음 한 번만 (새 PC)

```bash
# 그리퍼 모델 (ROBOTIS)
cd ~/ros2_ws/src && git clone -b jazzy https://github.com/ROBOTIS-GIT/RH-P12-RN-A
# 프로젝트 패키지를 워크스페이스에 연결 (저장소 경로에 맞게)
for p in aris3_interfaces aris3_bringup aris3_moveit_config aris3_sim_bridge aris3_task_manager \
         aris3_voice aris3_gui e0509_gripper_description; do
  ln -sfn ~/workspace/team/RobotTeamProject1/$p ~/ros2_ws/src/$p; done
# 음성 인식 (선택)
python3 -m venv --without-pip ~/venvs/mark1_voice
python3 -m pip --python ~/venvs/mark1_voice/bin/python install faster-whisper "numpy<2"
```

## 1. 빌드 (코드를 받거나 바꾼 뒤마다)

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select aris3_interfaces            # 메시지 (--symlink-install 없이)
source install/setup.bash
colcon build --symlink-install --packages-select rh_p12_rn_a_description e0509_gripper_description \
  aris3_bringup aris3_moveit_config aris3_sim_bridge aris3_task_manager aris3_voice aris3_gui
```

> `aris3_interfaces` 가 바뀌면(이번에 ExecuteTask 에 항목 추가) **반드시 다시 빌드**하고,
> 실행 중인 노드는 모두 껐다 켠다. 예전 Task Manager 가 떠 있으면 새 요청을 모른다.

## 2. 실행

```bash
source ~/ros2_ws/install/setup.bash
export GZ_IP=127.0.0.1                  # 네트워크가 바뀌어도 Gazebo 내부 통신이 끊기지 않게
ros2 run aris3_gui control_station      # (음성 없이: --no-voice / 순수 마찰 파지: --no-assist)
```

1. 오른쪽 위 **Connect** → Gazebo·RViz 창이 뜬다. **이 창들은 닫지 않는다** (닫으면 시뮬레이션이 꺼짐).
2. 왼쪽 **시스템 상태** 가 초록불이 될 때까지 30~40 초 기다린다 (Task Manager 가 마지막).
   음성 인식은 `--no-voice` 로 띄웠으면 빨간불이 정상.
3. 카메라 화면에 큐브 3개: **빨강 12시, 파랑 3시(로봇 정면), 노랑 9시**.
   화면은 위쪽이 로봇 정면(+X), 왼쪽이 +Y.

> 실행 순서(자동): Gazebo → 컨트롤러 준비 확인 → MoveIt → MoveIt 준비 확인 → Task Manager (+ 음성).
> GUI 없이 띄우려면 `ros2 launch aris3_bringup aris3_sim_full.launch.py`.

## 3. 테스트

| # | 시험 | 하는 법 | 기대 결과 |
|---|---|---|---|
| T1 | **화면에서 고르기 (신규)** | 카메라 화면에서 **노란 큐브 클릭** → 빈 탁자 위 **아무 곳 클릭** → 「▶ 선택한 물체를 그 자리로」 | 청록 점선이 큐브를, 청록 원·십자가 놓을 자리를 표시 → 그 자리에 놓이고 「완료: (x, y) 중심에서 ○ mm」 |
| T2 | 작업 공간 밖 (신규) | T1 과 같이 하되 놓을 자리를 **탁자 맨 끝 모서리** 근처로 | 로봇이 움직이지 않고 PLAN 단계에서 거부 (「허용 범위 밖」 또는 「닿지 않음」) |
| T3 | 겹치는 자리 (신규) | 다른 큐브를 고르고 **이미 큐브가 있는 자리**를 클릭 | PERCEIVE 단계에서 「이미 ○○ 가 있음 - 움직이지 않음」 |
| T4 | 계획만 | 작업 큐 「계획만」 체크 + `파랑 큐브 → 구역 A` 추가 → **Start Task** | 로봇은 가만히, PLAN 까지 ✓ |
| T5 | 구역에 놓기 | 「계획만」 해제 → `빨강 큐브 → 구역 B` → **Start Task** | VERIFY ✓ 「구역 B 중심에서 ○ mm」 |
| T6 | 문장 명령 | 음성 패널 입력창 `노란 거 집어줘` → 해석 → 놓을 곳 선택 → **실행** | 「노랑 · 집어서 옮기기 ✓」 후 실행 |
| T7 | 음성 | **🎤 누르고 말하기** 누른 채 「파란 큐브 집어줘」 → 떼기 | 인식 문장·해석 표시 → 실행 |
| T8 | 수동 조작 | 「조작 허용」 체크 → TCP `+Z`, Joint `J1 +` → `⌂` | 한 번에 10 mm / 10° |
| T9 | 정지 | 작업 중 **Space** 또는 빨간 「정지」 | 작업 취소, 팔 멈춤 |

- 큐브를 여러 번 옮긴 뒤 처음 자리로: `bash scripts/reset_aris3_cubes.sh` (그리퍼가 비어 있을 때)
- 큐브를 잡은 채 작업이 끝나면 새 작업이 거부된다 (안전 장치). 「조작 허용」 → 그리퍼 **열기** → 위 스크립트.

### 명령줄로 같은 시험 (GUI 없이)

```bash
ros2 run aris3_task_manager send_task --id blue_cube --at 0.30,-0.20     # 물체 이름 + 좌표 (world, m)
ros2 run aris3_task_manager send_task --id red_cube --at 1.20,0.0        # 작업 공간 밖 -> 거부
ros2 run aris3_task_manager send_task yellow C                           # 기존 방식: 색 + 구역
ros2 run aris3_task_manager send_task blue A --dry-run                   # 계획만
```

## 4. 종료

GUI 의 **Disconnect** → 창 닫기. (창만 닫으면 「시뮬레이션도 끌까요?」 를 묻는다 — **예** 를 누를 것.
「아니오」 를 고르면 시뮬레이션이 계속 돌고, 다음 Connect 때 두 개가 겹친다.)

남은 것이 있는지 확인·정리:
```bash
ps -eo pid,args | grep -E "[g]z sim|[m]ove_group|[t]ask_manager --ros|[v]oice_node"
pkill -f "ros2 launch aris3_bringup"; pkill -x ruby; pkill -x move_group; pkill -x task_manager
```

## 5. 문제가 생기면

| 증상 | 원인·조치 |
|---|---|
| 시스템 상태가 오래 빨간불 | `/tmp/aris3_gui_sim.log` 확인 |
| 「컨트롤러 초기화 시간 초과」, Task Manager 가 「같은 이름의 액션 서버가 이미 실행 중」 | 이전 시뮬레이션이 남아 있음 → 4장의 정리 명령 후 다시 Connect |
| 새 기능(화면 선택)이 「모르는 색 ''」 로 실패 | 예전 Task Manager 가 떠 있음 → 정리 후 다시 시작 |
| 음성이 「인식되지 않음」 만 나옴 | `amixer -c 1 get Capture` 가 `[on]` 인지, 너무 작게 말하지 않았는지 |
| 순수 마찰 파지(`--no-assist`)에서 놓기 오차가 큼 | 알려진 한계 (운반 중 미끄러짐, 약 37 mm). 시연은 기본값(파지 보조) |

## 5-1. 실물 모드 음성·손짓 (2026-11-09)

처음 한 번 (손짓 = MediaPipe, 가상환경):
```bash
python3 -m venv --without-pip ~/venvs/aris3_gesture
python3 -m pip --python ~/venvs/aris3_gesture/bin/python install mediapipe "numpy<2" pyyaml
mkdir -p ~/.config/aris3/models && curl -fsSL -o ~/.config/aris3/models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
ln -sfn ~/workspace/team/RobotTeamProject1/aris3_gesture ~/ros2_ws/src/aris3_gesture
```

빌드 (HandGesture 메시지 추가 -> **GUI·색 검출을 끈 뒤** 인터페이스부터):
```bash
cd ~/ros2_ws && colcon build --packages-select aris3_interfaces && source install/setup.bash
colcon build --symlink-install --packages-select aris3_gesture aris3_gui
```

실행:
```bash
python3 scripts/aris3_usb_camera_pub.py            # 위쪽 카메라
python3 scripts/aris3_color_detector.py            # 큐브 색 검출 (픽셀)
ros2 run aris3_gui control_station --mode real     # 음성·손짓 노드도 함께 뜸 (--no-voice / --no-gesture)
```
기록: `/tmp/aris3_gui_voice.log`, `/tmp/aris3_gui_gesture.log`. 전제: 카메라 보정 4점 이상, 「조작 허용」 체크.

**손짓** (카메라 화면에 분홍 점 = 검지 끝, 청록 고리 = 멈춘 시간)

| 손짓 | 동작 |
|---|---|
| ☝ 검지로 큐브를 가리키고 1 초 멈춤 | 그 큐브 선택 (클릭과 같음) |
| ☝ 빈 곳을 가리키고 1 초 멈춤 | 놓을 자리 선택 (청록 원) - **손끝을 판에 대고** 가리켜야 오차가 작다 |
| ☝ 가리키다가 ✊ 주먹 | 고른 큐브 집기 확인창 |
| ☝ 가리키다가 🖐 손바닥 펴기 | 고른 자리에 놓기 확인창 |

처음부터 주먹/펼친 손으로 들어오면 아무것도 하지 않는다 (가리킴 → 확정 순서만 인정).
확인창의 「예」 는 마우스로 누른다 = **손이 작업 공간에서 빠진 뒤에** 로봇이 움직인다.

**음성**

| 말 | 동작 |
|---|---|
| 「빨간 큐브 집어줘」 | 그 색 검출이 **하나**일 때 그 큐브를 고르고 집기 확인창 |
| (큐브 고른 뒤) 「이거 집어줘」 | 고른 큐브 집기 확인창 |
| (큐브·자리 고른 뒤) 「이거 저기로 옮겨줘」 | 집어서 고른 자리에 놓기 (잡지 못하면 옮기지 않음) |
| (들고 있을 때) 「저기 놔줘」 | 고른 자리에 놓기 |
| 「내려놔」 | 지금 위치 바로 아래로 내려놓기 |
| 「집으로 돌아가」, 「홈으로 가」, 「원위치」 | 홈 자세 [0, 0, 90, 0, 90, 0]° 로 (⌂ Home 버튼과 같음) |

음성·손짓 모두 로봇을 바로 움직이지 않고 **항상 확인창**을 거친다. 같은 색이 여러 개면 색 명령은 거부 → 클릭/손짓 후 「이거」.

## 5-2. 실물과 비슷한 시뮬레이션 - 실물 로봇 없이 실물 경로 시험 (2026-11-10)

실물 모드(GUI -> `aris3_real_bridge` -> RobotLink -> 두산 서비스)를 로봇 없이 시험한다. 팔은 **두산 가상 컨트롤러**
(Docker DRCF 에뮬레이터)가 실물과 같은 서비스로 움직이고, Gazebo 의 ARIS3 로봇이 그 관절을 따라간다.

```bash
# 터미널 1: 월드 (Gazebo + 실물 비슷한 카메라 + 색 검출 + 그리퍼). 「[PASS] ARIS3 실물 비슷한 시뮬레이션 준비 완료」 까지 기다림
ros2 launch aris3_bringup aris3_real_sim.launch.py
# 터미널 2: GUI 실물 모드(가상). 제목·모드에 「가상」 이 보여야 함 -> Connect (가상 컨트롤러 Docker 가 뜸, 약 10초)
ros2 run aris3_gui control_station --mode real --sim --no-gesture
# (보정 빠르게) 조작 허용 -> ⌂ Home 으로 팔을 카메라 밖으로 뺀 뒤
python3 scripts/aris3_sim_auto_calib.py        # ~/.config/aris3/sim_camera_calib.json (실물 보정과 따로)
ros2 service call /aris3/real/calibration aris3_interfaces/srv/RealCalibration "{command: 5}"   # 불러오기
```

| 실물 | 이 시뮬레이션 |
|---|---|
| 두산 컨트롤러 (`dsr_bringup2 mode:=real`) | 두산 가상 컨트롤러 (`mode:=virtual`, Connect 가 띄움) |
| RH-P12-RN-A 그리퍼 (DRL) | Gazebo RH-P12-RN-A (`/aris3/gripper/command`) |
| D455 JPEG 1280x720, 탁자 위 약 0.8 m, 영상 위 = 로봇 +X | Gazebo 카메라 1280x720, 0.85 m, 같은 방향·축척(약 1.17 mm/px), JPEG 만 |
| `scripts/aris3_color_detector.py` | 같은 스크립트 |
| 보정 `real_camera_calib.json` | `sim_camera_calib.json` (가상 모드는 실물 보정 파일을 쓰지 않음 - 브리지가 막음) |

**실물과 다른 점 (실물 재시험에서 확인할 것)**: 가상 컨트롤러는 충돌 감지·외력 안전 정지·관절 브레이크를 재현하지 않고
Gazebo 팔은 탁자·큐브를 뚫고 지날 수 있다(최저 높이 검사는 그대로). 그리퍼는 DRL·토크 확인이 아니라 서비스.
큐브 5 cm(실물 4 cm), 시뮬레이션 탁자는 x ±0.70 m (실물보다 작음 - 가장자리에 큐브를 놓으면 떨어짐).
홈 자세의 팔이 카메라 중앙 아래쪽을 가린다 (x 0.45 m 안쪽 큐브가 안 보일 수 있음).

2026-11-10 결과 (처음부터 다시 띄워 확인, 12/12): 접속 10 초, 홈 이동, 자동 보정 오차 평균 1.6 / 최대 6.6 mm,
「집어서 옮기기」 30 초로 빨강 큐브를 목표 (470, 95) mm 에 놓음 - Gazebo 정답 위치 오차 5.1 mm, 이동 중 취소 = 정지.

## 6. 지금 상태와 다음 단계 (손짓 상호작용)

| 단계 | 상태 |
|---|---|
| 1. Task Manager: 물체 이름 + 좌표에 놓기 | ✅ (위 T1~T3) |
| 2. `aris3_gesture`: MediaPipe → 손가락 끝·손짓 (실물 위쪽 카메라) | 1차 구현, 실물 손 시험 전 |
| 3. GUI: 손 포인터 → 선택·집기·놓기 (실물 모드, 카메라 보정 사용) | 1차 구현, 실물 시험 전 |
| 4. 실물: D455 가 손과 물체를 함께 보는 구조 | 이후 |
