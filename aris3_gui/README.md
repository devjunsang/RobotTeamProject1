# aris3_gui — ARIS3 Control Station

마스터플랜 07장 목업 배치의 통합 관제 화면(PyQt5). 시뮬레이션과 실물 로봇을 모두 다룬다.

```
┌ Connect · Disconnect · Home · Start Task · Stop · 정지(Space/Esc) ──────────────────────────┐
│ 시스템 상태   │ 카메라 Top View + 검출 표시          │ 작업 공간 2D (위에서 본 지도)       │
│ 로봇 상태     │                                       │ 작업 큐 / 단계 진행 (VALIDATE~VERIFY)│
│ 그리퍼 상태   ├───────────────────────────────────────┼───────────────┬─────────────────────┤
│ 카메라 정보   │ 로그                                  │ 음성 명령     │ 수동 조작 (TCP/관절) │
└───────────────┴───────────────────────────────────────┴───────────────┴─────────────────────┘
```

## 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 run aris3_gui control_station              # 시뮬레이션 모드: 「Connect」 = Gazebo -> MoveIt -> Task Manager -> 음성
ros2 run aris3_gui control_station --no-assist  # 순수 마찰 파지 (물체 붙이기 보조 끔)
ros2 run aris3_gui control_station --mode real  # 실물 E0509 (etc/e0509_manual 의 접속·안전 코드 사용)
```

이미 다른 터미널에서 시뮬레이션을 띄웠다면 GUI 는 그것을 그대로 쓴다 (「외부에서 실행한 시뮬레이션」).
시뮬레이션 전체를 GUI 없이 띄우려면 `ros2 launch aris3_bringup aris3_sim_full.launch.py`.

## 쓰는 법

| 영역 | 동작 |
|---|---|
| 작업 큐 | 색·구역을 고르고 `+ 추가` → `Start Task`. 큐의 작업을 차례로 `/aris3/execute_task` 로 보낸다. 실패하면 큐를 멈춘다. `계획만`은 로봇을 움직이지 않는 dry run |
| 음성 명령 | `누르고 말하기` 를 누른 채 말하고 떼면 인식·해석 결과가 나온다. 문장 입력창으로도 시험할 수 있다. 결과가 ✓ 이면 `실행` (또는 `자동 실행` 체크). 놓을 구역은 옆 선택 상자 |
| 수동 조작 | `조작 허용` 을 체크해야 버튼이 열린다 (2분 동안 명령이 없으면 다시 잠김). 시뮬레이션은 MoveIt 으로 계획해서 움직이므로 충돌·도달 불가 목표는 거부된다. 작업 중에는 동작하지 않는다 |
| 정지 | 빨간 버튼, Space, Esc — 작업 취소 + 이동 중단. **소프트웨어 정지이며 인증된 비상정지 장치를 대신하지 않는다** |

음성 예: 「빨간 거 집어줘」, 「노란 큐브 잡아줘」, 「파란 블록 옮겨줘」. 물체는 모두 정육면체라 **색이 필수**다.
「내려놔 / 원래대로」(집은 자리에 놓기)는 Task Manager 의 pick / place 분리 뒤 지원 예정이다.

## 음성 노드 준비 (한 번만)

```bash
sudo apt install alsa-utils                          # arecord (대부분 이미 설치됨)
python3 -m venv --without-pip ~/venvs/mark1_voice    # python3-venv 가 없어도 됨
python3 -m pip --python ~/venvs/mark1_voice/bin/python install faster-whisper "numpy<2"
```
`numpy<2`: 가상환경 Python 이 ROS(시스템 numpy 1.26) 메시지를 함께 쓰기 때문.
첫 실행 때 Whisper base 모델(약 145 MB)을 내려받는다.
마이크가 꺼져 있으면(`amixer -c 1 get Capture` 에 `[off]`) `amixer -c 1 set Capture cap` 으로 켜고,
소리가 너무 크게 들어오면(포화) `amixer -c 1 set Capture 25%` 처럼 낮춘다.

## 구조

| 파일 | 역할 |
|---|---|
| `ros_io.py` | ROS 노드 (별도 스레드). 토픽 구독, ExecuteTask·MoveIt·그리퍼 호출. 화면은 100 ms 마다 `snapshot()` 을 읽는다 |
| `backends.py` | 모드별 연결: `SimBackend`(aris3_sim_full.launch.py 실행, MoveIt 수동 조작) / `RealBackend`(e0509_manual) |
| `main_window.py` | 화면 배치와 버튼 동작 |
| `widgets.py` | 카메라 영상 + 검출 표시(TF + 핀홀 투영), 작업 공간 2D 지도 |

## 알려진 한계 (2026-11-08)
- 시뮬레이션의 물체 인식은 Gazebo 정답 위치(`gz_truth_detector`)다. aris3_vision(OpenCV)이 생기면 바뀐다.
- 실물 모드에서는 자동 작업·음성 실행을 막아 두었다 (실물 Task Manager 검증 전).
- 「RViz 3D」 영역은 위에서 본 2D 지도로 대신했다. RViz 패널 내장은 도전 과제.
- 실제 마이크로 말한 인식률은 아직 측정하지 않았다.
