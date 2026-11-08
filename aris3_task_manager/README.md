# aris3_task_manager — Pick & Place 작업 관리자

요청 하나("빨간 블록을 A로")를 받아 **인식 → 계획 → 집기 → 놓기 → 확인**을 순서대로 진행하는 C++17 ROS 2 액션 서버입니다.
원리와 공부할 것은 [`docs/ARIS3_TaskManager_학습가이드.md`](../docs/ARIS3_TaskManager_학습가이드.md)에 있습니다.

## 실행 (Gazebo)

```bash
# 한 번만: 팀 저장소 패키지를 워크스페이스에 연결하고 빌드
ln -s <저장소>/aris3_interfaces <저장소>/aris3_task_manager <저장소>/aris3_sim_bridge ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select aris3_interfaces aris3_sim_bridge aris3_task_manager
source install/setup.bash

export GZ_IP=127.0.0.1; ros2 launch e0509_sim e0509_workcell.launch.py   # 터미널 1: Gazebo 작업 셀 (GZ_IP: Wi-Fi 끊김 대비)
ros2 launch aris3_task_manager task_manager_sim.launch.py      # 터미널 2: Task Manager + 어댑터
ros2 run aris3_task_manager send_task red A --dry-run          # 터미널 3: 인식·계획만 (로봇 안 움직임)
ros2 run aris3_task_manager send_task red A                    # 집어서 옮김. Ctrl-C = 작업 취소
```
- `send_task <색> <구역> [--dry-run] [--shape cube] [--action 이름]`. 종료 코드 0 = 성공, 1 = 실패·취소, 2 = 서버 없음·거절.
- `ros2 action send_goal --feedback /aris3/execute_task aris3_interfaces/action/ExecuteTask "{target_color: red, destination: A}"` 로도 보낼 수 있습니다.
- 블록 되돌리기: `bash scripts/reset_sim_blocks.sh` / 시나리오 시험: `bash scripts/test_task_manager_sim.sh`
- **Task Manager는 하나만 띄우세요.** 같은 이름의 서버가 살아 있으면 두 번째는 시작을 거부합니다.

## 인터페이스

| 종류 | 이름 | 타입 | 방향 |
|---|---|---|---|
| 액션 | `/aris3/execute_task` | `aris3_interfaces/action/ExecuteTask` | 요청자 → Task Manager |
| 토픽 | `/aris3/detected_objects` | `aris3_interfaces/msg/DetectedObjectArray` | 비전 → Task Manager |
| 토픽 | `/aris3/task_status` | `aris3_interfaces/msg/TaskStatus` | Task Manager → GUI·모니터링 |
| 서비스 | `/aris3/gripper/command` | `aris3_interfaces/srv/GripperCommand` (OPEN / CLOSE / STATUS) | Task Manager → 그리퍼 어댑터 |
| 서비스 | `/dsr01/motion/{move_joint, move_line, move_stop, check_motion, ikin, fkin}` 등 | `dsr_msgs2` | Task Manager → 두산 컨트롤러 |
| TF | `workcell → world → base_link` | | 카메라·구역 좌표 → 로봇 좌표 |

시뮬레이션에서는 `aris3_sim_bridge`가 `e0509_sim`의 카메라 결과와 그리퍼를 위 계약으로 바꿔 줍니다(`bridges:=false`로 끌 수 있음).

## 단계

| # | 단계 | 하는 일 | 대표 실패 |
|---|---|---|---|
| 0 | VALIDATE | 색·구역 확인, 로봇 STANDBY, 그리퍼 응답·**빈손인지**, 카메라 결과 수신, TF | `INVALID_GOAL`, `NOT_READY` |
| 1 | OBSERVE | 관찰 자세(홈)로 이동 후 0.5초 대기 (dry_run이면 생략) | `MOTION_FAILED`, `TIMEOUT` |
| 2 | PERCEIVE | 새 영상 5장 평균, 흔들림 ≤ 5 mm, 모호함 확인, **놓을 구역이 비었는지** | `TARGET_NOT_FOUND / UNSTABLE / AMBIGUOUS`, `DESTINATION_OCCUPIED` |
| 3 | PLAN | 경로점 4개 계산, 작업 공간 경계 + IK→FK 왕복(≤ 1 mm) — **움직이기 전에** 거부 | `UNREACHABLE` |
| 4 | PICK | 열기 → 물체 위 → 하강 → 닫기(무엇을 잡았는지 확인) → 들어 올리기. 실패하면 다시 관찰해 1회 재시도 | `GRASP_FAILED` |
| 5 | PLACE | 구역 위 → 하강 → 열기 → 빠지기 | `MOTION_FAILED` |
| 6 | VERIFY | 홈에서 카메라로 다시 보고 구역 중심 30 mm 안인지 | `PLACE_VERIFY_FAILED` |
| – | RECOVER | 취소: 정지만. PICK/PLACE 중 실패: 정지 → 위로 10 cm → 홈. 잡은 물체는 놓지 않음 | |

이동은 모두 **비동기 명령 + 0.1초마다 확인**(취소·시간 초과 즉시 정지) + **도착 확인**(3 mm)입니다.

## 파일

```
aris3_task_manager/
├── config/task_manager.yaml          # 모든 수치 (구역, 높이, 속도, 허용 오차, 시간 제한)
├── launch/task_manager_sim.launch.py # 어댑터 2개 + workcell->world TF + Task Manager
├── include/aris3_task_manager/
│   ├── types.hpp                     # Vec3, Stage, ErrorCode, TaskError
│   ├── geometry.hpp                  # 경로점·작업 공간·플랜지 변환 (순수 함수)
│   ├── target_estimator.hpp          # 여러 장으로 목표 위치 추정, 구역 점유 (순수 함수)
│   ├── service_call.hpp              # 작업 스레드에서 서비스 보내고 기다리기
│   ├── robot_client.hpp              # 두산 서비스 어댑터 (이동·정지·도착·IK/FK)
│   ├── gripper_client.hpp            # /aris3/gripper/command 클라이언트
│   └── task_manager_node.hpp         # 액션 서버 + 상태 머신
├── src/  (위 헤더의 구현 + main.cpp, send_task.cpp)
└── test/ test_geometry.cpp, test_target_estimator.cpp   # gtest 12개
```

## 시험

```bash
colcon test --packages-select aris3_task_manager && colcon test-result --test-result-base build/aris3_task_manager --verbose
bash scripts/test_task_manager_sim.sh      # 시뮬레이션과 Task Manager가 떠 있을 때
```

## 검증 결과 (2026-11-07, Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · DRCF 에뮬레이터 3.0.1)

| 시험 | 결과 |
|---|---|
| 단위 시험 (gtest) | 12 / 12 통과 |
| dry_run red→A: 인식·계획만, 블록 위치 그대로 | 통과 |
| red→A, green→B, blue→C (카메라 기반) | 성공. 놓은 뒤 카메라로 본 구역 중심 오차 2.2 / 0.3 / 0.6 mm (Gazebo 실제 위치 기준 red 1.1 mm) |
| 작업 중 두 번째 요청 | 거절(종료 코드 2), 첫 작업은 계속 진행되어 성공 |
| 구역 A에 red가 있는데 blue→A | `DESTINATION_OCCUPIED`, 움직이지 않음 |
| 모르는 색 / 모르는 구역 | `INVALID_GOAL` |
| blue→C 5초 뒤 Ctrl-C | 취소됨, 로봇 정지(check_motion 0), 다시 요청하면 성공 |
| 1.2 m 떨어진 구역 (시험용 설정) | 계획 단계에서 `UNREACHABLE`, 움직이지 않음 |
| 6 cm 위를 집도록 설정 (시험용) | 집기 실패 → 다시 관찰·재시도 → `GRASP_FAILED` → 위로 피한 뒤 홈 복귀 |
| Task Manager / 그리퍼 어댑터 중복 실행 | 두 번째는 시작 거부. 강제 종료 직후 재시작은 정상 |

시나리오 시험 `scripts/test_task_manager_sim.sh`: **15 / 15 PASS**

## 알려진 한계
- 경로 계획은 두산 컨트롤러의 직선 이동(move_line)입니다. 장애물 회피·충돌 검사(MoveIt)는 없습니다.
- 블록 회전(yaw)은 반영하지 않습니다(그리퍼는 항상 같은 방향).
- 실제 로봇에는 아직 연결하지 않았습니다. 실제 로봇은 `robot_ns:=/e0509_real`과 실제 그리퍼 어댑터(`/aris3/gripper/command`)가 필요하고, 사람이 비상정지 옆에 있어야 합니다.
