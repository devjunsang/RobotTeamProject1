# ARIS3 개발 방향 확정 (2026-11-08)

기준 문서: `etc/e0509_manual/ARIS3_전체개발_마스터플랜.html`, `ARIS3_상세설계_V0.2_전체아키텍처.html`
이 문서는 두 문서의 목표를 유지하면서 **현재 저장소 상태를 반영해 확정한 결정**과 다음 작업 순서를 적는다.

## 1. 확정한 결정

| 항목 | 결정 | 이유 |
|---|---|---|
| 경로 계획·실행 | **MoveIt 2** (`aris3_moveit_config`) → Gazebo `arm_controller` (JointTrajectoryController) | 마스터플랜·상세설계 그대로. 충돌 검사·경로 계획 확보 |
| Task Manager 이동 | 기존 두산 서비스(move_line 등) 백엔드는 e0509_sim 용으로 남기고, **MoveIt 백엔드를 추가** (`RobotClient` 교체) | 이미 검증된 상태 머신·실패 처리 재사용 |
| 작업 물체 | **빨강·파랑·노랑 정육면체 3개** (한 변 5 cm), 로봇 중심 반경 0.45 m의 12시(+y) 빨강, 3시(+x) 파랑, 9시(−x) 노랑 | 팀 결정 (2026-11-08). **도형은 정육면체 하나뿐 → 물체는 색으로만 구분**. 원·삼각은 쓰지 않음 (도형 생성기는 지원은 하므로 필요해지면 `object_layout.csv` 만 바꾸면 됨) |
| 작업 공간 | 탁자 윗면 z = 0.75 m, 로봇 베이스가 탁자 위 (e0509_sim 과 같은 높이) | 실물 작업 셀과 비슷한 배치 |
| 음성 PLACE | "내려놔 / 돌려놔 / 원래대로" = **집었던 원래 자리**에 놓기 | MARK1 파서에 위치 단어가 없음 |
| 음성 PICK | **색 필수, 도형 선택** ("파란 거 집어줘", "노란 큐브 집어줘"). 조건에 맞는 물체가 여러 개면 실행하지 않고 되묻기 | 모양이 모두 같아 도형으로는 고를 수 없음. 현재 Task Manager 규칙(색 필수)과 같음 |
| 파지 | **실제 마찰 파지가 기본**, 시연 안정용 '손에 붙이기(DetachableJoint)'는 launch 옵션. 두 결과는 따로 기록 | 마스터플랜 13단계(물리 파지 검증) 유지 |
| 음성 경로 | ~~Windows MARK1 → TCP~~ → **Windows PC 쓰지 않음.** Ubuntu `aris3_voice` 노드(마이크·Whisper·해석) + GUI 「누르고 말하기」 버튼 | MARK1 의 Whisper 설정·파서를 옮겨 씀 (2026-11-08 결정) |
| 관제 GUI | **하나의 PyQt5 화면 `aris3_gui`** (시뮬레이션·실물). 마스터플랜 07장 목업 배치. 기존 `e0509_manual/gui.py`·안전 코드 재사용 | C++ 대신 Python: 기존 코드 재사용·음성과 같은 언어 (2026-11-08 결정) |

## 2. 현재 상태 (마스터플랜 GATE 기준, 2026-11-08 직접 확인)

| GATE | 상태 | 근거 |
|---|---|---|
| 1 관절 제어 | ✅ | 물리 실행 중 joint_1 0.5 rad 명령 → 도달 후 자세 유지 |
| 2 그리퍼 개폐 | ✅ (개폐만) | 0.8 닫기 / 0.0 열기, 손가락 4개 mimic 추종. 물체 파지는 미검증 |
| 3 인식 | 🔶 | 큐브 3개 + Top View 카메라 15 Hz. **검출 노드(aris3_vision) 없음** |
| 4 Pick & Place | ✅ (보조 파지) / 🔶 (마찰 파지) | Task Manager + MoveIt: 큐브 3개 → 구역 A/B/C, 오차 0.4~3.7 mm. 순수 마찰 파지는 옮기다 미끄러져 놓기 오차 36.8 mm |
| 5 음성 | 🔶 | `aris3_voice` 노드 + GUI 연동, 문장 입력 → 작업 실행까지 성공. **실제 마이크 인식률 측정 전** |
| 도전 GUI | 🔶 | `aris3_gui` 1차: 시뮬레이션 모드 전체 흐름 동작 (「노란 큐브 집어줘」 → 구역 A 0.2 mm) |

## 3. 다음 작업 순서

1. **Task Manager MoveIt 백엔드** — `RobotClient` 인터페이스를 추상화하고 `MoveItRobotClient` 추가
   (move_group_interface, 접근/하강은 Pilz LIN 직선 이동). 고정 좌표로 집기·놓기 시퀀스 확인.
2. **그리퍼 어댑터** — `/aris3/gripper/command` 를 Gazebo `gripper_position_controller` 에 연결
   (aris3_control 또는 aris3_sim_bridge). 손가락 위치·정지로 `holding` 판정.
3. **마찰 파지 검증** — 물체 질량·마찰·그리퍼 힘 조정. 실패 시 DetachableJoint 보조 옵션.
4. **aris3_vision (C++ OpenCV)** — Top View 영상 → 색·도형 → 탁자 평면 투영 → `/aris3/detected_objects`.
   카메라가 높아(탁자 위 2 m) 물체가 작게 보임 → 카메라 높이·화각 재조정 검토.
5. **aris3_voice 브리지** — TCP :5000 서버 (MARK1 JSON 수신, `{"status":"received"}` 응답) → `ExecuteTask`.
6. 통합 시연 → Qt GUI (도전).

## 4. 맞춰야 할 계약 (인터페이스)

| | MARK1 음성 | 현재 ExecuteTask (Task Manager) | 맞추는 방향 |
|---|---|---|---|
| 동작 | `pick` / `place` 따로 | `pick_place` 한 번에 | `task_type` 에 `pick`, `place` 추가 (메시지 필드 변경 없음) |
| 대상 | `target`(도형) 필수, `color` 선택 | `target_color` 필수 | **MARK1 파서를 색 필수·도형 선택으로 변경** (Task Manager 는 그대로) |
| 놓는 곳 | 없음 (원래 자리) | `destination` 구역 필수 | `place` 의 `destination` 이 비면 집었던 자리 |

**MARK1 파서 수정 필요 (2026-11-08 시험: 현재 물체 표현 0/3):**
- 색에 '노란/노랑/노란색 → yellow' 추가
- '큐브/정육면체/네모/사각형' → `cube` (도형은 선택 사항이므로 없어도 됨)
- PICK 유효 조건: 도형 필수 → **색 필수**

메시지 정의를 바꾸게 되면 `aris3_interfaces` 를 쓰는 모든 패키지를 다시 빌드한다.

## 5. 실행 방법 (현재)

```bash
# 터미널 1: Gazebo (탁자 + 로봇 + 큐브 3개 + 카메라, 팔 궤적 컨트롤러)
ros2 launch aris3_bringup simulation.launch.py
# 터미널 2: MoveIt 2 (rviz:=true 로 MotionPlanning 패널)
ros2 launch aris3_moveit_config move_group.launch.py
# 터미널 3: 연동 시험 (home → 파란 큐브 위 → home)
ros2 run aris3_moveit_config moveit_smoke_test.py --ros-args -p use_sim_time:=true
```

변경 전 원본 파일: `backup/20261108_before_table/` (README 참고)
