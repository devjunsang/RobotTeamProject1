# ARIS3 Task Manager — 만들면서 배운 것과 공부할 것

- 작성: 2026-11-07, demyank (AI 코딩 도우미 Claude와 함께 개발)
- 대상 코드: [`aris3_interfaces/`](../aris3_interfaces/), [`aris3_task_manager/`](../aris3_task_manager/), [`aris3_sim_bridge/`](../aris3_sim_bridge/), [`scripts/test_task_manager_sim.sh`](../scripts/test_task_manager_sim.sh)
- 목적: **구현보다 이해.** 코드 한 줄 한 줄이 "왜 그렇게 되어 있는지"를 팀원이 설명할 수 있게 한다.

> 읽는 순서 권장: 1장(큰 그림) → 2장(직접 실행) → 3장은 궁금한 절부터 → 4장(코드 읽기) → 5장(과제).
> 각 절 끝의 **해 볼 것**은 실제로 명령을 쳐 보는 것이 목적이다. 읽기만 하면 남지 않는다.

---

## 1. 큰 그림

### 1.1 Task Manager가 하는 일
"빨간 블록을 A로"라는 **요청 하나**를 받아, 여러 모듈을 정해진 순서로 부르고, 각 단계가 정말 성공했는지 확인하고,
실패하면 안전하게 멈추는 **지휘자**다. 직접 영상을 처리하거나 모터를 돌리지 않는다.

```
 요청 (CLI · 나중에 음성/GUI)
    │  /aris3/execute_task  (액션: goal → feedback… → result, 취소 가능)
    ▼
 ┌──────────────────────── aris3_task_manager (C++) ────────────────────────┐
 │ VALIDATE → OBSERVE → PERCEIVE → PLAN → PICK → PLACE → VERIFY → 성공       │
 │     └──── 어느 단계든 실패·취소·시간 초과 ──→ RECOVER → 실패/취소 결과       │
 └────┬───────────────┬───────────────────┬──────────────────┬──────────────┘
      │구독            │서비스              │서비스             │TF 조회
 /aris3/detected_  /aris3/gripper/   /dsr01/motion/*       workcell→base_link
 objects           command           (두산 컨트롤러)
      ▲                 ▲                  ▲
 aris3_sim_bridge   aris3_sim_bridge   DRCF 에뮬레이터 ── Gazebo 로봇
 (vision_bridge)    (gripper_bridge)
      ▲                 ▲
 e0509_sim 카메라   e0509_sim 그리퍼
 (OpenCV)
```

### 1.2 ARIS3 설계와의 대응
| 설계 문서(`ARIS3_상세설계_V0.2`)의 요구 | 구현 |
|---|---|
| `aris3_task_manager`: C++17 · Action · 상태 머신 | `TaskManagerNode` (rclcpp_action 서버) |
| `TaskRequest → Validate → Plan → Pick → Place` | VALIDATE / OBSERVE / PERCEIVE / PLAN / PICK / PLACE / VERIFY |
| 취소·실패·타임아웃을 단계마다 처리, Feedback/Result 제공 | 모든 대기 지점에서 `abortReason()` 확인, `ExecuteTask` feedback/result |
| `aris3_interfaces`에 msg/srv/action 정의, C++·Python 공통 | `DetectedObject(Array)`, `TaskStatus`, `GripperCommand`, `ExecuteTask` |
| 모듈 간 결합 최소화 (내부 헤더 대신 ROS 인터페이스) | Task Manager는 ARIS3 토픽·서비스 이름만 안다. 시뮬레이션 쪽 이름은 `aris3_sim_bridge`가 바꿔 준다 |
| 검출 불확실 → 목표 재선택 / IK 실패 → 실행하지 않고 실패 / 파지 실패 → 재시도 제한 | `TARGET_*` 오류, PLAN 단계 거부, `max_grasp_attempts` |
| 실패 시 정지·복귀 (`ROLE.txt`) | `recover()`: 정지 → 위로 피하기 → 홈 |

**설계와 다른 점 (의도적)**
- 설계는 "MoveIt 계획 요청"이지만, 지금 Gazebo 셀은 두산 가상 컨트롤러(DRCF)가 팔을 움직이고 Gazebo가 따라 그리는 구조라서 **두산 서비스(move_line, ikin/fkin)** 로 계획·검증한다. MoveIt으로 바꿀 자리는 `RobotClient` 하나로 모아 두었다(5장 과제).
- 음성이 아직 없어서 요청은 `send_task` 명령(액션 클라이언트)으로 보낸다. 음성 모듈은 같은 액션에 goal을 보내기만 하면 된다.

---

## 2. 직접 실행해 보기

```bash
# 빌드 (팀 저장소 패키지를 워크스페이스에 심볼릭 링크로 연결 — 설계 문서 7장 권장 방식)
ln -s <저장소>/aris3_interfaces <저장소>/aris3_task_manager <저장소>/aris3_sim_bridge ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select aris3_interfaces aris3_sim_bridge aris3_task_manager
source install/setup.bash

# 터미널 1: Gazebo 작업 셀 (두산 에뮬레이터 + 카메라 + 그리퍼)
export GZ_IP=127.0.0.1        # Gazebo 내부 통신을 루프백에 고정 (Wi-Fi 끊김에 영향받지 않게, 3.14절)
ros2 launch e0509_sim e0509_workcell.launch.py
# 터미널 2: Task Manager + 시뮬레이션 어댑터
ros2 launch aris3_task_manager task_manager_sim.launch.py
# 터미널 3: 요청
ros2 run aris3_task_manager send_task red A --dry-run   # 로봇은 안 움직이고 인식·계획만
ros2 run aris3_task_manager send_task red A             # 실제로 집어서 옮김
```

**해 볼 것 (관찰)**
```bash
ros2 action list -t                         # /aris3/execute_task [aris3_interfaces/action/ExecuteTask]
ros2 action info /aris3/execute_task        # 서버 1개인지 확인 (2개면 위험! 3.1절)
ros2 topic echo /aris3/task_status          # 단계가 바뀔 때마다 + 1초마다
ros2 topic echo /aris3/detected_objects --once
ros2 action send_goal --feedback /aris3/execute_task aris3_interfaces/action/ExecuteTask \
  "{task_type: pick_place, target_color: green, destination: B, dry_run: true}"   # CLI 대신 ros2 도구로
ros2 run rqt_graph rqt_graph                # 노드·토픽 연결 그림
bash scripts/reset_sim_blocks.sh            # 블록을 처음 자리로
bash scripts/test_task_manager_sim.sh       # 시나리오 시험 전체
```

---

## 3. 핵심 개념

### 3.1 ROS 2 액션 — 오래 걸리는 요청
- **무엇**: 서비스(요청-응답)와 토픽(흐름)을 합친 것. **goal**(요청) → **feedback**(진행 상황, 여러 번) → **result**(결과, 한 번), 그리고 **cancel**(취소).
  내부적으로는 서비스 3개(`_action/send_goal`, `_action/cancel_goal`, `_action/get_result`) + 토픽 2개(`_action/feedback`, `_action/status`)다.
- **이 코드에서**: `task_manager_node.cpp`의 `create_server<ExecuteTask>` + 콜백 3개
  - `handleGoal`: 받을지 말지 결정(**빨리 끝나야 함**). 작업 중이면 `REJECT`.
  - `handleCancel`: 취소 요청을 받을지 결정. 실제 정지는 작업 스레드가 다음 확인 시점(최대 0.1초)에 한다.
  - `handleAccepted`: 실제 작업을 **별도 스레드**(`worker_`)로 시작하고 바로 돌아온다.
  - 작업 스레드 끝에서 `succeed()` / `abort()` / `canceled()` 중 하나를 꼭 부른다(안 부르면 클라이언트가 영원히 기다린다).
- **실제로 겪은 일**: Task Manager가 실수로 **두 개** 떠 있었는데, 요청 하나가 **양쪽 서버에 모두 전달되어** 두 프로세스가 같은 로봇을 동시에 움직였다. 결과는 집기 실패와 블록 쌓임이었다.
  → 시작할 때 같은 이름의 서버가 살아 있으면 시작을 거부하도록 했다(`actionServerAlive()`).
- **해 볼 것**: `ros2 action info /aris3/execute_task`. `ros2 service list --include-hidden-services | grep execute_task`로 숨은 서비스 3개를 찾아보기.
- **생각해 볼 것**: 서비스로 만들었다면 취소와 진행 상황을 어떻게 구현해야 했을까?

### 3.2 상태 머신과 예외로 실패 전달
- **무엇**: "지금 어느 단계인가"를 하나의 값(`Stage`)으로 관리하고, 단계마다 할 일과 다음 단계가 정해진 구조.
- **이 코드에서**: `execute()`가 단계 함수를 순서대로 부른다. 각 단계는 실패하면 `TaskError(코드, 설명)`를 **던지고**, `execute()` 한 곳에서 받아 Result로 바꾼다.
  - 장점: 단계마다 `if (실패) { 정리; return; }`가 반복되지 않아 정상 흐름이 한눈에 보인다.
  - 집기 재시도: `for (attempt...) { try { stagePick } catch (GRASP_FAILED) { 다시 관찰·인식·계획 } }`
- **복구 정책** (`recover()`) — 왜 이렇게 정했나
  | 원인 | 행동 | 이유 |
  |---|---|---|
  | 취소 | **정지만** | 사용자가 "멈춰"라고 했는데 더 움직이면 놀란다 |
  | PICK/PLACE 중 실패 | 정지 → 위로 10 cm → 홈 | 팔이 테이블 가까이에 있으면 다음 동작이 위험하다 |
  | 그 외(인식·계획 실패 등) | 정지만 | 아직 움직이지 않았거나 이미 홈에 있다 |
  | 물체를 잡은 채 실패 | 놓지 않음 | 높은 곳에서 놓으면 떨어진다 |
- **생각해 볼 것**: 실제 로봇이라면 "취소 = 정지만"이 늘 옳을까? 그리퍼를 연 채로 물체 바로 위에서 멈췄다면?

### 3.3 인터페이스 정의 (rosidl)
- **무엇**: `.msg / .srv / .action` 파일을 쓰면 빌드할 때 C++ 헤더와 Python 모듈이 **자동 생성**된다. C++ Task Manager와 Python 어댑터가 같은 타입을 쓰는 이유다.
- **이 코드에서**: `aris3_interfaces/CMakeLists.txt`의 `rosidl_generate_interfaces(...)`. 상수(`uint8 ERROR_CANCELED=11`)도 정의할 수 있다.
  `types.hpp`의 `ErrorCode`는 이 상수와 **같은 값**이어야 한다(한쪽만 바꾸면 틀어진다 — 주석으로 표시).
- **주의**: 인터페이스를 바꾸면 그것을 쓰는 **모든 패키지를 다시 빌드**해야 한다. 이번에도 `GripperCommand`에 `STATUS`, `ExecuteTask`에 `ERROR_DESTINATION_OCCUPIED`를 추가하면서 3개 패키지를 다시 빌드했다.
- **해 볼 것**: `ros2 interface show aris3_interfaces/action/ExecuteTask`, `ros2 interface proto aris3_interfaces/srv/GripperCommand`.

### 3.4 Executor, 콜백 그룹, 스레드 — 가장 많이 막히는 곳
- **무엇**: ROS 2 노드의 콜백(구독·서비스 응답·타이머)은 **executor**가 실행한다. `SingleThreadedExecutor`는 한 번에 하나, `MultiThreadedExecutor`는 여러 개를 동시에 실행한다.
  **콜백 그룹**은 "이 콜백들은 동시에 돌아도 되는가"를 정한다(`MutuallyExclusive` = 하나씩, `Reentrant` = 동시에).
- **이 코드에서**
  - `main.cpp`: `MultiThreadedExecutor`(스레드 4개).
  - 서비스 클라이언트는 `Reentrant` 그룹(`client_group_`), 카메라 구독은 `MutuallyExclusive` 그룹(`input_group_`).
  - 작업은 executor 밖의 `std::thread`에서 돈다. 서비스를 보내고 `future.wait_for()`로 기다리는 동안 **응답은 executor의 다른 스레드가 받아서** future를 채워 준다(`service_call.hpp`).
- **왜 멈추는가(deadlock)**: 콜백 안에서 같은 executor 스레드로 응답을 기다리면, 응답을 처리할 스레드가 바로 자신이라 영원히 기다린다.
  Python 어댑터(`gripper_bridge.py`)도 서비스 콜백 안에서 다른 서비스를 부르므로 `MultiThreadedExecutor + ReentrantCallbackGroup + threading.Event`를 쓴다.
- **공부**: ROS 2 문서 "Using Callback Groups", "Executors". 단일 스레드 executor로 바꿔서 실제로 멈추는 것을 보는 것이 가장 빨리 이해된다(5장 과제 3).

### 3.5 공유 데이터: mutex와 atomic
- **카메라 버퍼** `frames_`: 구독 콜백(executor 스레드)이 쓰고 작업 스레드가 읽는다 → `std::mutex`로 보호. 콜백은 버퍼에 넣기만 하고 빨리 끝난다.
- **한 번에 한 작업** `busy_.exchange(true)`: "이전 값을 돌려주고 true로 바꾸기"를 **한 동작**으로 한다. `if (!busy_) busy_ = true;`로 쓰면 두 요청이 거의 동시에 오면 둘 다 통과할 수 있다(경쟁 상태, race condition).
- **새 영상만 쓰기**: 인식을 시작할 때의 영상 번호(`frame_seq_`)를 기억하고 그 **이후**에 들어온 영상만 쓴다. 팔이 움직이던 중에 찍힌 영상을 섞지 않기 위해서다.

### 3.6 좌표계와 TF2 — 이름이 같다고 같은 좌표계가 아니다
- **무엇**: TF2는 좌표계(frame) 사이의 변환을 나무(tree) 구조로 관리한다. `lookupTransform(target, source)`는 "source에서 표현된 점을 target으로 바꾸는 변환"이다.
- **이 코드의 좌표계**
  ```
  workcell (Gazebo 바닥 기준: 카메라 결과·구역 좌표)
     └─(0, 0, 0.75)─ world (시뮬레이션 URDF의 뿌리 = 로봇 베이스와 같은 점)
                        └─(0)─ base_link (로봇 이동 명령의 기준) ─ link_1 ...
  ```
- **실제로 겪은 일**: 처음엔 카메라 좌표를 `world`라고 부르고 TF로 `world → base_link`를 조회했다. 그런데 시뮬레이션의 TF는 이 변환을 **0**으로 내보내고 있었고, Gazebo에서 로봇은 테이블 위 **0.75 m**에 있었다. 그래서 경로점 높이가 0.875 m로 계산되어 계획 단계에서 거부됐다(움직이기 전이라 안전했다).
  → 카메라 좌표를 `workcell`이라고 정직하게 이름 붙이고, `workcell → world`(z 0.75)를 Task Manager launch가 내보내도록 했다.
- **교훈**: 좌표 값만 보지 말고 **frame_id와 그 frame이 실제로 어디인지**를 확인한다. `ros2 run tf2_ros tf2_echo workcell base_link`로 직접 확인하는 습관.
- **해 볼 것**: `ros2 run tf2_tools view_frames` → `frames.pdf` 열어서 나무 구조 보기.

### 3.7 경로점 계산 — TCP, 플랜지, 단위
- **TCP**(Tool Center Point): 실제로 물체를 잡는 점. **플랜지**: 로봇 손목 끝(그리퍼를 붙이는 면). 두산 `move_line`은 **플랜지 자세**를 받는다.
- 그리퍼가 수직으로 내려다보므로(`tool_orientation_zyz_deg: [0, 180, 0]`) 플랜지 = TCP + (0, 0, `tcp_offset_m`).
- 단위: ROS 표준은 m, 두산 명령은 mm·deg. 변환은 `flangePoseMmDeg()` **한 곳**에서만 한다.
- 경로점 4개 (`makePickPlacePlan`): 물체 위 → 집기 → (들어 올리기 = 물체 위) → 구역 위 → 놓기 → (빠지기 = 구역 위).
- **ZYZ 오일러각**: 두산의 자세 표현. `[45, 180, 45]`와 `[0, 180, 0]`은 같은 자세다(B=180°일 때 A와 C가 합쳐짐). 그래서 도착 확인은 위치만 비교한다.

### 3.8 검증을 여러 겹으로 — "명령 접수"와 "성공"은 다르다
움직이기 **전**과 **후**에 확인한다.
| 시점 | 확인 | 코드 | 왜 |
|---|---|---|---|
| 전 | 작업 공간 경계(수평 거리, 높이) | `checkWorkspace` | 테이블 아래(z<0)도 IK는 풀린다 |
| 전 | **IK → FK 왕복 오차** ≤ 1 mm | `reachabilityErrorMm` | 두산 `ikin`은 닿지 않는 점에도 `success=true`를 준다(실측: 1.5 m 요청 → FK로 되돌리면 0.74 m) |
| 중 | 0.1초마다 `check_motion`, 취소·시간 초과 | `waitForMotion` | 동기 호출은 중간에 멈출 수 없다 |
| 후 | 현재 위치와 목표의 거리 ≤ 3 mm | `moveLinear` | 닿지 않는 위치로 비동기 이동을 보내도 `success=true`, 그리고 아무 일도 안 일어났다(실측) |
| 후 | 그리퍼가 **무엇을** 잡았는지 | `stagePick` | 다른 물체를 잡았을 수도 있다 |
| 후 | 놓은 뒤 카메라로 구역 안인지 | `stageVerify` | 마지막 판정은 실제 결과(닫힌 고리, closed loop) |

**교훈**: 외부 API의 `success`는 "요청을 받았다"는 뜻일 수 있다. 중요한 것은 **결과를 직접 측정**해서 확인한다.

### 3.9 인식의 불확실성 다루기
- **N장 평균**(`perception_frames: 5`)과 **흔들림**(spread ≤ 5 mm): 한 장의 잡음이나 순간 오검출을 믿지 않는다.
- **모호함**: 조건에 맞는 물체가 서로 2 cm 이상 떨어져 여러 개 보이면 `TARGET_AMBIGUOUS`. 2 cm 안이면 한 물체가 쪼개진 것으로 보고 합친다.
- **신뢰도**: `vision_bridge`가 넓이·조각 수로 간단히 계산한다(자리표시자). 실제 비전 모듈이 더 나은 값을 주면 된다.
- **구역 점유**: 놓을 구역 반경 6 cm 안에 다른 물체가 있으면 `DESTINATION_OCCUPIED`. 중복 실행 사고 때 블록이 다른 블록 위에 쌓인 것을 보고 추가했다.
- 모두 **팀 안전 원칙 "인식이 불확실하면 동작을 실행하지 않는다"** 를 코드로 옮긴 것이다.

### 3.10 오류 코드 설계
- 사람이 읽는 `message`와 별도로 **기계가 읽는 `error_code`**를 둔다. GUI나 음성 응답이 코드별로 다르게 반응할 수 있다("빨간 블록이 안 보여요" vs "A 구역이 차 있어요").
- `failed_stage`로 어디서 실패했는지 알 수 있다. 시험 스크립트는 종료 코드와 `error_code`를 함께 확인한다.

### 3.11 파라미터와 launch
- `config/task_manager.yaml`: 맨 위 키 아래 `ros__parameters`. 코드의 `declare_parameter(이름, 기본값)`이 기본값이고 YAML이 덮어쓴다.
  launch나 명령줄의 `-p 이름:=값`이 다시 덮어쓴다(시험 스크립트가 이 방법으로 "먼 구역"을 만든다).
- **맨 위 키는 노드 이름과 짝지어진다.** 처음엔 `aris3_task_manager:`로 썼는데, 시험용 서버를 `-r __node:=aris3_task_manager_test`로
  띄우자 이 파일의 값이 **하나도 적용되지 않고** 코드 기본값이 조용히 쓰였다. 기본 좌표계가 달라 경로가 틀어졌다(경고도 없음).
  → 키를 `/**:`(모든 노드)로 바꾸고, 코드 기본값도 YAML과 같게 맞췄다. 확인 방법: `ros2 param get /노드이름 world_frame`.
- YAML은 `install/.../share`로 **복사**되므로 고친 뒤 `colcon build`가 필요하다(`--symlink-install`로 빌드하면 필요 없음).
- launch: `DeclareLaunchArgument`, `LaunchConfiguration`, `IfCondition(bridges)`, `static_transform_publisher`.

### 3.12 C++ 빌드와 경고
- 순수 로직(`geometry.cpp`, `target_estimator.cpp`)을 **ROS 없는 정적 라이브러리**(`task_logic`)로 분리했다 → 단위 시험이 ROS 없이 빠르게 돈다.
- `ament_target_dependencies`, `install(TARGETS ...)`, `install(DIRECTORY config launch ...)`.
- **경고를 무시하지 않기**: `-Wdangling-reference` 경고가 진짜 버그였다.
  `for (auto & c : get_parameter("x").as_string_array())`는 임시 객체가 반복 도중 사라진 상태에서 그 안을 읽는다 → 먼저 지역 변수로 복사.

### 3.13 시험의 층
| 층 | 무엇 | 어디 | 걸리는 시간 |
|---|---|---|---|
| 단위 시험 (gtest) | 경로점 높이, 작업 공간, 평균·흔들림·모호함·점유 판정 | `test/*.cpp` (12개) | 1초 |
| 시나리오 시험 | 성공 3건, 동시 요청 거절, 점유·잘못된 요청 거부, 취소와 재개, 도달 불가, 집기 실패와 복구 | `scripts/test_task_manager_sim.sh` | 수 분 |
| 관찰 | Gazebo 화면, `rqt_image_view /e0509/vision/debug_image`, `/aris3/task_status` | 사람 | — |

- 위험한 경우(도달 불가, 집기 실패)는 **설정만 바꾼 시험용 서버**(`/aris3/test_execute_task`)로 일부러 만든다. 코드를 고치지 않고 시험한다.
- `colcon test --packages-select aris3_task_manager && colcon test-result --verbose`

### 3.14 운영하면서 겪은 함정 (리눅스·ROS)
| 함정 | 증상 | 대처 |
|---|---|---|
| 같은 이름의 액션 서버 2개 | 요청 하나에 두 프로세스가 같은 로봇을 움직임 | 시작할 때 살아 있는 서버가 있으면 거부 |
| 강제 종료된 노드의 이름이 그래프에 남음 | "이미 실행 중" 오판으로 정상 재시작이 막힘 | 이름 확인 대신 **실제로 요청을 보내 응답이 오는지** 확인 |
| 스크립트에서 `&`로 띄운 프로그램은 SIGINT 무시 | Ctrl-C 신호로 안 꺼져서 옛 프로세스가 남음 | SIGTERM 사용, 끈 뒤 `pgrep`으로 확인 |
| `pkill -f '패턴'`이 자기 셸까지 죽임 | 명령이 갑자기 종료 | 패턴을 `^/usr/bin/...`처럼 시작에 고정하거나 스크립트 파일 안에 둔다 |
| `source ... && (A) & B` | `B`가 ROS 환경 없이 실행됨 | 환경을 먼저 불러온 뒤 따로 실행 |
| YAML 맨 위 키 = 노드 이름 | 이름을 바꿔 띄운 노드에 설정이 안 들어가고 기본값이 쓰임 | `/**:` 사용, `ros2 param get`으로 확인 |
| Wi-Fi가 끊겼다 다시 붙음 | Gazebo 로그에 `Network is unreachable`, Gazebo 토픽·서비스가 사라지고 카메라 결과가 멈춤 | 시뮬레이션을 `GZ_IP=127.0.0.1`로 띄워 Gazebo 통신을 루프백에 고정 |
| 정리 스크립트의 넓은 `pkill` | `e0509_sim/tools/stop_sim.sh`의 `pkill -x ros2_control_node`는 **실제 로봇 접속의 드라이버까지** 끈다 | 실제 로봇이 접속된 PC에서는 쓰지 말 것. 이 문서의 시험도 시뮬레이션 전용 |

---

## 4. 코드 읽기 순서

1. [`aris3_interfaces/action/ExecuteTask.action`](../aris3_interfaces/action/ExecuteTask.action) — 무엇을 요청하고 무엇을 돌려받는가.
2. [`include/.../task_manager_node.hpp`](../aris3_task_manager/include/aris3_task_manager/task_manager_node.hpp) 머리말 그림 → [`src/task_manager_node.cpp`](../aris3_task_manager/src/task_manager_node.cpp)의 `execute()` — 전체 흐름.
3. 같은 파일의 `stage*()` 함수들을 위에서부터 — 단계별 확인 항목.
4. [`geometry.cpp`](../aris3_task_manager/src/geometry.cpp), [`target_estimator.cpp`](../aris3_task_manager/src/target_estimator.cpp) + `test/` — 순수 로직과 그 시험.
5. [`robot_client.cpp`](../aris3_task_manager/src/robot_client.cpp) — 두산 서비스, 비동기 이동, 도착 확인, IK/FK 왕복.
6. [`service_call.hpp`](../aris3_task_manager/include/aris3_task_manager/service_call.hpp) — 작업 스레드에서 서비스 기다리기.
7. [`aris3_sim_bridge/`](../aris3_sim_bridge/) — 계약에 맞춰 주는 어댑터(Python).
8. [`send_task.cpp`](../aris3_task_manager/src/send_task.cpp) — 액션 **클라이언트** 쪽(feedback 출력, Ctrl-C → 취소).
9. [`config/task_manager.yaml`](../aris3_task_manager/config/task_manager.yaml), [`launch/task_manager_sim.launch.py`](../aris3_task_manager/launch/task_manager_sim.launch.py).

---

## 5. 과제 (이해를 확인하고 프로젝트를 이어가는 순서)

1. **관찰**: `send_task red A`를 실행하면서 `ros2 topic echo /aris3/task_status`와 Gazebo 화면을 나란히 본다. 단계 이름과 로봇 동작을 짝지어 적어 본다.
2. **파라미터**: `approach_height_m`를 0.05로 바꿔 실행해 보고, 경로점 출력과 동작이 어떻게 바뀌는지 설명한다.
3. **deadlock 체험**: `main.cpp`의 `MultiThreadedExecutor`를 `SingleThreadedExecutor`로 바꿔 빌드하고 `send_task`를 보내 본다. 무엇이 멈추는지, 왜 멈추는지 3.4절로 설명한다(확인 후 되돌리기).
4. **단위 시험 추가**: "신뢰도가 낮은 검출만 있으면 NOT_FOUND"를 `test_target_estimator.cpp`에 추가한다.
5. **음성 연결 (PHASE 5)**: `/aris3/voice_command`(설계 문서 4장)를 구독해서 "빨간 블록을 A로" → `ExecuteTask` goal을 보내는 작은 노드를 만든다. Task Manager 코드는 바꾸지 않는다.
6. **실제 그리퍼 어댑터**: `etc/e0509_manual/e0509_manual/gripper_drl.py`(실제 RH-P12-RN(A), Modbus over DRL)를 이용해 `/aris3/gripper/command` 서버를 만든다. `STATUS`는 Present Position·Moving으로 구현한다.
7. **MoveIt 백엔드**: `RobotClient`와 같은 함수(`moveLinear`, `reachabilityErrorMm` …)를 MoveIt으로 구현한 클래스를 만들어 바꿔 끼운다. 두산 MoveIt은 컨트롤러 2.12 이상이 필요하다는 점(실제 로봇은 2.10.3)을 확인한다.
8. **블록 회전(yaw)**: 비전이 yaw를 주면 `tool_orientation`의 마지막 각에 반영하도록 `makePickPlacePlan`을 확장하고 단위 시험을 쓴다.

---

## 6. 스스로 점검

<details><summary>1. handleGoal 안에서 로봇을 움직이면 안 되는 이유는?</summary>
executor 스레드를 오래 붙잡아 다른 콜백(취소 요청, 서비스 응답, 카메라)이 처리되지 않는다. 받을지만 정하고 실제 작업은 별도 스레드에서 한다.</details>

<details><summary>2. 두산 move_line을 동기(sync_type=0)로 쓰면 무엇이 안 되나?</summary>
이동이 끝날 때까지 호출이 돌아오지 않아, 그 사이 취소·시간 초과를 처리할 수 없다.</details>

<details><summary>3. IK 서비스가 success=true를 줬는데 왜 또 FK로 확인하나?</summary>
이 컨트롤러의 ikin은 닿지 않는 점에도 success=true와 엉뚱한 관절각을 준다. FK로 되돌린 위치가 목표와 다르면 닿지 않는 점이다.</details>

<details><summary>4. lookupTransform("base_link", "workcell")의 결과로 무엇을 할 수 있나?</summary>
workcell에서 표현된 점(카메라 결과, 구역)을 base_link 좌표(로봇 명령)로 바꿀 수 있다.</details>

<details><summary>5. busy_를 bool이 아니라 atomic의 exchange로 쓰는 이유는?</summary>
"확인하고 바꾸기"를 한 동작으로 해야 거의 동시에 온 두 요청이 둘 다 통과하는 경쟁 상태를 막을 수 있다.</details>

<details><summary>6. 취소했을 때 홈으로 돌아가지 않는 이유와, 그 대신 생기는 문제는?</summary>
사용자의 "멈춰"를 존중하기 위해서다. 대신 물체를 잡은 채 멈출 수 있어서, 다음 작업의 VALIDATE가 그리퍼 STATUS로 "잡고 있음"을 확인하고 시작을 거부한다.</details>

---

## 7. 참고 자료
- ROS 2 Jazzy — Actions(개념, C++ 서버/클라이언트 튜토리얼): https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
- ROS 2 — Executors, Callback Groups: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html , https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html
- ROS 2 — 인터페이스(msg/srv/action) 만들기: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
- TF2 튜토리얼: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html
- 파라미터: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters.html
- GoogleTest 입문: https://google.github.io/googletest/primer.html
- 두산 ROS 2 드라이버: https://github.com/DoosanRobotics/doosan-robot2
- 앞선 문서: [`docs/E0509_실기연동_학습가이드.md`](E0509_실기연동_학습가이드.md) (실제 로봇·그리퍼 연동, PR #1)
