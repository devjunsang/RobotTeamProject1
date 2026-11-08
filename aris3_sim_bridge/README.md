# aris3_sim_bridge — Gazebo 작업 셀(e0509_sim) ↔ ARIS3 계약 어댑터

Task Manager는 ARIS3 공통 인터페이스(`aris3_interfaces`)만 사용한다. 이 패키지는 이미 동작하는
Gazebo 작업 셀(`e0509_sim`)을 그 계약에 맞춰 주는 **임시 다리**다.

| 노드 | 입력 | 출력 |
|---|---|---|
| `vision_bridge` | `/e0509/vision/detections` (JSON 문자열) | `/aris3/detected_objects` (`DetectedObjectArray`, frame `workcell`) |
| `gripper_bridge` | `/aris3/gripper/command` (`GripperCommand` 서비스) | `/dsr01/gripper/open`, `/dsr01/gripper/close` (`Trigger`) 호출 |

- 팀의 `aris3_vision`이 `/aris3/detected_objects`를, `aris3_control`이 `/aris3/gripper/command`를 제공하게 되면
  이 패키지는 실행하지 않으면 된다. **Task Manager 코드는 바꿀 필요가 없다.** (모듈 간 결합 최소화)
- 실행은 `ros2 launch aris3_task_manager task_manager_sim.launch.py`가 함께 띄운다.
