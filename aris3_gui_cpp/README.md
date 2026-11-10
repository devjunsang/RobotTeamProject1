# ARIS (aris3_gui_cpp)

ARIS3 Control Station 의 C++ Qt5 버전. Python 버전 `aris3_gui` 와 나란히 유지하고, 같은 공용 인터페이스만 쓴다.

## 빌드·실행

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select aris3_gui_cpp
ros2 run aris3_gui_cpp aris                    # 시뮬레이션 모드: Connect = aris3_sim_full.launch.py
ros2 run aris3_gui_cpp aris --no-voice         # 음성 노드 없이
# 시험: Connect 와 작업을 자동으로, 화면 저장 후 종료
ros2 run aris3_gui_cpp aris --connect --task blue:A --screenshot /tmp/aris.png --quit-after 240
```

## 구조

| 파일 | 역할 |
|---|---|
| `src/main.cpp` | 인자, Qt 앱, ROS 노드를 별도 스레드 executor 에서 실행 |
| `src/ros_io.cpp` | `GuiNode`: 구독·작업 액션·그리퍼·음성. 받은 값은 잠금 아래 보관, 화면은 100 ms 마다 `snapshot()` |
| `src/sim_backend.cpp` | 시뮬레이션 시작·종료 (`setsid` 프로세스 그룹으로 launch 를 띄우고 한 번에 정리) |
| `src/main_window.cpp` | 화면 배치(목업과 같음), 버튼 동작, 주기 갱신 |
| `src/widgets.cpp` | `Panel`(붉은 제목줄·금속판·그림자), `CameraView`, `LogoBadge` |
| `src/theme.cpp` | 아이언맨 슈트 색 스타일시트, 상태 점 (정상 시안 / 경고 앰버 / 오류 밝은 빨강 + ⚠) |
| `src/gui_logic.cpp` | Qt·ROS 없는 순수 로직 (수신 주기, 로그 거르기, 작업 단계) - `test/test_gui_logic.cpp` |

## 단계

- **M1 (지금)**: 시스템 상태, 카메라 영상(원본·JPEG), 로그, 작업 큐·단계, 음성 「<색> 큐브 집어줘」, Connect·정지.
- **M2**: 수동 조작(MoveIt), 카메라 검출 TF 투영·클릭 선택(이거/저기), 작업 공간 2D.
- **M3**: 실물 모드 (`aris3_real_bridge`), 실물 집기·보정, 손짓 - `aris3_real_sim.launch.py` 로 시험.

M2·M3 전에는 그 기능을 `aris3_gui`(PyQt)에서 쓴다.

## 안전

정지는 언제나 동작한다 (버튼, Space, Esc). 화면의 정지는 인증된 비상정지 장치를 대신하지 않는다.
