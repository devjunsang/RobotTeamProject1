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

- **M1**: 시스템 상태, 카메라 영상(원본·JPEG), 로그, 작업 큐·단계, 음성(음성 명령 모드 + 「아리스」), Connect·정지.
- **M2**: 수동 조작(조작 허용 2분 잠금, TCP·관절 조그, 그리퍼, 홈 - MoveIt), 손끝 TCP 표시, 카메라 검출 TF 투영,
  클릭 선택(이거/저기)과 「선택한 물체를 그 자리로」, 음성 「이거/저기/홈」, 작업 공간 2D.
  시험: `ros2 run aris3_gui_cpp aris --selftest` (시뮬레이션이 떠 있을 때, 조작 허용·홈·조그·화면 선택 작업 자동 확인)
- **M3**: 실물 모드 `--mode real` (`aris3_real_bridge` 를 통해서만 로봇을 움직임). 실물 집기 패널(화면 클릭으로
  물체·놓을 자리 고르기, 집기/놓기/집어서 옮기기 - 모두 확인 창), 보정 불러오기·상태, 손짓 이벤트, 손 안내, 복구 홈.
  `--sim` 이면 가상 컨트롤러(DRCF 에뮬레이터) + `aris3_real_sim.launch.py` 로 실물과 비슷하게 시험.
  시험: `ros2 run aris3_gui_cpp aris --mode real --sim --selftest` (접속·조작 허용·홈·보정·선택·집어서 옮기기·정지 7가지)

| 파일 | 하는 일 |
|---|---|
| `include/aris3_gui_cpp/backend.hpp` | 모드 공통 `Backend` 와 프로세스 그룹 관리 `LaunchProc` |
| `src/real_backend.cpp` | 실물 모드: `/aris3/real/*` 상태·서비스·액션, 브리지·음성·손짓 프로세스 |
| `src/main_window_real.cpp` | 실물 집기 패널, 실물 화면 갱신, 실물 자체 시험 |

## 안전

정지는 언제나 동작한다 (버튼, Space, Esc). 화면의 정지는 인증된 비상정지 장치를 대신하지 않는다.
