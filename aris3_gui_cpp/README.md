# ARIS (aris3_gui_cpp)

ARIS3 Control Station 의 C++ Qt5 버전. Python 버전 `aris3_gui` 와 나란히 유지하고, 같은 공용 인터페이스만 쓴다.

## 빌드·실행

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select aris3_gui_cpp
ros2 run aris3_gui_cpp aris                    # 기본: 실물 로봇이 응답하면 실물, 아니면 실물 비슷한 시뮬레이션
ros2 run aris3_gui_cpp aris --no-auto-home     # 창이 열려도 자동 접속·홈 하지 않음
ros2 run aris3_gui_cpp aris --no-voice         # 음성 노드 없이
ros2 run aris3_gui_cpp aris --mode sim         # 예전 시뮬레이션 모드: Connect = aris3_sim_full.launch.py (MoveIt)
# 시험 (예전 시뮬레이션): Connect 와 작업을 자동으로, 화면 저장 후 종료
ros2 run aris3_gui_cpp aris --mode sim --connect --task blue:A --screenshot /tmp/aris.png --quit-after 240
```

시작 규칙 (2026-11-11):

1. 실물 로봇(`--host`, 기본 110.120.1.66 의 12345 포트)이 1.5 초 안에 응답하지 않으면 **시뮬레이션으로 시작**한다.
   ARIS 가 `aris3_real_sim.launch.py`(Gazebo 월드·카메라·그리퍼)를 띄우고(이미 떠 있으면 그것을 씀), 브리지는 두산 가상
   컨트롤러로 붙는다. 창을 닫으면 ARIS 가 띄운 월드도 같이 끈다. `--sim` 을 주면 로봇이 있어도 시뮬레이션.
2. 창이 열리면 **접속 → 조작 허용 → 홈 자세**로 시작한다 (실물·시뮬레이션 같음). 실물이면 접속 전에 확인창을 한 번
   띄운다(주변이 비었는지·비상정지). 3 분 안에 준비되지 않으면 그만두고 로그에 남긴다. `--no-auto-home` 으로 끈다.

음성 대답 (2026-11-11): 음성 명령마다 ARIS 가 소리로 대답한다 (`/aris3/voice/say` -> aris3_voice 가 말함).
실행하면 「네, 노란색 큐브를 잡습니다.」, 못 하면 이유(「조작 허용을 먼저 켜 주세요.」 「카메라에 노란색 큐브가 보이지
않아요.」). 문장은 `gui_logic.cpp` 의 `voiceReply` (받침에 맞는 조사, 호출어 「아리스」 를 넣지 않음 - 시험 3개).

색 (2026-11-12): 아이언맨 테마 6가지 (`theme.cpp`, 핫로드 레드·금색·아크 리액터 청록). 기본은
**자비스 HUD** - 남색 유리판 + 청록 제목줄·글자·테두리 (빨강은 정지·오류에만). 실행할 때 `ARIS_THEME=classic|gold|night|hotrod|mark85`
로 바꿀 수 있다. 정지 버튼은 어느 테마에서나 밝은 빨강 + 노란 굵은 테두리.


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
  시험: `ros2 run aris3_gui_cpp aris --mode sim --selftest` (시뮬레이션이 떠 있을 때, 조작 허용·홈·조그·화면 선택 작업 자동 확인)
- **M3**: 실물 모드 `--mode real` (`aris3_real_bridge` 를 통해서만 로봇을 움직임). 실물 집기 패널(화면 클릭으로
  물체·놓을 자리 고르기, 집기/놓기/집어서 옮기기 - 모두 확인 창), 보정 불러오기·상태, 손짓 이벤트, 손 안내, 복구 홈.
  `--sim` 이면 가상 컨트롤러(DRCF 에뮬레이터) + `aris3_real_sim.launch.py` 로 실물과 비슷하게 시험.
  시험: `ros2 run aris3_gui_cpp aris --mode real --sim --selftest` (접속·조작 허용·홈·보정·선택·집어서 옮기기·정지 7가지)

| 파일 | 하는 일 |
|---|---|
| `include/aris3_gui_cpp/backend.hpp` | 모드 공통 `Backend` 와 프로세스 그룹 관리 `LaunchProc` |
| `src/real_backend.cpp` | 실물 모드: `/aris3/real/*` 상태·서비스·액션, 브리지·음성·손짓 프로세스 |
| `src/main_window_real.cpp` | 실물 집기 패널, 실물 화면 갱신, 실물 자체 시험 |

## 음성 명령 (실물 모드, 2026-11-11)

「아리스 …」 로 말하면 확인창 없이 바로 실행한다 (버튼·클릭·손짓은 계속 확인창). 조작 허용·안전 검사·정지는 그대로.
「아리스」 만 말하고 쉬었다가 이어 말해도 된다 (8 초 안).

| 말 | 동작 |
|---|---|
| 노란색 큐브 잡아 / 이거 저기로 옮겨줘 / 저기 놔줘 | 집기 / 집어서 옮기기 / 고른 자리에 놓기 |
| 노란 큐브를 파란 큐브 위로 옮겨줘 / 이거 파란 큐브 위에 올려 | 쌓기 (2026-11-13): 두 색이 카메라에 하나씩 보일 때만. 브리지 `PICK_AND_STACK` (놓는 높이 + 큐브 한 개: 실물 40 · 시뮬레이션 50 mm) |
| (색이 안 보일 때) | 가려진 색 찾기 (2026-11-13): 말한 색이 팔에 가려 안 보이면 J1 을 10° 씩 천천히(15 %) +20 → +40 → −20 → −40 → 0° 로 돌리며 찾고, 찾으면 원래 명령을 이어서. `searchTick()`, 정지하면 그만 |
| 홈 | 홈 자세 [0, 0, 90, 0, 90, 0]° |
| 멈춰 (정지, 그만) | 정지 버튼과 같음, 받는 즉시 (인식까지 약 1 초 - 비상정지 대신이 아님) |
| 계속해 (이어서) | 정지로 끊긴 집기 순서를 멈춘 단계부터. 정지 뒤 다른 동작을 했으면 잇지 않음 |
| 가동 | 접속 -> (조작 허용이 켜지면) 모든 관절 0° 로 팔을 위로 일자로. 집기 전에는 「홈」 |
| 연결해제 | 접속 끊기 (서보 꺼짐) |

## 안전

정지는 언제나 동작한다 (버튼, Space, Esc). 화면의 정지는 인증된 비상정지 장치를 대신하지 않는다.
