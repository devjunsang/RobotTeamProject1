# ARIS3 — 말과 손짓으로 움직이는 Pick & Place 로봇

두산 **E0509** 로봇 팔과 **RH-P12-RN-A** 그리퍼로 작업대 위 큐브를 집어 옮기는 ROS 2 프로젝트입니다.
조종 화면 **ARIS**에서 버튼, 카메라 화면 클릭, 손짓, **음성**(「아리스, 노란 큐브 잡아」)으로 명령하면
ARIS가 「네, 노란색 큐브를 잡습니다.」라고 대답하고 움직입니다.

- 환경: Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · MoveIt 2
- 실물: 두산 E0509 (컨트롤러 110.120.1.66:12345) + RH-P12-RN-A + 위쪽 RealSense D455
- 실물 로봇이 없으면 같은 실물 경로를 **두산 가상 컨트롤러(DRCF) + Gazebo**로 시험합니다.

> 실물 지원은 시험 중입니다. 시뮬레이션에서 성공했다고 실물에서도 된다는 뜻은 아닙니다.
> 비상정지는 항상 손이 닿는 곳에 두세요. 음성 「멈춰」는 약 1초가 걸리므로 비상정지를 대신하지 않습니다.

---

## 빠른 시작

```bash
source ~/ros2_ws/install/setup.bash
export ANTHROPIC_API_KEY=...        # 음성 명령 LLM 해석 (없으면 낱말 표 해석기만 씀)
ros2 run aris3_gui_cpp aris
```

ARIS가 켜지면 이렇게 동작합니다.

1. 실물 로봇(110.120.1.66:12345)이 1.5초 안에 응답하면 **실물**, 아니면 **시뮬레이션**으로 시작합니다.
   시뮬레이션이면 ARIS가 Gazebo 월드(카메라·그리퍼·큐브)를 직접 띄우고, 월드 준비가 끝난 뒤 가상 컨트롤러에 접속합니다.
2. **접속 → 조작 허용 → 홈 자세**까지 저절로 진행합니다. 실물이면 접속 전에 확인창을 한 번 띄웁니다.
3. 「음성 명령 모드」를 켜고 「아리스, …」로 말합니다.

| 옵션 | 뜻 |
|---|---|
| `--sim` | 실물 로봇이 있어도 시뮬레이션으로 |
| `--no-auto-home` | 창이 열려도 자동 접속·홈 하지 않음 |
| `--no-voice` · `--no-gesture` | 음성·손짓 노드를 띄우지 않음 |
| `--mode sim` | 예전 MoveIt 시뮬레이션 모드 (Connect = `aris3_sim_full.launch.py`) |

자세한 순서는 [실행·테스트 가이드](docs/ARIS3_실행_테스트_가이드.md)에 있습니다.

## 음성 명령

「아리스」로 부른 뒤 말합니다. 「아리스」만 말하고 쉬었다가(8초 안) 명령해도 됩니다. ARIS는 모든 명령에 소리로 대답합니다.

| 말 | 동작 | ARIS 대답 |
|---|---|---|
| 아리스, 노란 큐브 잡아 | 그 색 큐브 집기 | 네, 노란색 큐브를 잡습니다. |
| 아리스, 이거 저기로 옮겨 | 화면에서 고른 물체를 고른 자리로 | 네, 고르신 물체를 집어서 고르신 자리로 옮깁니다. |
| 아리스, 노란 큐브를 파란 큐브 위로 옮겨줘 | 노랑을 집어 파랑 위에 쌓기 | 네, 노란색 큐브를 파란색 큐브 위에 올립니다. |
| 아리스, 내려놔 | 들고 있는 것 내려놓기 | 네, 여기에 내려놓겠습니다. |
| 아리스, 홈으로 | 홈 자세 | 네, 홈 자세로 돌아갑니다. |
| 아리스, 멈춰 | 정지 (LLM을 거치지 않고 바로) | 네, 멈춥니다. |
| 아리스, 계속해 | 멈춘 집기를 이어서 | 네, 이어서 하겠습니다. |
| 아리스, 가동 | 접속 후 팔을 위로 일자로 펴기 | 네, 가동합니다. |
| 아리스, 연결해제 | 접속 끊기 (서보 꺼짐) | 네, 연결을 해제합니다. |
| 아리스 | (부르기만) | 네, 말씀하세요. |

못 할 때는 이유를 말합니다. 예: 「조작 허용을 먼저 켜 주세요.」, 「카메라에 노란색 큐브가 보이지 않아요.」, 「잘 못 알아들었어요. 다시 말씀해 주세요.」

처리 흐름:

```
마이크 ─▶ Whisper large-v3-turbo (GPU) ─▶ 호출어 「아리스」 ─▶ Claude Haiku 5.5 해석 ─▶ ARIS 화면 ─▶ 실물 브리지 ─▶ 로봇
                                                         (실패하면 낱말 표 해석기)   │
                                                                                     └─▶ 대답 문장 ─▶ ARIS 목소리 (노트북 내장 스피커)
```

- **조작 허용**은 음성으로 켜지지 않습니다. 사람이 주변을 보고 체크합니다(2분 동안 명령이 없으면 잠김).
- LLM은 정해진 명령 목록(JSON 스키마) 밖의 일을 시킬 수 없고, 모든 동작은 화면과 브리지의 안전 검사를 거칩니다.
- 자세한 내용: [aris3_voice/README.md](aris3_voice/README.md)

## 패키지

| 패키지 | 역할 |
|---|---|
| [`aris3_gui_cpp`](aris3_gui_cpp/README.md) | **ARIS** — C++ Qt5 조종 화면 (상태·카메라·수동 조작·실물 집기·음성·손짓) |
| `aris3_gui` | ARIS3 Control Station — 같은 기능의 Python(PyQt5) 버전 |
| [`aris3_voice`](aris3_voice/README.md) | 음성 인식(Whisper) → 명령 해석(Claude Haiku / 낱말 표) → ARIS 목소리(TTS) |
| `aris3_gesture` | 손짓 인식 (MediaPipe) — 손으로 물체와 놓을 자리를 가리킴 |
| [`aris3_real_bridge`](aris3_real_bridge/README.md) | 실물 E0509 브리지 — 접속·이동·그리퍼·복구·카메라 보정·집기 순서 |
| `aris3_interfaces` | 노드 공통 메시지·서비스·액션 |
| `aris3_task_manager` | Pick & Place 작업 순서 관리 (C++, 시뮬레이션 MoveIt 경로) |
| `aris3_sim_bridge` | Gazebo 작업 셀을 ARIS3 인터페이스에 맞추는 어댑터 (그리퍼·관절 따라가기·카메라) |
| `aris3_bringup` | ROS 2 + Gazebo 통합 실행 (`aris3_real_sim.launch.py` = 실물 비슷한 시뮬레이션) |
| `aris3_moveit_config` | E0509 + 그리퍼 MoveIt 2 설정 |
| `aris3_vision` | 카메라 영상 처리·물체 위치 추정 |
| `aris3_description` · `e0509_gripper_description` | 로봇 + 그리퍼 통합 모델 |
| `aris3_control` | 관절·그리퍼 제어 |

각 패키지의 `ROLE.txt`에 역할, 현재 상태, 제약이 적혀 있습니다.

## 빌드

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select aris3_interfaces          # 인터페이스 (--symlink-install 없이)
source install/setup.bash
colcon build --symlink-install --packages-select aris3_bringup aris3_sim_bridge aris3_real_bridge \
  aris3_task_manager aris3_voice aris3_gesture aris3_gui aris3_gui_cpp
```

인터페이스가 바뀌면 그것을 쓰는 패키지를 모두 다시 빌드하고, 실행 중인 노드는 껐다 켭니다.

음성 가상환경(처음 한 번):

```bash
python3 -m venv --without-pip ~/venvs/mark1_voice
P="python3 -m pip --python $HOME/venvs/mark1_voice/bin/python install"
$P faster-whisper "numpy<2" anthropic edge-tts                 # 인식·LLM·목소리
$P nvidia-cublas-cu12 "nvidia-cudnn-cu12==9.*"                 # GPU Whisper (없으면 CPU base 모델로)
```

## 시험

```bash
colcon test --packages-select aris3_gui_cpp aris3_voice aris3_real_bridge aris3_sim_bridge
colcon test-result --verbose
ros2 run aris3_gui_cpp aris --mode real --sim --selftest      # 실물 경로 자체 시험 7단계 (가상 컨트롤러)
```

음성 LLM 실제 API 시험(키가 있을 때만 돎):

```bash
cd aris3_voice && ANTHROPIC_API_KEY=... PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ~/venvs/mark1_voice/bin/python -m pytest -q test/test_llm_parser.py
```

## 문서

`docs/`의 문서는 HTML로도 볼 수 있습니다(`docs/index.html`). `.md`를 고친 뒤 `python3 scripts/docs_md_to_html.py`로 다시 만듭니다.

| 문서 | 내용 |
|---|---|
| [**ARIS3 최종 통합 보고서**](docs/ARIS3_최종_통합보고서.html) | 마스터플랜(11.05) 대비 실제 실행 결과 전체: A–Z 26단계 결과, GATE 판정, 날짜별 기록, 최종 구조도, 검증 수치, 버그 사례, 남은 일 (2026-11-14, 단일 HTML) |
| [ARIS3 학습 가이드](docs/ARIS3_학습가이드.html) | 전체 통신 흐름, 실물 집기 순서, 안전장치, 버그 사례 |
| [실행·테스트 가이드](docs/ARIS3_실행_테스트_가이드.md) | 설치, 빌드, ARIS 실행, 음성 설정, 문제 해결 |
| [실물 집기·보정 기능](docs/ARIS3_실물_집기_보정_기능.md) | D455 깊이 층·원근 보정·각도, 그리퍼 각도·중심 맞추기, 탁자 블록 마커 보정·자동 기록, 「그리퍼 열어」 (2026-11-13~14) |
| [개발 방향 확정](docs/ARIS3_개발방향_확정.md) | 목표와 단계 |
| [Task Manager 학습 가이드](docs/ARIS3_TaskManager_학습가이드.md) | 시뮬레이션 작업 순서 관리 |
| [E0509 실기 연동 학습 가이드](docs/E0509_실기연동_학습가이드.md) | 실물 로봇 접속·보정 |

## 보안

- API 키는 코드나 저장소에 넣지 않습니다. `ANTHROPIC_API_KEY` 환경 변수로만 넘깁니다.
- ARIS 목소리(edge-tts)와 LLM 해석은 인터넷을 씁니다. 끊기면 목소리는 조용히 넘어가고, 해석은 낱말 표로 바뀝니다. 로봇 동작은 영향을 받지 않습니다.
