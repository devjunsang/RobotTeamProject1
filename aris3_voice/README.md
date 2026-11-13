# aris3_voice — 듣고, 이해하고, 대답하기

마이크로 들은 한국어 명령을 글자로 바꾸고(Whisper), 명령으로 해석하고(Claude Haiku 5.5 / 낱말 표),
ARIS 화면이 정한 대답을 ARIS 목소리로 말합니다(edge-tts). **이 노드는 로봇을 움직이지 않습니다.** 실행은 ARIS 화면이 안전 검사를 거쳐 합니다.

```
arecord ─▶ 말 한 마디 자르기 ─▶ Whisper ─▶ 호출어 「아리스」 ─▶ 해석 ─▶ /aris3/voice_command ─▶ ARIS 화면
(16 kHz)   (UtteranceSegmenter)  (GPU)      (WakeGate)        (Haiku / 낱말 표)                       │
                                                                                                    ▼
                                            노트북 내장 스피커 ◀─ edge-tts ◀─ /aris3/voice/say ◀── 대답 문장
```

## 실행

ARIS(`ros2 run aris3_gui_cpp aris`)가 함께 띄웁니다. 따로 띄우려면:

```bash
export ANTHROPIC_API_KEY=...                     # 없으면 낱말 표 해석기만 씀
ros2 launch aris3_voice voice.launch.py          # 음성 가상환경(~/venvs/mark1_voice)의 Python 으로 실행
```

음성 가상환경에 필요한 패키지:

```bash
P="python3 -m pip --python $HOME/venvs/mark1_voice/bin/python install"
$P faster-whisper "numpy<2" anthropic edge-tts
$P nvidia-cublas-cu12 "nvidia-cudnn-cu12==9.*"   # GPU Whisper. 없으면 CPU base 모델로 바뀜
```

## 단계별 설명

### 1. 듣기 — `listening.py`
- **UtteranceSegmenter**: 마이크 소리를 30 ms 조각으로 받아, 방 소음보다 충분히 큰 소리가 시작되면 말로 보고, 0.8초 조용하면 말 한 마디로 잘라 냅니다.
- **WakeGate**: 「아리스 …」로 시작하는 말만 명령으로 받습니다.
  - 「아리스」만 말하고 쉬었다가 8초 안에 한 말도 명령으로 받습니다.
  - Whisper가 잘못 적은 호출어(알리스, 아리수, 아래스, RS …)도 받습니다.
  - ARIS가 말하는 중에 녹음된 말은 ARIS 목소리와 섞여 적히므로, 문장 중간의 「아리스」도 찾습니다. 그래서 말하는 중의 「아리스 멈춰」도 놓치지 않습니다.
  - ARIS 대답에는 「아리스」가 없어서, 마이크가 ARIS 목소리를 다시 들어도 명령이 되지 않습니다.

### 2. 글자로 — Whisper (`voice_node.py`)
- GPU가 되면 `large-v3-turbo`(float16, 한 마디 약 0.2초), 안 되면 CPU `base`(약 0.7초)를 씁니다.
- 명령에 쓰는 낱말(아리스, 홈으로, 잡아, 멈춰, 노란색 …)을 미리 알려 줘(`stt_prompt`) 그쪽으로 적게 합니다.
- 말소리가 없는 구간은 잘라 냅니다(`vad_filter`). 끄면 소음에서 없는 문장을 지어냅니다.

### 3. 이해하기 — `llm_parser.py`, `command_parser.py`
- 「멈춰」(먹쳐, 멈쳐, 정지, 그만 …)는 **낱말 표로 바로** 처리합니다. 정지는 네트워크에 기대지 않습니다.
- 나머지는 **Claude Haiku 5.5**가 먼저 해석합니다.
  - 답은 JSON 스키마로 강제합니다. 동작은 pick, place, home, stop, resume, disconnect, power_on 중 하나이고, 색과 도형도 정해진 값만 됩니다.
  - 쌓기(2중 명령): 「노란 큐브를 파란 큐브 위로 옮겨줘」 → pick, color=yellow, **target_color=blue**. 낱말 표도 같은 규칙(「위」 앞의 색이 받침)으로 읽습니다.
  - 음성 인식이 잘못 적은 말을 소리로 고쳐 읽습니다. 예: 「혐의로」→홈, 「되려나」→놓기, 「노란색 큐구 자바」→노랑 집기.
  - 명령이 아닌 말(「오늘 날씨 어때」)은 빈 동작이 되어 움직이지 않습니다.
- 키가 없거나, 4초 안에 응답이 없거나, 오류가 나면 **낱말 표**로 넘어갑니다. 두 해석이 다르면 로그에 「해석 차이」로 남깁니다.
- 비용: 명령 1번에 약 0.0002달러(Haiku 5.5 기준).

### 4. 대답하기 — `speaker.py`
- 대답 문장은 **ARIS 화면이 정합니다.** 실제로 실행하는지는 화면만 알기 때문입니다. 예: 조작 허용이 꺼져 있으면 「네, 잡습니다」 대신 「조작 허용을 먼저 켜 주세요.」
- 목소리: edge-tts `de-DE-SeraphinaMultilingualNeural`(다국어 여성 음성이 한국어로 말함). 빠르기 -5%.
- 출력: **노트북 내장 스피커**. 시스템 기본 출력이 블루투스여도 PipeWire의 `alsa_output.pci…` 장치로 냅니다.
- 같은 문장은 한 번 만든 소리 파일(`/tmp/aris3_tts_cache`)을 다시 씁니다.
- 밀린 문장은 오래된 것부터 버리고(늦은 대답은 헷갈림), 인터넷이 끊기면 로그만 남기고 넘어갑니다.

## 토픽

| 토픽 | 방향 | 뜻 |
|---|---|---|
| `/aris3/voice/active` (Bool) | 받음 | 음성 명령 모드 켜기/끄기 (ARIS 버튼) |
| `/aris3/voice/text` (String) | 받음 | 마이크 없이 문장 해석 (입력창·시험). 호출어 없이도 받음 |
| `/aris3/voice/say` (String) | 받음 | 이 문장을 ARIS 목소리로 말함 |
| `/aris3/voice/listen` (Bool) | 받음 | (예전) 누르는 동안 녹음 |
| `/aris3/voice_command` (VoiceCommand) | 냄 | 인식 문장 + 해석 결과 + 실행 가능 여부 |
| `/aris3/voice/state` (String) | 냄 | loading · ready · listening · recognizing · error:… |

## 파라미터

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `model` / `cpu_model` | `large-v3-turbo` / `base` | GPU / CPU Whisper 모델 |
| `device` | `auto` | auto(GPU 먼저) · cuda · cpu |
| `stt_prompt` | 명령 낱말 목록 | Whisper에 미리 알려 줄 낱말 (빈 문자열이면 끔) |
| `llm` / `llm_model` / `llm_timeout_s` | `true` / `claude-haiku-5-5` / `4.0` | LLM 해석 |
| `tts` / `tts_voice` / `tts_rate` | `true` / Seraphina / `-5%` | 목소리 |
| `tts_output` | `internal` | internal(내장 스피커) · ''(시스템 기본) · PipeWire 장치 이름 |
| `vad_min_rms` · `vad_start_ratio` · `vad_end_silence_s` | 300 · 3.0 · 0.8 | 말 자르기 |

## 시험

```bash
cd aris3_voice && python3 -m pytest -q test/          # 키·네트워크·스피커 없이 (60개)
ANTHROPIC_API_KEY=... PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ~/venvs/mark1_voice/bin/python -m pytest -q test/test_llm_parser.py   # 실제 Haiku 10문장 포함
```

## 알려진 한계

- 실제 마이크로 GPU Whisper 인식률을 잰 기록은 아직 없습니다(합성 음성 4문장으로만 확인).
- edge-tts는 키 없이 무료로 쓰는 Microsoft 서비스로, 공식 지원 서비스가 아닙니다. 막히면 유료 클라우드 TTS로 바꿉니다.
- 무슨 말인지 알 수 없는 문장(「킥으로 내려가」)은 LLM도 명령으로 받지 않습니다. 다시 말해야 합니다.
