"""LLM(Claude Haiku 5.5) 음성 명령 해석기 (2026-11-11).

Whisper 가 적은 문장 -> command_parser.parse_command 와 **같은 모양의 dict** (action, color, shape, ref_object,
ref_place, ambiguous, original_text, normalized_text, target_color, number, target_number). 그래서 validate_command·
VoiceCommand·GUI·브리지는 그대로다. number·target_number (2026-11-14): 「1번 파란색 큐브」 의 큐브 번호, 0 = 말하지 않음.

  - 출력은 JSON 스키마로 강제한다 (output_config.format) - LLM 은 정해진 동작 목록 밖의 일을 시킬 수 없다.
    로봇을 실제로 움직이는 것은 여전히 GUI 와 브리지의 안전 검사를 거친다.
  - 「멈춰」 는 이 해석기를 거치지 않는다 (voice_node 가 낱말 표로 먼저 처리 - 정지는 네트워크에 기대지 않는다).
  - 실패하면 None 을 돌려주고, 부르는 쪽은 낱말 표 해석기(parse_command)를 쓴다:
      키 없음 / 인증 실패 -> 이 해석기를 끈다 (매번 기다리지 않게)
      시간 초과 / 연결 실패 / 속도 제한 / 서버 오류 / 거절 / 형식 오류 -> 이번만 None
  - anthropic 패키지가 없는 Python 에서도 import 는 된다 (그때는 꺼진 채로).

키: ANTHROPIC_API_KEY 환경 변수 (또는 SDK 가 읽는 다른 인증 방법). 코드에 키를 적지 않는다.
"""
import json
import time

try:                                    # 음성 가상환경(~/venvs/mark1_voice)에 설치. 없으면 꺼진 채로
    import anthropic
except ImportError:                     # pragma: no cover - 설치 환경에 따라
    anthropic = None

# 동작·색·도형: command_parser 와 같은 값만 (빈 문자열 = 없음/모름)
ACTIONS = ['pick', 'place', 'home', 'stop', 'resume', 'disconnect', 'power_on', 'gripper_open', 'find', 'scan', '']
COLORS = ['red', 'blue', 'green', 'yellow', '']
SHAPES = ['cube', 'triangle', 'circle', '']
NUMBERS = list(range(10))               # 큐브 번호 0(말하지 않음) ~ 9 (2026-11-14)

# 응답 JSON 스키마 - 이 밖의 값은 나올 수 없다
SCHEMA = {
    'type': 'object',
    'properties': {
        'action': {'type': 'string', 'enum': ACTIONS},
        'color': {'type': 'string', 'enum': COLORS},
        'shape': {'type': 'string', 'enum': SHAPES},
        'target_color': {'type': 'string', 'enum': COLORS},
        'number': {'type': 'integer', 'enum': NUMBERS},
        'target_number': {'type': 'integer', 'enum': NUMBERS},
        'ref_object': {'type': 'boolean'},
        'ref_place': {'type': 'boolean'},
        'ambiguous': {'type': 'boolean'},
    },
    'required': ['action', 'color', 'shape', 'target_color', 'number', 'target_number', 'ref_object', 'ref_place',
                 'ambiguous'],
    'additionalProperties': False,
}

# 시스템 프롬프트: 매번 같다 (프롬프트 캐싱이 되도록 바뀌는 값은 넣지 않는다)
SYSTEM = """너는 로봇 팔 ARIS 의 한국어 음성 명령 해석기다. 사람이 한 말(음성 인식 결과라 오타가 있을 수 있음)을
정해진 명령 하나로 바꾼다. 호출어 「아리스」 는 이미 떼어 냈다. 로봇을 직접 움직이지 않고, 해석 결과만 낸다.

작업 공간에는 색이 다른 큐브가 있다 (빨강 red, 파랑 blue, 초록 green, 노랑 yellow).
같은 색 큐브가 여럿이면 화면에 번호가 붙는다 (1번 파란색 큐브, 2번 파란색 큐브 ...). 사람은 그 번호로 큐브를 고를 수 있다.
사람은 화면을 클릭하거나 손으로 가리켜 물체(「이거/그거/저거」)와 자리(「저기/여기/거기」)를 고를 수 있다.

action:
- pick: 물체를 집기/들기/옮기기. 「잡아」「집어」「들어」「가져와」「옮겨」.
  물체와 자리가 같이 나오면(「이거 저기로 옮겨」) pick + ref_place=true (집어서 그 자리에 놓기).
- place: 들고 있는 것을 내려놓기. 「내려놔」「놔줘」「놓아」. 자리를 말하면 ref_place=true.
  집을 물체를 말하지 않고 받침 큐브만 말하면(「빨간 큐브 위로 올려놔」「2번 파란 큐브 위에 놓아」) 지금 들고 있는 것을
  그 큐브 위에 올려놓기다 -> place, color="", target_color=받침 색 (번호를 말했으면 target_number), ref_object=false.
- home: 홈(기본) 자세로. 「홈」「원위치」「제자리로」「처음 자세」「집으로 돌아가」(집 = 홈, 집기 아님).
- stop: 멈추기. 「멈춰」「정지」「그만」「스톱」「잠깐」.
- resume: 멈춘 작업을 이어서. 「계속해」「이어서 해」「다시 진행」.
- disconnect: 로봇 연결 끊기. 「연결 해제」「연결 끊어」「접속 끊어」.
- power_on: 로봇 가동(접속 후 팔을 펴기). 「가동」「깨어나」「일어나」「작동 시작」.
- gripper_open: 그리퍼(손·집게)만 열기, 팔은 움직이지 않음. 「그리퍼 열어」「손 펴」「집게 벌려」.
  그리퍼·손·집게를 말하지 않은 「열어」 만은 gripper_open 이 아니다 ("" 로).
- find: 그 색 큐브를 찾기만 하기 (집지 않음). 「파란색 큐브 찾아」「빨간 큐브 어디 있어」. color 에 찾을 색 (번호를 말했으면 number).
  집으라는 말이 함께 있으면(「파란 큐브 찾아서 집어줘」) find 가 아니라 pick 이다.
- scan: 색을 말하지 않고 주변을 찾아보라는 말. 「탐색」「탐색해」「둘러봐」「큐브 찾아봐」(색 없음). color 는 "".
  색을 말했으면 scan 이 아니라 find 다 (「파란 큐브 탐색해」 -> find, color=blue).
- "" : 위 어느 것도 아니거나 로봇 명령이 아닌 말 (잡담, 질문 등).

color: 집을 물체의 색. 부정(「빨간 거 말고 파란 거」)이면 실제로 집을 색(blue). 말하지 않았으면 "".
shape: 「큐브/정육면체/블록/네모/사각형」 -> cube, 「세모/삼각형」 -> triangle, 「동그라미/원」 -> circle, 없으면 "".
target_color: 쌓기. 집은 큐브를 다른 큐브 「위에/위로」 놓으라고 하면 그 받침 큐브의 색, 아니면 "".
  「노란 큐브를 파란 큐브 위로 옮겨줘」 -> pick, color=yellow, target_color=blue.
  「파란 거 위에 빨간 거 올려」 -> pick, color=red, target_color=blue (위에 놓이는 쪽이 color).
  「이거 초록 큐브 위에 쌓아」 -> pick, color="", ref_object=true, target_color=green.
  쌓기일 때 ref_place 는 false (놓을 자리는 받침 큐브).
number: 집을 큐브의 번호. 「1번 파란색 큐브 집어줘」「파란 큐브 2번 잡아」「두 번째 빨간 큐브」 -> 1, 2, 2. 번호를 말하지 않았으면 0.
  「일번」 = 1, 「이번 큐브」「파란 큐브 이번」 = 2 (단, 「이번에는 파란 큐브」 의 「이번에는」 은 번호가 아니다 -> 0), 「삼번」 = 3.
target_number: 쌓기 받침 큐브의 번호. 「1번 파란 큐브를 2번 파란 큐브 위로」 -> color=blue, number=1, target_color=blue,
  target_number=2 (같은 색끼리도 번호가 다르면 쌓기다). 받침 번호를 말하지 않았으면 0.
ref_object: 「이거/그거/저거/이것」 처럼 화면에서 고른 물체를 가리키면 true. 색을 말했으면 false.
ref_place: 「저기/여기/거기/그 자리/고른 자리」 처럼 화면에서 고른 자리를 가리키면 true.
ambiguous: 집으라는 건지 다른 뜻인지 정말 판단할 수 없을 때만 true (예: 「빨간 거 지워줘」 - 「집어줘」 오인식일 수도).

규칙:
- 확실하지 않으면 지어내지 말고 action 을 "" 로 하거나 ambiguous=true 로 한다. 잘못 움직이는 것보다 다시 묻는 게 낫다.
- 문장은 음성 인식(Whisper)이 적은 것이라 소리가 비슷한 다른 낱말로 잘못 적혀 있는 경우가 많다.
  글자 그대로 읽지 말고, 소리 내어 읽었을 때 위 명령 중 무엇과 비슷한지로 판단한다. 실제로 있었던 예:
    「노란색 큐구 자바」 -> 노랑 큐브 집기      「바란색」 -> 파랑
    「혐의로」 「허무로」 「흠으로」 -> 홈으로 (home)     「먹쳐」 「멈쳐」 -> 멈춰 (stop)
    「내려나」 「되려나」 「내려 놔」 -> 내려놔 (place)    「자바」 「자파」 「잡하」 -> 잡아 (pick)
    「가동해」 「가도」 -> 가동 (power_on)
  짧은 말이 명령 하나와 소리가 매우 비슷하면 그 명령으로 본다. 어느 명령과도 비슷하지 않으면 "" 로 한다.
- 「A 를 B 위로 옮겨」 는 쌓기 명령 하나다 (color=A, target_color=B). 그 밖에 명령이 여러 개면 맨 처음 것 하나만.
"""


class LlmParser:
    """Claude 로 음성 문장을 명령 dict 로 바꾼다. 실패하면 None (부르는 쪽이 낱말 표 해석기를 쓴다)."""

    def __init__(self, model='claude-haiku-5-5', timeout_s=4.0, log=print):
        """model: Claude 모델 ID, timeout_s: 한 번 기다릴 최대 시간 (넘으면 낱말 표로), log: 문자열 로그 함수."""
        self.model, self.timeout_s, self.log = model, float(timeout_s), log
        self.enabled = anthropic is not None
        self.why_off = '' if self.enabled else 'anthropic 패키지 없음'
        self.client = None
        if self.enabled:
            # 다시 시도하지 않는다 (음성 명령은 늦게 오는 것보다 낱말 표로 바로 하는 게 낫다)
            self.client = anthropic.Anthropic(timeout=self.timeout_s, max_retries=0)
        self.last_latency = 0.0

    def _off(self, why):
        """이 해석기를 끈다 (키 없음·인증 실패처럼 다시 해도 안 되는 경우) - 로그는 한 번만."""
        if self.enabled:
            self.log(f'LLM 해석 끔: {why} - 낱말 표 해석기만 사용')
        self.enabled, self.why_off = False, why

    def parse(self, text):
        """문장 -> 명령 dict (command_parser.parse_command 와 같은 키) 또는 None (실패, 낱말 표를 쓸 것)."""
        if not self.enabled or not text or not text.strip():
            return None
        t0 = time.monotonic()
        try:
            resp = self.client.messages.create(
                model=self.model,
                max_tokens=1024,                              # 짧은 JSON + 짧은 생각
                output_config={'effort': 'low',               # 짧은 분류 작업: 빠르게
                               'format': {'type': 'json_schema', 'schema': SCHEMA}},
                system=[{'type': 'text', 'text': SYSTEM, 'cache_control': {'type': 'ephemeral'}}],
                messages=[{'role': 'user', 'content': text.strip()}],
            )
        except TypeError as e:                                # 인증 방법 없음 (키가 설정되지 않음)
            if 'authentication' in str(e).lower():
                self._off('API 키 없음 (ANTHROPIC_API_KEY)')
                return None
            raise
        except Exception as e:  # noqa: BLE001 - 아래에서 종류별로 나눈다
            return self._on_error(e)
        finally:
            self.last_latency = time.monotonic() - t0
        if resp.stop_reason != 'end_turn':                    # refusal / max_tokens 등 -> 낱말 표로
            self.log(f'LLM 해석 실패: stop_reason={resp.stop_reason}')
            return None
        try:
            raw = next(b.text for b in resp.content if b.type == 'text')
            data = json.loads(raw)
        except (StopIteration, ValueError) as e:
            self.log(f'LLM 해석 실패: 응답 형식 ({e})')
            return None
        return to_command(data, text)

    def _on_error(self, e):
        """API 오류 처리: 인증 실패면 끄고, 나머지(시간 초과·연결·속도 제한·서버)는 이번만 None."""
        if anthropic is not None:
            if isinstance(e, (anthropic.AuthenticationError, anthropic.PermissionDeniedError)):
                self._off(f'인증 실패 ({type(e).__name__})')
                return None
            if isinstance(e, anthropic.APITimeoutError):      # APIConnectionError 의 하위라 먼저 본다
                self.log(f'LLM 해석: {self.timeout_s:.0f} 초 안에 응답 없음 -> 낱말 표')
                return None
            if isinstance(e, anthropic.APIConnectionError):
                self.log('LLM 해석: 연결 실패 (인터넷 확인) -> 낱말 표')
                return None
            if isinstance(e, anthropic.RateLimitError):
                self.log('LLM 해석: 요청 한도 초과 -> 낱말 표')
                return None
            if isinstance(e, anthropic.APIStatusError):
                self.log(f'LLM 해석: API 오류 {e.status_code} -> 낱말 표')
                return None
        self.log(f'LLM 해석: 알 수 없는 오류 {e!r} -> 낱말 표')
        return None


def to_command(data, text):
    """LLM JSON -> parse_command 와 같은 dict. 스키마 밖 값이 와도 안전한 값으로 바꾼다 (이중 확인)."""
    def pick(key, allowed):
        """allowed 안의 값만, 아니면 빈 문자열."""
        v = data.get(key, '')
        return v if v in allowed else ''

    def num(key):
        """0 ~ 9 의 정수만, 아니면 0 (bool·문자열·범위 밖은 번호 없음으로)."""
        v = data.get(key, 0)
        return v if isinstance(v, int) and not isinstance(v, bool) and v in NUMBERS else 0
    return {
        'action': pick('action', ACTIONS),
        'color': pick('color', COLORS),
        'shape': pick('shape', SHAPES),
        'target_color': pick('target_color', COLORS),
        'number': num('number'),
        'target_number': num('target_number'),
        'ref_object': bool(data.get('ref_object', False)),
        'ref_place': bool(data.get('ref_place', False)),
        'ambiguous': bool(data.get('ambiguous', False)),
        'original_text': text,
        'normalized_text': ''.join(text.split()),
    }
