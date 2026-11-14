"""LLM 음성 명령 해석기 시험 (2026-11-11).

앞부분은 네트워크·키 없이 돈다 (가짜 응답). 맨 아래 실제 API 시험은 ANTHROPIC_API_KEY 가 있고
anthropic 패키지가 있을 때만 돈다 (없으면 건너뜀):
  ANTHROPIC_API_KEY=... ~/venvs/mark1_voice/bin/python -m pytest -q aris3_voice/test/test_llm_parser.py
"""
import json
import os
from types import SimpleNamespace

import pytest

from aris3_voice import llm_parser
from aris3_voice.command_parser import validate_command
from aris3_voice.llm_parser import LlmParser, to_command


class FakeMessages:
    """client.messages.create 대신: 정해 둔 JSON 을 text 블록으로 돌려준다."""

    def __init__(self, payload, stop_reason='end_turn'):
        """payload: dict(-> JSON) 또는 문자열(그대로), stop_reason: 응답의 stop_reason."""
        self.payload, self.stop_reason, self.calls = payload, stop_reason, []

    def create(self, **kw):
        """요청 인자를 기록하고 가짜 응답을 만든다."""
        self.calls.append(kw)
        text = self.payload if isinstance(self.payload, str) else json.dumps(self.payload)
        return SimpleNamespace(stop_reason=self.stop_reason,
                               content=[SimpleNamespace(type='text', text=text)])


def make(payload, stop_reason='end_turn'):
    """가짜 클라이언트를 단 LlmParser (anthropic 패키지가 없어도 켠 상태로)."""
    p = LlmParser(log=lambda s: None)
    p.enabled = True
    p.client = SimpleNamespace(messages=FakeMessages(payload, stop_reason))
    return p


def test_to_command_same_shape_as_rule_parser():
    """LLM 결과는 parse_command 와 같은 키 -> validate_command 를 그대로 쓴다."""
    cmd = to_command({'action': 'pick', 'color': 'blue', 'shape': 'cube', 'ref_object': False,
                      'ref_place': False, 'ambiguous': False}, '빨간 거 말고 파란 거 잡아')
    assert cmd['action'] == 'pick' and cmd['color'] == 'blue'
    assert set(cmd) >= {'action', 'color', 'shape', 'ref_object', 'ref_place', 'ambiguous',
                        'original_text', 'normalized_text'}
    assert validate_command(cmd)[0]


def test_to_command_rejects_values_outside_schema():
    """스키마 밖 값(지어낸 동작·색)은 빈 값 -> 실행 불가로 떨어진다 (이중 확인)."""
    cmd = to_command({'action': 'throw', 'color': 'purple', 'target_color': 'black'}, '보라색 던져')
    assert cmd['action'] == '' and cmd['color'] == '' and cmd['target_color'] == ''
    assert not validate_command(cmd)[0]


def test_to_command_cube_numbers():
    """큐브 번호 (2026-11-14): 0 ~ 9 의 정수만 받고, 없거나 이상한 값(문자열·범위 밖·bool)은 0."""
    cmd = to_command({'action': 'pick', 'color': 'blue', 'target_color': 'blue', 'number': 1, 'target_number': 2},
                     '1번 파란 큐브를 2번 파란 큐브 위로')
    assert (cmd['number'], cmd['target_number']) == (1, 2)
    assert validate_command(cmd)[0]                              # 같은 색이라도 번호가 다르면 쌓기
    assert to_command({'action': 'pick', 'color': 'red'}, '빨간 큐브')['number'] == 0
    for bad in ('2', 12, -1, True, None, 1.5):
        assert to_command({'action': 'pick', 'color': 'red', 'number': bad}, 'x')['number'] == 0, bad


def test_parse_with_fake_response_and_request_shape():
    """가짜 응답 해석 + 요청이 Haiku·낮은 effort·JSON 스키마로 나가는지."""
    p = make({'action': 'home', 'color': '', 'shape': '', 'ref_object': False,
              'ref_place': False, 'ambiguous': False})
    assert p.parse('처음 자세로 돌아가')['action'] == 'home'
    kw = p.client.messages.calls[0]
    assert kw['model'] == 'claude-haiku-5-5'
    assert kw['output_config']['effort'] == 'low'
    assert kw['output_config']['format']['type'] == 'json_schema'


def test_parse_returns_none_on_refusal_or_bad_json():
    """거절·잘림·형식 오류면 None -> 부르는 쪽이 낱말 표를 쓴다."""
    assert make({}, stop_reason='refusal').parse('노란 큐브 잡아') is None
    assert make({}, stop_reason='max_tokens').parse('노란 큐브 잡아') is None
    assert make('{깨진 json').parse('노란 큐브 잡아') is None


def test_disabled_parser_returns_none():
    """꺼져 있으면(키·패키지 없음) 부르지 않고 None."""
    p = LlmParser(log=lambda s: None)
    p.enabled = False
    assert p.parse('노란 큐브 잡아') is None


def test_missing_key_turns_parser_off():
    """키가 없을 때 SDK 가 내는 TypeError -> 해석기를 끈다 (다음부터 기다리지 않음)."""
    p = LlmParser(log=lambda s: None)
    p.enabled = True

    def no_key(**_kw):
        """인증 방법이 없을 때의 SDK 예외 흉내."""
        raise TypeError('Could not resolve authentication method. Expected one of api_key ...')
    p.client = SimpleNamespace(messages=SimpleNamespace(create=no_key))
    assert p.parse('노란 큐브 잡아') is None
    assert not p.enabled and '키' in p.why_off


# ---- 실제 API (키가 있을 때만): 낱말 표가 잘 읽는 문장 + 낱말 표가 못 읽는 문장
LIVE = (llm_parser.anthropic is not None) and bool(os.environ.get('ANTHROPIC_API_KEY'))


@pytest.mark.skipif(not LIVE, reason='ANTHROPIC_API_KEY 또는 anthropic 패키지 없음')
@pytest.mark.parametrize('text, action, color', [
    ('노란색 큐브 잡아', 'pick', 'yellow'),
    ('이거 저기로 옮겨줘', 'pick', ''),
    ('홈으로 가', 'home', ''),
    ('계속해', 'resume', ''),
    ('연결 해제', 'disconnect', ''),
    ('가동', 'power_on', ''),
    ('그리퍼 열어', 'gripper_open', ''),
    ('손 좀 펴 줄래', 'gripper_open', ''),
    ('빨간 거 말고 파란 거 들어줘', 'pick', 'blue'),       # 낱말 표는 빨강으로 읽음
    ('노란색 큐구 자바', 'pick', 'yellow'),                # 음성 인식 오타
    ('처음 자세로 돌아가 줄래', 'home', ''),
    ('오늘 날씨 어때', '', ''),                            # 명령 아님
])
def test_live_haiku(text, action, color):
    """Claude Haiku 5.5 가 기대한 동작·색으로 읽는가."""
    p = LlmParser(timeout_s=10.0)
    cmd = p.parse(text)
    assert cmd is not None, p.why_off
    assert (cmd['action'], cmd['color']) == (action, color), cmd
    assert cmd['target_color'] == '', cmd


@pytest.mark.skipif(not LIVE, reason='ANTHROPIC_API_KEY 또는 anthropic 패키지 없음')
@pytest.mark.parametrize('text, color, target', [
    ('노란색 큐브를 파란색 큐브 위로 옮겨줘', 'yellow', 'blue'),
    ('파란 거 위에 빨간 거 올려', 'red', 'blue'),
    ('노란 큐브 초록 큐브 위에 쌓아 줘', 'yellow', 'green'),
])
def test_live_haiku_stack(text, color, target):
    """쌓기 2중 명령: 집을 색과 받침 색을 나눠 읽는가."""
    p = LlmParser(timeout_s=10.0)
    cmd = p.parse(text)
    assert cmd is not None, p.why_off
    assert (cmd['action'], cmd['color'], cmd['target_color']) == ('pick', color, target), cmd


@pytest.mark.skipif(not LIVE, reason='ANTHROPIC_API_KEY 또는 anthropic 패키지 없음')
@pytest.mark.parametrize('text, color, number, target, target_number', [
    ('1번 파란색 큐브 집어줘', 'blue', 1, '', 0),
    ('파란 큐브 2번 잡아', 'blue', 2, '', 0),
    ('두 번째 빨간 큐브 들어줘', 'red', 2, '', 0),
    ('이번에는 파란 큐브 집어줘', 'blue', 0, '', 0),                 # 「이번에는」 은 번호가 아니다
    ('1번 파란 큐브를 2번 파란 큐브 위로 옮겨줘', 'blue', 1, 'blue', 2),
    ('노란 큐브를 2번 파란 큐브 위에 올려', 'yellow', 0, 'blue', 2),
])
def test_live_haiku_cube_number(text, color, number, target, target_number):
    """큐브 번호 (2026-11-14): 집을 큐브·받침 큐브의 번호를 읽는가 (스키마의 정수 enum 이 API 에서 받아들여지는지도 함께)."""
    p = LlmParser(timeout_s=10.0)
    cmd = p.parse(text)
    assert cmd is not None, p.why_off
    assert (cmd['action'], cmd['color'], cmd['number'], cmd['target_color'], cmd['target_number']) == \
        ('pick', color, number, target, target_number), cmd

