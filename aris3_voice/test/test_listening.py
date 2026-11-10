"""음성 명령 모드 순수 로직 시험: 호출어(아리스) 떼기, 말 한 마디 자르기."""
import numpy as np

from aris3_voice.command_parser import parse_command, validate_command
from aris3_voice.listening import UtteranceSegmenter, strip_wake_word


def test_wake_word_at_start_is_removed():
    """「아리스 빨간 도형 잡아」 -> 호출어 있음, 나머지 「빨간 도형 잡아」 (쉼표·공백이 있어도)."""
    assert strip_wake_word('아리스 빨간 도형 잡아') == (True, '빨간 도형 잡아')
    assert strip_wake_word('아리스, 빨간 도형 잡아.') == (True, '빨간 도형 잡아')
    assert strip_wake_word('  ARIS 파란 큐브 집어줘') == (True, '파란 큐브 집어줘')


def test_whisper_variants_are_accepted():
    """Whisper 가 다르게 적는 형태 (알리스, 아리수, 아리스야) 도 호출어로 받는다."""
    assert strip_wake_word('알리스 노란 거 잡아')[0]
    assert strip_wake_word('아리수 홈으로')[0]
    assert strip_wake_word('아리스야 내려놔') == (True, '내려놔')


def test_no_wake_word_or_not_at_start_is_ignored():
    """호출어가 없거나 맨 앞이 아니면 명령이 아니다 (다른 사람끼리 하는 말)."""
    assert strip_wake_word('빨간 도형 잡아') == (False, '빨간 도형 잡아')
    assert not strip_wake_word('그거 아리스 빨간 거 잡아')[0]
    assert strip_wake_word('') == (False, '')


def test_wake_command_parses_to_pick():
    """호출어를 뗀 「빨간 도형 잡아」 는 빨강 집기 명령 (도형 = 모양 상관없음)."""
    ok, rest = strip_wake_word('아리스 빨간 도형 잡아')
    cmd = parse_command(rest)
    assert ok and cmd['action'] == 'pick' and cmd['color'] == 'red' and cmd['shape'] == ''
    assert validate_command(cmd, ['red', 'blue', 'yellow'])[0]


def tone(n, amp, rate=16000):
    """크기 amp 인 440 Hz 소리 n 개 표본."""
    t = np.arange(n) / rate
    return (amp * np.sin(2 * np.pi * 440 * t)).astype(np.int16)


def frames(signal, size=480):
    """30 ms(480 표본) 조각들로 나눈다."""
    return [signal[i:i + size] for i in range(0, len(signal) - size + 1, size)]


def run(seg, signal):
    """조각을 차례로 넣고 나온 말들을 모은다."""
    out = []
    for f in frames(signal):
        u = seg.feed(f)
        if u is not None:
            out.append(u)
    return out


def test_segmenter_cuts_one_utterance_with_pre_roll():
    """조용함 1 s -> 말 1 s -> 조용함 1.2 s: 말 하나, 길이는 말 + 앞 여유 + 끝 조용함 정도."""
    noise = np.random.default_rng(0).normal(0, 50, 16000).astype(np.int16)
    sig = np.concatenate([noise, tone(16000, 6000), noise, noise[:3200]])
    outs = run(UtteranceSegmenter(), sig)
    assert len(outs) == 1
    assert 1.0 <= len(outs[0]) / 16000 <= 2.3


def test_segmenter_ignores_short_click_and_quiet_noise():
    """조용한 소음만, 또는 0.1 s 짧은 소리는 말로 보지 않는다."""
    noise = np.random.default_rng(1).normal(0, 50, 48000).astype(np.int16)
    assert run(UtteranceSegmenter(), noise) == []
    click = np.concatenate([noise[:16000], tone(1600, 8000), noise[:32000]])
    assert run(UtteranceSegmenter(), click) == []


def test_segmenter_splits_long_speech_at_max():
    """말이 max_utt_s 보다 길면 잘라서 낸다 (끝없이 모으지 않게)."""
    seg = UtteranceSegmenter(max_utt_s=2.0)
    outs = run(seg, tone(16000 * 5, 6000))
    assert len(outs) >= 2


# ---- WakeGate: 「아리스」 (쉬고) 「명령」 두 마디 (2026-11-11 시험에서 실제로 이렇게 말함)
from aris3_voice.listening import WakeGate  # noqa: E402


def test_wake_gate_one_breath():
    """한 번에 말하면 바로 명령."""
    g = WakeGate(follow_s=8.0)
    assert g.feed('아리스 노란색 큐브 잡아', 0.0) == (True, '노란색 큐브 잡아', 'command')


def test_wake_gate_pause_then_command():
    """「아리스」 만 -> 기다림, 8 초 안의 다음 말은 호출어 없어도 명령."""
    g = WakeGate(follow_s=8.0)
    assert g.feed('아리스', 0.0)[2] == 'wait'
    assert g.feed('노란색 큐브 잡아', 2.0) == (True, '노란색 큐브 잡아', 'follow')
    # 한 번만: 그 다음 말은 다시 호출어가 있어야 한다
    assert g.feed('빨간 큐브 잡아', 3.0)[0] is False


def test_wake_gate_pause_too_long():
    """8 초가 지나면 호출어 없는 말은 무시."""
    g = WakeGate(follow_s=8.0)
    g.feed('아리스', 0.0)
    assert g.feed('노란색 큐브 잡아', 9.0) == (False, '노란색 큐브 잡아', 'ignore')


def test_wake_gate_ignores_without_call():
    """부르지 않은 말은 무시 (옆 사람과의 대화)."""
    assert WakeGate().feed('노란색 큐브 잡아', 0.0)[0] is False
