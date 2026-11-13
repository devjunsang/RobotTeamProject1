"""ARIS 목소리(speaker) 시험 (2026-11-11). 네트워크·스피커 없이: 가짜 합성 함수와 가짜 재생 명령(true)을 쓴다."""
import time

from aris3_voice.speaker import Speaker


def wait_idle(sp, timeout=3.0):
    """대기열이 비고 말하기가 끝날 때까지 기다린다."""
    end = time.monotonic() + timeout
    while (not sp.queue.empty() or sp.speaking) and time.monotonic() < end:
        time.sleep(0.02)


def test_speaks_and_caches_same_sentence():
    """처음엔 합성, 같은 문장 두 번째는 캐시 파일을 그대로 재생 (합성하지 않음)."""
    made, logs = [], []

    def synth(text, path):
        """가짜 합성: 파일에 글자를 써 둔다."""
        made.append(text)
        with open(path, 'w') as f:
            f.write(text)
    sp = Speaker(voice=f'test-{time.time()}', log=logs.append, player=('true',), synth=synth)
    assert sp.say('네, 노란색 큐브를 잡습니다.')
    wait_idle(sp)
    assert sp.say('네, 노란색 큐브를 잡습니다.')
    wait_idle(sp)
    assert made == ['네, 노란색 큐브를 잡습니다.']
    assert sum('말함' in s for s in logs) == 2


def test_synth_failure_is_logged_not_raised():
    """합성 실패(인터넷 끊김)는 로그만 남기고 넘어간다 - 다음 문장은 계속 말한다."""
    logs = []

    def synth(text, path):
        """가짜 합성: 「실패」 가 든 문장은 예외."""
        if '실패' in text:
            raise OSError('network down')
        with open(path, 'w') as f:
            f.write(text)
    sp = Speaker(voice=f'test-{time.time()}', log=logs.append, player=('true',), synth=synth)
    sp.say('실패 문장')
    wait_idle(sp)
    sp.say('정상 문장')
    wait_idle(sp)
    assert any('말하기 실패' in s for s in logs)
    assert any('말함: 「정상 문장」' in s for s in logs)


def test_empty_text_is_ignored():
    """빈 문장은 넣지 않는다."""
    sp = Speaker(log=lambda s: None, player=('true',), synth=lambda t, p: None)
    assert not sp.say('   ')


def test_internal_speaker_is_chosen_over_bluetooth():
    """기본 출력이 블루투스여도 내장 스피커(alsa_output.pci)를 고른다. 없으면 기본 출력(인자 없음)."""
    sinks = [('bluez_output.6C_D3.1', 'Redmi Buds 3 Pro'), ('alsa_output.pci-0000_80_1f.3.analog-stereo', 'HDA Intel PCH')]
    sp = Speaker(log=lambda s: None, player=('true',), synth=lambda t, p: None, sinks=lambda: sinks)
    assert sp._target_args() == ['--target=alsa_output.pci-0000_80_1f.3.analog-stereo']
    sp.sinks = lambda: sinks[:1]
    assert sp._target_args() == []
    sp.output = ''
    assert sp._target_args() == []
