"""음성 명령 모드 (2026-11-10): 계속 듣다가 「아리스」 로 시작하는 말만 명령으로 받는다.

ROS·Whisper 에 의존하지 않는 순수 함수·클래스라서 단위 시험(test/test_listening.py)에서 바로 쓴다.

  strip_wake_word(text) : 문장 앞의 호출어(아리스)를 찾아 떼어 낸다. 없으면 명령이 아니다.
  WakeGate              : 「아리스」 만 말하고 쉬었다가 명령해도 받는다 (부른 뒤 8 초 안의 다음 말 하나).
  UtteranceSegmenter    : 마이크 소리를 30 ms 조각으로 받아 「말 한 마디」(말소리 시작 ~ 조용해질 때까지)를 잘라 낸다.
                          방 소음 크기를 따라가며(적응형 문턱) 그보다 충분히 큰 소리만 말로 본다.
"""
import re

import numpy as np

# 호출어: 「아리스(ARIS)」. Whisper 가 자주 다르게 적는 형태도 받는다 (공백·문장부호를 지운 뒤 비교).
# 너무 넓히면 보통 말이 명령이 될 수 있으므로 앞부분이 「아리스」 와 소리가 아주 가까운 것만.
WAKE_WORDS = ['아리스', '알리스', '아리수', '아리쓰', '애리스', '에리스', '아리스야', '아리스여', 'aris', 'arisu', 'alice']
_PUNCT = re.compile(r'[\s,.!?~·…"\'「」()\-]+')


def strip_wake_word(text):
    """(호출어가 있었나, 호출어를 뗀 나머지 문장).

    호출어는 문장 맨 앞에 있어야 한다 (예: 「아리스, 빨간 도형 잡아」 -> (True, '빨간 도형 잡아')).
    맨 앞이 아니면 (False, 원문) - 다른 사람끼리 하는 말 속의 「아리스」 로 움직이지 않게.
    """
    if not text:
        return False, ''
    raw = text.strip()
    # 앞쪽 문장부호만 지우고 원문 나머지는 살린다
    lead = _PUNCT.match(raw)
    body = raw[lead.end():] if lead else raw
    low = body.lower()
    for w in sorted(WAKE_WORDS, key=len, reverse=True):     # 긴 것부터 (아리스야 > 아리스)
        if low.startswith(w):
            rest = body[len(w):]
            rest = _PUNCT.sub(' ', rest).strip()
            return True, rest
    return False, raw


class WakeGate:
    """호출어 문지기: 이 말을 명령으로 받을까.

    - 「아리스 빨간 도형 잡아」 처럼 한 번에 말하면 바로 명령.
    - 「아리스」 만 말하고 쉬었다가 「빨간 도형 잡아」 라고 하면, 말이 두 마디로 잘려도
      follow_s 초 안의 다음 말 하나를 명령으로 받는다 (사람은 부르고 잠깐 쉬는 게 자연스럽다, 2026-11-11 시험).
    - 그 밖에 호출어 없는 말은 무시.
    """

    def __init__(self, follow_s=8.0):
        self.follow_s = follow_s
        self.until = float('-inf')            # 이 시각(monotonic)까지는 호출어 없는 다음 말도 명령 (처음엔 없음)

    def feed(self, text, now):
        """(명령인가, 해석할 문장, 상태) - 상태는 'command' | 'follow' (뒤 말 명령) | 'wait' (호출어만) | 'ignore'."""
        woke, rest = strip_wake_word(text)
        if woke and rest:
            self.until = float('-inf')
            return True, rest, 'command'
        if woke:                              # 「아리스」 만: 다음 말을 기다린다
            self.until = now + self.follow_s
            return False, '', 'wait'
        if text and now <= self.until:        # 부른 뒤 이어서 한 말
            self.until = float('-inf')
            return True, text.strip(), 'follow'
        return False, text, 'ignore'


class UtteranceSegmenter:
    """마이크 소리 조각 -> 말 한 마디씩.

    rate: 표본 주파수, frame_ms: 조각 길이. feed(조각) 을 부르면 말이 끝났을 때 그 말 전체(int16 배열)를 돌려준다.
    말 시작 = 소리 크기(RMS)가 소음 바닥 x start_ratio 이상이고 최소 min_rms 이상인 조각이 start_frames 개 연속.
    말 끝   = 그 뒤 end_silence_s 동안 조용함. 너무 짧은 말(min_utt_s)은 버리고, max_utt_s 가 넘으면 잘라서 낸다.
    pre_roll_s 만큼 말 시작 전 소리도 붙인다 (첫 글자가 잘리지 않게).
    """

    def __init__(self, rate=16000, frame_ms=30, start_ratio=3.0, min_rms=300.0, start_frames=3,
                 end_silence_s=0.8, min_utt_s=0.4, max_utt_s=8.0, pre_roll_s=0.3):
        """설정값 (위 설명). 기본값은 노트북·USB 마이크 조용한 방 기준."""
        self.rate, self.frame = rate, int(rate * frame_ms / 1000)
        self.start_ratio, self.min_rms, self.start_frames = start_ratio, min_rms, start_frames
        self.end_frames = int(end_silence_s * 1000 / frame_ms)
        self.min_frames = int(min_utt_s * 1000 / frame_ms)
        self.max_frames = int(max_utt_s * 1000 / frame_ms)
        self.pre_frames = int(pre_roll_s * 1000 / frame_ms)
        self.noise = None                # 소음 바닥 RMS (말이 아닐 때만 따라감)
        self.reset()

    def reset(self):
        """듣기를 새로 시작 (모아 둔 소리 버림, 소음 바닥은 유지)."""
        self.pre = []                    # 말 시작 전 최근 조각들
        self.buf = []                    # 말 조각들
        self.loud = 0                    # 연속으로 큰 조각 수
        self.quiet = 0                   # 말하는 중 연속으로 조용한 조각 수
        self.speaking = False

    def threshold(self):
        """지금 말로 볼 최소 소리 크기."""
        return max(self.min_rms, (self.noise or 0.0) * self.start_ratio)

    def feed(self, frame):
        """int16 조각 하나. 말이 끝났으면 그 말(int16 배열), 아니면 None."""
        x = np.asarray(frame, dtype=np.float32)
        rms = float(np.sqrt(np.mean(x * x))) if x.size else 0.0
        loud = rms >= self.threshold()
        if not self.speaking:
            # 소음 바닥: 조용한 조각으로만 천천히 따라간다
            if not loud:
                self.noise = rms if self.noise is None else 0.95 * self.noise + 0.05 * rms
            self.pre.append(frame)
            del self.pre[:-self.pre_frames or None]
            self.loud = self.loud + 1 if loud else 0
            if self.loud >= self.start_frames:
                self.speaking = True
                self.buf = list(self.pre)
                self.quiet = 0
            return None
        self.buf.append(frame)
        self.quiet = 0 if loud else self.quiet + 1
        if self.quiet >= self.end_frames or len(self.buf) >= self.max_frames:
            speech = len(self.buf) - self.quiet
            out = np.concatenate(self.buf).astype(np.int16) if speech >= self.min_frames else None
            self.reset()
            return out
        return None
