"""ARIS 목소리: 문장 -> 한국어 여성 음성으로 말하기 (2026-11-11).

  엔진: edge-tts (Microsoft 신경망 음성, 무료·키 없음·인터넷 필요), 기본 음성 de-DE-SeraphinaMultilingualNeural (세라피나 - 다국어 음성이 한국어로 말함, 사용자 선택).
  재생: PipeWire pw-play (mp3 를 그대로 재생).
  출력 장치 (2026-11-11 사용자 결정): 기본은 노트북 내장 스피커 (output='internal').
    시스템 기본 출력이 블루투스 이어폰·스피커여도 ARIS 목소리는 내장 스피커로만 낸다.
    내장 스피커 = PipeWire 출력(Audio/Sink) 중 이름이 alsa_output.pci 로 시작하는 것 (USB·블루투스·HDMI 제외).
    output='' 이면 시스템 기본 출력, 그 밖의 문자열이면 그 node.name 으로.

  - 말할 문장은 GUI 가 정한다 (/aris3/voice/say). GUI 만 명령을 실제로 실행했는지 알기 때문
    (예: 조작 허용이 꺼져 있으면 「네, 잡습니다」 대신 「조작 허용을 먼저 켜 주세요」).
  - 말하기는 작업 스레드 하나가 차례로 한다. 밀린 문장이 많으면 오래된 것을 버린다 (늦은 대답은 헷갈린다).
  - 같은 문장은 한 번 만든 소리 파일을 다시 쓴다 (두 번째부터 인터넷 없이 바로).
  - 실패(인터넷 끊김·재생 장치 없음)하면 로그만 남기고 조용히 넘어간다. 로봇 동작에는 영향이 없다.
  - 말하는 동안 마이크가 이 소리를 들어도, 문장에 「아리스」 가 없으므로 호출어 문지기가 명령으로 받지 않는다.
    그래서 말하는 중에도 「아리스 멈춰」 는 그대로 들린다 (마이크를 막지 않음).
"""
import asyncio
import hashlib
import json
import os
import queue
import subprocess
import tempfile
import threading
import time

try:                                    # 음성 가상환경(~/venvs/mark1_voice)에 설치. 없으면 꺼진 채로
    import edge_tts
except ImportError:                     # pragma: no cover - 설치 환경에 따라
    edge_tts = None

CACHE_DIR = os.path.join(tempfile.gettempdir(), 'aris3_tts_cache')


def list_sinks():
    """PipeWire 출력 장치 목록 [(node.name, 설명)] (pw-dump). 못 읽으면 빈 목록."""
    try:
        r = subprocess.run(['pw-dump'], capture_output=True, timeout=5)
        objs = json.loads(r.stdout or b'[]')
    except Exception:  # noqa: BLE001 - PipeWire 가 없거나 응답이 이상하면 목록 없음
        return []
    out = []
    for o in objs:
        props = (o.get('info') or {}).get('props') or {}
        if props.get('media.class') == 'Audio/Sink' and props.get('node.name'):
            out.append((props['node.name'], props.get('node.description', '')))
    return out


def pick_internal(sinks):
    """출력 목록에서 노트북 내장 스피커 (alsa_output.pci...). 없으면 None."""
    for name, desc in sinks:
        if name.startswith('alsa_output.pci'):
            return name, desc
    return None


class Speaker:
    """문장을 차례로 말한다. say() 는 바로 돌아온다 (말하기는 작업 스레드에서)."""

    def __init__(self, voice='de-DE-SeraphinaMultilingualNeural', rate='-5%', pitch='+0Hz', log=print, max_pending=2,
                 player=('pw-play',), synth=None, output='internal', sinks=list_sinks):
        """voice: edge-tts 음성 이름, rate/pitch: 빠르기·높이 (예 '-5%', '+0Hz'), log: 문자열 로그 함수,
        max_pending: 기다리는 문장 최대 수 (넘으면 오래된 것을 버림), player: 재생 명령 (파일 경로를 뒤에 붙임),
        synth: (문장, 파일 경로) -> None 합성 함수 (시험용, 없으면 edge-tts),
        output: 'internal'(노트북 내장 스피커) | ''(시스템 기본) | PipeWire node.name, sinks: 출력 목록 함수 (시험용)."""
        self.voice, self.rate, self.pitch, self.log = voice, rate, pitch, log
        self.player = list(player)
        self.output, self.sinks = output, sinks
        self.synth = synth or self._edge_synth
        self.enabled = synth is not None or edge_tts is not None
        self.why_off = '' if self.enabled else 'edge-tts 패키지 없음'
        self.queue = queue.Queue(maxsize=max_pending)
        self.speaking = False
        self.last_end = float('-inf')       # 마지막으로 말하기가 끝난 시각 (monotonic) - 마이크가 들은 자기 목소리 거르기용
        self.last_error = ''
        os.makedirs(CACHE_DIR, exist_ok=True)
        if self.enabled:
            threading.Thread(target=self._loop, daemon=True).start()

    # ------------------------------------------------------------------ 바깥에서 부르는 것
    def say(self, text):
        """문장을 말하기 대기열에 넣는다. 꽉 차 있으면 가장 오래된 문장을 버린다. 넣었으면 True."""
        text = (text or '').strip()
        if not self.enabled or not text:
            return False
        while True:
            try:
                self.queue.put_nowait(text)
                return True
            except queue.Full:
                try:
                    dropped = self.queue.get_nowait()
                    self.log(f'말하기 밀림 - 버림: 「{dropped}」')
                except queue.Empty:
                    pass

    def overlaps(self, t_start, t_end, margin=0.5):
        """[t_start, t_end] (monotonic) 동안 말하고 있었는가 (끝난 뒤 margin 초의 울림까지). 마이크 소리 거르기용."""
        return self.speaking or self.last_end + margin >= t_start

    # ------------------------------------------------------------------ 안쪽
    def cache_path(self, text):
        """문장·음성·빠르기·높이로 정한 소리 파일 경로 (같은 문장은 같은 파일)."""
        key = hashlib.sha1(f'{self.voice}|{self.rate}|{self.pitch}|{text}'.encode()).hexdigest()[:16]
        return os.path.join(CACHE_DIR, f'{key}.mp3')

    def _edge_synth(self, text, path):
        """edge-tts 로 mp3 파일을 만든다 (인터넷 필요). 실패하면 예외."""
        async def run():
            """edge-tts 는 비동기 API 라 이 스레드 안에서 이벤트 루프를 돌린다."""
            await edge_tts.Communicate(text, self.voice, rate=self.rate, pitch=self.pitch).save(path)
        asyncio.run(asyncio.wait_for(run(), timeout=8.0))

    def _speak_one(self, text):
        """한 문장: 소리 파일이 없으면 만들고 재생한다. 성공하면 True."""
        path = self.cache_path(text)
        t0 = time.monotonic()
        if not (os.path.exists(path) and os.path.getsize(path) > 0):
            tmp = path + '.part'
            try:
                self.synth(text, tmp)
                os.replace(tmp, path)             # 다 만든 파일만 캐시에 (중간에 끊긴 파일을 재생하지 않게)
            except Exception as e:  # noqa: BLE001 - 인터넷·서비스 오류 모두 로그만
                self.last_error = f'음성 합성 실패 (인터넷 확인): {e!r}'
                self.log(f'말하기 실패 「{text}」: {self.last_error}')
                if os.path.exists(tmp):
                    os.remove(tmp)
                return False
        synth_s = time.monotonic() - t0
        try:
            r = subprocess.run(self.player + self._target_args() + [path], timeout=30, capture_output=True)
            if r.returncode != 0:
                raise RuntimeError(r.stderr.decode(errors='replace').strip() or f'종료 코드 {r.returncode}')
        except Exception as e:  # noqa: BLE001 - 재생 장치 문제는 로그만
            self.last_error = f'재생 실패: {e}'
            self.log(f'말하기 실패 「{text}」: {self.last_error}')
            return False
        self.log(f'말함: 「{text}」 (합성 {synth_s:.1f}s)')
        return True

    def _target_args(self):
        """재생 명령에 붙일 출력 장치 인자. 장치 번호는 다시 꽂으면 바뀌므로 말할 때마다 이름으로 찾는다."""
        if not self.output:
            return []
        if self.output != 'internal':
            return [f'--target={self.output}']
        found = pick_internal(self.sinks())
        if found is None:                                  # 내장 스피커를 못 찾으면 기본 출력으로 (로그만)
            self.log('내장 스피커를 찾지 못함 - 시스템 기본 출력으로 말함')
            return []
        return [f'--target={found[0]}']

    def _loop(self):
        """작업 스레드: 대기열의 문장을 하나씩 말한다."""
        while True:
            text = self.queue.get()
            self.speaking = True
            try:
                self._speak_one(text)
            finally:
                self.speaking = False
                self.last_end = time.monotonic()
