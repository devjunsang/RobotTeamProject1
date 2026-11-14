"""ARIS3 음성 명령 노드: 마이크 -> Whisper(한국어 STT) -> 명령 해석 -> /aris3/voice_command.

  ros2 launch aris3_voice voice.launch.py          # Whisper 가 설치된 Python(가상환경)으로 실행

음성 명령 모드 (2026-11-10, GUI 의 ON/OFF 버튼):
  /aris3/voice/active  (std_msgs/Bool)   true = 계속 듣기 시작, false = 멈춤
     듣는 동안 말 한 마디(말소리 시작 ~ 조용해질 때)마다 인식하고, 「아리스(ARIS)」 로 시작하는 말만 명령으로 낸다.
     예: 「아리스 빨간 도형 잡아」 -> 호출어를 떼고 「빨간 도형 잡아」 를 해석. 호출어가 없으면 무시 (로그만).
  /aris3/voice/text    (std_msgs/String) 마이크 없이 문장을 바로 해석 (GUI 입력창·시험용). 직접 친 문장이라
     호출어가 없어도 받고, 있으면 떼고 해석한다.
  /aris3/voice/listen  (std_msgs/Bool)   (예전 방식) true 동안 녹음 -> 인식. 호출어 없이 받는다.
출력:
  /aris3/voice_command (aris3_interfaces/VoiceCommand)  인식 문장 + 해석 결과 + 실행 가능 여부
  /aris3/voice/state   (std_msgs/String)  loading | ready | listening | recording | recognizing | error:<이유>
     listening = 음성 명령 모드 켜짐 (말을 기다림)
목소리 (2026-11-11, speaker.py):
  /aris3/voice/say     (std_msgs/String)  이 문장을 ARIS 목소리(한국어 여성, edge-tts)로 말한다. 문장은 GUI 가 정한다
     (예: 「네, 노란색 큐브를 잡습니다.」). 파라미터 tts:=false 면 말하지 않음.

명령 해석 (2026-11-11): 「멈춰」 는 낱말 표(command_parser)로 바로. 나머지는 LLM(Claude Haiku 5.5, llm_parser)을
먼저 쓰고, 키가 없거나 늦거나 실패하면 낱말 표로. 파라미터 llm:=false 면 낱말 표만. 키는 ANTHROPIC_API_KEY.

이 노드는 로봇을 움직이지 않는다. 실행은 GUI 가 결과를 보여 주고 ExecuteTask 로 요청한다.
녹음은 ALSA arecord 를 쓴다 (sounddevice 불필요). Whisper 설정은 MARK1 과 같다
(faster-whisper, language=ko, beam_size=5, condition_on_previous_text=False) + 음성 구간 검출(vad_filter).
"""
import os
import queue
import signal
import subprocess
import tempfile
import threading
import time
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from aris3_interfaces.msg import VoiceCommand
from aris3_voice.command_parser import parse_command, validate_command
from aris3_voice.llm_parser import LlmParser
from aris3_voice.listening import UtteranceSegmenter, WakeGate, strip_wake_word
from aris3_voice.speaker import Speaker

RATE = 16000


def preload_cuda_libs():
    """pip 패키지(nvidia-cublas-cu12, nvidia-cudnn-cu12)의 .so 를 미리 올린다.

    CTranslate2 는 libcublas.so.12 등을 이름으로 찾는데, pip 설치 경로는 기본 검색 경로가 아니다.
    같은 이름의 라이브러리가 이미 올라가 있으면 그것을 쓰므로 LD_LIBRARY_PATH 를 바꾸지 않아도 된다. 없으면 조용히 넘어감.
    """
    import ctypes
    import glob
    import site
    for sp in site.getsitepackages() * 2:      # 두 번: cuDNN 하위 라이브러리끼리 서로 기대는 순서 때문
        for pat in ('nvidia/cublas/lib/libcublasLt.so.*', 'nvidia/cublas/lib/libcublas.so.*',
                    'nvidia/cudnn/lib/libcudnn*.so.*'):
            for lib in sorted(glob.glob(os.path.join(sp, pat))):
                try:
                    ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
                except OSError:
                    pass


class VoiceNode(Node):
    def __init__(self):
        super().__init__('aris3_voice')
        # Whisper (2026-11-11): GPU 가 되면 큰 모델(large-v3-turbo, 한국어가 훨씬 정확), 안 되면 CPU 의 작은 모델.
        # device=auto: cuda 로 먼저 해 보고 실패하면 cpu_model 로. 인식이 「혐의로(홈으로)」 처럼 틀리던 주원인이 base 모델.
        self.declare_parameter('model', 'large-v3-turbo')     # GPU 모델: tiny | base | small | medium | large-v3-turbo
        self.declare_parameter('cpu_model', 'base')           # GPU 가 안 될 때 (CPU 에서 빠른 크기)
        self.declare_parameter('device', 'auto')              # auto | cuda | cpu
        self.declare_parameter('compute_type', 'int8')        # CPU 일 때 (GPU 는 float16)
        # 명령에 쓰는 낱말을 Whisper 에 미리 알려 준다 (이 낱말 쪽으로 적게 함). 빈 문자열이면 끔
        self.declare_parameter('stt_prompt', '아리스, 홈으로, 집어, 잡아, 내려놔, 멈춰, 계속해, 가동, 연결 해제, '
                                             '빨간색, 파란색, 노란색, 초록색 큐브, 이거, 저기')
        self.declare_parameter('language', 'ko')
        self.declare_parameter('audio_device', 'default')     # arecord -D
        self.declare_parameter('max_record_s', 8.0)
        self.declare_parameter('min_record_s', 0.3)
        self.declare_parameter('known_colors', ['red', 'blue', 'yellow'])
        # 음성 명령 모드: 말 한 마디 자르기 설정 (listening.UtteranceSegmenter)
        self.declare_parameter('vad_min_rms', 300.0)          # 이보다 작은 소리는 말이 아님 (int16)
        self.declare_parameter('vad_start_ratio', 3.0)        # 방 소음의 몇 배 이상을 말로 볼지
        self.declare_parameter('vad_end_silence_s', 0.8)      # 이만큼 조용하면 말이 끝난 것
        # LLM 해석 (llm_parser): 실패하면 낱말 표. 키는 ANTHROPIC_API_KEY 환경 변수
        self.declare_parameter('llm', True)
        self.declare_parameter('llm_model', 'claude-haiku-5-5')
        self.declare_parameter('llm_timeout_s', 4.0)
        # 목소리 (speaker): GUI 가 /aris3/voice/say 로 보낸 문장을 말한다
        self.declare_parameter('tts', True)
        self.declare_parameter('tts_voice', 'de-DE-SeraphinaMultilingualNeural')   # 다국어 여성 (세라피나, 2026-11-11 사용자 선택)
        self.declare_parameter('tts_rate', '-5%')                  # 조금 천천히 (차분하게)
        self.declare_parameter('tts_output', 'internal')           # 노트북 내장 스피커 (블루투스 말고)
        self.known = list(self.get_parameter('known_colors').value)
        self.speaker = None
        if self.get_parameter('tts').value:
            self.speaker = Speaker(self.get_parameter('tts_voice').value, self.get_parameter('tts_rate').value,
                                   log=lambda s: self.get_logger().info(s),
                                   output=self.get_parameter('tts_output').value)
            self.get_logger().info(f'목소리: {self.speaker.voice}' if self.speaker.enabled
                                   else f'목소리 끔: {self.speaker.why_off}')
        self.llm = None
        if self.get_parameter('llm').value:
            self.llm = LlmParser(self.get_parameter('llm_model').value,
                                 float(self.get_parameter('llm_timeout_s').value),
                                 log=lambda s: self.get_logger().info(s))
            self.get_logger().info(f'명령 해석: LLM {self.llm.model} (실패하면 낱말 표)' if self.llm.enabled
                                   else f'명령 해석: 낱말 표 ({self.llm.why_off})')

        self.pub_cmd = self.create_publisher(VoiceCommand, '/aris3/voice_command', 10)
        self.pub_state = self.create_publisher(String, '/aris3/voice/state', 10)
        self.create_subscription(Bool, '/aris3/voice/listen', self._on_listen, 10)
        self.create_subscription(String, '/aris3/voice/text', self._on_text, 10)
        self.create_subscription(Bool, '/aris3/voice/active', self._on_active, 10)
        self.create_subscription(String, '/aris3/voice/say', self._on_say, 10)

        self.lock = threading.Lock()
        self.model = None
        self.state = 'loading'
        self.rec = None             # (Popen, wav path, 시작 시각)
        self.listen_proc = None     # 음성 명령 모드: 계속 녹음하는 arecord
        self.utterances = queue.Queue(maxsize=4)   # 잘라 낸 말 -> 인식 스레드
        self.wake = WakeGate(follow_s=8.0)          # 호출어 문지기 (「아리스」 만 말하고 쉬어도 다음 말을 받음)
        threading.Thread(target=self._recognize_loop, daemon=True).start()
        self.create_timer(1.0, lambda: self._set_state(self.state))   # 늦게 붙은 GUI 도 상태를 알 수 있게
        threading.Thread(target=self._load_model, daemon=True).start()

    # ------------------------------------------------------------------ 상태
    def _set_state(self, s):
        # 음성 명령 모드 중에는 「대기」 대신 「듣는 중」 으로 보고한다
        if s == 'ready' and self.listen_proc is not None:
            s = 'listening'
        self.state = s
        self.pub_state.publish(String(data=s))

    def _load_model(self):
        """Whisper 모델 로딩 (별도 스레드). device=auto 면 GPU 먼저, 안 되면 CPU 작은 모델."""
        preload_cuda_libs()                       # pip 으로 설치한 cuBLAS·cuDNN 을 먼저 올린다 (LD_LIBRARY_PATH 없이)
        from faster_whisper import WhisperModel   # 무거운 import 는 시작 뒤 별도 스레드에서
        device = self.get_parameter('device').value
        tries = []
        if device in ('auto', 'cuda'):
            tries.append((self.get_parameter('model').value, 'cuda', 'float16'))
        if device in ('auto', 'cpu'):
            tries.append((self.get_parameter('cpu_model').value if device == 'auto' else self.get_parameter('model').value,
                          'cpu', self.get_parameter('compute_type').value))
        for name, dev, ctype in tries:
            t0 = time.monotonic()
            try:
                model = WhisperModel(name, device=dev, compute_type=ctype)
                # GPU 는 실제로 한 번 돌려 봐야 라이브러리 문제가 드러난다 (로딩은 되고 첫 인식에서 실패)
                list(model.transcribe(np.zeros(RATE // 2, dtype=np.float32), language='ko')[0])
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'Whisper {name} ({dev}) 실패: {e}')
                continue
            self.model = model
            self.get_logger().info(f'Whisper {name} ({dev}, {ctype}) 준비 ({time.monotonic() - t0:.1f} s)')
            self._set_state('ready')
            return
        self.get_logger().error('Whisper 모델 로딩 실패 (위 기록 확인)')
        self._set_state('error:모델 로딩 실패')

    def _transcribe(self, audio):
        """float32 소리 -> 글자. 설정은 MARK1 과 같고, 명령 낱말을 미리 알려 준다 (stt_prompt)."""
        # vad_filter: 말소리가 없는 구간(방 소음)을 잘라 낸다. 끄면 소음에서 없는 문장을 지어냈다
        # (2026-11-08: 말하지 않은 2 초 -> 「오, 가슴, 여유, ...」)
        prompt = self.get_parameter('stt_prompt').value or None
        segs, _ = self.model.transcribe(audio, language=self.get_parameter('language').value, beam_size=5,
                                        condition_on_previous_text=False, vad_filter=True, initial_prompt=prompt)
        return ' '.join(s.text.strip() for s in segs).strip()

    # ------------------------------------------------------------------ 녹음
    def _on_listen(self, msg):
        with self.lock:
            if msg.data:
                self._start_recording()
            else:
                self._stop_recording()

    def _start_recording(self):
        if self.rec is not None or self.state != 'ready' or self.listen_proc is not None:
            return
        fd, path = tempfile.mkstemp(prefix='aris3_voice_', suffix='.wav')
        os.close(fd)
        max_s = int(self.get_parameter('max_record_s').value)
        proc = subprocess.Popen(
            ['arecord', '-q', '-D', self.get_parameter('audio_device').value, '-f', 'S16_LE',
             '-r', str(RATE), '-c', '1', '-d', str(max_s), path],
            stderr=subprocess.PIPE)
        self.rec = (proc, path, time.monotonic())
        self._set_state('recording')

    def _stop_recording(self):
        if self.rec is None:
            return
        proc, path, t0 = self.rec
        self.rec = None
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)     # arecord 는 SIGINT 를 받으면 wav 헤더를 마무리하고 끝난다
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        if time.monotonic() - t0 < float(self.get_parameter('min_record_s').value):
            os.remove(path)
            self._set_state('ready')
            self._publish('', '', 0.0, note='녹음이 너무 짧습니다')
            return
        self._set_state('recognizing')
        threading.Thread(target=self._recognize, args=(path,), daemon=True).start()

    def _recognize(self, path):
        try:
            with wave.open(path) as w:
                pcm = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
            audio = pcm.astype(np.float32) / 32768.0
            t0 = time.monotonic()
            text = self._transcribe(audio)
            dt = time.monotonic() - t0
            self._publish(text, text, dt)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'음성 인식 실패: {e}')
            self._publish('', '', 0.0, note=f'음성 인식 실패: {e}')
        finally:
            os.remove(path)
            self._set_state('ready')

    # ------------------------------------------------------------------ 음성 명령 모드 (계속 듣기 + 호출어)
    def _on_active(self, msg):
        """음성 명령 모드 켜기/끄기 (GUI ON/OFF 버튼)."""
        with self.lock:
            if msg.data:
                self._start_listening()
            else:
                self._stop_listening()

    def _start_listening(self):
        """arecord 를 raw 로 계속 돌리고 읽기 스레드를 시작한다. 모델 준비 전이면 거부."""
        if self.listen_proc is not None:
            return
        if self.model is None:
            self.get_logger().warn('음성 명령 모드: Whisper 모델이 아직 준비되지 않음')
            return
        proc = subprocess.Popen(
            ['arecord', '-q', '-D', self.get_parameter('audio_device').value, '-t', 'raw', '-f', 'S16_LE',
             '-r', str(RATE), '-c', '1'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.listen_proc = proc
        threading.Thread(target=self._read_loop, args=(proc,), daemon=True).start()
        self.get_logger().info('음성 명령 모드 켬 - 「아리스 ...」 로 말하세요')
        self._set_state('listening')

    def _stop_listening(self):
        """듣기 멈춤 (arecord 종료 -> 읽기 스레드도 끝남)."""
        proc, self.listen_proc = self.listen_proc, None
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        self.get_logger().info('음성 명령 모드 끔')
        if self.state == 'listening':
            self._set_state('ready')

    def _read_loop(self, proc):
        """작업 스레드: 마이크 30 ms 조각을 읽어 말 한 마디씩 잘라 인식 대기열에 넣는다."""
        seg = UtteranceSegmenter(
            rate=RATE, min_rms=float(self.get_parameter('vad_min_rms').value),
            start_ratio=float(self.get_parameter('vad_start_ratio').value),
            end_silence_s=float(self.get_parameter('vad_end_silence_s').value),
            max_utt_s=float(self.get_parameter('max_record_s').value))
        nbytes = seg.frame * 2
        while self.listen_proc is proc:
            data = proc.stdout.read(nbytes)
            if not data or len(data) < nbytes:
                break                                      # arecord 끝남 (끄기 또는 장치 오류)
            utt = seg.feed(np.frombuffer(data, dtype=np.int16))
            if utt is None:
                continue
            try:
                t_end = time.monotonic()
                self.utterances.put_nowait((utt, t_end - len(utt) / RATE, t_end))   # (소리, 시작, 끝 시각)
            except queue.Full:
                self.get_logger().warn('인식이 밀려 말 한 마디를 버림')
        if self.listen_proc is proc:                       # 끄지 않았는데 끝남 = 마이크 문제
            self.listen_proc = None
            self.get_logger().error('음성 명령 모드: 마이크 녹음이 끝남 (장치 확인) - 모드를 껐습니다')
            self._set_state('ready')

    def _recognize_loop(self):
        """인식 스레드: 잘라 낸 말을 Whisper 로 글자로 바꾸고, 「아리스」 로 시작하는 말만 명령으로 낸다."""
        while True:
            utt, t_start, t_end = self.utterances.get()
            # 이 말을 녹음하는 동안 ARIS 가 말하고 있었는가 (마이크가 ARIS 목소리를 들었을 수 있음)
            echo = self.speaker is not None and self.speaker.overlaps(t_start, t_end)
            if self.model is None:
                continue
            self._set_state('recognizing')
            try:
                t0 = time.monotonic()
                text = self._transcribe(utt.astype(np.float32) / 32768.0)
                dt = time.monotonic() - t0
                # 「아리스 ...」 한 번에, 또는 「아리스」 (쉬고) 「...」 두 마디로 (WakeGate)
                is_cmd, rest, how = self.wake.feed(text, time.monotonic(), echo=echo)
                if how == 'follow' and echo:
                    # 「아리스」 뒤에 들린 말이 ARIS 자신의 대답(「네, 말씀하세요.」)일 수 있다 -> 버리고 다시 기다린다.
                    # 호출어가 든 말(「아리스 멈춰」)은 ARIS 가 말하는 중에도 그대로 받는다.
                    self.wake.until = time.monotonic() + self.wake.follow_s
                    self.get_logger().info(f'「{text}」 - ARIS 가 말하는 중에 들린 말이라 무시, 명령을 계속 기다림')
                    continue
                if is_cmd:
                    self._publish(text if how == 'command' else f'아리스 … {text}', rest, dt)
                elif how == 'wait':
                    if self.speaker is not None:
                        self.speaker.say('네, 말씀하세요.')        # 부르기만 했을 때도 대답한다
                    self.get_logger().info(f'「{text}」 - 호출어만 들림: {self.wake.follow_s:.0f} 초 안의 다음 말을 명령으로 받음')
                elif text:
                    self.get_logger().info(f'「{text}」 - 호출어(아리스) 없음, 무시 ({dt:.2f}s)')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'음성 인식 실패: {e}')
            finally:
                self._set_state('ready')                   # 듣는 중이면 listening 으로 바뀜

    # ------------------------------------------------------------------ 목소리
    def _on_say(self, msg):
        """GUI 가 정한 대답을 말한다 (작업 스레드에서 - 이 콜백은 바로 돌아온다)."""
        if self.speaker is not None:
            self.speaker.say(msg.data)

    # ------------------------------------------------------------------ 해석·발행
    def _on_text(self, msg):
        # 직접 친 문장: 호출어가 없어도 받고, 있으면 떼고 해석
        _woke, rest = strip_wake_word(msg.data)
        self._publish(msg.data, rest, 0.0)

    def _interpret(self, to_parse):
        """(명령 dict, 해석기 설명). 「멈춰」 는 낱말 표로 바로, 나머지는 LLM 먼저 -> 실패하면 낱말 표."""
        rule = parse_command(to_parse)
        if rule['action'] == 'stop' or self.llm is None or not self.llm.enabled:
            return rule, '낱말 표'
        llm = self.llm.parse(to_parse)
        if llm is None:
            return rule, '낱말 표 (LLM 실패)'
        keys = ('action', 'color', 'target_color', 'number', 'target_number')
        if tuple(llm[k] for k in keys) != tuple(rule[k] for k in keys):
            # 둘이 다르게 읽은 문장은 로그에 남긴다 (낱말 표를 보완하거나 LLM 을 점검할 때 참고)
            self.get_logger().info(f'해석 차이 「{to_parse}」: LLM {llm["action"]}/{llm["color"] or "-"}/{llm["number"]}번 · '
                                   f'낱말 표 {rule["action"] or "-"}/{rule["color"] or "-"}/{rule["number"]}번')
        return llm, f'LLM {self.llm.last_latency:.1f}s'

    def _publish(self, text, to_parse, stt_time, note=''):
        cmd, how = self._interpret(to_parse) if text else (parse_command(to_parse), '')
        valid, why = validate_command(cmd, self.known) if text else (False, note or '음성이 인식되지 않았습니다')
        why = f'{why} [{how}]' if how else why
        out = VoiceCommand(text=text, normalized_text=cmd['normalized_text'], action=cmd['action'],
                           color=cmd['color'], shape=cmd['shape'], valid=valid, message=why,
                           refers_selected_object=bool(cmd['ref_object']),
                           refers_selected_place=bool(cmd['ref_place']),
                           target_color=cmd.get('target_color', ''),
                           number=int(cmd.get('number', 0)), target_number=int(cmd.get('target_number', 0)),
                           stt_time_s=float(stt_time))
        out.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(out)
        self.get_logger().info(f'「{text}」 -> action={cmd["action"]} color={cmd["color"]} '
                               f'number={cmd.get("number", 0)} '
                               f'target={cmd.get("target_color", "") or "-"} target_number={cmd.get("target_number", 0)} '
                               f'shape={cmd["shape"]} valid={valid} ({why}) {stt_time:.2f}s')


def main():
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._stop_listening()          # 마이크(arecord)를 남겨 두지 않게
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
