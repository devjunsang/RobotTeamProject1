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
from aris3_voice.listening import UtteranceSegmenter, WakeGate, strip_wake_word

RATE = 16000


class VoiceNode(Node):
    def __init__(self):
        super().__init__('aris3_voice')
        self.declare_parameter('model', 'base')               # tiny | base | small | medium
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('compute_type', 'int8')
        self.declare_parameter('language', 'ko')
        self.declare_parameter('audio_device', 'default')     # arecord -D
        self.declare_parameter('max_record_s', 8.0)
        self.declare_parameter('min_record_s', 0.3)
        self.declare_parameter('known_colors', ['red', 'blue', 'yellow'])
        # 음성 명령 모드: 말 한 마디 자르기 설정 (listening.UtteranceSegmenter)
        self.declare_parameter('vad_min_rms', 300.0)          # 이보다 작은 소리는 말이 아님 (int16)
        self.declare_parameter('vad_start_ratio', 3.0)        # 방 소음의 몇 배 이상을 말로 볼지
        self.declare_parameter('vad_end_silence_s', 0.8)      # 이만큼 조용하면 말이 끝난 것
        self.known = list(self.get_parameter('known_colors').value)

        self.pub_cmd = self.create_publisher(VoiceCommand, '/aris3/voice_command', 10)
        self.pub_state = self.create_publisher(String, '/aris3/voice/state', 10)
        self.create_subscription(Bool, '/aris3/voice/listen', self._on_listen, 10)
        self.create_subscription(String, '/aris3/voice/text', self._on_text, 10)
        self.create_subscription(Bool, '/aris3/voice/active', self._on_active, 10)

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
        from faster_whisper import WhisperModel   # 무거운 import 는 시작 뒤 별도 스레드에서
        name = self.get_parameter('model').value
        t0 = time.monotonic()
        try:
            self.model = WhisperModel(name, device=self.get_parameter('device').value,
                                      compute_type=self.get_parameter('compute_type').value)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Whisper 모델 로딩 실패: {e}')
            self._set_state(f'error:모델 로딩 실패 ({e})')
            return
        self.get_logger().info(f'Whisper {name} 준비 ({time.monotonic() - t0:.1f} s)')
        self._set_state('ready')

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
            # vad_filter: 말소리가 없는 구간(방 소음)을 잘라 낸다. 끄면 소음에서 없는 문장을 지어냈다
            # (2026-11-08: 말하지 않은 2 초 -> 「오, 가슴, 여유, ...」)
            segs, _ = self.model.transcribe(audio, language=self.get_parameter('language').value,
                                            beam_size=5, condition_on_previous_text=False,
                                            vad_filter=True)
            text = ' '.join(s.text.strip() for s in segs).strip()
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
                self.utterances.put_nowait(utt)
            except queue.Full:
                self.get_logger().warn('인식이 밀려 말 한 마디를 버림')
        if self.listen_proc is proc:                       # 끄지 않았는데 끝남 = 마이크 문제
            self.listen_proc = None
            self.get_logger().error('음성 명령 모드: 마이크 녹음이 끝남 (장치 확인) - 모드를 껐습니다')
            self._set_state('ready')

    def _recognize_loop(self):
        """인식 스레드: 잘라 낸 말을 Whisper 로 글자로 바꾸고, 「아리스」 로 시작하는 말만 명령으로 낸다."""
        while True:
            utt = self.utterances.get()
            if self.model is None:
                continue
            self._set_state('recognizing')
            try:
                t0 = time.monotonic()
                segs, _ = self.model.transcribe(utt.astype(np.float32) / 32768.0,
                                                language=self.get_parameter('language').value, beam_size=5,
                                                condition_on_previous_text=False, vad_filter=True)
                text = ' '.join(x.text.strip() for x in segs).strip()
                dt = time.monotonic() - t0
                # 「아리스 ...」 한 번에, 또는 「아리스」 (쉬고) 「...」 두 마디로 (WakeGate)
                is_cmd, rest, how = self.wake.feed(text, time.monotonic())
                if is_cmd:
                    self._publish(text if how == 'command' else f'아리스 … {text}', rest, dt)
                elif how == 'wait':
                    self.get_logger().info(f'「{text}」 - 호출어만 들림: {self.wake.follow_s:.0f} 초 안의 다음 말을 명령으로 받음')
                elif text:
                    self.get_logger().info(f'「{text}」 - 호출어(아리스) 없음, 무시 ({dt:.2f}s)')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'음성 인식 실패: {e}')
            finally:
                self._set_state('ready')                   # 듣는 중이면 listening 으로 바뀜

    # ------------------------------------------------------------------ 해석·발행
    def _on_text(self, msg):
        # 직접 친 문장: 호출어가 없어도 받고, 있으면 떼고 해석
        _woke, rest = strip_wake_word(msg.data)
        self._publish(msg.data, rest, 0.0)

    def _publish(self, text, to_parse, stt_time, note=''):
        cmd = parse_command(to_parse)
        valid, why = validate_command(cmd, self.known) if text else (False, note or '음성이 인식되지 않았습니다')
        out = VoiceCommand(text=text, normalized_text=cmd['normalized_text'], action=cmd['action'],
                           color=cmd['color'], shape=cmd['shape'], valid=valid, message=why,
                           refers_selected_object=bool(cmd['ref_object']),
                           refers_selected_place=bool(cmd['ref_place']),
                           stt_time_s=float(stt_time))
        out.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(out)
        self.get_logger().info(f'「{text}」 -> action={cmd["action"]} color={cmd["color"]} '
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
