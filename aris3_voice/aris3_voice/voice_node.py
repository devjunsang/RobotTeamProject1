"""ARIS3 음성 명령 노드: 마이크 -> Whisper(한국어 STT) -> 명령 해석 -> /aris3/voice_command.

  ros2 launch aris3_voice voice.launch.py          # Whisper 가 설치된 Python(가상환경)으로 실행

누르는 동안 녹음 (push-to-talk):
  /aris3/voice/listen  (std_msgs/Bool)   true = 녹음 시작, false = 녹음 끝 -> 인식
  /aris3/voice/text    (std_msgs/String) 마이크 없이 문장을 바로 해석 (GUI 입력창·시험용)
출력:
  /aris3/voice_command (aris3_interfaces/VoiceCommand)  인식 문장 + 해석 결과 + 실행 가능 여부
  /aris3/voice/state   (std_msgs/String)  loading | ready | recording | recognizing | error:<이유>

이 노드는 로봇을 움직이지 않는다. 실행은 GUI 가 결과를 보여 주고 ExecuteTask 로 요청한다.
녹음은 ALSA arecord 를 쓴다 (sounddevice 불필요). Whisper 설정은 MARK1 과 같다
(faster-whisper, language=ko, beam_size=5, condition_on_previous_text=False) + 음성 구간 검출(vad_filter).
"""
import os
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
        self.known = list(self.get_parameter('known_colors').value)

        self.pub_cmd = self.create_publisher(VoiceCommand, '/aris3/voice_command', 10)
        self.pub_state = self.create_publisher(String, '/aris3/voice/state', 10)
        self.create_subscription(Bool, '/aris3/voice/listen', self._on_listen, 10)
        self.create_subscription(String, '/aris3/voice/text', self._on_text, 10)

        self.lock = threading.Lock()
        self.model = None
        self.state = 'loading'
        self.rec = None             # (Popen, wav path, 시작 시각)
        self.create_timer(1.0, lambda: self._set_state(self.state))   # 늦게 붙은 GUI 도 상태를 알 수 있게
        threading.Thread(target=self._load_model, daemon=True).start()

    # ------------------------------------------------------------------ 상태
    def _set_state(self, s):
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
        if self.rec is not None or self.state != 'ready':
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

    # ------------------------------------------------------------------ 해석·발행
    def _on_text(self, msg):
        self._publish(msg.data, msg.data, 0.0)

    def _publish(self, text, to_parse, stt_time, note=''):
        cmd = parse_command(to_parse)
        valid, why = validate_command(cmd, self.known) if text else (False, note or '음성이 인식되지 않았습니다')
        out = VoiceCommand(text=text, normalized_text=cmd['normalized_text'], action=cmd['action'],
                           color=cmd['color'], shape=cmd['shape'], valid=valid, message=why,
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
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
