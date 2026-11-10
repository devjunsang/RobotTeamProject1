"""ARIS3 손짓 노드: 위쪽 카메라 영상 -> MediaPipe Hand Landmarker -> 검지 끝 픽셀 + 손짓 -> /aris3/gesture.

  ros2 launch aris3_gesture gesture.launch.py      # MediaPipe 가 설치된 Python(가상환경)으로 실행

입력:  /aris3/overhead_camera/image/compressed (sensor_msgs/CompressedImage)  - 물체 색 검출과 같은 영상
출력:  /aris3/gesture      (aris3_interfaces/HandGesture)  손이 보이든 안 보이든 처리한 영상마다
                                                             (hold_progress: 가리키고 멈추기 진행 0~1)
       /aris3/gesture/event (aris3_interfaces/GestureEvent)  선택 / 집기 확정 / 놓기 확정 (2026-11-10)
       /aris3/gesture/state (std_msgs/String)  loading | ready | error:<이유>

선택·확정 판단(gesture_events.GestureTracker)은 2026-11-10 GUI 에서 이 노드로 옮겼다 - 두 GUI 가 같은 판단을 쓴다.
이벤트는 로봇을 움직이지 않는다: GUI 가 확인창 + 조작 허용을 거쳐 실행한다.

검지 끝 픽셀은 색 검출(/aris3/vision/detections_px)과 같은 영상 좌표라서,
GUI 의 카메라 보정(픽셀 -> 로봇 좌표)을 그대로 쓴다. 이 노드는 로봇을 움직이지 않는다.

MediaPipe 1.x 는 mp.solutions 가 없어 Tasks API(HandLandmarker, VIDEO 모드)와 모델 파일을 쓴다:
  ~/.config/aris3/models/hand_landmarker.task  (README 의 내려받기 명령)
"""
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from aris3_gesture.gesture_classifier import GestureSmoother, classify
from aris3_gesture.gesture_events import CONFIRM_PICK, CONFIRM_PLACE, SELECT, GestureTracker
from aris3_interfaces.msg import GestureEvent, HandGesture

DEFAULT_MODEL = os.path.expanduser('~/.config/aris3/models/hand_landmarker.task')
# GestureTracker 이벤트 이름 -> GestureEvent 메시지 번호
EVENT_TYPE = {SELECT: GestureEvent.SELECT, CONFIRM_PICK: GestureEvent.CONFIRM_PICK,
              CONFIRM_PLACE: GestureEvent.CONFIRM_PLACE}


class GestureNode(Node):
    """위쪽 카메라 영상에서 손을 찾아 손짓·검지 끝 픽셀·선택/확정 이벤트를 발행하는 노드."""

    def __init__(self):
        """파라미터 선언, 발행·구독 준비, MediaPipe 를 읽고 영상을 처리할 작업 스레드 시작."""
        super().__init__('aris3_gesture')
        self.declare_parameter('image_topic', '/aris3/overhead_camera/image/compressed')
        self.declare_parameter('model_path', DEFAULT_MODEL)
        self.declare_parameter('max_hz', 15.0)              # CPU 를 아끼려고 처리 빈도 제한
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('smooth_frames', 7)
        self.declare_parameter('smooth_votes', 5)
        self.declare_parameter('hold_sec', 1.0)             # 가리키고 이 시간 멈추면 선택 (예전 GUI GESTURE_HOLD_SEC)
        self.declare_parameter('hold_radius_px', 25.0)      # 이 반경 안이면 멈춘 것 (예전 GUI GESTURE_HOLD_RADIUS_PX)

        self.pub = self.create_publisher(HandGesture, '/aris3/gesture', 10)
        self.pub_state = self.create_publisher(String, '/aris3/gesture/state', 10)
        self.pub_event = self.create_publisher(GestureEvent, '/aris3/gesture/event', 10)
        self.tracker = GestureTracker(float(self.get_parameter('hold_sec').value),
                                      float(self.get_parameter('hold_radius_px').value))
        self.smoother = GestureSmoother(self.get_parameter('smooth_frames').value,
                                        self.get_parameter('smooth_votes').value)
        self.state = 'loading'
        self.landmarker = None
        self.latest = None          # 가장 최근 영상만 처리 (밀린 영상은 버림)
        self.cv = threading.Condition()
        self.t0 = time.monotonic()
        self.last_ts_ms = -1
        self.create_subscription(CompressedImage, self.get_parameter('image_topic').value, self._on_image,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, lambda: self._set_state(self.state))
        self.running = True
        threading.Thread(target=self._worker, daemon=True).start()

    def _set_state(self, s):
        self.state = s
        self.pub_state.publish(String(data=s))

    def _load(self):
        import mediapipe as mp                      # 무거운 import 는 작업 스레드에서
        from mediapipe.tasks.python import BaseOptions, vision
        path = self.get_parameter('model_path').value
        if not os.path.exists(path):
            raise FileNotFoundError(f'손 모델 없음: {path}')
        conf = float(self.get_parameter('min_detection_confidence').value)
        opts = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=path), running_mode=vision.RunningMode.VIDEO,
            num_hands=1, min_hand_detection_confidence=conf, min_hand_presence_confidence=conf,
            min_tracking_confidence=conf)
        self.mp = mp
        self.landmarker = vision.HandLandmarker.create_from_options(opts)

    def _on_image(self, msg):
        with self.cv:
            self.latest = msg
            self.cv.notify()

    def _worker(self):
        try:
            self._load()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'MediaPipe 준비 실패: {e}')
            self._set_state(f'error:{e}')
            return
        self.get_logger().info('MediaPipe 손 인식 준비')
        self._set_state('ready')
        period = 1.0 / max(1.0, float(self.get_parameter('max_hz').value))
        while self.running and rclpy.ok():
            with self.cv:
                self.cv.wait_for(lambda: self.latest is not None or not self.running, timeout=1.0)
                msg, self.latest = self.latest, None
            if msg is None:
                continue
            t = time.monotonic()
            try:
                self._process(msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'손 인식 오류: {e}', throttle_duration_sec=5.0)
            rest = period - (time.monotonic() - t)
            if rest > 0:
                time.sleep(rest)

    def _process(self, msg):
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        h, w = bgr.shape[:2]
        rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        ts = int((time.monotonic() - self.t0) * 1000)
        ts = max(ts, self.last_ts_ms + 1)             # VIDEO 모드는 시각이 계속 커져야 한다
        self.last_ts_ms = ts
        res = self.landmarker.detect_for_video(self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), ts)

        out = HandGesture()
        out.stamp = msg.header.stamp
        out.image_width, out.image_height = w, h
        now = time.monotonic()
        if not res.hand_landmarks:
            self.smoother.reset()
            self.tracker.update(now, False)                # 손이 사라짐 -> 멈춤·전환 판단 초기화
            self.pub.publish(out)
            return
        lm = res.hand_landmarks[0]
        pts = [(p.x * w, p.y * h) for p in lm]        # 픽셀로 바꿔 가로세로 비율을 맞춘다
        raw = classify(pts)
        out.hand_present = True
        out.raw_gesture = raw
        out.gesture = self.smoother.update(raw)
        out.tip_u, out.tip_v = float(pts[8][0]), float(pts[8][1])
        if res.handedness and res.handedness[0]:
            c = res.handedness[0][0]
            out.confidence = float(c.score)
            out.handedness = c.category_name or ''
        # 선택(가리키고 멈춤)·확정(가리킴 -> 주먹/펼침) 판단: 진행도는 HandGesture 에, 확정은 GestureEvent 로
        progress, events = self.tracker.update(now, True, out.gesture, out.tip_u, out.tip_v)
        out.hold_progress = float(progress)
        self.pub.publish(out)
        for kind, u, v in events:
            ev = GestureEvent(type=EVENT_TYPE[kind], u=float(u), v=float(v))
            ev.stamp = msg.header.stamp
            self.pub_event.publish(ev)
            self.get_logger().info(f'손짓 이벤트: {kind} ({u:.0f}, {v:.0f})')

    def destroy_node(self):
        self.running = False
        with self.cv:
            self.cv.notify()
        if self.landmarker is not None:
            self.landmarker.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GestureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
