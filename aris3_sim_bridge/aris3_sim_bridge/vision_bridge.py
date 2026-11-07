"""e0509_sim 카메라 인식 결과 -> ARIS3 계약 /aris3/detected_objects 로 바꿔 보내는 어댑터.

입력:  /e0509/vision/detections  (std_msgs/String, JSON)
       {"red_block": {"u": 320.1, "v": 240.5, "area_px": 700, "world": [x, y, z], "blobs": 1}, ...}
출력:  /aris3/detected_objects   (aris3_interfaces/DetectedObjectArray, frame "workcell")

왜 필요한가: Task Manager는 "누가 만든 비전인지" 몰라도 되도록 ARIS3 공통 메시지만 구독한다.
나중에 팀의 aris3_vision(C++/OpenCV)이 같은 토픽을 내보내면 이 노드는 끄기만 하면 된다.

주의: 입력 JSON에는 영상 시각이 없어서, 받은 시각을 header.stamp 로 쓴다(수 ms 차이).
좌표계 이름: e0509_sim 은 Gazebo 바닥 기준 좌표를 "world" 라고 부르지만, 같은 시뮬레이션의 TF 에서
"world" 는 로봇 베이스와 같은 점(테이블 위 0.75 m)이다. 이름이 같은데 다른 좌표계라서,
여기서는 Gazebo 바닥 기준 좌표를 "workcell" 이라고 정직하게 붙인다 (TF: workcell -> world, z=0.75).
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from aris3_interfaces.msg import DetectedObject, DetectedObjectArray


class VisionBridge(Node):
    def __init__(self):
        super().__init__('aris3_vision_bridge')
        self.declare_parameter('input_topic', '/e0509/vision/detections')
        self.declare_parameter('output_topic', '/aris3/detected_objects')
        self.declare_parameter('frame_id', 'workcell')
        self.declare_parameter('shape', 'cube')            # e0509_sim 블록은 모두 정육면체
        self.declare_parameter('expected_area_px', 700.0)  # 4 cm 블록 윗면이 카메라에서 차지하는 넓이
        self.frame_id = self.get_parameter('frame_id').value
        self.shape = self.get_parameter('shape').value
        self.expected_area = float(self.get_parameter('expected_area_px').value)
        self.pub = self.create_publisher(DetectedObjectArray, self.get_parameter('output_topic').value, 10)
        self.create_subscription(String, self.get_parameter('input_topic').value, self._on_json, 10)
        self.count = 0
        self.get_logger().info(f'{self.get_parameter("input_topic").value} -> '
                               f'{self.get_parameter("output_topic").value}')

    def _confidence(self, d):
        """간단한 신뢰도: 조각(blob)이 하나이고 넓이가 예상과 비슷할수록 1에 가깝다."""
        ratio = float(d.get('area_px', 0)) / self.expected_area if self.expected_area > 0 else 1.0
        size_score = max(0.0, 1.0 - abs(1.0 - ratio))           # 넓이가 예상의 2배 이상 다르면 0
        blob_score = 1.0 if int(d.get('blobs', 1)) == 1 else 0.5  # 여러 조각으로 갈라졌으면 의심
        return round(max(0.05, min(1.0, 0.5 + 0.5 * size_score)) * blob_score, 3)

    def _on_json(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn('JSON 해석 실패 - 무시')
            return
        out = DetectedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        for name, d in data.items():
            w = d.get('world')
            if not w or len(w) != 3:
                continue
            o = DetectedObject()
            o.id = name
            o.color = name.split('_')[0]                       # "red_block" -> "red"
            o.shape = self.shape
            o.pose.header = out.header
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, w)
            o.pose.pose.orientation.w = 1.0                    # 회전(yaw)은 아직 추정하지 않음
            o.confidence = float(self._confidence(d))
            o.size_px = float(d.get('area_px', 0))
            out.objects.append(o)
        self.pub.publish(out)                                  # 물체가 없어도 빈 목록을 보낸다
        self.count += 1
        if self.count == 1:
            self.get_logger().info(f'첫 영상 결과 전달: 물체 {len(out.objects)}개')


def main():
    rclpy.init()
    node = VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
