"""[임시] Gazebo 물체의 실제 위치 -> ARIS3 계약 /aris3/detected_objects.

aris3_vision(C++ OpenCV)이 아직 없어서, Task Manager 와 MoveIt 연동을 먼저 검증하기 위한 대체 노드다.
카메라로 인식한 것이 아니라 시뮬레이터가 알고 있는 정답 위치이므로 confidence=1.0 이고,
비전 성능(GATE 3)의 증거로 쓰면 안 된다. aris3_vision 이 같은 토픽을 내보내면 이 노드는 끈다.

입력: /model/<물체>/pose (ros_gz_bridge: gz.msgs.Pose -> geometry_msgs/PoseStamped)
      spawn_objects.py 가 물체마다 넣은 PosePublisher 플러그인이 발행. 좌표 = Gazebo world (= ROS TF 'world')
      (/world/<world>/pose/info 는 브리지를 거치면 엔티티 이름이 빠져서 쓸 수 없었다)
출력: /aris3/detected_objects (aris3_interfaces/DetectedObjectArray, frame 'world'), rate_hz 로
물체: objects 파라미터의 이름 '<색>_<도형>' (예: red_cube)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from aris3_interfaces.msg import DetectedObject, DetectedObjectArray


class GzTruthDetector(Node):
    def __init__(self):
        super().__init__('aris3_gz_truth_detector')
        self.declare_parameter('objects', [''])
        self.declare_parameter('output_topic', '/aris3/detected_objects')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('rate_hz', 10.0)
        self.names = [n for n in self.get_parameter('objects').value if n]
        self.poses = {}
        self.frame = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(DetectedObjectArray, self.get_parameter('output_topic').value, 10)
        for n in self.names:
            self.create_subscription(PoseStamped, f'/model/{n}/pose',
                                     lambda m, n=n: self.poses.__setitem__(n, m.pose), 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._publish)
        self.get_logger().warn('정답 위치(시뮬레이터) 기반 임시 인식 노드 - 비전 결과가 아님')

    def _publish(self):
        out = DetectedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        for name, pose in list(self.poses.items()):
            color, _, shape = name.partition('_')
            o = DetectedObject(id=name, color=color, shape=shape, confidence=1.0, size_px=0.0)
            o.pose.header = out.header
            o.pose.pose = pose
            out.objects.append(o)
        self.pub.publish(out)   # 물체가 없어도 빈 목록을 보낸다 (계약)


def main():
    rclpy.init()
    node = GzTruthDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
