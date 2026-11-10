"""두산 가상 컨트롤러(DRCF 에뮬레이터)의 관절 각도 -> Gazebo 로봇 팔 (2026-11-10, 실물 비슷한 시뮬레이션용).

실물 경로(aris3_real_bridge -> RobotLink -> 두산 서비스)를 로봇 없이 시험하려고, 팔 움직임은 두산 가상 컨트롤러가
계산하고 Gazebo 의 ARIS3 로봇(RH-P12-RN-A 그리퍼·큐브·카메라가 있는 월드)은 그 관절 각도를 그대로 따라가게 한다.
e0509_sim 의 dsr_bringup2 gazebo_connection 과 같은 역할이지만, ARIS3 월드의 이름 공간·제어기 이름에 맞춘다.

  구독: /<emulator_ns>/joint_states                       (sensor_msgs/JointState, 에뮬레이터 = 실물과 같은 이름)
  발행: <robot_ns>/dsr_position_controller/commands         (std_msgs/Float64MultiArray, joint_1..6 [rad])

aris3_bringup 의 dsr_position_controller 가 active 여야 한다 (aris3_real_sim.launch.py 가 arm_controller 대신 켬).
에뮬레이터가 꺼져 있으면 아무것도 보내지 않으므로 팔은 마지막 자세를 유지한다 (실물: 브레이크가 잡음).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARM_JOINTS = [f'joint_{i}' for i in range(1, 7)]


def ordered_positions(names, positions, joints=ARM_JOINTS):
    """JointState 의 (이름, 값) 을 joints 순서로 바꾼다. 하나라도 없으면 None (반쪽 명령을 보내지 않게)."""
    table = dict(zip(names, positions))
    if not all(j in table for j in joints):
        return None
    return [float(table[j]) for j in joints]


class DsrJointMirror(Node):
    """에뮬레이터 관절 상태를 받아 Gazebo 위치 제어기 명령으로 그대로 넘기는 노드."""

    def __init__(self):
        """파라미터: emulator_ns (기본 e0509_real = 실물 브리지 기본 이름), robot_ns (ARIS3 Gazebo 로봇)."""
        super().__init__('aris3_dsr_joint_mirror')
        self.declare_parameter('emulator_ns', 'e0509_real')
        self.declare_parameter('robot_ns', '/e0509_gripper')
        emu = self.get_parameter('emulator_ns').value.strip('/')
        robot = '/' + self.get_parameter('robot_ns').value.strip('/')
        self.pub = self.create_publisher(Float64MultiArray, f'{robot}/dsr_position_controller/commands', 10)
        self.create_subscription(JointState, f'/{emu}/joint_states', self._on_js, 50)
        self.count = 0
        self.get_logger().info(f'/{emu}/joint_states -> {robot}/dsr_position_controller/commands')

    def _on_js(self, msg):
        """관절 상태 한 장 -> 위치 명령 한 장. 처음 받았을 때 한 번 로그."""
        q = ordered_positions(msg.name, msg.position)
        if q is None:
            return
        self.pub.publish(Float64MultiArray(data=q))
        self.count += 1
        if self.count == 1:
            self.get_logger().info('가상 컨트롤러 관절 상태 수신 - Gazebo 팔이 따라갑니다')


def main():
    """노드 실행."""
    rclpy.init()
    node = DsrJointMirror()
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
