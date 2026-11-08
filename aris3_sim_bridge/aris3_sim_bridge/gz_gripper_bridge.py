"""ARIS3 계약 /aris3/gripper/command -> aris3_bringup Gazebo 의 RH-P12-RN-A 그리퍼 컨트롤러.

(e0509_sim 용 gripper_bridge.py 와 같은 계약을 aris3_bringup 시뮬레이션에 제공한다.)

제공: /aris3/gripper/command  (aris3_interfaces/srv/GripperCommand)
발행: <ns>/gripper_position_controller/commands  (std_msgs/Float64MultiArray, [gripper_rh_r1 목표])
구독: <ns>/joint_states  (gripper_rh_r1 위치·속도)

잡았는지 판단 (마찰 파지):
  닫기 명령 후 손가락이 멈출 때까지 기다린다. 끝까지 닫히지 못하고 멈췄으면(물체에 막힘)
  holding=true, 끝까지 닫혔으면(빈손) holding=false.
  OPEN 0.0 / CLOSED 1.1 rad 은 rh_p12_rn_a.xacro 의 관절 한계([-0.001, 1.101])에서 정했다.

파지 보조 (assist_objects 파라미터가 비어 있지 않을 때, simulation.launch.py grasp_assist:=true 와 함께):
  CLOSE 후 손가락이 물체에 막혀 멈췄으면, TCP(link_6 + tcp_offset_m) 에서 attach_radius_m 안의
  가장 가까운 물체를 /aris3/sim/<물체>/attach 로 손에 고정한다. OPEN 하면 detach.
  물체 위치는 /aris3/detected_objects 에서 읽는다. 시작할 때 DetachableJoint 가 물체를 붙인 채로
  시작하므로, 모든 물체가 'detached' 로 확인될 때까지 detach 를 보낸다.

서비스 콜백 안에서 joint_states 를 기다리므로 MultiThreadedExecutor + ReentrantCallbackGroup 을 쓴다.
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

from rclpy.time import Time
from std_msgs.msg import Empty, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

from aris3_interfaces.msg import DetectedObjectArray
from aris3_interfaces.srv import GripperCommand


class GzGripperBridge(Node):
    def __init__(self):
        super().__init__('aris3_gz_gripper_bridge')
        self.declare_parameter('robot_ns', '/e0509_gripper')
        self.declare_parameter('service', '/aris3/gripper/command')
        self.declare_parameter('joint', 'gripper_rh_r1')
        self.declare_parameter('open_position', 0.0)
        self.declare_parameter('closed_position', 1.1)
        # 이 값보다 덜 닫힌 채 멈추면 물체에 막힌 것 = 잡음 (빈손이면 closed_position 근처까지 닫힘)
        self.declare_parameter('empty_threshold', 1.0)
        self.declare_parameter('settle_velocity', 0.02)   # rad/s, 이보다 느리면 멈춘 것
        self.declare_parameter('timeout_s', 4.0)
        self.declare_parameter('assist_objects', [''])     # 비어 있으면 보조 끔 (순수 마찰 파지)
        self.declare_parameter('tcp_offset_m', 0.105)       # link_6 -> 집는 점, 공구 z 방향
        self.declare_parameter('attach_radius_m', 0.03)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('flange_frame', 'link_6')
        ns = self.get_parameter('robot_ns').value.rstrip('/')
        self.joint = self.get_parameter('joint').value
        self.open_pos = float(self.get_parameter('open_position').value)
        self.closed_pos = float(self.get_parameter('closed_position').value)
        self.empty_th = float(self.get_parameter('empty_threshold').value)
        self.settle_vel = float(self.get_parameter('settle_velocity').value)
        self.timeout = float(self.get_parameter('timeout_s').value)

        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.pos = None
        self.vel = 0.0
        self.closed_cmd = False      # 마지막 명령이 CLOSE 였는가
        self.pub = self.create_publisher(Float64MultiArray, f'{ns}/gripper_position_controller/commands', 10)
        self.create_subscription(JointState, f'{ns}/joint_states', self._on_js, 50, callback_group=cb)
        # ---- 파지 보조
        self.assist = [n for n in self.get_parameter('assist_objects').value if n]
        self.tcp_offset = float(self.get_parameter('tcp_offset_m').value)
        self.attach_r = float(self.get_parameter('attach_radius_m').value)
        self.world = self.get_parameter('world_frame').value
        self.flange = self.get_parameter('flange_frame').value
        self.held = ''
        self.objects = {}
        self.joint_state = {}
        if self.assist:
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.attach_pub = {n: self.create_publisher(Empty, f'/aris3/sim/{n}/attach', 10) for n in self.assist}
            self.detach_pub = {n: self.create_publisher(Empty, f'/aris3/sim/{n}/detach', 10) for n in self.assist}
            for n in self.assist:
                self.create_subscription(String, f'/aris3/sim/{n}/state',
                                         lambda m, n=n: self.joint_state.__setitem__(n, m.data), 10,
                                         callback_group=cb)
            self.create_subscription(DetectedObjectArray, '/aris3/detected_objects', self._on_objects, 10,
                                     callback_group=cb)
            self.release_count = 0
            self.release_timer = self.create_timer(0.5, self._release_at_startup, callback_group=cb)
            self.get_logger().warn(f'파지 보조 켜짐 (DetachableJoint): {self.assist}')

        srv = self.get_parameter('service').value
        self.create_service(GripperCommand, srv, self._on_cmd, callback_group=cb)
        self.get_logger().info(f'{srv} -> {ns}/gripper_position_controller ({self.joint})')

    def _on_js(self, msg):
        if self.joint not in msg.name:
            return
        i = msg.name.index(self.joint)
        with self.lock:
            self.pos = msg.position[i]
            self.vel = msg.velocity[i] if i < len(msg.velocity) else 0.0

    def _on_objects(self, msg):
        self.objects = {o.id: o.pose.pose.position for o in msg.objects if o.id}

    def _release_at_startup(self):
        # DetachableJoint 는 상태가 바뀔 때만 state 를 보내므로, 이미 분리된 물체는 확인 메시지가 오지 않는다.
        # 그래서 확인을 기다리지 않고 정해진 횟수(3 s)만 detach 를 보낸다.
        self.release_count += 1
        for n in self.assist:
            self.detach_pub[n].publish(Empty())
        if self.release_count >= 6:
            self.release_timer.cancel()
            self.get_logger().info('파지 보조: 시작 시 물체 분리 명령 완료')

    def _tcp_world(self):
        t = self.tf.lookup_transform(self.world, self.flange, Time())
        q, p = t.transform.rotation, t.transform.translation
        # 공구 z축(플랜지 z) 방향 = 회전 행렬의 3번째 열
        zx = 2 * (q.x * q.z + q.w * q.y)
        zy = 2 * (q.y * q.z - q.w * q.x)
        zz = 1 - 2 * (q.x * q.x + q.y * q.y)
        return (p.x + zx * self.tcp_offset, p.y + zy * self.tcp_offset, p.z + zz * self.tcp_offset)

    def _attach_nearest(self):
        try:
            tcp = self._tcp_world()
        except Exception as e:  # noqa: BLE001 - TF 가 없으면 보조 없이 진행
            return '', f'TCP 위치 모름 ({e})'
        best, best_d = '', float('inf')
        for n in self.assist:
            p = self.objects.get(n)
            if p is None:
                continue
            d = math.dist(tcp, (p.x, p.y, p.z))
            if d < best_d:
                best, best_d = n, d
        if not best or best_d > self.attach_r:
            return '', f'TCP {self.attach_r * 1000:.0f} mm 안에 물체 없음 (가장 가까운 {best} {best_d * 1000:.0f} mm)'
        if not self._set_joint(best, attach=True):
            return '', f'보조: {best} 고정 확인 실패'
        self.held = best
        return best, f'보조: {best} 고정 (TCP 와 {best_d * 1000:.0f} mm)'

    def _set_joint(self, name, attach, timeout=2.0):
        """attach/detach 를 상태 토픽에서 확인될 때까지 반복해서 보낸다.

        한 번만 보낸 detach 가 처리되지 않아 물체가 손에 붙은 채 남은 일이 있었다 (2026-11-08).
        """
        want = 'attached' if attach else 'detached'
        pub = (self.attach_pub if attach else self.detach_pub)[name]
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pub.publish(Empty())
            deadline = time.monotonic() + 0.5          # 한 번 보내고 상태 변화를 0.5 s 기다린다
            while time.monotonic() < deadline:
                if self.joint_state.get(name) == want:
                    return True
                time.sleep(0.05)
        self.get_logger().error(f'{name}: {want} 상태 확인 실패 (현재 {self.joint_state.get(name)})')
        return False

    def _state(self):
        with self.lock:
            return self.pos, self.vel

    def _move_and_settle(self, target):
        """목표를 보내고 손가락이 멈출 때까지 기다린다. (최종 위치, 멈췄는가)"""
        self.pub.publish(Float64MultiArray(data=[target]))
        start = time.monotonic()
        time.sleep(0.3)                       # 움직이기 시작할 시간
        still_since = None
        while time.monotonic() - start < self.timeout:
            pos, vel = self._state()
            if pos is not None and abs(vel) < self.settle_vel:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.3:
                    return pos, True
            else:
                still_since = None
            time.sleep(0.05)
        return self._state()[0], False

    def _on_cmd(self, req, res):
        if self._state()[0] is None:
            res.success, res.message = False, 'joint_states 에 그리퍼 관절이 아직 없음 (시뮬레이션 확인)'
            return res
        if req.command == GripperCommand.Request.STATUS:
            pos, _ = self._state()
            res.success = True
            res.holding = self.closed_cmd and pos < self.empty_th
            res.object_id = self.held
            res.message = f'{self.joint}={pos:.3f} ({"닫힘" if self.closed_cmd else "열림"})'
            return res
        if req.command not in (GripperCommand.Request.OPEN, GripperCommand.Request.CLOSE):
            res.success, res.message = False, f'모르는 명령 {req.command}'
            return res
        closing = req.command == GripperCommand.Request.CLOSE
        if not closing and self.held:                      # 열기 전에 보조 고정을 푼다
            if not self._set_joint(self.held, attach=False):
                res.success, res.message = False, f'보조 고정 해제 실패: {self.held}'
                return res
            self.held = ''
        pos, settled = self._move_and_settle(self.closed_pos if closing else self.open_pos)
        self.closed_cmd = closing
        res.success = settled
        res.holding = closing and pos < self.empty_th
        what = 'CLOSE' if closing else 'OPEN'
        if not settled:
            res.message = f'{what}: {self.timeout:.0f}s 안에 손가락이 멈추지 않음 ({self.joint}={pos:.3f})'
        elif closing:
            res.message = (f'CLOSE: {self.joint}={pos:.3f} -> '
                           f'{"물체에 막혀 멈춤 (잡음)" if res.holding else "끝까지 닫힘 (빈손)"}')
        # 막혀서 멈추는 중이면(settled 가 아니어도) 잡은 것으로 보고 보조 고정을 시도
        if closing and res.holding and self.assist:
            # 손가락을 막힌 자리에 멈춘다. 계속 닫으려 하면 손에 고정된 물체를 밀어
            # 손가락 메시 안으로 박아 넣는다 (2026-11-08: 고정 해제 뒤에도 물체가 손가락에 끼어 있었음).
            self.pub.publish(Float64MultiArray(data=[pos]))
            res.success = True
            res.object_id, note = self._attach_nearest()
            res.message += f' | {note}'
            if not res.object_id:
                res.holding = False
        else:
            res.message = f'OPEN: {self.joint}={pos:.3f}'
        self.get_logger().info(f'{res.message} success={res.success} holding={res.holding}')
        return res


def main():
    rclpy.init()
    node = GzGripperBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
