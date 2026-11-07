"""ARIS3 계약 /aris3/gripper/command -> e0509_sim 그리퍼 서비스(/dsr01/gripper/open|close)로 바꿔 주는 어댑터.

제공: /aris3/gripper/command (aris3_interfaces/srv/GripperCommand)
호출: /<robot_ns>/gripper/open, /<robot_ns>/gripper/close (std_srvs/srv/Trigger)
구독: /<robot_ns>/gripper/state (std_msgs/String: "open" | "closed" | "holding:<block>") -> STATUS 명령에 사용

서비스 콜백 안에서 다른 서비스를 부르는 방법(중요):
  콜백이 끝나기를 기다리는 동안에도 응답을 받아야 하므로
  1) MultiThreadedExecutor 로 스레드를 여러 개 돌리고
  2) 서버와 클라이언트를 ReentrantCallbackGroup 에 넣고
  3) 콜백 안에서는 call_async + threading.Event 로 기다린다.
  단일 스레드에서 spin_until_future_complete 를 콜백 안에서 쓰면 멈춘다(deadlock).
"""
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from aris3_interfaces.srv import GripperCommand


class GripperBridge(Node):
    def __init__(self):
        super().__init__('aris3_gripper_bridge')
        self.declare_parameter('robot_ns', '/dsr01')
        self.declare_parameter('service', '/aris3/gripper/command')
        self.declare_parameter('timeout_s', 10.0)
        ns = self.get_parameter('robot_ns').value.rstrip('/')
        self.timeout = float(self.get_parameter('timeout_s').value)
        # 중복 실행 방지: 같은 이름의 서비스 서버가 둘이면 요청 하나가 양쪽에서 처리될 수 있다.
        # 그래프에 이름이 보이는지만 보면, 강제 종료된 이전 프로세스의 이름이 남아 있어서 재시작까지 막는다.
        # 그래서 STATUS(움직이지 않는 명령)를 실제로 보내 응답이 오는지 확인한다.
        srv_name = self.get_parameter('service').value
        probe = self.create_client(GripperCommand, srv_name)
        if probe.wait_for_service(timeout_sec=3.0):   # 다른 노드 발견에 1~2 초 걸릴 수 있다
            fut = probe.call_async(GripperCommand.Request(command=GripperCommand.Request.STATUS))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.5)
            if fut.done() and fut.result() is not None:
                raise RuntimeError(f'{srv_name} 서버가 이미 실행 중 - 두 번 띄우지 마세요')
        self.destroy_client(probe)
        cb = ReentrantCallbackGroup()
        self.cli_open = self.create_client(Trigger, f'{ns}/gripper/open', callback_group=cb)
        self.cli_close = self.create_client(Trigger, f'{ns}/gripper/close', callback_group=cb)
        self.state = ''
        self.create_subscription(String, f'{ns}/gripper/state', self._on_state, 10, callback_group=cb)
        self.create_service(GripperCommand, self.get_parameter('service').value, self._on_cmd, callback_group=cb)
        self.get_logger().info(f'{self.get_parameter("service").value} -> {ns}/gripper/open|close')

    def _call(self, cli, timeout):
        if not cli.wait_for_service(timeout_sec=2.0):
            return None, f'{cli.srv_name} 없음 (시뮬레이션이 실행 중인가?)'
        done = threading.Event()
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None, f'{cli.srv_name} 응답 없음 ({timeout:.0f}s)'
        return fut.result(), ''

    def _on_state(self, msg):
        self.state = msg.data

    def _on_cmd(self, req, res):
        if req.command == GripperCommand.Request.STATUS:
            if not self.state:
                res.success, res.message = False, '그리퍼 상태가 아직 들어오지 않음'
                return res
            res.success, res.message = True, self.state
            res.holding = self.state.startswith('holding:')
            res.object_id = self.state.split(':', 1)[1] if res.holding else ''
            return res
        closing = req.command == GripperCommand.Request.CLOSE
        if req.command not in (GripperCommand.Request.OPEN, GripperCommand.Request.CLOSE):
            res.success, res.message = False, f'모르는 명령 {req.command}'
            return res
        r, err = self._call(self.cli_close if closing else self.cli_open, self.timeout)
        if r is None:
            res.success, res.message = False, err
            return res
        res.success, res.message = bool(r.success), r.message
        # e0509_sim 그리퍼의 응답 문장: "grasped red_block (...)", "already holding red_block"
        if closing and r.success:
            words = r.message.replace('(', ' ').split()
            held = [w for w in words if w.endswith('_block')]
            res.holding = bool(held)
            res.object_id = held[0] if held else ''
        else:
            res.holding = False
        self.get_logger().info(f'{"CLOSE" if closing else "OPEN"} -> success={res.success} '
                               f'holding={res.holding} {res.object_id} | {res.message}')
        return res


def main():
    rclpy.init()
    try:
        node = GripperBridge()
    except RuntimeError as e:
        print(f'[gripper_bridge] 시작 실패: {e}')
        rclpy.shutdown()
        raise SystemExit(1)
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
