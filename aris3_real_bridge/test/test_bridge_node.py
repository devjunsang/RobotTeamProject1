"""RealBridge 노드 시험: 가짜 RobotLink 로 실제 ROS 서비스·액션 호출 (로봇 없이).

확인하는 것: 조작 허용(armed) 없으면 거부, 정지는 언제나, 취소 = 정지, 보정 없으면 집기 거부,
집기 직전 검출로 픽셀을 고침, 상태 토픽 필드, 동작 하나씩만.
"""
import os
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool, Trigger

from aris3_interfaces.action import RealMotion, RealPickPlace
from aris3_interfaces.msg import DetectedObject, DetectedObjectArray, RealRobotState
from aris3_interfaces.srv import RealCalibration
from aris3_real_bridge.bridge_node import RealBridge, nearest_px
from aris3_real_bridge.real_pick import PixelCalibration


class Ready:
    """service_is_ready() 만 있는 가짜 서비스 클라이언트. ok 로 준비 여부를 정한다."""
    def __init__(self, ok=True):
        """ok=True 면 준비된 서비스로 본다."""
        self.ok = ok

    def service_is_ready(self):
        """준비 여부."""
        return self.ok


class FakeLink:
    """RealBridge·PickSequence 가 쓰는 RobotLink 부분. 이동은 move_time 초 걸린 뒤 도착한다."""

    def __init__(self, move_time=0.2):
        """대기 상태·자동 모드·홈 자세에서 시작. move_time: 이동 한 번에 걸리는 시간 [s]."""
        self.move_time = move_time
        self.joints_deg = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.posx = [400.0, 0.0, 300.0, 180.0, 0.0, 90.0]
        self.robot_state, self.robot_mode = 1, 1
        self.mode_switching = self.recovering = self.busy = self.gripper_running = False
        self.gripper, self.gripper_ok = None, None
        self.cli_drl_start, self.cli_control, self.cli_stop = Ready(), Ready(), Ready()
        self.log_lines = []
        self.last_ok = False
        self._stop_requested = False
        self.calls = []
        self.stops = 0

    def log(self, msg):
        """RobotLink 처럼 '시각 + 문장' 으로 기록한다 (브리지가 마지막 줄을 결과 문장으로 씀)."""
        self.log_lines.append('00:00:00 ' + msg)

    def joints_fresh(self):
        """관절 상태가 늘 들어오는 것으로 한다 (접속됨)."""
        return True

    def state_name(self):
        """상태 번호 -> 이름."""
        return {1: 'STANDBY', 2: 'MOVING'}.get(self.robot_state, 'UNKNOWN')

    def why_not_ready(self):
        """이동 중이면 거부 이유, 아니면 None."""
        return '이전 동작이 끝나지 않음' if self.busy else None

    def _motion(self, name, target=None):
        """모든 이동의 공통: 기록하고 move_time 동안 busy, 정지가 오면 중간에 멈춘다."""
        if self.busy:
            return False
        self.calls.append(name)
        self.busy, self.last_ok, self._stop_requested = True, False, False

        def run():
            """move_time 동안 기다리다가(정지 오면 즉시) 목표 위치로 옮기고 결과를 남긴다."""
            t0 = time.time()
            while time.time() - t0 < self.move_time and not self._stop_requested:
                time.sleep(0.01)
            if target is not None and not self._stop_requested:
                self.posx[:3] = list(target)
            self.last_ok = not self._stop_requested
            self.log(f'{name}: ' + ('완료' if self.last_ok else '정지 명령으로 멈춤'))
            self.busy = False
        threading.Thread(target=run, daemon=True).start()
        return True

    def jog_joint(self, i, d, v):
        """관절 조그 (기록만)."""
        return self._motion(f'jog_joint {i} {d:+.0f}')

    def jog_cart(self, a, d, v):
        """TCP 조그 (기록만)."""
        return self._motion(f'jog_cart {a} {d:+.0f}')

    def move_home(self, v):
        """홈 이동 (기록만)."""
        return self._motion('home')

    def recover_and_home(self, v):
        """복구 후 홈 (기록만)."""
        return self._motion('recover')

    def move_tcp_to(self, x, y, z, vel, label=None):
        """좌표 이동: 끝나면 위치를 목표로 바꾼다."""
        return self._motion(f'move {x:.0f} {y:.0f} {z:.0f}', (x, y, z))

    def gripper_command(self, cmd, pos=0, cur=0, vel=0, label=None):
        """그리퍼: 닫으면 500 에서 멈춘 것으로 (= 잡음)."""
        self.gripper = {'position': 500 if pos else 0, 'torque': True}
        self.gripper_ok = True
        return self._motion(f'grip {pos}')

    def wait_motion(self, timeout=90.0):
        """이동이 끝날 때까지 기다리고 정상 종료였는지 돌려준다."""
        time.sleep(0.02)
        while self.busy:
            time.sleep(0.01)
        return self.last_ok and not self._stop_requested

    def set_hand_guide(self, on):
        """손으로 움직이기 요청을 기록한다."""
        self.calls.append(f'hand_guide {on}')
        return True

    def stop(self):
        """정지 횟수를 세고 진행 중 이동을 멈춘다."""
        self.stops += 1
        self._stop_requested = True


class FakeMgr:
    """가짜 BringupManager: 접속 설명만 (빈 문자열)."""
    def describe(self):
        """접속 설명 없음."""
        return ''


class FakeConn:
    """가짜 ConnectionController: 접속·해제 작업이 없는 상태."""
    def busy(self):
        """접속·해제 작업 중이 아님."""
        return False

    def connect_async(self):
        """접속 시작 성공으로 한다."""
        return True

    def disconnect_async(self):
        """해제 시작 성공으로 한다."""
        return True


# 안전: 이 시험은 조작 허용·조그 요청을 보낸다. 같은 네트워크에서 진짜 브리지가 로봇에 붙어 있으면 그 로봇이
# 움직일 수 있으므로, 시험 전용 ROS 도메인(다른 노드와 통신하지 않음)에서만 돌린다.
TEST_DOMAIN_ID = 90 + os.getpid() % 10


@pytest.fixture(scope='module')
def ros():
    """시험 모듈 전체에서 rclpy 를 한 번 켜고 끈다 (시험 전용 도메인)."""
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.shutdown()


@pytest.fixture
def env(ros, tmp_path, monkeypatch):
    # 보정 파일을 실제 사용자 폴더에서 읽지 않게
    """가짜 로봇에 붙은 브리지 + 시험 클라이언트 노드를 백그라운드에서 돌린다. (link, bridge, client) 를 준다."""
    monkeypatch.setattr(PixelCalibration, 'load', lambda self, path=None: (_ for _ in ()).throw(OSError('none')))
    link = FakeLink()
    bridge = RealBridge(link, FakeMgr(), FakeConn(), state_ko={'STANDBY': '대기'})
    client = rclpy.create_node('test_client')
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(bridge)
    ex.add_node(client)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    yield link, bridge, client
    ex.shutdown()
    bridge.destroy_node()
    client.destroy_node()


def call(client, srv_type, name, req, timeout=5.0):
    """서비스를 한 번 부르고 응답을 기다린다 (timeout 초)."""
    cli = client.create_client(srv_type, name)
    assert cli.wait_for_service(timeout_sec=timeout)
    fut = cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.01)
    return fut.result()


def send(client, action_type, name, goal, cancel_after=None, timeout=10.0):
    """액션 목표를 보내고 결과를 기다린다. 거부되면 None. cancel_after 초 뒤 취소할 수 있다."""
    ac = ActionClient(client, action_type, name)
    assert ac.wait_for_server(timeout_sec=timeout)
    gf = ac.send_goal_async(goal)
    t0 = time.time()
    while not gf.done() and time.time() - t0 < timeout:
        time.sleep(0.01)
    gh = gf.result()
    if not gh.accepted:
        return None
    if cancel_after is not None:
        time.sleep(cancel_after)
        gh.cancel_goal_async()
    rf = gh.get_result_async()
    while not rf.done() and time.time() - t0 < timeout:
        time.sleep(0.01)
    return rf.result()


def arm(client, on=True):
    """/aris3/real/set_armed 로 조작 허용을 켠다(끈다)."""
    assert call(client, SetBool, '/aris3/real/set_armed', SetBool.Request(data=on)).success


def jog_goal(delta=10.0):
    """TCP Z 축으로 delta mm 조그하는 RealMotion 목표."""
    return RealMotion.Goal(kind=RealMotion.Goal.JOG_TCP, index=2, delta=delta, speed_percent=30.0)


def test_motion_refused_when_not_armed(env):
    """조작 허용이 꺼져 있으면 이동을 거부하고 로봇에 아무 명령도 보내지 않는다."""
    link, _bridge, client = env
    r = send(client, RealMotion, '/aris3/real/motion', jog_goal())
    assert not r.result.success
    assert '조작 허용' in r.result.message
    assert link.calls == []


def test_motion_runs_when_armed(env):
    """조작 허용을 켜면 조그가 실행되고 성공으로 끝난다."""
    link, _bridge, client = env
    arm(client)
    r = send(client, RealMotion, '/aris3/real/motion', jog_goal())
    assert r.result.success, r.result.message
    assert link.calls == ['jog_cart 2 +10']


def test_cancel_sends_stop(env):
    """이동 중 액션을 취소하면 로봇에 정지가 가고 결과는 실패다."""
    link, _bridge, client = env
    link.move_time = 2.0
    arm(client)
    r = send(client, RealMotion, '/aris3/real/motion', jog_goal(), cancel_after=0.3)
    assert not r.result.success
    assert link.stops >= 1


def test_stop_works_without_arm(env):
    """정지는 조작 허용과 상관없이 언제나 받는다."""
    link, _bridge, client = env
    assert call(client, Trigger, '/aris3/real/stop', Trigger.Request()).success
    assert link.stops == 1


def test_stop_reports_when_not_connected(env):
    """로봇 연결이 없으면 정지를 보내지 못했다고 정직하게 알린다."""
    link, _bridge, client = env
    link.cli_stop = Ready(False)
    r = call(client, Trigger, '/aris3/real/stop', Trigger.Request())
    assert not r.success and '연결' in r.message
    assert link.stops == 1                             # 그래도 RobotLink.stop 은 부른다


def test_only_one_goal_at_a_time(env):
    """이동 중에 두 번째 목표가 오면 거부하고, 첫 목표는 끝까지 간다."""
    link, bridge, client = env
    link.move_time = 1.0
    arm(client)
    ac = ActionClient(client, RealMotion, '/aris3/real/motion')
    assert ac.wait_for_server(timeout_sec=5)
    first = ac.send_goal_async(jog_goal())
    time.sleep(0.3)
    second = send(client, RealMotion, '/aris3/real/motion', jog_goal(-10.0))
    assert second is None                              # 거부됨
    while not first.done():
        time.sleep(0.01)
    rf = first.result().get_result_async()
    while not rf.done():
        time.sleep(0.01)
    assert rf.result().result.success
    assert not bridge.active_goal


def test_pick_refused_without_calibration(env):
    """카메라 보정이 없으면 집기를 거부한다."""
    _link, _bridge, client = env
    arm(client)
    g = RealPickPlace.Goal(kind=RealPickPlace.Goal.PICK, target_name='red', target_u=100.0, target_v=100.0)
    r = send(client, RealPickPlace, '/aris3/real/pick_place', g)
    assert not r.result.success and '보정' in r.result.message


def px_obj(oid, color, u, v):
    """영상 픽셀 (u, v) 에 있는 색 검출 하나 (pose.position.x/y = 픽셀)."""
    o = DetectedObject()
    o.id, o.color = oid, color
    o.pose.pose.position.x, o.pose.pose.position.y = float(u), float(v)
    return o


def calibrate(bridge):
    # 픽셀 -> 로봇 mm: x = 300 + u, y = v - 200 (쉬운 정답)
    """쉬운 정답 보정 5 점을 넣는다: 로봇 x = 300 + u, y = v - 200 [mm], 집는 높이 180 mm."""
    for u, v in ((0, 0), (200, 0), (200, 200), (0, 200), (100, 50)):
        bridge.calib.add(u, v, 300.0 + u, v - 200.0, 180.0)


def test_pick_uses_fresh_detection_pixel(env):
    """GUI 가 고른 뒤 큐브가 움직였으면, 집기 직전의 검출 위치로 고쳐서 내려간다."""
    link, bridge, client = env
    link.move_time = 0.02
    calibrate(bridge)
    pub = client.create_publisher(DetectedObjectArray, '/aris3/vision/detections_px', 10)
    t0 = time.time()
    while time.time() - bridge.px_stamp > 0.5 and time.time() - t0 < 3:
        pub.publish(DetectedObjectArray(objects=[px_obj('red_cube', 'red', 110, 60)]))
        time.sleep(0.05)
    arm(client)
    # GUI 가 고른 뒤 큐브가 (100, 50) -> (110, 60) 으로 10 px 움직였다
    g = RealPickPlace.Goal(kind=RealPickPlace.Goal.PICK, target_name='red', target_u=100.0, target_v=50.0)
    r = send(client, RealPickPlace, '/aris3/real/pick_place', g)
    assert r.result.success, r.result.message
    assert r.result.holding == RealPickPlace.Result.HOLD_YES
    assert 'move 410 -140 180' in link.calls           # 고친 픽셀 (110, 60) 기준으로 하강


def test_stack_puts_cube_on_base_cube(env):
    """쌓기: 노랑을 집어 지금 보이는 파랑 위(놓는 높이 180 + 40 + 5 mm)에 놓는다."""
    link, bridge, client = env
    link.move_time = 0.02
    calibrate(bridge)
    pub = client.create_publisher(DetectedObjectArray, '/aris3/vision/detections_px', 10)
    objs = [px_obj('yellow_cube', 'yellow', 100, 50), px_obj('blue_cube', 'blue', 20, 150)]
    t0 = time.time()
    while time.time() - bridge.px_stamp > 0.5 and time.time() - t0 < 3:
        pub.publish(DetectedObjectArray(objects=objs))
        time.sleep(0.05)
    arm(client)
    g = RealPickPlace.Goal(kind=RealPickPlace.Goal.PICK_AND_STACK, target_name='yellow',
                           target_u=100.0, target_v=50.0, place_u=22.0, place_v=148.0)
    r = send(client, RealPickPlace, '/aris3/real/pick_place', g)
    assert r.result.success, r.result.message
    assert 'move 400 -150 180' in link.calls            # 노랑 집기
    assert 'move 320 -50 225' in link.calls             # 파랑(20, 150) 위 큐브 한 개(40) + 여유 5 mm


def test_stack_refused_when_base_not_visible(env):
    """받침 큐브가 지금 안 보이면 쌓기를 거부한다 (빈 곳 위에서 떨어뜨리지 않게)."""
    _link, bridge, client = env
    calibrate(bridge)
    arm(client)
    g = RealPickPlace.Goal(kind=RealPickPlace.Goal.PICK_AND_STACK, target_name='yellow',
                           target_u=100.0, target_v=50.0, place_u=20.0, place_v=150.0)
    r = send(client, RealPickPlace, '/aris3/real/pick_place', g)
    assert not r.result.success and '받침' in r.result.message


def test_state_message_fields(env):
    """상태 토픽에 접속·상태·관절·보정 행렬·조작 허용 남은 시간이 맞게 실린다."""
    link, bridge, client = env
    calibrate(bridge)
    got = []
    client.create_subscription(RealRobotState, '/aris3/real/state', got.append, 10)
    arm(client)
    t0 = time.time()
    while not any(m.armed for m in got) and time.time() - t0 < 3:
        time.sleep(0.05)
    m = [m for m in got if m.armed][-1]
    assert m.connected and m.robot_state_ok and m.robot_state_text == '대기'
    assert m.joints_valid and list(m.joints_deg) == [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    assert m.calib_ready and m.calib_points == 5 and m.calib_grasp_z_mm == pytest.approx(180.0)
    h = list(m.calib_homography)
    x, y, w = (h[0] * 10 + h[1] * 20 + h[2], h[3] * 10 + h[4] * 20 + h[5], h[6] * 10 + h[7] * 20 + h[8])
    assert (x / w, y / w) == pytest.approx((310.0, -180.0), abs=1e-6)
    assert 0 < m.armed_remaining_s <= 120.0
    assert m.why_not_ready == ''


def test_arm_times_out(env):
    """명령이 없으면 잠금 시간 뒤 조작 허용이 저절로 꺼진다."""
    _link, bridge, client = env
    bridge.arm_timeout = 0.3
    arm(client)
    time.sleep(0.8)
    assert not bridge.armed


def test_calibration_service_remember_and_record(env):
    """보정: ① 없이 ② 는 실패, ① 은 가까운 검출 픽셀을 기억, ② 는 지금 TCP 와 짝지어 기록."""
    link, bridge, client = env
    bridge.px_objects, bridge.px_stamp = [px_obj('blue_cube', 'blue', 50, 60)], time.time()
    C = RealCalibration.Request
    r = call(client, RealCalibration, '/aris3/real/calibration', C(command=C.RECORD))
    assert not r.success                               # ① 먼저
    r = call(client, RealCalibration, '/aris3/real/calibration', C(command=C.REMEMBER, u=55.0, v=58.0))
    assert r.success and bridge.calib_px == (50.0, 60.0)
    r = call(client, RealCalibration, '/aris3/real/calibration', C(command=C.RECORD, label='blue'))
    assert r.success and len(bridge.calib.points) == 1
    assert bridge.calib.points[0]['x'] == link.posx[0]


def test_hand_guide_disarms(env):
    """손으로 움직이기를 켜면 조작 허용이 잠긴다."""
    link, bridge, client = env
    arm(client)
    r = call(client, SetBool, '/aris3/real/set_hand_guide', SetBool.Request(data=True))
    assert r.success and not bridge.armed and 'hand_guide True' in link.calls


def test_nearest_px_limit():
    """가장 가까운 검출을 고르고, 거리 제한을 넘으면 None."""
    objs = [px_obj('a', 'red', 0, 0), px_obj('b', 'blue', 100, 0)]
    assert nearest_px(objs, 90, 0).id == 'b'
    assert nearest_px(objs, 50, 50, max_dist=10) is None
    assert nearest_px([], 0, 0) is None
