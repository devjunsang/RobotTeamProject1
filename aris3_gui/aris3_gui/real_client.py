"""실물 모드: GUI 쪽에서 aris3_real_bridge 를 부르는 얇은 클라이언트 (2026-11-10).

예전에는 GUI 프로세스 안에서 RobotLink(로봇)·PixelCalibration(보정)·PickSequence(집기 순서)를 직접 돌렸다.
이제 그것들은 aris3_real_bridge 노드에 있고, GUI 는 /aris3/real/* 서비스·액션을 부르고 상태 토픽을 읽기만 한다.
GUI 를 꺼도 브리지는 계속 돌고, C++ GUI(ARIS)도 같은 브리지를 쓴다.

화면 코드가 예전과 거의 같은 모양으로 쓰도록 두 개의 「보기」 객체를 준다:
  calib  : StateCalibration - ready(), to_robot(u, v), grasp_z, summary()  (상태 토픽의 보정 행렬로 계산)
  picker : StatePicker      - running, holding                              (상태 토픽 + 보낸 뒤 결과 대기 중)

모든 호출은 비동기다: 결과는 GUI 로그에 남고, 상태는 다음 상태 토픽으로 화면에 반영된다.
"""
import threading
import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger

from aris3_interfaces.action import RealMotion, RealPickPlace
from aris3_interfaces.msg import RealRobotState
from aris3_interfaces.srv import RealCalibration

STATE_FRESH_S = 1.0       # 이보다 오래된 상태는 「브리지 없음」 으로 본다


class StateCalibration:
    """상태 토픽의 보정 정보로 픽셀 -> 로봇 좌표를 계산하는 보기 (예전 PixelCalibration 과 같은 이름의 함수)."""

    def __init__(self, client):
        """client: RealBridgeClient (마지막 상태를 읽는다)."""
        self.client = client

    def _st(self):
        """신선한 마지막 상태 또는 None."""
        return self.client.state()

    def ready(self):
        """보정 점이 4개 이상이라 변환을 쓸 수 있는가."""
        st = self._st()
        return bool(st and st.calib_ready)

    def to_robot(self, u, v):
        """픽셀 (u, v) -> 로봇 (x, y) mm, 보정이 없으면 None. 브리지와 같은 3x3 호모그래피 계산."""
        st = self._st()
        if not (st and st.calib_ready):
            return None
        h = list(st.calib_homography)
        x = h[0] * u + h[1] * v + h[2]
        y = h[3] * u + h[4] * v + h[5]
        w = h[6] * u + h[7] * v + h[8]
        if abs(w) < 1e-12:
            return None
        return x / w, y / w

    @property
    def grasp_z(self):
        """집는 높이 [mm] (기록한 z 의 중앙값), 보정이 없으면 None."""
        st = self._st()
        return float(st.calib_grasp_z_mm) if (st and st.calib_ready) else None

    @property
    def points(self):
        """기록한 보정 점 개수 (예전 코드가 목록 길이로 쓰던 곳에 맞춰 range 로 준다)."""
        st = self._st()
        return range(st.calib_points if st else 0)

    def summary(self):
        """사람이 읽는 보정 요약 (브리지가 만든 문장)."""
        st = self._st()
        return st.calib_summary if st else '보정 정보 없음 (실물 브리지 연결 전)'

    def pending_px(self):
        """보정 ① 에서 기억한 픽셀 (화면 표시용) 또는 None."""
        st = self._st()
        return (st.calib_pending_u, st.calib_pending_v) if (st and st.calib_pending) else None


class StatePicker:
    """집기 순서 상태 보기: 브리지가 실행 중이거나, 요청을 보내고 결과를 기다리는 중이면 running."""

    def __init__(self, client):
        """client: RealBridgeClient."""
        self.client = client

    @property
    def running(self):
        """집기·놓기 순서 진행 중 (보낸 요청의 결과를 아직 못 받았어도 진행 중으로 본다 - 두 번 보내지 않게)."""
        st = self.client.state()
        return self.client.pick_pending or bool(st and st.sequence_running)

    @property
    def holding(self):
        """마지막 닫기 결과: True 잡음 / False 빈손 / None 모름 (예전 PickSequence.holding 과 같은 뜻)."""
        st = self.client.state()
        if st is None or st.holding == RealRobotState.HOLD_UNKNOWN:
            return None
        return st.holding == RealRobotState.HOLD_YES


class RealBridgeClient:
    """aris3_real_bridge 의 서비스·액션·상태 토픽을 GUI 노드(GuiNode) 위에 만든다."""

    def __init__(self, node):
        """node: ros_io.GuiNode (로그 함수 node.log 와 executor 를 같이 쓴다)."""
        self.node = node
        cb = ReentrantCallbackGroup()
        p = '/aris3/real/'
        self._lock = threading.Lock()
        self._state, self._state_time = None, 0.0
        self.pick_pending = False       # pick_place 요청을 보냈고 결과를 아직 받지 못함
        self.motion_pending = False     # motion 요청을 보냈고 결과를 아직 받지 못함
        node.create_subscription(RealRobotState, p + 'state', self._on_state, 10, callback_group=cb)
        self.srv = {name: node.create_client(Trigger, p + name, callback_group=cb)
                    for name in ('connect', 'disconnect', 'stop')}
        self.srv_armed = node.create_client(SetBool, p + 'set_armed', callback_group=cb)
        self.srv_hand = node.create_client(SetBool, p + 'set_hand_guide', callback_group=cb)
        self.srv_calib = node.create_client(RealCalibration, p + 'calibration', callback_group=cb)
        self.motion_client = ActionClient(node, RealMotion, p + 'motion', callback_group=cb)
        self.pick_client = ActionClient(node, RealPickPlace, p + 'pick_place', callback_group=cb)
        self.calib = StateCalibration(self)
        self.picker = StatePicker(self)

    # ---------------------------------------------------------------- 상태
    def _on_state(self, m):
        """상태 토픽 수신: 마지막 한 장과 받은 시각을 보관."""
        with self._lock:
            self._state, self._state_time = m, time.monotonic()

    def state(self):
        """신선한(STATE_FRESH_S 안) 마지막 상태, 브리지가 없으면 None."""
        with self._lock:
            return self._state if time.monotonic() - self._state_time < STATE_FRESH_S else None

    # ---------------------------------------------------------------- 서비스
    def _call(self, client, req, what, log_ok=True):
        """서비스를 비동기로 부르고 결과 문장을 GUI 로그에 남긴다. 서비스가 없으면 False."""
        if not client.service_is_ready():
            self.node.log('ERROR', f'{what}: 실물 브리지 서비스 없음 (aris3_real_bridge 실행 확인)')
            return False
        fut = client.call_async(req)

        def done(f):
            """응답: 실패는 언제나, 성공은 log_ok 일 때만 로그."""
            r = f.result()
            if r is None:
                self.node.log('ERROR', f'{what}: 응답 없음')
            elif not r.success:
                self.node.log('WARN', f'{what}: {r.message}')
            elif log_ok:
                self.node.log('INFO', f'{what}: {r.message}')
        fut.add_done_callback(done)
        return True

    def trigger(self, name, what):
        """connect / disconnect / stop 서비스 호출."""
        return self._call(self.srv[name], Trigger.Request(), what)

    def stop(self):
        """정지: 결과를 기다리지 않고 바로 보낸다 (브리지는 별도 콜백 그룹으로 항상 받음)."""
        return self.trigger('stop', '정지')

    def set_armed(self, on):
        """브리지 쪽 조작 허용 켜기/끄기 (GUI 「조작 허용」 체크와 함께)."""
        return self._call(self.srv_armed, SetBool.Request(data=bool(on)), '조작 허용', log_ok=False)

    def set_hand_guide(self, on):
        """손으로 움직이기(수동 모드) 켜기/끄기."""
        return self._call(self.srv_hand, SetBool.Request(data=bool(on)), '손으로 움직이기')

    def calibration(self, command, u=0.0, v=0.0, label=''):
        """카메라 보정 명령 (RealCalibration.Request.REMEMBER / RECORD / ...)."""
        req = RealCalibration.Request(command=command, u=float(u), v=float(v), label=label or '')
        return self._call(self.srv_calib, req, '보정')

    # ---------------------------------------------------------------- 액션
    def _send(self, client, goal, what, pending_attr):
        """액션 목표를 보내고, 끝나면 결과 문장을 로그에 남긴다. pending_attr 플래그로 「보내고 기다리는 중」 표시."""
        if not client.server_is_ready():
            self.node.log('ERROR', f'{what}: 실물 브리지 액션 없음 (aris3_real_bridge 실행 확인)')
            return False
        if getattr(self, pending_attr):
            self.node.log('WARN', f'{what}: 이전 요청의 결과를 기다리는 중')
            return False
        setattr(self, pending_attr, True)
        fut = client.send_goal_async(goal)

        def accepted(f):
            """브리지가 받았는지 확인하고 결과를 기다린다."""
            h = f.result()
            if h is None or not h.accepted:
                setattr(self, pending_attr, False)
                self.node.log('WARN', f'{what}: 브리지가 거부 (이전 동작이 진행 중)')
                return
            h.get_result_async().add_done_callback(finished)

        def finished(f):
            """결과: 성공·실패·취소를 로그에 남기고 대기 표시를 푼다."""
            setattr(self, pending_attr, False)
            res = f.result()
            r = res.result
            if res.status == GoalStatus.STATUS_SUCCEEDED and r.success:
                self.node.log('INFO', f'{what}: {r.message or "완료"}')
            elif res.status == GoalStatus.STATUS_CANCELED:
                self.node.log('WARN', f'{what}: 취소됨 (정지)')
            else:
                self.node.log('WARN', f'{what}: {r.message or "실패"}')
        fut.add_done_callback(accepted)
        return True

    def motion(self, kind, what, index=0, delta=0.0, speed_percent=30.0):
        """조그·홈·복구·그리퍼 (RealMotion.Goal.JOG_JOINT ...)."""
        goal = RealMotion.Goal(kind=kind, index=int(index), delta=float(delta), speed_percent=float(speed_percent))
        return self._send(self.motion_client, goal, what, 'motion_pending')

    def pick_place(self, kind, what, target_name='', target_px=None, place_px=None):
        """집기·놓기 (RealPickPlace.Goal.PICK ...). 대상·놓을 자리는 영상 픽셀 (브리지가 직전에 다시 고침)."""
        goal = RealPickPlace.Goal(kind=kind, target_name=target_name or '')
        if target_px is not None:
            goal.target_u, goal.target_v = float(target_px[0]), float(target_px[1])
        if place_px is not None:
            goal.place_u, goal.place_v = float(place_px[0]), float(place_px[1])
        return self._send(self.pick_client, goal, what, 'pick_pending')
