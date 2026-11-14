"""aris3_real_bridge - 실물 E0509 를 ROS 인터페이스로 내놓는 노드 (2026-11-10).

GUI(aris3_gui PyQt, aris3_gui_cpp ARIS)는 로봇에 직접 붙지 않고 이 노드만 부른다.
로봇 쪽 코드는 새로 짜지 않고 그대로 쓴다:
  e0509_manual.robot_link.RobotLink   이동 전 안전 검사·도착 확인·정지·그리퍼 DRL·복구 (실물 검증됨)
  e0509_manual.bringup                dsr_bringup2 접속·해제
  aris3_real_bridge.real_pick         카메라 보정(PixelCalibration)·집기 순서(PickSequence)

  토픽      /aris3/real/state            RealRobotState 10 Hz
  서비스    /aris3/real/connect|disconnect|stop   std_srvs/Trigger   (stop 은 별도 콜백 그룹: 항상 응답)
            /aris3/real/set_armed|set_hand_guide  std_srvs/SetBool
            /aris3/real/calibration               RealCalibration
  액션      /aris3/real/motion           RealMotion     (취소 = 정지)
            /aris3/real/pick_place       RealPickPlace  (취소 = 정지)
  구독      /aris3/vision/detections_px  색 검출 픽셀 (집기 직전에 큐브 위치를 다시 읽음, 보정 ① 에 사용)

조작 허용(armed): GUI 의 「조작 허용」 과 별개로 이 노드도 막는다. armed 가 아니면 이동·그리퍼·집기 요청을
거부하고, arm_timeout_s 동안 명령이 없으면 스스로 잠근다. 정지는 armed 와 상관없이 언제나 동작한다.

그리퍼 각도 맞추기 (2026-11-13, 파라미터 align_yaw, 기본 켬): 색 검출이 pose.orientation 에 담은 큐브 면 방향(영상)을
카메라 보정으로 로봇 각도로 바꿔, 집을 때는 그 각도로, 놓을 때는 팔 방향(베이스 -> 놓을 자리)과 평행·수직으로,
쌓을 때는 받침 큐브와 같은 각도로 J6 를 돌린다 (real_pick.PickSequence._align).

쌓인 큐브 위치 (2026-11-13): 로봇 좌표는 검출의 grasp_u, grasp_v (원근 보정 픽셀) 로 계산한다 - 0 이면 pose.position.
큐브 중심 맞추기 (2026-11-13, 파라미터 center_grasp, 기본 켬): 접근 높이에 멈춘 뒤 새로 찍힌 검출 여러 장의 중앙값으로
큐브 중심을 다시 재서 그리퍼를 그 바로 위로 옮기고 내려간다 (PickSequence._center). 넓이가 처음과 크게 다르면
(그리퍼에 일부 가려짐) 그 장은 쓰지 않는다.
"""
import math
import os
import shutil
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import SetBool, Trigger

from aris3_interfaces.action import RealMotion, RealPickPlace
from aris3_interfaces.msg import DetectedObjectArray, RealRobotState
from aris3_interfaces.srv import RealCalibration
from aris3_real_bridge.real_pick import (CALIB_PATH, MAX_LEVEL, STACK_MM, STACK_SIM_MM, PickSequence, PixelCalibration,
                                         radial_yaw_deg, wrap90)
from aris3_real_bridge import marker_calib as MC
from aris3_real_bridge.virtual import ServiceGripper, make_virtual_bringup

# 가상 모드(시뮬레이션) 보정 파일 - 실물 보정(CALIB_PATH)과 따로 둔다
SIM_CALIB_PATH = os.path.expanduser('~/.config/aris3/sim_camera_calib.json')

MAX_JOINT_VEL = 45.0          # deg/s - 속도 비율 100 % (RobotLink 도 같은 값으로 자른다)
MAX_CART_VEL = 100.0          # mm/s
GRIP_CLOSE_POS, GRIP_CURRENT_MA, GRIP_VEL = 740, 300, 1500
REFRESH_PX = 40.0             # 집기 직전: 받은 픽셀에서 이 거리 안의 검출로 큐브 위치를 고친다
PX_FRESH_S = 1.0              # 이보다 오래된 검출은 쓰지 않는다 (보정 ① 은 2 s)
MARKER_WAIT_S = 1.5           # 마커 보정 ①: 새 영상을 기다리는 최대 시간
MARKER_FRAMES = 5             # 마커 보정 ①: 모으는 장 수 (중앙값)
MARKER_BLOCK_TOL_MM = 5.0     # 마커 블록 높이가 큐브와 이 안으로 같으면 높이를 고치지 않는다
CENTER_SETTLE_S = 0.3        # 중심 맞추기: 멈춘 뒤 이만큼 지난 다음에 받은 검출만 (영상·검출 지연)
CENTER_WAIT_S = 2.0           # 중심 맞추기: 검출을 기다리는 최대 시간
CENTER_SAMPLES = 3            # 중심 맞추기: 이만큼 모아 중앙값 (한 장 흔들림 줄이기)
CENTER_AREA_TOL = 0.3         # 중심 맞추기: 넓이가 처음보다 이 비율 넘게 다르면(일부 가려짐) 그 장은 버림
STACK_MIN_SEP_MM = 30.0       # 쌓기: 집을 큐브와 받침 큐브가 이보다 가까우면 같은 큐브로 보고 거부 (큐브 40 mm)
STANDBY, MOVING = 1, 2
# launch 에서 z_min_mm:=77 처럼 정수를 줘도 받도록 (기본은 타입 고정이라 노드가 죽었다), 읽을 때 float()/int()
ANY_TYPE = ParameterDescriptor(dynamic_typing=True)


def grasp_px(o):
    """검출 o 의 집기용 픽셀: 원근 보정 픽셀(grasp_u, grasp_v), 채우지 않았으면(0, 0) 화면 위치 pose.position."""
    if o.grasp_u or o.grasp_v:
        return float(o.grasp_u), float(o.grasp_v)
    return float(o.pose.pose.position.x), float(o.pose.pose.position.y)


def px_yaw_deg(o):
    """검출의 pose.orientation (영상 픽셀 좌표계 z 축 회전) -> 큐브 면 방향 [deg] (색 검출 스크립트가 담음)."""
    q = o.pose.pose.orientation
    return math.degrees(2.0 * math.atan2(q.z, q.w))


def nearest_px(objects, u, v, max_dist=None):
    """검출 목록에서 (u, v) 에 가장 가까운 것. max_dist 를 넘으면 None."""
    best = min(objects, key=lambda o: math.hypot(o.pose.pose.position.x - u, o.pose.pose.position.y - v),
               default=None)
    if best is None:
        return None
    if max_dist is not None and math.hypot(best.pose.pose.position.x - u, best.pose.pose.position.y - v) > max_dist:
        return None
    return best


def joint_vel(pct):
    """속도 비율(%) -> 관절 속도 deg/s. 너무 느려 멈춘 것처럼 보이지 않게 최소 5 deg/s."""
    return max(5.0, MAX_JOINT_VEL * float(pct) / 100.0)


def cart_vel(pct):
    """속도 비율(%) -> 직선 속도 mm/s. 최소 10 mm/s."""
    return max(10.0, MAX_CART_VEL * float(pct) / 100.0)


def hold_code(holding, cls=RealRobotState):
    """PickSequence.holding(True/False/None) -> 메시지의 HOLD_YES / HOLD_NO / HOLD_UNKNOWN 숫자."""
    return cls.HOLD_UNKNOWN if holding is None else (cls.HOLD_YES if holding else cls.HOLD_NO)


class RealBridge(Node):
    """실물 로봇 브리지 노드. RobotLink·보정·집기 순서를 감싸 /aris3/real/* 서비스·액션·상태 토픽으로 내놓는다."""
    def __init__(self, link, mgr, conn, state_ko=None, calib_path=None, gripper_ready=None):
        """link: RobotLink, mgr: BringupManager, conn: ConnectionController, state_ko: 상태 이름 -> 한국어 표.

        시험에서는 가짜 객체를 넣어 로봇 없이 돌린다 (test/test_bridge_node.py).
        """
        super().__init__('aris3_real_bridge')
        self.declare_parameter('arm_timeout_s', 120.0, ANY_TYPE)
        self.declare_parameter('state_rate_hz', 10.0, ANY_TYPE)
        # 그리퍼 각도 맞추기 (집기: 큐브 면, 놓기: 팔 방향, 쌓기: 받침 큐브). false = 예전처럼 지금 각도 그대로
        self.declare_parameter('align_yaw', True, ANY_TYPE)
        # 큐브 중심 맞추기 (접근 높이에서 큐브를 다시 재서 그 바로 위로). false = 처음 좌표 그대로
        self.declare_parameter('center_grasp', True, ANY_TYPE)
        # 탁자 블록 마커 보정 (2026-11-14): 블록 윗면에 붙인 ArUco 4X4_50 마커 ID (보정 ① 이 마커 중심을 씀)
        self.declare_parameter('marker_id', 0, ANY_TYPE)
        self.link, self.mgr, self.conn = link, mgr, conn
        self.state_ko = state_ko or {}
        self.arm_timeout = float(self.get_parameter('arm_timeout_s').value)
        # 'false' 문자열로 와도 끄기 (bool('false') 는 True 라서)
        self.align_yaw = str(self.get_parameter('align_yaw').value).strip().lower() in ('true', '1', 'yes')
        self.center_grasp = str(self.get_parameter('center_grasp').value).strip().lower() in ('true', '1', 'yes')

        # 보정 파일: 실물과 가상 모드는 카메라가 달라 파일을 나눈다 (가상 시험이 실물 보정을 덮어쓰지 않게)
        self.calib_path = calib_path or CALIB_PATH
        # 그리퍼가 명령을 받을 수 있는가: 실물 = DRL 서비스, 가상 = /aris3/gripper/command (main 에서 바꿔 줌)
        self.gripper_ready = gripper_ready or (lambda: link.cli_drl_start.service_is_ready())
        self.calib = PixelCalibration()
        self.picker = PickSequence(link, lambda s: self.get_logger().info(f'[집기] {s}'))
        self.calib_px = None                # 보정 ①: (u, v)
        try:
            self.calib.load(self.calib_path)
            self.get_logger().info(f'카메라 보정 불러옴 ({self.calib_path}): {self.calib.summary()}')
        except (OSError, ValueError, KeyError):
            self.get_logger().info('카메라 보정 파일 없음 (보정 ① ② 로 점 4개 이상 기록)')

        self._lock = threading.Lock()
        self.armed = False
        self.last_cmd = 0.0
        self.active_goal = False            # 이동 액션 하나만 (motion, pick_place 공통)
        self.px_objects, self.px_stamp = [], 0.0
        # 마커 보정 ① 용 카메라 (마지막 장만 두고 쓸 때 푼다 - 30 Hz 를 늘 풀지 않게)
        self.cam_img, self.cam_img_t = None, 0.0
        self.cam_depth, self.cam_depth_t = None, 0.0
        self.cam_K = None
        self.calib_mark = None              # 보정 ① 이 마커였으면 {'cam': 3D 점, 'block_mm': 블록 높이} (② 에서 씀)
        # 블록 자동 보정점 (2026-11-14, AUTO_RECORD): 지금 보정과 따로 모으고 「보정 저장」 때 새 보정으로 바꾼다
        self.auto_calib = PixelCalibration()
        self.auto_grasp = None              # 진행 중인 자동 보정점의 블록 중심 TCP (x, y, z)

        cb = ReentrantCallbackGroup()
        cb_stop = MutuallyExclusiveCallbackGroup()   # 다른 콜백이 모두 바빠도 정지는 따로 처리
        p = '/aris3/real/'
        self.pub_state = self.create_publisher(RealRobotState, p + 'state', 10)
        self.create_subscription(DetectedObjectArray, '/aris3/vision/detections_px', self._on_px, 10,
                                 callback_group=cb)
        # 위쪽 카메라 (scripts/aris3_realsense_pub.py) - 보정 ① 마커를 잴 때만 푼다
        # 영상·깊이는 카메라 노드가 BEST_EFFORT 로 보낸다 -> 센서용 QoS 로 받아야 연결된다 (RELIABLE 구독은 한 장도 못 받음)
        cam = '/aris3/overhead_camera/'
        self.create_subscription(CompressedImage, cam + 'image/compressed', self._on_cam_img, qos_profile_sensor_data,
                                 callback_group=cb)
        self.create_subscription(CompressedImage, cam + 'depth/compressed', self._on_cam_depth, qos_profile_sensor_data,
                                 callback_group=cb)
        self.create_subscription(CameraInfo, cam + 'camera_info', self._on_cam_info, 2, callback_group=cb)
        self.create_service(Trigger, p + 'stop', self._srv_stop, callback_group=cb_stop)
        self.create_service(Trigger, p + 'connect', self._srv_connect, callback_group=cb)
        self.create_service(Trigger, p + 'disconnect', self._srv_disconnect, callback_group=cb)
        self.create_service(SetBool, p + 'set_armed', self._srv_set_armed, callback_group=cb)
        self.create_service(SetBool, p + 'set_hand_guide', self._srv_set_hand_guide, callback_group=cb)
        self.create_service(RealCalibration, p + 'calibration', self._srv_calibration, callback_group=cb)
        self.motion_server = ActionServer(
            self, RealMotion, p + 'motion', self._exec_guard(self._exec_motion, RealMotion.Result), goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb, callback_group=cb)
        self.pick_server = ActionServer(
            self, RealPickPlace, p + 'pick_place', self._exec_guard(self._exec_pick, RealPickPlace.Result), goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb, callback_group=cb)
        rate = float(self.get_parameter('state_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick, callback_group=cb)
        self.get_logger().info('실물 브리지 시작 (조작 허용 꺼짐)')

    # ================================================================== 공통 검사
    def _touch(self):
        self.last_cmd = time.time()

    def why_blocked(self, need_ready=True):
        """이동 명령을 지금 받을 수 없는 이유, 없으면 None."""
        if not self.armed:
            return '「조작 허용」 이 꺼져 있음'
        if self.conn.busy():
            return self.mgr.describe()
        if self.picker.running:
            return '집기/놓기 순서가 진행 중'
        if need_ready:
            return self.link.why_not_ready()
        return None

    def _last_link_log(self, default):
        """RobotLink 가 방금 남긴 거부 이유 (시각 제거). RobotLink 는 거부할 때 로그만 남기고 False 를 돌려준다."""
        lines = list(self.link.log_lines)
        return lines[-1][9:] if lines else default

    def _on_px(self, msg):
        """색 검출(영상 픽셀) 수신: 마지막 목록과 받은 시각을 저장 (집기 직전·보정 ① 에서 다시 읽음)."""
        self.px_objects, self.px_stamp = list(msg.objects), time.time()

    def _on_cam_img(self, msg):
        """컬러 영상 수신: 마지막 장과 받은 시각만 둔다."""
        self.cam_img, self.cam_img_t = msg, time.time()

    def _on_cam_depth(self, msg):
        """깊이 영상 수신: 마지막 장과 받은 시각만 둔다."""
        self.cam_depth, self.cam_depth_t = msg, time.time()

    def _on_cam_info(self, msg):
        """카메라 내부 파라미터 (fx, fy, cx, cy)."""
        self.cam_K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    # ================================================================== 탁자 블록 마커 보정 (2026-11-14)
    def _measure_marker(self, near=None):
        """보정 ①: 새 영상 여러 장에서 블록 마커를 잰다. 돌려줌 dict(px=(u, v) 큐브 높이 픽셀, raw=마커 중심 픽셀,
        cam=마커 3D 점 또는 None, block_mm=블록 높이 또는 None, frames, side_px) 또는 못 찾은 이유(str).

        near (u, v): 마커가 여럿이면 그 픽셀에 가장 가까운 것 (없으면 가장 큰 것).
        블록 높이가 큐브(stack_mm)와 MARKER_BLOCK_TOL_MM 넘게 다르면 픽셀을 큐브 높이로 옮긴다 (to_cube_height).
        """
        if self.cam_img is None or time.time() - self.cam_img_t > 1.0:
            return '위쪽 카메라 영상이 오지 않음'
        mid = int(self.get_parameter('marker_id').value)
        t0 = time.time()
        raws, cams, blocks, angs, last, side = [], [], [], [], 0.0, 0.0
        while time.time() - t0 < MARKER_WAIT_S and len(raws) < MARKER_FRAMES:
            img, img_t = self.cam_img, self.cam_img_t
            if img is None or img_t == last:
                time.sleep(0.02)
                continue
            last = img_t
            gray = cv2.imdecode(np.frombuffer(bytes(img.data), np.uint8), cv2.IMREAD_GRAYSCALE)
            m = MC.pick_marker(MC.find_markers(gray, mid), near) if gray is not None else None
            if m is None:
                continue
            raws.append(m[0])
            angs.append(MC.marker_angle(m[1]))
            side = m[2]
            dd = MC.decode_depth(self.cam_depth) if self.cam_depth is not None else None
            if dd is None or self.cam_K is None or time.time() - self.cam_depth_t > 1.0:
                continue
            pc = MC.marker_cam_point(dd[0], dd[1], m[1], m[0], self.cam_K)
            if pc is None:
                continue
            cams.append(pc)
            r = MC.to_cube_height(pc, MC.table_around(dd[0], dd[1], m[0], m[2], self.cam_K), self.picker.stack_mm,
                                  self.cam_K)
            if r is not None:
                blocks.append(r)
        if not raws:
            return f'카메라에 마커(4X4_50, ID {mid})가 보이지 않음'
        raw = tuple(float(v) for v in np.median(np.array(raws), axis=0))
        out = {'raw': raw, 'px': raw, 'cam': None, 'block_mm': None, 'frames': len(raws), 'side_px': side,
               'ang': MC.mean_angle90(angs)}
        if cams:
            out['cam'] = np.median(np.array(cams), axis=0)
        if blocks:
            hb = float(np.median([b[0] for b in blocks]))
            out['block_mm'] = hb
            if abs(hb - self.picker.stack_mm) > MARKER_BLOCK_TOL_MM:
                out['px'] = tuple(float(v) for v in np.median(np.array([b[1] for b in blocks]), axis=0))
        return out

    def _auto_start(self):
        """AUTO_RECORD: 블록 마커를 재서 지금 보정으로 위치·각도 계산 -> 잡아서 가운데 맞추기 순서 시작. (성공, 문장)."""
        why = self.why_blocked()
        if why:
            return False, f'블록 자동 기록 거부: {why}'
        if not self.calib.ready():
            return False, '블록 자동 기록 거부: 지금 보정이 없음 (블록을 찾아갈 때 씀 - 손으로 4 점 먼저)'
        if self.picker.holding:
            return False, '블록 자동 기록 거부: 그리퍼가 무언가를 잡고 있음 (먼저 내려놓기)'
        mk = self._measure_marker()
        if not isinstance(mk, dict):
            return False, f'블록 자동 기록 거부: {mk}'
        u, v = mk['px']
        x, y = self.calib.to_robot(u, v)
        if not 150.0 <= math.hypot(x, y) <= 750.0:
            return False, f'블록 자동 기록 거부: 블록이 로봇 도달 범위 밖 ({x:.0f}, {y:.0f}) mm'
        yaw = self.calib.angle_to_robot(u, v, mk['ang'])
        self.auto_grasp = None

        def record():
            """두 번 닫아 가운데로 맞춘 그때의 TCP = 블록 중심."""
            p = self.link.posx
            if p is None:
                self.picker.log('블록 중심 기록 실패: TCP 를 읽지 못함')
                return False
            self.auto_grasp = (float(p[0]), float(p[1]), float(p[2]))
            self.picker.log(f'블록 중심 TCP ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm '
                            f'(처음 계산 ({x:.1f}, {y:.1f}) 에서 {math.hypot(p[0] - x, p[1] - y):.1f} mm)')
            return True

        def measure():
            """홈에서 마커를 다시 재서 (블록은 가운데 맞추며 조금 밀렸을 수 있음) 자동 보정점 추가."""
            m2 = self._measure_marker()
            if not isinstance(m2, dict) or self.auto_grasp is None:
                self.picker.log(f'블록 자동 기록 실패: 마커를 다시 재지 못함 ({m2}) - 블록이 가려졌는지 확인')
                return False
            gx, gy, gz = self.auto_grasp
            a = self.auto_calib
            a.add(m2['px'][0], m2['px'][1], gx, gy, gz, label='auto', cam=m2['cam'])
            self.picker.log(f'블록 자동 보정점 {len(a.points)}: 픽셀 ({m2["px"][0]:.1f}, {m2["px"][1]:.1f}) -> '
                            f'TCP ({gx:.1f}, {gy:.1f}, {gz:.1f}) mm  |  {self._auto_summary()}')
            return True
        if not self.picker.block_calibration_point(x, y, self.calib.grasp_z, yaw, record, measure):
            return False, '블록 자동 기록 거부: 다른 순서가 진행 중'
        self._touch()
        return True, (f'블록 자동 기록 시작: 마커 ({mk["raw"][0]:.1f}, {mk["raw"][1]:.1f}) -> 지금 보정으로 '
                      f'({x:.0f}, {y:.0f}) mm, 각도 {yaw:.0f}°. 잡아서 가운데 맞춘 뒤 홈에서 마커를 다시 잽니다')

    def _auto_summary(self):
        """자동 보정점 모음 요약 (저장 전)."""
        a = self.auto_calib
        n = len(a.points)
        if n == 0:
            return ''
        if not a.ready():
            return f'자동 보정점 {n}개 (4개 이상이면 「보정 저장」 으로 새 보정)'
        return f'자동 보정점 (저장 전): {a.summary()}'

    def _fresh_px(self, max_age):
        """max_age 초 안에 받은 검출만 돌려준다. 검출 노드가 꺼졌으면 빈 목록 (오래된 위치로 움직이지 않게)."""
        return self.px_objects if time.time() - self.px_stamp <= max_age else []

    # ================================================================== 서비스
    def _srv_stop(self, _req, res):
        sent = self.link.cli_stop.service_is_ready()
        self.picker.request_stop()          # 끊긴 집기 순서를 「계속해」 로 이을 수 있게 표시
        self.link.stop()                    # 연결이 없어도 부른다 (RobotLink 가 로그를 남김)
        self.get_logger().warn('정지 요청 받음' + ('' if sent else ' - 로봇 연결 없음, 보내지 못함'))
        res.success = bool(sent)
        res.message = '정지 명령 보냄' if sent else '정지 명령을 보낼 수 없음: 로봇에 연결되어 있지 않음'
        return res

    def _srv_connect(self, _req, res):
        """/aris3/real/connect: dsr_bringup2 접속 시작 (오래 걸리므로 백그라운드에서, 결과는 상태 토픽으로)."""
        if self.link.joints_fresh():
            res.success, res.message = True, '이미 접속됨'
        else:
            self.picker.forget_resume('새로 접속')
            res.success = bool(self.conn.connect_async())
            res.message = '접속 시작 (서보가 켜지고 펜던트 제어권을 가져옴, 팔은 움직이지 않음)' if res.success \
                else '이미 접속/끊기 작업 중'
        return res

    def _srv_disconnect(self, _req, res):
        """/aris3/real/disconnect: 접속 끊기 (서보 꺼짐). 먼저 조작 허용을 잠근다."""
        self.armed = False
        self.picker.forget_resume('접속 끊기')
        res.success = bool(self.conn.disconnect_async())
        res.message = '접속 끊기 시작 (서보 꺼짐, 브레이크가 팔을 잡음)' if res.success else '이미 접속/끊기 작업 중'
        return res

    def _srv_set_armed(self, req, res):
        """/aris3/real/set_armed: 조작 허용 켜기/끄기. 켜면 잠금 시계를 지금부터 다시 센다."""
        with self._lock:
            self.armed = bool(req.data)
            self._touch()
        self.get_logger().info('조작 허용 켬' if self.armed else '조작 잠금')
        res.success, res.message = True, ('조작 허용' if self.armed else '조작 잠금')
        return res

    def _srv_set_hand_guide(self, req, res):
        """/aris3/real/set_hand_guide: 손으로 움직이기(수동 모드) 켜기/끄기.

        집기 순서 중에는 켜지 않는다. 켜든 끄든 조작 허용은 잠근다 (예전 GUI 와 같은 동작).
        """
        on = bool(req.data)
        if on and self.picker.running:
            res.success, res.message = False, '집기/놓기 순서가 진행 중 - 끝난 뒤에 켜세요'
            return res
        self.armed = False                  # 손으로 움직이는 동안 GUI 명령은 잠근다 (예전 GUI 와 같음)
        self.picker.forget_resume('손으로 움직이기')
        res.success = bool(self.link.set_hand_guide(on))
        res.message = ('모드 전환 시작' if res.success else self._last_link_log('모드 전환 거부'))
        return res

    def _srv_calibration(self, req, res):
        """/aris3/real/calibration: 카메라 보정 명령 (① 기억, ② 기록, 삭제, 저장, 불러오기).

        결과 문장을 로그와 응답으로 돌려주고, 점 개수·오차·행렬은 상태 토픽으로 나간다.
        """
        C = RealCalibration.Request
        c = self.calib
        ok, msg = True, ''
        if req.command == C.REMEMBER:
            # 탁자 블록 마커가 보이면 마커 중심 (2026-11-14), 아니면 예전처럼 (u, v) 가까운 색 큐브
            mk = self._measure_marker((req.u, req.v) if (req.u or req.v) else None)
            # 큐브로 기억할 때는 고른 픽셀이 있어야 하고 그 가까이(60 px) 큐브만 (아무 큐브나 고르지 않게)
            given = bool(req.u or req.v)
            near = (None if isinstance(mk, dict) or not given
                    else nearest_px(self._fresh_px(2.0), req.u, req.v, 60.0))
            if isinstance(mk, dict):
                self.calib_px, self.calib_mark = mk['px'], mk
                hb = mk['block_mm']
                msg = (f'보정 ①: 마커 픽셀 ({mk["raw"][0]:.1f}, {mk["raw"][1]:.1f}) 기억 ({mk["frames"]}장, 한 변 '
                       f'{mk["side_px"]:.0f} px' + (f', 블록 높이 {hb:.0f} mm' if hb is not None else ', 깊이 없음') + ')')
                if mk['px'] != mk['raw']:
                    msg += (f' -> 큐브 높이({self.picker.stack_mm:.0f} mm) 픽셀 ({mk["px"][0]:.1f}, {mk["px"][1]:.1f})'
                            f' 로 고침')
                msg += ' → 조그로 그리퍼를 그 블록을 집는 자세까지 내린 뒤 ②'
            elif near is None:
                ok, msg = False, (f'보정 ①: {mk}' + (' - 고른 자리 가까이 큐브도 보이지 않음' if given else
                                                    ' (큐브로 하려면 카메라 화면에서 큐브를 먼저 클릭)'))
            else:
                self.calib_px, self.calib_mark = (near.pose.pose.position.x, near.pose.pose.position.y), None
                msg = (f'보정 ①: {near.id or near.color} 픽셀 ({self.calib_px[0]:.1f}, {self.calib_px[1]:.1f}) 기억 '
                       '→ 조그로 그리퍼를 그 큐브를 집는 자세까지 내린 뒤 ②')
        elif req.command == C.RECORD:
            p = self.link.posx
            if self.calib_px is None:
                ok, msg = False, '보정 ②: 먼저 ① 로 큐브 픽셀을 기억하세요'
            elif p is None or not self.link.joints_fresh():
                ok, msg = False, '보정 ②: 로봇 TCP 좌표를 읽지 못함 (접속 확인)'
            else:
                mk, z, note = self.calib_mark, p[2], ''
                hb = mk['block_mm'] if mk else None
                if hb is not None and abs(hb - self.picker.stack_mm) > MARKER_BLOCK_TOL_MM:
                    z = p[2] - (mk['block_mm'] - self.picker.stack_mm)       # 큐브 높이에서 집는 TCP z
                    note = f' (블록 {mk["block_mm"]:.0f} mm -> 큐브 높이로 Z {z:.1f})'
                cam = mk['cam'] if mk else None
                c.add(self.calib_px[0], self.calib_px[1], p[0], p[1], z,
                      label=req.label or ('marker' if mk else ''), cam=cam)
                msg = (f'보정 ②: 점 {len(c.points)} 픽셀 ({self.calib_px[0]:.1f}, {self.calib_px[1]:.1f}) '
                       f'-> TCP ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm{note}  |  {c.summary()}')
                self.calib_px, self.calib_mark = None, None
        elif req.command == C.AUTO_RECORD:
            ok, msg = self._auto_start()
        elif req.command == C.REMOVE_LAST:
            if self.auto_calib.points:                  # 자동 보정점을 모으는 중이면 그쪽에서 지움
                self.auto_calib.remove_last()
                msg = f'마지막 자동 보정점 삭제  |  {self._auto_summary() or "자동 보정점 0개"}'
            else:
                c.remove_last()
                msg = f'마지막 점 삭제  |  {c.summary()}'
        elif req.command == C.CLEAR:
            c.clear()
            self.auto_calib.clear()
            self.calib_px, self.calib_mark = None, None
            msg = '보정 점 모두 삭제'
        elif req.command == C.SAVE and self.auto_calib.points:
            # 자동 보정점이 있으면 그것으로 새 보정 (예전 파일은 백업)
            a = self.auto_calib
            if not a.ready():
                ok, msg = False, f'보정 저장 거부: 자동 보정점 {len(a.points)}개 (4개 이상 필요)'
            else:
                bak = ''
                if os.path.exists(self.calib_path):
                    bak = self.calib_path + time.strftime('.bak-%Y%m%d-%H%M%S')
                    shutil.copy2(self.calib_path, bak)
                a.save(self.calib_path)
                c.load(self.calib_path)
                self.auto_calib = PixelCalibration()
                msg = (f'보정 저장 (블록 자동 보정점으로 새 보정): {self.calib_path}' +
                       (f', 예전 파일 {bak}' if bak else '') + f'  |  {c.summary()}')
        elif req.command == C.SAVE:
            if not c.points:
                ok, msg = False, '보정 저장: 기록한 점이 없음'
            else:
                msg = f'보정 저장: {c.save(self.calib_path)}'
        elif req.command == C.LOAD:
            try:
                c.load(self.calib_path)
                msg = f'보정 불러옴: {c.summary()}'
            except (OSError, ValueError, KeyError) as e:
                ok, msg = False, f'보정 불러오기 실패: {e}'
        else:
            ok, msg = False, f'모르는 명령 {req.command}'
        if ok and c.ready() and req.command in (C.RECORD, C.REMOVE_LAST, C.LOAD):
            pts = '  '.join(f'{i + 1}:{e:.1f}' for i, e in enumerate(c.errors_mm))
            msg += f'  |  점별 오차 mm {pts}'
        if ok:      # rclpy: 한 호출 위치에서 수준을 바꿔 쓰면 ValueError -> 두 줄로 나눈다
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)
        res.success, res.message = ok, msg
        return res

    # ================================================================== 액션 공통
    def _goal_cb(self, _goal):
        with self._lock:
            if self.active_goal:
                self.get_logger().warn('거부: 이전 동작이 진행 중')
                return GoalResponse.REJECT
            self.active_goal = True
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        """액션 취소 요청: 바로 로봇 정지를 보내고 받아들인다 (실행 콜백이 이어서 취소로 끝낸다)."""
        self.picker.request_stop()
        self.link.stop()
        return CancelResponse.ACCEPT

    def _wait(self, goal_handle, busy, feedback=None, settle=0.3):
        """busy() 가 settle 초 동안 계속 False 일 때까지 기다린다. 취소 요청이 오면 정지를 보낸다."""
        time.sleep(0.1)
        idle_since, stop_sent, last_fb = None, False, 0.0
        while True:
            if goal_handle.is_cancel_requested and not stop_sent:
                self.picker.request_stop()
                self.link.stop()
                stop_sent = True
            if busy():
                idle_since = None
            else:
                idle_since = idle_since or time.time()
                if time.time() - idle_since >= settle:
                    return
            if feedback and time.time() - last_fb > 0.2:
                feedback()
                last_fb = time.time()
            time.sleep(0.05)

    def _exec_guard(self, fn, result_cls):
        """실행 콜백이 어떻게 끝나든(예외 포함) 다음 목표를 받을 수 있게 active_goal 을 푼다."""
        def run(goal_handle):
            """실제로 액션 서버가 부르는 함수: fn 을 실행하고, 예외가 나도 로봇을 멈추고 실패 결과를 돌려준다."""
            try:
                return fn(goal_handle)
            except Exception as e:      # noqa: BLE001 - 로봇 정지 후 실패로 끝낸다
                self.link.stop()
                self.get_logger().error(f'동작 중 예외 -> 정지: {e!r}')
                goal_handle.abort()
                res = result_cls()
                res.success, res.message = False, f'내부 오류로 정지: {e!r}'
                return res
            finally:
                with self._lock:
                    self.active_goal = False
        return run

    def _finish(self, goal_handle, result):
        """액션 끝내기: 취소 요청이 있었으면 canceled, 성공이면 succeed, 아니면 abort 로 결과를 돌려준다."""
        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = result.message or '취소됨 (정지)'
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ================================================================== 액션: 이동·그리퍼
    def _exec_motion(self, goal_handle):
        g = goal_handle.request
        G = RealMotion.Goal
        res = RealMotion.Result()
        names = {G.JOG_JOINT: f'J{g.index + 1} {g.delta:+.1f}°', G.JOG_TCP: f'{"XYZ"[min(g.index, 2)]} {g.delta:+.1f} mm',
                 G.HOME: '홈 이동', G.RECOVER_HOME: '복구 후 홈', G.GRIPPER_OPEN: '그리퍼 열기',
                 G.GRIPPER_CLOSE: '그리퍼 닫기', G.STRETCH: '가동 자세 (일자로 펴기)'}
        what = names.get(g.kind)
        if what is None:
            res.success, res.message = False, f'모르는 동작 {g.kind}'
            return self._finish(goal_handle, res)
        # 복구는 로봇이 대기가 아닐 때 쓰는 것이라 why_not_ready 를 보지 않는다 (RobotLink 가 자체 검사)
        why = self.why_blocked(need_ready=g.kind != G.RECOVER_HOME)
        if why:
            res.success, res.message = False, f'{what} 거부: {why}'
            self.get_logger().warn(res.message)
            return self._finish(goal_handle, res)
        self.picker.forget_resume(what)      # 정지 뒤 다른 동작을 했으면 끊긴 집기 순서는 잇지 않는다
        self._touch()
        L = self.link
        L.last_ok = False
        if g.kind == G.JOG_JOINT:
            started = L.jog_joint(int(g.index), float(g.delta), joint_vel(g.speed_percent))
        elif g.kind == G.JOG_TCP:
            started = L.jog_cart(int(g.index), float(g.delta), cart_vel(g.speed_percent))
        elif g.kind == G.HOME:
            started = L.move_home(joint_vel(g.speed_percent))
        elif g.kind == G.RECOVER_HOME:
            started = L.recover_and_home(joint_vel(g.speed_percent))
        elif g.kind == G.STRETCH:                    # 「아리스 가동」: 모든 관절 0° (홈과 같은 검사)
            started = L.move_stretch(joint_vel(g.speed_percent))
        else:
            close = g.kind == G.GRIPPER_CLOSE
            started = L.gripper_command('move', pos=GRIP_CLOSE_POS if close else 0, cur=GRIP_CURRENT_MA,
                                        vel=GRIP_VEL, label=what)
        if not started:
            res.success, res.message = False, self._last_link_log(f'{what} 거부')
            return self._finish(goal_handle, res)

        def fb():
            """이동 중 feedback: RobotLink 가 마지막으로 남긴 로그 한 줄 ("서보 켜기 요청 성공 ..." 등)."""
            m = RealMotion.Feedback()
            m.message = self._last_link_log(what)
            goal_handle.publish_feedback(m)
        # 복구: recovering 이 끝난 뒤 홈 이동(busy)이 이어지므로 둘 다 볼 때까지 settle 을 길게
        self._wait(goal_handle, lambda: L.busy or L.recovering, fb,
                   settle=0.8 if g.kind == G.RECOVER_HOME else 0.3)
        self._touch()
        res.success = bool(L.last_ok) and not goal_handle.is_cancel_requested
        res.message = self._last_link_log(what)
        if res.success and g.kind == G.GRIPPER_OPEN:
            self.picker.holding = False         # 열었으면 빈손 (상태 토픽·「저기 놔줘」 판단이 맞게, 2026-11-14)
        elif res.success and g.kind == G.GRIPPER_CLOSE:
            self.picker.update_hold()           # 닫기 버튼으로 잡았는지도 판단 (화면의 「잡은 상태」 표시용, 2026-11-14)
        return self._finish(goal_handle, res)

    # ================================================================== 액션: 집기·놓기
    def _cube_yaw(self, o):
        """검출 o 의 큐브 면 방향 -> 로봇 베이스 각도 [deg, 90° 대칭]. 각도 맞추기를 껐거나 계산할 수 없으면 None."""
        if not self.align_yaw or o is None:
            return None
        return self.calib.angle_to_robot(o.pose.pose.position.x, o.pose.pose.position.y, px_yaw_deg(o))

    def _place_yaw(self, x, y):
        """놓는 각도: 큐브 면이 팔 방향(베이스 -> (x, y))과 평행·수직. 각도 맞추기를 껐으면 None."""
        return wrap90(radial_yaw_deg(x, y)) if self.align_yaw else None

    def _locator(self, near):
        """중심 맞추기에 줄 함수: 멈춘 뒤 새로 받은 검출 CENTER_SAMPLES 장의 집기용 픽셀 중앙값 -> 로봇 (x, y).

        near: 처음 고른 검출 (위치·넓이 기준). 없거나 중심 맞추기를 껐으면 None (단계 없음).
        그리퍼에 일부 가려져 넓이가 크게 다른 장은 버린다 - 가려진 쪽만큼 중심이 밀려 보이므로.
        """
        if not self.center_grasp or near is None:
            return None
        u0, v0, area0 = near.pose.pose.position.x, near.pose.pose.position.y, float(near.size_px)

        def locate():
            """작업 스레드에서 부름: 새 검출을 기다려 모은 뒤 중앙값. 못 모으면 이유(str)."""
            since = time.time() + CENTER_SETTLE_S
            deadline = since + CENTER_WAIT_S
            last, us, vs, hidden = 0.0, [], [], 0
            while time.time() < deadline and len(us) < CENTER_SAMPLES:
                stamp, objs = self.px_stamp, self.px_objects
                if stamp >= since and stamp != last:
                    last = stamp
                    o = nearest_px(objs, u0, v0, REFRESH_PX)
                    if o is not None and area0 > 0 and abs(o.size_px / area0 - 1.0) > CENTER_AREA_TOL:
                        hidden += 1
                    elif o is not None:
                        gu, gv = grasp_px(o)
                        us.append(gu)
                        vs.append(gv)
                time.sleep(0.03)
            if len(us) < CENTER_SAMPLES:
                return ('큐브가 잘 보이지 않음' + (' (그리퍼에 일부 가려짐)' if hidden else '')
                        + f' - {len(us)}/{CENTER_SAMPLES} 장')
            us.sort()
            vs.sort()
            return self.calib.to_robot(us[len(us) // 2], vs[len(vs) // 2])
        return locate

    def _target_xy(self, name, u, v, notes):
        """집기 직전: 받은 픽셀 가까이의 지금 검출로 큐브 위치를 고친 뒤 로봇 (x, y, 층, 각도).

        층은 깊이로 잰 값 (2026-11-13). 깊이가 없거나 안 보이면 0 (모름) -> 1층 높이로 집는다 (예전과 같음).
        각도는 큐브 면 방향 [deg] (2026-11-13). 안 보이거나 각도 맞추기를 껐으면 None -> 지금 각도로 집는다.
        마지막은 중심 맞추기 함수 (_locator, 안 보이거나 껐으면 None). 쌓인 큐브는 원근 보정 픽셀(grasp_px)로 계산.
        """
        near = nearest_px(self._fresh_px(PX_FRESH_S), u, v, REFRESH_PX)
        level = 0
        if near is not None:
            u, v = grasp_px(near)                   # 쌓인 큐브는 원근 보정 픽셀 (2026-11-13)
            level = int(near.level)
            if level == 0:
                notes.append(f'{name or "큐브"} 의 층을 모름 (깊이 없음) - 1층 높이로 집음')
        else:
            notes.append(f'{name or "큐브"} 가 지금 보이지 않음 - 받은 픽셀 사용' +
                         (', 각도는 맞추지 않음' if self.align_yaw else ''))
        x, y = self.calib.to_robot(u, v)
        return x, y, level, self._cube_yaw(near), self._locator(near)

    def _stack_base_xy(self, g, x, y):
        """쌓기의 받침 큐브 로봇 (x, y), 또는 거부 이유(str).

        받침 큐브는 지금 검출에서 다시 찾는다 (안 보이면 거부 - 빈 곳 4 cm 위에서 떨어뜨리지 않게).
        집을 큐브와 같은 자리면(같은 큐브를 두 번 고름) 거부한다.
        """
        near = nearest_px(self._fresh_px(PX_FRESH_S), g.place_u, g.place_v, REFRESH_PX)
        if near is None:
            return '받침 큐브가 지금 카메라에 보이지 않음'
        bx, by = self.calib.to_robot(*grasp_px(near))         # 쌓인 받침은 원근 보정 픽셀
        if math.hypot(bx - x, by - y) < STACK_MIN_SEP_MM:
            return '집을 큐브와 받침 큐브가 같은 자리'
        if int(near.level) >= MAX_LEVEL:            # 그 위에 하나 더 = MAX_LEVEL 넘음
            return f'받침이 이미 {near.level}층 (최대 {MAX_LEVEL}층까지만 쌓음)'
        # 층 모름(0) = 1층으로 봄 (예전과 같음). 각도: 받침 큐브 면 방향 (못 재면 팔 방향)
        base_yaw = self._cube_yaw(near)
        if base_yaw is None:
            base_yaw = self._place_yaw(bx, by)
        return bx, by, max(1, int(near.level)), base_yaw

    def _exec_pick(self, goal_handle):
        """/aris3/real/pick_place 실행: 검사 -> 대상 픽셀을 지금 검출로 고쳐 로봇 좌표로 -> PickSequence 시작 -> 끝날 때까지 단계 feedback."""
        g = goal_handle.request
        G = RealPickPlace.Goal
        res = RealPickPlace.Result()
        names = {G.HOVER: '큐브 위로', G.PICK: '집기', G.PLACE_AT: '고른 자리에 놓기',
                 G.PICK_AND_PLACE: '집어서 옮기기', G.PLACE_HERE: '여기 내려놓기', G.RESUME: '이어서 하기',
                 G.PICK_AND_STACK: '쌓기'}
        what = names.get(g.kind)
        res.holding = hold_code(self.picker.holding, RealPickPlace.Result)
        if what is None:
            res.success, res.message = False, f'모르는 동작 {g.kind}'
            return self._finish(goal_handle, res)
        why = self.why_blocked() or (None if self.calib.ready() else '카메라 보정이 없음 (점 4개 이상 기록)')
        if why:
            res.success, res.message = False, f'{what} 거부: {why}'
            self.get_logger().warn(res.message)
            return self._finish(goal_handle, res)
        notes = []
        z = self.calib.grasp_z
        P = self.picker
        level, yaw, locate = 0, None, None
        if g.kind in (G.HOVER, G.PICK, G.PICK_AND_PLACE, G.PICK_AND_STACK):
            x, y, level, yaw, locate = self._target_xy(g.target_name, g.target_u, g.target_v, notes)
            if level > MAX_LEVEL:                   # 잘못 잰 깊이로 높이 올라가 허공을 집지 않게
                res.success, res.message = False, f'{what} 거부: 큐브가 {level}층으로 재짐 (최대 {MAX_LEVEL}층)'
                self.get_logger().warn(res.message)
                return self._finish(goal_handle, res)
            if level > 1:
                notes.append(f'{g.target_name or "큐브"} 는 {level}층 - 집는 높이 +{P.level_lift(level):.0f} mm')
        pick_lift = P.level_lift(level)
        if g.kind in (G.PLACE_AT, G.PICK_AND_PLACE):
            px, py = self.calib.to_robot(g.place_u, g.place_v)
            place_yaw = self._place_yaw(px, py)
        if g.kind == G.PICK_AND_STACK:
            why = self._stack_base_xy(g, x, y)
            if isinstance(why, str):
                res.success, res.message = False, f'{what} 거부: {why}'
                self.get_logger().warn(res.message)
                return self._finish(goal_handle, res)
            px, py, base_level, place_yaw = why
        for n in notes:
            self.get_logger().warn(f'{what}: {n}')
        self._touch()
        if g.kind == G.RESUME:                # 정지로 끊긴 순서를 멈춘 단계부터 (좌표는 처음 계산한 그대로)
            started = P.resume()
        elif g.kind == G.HOVER:
            started = P.hover(x, y, z, pick_lift, yaw=yaw, locate=locate)
        elif g.kind == G.PICK:
            started = P.pick(x, y, z, pick_lift, yaw=yaw, locate=locate)
        elif g.kind == G.PLACE_AT:
            started = P.place_at(px, py, z, yaw=place_yaw)
        elif g.kind == G.PICK_AND_PLACE:
            started = P.pick_and_place(x, y, z, px, py, pick_lift=pick_lift, yaw=yaw, place_yaw=place_yaw,
                                       locate=locate)
        elif g.kind == G.PICK_AND_STACK:
            started = P.pick_and_stack(x, y, z, px, py, base_level, pick_lift, yaw=yaw, base_yaw=place_yaw,
                                       locate=locate)
        else:
            started = P.place_here(z)
        if not started:
            res.success, res.message = False, (f'{what} 거부: 정지로 멈춘 집기 순서가 없음 (정지 뒤 다른 동작을 했으면 잇지 않음)'
                                               if g.kind == G.RESUME else f'{what} 거부 (이전 순서 진행 중이거나 현재 좌표 모름)')
            return self._finish(goal_handle, res)

        def fb():
            """집기 중 feedback: 지금 단계 이름·번호와 RobotLink 마지막 로그."""
            m = RealPickPlace.Feedback()
            m.step, m.step_index, m.step_count = P.step, P.step_index, P.step_count
            m.detail = self._last_link_log(P.step)
            goal_handle.publish_feedback(m)
        self._wait(goal_handle, lambda: P.running, fb, settle=0.0)
        self._touch()
        res.holding = hold_code(P.holding, RealPickPlace.Result)
        res.success = bool(P.last_ok) and not goal_handle.is_cancel_requested
        res.message = f'{what}: ' + ('완료' if res.success else f'「{P.step}」 에서 멈춤 - {self._last_link_log("")}')
        return self._finish(goal_handle, res)

    # ================================================================== 주기: 잠금 시간, 상태 발행
    def _tick(self):
        now = time.time()
        busy = self.link.busy or self.picker.running or self.active_goal
        if busy:
            self._touch()                   # 움직이는 동안에는 잠그지 않는다
        if self.armed and now - self.last_cmd > self.arm_timeout:
            self.armed = False
            self.get_logger().info(f'{self.arm_timeout:.0f}초 동안 명령이 없어 조작을 잠갔습니다')
        self.pub_state.publish(self.state_msg(now))

    def state_msg(self, now=None):
        """지금 상태를 RealRobotState 한 장으로 만든다 (10 Hz 타이머가 발행, 시험에서도 직접 부름)."""
        now = now or time.time()
        L, P, c = self.link, self.picker, self.calib
        m = RealRobotState()
        m.stamp = self.get_clock().now().to_msg()
        m.connected = bool(L.joints_fresh())
        m.connecting = bool(self.conn.busy())
        m.connection_text = self.mgr.describe() or ('접속됨' if m.connected else '접속 안 됨')
        m.robot_state = int(L.robot_state or 0)
        name = L.state_name()
        m.robot_state_text = self.state_ko.get(name, name)
        m.robot_state_ok = L.robot_state in (STANDBY, MOVING)
        m.robot_mode = RealRobotState.ROBOT_MODE_UNKNOWN if L.robot_mode is None else int(L.robot_mode)
        m.mode_switching = bool(L.mode_switching)
        m.recovering = bool(L.recovering)
        m.can_recover = bool(L.cli_control.service_is_ready()) and not L.recovering
        m.busy = bool(L.busy)
        m.why_not_ready = (self.mgr.describe() if self.conn.busy() else L.why_not_ready()) or ''
        m.armed = bool(self.armed)
        m.armed_remaining_s = float(max(0.0, self.arm_timeout - (now - self.last_cmd))) if self.armed else 0.0
        if L.joints_deg is not None:
            m.joints_valid, m.joints_deg = True, [float(v) for v in L.joints_deg[:6]]
        if L.posx is not None:
            m.tcp_valid, m.tcp = True, [float(v) for v in L.posx[:6]]
        m.gripper_service_ready = bool(self.gripper_ready())
        m.gripper_running = bool(L.gripper_running)
        m.gripper_result = {None: RealRobotState.GRIPPER_RESULT_UNKNOWN, True: RealRobotState.GRIPPER_RESULT_OK,
                            False: RealRobotState.GRIPPER_RESULT_FAILED}[L.gripper_ok]
        if L.gripper:
            m.gripper_valid = True
            m.gripper_position = int(max(0, min(65535, L.gripper.get('position', 0))))
            m.gripper_torque = bool(L.gripper.get('torque'))
        m.sequence_running = bool(P.running)
        m.sequence_name, m.sequence_step = P.name, P.step
        m.holding = hold_code(P.holding)
        m.calib_points = len(c.points)
        m.calib_ready = bool(c.ready())
        if c.ready():
            m.calib_homography = [float(v) for v in c.H.reshape(-1)]
            m.calib_grasp_z_mm = float(c.grasp_z)
            e = [v for v in c.errors_mm if not math.isnan(v)]
            if e:
                m.calib_error_mean_mm, m.calib_error_max_mm = float(sum(e) / len(e)), float(max(e))
        m.calib_summary = c.summary() + (f'  |  {self._auto_summary()}' if self.auto_calib.points else '')
        if self.calib_px is not None:
            m.calib_pending, (m.calib_pending_u, m.calib_pending_v) = True, map(float, self.calib_px)
        return m


def _install_service_gripper(node, link, joint_topic, joint_name, holder):
    """가상 모드: RobotLink 의 그리퍼를 /aris3/gripper/command 서비스로 바꿔 끼운다 (손가락 각도는 관절 상태에서).

    손가락 각도를 읽는 관절 상태(Gazebo, 100 Hz)는 브리지 실행기가 아니라 **전용 작은 노드 + 단일 스레드 실행기**에서
    받는다 (2026-11-11). 브리지 실행기(rclpy MultiThreadedExecutor)는 메시지가 올 때마다 기다릴 대상 목록을 다시 만들어
    초당 100 번 깨면 CPU 를 많이 쓰고, 다른 스레드가 GIL 을 못 얻어 정지·서비스 응답이 수 초 늦어진 일이 있었다.
    """
    from aris3_interfaces.srv import GripperCommand
    from sensor_msgs.msg import JointState
    from rclpy.executors import SingleThreadedExecutor
    cb = ReentrantCallbackGroup()
    cli = node.create_client(GripperCommand, '/aris3/gripper/command', callback_group=cb)
    holder['grip_cli'] = cli
    joint = {'rad': None}

    def on_js(m):
        """Gazebo 손가락 관절 각도를 기억 (그리퍼 위치 = 잡음/빈손 판정용)."""
        if joint_name in m.name:
            joint['rad'] = float(m.position[m.name.index(joint_name)])
    watcher = rclpy.create_node('aris3_real_bridge_gripper_joint')
    watcher.create_subscription(JointState, joint_topic, on_js, 10)
    wex = SingleThreadedExecutor()
    wex.add_node(watcher)
    threading.Thread(target=wex.spin, daemon=True).start()
    holder['watcher'] = (watcher, wex)

    def call(req, timeout):
        """서비스를 부르고 응답을 기다린다 (작업 스레드에서). 없거나 시간 초과면 None."""
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        ev = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _f: ev.set())
        return fut.result() if ev.wait(timeout) else None
    ServiceGripper(link, call, lambda: joint['rad']).install()


def main(args=None):
    """노드 실행: 접속 설정 파라미터를 읽고 RobotLink·BringupManager 를 만든 뒤 브리지를 돌린다.

    끝날 때는 로봇을 정지시키고, 이 노드가 시작한 로봇 접속만 끊는다 (다른 곳에서 띄운 접속은 그대로).
    """
    from e0509_manual.bringup import BringupManager, ConnectionController
    from e0509_manual.robot_link import STATE_KO, start_link

    # 진단: `kill -USR1 <pid>` 를 보내면 모든 스레드의 현재 스택을 기록(표준 오류)에 남긴다 (멈춤 원인 찾기용)
    import faulthandler
    import signal
    faulthandler.register(signal.SIGUSR1, all_threads=True)

    rclpy.init(args=args)
    # 접속 설정은 이 노드의 파라미터로 받는다 (launch/real_bridge.launch.py)
    cfg = rclpy.create_node('aris3_real_bridge_cfg')
    for k, v in (('ns', 'e0509_real'), ('host', '110.120.1.66'), ('port', 12345), ('z_min_mm', 50.0),
                 ('mode', 'real'), ('gripper', 'drl'), ('calib_path', ''), ('cube_mm', 0.0),
                 ('gripper_joint_topic', '/e0509_gripper/joint_states'), ('gripper_joint', 'gripper_rh_r1')):
        cfg.declare_parameter(k, v, ANY_TYPE)
    P = {k: cfg.get_parameter(k).value for k in ('ns', 'host', 'port', 'z_min_mm', 'mode', 'gripper', 'calib_path', 'cube_mm',
                                                  'gripper_joint_topic', 'gripper_joint')}
    cfg.destroy_node()
    ns, host, port, z_min = str(P['ns']), str(P['host']), int(P['port']), float(P['z_min_mm'])
    mode, gripper = str(P['mode']), str(P['gripper'])
    # 안전: 가상 전용 부품(시뮬레이션 그리퍼 서비스)은 가상 모드에서만. 실물 로봇에 잘못 붙지 않게 막는다.
    if mode not in ('real', 'virtual') or gripper not in ('drl', 'service') or (mode == 'real' and gripper != 'drl'):
        raise SystemExit(f'[aris3_real_bridge] 잘못된 설정: mode={mode}, gripper={gripper} '
                         '(실물은 mode:=real gripper:=drl, 시뮬레이션은 mode:=virtual gripper:=service)')
    calib_path = str(P['calib_path']) or (CALIB_PATH if mode == 'real' else SIM_CALIB_PATH)
    if mode == 'virtual' and os.path.abspath(os.path.expanduser(calib_path)) == os.path.abspath(CALIB_PATH):
        raise SystemExit('[aris3_real_bridge] 가상 모드에서 실물 보정 파일을 쓰지 않습니다 (덮어쓰기 방지)')

    link, link_ex = start_link(ns, z_min)
    if mode == 'virtual':
        host = '127.0.0.1'
        mgr = make_virtual_bringup(BringupManager)(ns, host, port, rviz=False, log=link.log)
    else:
        mgr = BringupManager(ns, host, port, rviz=False, log=link.log)
    conn = ConnectionController(link, mgr)
    gripper_ready = None
    node_holder = {}
    if gripper == 'service':
        gripper_ready = lambda: node_holder['grip_cli'].service_is_ready()     # noqa: E731
    node = RealBridge(link, mgr, conn, state_ko=STATE_KO, calib_path=os.path.expanduser(calib_path),
                      gripper_ready=gripper_ready)
    # 쌓기 높이 = 큐브 한 개: cube_mm 를 주면 그 값, 아니면 실물 4 cm / 시뮬레이션 5 cm (큐브 크기가 다름)
    node.picker.stack_mm = float(P['cube_mm']) or (STACK_MM if mode == 'real' else STACK_SIM_MM)
    node.get_logger().info(f'쌓기 높이(큐브 한 개): {node.picker.stack_mm:.0f} mm')
    if gripper == 'service':
        _install_service_gripper(node, link, str(P['gripper_joint_topic']), str(P['gripper_joint']), node_holder)
    node.get_logger().info(f'로봇 {ns} @ {host}:{port} ({"가상 컨트롤러" if mode == "virtual" else "실물"}), '
                           f'그리퍼 {"서비스(시뮬레이션)" if gripper == "service" else "DRL"}, 최저 높이 {z_min:.0f} mm, '
                           f'보정 파일 {calib_path}')
    # 4 스레드: 오래 걸리는 액션 실행 1 + 정지 서비스(별도 콜백 그룹) + 나머지. 많을수록 GIL 다툼만 늘었다
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                # Ctrl-C / launch 종료(SIGINT) - 아래에서 정리
    finally:
        # 정리하는 동안 들어오는 SIGINT 는 무시한다: GUI 가 프로세스 그룹에 SIGINT 를 보내면 ros2 launch 가
        # 한 번 더 넘겨줘 두 번 받는다. 두 번째가 접속 끊기 도중 KeyboardInterrupt 를 내서 로봇 접속
        # (dsr_bringup2, 실물이면 서보 켜짐)이 남은 채 끝났다 (2026-11-11 ARIS 시험에서 발견).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # 순서가 중요: SIGINT 로 rclpy 가 이미 내려갔으면 ROS 호출(정지)은 실패할 수 있으므로,
        # ROS 와 상관없는 로봇 접속 끊기(dsr_bringup2 프로세스 종료 -> 서보 꺼짐)를 먼저 한다.
        if mgr.own_running():
            print('[aris3_real_bridge] 이 노드가 실행한 로봇 접속을 끊습니다', flush=True)
            mgr.stop()
        try:
            link.stop()
        except BaseException:  # noqa: BLE001 - 종료 중이라 ROS 가 없을 수 있음 (접속은 위에서 이미 끊음)
            pass
        ex.shutdown()
        if 'watcher' in node_holder:
            node_holder['watcher'][1].shutdown()
        link_ex.shutdown()
        node.destroy_node()
        link.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
