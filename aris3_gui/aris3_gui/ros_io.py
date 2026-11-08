"""ARIS3 GUI 의 ROS 2 쪽 (시뮬레이션 모드).

GUI(Qt) 와 분리된 노드 하나가 별도 스레드의 executor 에서 돈다. 받은 값은 잠금(lock) 아래 보관하고,
Qt 타이머가 100 ms 마다 snapshot() 으로 읽어 화면을 그린다 (e0509_manual/gui.py 와 같은 구조).

이 노드가 하는 일은 모두 ARIS3 공통 인터페이스를 통한다. 로봇을 직접 움직이는 경로는 두 가지뿐이다:
  - 작업:    /aris3/execute_task (Task Manager 가 검증·계획·실패 처리를 맡음)
  - 수동 조작: MoveIt /move_action (계획 단계에서 충돌·관절 한계 검사), 한 번에 하나, 작업 중에는 거부
정지는 언제나 허용: 작업 취소 + MoveIt 실행 중단(/trajectory_execution_event "stop").
"""
import math
import threading
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from aris3_interfaces.action import ExecuteTask
from aris3_interfaces.msg import DetectedObjectArray, TaskStatus, VoiceCommand
from aris3_interfaces.srv import GripperCommand

ARM_JOINTS = [f'joint_{i}' for i in range(1, 7)]
HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]          # Task Manager 관찰 자세와 같음
STAGES = ['VALIDATE', 'OBSERVE', 'PERCEIVE', 'PLAN', 'PICK', 'PLACE', 'VERIFY']
LOG_NODES = ('aris3', 'task_manager')   # 'ARIS3 로그만' 에서 INFO 까지 보여 줄 노드 (나머지는 WARN 이상만)

# 수동 조작 제한 (e0509_manual/robot_link.py 의 실물 한계와 같은 값)
MAX_JOINT_STEP_DEG = 10.0
MAX_CART_STEP_MM = 20.0


class RateMeter:
    """최근 메시지 간격으로 수신 주기(Hz)를 잰다."""

    def __init__(self, window=20):
        self.t = deque(maxlen=window)

    def tick(self):
        self.t.append(time.monotonic())

    def hz(self):
        if len(self.t) < 2 or time.monotonic() - self.t[-1] > 2.0:
            return 0.0
        return (len(self.t) - 1) / (self.t[-1] - self.t[0])

    def fresh(self, max_age=1.0):
        return bool(self.t) and time.monotonic() - self.t[-1] < max_age


class GuiNode(Node):
    def __init__(self, robot_ns='/e0509_gripper', camera_topic='/aris3/overhead_camera/image'):
        super().__init__('aris3_gui')
        self.lock = threading.Lock()
        cb = ReentrantCallbackGroup()

        # ---- 받은 값
        self.joints = {}            # 이름 -> rad
        self.js_rate = RateMeter()
        self.clock_rate = RateMeter()
        self.cam_rate = RateMeter()
        self.image = None           # (width, height, encoding, bytes)
        self.image_frame = ''
        self.objects = []           # DetectedObject 목록 (world)
        self.det_rate = RateMeter()
        self.task_status = None     # TaskStatus
        self.voice = None           # 마지막 VoiceCommand
        self.voice_state = ''
        self.voice_rate = RateMeter()
        self.logs = deque(maxlen=400)   # (seq, level, name, text)
        self.log_seq = 0

        # ---- 작업 상태
        self.goal_handle = None
        self.task_running = False
        self.feedback = None        # 마지막 ExecuteTask feedback
        self.stage_state = {s: '' for s in STAGES}      # '' | 'run' | 'done' | 'fail'
        self.stage_time = {s: 0.0 for s in STAGES}
        self._stage_t0 = 0.0
        self.last_result = None     # (success, message)
        self.on_task_done = None    # GUI 가 다음 작업을 이어서 보내기 위한 콜백 (Qt 스레드 아님 주의)
        self.manual_busy = False
        self.gripper_busy = False
        self.gripper_msg = ''

        self.create_subscription(JointState, f'{robot_ns}/joint_states', self._on_js, 50, callback_group=cb)
        self.create_subscription(Clock, '/clock', lambda _m: self.clock_rate.tick(), 10, callback_group=cb)
        self.create_subscription(Image, camera_topic, self._on_image, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(DetectedObjectArray, '/aris3/detected_objects', self._on_objects, 10,
                                 callback_group=cb)
        self.create_subscription(TaskStatus, '/aris3/task_status', self._on_status, 10, callback_group=cb)
        self.create_subscription(VoiceCommand, '/aris3/voice_command', self._on_voice, 10, callback_group=cb)
        self.create_subscription(String, '/aris3/voice/state', self._on_voice_state, 10, callback_group=cb)
        self.create_subscription(Log, '/rosout', self._on_rosout, 100, callback_group=cb)

        self.task_client = ActionClient(self, ExecuteTask, '/aris3/execute_task', callback_group=cb)
        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=cb)
        self.gripper_client = self.create_client(GripperCommand, '/aris3/gripper/command', callback_group=cb)
        self.pub_listen = self.create_publisher(Bool, '/aris3/voice/listen', 10)
        self.pub_text = self.create_publisher(String, '/aris3/voice/text', 10)
        self.pub_traj_event = self.create_publisher(String, '/trajectory_execution_event', 10)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    # ================================================================== 구독 콜백
    def _on_js(self, m):
        with self.lock:
            self.joints.update(zip(m.name, m.position))
        self.js_rate.tick()

    def _on_image(self, m):
        with self.lock:
            self.image = (m.width, m.height, m.encoding, bytes(m.data))
            self.image_frame = m.header.frame_id
        self.cam_rate.tick()

    def _on_objects(self, m):
        with self.lock:
            self.objects = list(m.objects)
        self.det_rate.tick()

    def _on_status(self, m):
        with self.lock:
            self.task_status = m

    def _on_voice(self, m):
        with self.lock:
            self.voice = m
        self.log('INFO', f'음성: 「{m.text}」 -> {m.action or "?"} {m.color or "?"} '
                         f'{"(실행 가능)" if m.valid else "- " + m.message}')

    def _on_voice_state(self, m):
        with self.lock:
            self.voice_state = m.data
        self.voice_rate.tick()

    def _on_rosout(self, m):
        level = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(m.level, str(m.level))
        if m.name == self.get_name():
            return
        self._add_log(level, m.name, m.msg)

    def _add_log(self, level, name, text):
        with self.lock:
            self.log_seq += 1
            self.logs.append((self.log_seq, level, name, text, time.strftime('%H:%M:%S')))

    def log(self, level, text):
        self._add_log(level, 'gui', text)

    # ================================================================== 상태 읽기
    def tcp_pose(self, tcp_offset=0.105):
        """world 기준 (link_6 위치, 집는 점 위치, RPY) 또는 None."""
        try:
            t = self.tf.lookup_transform('world', 'link_6', Time())
        except Exception:  # noqa: BLE001
            return None
        p, q = t.transform.translation, t.transform.rotation
        zx = 2 * (q.x * q.z + q.w * q.y)
        zy = 2 * (q.y * q.z - q.w * q.x)
        zz = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tcp = (p.x + zx * tcp_offset, p.y + zy * tcp_offset, p.z + zz * tcp_offset)
        return (p.x, p.y, p.z), tcp, (roll, pitch, yaw)

    def camera_transform(self):
        """world 점 -> 카메라 광학 좌표 변환 (TransformStamped) 또는 None."""
        with self.lock:
            frame = self.image_frame
        if not frame:
            return None
        try:
            return self.tf.lookup_transform(frame, 'world', Time())
        except Exception:  # noqa: BLE001
            return None

    def snapshot(self):
        with self.lock:
            return {
                'joints': dict(self.joints),
                'image': self.image,
                'objects': list(self.objects),
                'task_status': self.task_status,
                'voice': self.voice,
                'voice_state': self.voice_state,
                'feedback': self.feedback,
                'stage_state': dict(self.stage_state),
                'stage_time': dict(self.stage_time),
                'last_result': self.last_result,
                'logs': list(self.logs),
                'log_seq': self.log_seq,
            }

    def status_flags(self):
        """시스템 상태 패널용: 각 구성 요소가 살아 있는가."""
        return {
            'gazebo': self.clock_rate.fresh(2.0),
            'joint_states': self.js_rate.fresh(1.0),
            'js_hz': self.js_rate.hz(),
            'camera_hz': self.cam_rate.hz(),
            'detections': self.det_rate.fresh(2.0),
            'gripper': self.gripper_client.service_is_ready(),
            'moveit': self.move_client.server_is_ready(),
            'task_manager': self.task_client.server_is_ready(),
            'voice': self.voice_rate.fresh(3.0),
        }

    # ================================================================== 작업 (ExecuteTask)
    def send_task(self, color, zone, dry_run=False, shape=''):
        if self.task_running:
            self.log('WARN', '이미 작업 중 - 요청 무시')
            return False
        if not self.task_client.server_is_ready():
            self.log('ERROR', 'Task Manager(/aris3/execute_task) 없음')
            return False
        goal = ExecuteTask.Goal(task_type='pick_place', target_color=color, target_shape=shape,
                                destination=zone, dry_run=dry_run)
        with self.lock:
            self.task_running = True
            self.stage_state = {s: '' for s in STAGES}
            self.stage_time = {s: 0.0 for s in STAGES}
            self.feedback = None
            self.last_result = None
        self.log('INFO', f'작업 요청: {color} -> {zone}' + (' (dry_run)' if dry_run else ''))
        fut = self.task_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, fut):
        handle = fut.result()
        if handle is None or not handle.accepted:
            with self.lock:
                self.task_running = False
                self.last_result = (False, 'Task Manager 가 작업을 거절함 (이미 작업 중?)')
            self.log('ERROR', '작업 거절됨')
            self._notify_done(False)
            return
        with self.lock:
            self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        fb = msg.feedback
        now = time.monotonic()
        with self.lock:
            self.feedback = fb
            if fb.stage in self.stage_state and self.stage_state[fb.stage] != 'run':
                # 새 단계 시작: 이전에 진행 중이던 단계는 끝난 것
                for s, st in self.stage_state.items():
                    if st == 'run':
                        self.stage_state[s] = 'done'
                        self.stage_time[s] = now - self._stage_t0
                self.stage_state[fb.stage] = 'run'
                self._stage_t0 = now

    def _on_result(self, fut):
        res = fut.result()
        r = res.result
        ok = res.status == GoalStatus.STATUS_SUCCEEDED and r.success
        now = time.monotonic()
        with self.lock:
            for s, st in self.stage_state.items():
                if st == 'run':
                    self.stage_state[s] = 'done' if ok else 'fail'
                    self.stage_time[s] = now - self._stage_t0
            if not ok and r.failed_stage in self.stage_state:
                self.stage_state[r.failed_stage] = 'fail'
            self.task_running = False
            self.goal_handle = None
            self.last_result = (ok, r.message)
        self.log('INFO' if ok else 'ERROR', f'작업 {"성공" if ok else "실패"}: {r.message}')
        self._notify_done(ok)

    def _notify_done(self, ok):
        if self.on_task_done:
            self.on_task_done(ok)

    def cancel_task(self):
        with self.lock:
            h = self.goal_handle
        if h is not None:
            h.cancel_goal_async()
            self.log('WARN', '작업 취소 요청')

    def stop_all(self):
        """정지: 작업 취소 + MoveIt 실행 중단. 언제든 허용 (소프트웨어 정지 - 인증된 비상정지가 아님)."""
        self.cancel_task()
        self.pub_traj_event.publish(String(data='stop'))
        self.log('WARN', '정지: 작업 취소 + MoveIt 실행 중단')

    # ================================================================== 그리퍼
    def gripper(self, command):
        if self.task_running:
            self.log('WARN', '작업 중에는 그리퍼를 직접 조작하지 않음')
            return
        if not self.gripper_client.service_is_ready():
            self.log('ERROR', '그리퍼 서비스(/aris3/gripper/command) 없음')
            return
        self.gripper_busy = True
        fut = self.gripper_client.call_async(GripperCommand.Request(command=command, force=0.5))

        def done(f):
            r = f.result()
            self.gripper_busy = False
            self.gripper_msg = r.message if r else '응답 없음'
            self.log('INFO' if (r and r.success) else 'ERROR', f'그리퍼: {self.gripper_msg}')
        fut.add_done_callback(done)

    # ================================================================== 음성
    def voice_listen(self, on):
        self.pub_listen.publish(Bool(data=on))

    def voice_text(self, text):
        self.pub_text.publish(String(data=text))

    # ================================================================== 수동 조작 (MoveIt)
    def _move_goal(self, constraints, label, pipeline='ompl', planner='', vel=0.2):
        if self.task_running:
            self.log('WARN', f'{label}: 작업 중에는 수동 조작 불가')
            return
        if self.manual_busy:
            self.log('WARN', f'{label}: 이전 수동 이동이 아직 끝나지 않음')
            return
        if not self.move_client.server_is_ready():
            self.log('ERROR', 'MoveIt(/move_action) 없음')
            return
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        req.pipeline_id = pipeline
        req.planner_id = planner
        req.num_planning_attempts = 3
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = vel
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        self.manual_busy = True
        self.log('INFO', f'수동: {label}')
        fut = self.move_client.send_goal_async(goal)

        def accepted(f):
            h = f.result()
            if h is None or not h.accepted:
                self.manual_busy = False
                self.log('ERROR', f'{label}: MoveIt 이 거절')
                return
            h.get_result_async().add_done_callback(finished)

        def finished(f):
            self.manual_busy = False
            code = f.result().result.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                self.log('INFO', f'{label}: 완료')
            else:
                self.log('ERROR', f'{label}: 실패 (MoveIt error_code {code} - 도달 불가·충돌·관절 한계일 수 있음)')
        fut.add_done_callback(accepted)

    def _joint_constraints(self, target):
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=j, position=v, tolerance_above=0.005,
                                               tolerance_below=0.005, weight=1.0)
                               for j, v in zip(ARM_JOINTS, target)]
        return c

    def home(self, vel=0.3):
        self._move_goal(self._joint_constraints([math.radians(d) for d in HOME_DEG]), '홈 자세', vel=vel)

    def jog_joint(self, index, step_deg, vel):
        step = max(-MAX_JOINT_STEP_DEG, min(MAX_JOINT_STEP_DEG, step_deg))
        with self.lock:
            cur = [self.joints.get(j) for j in ARM_JOINTS]
        if None in cur or not self.js_rate.fresh():
            self.log('ERROR', '관절 상태가 들어오지 않음')
            return
        cur[index] += math.radians(step)
        self._move_goal(self._joint_constraints(cur), f'J{index + 1} {step:+g}°', vel=vel)

    def jog_tcp(self, axis, step_mm, vel):
        """link_6 를 world 기준 X/Y/Z 로 직선 이동 (방향 유지, Pilz LIN)."""
        step = max(-MAX_CART_STEP_MM, min(MAX_CART_STEP_MM, step_mm)) / 1000.0
        try:
            t = self.tf.lookup_transform('world', 'link_6', Time())
        except Exception as e:  # noqa: BLE001
            self.log('ERROR', f'현재 손끝 위치 모름 (TF): {e}')
            return
        target = PoseStamped()
        target.header.frame_id = 'world'
        p = t.transform.translation
        target.pose.position.x, target.pose.position.y, target.pose.position.z = p.x, p.y, p.z
        setattr(target.pose.position, 'xyz'[axis], getattr(target.pose.position, 'xyz'[axis]) + step)
        target.pose.orientation = t.transform.rotation
        pc = PositionConstraint(link_name='link_6', weight=1.0)
        pc.header.frame_id = 'world'
        pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])]
        pc.constraint_region.primitive_poses = [target.pose]
        oc = OrientationConstraint(link_name='link_6', orientation=target.pose.orientation,
                                   absolute_x_axis_tolerance=0.01, absolute_y_axis_tolerance=0.01,
                                   absolute_z_axis_tolerance=0.01, weight=1.0)
        oc.header.frame_id = 'world'
        c = Constraints(position_constraints=[pc], orientation_constraints=[oc])
        self._move_goal(c, f'TCP {"XYZ"[axis]} {step * 1000:+.0f} mm', pipeline='pilz_industrial_motion_planner',
                        planner='LIN', vel=vel)


def start_node(**kw):
    """노드를 만들고 별도 스레드에서 돌린다. (node, executor) 를 돌려준다."""
    if not rclpy.ok():
        rclpy.init()
    node = GuiNode(**kw)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    return node, ex
