"""ARIS3 Control Station 실행 파일.

  ros2 run aris3_gui control_station                      # 시뮬레이션 모드 (Connect = 시뮬레이션 시작)
  ros2 run aris3_gui control_station --mode real          # 실물 E0509 (etc/e0509_manual 의 접속·안전 코드)
                                                          #   음성·손짓 노드도 함께 실행 (--no-voice / --no-gesture)
  ros2 run aris3_gui control_station --no-assist          # 시뮬레이션 순수 마찰 파지

시험용: --screenshot <png> --quit-after <초>  (화면을 파일로 저장하고 종료)
"""
import argparse
import os
import signal
import sys

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from aris3_gui.backends import RealBackend, SimBackend
from aris3_gui.main_window import ControlStation
from aris3_gui.ros_io import start_node


def simulate_selection(w, node, spec):
    """카메라 화면 클릭을 흉내 낸다: world 점 -> 화면 픽셀(project) -> 클릭과 같은 경로(unproject)."""
    oid, _, xy = spec.partition(':')
    tx, ty = (float(v) for v in xy.split(','))
    obj = next((o for o in node.snapshot()['objects'] if o.id == oid), None)
    cam = w.camera
    if obj is None or cam.cam_tf is None or cam.qimage is None:
        return False                        # 아직 영상·TF·인식 결과가 안 들어옴 -> 다시 시도
    for wx, wy in ((obj.pose.pose.position.x, obj.pose.pose.position.y), (tx, ty)):
        u, v, _d, _f = cam.project((wx, wy, cam.plane_z))
        back = cam.unproject(u, v)
        node.log('INFO', f'--select: world ({wx:.3f}, {wy:.3f}) -> 픽셀 ({u:.1f}, {v:.1f}) -> '
                         f'world ({back[0]:.3f}, {back[1]:.3f})')
        w._on_camera_click(*back)
    if not node.voice_rate.fresh(3.0):
        w._run_selection()             # 음성 노드가 없으면 버튼으로 실행
    return True


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['sim', 'real'], default='sim')
    ap.add_argument('--no-assist', action='store_true', help='시뮬레이션: 물체 붙이기 보조 끄기 (순수 마찰 파지)')
    ap.add_argument('--no-voice', action='store_true', help='음성 노드를 함께 띄우지 않음 (시뮬레이션·실물 공통)')
    ap.add_argument('--no-gesture', action='store_true', help='실물: 손짓(MediaPipe) 노드를 함께 띄우지 않음')
    ap.add_argument('--host', default='110.120.1.66', help='실물: E0509 컨트롤러 IP')
    ap.add_argument('--ns', default='e0509_real', help='실물: dsr_bringup2 이름')
    ap.add_argument('--z-min', type=float, default=50.0, help='실물: 손끝 최저 높이 [mm]')
    ap.add_argument('--task', action='append', default=[], metavar='COLOR:ZONE',
                    help='시작 3 초 뒤 작업 큐에 넣고 실행 (예: --task blue:A --task red:B)')
    ap.add_argument('--voice-text', default='', help='시험용: 시작 3 초 뒤 이 문장을 음성 해석기에 보냄')
    ap.add_argument('--select', default='', metavar='ID:X,Y',
                    help='시험용: 카메라 화면 클릭을 흉내 냄 - 물체 ID 와 world (X, Y) 빈 곳을 클릭. '
                         '음성 노드가 있으면 --voice-text 로 「이거 저기로 옮겨줘」 등을 함께 시험')
    ap.add_argument('--voice-auto', action='store_true', help='음성 명령 「자동 실행」 을 켠 채 시작')
    ap.add_argument('--screenshot', default='', help='시험용: 화면을 이 PNG 로 저장')
    ap.add_argument('--quit-after', type=float, default=0.0, help='시험용: N 초 뒤 종료')
    a = ap.parse_args(argv[1:])

    if os.environ.get('DISPLAY') and 'QT_QPA_PLATFORM' not in os.environ:
        os.environ['QT_QPA_PLATFORM'] = 'xcb'      # RViz 와 같은 X11 백엔드
    app = QApplication(sys.argv[:1])
    node, ex = start_node()
    if a.mode == 'real':
        backend = RealBackend(node, ns=a.ns, host=a.host, z_min=a.z_min, voice=not a.no_voice,
                              gesture=not a.no_gesture)
    else:
        backend = SimBackend(node, grasp_assist=not a.no_assist, voice=not a.no_voice,
                              gesture=not a.no_gesture)
    w = ControlStation(node, backend)
    w.resize(1680, 980)
    w.show()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: QTimer.singleShot(0, w.close))
    tick = QTimer()
    tick.timeout.connect(lambda: None)        # Qt 가 쉬는 동안 Python 신호 처리기가 돌 수 있게
    tick.start(200)
    w.chk_voice_auto.setChecked(a.voice_auto)

    def startup_actions():
        for spec in a.task:
            color, _, zone = spec.partition(':')
            w._queue_add(color, zone.upper() or 'A', False)
        if a.task:
            w._start_queue()
        if a.voice_text and not a.select:
            node.voice_text(a.voice_text)
        if a.select:
            tries = {'n': 0}

            def attempt():
                tries['n'] += 1
                if simulate_selection(w, node, a.select):
                    if a.voice_text:          # 클릭(선택)이 끝난 뒤에 말한다: 「저기로 옮겨줘」
                        QTimer.singleShot(1500, lambda: node.voice_text(a.voice_text))
                else:
                    if tries['n'] < 20:
                        QTimer.singleShot(1000, attempt)
                    else:
                        node.log('ERROR', f'--select: {a.select} - 물체·카메라 영상·TF 가 20 초 안에 준비되지 않음')
            attempt()
    QTimer.singleShot(3000, startup_actions)
    if a.screenshot:
        QTimer.singleShot(int(max(1.0, a.quit_after - 0.5) * 1000), lambda: w.grab().save(a.screenshot))
    if a.quit_after:
        QTimer.singleShot(int(a.quit_after * 1000), app.quit)
    code = app.exec_()
    ex.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
