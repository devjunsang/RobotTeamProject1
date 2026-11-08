"""ARIS3 Control Station 실행 파일.

  ros2 run aris3_gui control_station                      # 시뮬레이션 모드 (Connect = 시뮬레이션 시작)
  ros2 run aris3_gui control_station --mode real          # 실물 E0509 (etc/e0509_manual 의 접속·안전 코드)
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


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['sim', 'real'], default='sim')
    ap.add_argument('--no-assist', action='store_true', help='시뮬레이션: 물체 붙이기 보조 끄기 (순수 마찰 파지)')
    ap.add_argument('--no-voice', action='store_true', help='시뮬레이션: 음성 노드를 함께 띄우지 않음')
    ap.add_argument('--host', default='110.120.1.66', help='실물: E0509 컨트롤러 IP')
    ap.add_argument('--ns', default='e0509_real', help='실물: dsr_bringup2 이름')
    ap.add_argument('--z-min', type=float, default=50.0, help='실물: 손끝 최저 높이 [mm]')
    ap.add_argument('--task', action='append', default=[], metavar='COLOR:ZONE',
                    help='시작 3 초 뒤 작업 큐에 넣고 실행 (예: --task blue:A --task red:B)')
    ap.add_argument('--voice-text', default='', help='시험용: 시작 3 초 뒤 이 문장을 음성 해석기에 보냄')
    ap.add_argument('--voice-auto', action='store_true', help='음성 명령 「자동 실행」 을 켠 채 시작')
    ap.add_argument('--screenshot', default='', help='시험용: 화면을 이 PNG 로 저장')
    ap.add_argument('--quit-after', type=float, default=0.0, help='시험용: N 초 뒤 종료')
    a = ap.parse_args(argv[1:])

    if os.environ.get('DISPLAY') and 'QT_QPA_PLATFORM' not in os.environ:
        os.environ['QT_QPA_PLATFORM'] = 'xcb'      # RViz 와 같은 X11 백엔드
    app = QApplication(sys.argv[:1])
    node, ex = start_node()
    if a.mode == 'real':
        backend = RealBackend(node, ns=a.ns, host=a.host, z_min=a.z_min)
    else:
        backend = SimBackend(node, grasp_assist=not a.no_assist, voice=not a.no_voice)
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
        if a.voice_text:
            node.voice_text(a.voice_text)
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
