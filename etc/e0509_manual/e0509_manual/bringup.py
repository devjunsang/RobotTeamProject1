"""Connect / disconnect / reconnect the real E0509 from the manual controller.

"Connected" means a dsr_bringup2 process is running for our namespace. This module starts that
process itself (so it can stop it again) and can also stop a bringup that was started elsewhere
(e.g. from a terminal) as long as it is `ros2 launch dsr_bringup2 ... name:=<ns>`.
The GUI stops its own bringup when it closes. If the GUI is killed hard, the bringup keeps running;
the next GUI start finds it as "external" and can disconnect it.

What happens on the robot:
  connect     dsr_hardware2 takes control authority from the pendant, turns the servo ON and sets
              AUTONOMOUS mode (robot goes SAFE_OFF -> STANDBY). The arm does not move.
  disconnect  the bringup shuts down; the controller turns the servo OFF (brakes hold the arm).
              The flange gripper also seems to lose power (after a reconnect its RAM settings are back at
              power-on defaults and torque is off), so anything it holds may drop.
  reconnect   disconnect + connect. This also clears a stuck flange RS-485 link (gripper not answering).
"""
import os
import shutil
import signal
import socket
import subprocess
import threading
import time

LOG_DIR = '/tmp'
GRIPPER_BOOT_S = 2.0     # the gripper RAM is back at power-on defaults after a reconnect -> it was power cycled
RECONNECT_GAP_S = 10.0   # after a disconnect the controller needs a few s to free the RT (UDP 12347) session;
                         # reconnecting after 2 s made dsr_hardware2 abort in connect_rt_control (double free)

# bringup output lines worth showing in the GUI log
_KEYS = [('Connected to DRCF', '컨트롤러에 연결됨'),
         ('INITIAL STATE_STANDBY', '로봇 대기(STANDBY) 상태'),
         ('Connected RT control stream', '실시간 데이터 연결됨'),
         ('CONNECTION IS CLOSED', '컨트롤러 연결 닫힘')]
_ERR_KEYS = ('[ERROR]', 'process has died', 'failed to', 'Failed to')


def _alive(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().split(')')[-1].split()[0] != 'Z'
    except OSError:
        return False


def _ignores_sigint(pid):
    """True if the process ignores SIGINT (e.g. started with '&' from a script): use SIGTERM instead."""
    try:
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                if line.startswith('SigIgn:'):
                    return bool(int(line.split()[1], 16) & (1 << (signal.SIGINT - 1)))
    except OSError:
        pass
    return False


def find_bringup_pids(ns):
    """PIDs of `ros2 launch dsr_bringup2 ... name:=<ns>` processes (any owner session)."""
    pids = []
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                args = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        if 'launch' in args and 'dsr_bringup2' in args and f'name:={ns}' in args:
            pids.append(int(d))
    return sorted(pids)


def controller_reachable(host, port, timeout=1.5):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


class BringupManager:
    def __init__(self, ns, host, port=12345, rviz=True, log=print):
        self.ns, self.host, self.port, self.rviz = ns.strip('/'), host, int(port), bool(rviz)
        self.log = log
        self.proc = None
        self.log_path = os.path.join(LOG_DIR, f'e0509_manual_bringup_{self.ns}.log')
        self.phase = 'idle'          # idle | connecting | disconnecting
        self.crashed = False         # ros2_control_node (Doosan driver) died in the current bringup
        self.last_stop = 0.0         # wall time the last bringup ended
        self._stopping = False       # we are shutting it down: its shutdown noise is not an error
        self._lock = threading.Lock()

    # ---------------------------------------------------------------- status
    def own_running(self):
        return self.proc is not None and self.proc.poll() is None

    def external_pids(self):
        own = self.proc.pid if self.own_running() else None
        return [p for p in find_bringup_pids(self.ns) if p != own]

    def running(self):
        return self.own_running() or bool(self.external_pids())

    def describe(self):
        if self.phase == 'connecting':
            return '접속하는 중…'
        if self.phase == 'disconnecting':
            return '접속을 끊는 중…'
        if self.own_running():
            return f'이 화면이 실행한 접속 (pid {self.proc.pid})'
        ext = self.external_pids()
        if ext:
            return f'다른 곳에서 실행한 접속 (pid {", ".join(map(str, ext))})'
        return '접속 안 됨'

    # ---------------------------------------------------------------- start / stop
    def start(self):
        """Start dsr_bringup2 for the real robot. Returns True if the process was started."""
        with self._lock:
            if self.running():
                self.log('접속 거부: 이미 접속되어 있음')
                return False
            if shutil.which('ros2') is None:
                self.log('접속 실패: ros2 명령을 찾을 수 없음 (ROS 환경을 source 했는지 확인)')
                return False
            if not controller_reachable(self.host, self.port):
                self.log(f'접속 실패: 컨트롤러 {self.host}:{self.port}에 닿지 않음 '
                         '(랜선, PC 유선 IP 110.120.1.x, 컨트롤러 전원 확인)')
                return False
            cmd = ['ros2', 'launch', 'dsr_bringup2', 'dsr_bringup2_rviz.launch.py',
                   'mode:=real', f'host:={self.host}', f'port:={self.port}', 'model:=e0509',
                   'color:=white', f'name:={self.ns}', 'remap_tf:=true',
                   f'gui:={"true" if self.rviz else "false"}']
            gap = RECONNECT_GAP_S - (time.time() - self.last_stop)
            if gap > 0:
                self.log(f'컨트롤러가 이전 연결을 정리하도록 {gap:.0f}초 기다린 뒤 접속합니다')
                time.sleep(gap)
            self.crashed = False
            self._stopping = False
            logf = open(self.log_path, 'a')
            logf.write(f'\n===== {time.strftime("%F %T")} {" ".join(cmd)}\n')
            logf.flush()
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                         bufsize=1, start_new_session=True)
            threading.Thread(target=self._pump, args=(self.proc, logf), daemon=True).start()
            self.log(f'접속 시작: {self.host} (이름 /{self.ns}, 기록 {self.log_path})')
            return True

    def _pump(self, proc, logf):
        for line in proc.stdout:
            logf.write(line)
            logf.flush()
            for key, ko in _KEYS:
                if key in line:
                    self.log(f'[접속] {ko}')
            if 'process has died' in line and 'ros2_control_node' in line:
                self.crashed = True
                self.log('[접속] 두산 드라이버(ros2_control_node)가 종료됨')
            elif self._stopping or 'pal_statistics' in line:
                pass
            elif any(k in line for k in _ERR_KEYS) and 'RT scheduling' not in line:
                self.log('[접속] ' + line.strip()[-160:])
        logf.write(f'===== process exited with {proc.wait()}\n')
        logf.close()

    def stop(self, timeout=20.0):
        """Stop our bringup or an external one for this namespace. Returns True when none is left."""
        with self._lock:
            targets = []
            if self.own_running():
                targets.append(('own', self.proc.pid))
            targets += [('ext', p) for p in self.external_pids()]
            if not targets:
                return True
            self._stopping = True
            for kind, pid in targets:
                try:
                    if kind == 'own':
                        os.killpg(pid, signal.SIGINT)   # whole launch tree, like Ctrl-C
                    elif _ignores_sigint(pid):
                        os.kill(pid, signal.SIGTERM)    # launch started in the background ignores Ctrl-C
                    else:
                        os.kill(pid, signal.SIGINT)     # only the launch process; it stops its children
                except ProcessLookupError:
                    pass
            t0 = time.time()
            for sig, wait_s in ((None, timeout), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
                if sig is not None:
                    left = [(k, p) for k, p in targets if _alive(p)]
                    if not left:
                        break
                    self.log(f'접속 종료가 늦어 {sig.name} 보냄')
                    for kind, pid in left:
                        try:
                            os.killpg(pid, sig) if kind == 'own' else os.kill(pid, sig)
                        except ProcessLookupError:
                            pass
                t1 = time.time()
                while time.time() - t1 < wait_s and any(_alive(p) for _, p in targets):
                    time.sleep(0.2)
            if self.proc is not None:
                try:
                    self.proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
            ok = not any(_alive(p) for _, p in targets)
            self.last_stop = time.time()
            self._stopping = False
            self.log(f'접속 끊음 ({time.time() - t0:.1f}s)' if ok else '접속 종료 실패: 프로세스가 남아 있음')
            return ok


class ConnectionController:
    """Connect / disconnect / reconnect with the safety steps around the bringup process."""

    STANDBY_WAIT_S = 45.0

    def __init__(self, link, manager):
        self.link, self.mgr = link, manager

    def busy(self):
        return self.mgr.phase != 'idle'

    def _run(self, fn):
        if self.busy():
            self.link.log('이미 접속/끊기 작업 중')
            return False
        threading.Thread(target=fn, daemon=True).start()
        return True

    def connect_async(self):
        return self._run(self._connect)

    def disconnect_async(self):
        return self._run(self._disconnect)

    def reconnect_async(self):
        return self._run(self._reconnect)

    def _connect(self):
        self.mgr.phase = 'connecting'
        try:
            self._do_connect()
        finally:
            self.mgr.phase = 'idle'

    def _disconnect(self):
        self.mgr.phase = 'disconnecting'
        try:
            self._do_disconnect()
        finally:
            self.mgr.phase = 'idle'

    def _reconnect(self):
        self.mgr.phase = 'disconnecting'
        try:
            if not self._do_disconnect():
                return
            self.mgr.phase = 'connecting'    # start() waits RECONNECT_GAP_S after the disconnect
            self._do_connect()
        finally:
            self.mgr.phase = 'idle'

    def _do_connect(self, retry=True):
        if not self.mgr.start():
            return False
        t0 = time.time()
        while time.time() - t0 < self.STANDBY_WAIT_S:
            time.sleep(0.5)
            if self.mgr.crashed:
                self.mgr.stop()
                self.link.on_disconnected()
                if retry:
                    self.link.log('접속 중 드라이버가 종료되어 한 번 더 시도합니다')
                    return self._do_connect(retry=False)
                self.link.log('접속 실패: 드라이버가 다시 종료됨. 잠시 뒤 「접속」을 눌러 보세요 '
                              f'(기록 {self.mgr.log_path})')
                return False
            if not self.mgr.own_running():
                self.link.log('접속 실패: bringup이 종료됨 (위 기록과 ' + self.mgr.log_path + ' 확인)')
                return False
            if self.link.robot_state == 1 and self.link.joints_fresh():
                time.sleep(GRIPPER_BOOT_S)       # flange power comes back with the servo: let the gripper boot
                self.link.on_connected()
                self.link.log(f'접속 완료: 로봇 대기(STANDBY) ({time.time() - t0:.0f}s)')
                return True
        self.link.log(f'{self.STANDBY_WAIT_S:.0f}초 안에 대기(STANDBY)가 되지 않음 — 펜던트의 알람/비상정지 확인. '
                      '접속은 유지됨')
        return False

    def _do_disconnect(self):
        if self.link.busy:
            self.link.log('접속 끊기 전 동작 정지')
            self.link.stop()
            t0 = time.time()
            while self.link.busy and time.time() - t0 < 8.0:
                time.sleep(0.1)
        ok = self.mgr.stop()
        self.link.on_disconnected()
        return ok
