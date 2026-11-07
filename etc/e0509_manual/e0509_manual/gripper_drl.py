"""RH-P12-RN(A) gripper on the E0509 tool flange: DRL program builder and result decoder.

How the gripper is reached (verified on this robot 2026-11-07):
  PC --ROS drl_start--> Doosan controller runs a DRL program --flange RS-485--> gripper
  The gripper is in Modbus-RTU mode, 57600 bps 8N1, slave ID 1. Use ONLY 57600 bps.

DRL programs cannot return values to ROS, so each program writes its results into the controller's
general-purpose output registers (set_output_register_int) and ROS reads them back with
<ns>/plc/get_output_register_int. On this controller (DRCF 2.10.3) only the LOW BYTE of a register
survives that round trip and it comes back shifted (value << 24), so every register carries one byte.

Register layout (R = general-purpose output register):
  R0  status   1 running, 2 done, 3 DRL exception, 4 gripper did not answer
  R1  stage    which step failed (see STAGES)
  R2,R3 present position (lo, hi)          R4,R5 present current mA (lo, hi; signed 16 bit)
  R6  moving   R7 torque enable            R8 hardware error status
  R9  temperature (deg C)                  R10 input voltage (0.1 V)
  R11 1 = still moving when the wait timed out
  R12,R13 goal current readback            R14,R15 goal position readback
  R16,R17 profile velocity readback
  R22 run token (written last; ROS waits for its own token)    R23 = 7 (marker)

Modbus map (RH-P12-RN(A) e-Manual, 40001-based; register = address - 40001):
  256 lo Torque Enable | 259 lo Hardware Error | 275 Goal Current | 280-281 Profile Velocity
  282-283 Goal Position (low word first) | 285 lo Moving | 287 Present Current
  290-291 Present Position (low word first) | 296 Input Voltage | 297 lo Temperature
Goal Position 0 = fully open; about 740 = closed (fingers meet). Max Position Limit is 1150.
"""

POS_OPEN = 0
POS_CLOSE = 740
POS_MAX = 740            # GUI never commands beyond this
CURRENT_LIMIT_MA = 661   # gripper EEPROM Current Limit; Goal Current may not exceed it
MOVE_TIMEOUT_S = 8.0

STAGES = {0: '', 1: '포트 열기', 2: '힘(전류) 설정', 3: '속도 설정', 4: '목표 위치 설정',
          5: '토크 켜기', 6: '토크 끄기', 7: '상태 읽기', 8: '안전 확인(토크가 켜져 있음)'}

_DRL_COMMON = r'''
def crc16(d):
    c = 0xFFFF
    for b in d:
        c = c ^ b
        for k in range(8):
            if c & 1:
                c = (c >> 1) ^ 0xA001
            else:
                c = c >> 1
    return [c & 0xFF, c >> 8]

def xfer(req, n):
    for attempt in range(3):
        flange_serial_write(bytes(req))
        got = []
        for k in range(15):
            res, rx = flange_serial_read(0.03)
            if res is not None and res > 0:
                got = got + list(rx)
            if len(got) >= n:
                f = got[len(got) - n:]
                if f[0] == 1 and f[1] == req[1] and crc16(f[:n - 2]) == f[n - 2:]:
                    return f
        wait(0.1)
    return None

def rd(start, count):
    req = [1, 3, start >> 8, start & 0xFF, 0, count]
    req = req + crc16(req)
    f = xfer(req, 5 + 2 * count)
    if f is None:
        return None
    return [(f[3 + 2 * i] << 8) | f[4 + 2 * i] for i in range(count)]

def wr1(reg, val):
    req = [1, 6, reg >> 8, reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
    req = req + crc16(req)
    return xfer(req, 8) is not None

def wr2(reg, v):
    lo = v & 0xFFFF
    hi = (v >> 16) & 0xFFFF
    req = [1, 16, reg >> 8, reg & 0xFF, 0, 2, 4, lo >> 8, lo & 0xFF, hi >> 8, hi & 0xFF]
    req = req + crc16(req)
    return xfer(req, 8) is not None

def put(a, v):
    set_output_register_int(a, v & 0xFF)

def put16(a, v):
    put(a, v & 0xFF)
    put(a + 1, (v >> 8) & 0xFF)

def report():
    t = rd(256, 4)
    m = rd(285, 13)
    g = rd(275, 9)
    if t is None or m is None:
        return False
    put(7, t[0] & 0xFF)
    put(8, t[3] & 0xFF)
    put(6, m[0] & 0xFF)
    put16(4, m[2])
    put16(2, m[5])
    put(10, m[11])
    put(9, m[12] & 0xFF)
    if g is not None:
        put16(12, g[0])
        put16(16, g[5])
        put16(14, g[7])
    return True
'''

_DRL_MAIN = r'''
for a in range(24):
    put(a, 0)
put(23, 7)
put(0, 1)
CMD = "{cmd}"
stage = 0
try:
    stage = 1
    flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
    wait(0.3)
    ok = True
    if CMD == "move":
        stage = 2
        ok = wr1(275, {cur})
        if ok:
            stage = 3
            ok = wr2(280, {vel})
        if ok:
            stage = 4
            ok = wr2(282, {pos})
        if ok:
            stage = 5
            ok = wr1(256, 1)
        if ok:
            wait(0.25)
            still = 1
            for k in range({poll_n}):
                s = rd(285, 1)
                if s is not None and (s[0] & 0xFF) == 0:
                    still = 0
                    break
                wait(0.1)
            put(11, still)
    elif CMD == "torque_off":
        stage = 6
        ok = wr1(256, 0)
    elif CMD == "prepare":
        stage = 8
        t = rd(256, 1)
        ok = t is not None and (t[0] & 0xFF) == 0
        if ok:
            stage = 2
            ok = wr1(275, {cur})
        if ok:
            stage = 3
            ok = wr2(280, {vel})
        if ok:
            stage = 4
            ok = wr2(282, {pos})
    if ok:
        stage = 7
        ok = report()
    flange_serial_close()
    if ok:
        put(1, 0)
        put(0, 2)
    else:
        put(1, stage)
        put(0, 4)
except Exception as e:
    put(1, stage)
    put(0, 3)
    try:
        flange_serial_close()
    except Exception:
        pass
put(22, {token})
'''


def build(cmd, token, pos=0, cur=200, vel=600):
    """DRL source for one gripper command.

    cmd: 'status'      read only
         'move'        set goal current, profile velocity, goal position, then torque ON, wait until stopped
         'torque_off'  release (fingers go limp)
         'prepare'     like move but WITHOUT torque on; refuses if torque is already on (no motion, for tests)
    """
    assert cmd in ('status', 'move', 'torque_off', 'prepare')
    pos = int(max(0, min(POS_MAX, pos)))
    cur = int(max(0, min(CURRENT_LIMIT_MA, cur)))
    vel = int(max(0, min(2970, vel)))
    token = int(token) & 0xFF or 1
    return _DRL_COMMON + _DRL_MAIN.format(cmd=cmd, cur=cur, vel=vel, pos=pos, token=token,
                                          poll_n=int(MOVE_TIMEOUT_S / 0.1))


def reg_byte(v):
    """Decode one general-purpose register value as read through ROS on this controller."""
    if v is None:
        return None
    v &= 0xFFFFFFFF
    return (v >> 24) & 0xFF if v > 0xFF else v


def decode(b):
    """b: list of 24 decoded bytes (R0..R23) -> dict."""
    def u16(i):
        return b[i] | (b[i + 1] << 8)

    def s16(i):
        v = u16(i)
        return v - 0x10000 if v & 0x8000 else v
    return {
        'status': b[0], 'stage': b[1], 'stage_name': STAGES.get(b[1], str(b[1])),
        'position': u16(2), 'current_mA': s16(4), 'moving': b[6], 'torque': b[7],
        'hw_error': b[8], 'temperature_C': b[9], 'voltage_V': b[10] / 10.0, 'timed_out': b[11],
        'goal_current_mA': u16(12), 'goal_position': u16(14), 'profile_velocity': u16(16),
        'token': b[22], 'marker': b[23],
    }
