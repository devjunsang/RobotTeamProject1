"""Offline test of gripper_drl: run the generated DRL against a simulated Modbus-RTU RH-P12-RN(A)."""
import sys
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from e0509_manual import gripper_drl as G


def crc(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return [c & 0xFF, c >> 8]


class Gripper:
    def __init__(self, dead=False, fn16=True, slow=False):
        self.r = {0: 0x8902, 3: 0x010F, 4: 0xFA01, 5: 0x0500, 6: 0x0AFF, 256: 0, 259: 0, 275: 0, 280: 0,
                  281: 0, 282: 0, 283: 0, 285: 0, 287: 7, 290: 701, 291: 0, 296: 238, 297: 46}
        self.dead, self.fn16, self.slow = dead, fn16, slow
        self.q, self.moves, self.writes = [], 0, []

    def tick(self):  # gripper physics: one step toward goal per read when torque on
        if self.r[256] & 1 and self.r[275] > 0:
            goal, p = self.r[282], self.r[290]
            step = 5 if self.slow else 200
            if p != goal:
                self.r[290] = p + max(-step, min(step, goal - p)); self.r[285] = 1; self.moves += 1
            else:
                self.r[285] = 0

    def rx(self, d):
        d = list(d)
        if self.dead or crc(d[:-2]) != d[-2:] or d[0] != 1:
            return
        fn, reg = d[1], (d[2] << 8) | d[3]
        if fn == 3:
            self.tick()
            n = d[5]; body = [1, 3, 2 * n]
            for i in range(n):
                v = self.r.get(reg + i, 0); body += [v >> 8, v & 0xFF]
            self.q += body + crc(body)
        elif fn == 6:
            self.r[reg] = (d[4] << 8) | d[5]; self.writes.append((reg, self.r[reg])); self.q += d
        elif fn == 16 and self.fn16:
            n = d[5]
            for i in range(n):
                self.r[reg + i] = (d[7 + 2 * i] << 8) | d[8 + 2 * i]; self.writes.append((reg + i, self.r[reg + i]))
            body = d[:6]; self.q += body + crc(body)
        elif fn == 16:
            body = [1, 0x90, 1]; self.q += body + crc(body)


def run(src, g):
    regs = {}
    def read(t):
        if not g.q: return -1, None
        c, g.q[:] = g.q[:6], g.q[6:]; return len(c), bytes(c)
    env = dict(flange_serial_open=lambda **k: None, flange_serial_write=g.rx, flange_serial_read=read,
               flange_serial_close=lambda: None, wait=lambda s: None,
               set_output_register_int=lambda a, v: regs.__setitem__(a, (v & 0xFF) << 24 if v else 0),
               DR_EIGHTBITS=8, DR_PARITY_NONE=0, DR_STOPBITS_ONE=1)
    exec(src, env)
    return G.decode([G.reg_byte(regs.get(a, 0)) for a in range(24)])


fails = 0
def check(name, cond, info=''):
    global fails
    print(('PASS ' if cond else 'FAIL ') + name + ('' if cond else f'   {info}'))
    fails += 0 if cond else 1

g = Gripper(); d = run(G.build('status', 11), g)
check('status: done, token, values', d['status'] == 2 and d['token'] == 11 and d['position'] == 701 and
      d['voltage_V'] == 23.8 and d['temperature_C'] == 46 and d['torque'] == 0 and d['current_mA'] == 7, d)
check('status: no writes', g.writes == [], g.writes)

g = Gripper(); d = run(G.build('prepare', 12, pos=300, cur=150, vel=600), g)
check('prepare: writes goal current/vel/pos, torque stays off, no motion',
      d['status'] == 2 and d['torque'] == 0 and d['goal_position'] == 300 and d['goal_current_mA'] == 150
      and d['profile_velocity'] == 600 and g.moves == 0 and d['position'] == 701, (d, g.writes))

g = Gripper(); g.r[256] = 1; d = run(G.build('prepare', 13, pos=300), g)
check('prepare: refuses when torque already on', d['status'] == 4 and d['stage'] == 8 and g.writes == [], (d, g.writes))

g = Gripper(); d = run(G.build('move', 14, pos=0, cur=200, vel=600), g)
check('move open: reaches 0, torque on, not timed out', d['status'] == 2 and d['position'] == 0 and d['torque'] == 1
      and d['timed_out'] == 0 and d['moving'] == 0, d)
check('move: order current -> vel -> pos -> torque', [w[0] for w in g.writes] == [275, 280, 281, 282, 283, 256], g.writes)

g = Gripper(slow=True); d = run(G.build('move', 15, pos=0), g)
check('move slow: timed_out flag set', d['status'] == 2 and d['timed_out'] == 1, d)

g = Gripper(); d = run(G.build('move', 16, pos=5000, cur=5000, vel=99999), g)
check('move: clamps pos<=740, current<=661, vel<=2970', d['goal_position'] == 740 and d['goal_current_mA'] == 661
      and d['profile_velocity'] == 2970, d)

g = Gripper(); g.r[256] = 1; d = run(G.build('torque_off', 17), g)
check('torque_off: torque 0', d['status'] == 2 and d['torque'] == 0 and g.writes == [(256, 0)], (d, g.writes))

g = Gripper(dead=True); d = run(G.build('move', 18, pos=0), g)
check('dead gripper: status 4 at stage 2, nothing else', d['status'] == 4 and d['stage'] == 2 and d['token'] == 18, d)

g = Gripper(fn16=False); d = run(G.build('move', 19, pos=0), g)
check('fn16 unsupported: fails at stage 3 (velocity), torque never turned on', d['status'] == 4 and d['stage'] == 3
      and g.r[256] == 0, d)

g = Gripper(); g.r[287] = 0xFFF6; d = run(G.build('status', 20), g)
check('negative current decodes', d['current_mA'] == -10, d)
print('FAILED' if fails else 'ALL PASS', fails)
