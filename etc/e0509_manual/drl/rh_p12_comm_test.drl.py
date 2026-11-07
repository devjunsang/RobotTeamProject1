# RH-P12-RN(A) status read over the E0509 flange RS-485 (DRL, for the teach pendant "Custom Code")
#
# READ-ONLY: does NOT move the robot arm or the gripper fingers.
# Verified 2026-11-07 on this robot: gripper is Modbus-RTU, 57600 bps 8N1, slave ID 1.
# Use ONLY 57600 bps. Probing other baud rates once left the flange link dead until the robot was
# reconnected (servo off -> on).

def crc16(data):
    c = 0xFFFF
    for b in data:
        c = c ^ b
        for k in range(8):
            if c & 1:
                c = (c >> 1) ^ 0xA001
            else:
                c = c >> 1
    return [c & 0xFF, c >> 8]


def mb_read(start, count):
    req = [0x01, 0x03, start >> 8, start & 0xFF, 0x00, count]
    req = req + crc16(req)
    for attempt in range(3):
        flange_serial_write(bytes(req))
        got = []
        for k in range(10):
            res, rx = flange_serial_read(0.05)
            if res is not None and res > 0:
                got = got + list(rx)
        if got[:len(req)] == req:
            got = got[len(req):]
        n = 5 + 2 * count
        if len(got) >= n and got[1] == 0x03 and crc16(got[:n - 2]) == got[n - 2:n]:
            return [(got[3 + 2 * i] << 8) | got[4 + 2 * i] for i in range(count)]
        wait(0.1)
    return None


flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
wait(0.5)
info = mb_read(0, 7)        # 40001..40007: model, model info, fw/ID, baud/RDT, drive/op mode, 2nd ID/protocol
pos = mb_read(290, 2)       # 40291..40292: present position (low word, high word)
volt = mb_read(296, 2)      # 40297..40298: input voltage (0.1 V), temperature
flange_serial_close()

if info is None:
    tp_popup("Gripper: NO REPLY (check cable/power; reconnect robot)", DR_PM_MESSAGE)
else:
    msg = "model %d fw %d ID %d baud# %d mode %d protocol %d" % (
        info[0], info[3] & 0xFF, info[3] >> 8, info[4] & 0xFF, info[5] >> 8, info[6] >> 8)
    if pos is not None:
        msg = msg + " | pos %d" % (pos[0] | (pos[1] << 16))
    if volt is not None:
        msg = msg + " | %.1f V %d C" % (volt[0] / 10.0, volt[1] & 0xFF)
    tp_log(msg)
    tp_popup("Gripper OK: " + msg, DR_PM_MESSAGE)
