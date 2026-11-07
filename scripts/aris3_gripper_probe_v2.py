#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ARIS3 / E0509 X1 플랜지 / RH-P12-RN 읽기 전용 첫 통신 시험.

실제 이동을 명령하지 않는다. DYNAMIXEL Protocol 2.0 READ(주소 0, 2바이트)만 전송한다.
DRL 실행의 응답은 작업 시작 여부만 알려주므로, E0509가 TCP로 반환한 바이트를
Ubuntu 노트북에서 받아 검증한다. Doosan 드라이버의 C++ 수정이나 빌드는 필요 없다.
"""

import socket
import sys

import rclpy
from dsr_msgs2.srv import DrlStart, GetRobotMode, GetRobotState

PC_IP = "110.120.1.67"       # 기존 유선 LAN enp3s0 주소
PORT = 20092                # 로봇→노트북 진단용 TCP 포트
DYNAMIXEL_ID = 1           # 출고 기본값; 변경된 장치는 응답하지 않을 수 있음


def crc16_dxl(data: bytes) -> int:
    """DYNAMIXEL Protocol 2.0 CRC-16 (다항식 0x8005, 초기값 0)."""
    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x8005) if crc & 0x8000 else (crc << 1)) & 0xFFFF
    return crc


def read_model_packet(device_id: int) -> bytes:
    """모터를 움직이지 않는 READ 패킷: 주소 0 / 길이 2."""
    packet = bytes([0xFF, 0xFF, 0xFD, 0x00, device_id,
                    0x07, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00])
    return packet + crc16_dxl(packet).to_bytes(2, "little")


def build_drl(packet: bytes) -> str:
    """DRL은 실제 로봇 제어기에서 실행된다. 위치 및 토크 쓰기 명령은 없다."""
    return f'''# ARIS3: RH-P12-RN 읽기 전용 모델 번호 요청
# 이 소켓은 노트북에 진단 결과를 전달하는 용도임
var_sock = client_socket_open("{PC_IP}", {PORT})
var_open = flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
if var_open == 0:
    wait(0.1)
    # DRL: X1(port=1)에 DYNAMIXEL 읽기 패킷을 보낸 뒤 송신 결과를 먼저 보고한다.
    # Doosan DRL 반환값: 0=전송 성공, 비정상 값=전송 실패.
    var_tx = flange_serial_write({packet!r}, 1)
    client_socket_write(var_sock, ("TX_RESULT:" + str(var_tx) + ";").encode())
    if var_tx == 0:
        # 구형 Pseudo Flange Serial의 처리 지연을 고려한다.
        # 이 요청은 READ 명령이며 그리퍼 구동 명령을 포함하지 않는다.
        wait(0.25)
        var_count, var_data = flange_serial_read(2.0, 1)
        if var_count > 0:
            client_socket_write(var_sock, b"RX:" + var_data)
        else:
            client_socket_write(var_sock, ("READ_FAIL:" + str(var_count)).encode())
    else:
        client_socket_write(var_sock, ("WRITE_FAIL:" + str(var_tx)).encode())
    flange_serial_close()
else:
    client_socket_write(var_sock, ("OPEN_FAIL:" + str(var_open)).encode())
client_socket_close(var_sock)
'''


def call_read_only(node, srv_type, name):
    """조회 서비스 호출: 상태를 변경하는 명령은 전송하지 않는다."""
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=4.0):
        raise RuntimeError(f"서비스 미발견: {name}")
    future = client.call_async(srv_type.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"응답 시간 초과: {name}")
    return future.result()


def parse_received(reply: bytes):
    """실제로 수신한 프레임만 검증. 실패는 전원 이상이라고 단정하지 않는다."""
    print("[수신 원본]", reply.hex(" "))
    # 첫 요청이 플랜지에 실제로 써졌는지 송신 함수 반환값을 확인한다.
    if reply.startswith(b"TX_RESULT:"):
        header, separator, payload = reply.partition(b";")
        print("[플랜지 송신 결과]", header.decode(errors="replace"))
        if not separator:
            print("[실패] 송신 메타데이터가 불완전해.")
            return
        reply = payload
    if reply.startswith(b"OPEN_FAIL:"):
        print("[실패] E0509 플랜지 시리얼 포트를 열지 못했어:", reply.decode(errors="replace"))
        return
    if reply.startswith(b"WRITE_FAIL:"):
        print("[실패] 플랜지 패킷 송신 실패:", reply.decode(errors="replace"))
        return
    if reply.startswith(b"READ_FAIL:"):
        print("[실패]", "수신 시간 초과" if reply == b"READ_FAIL:-1" else "수신 오류", reply.decode(errors="replace"))
        return
    if not reply.startswith(b"RX:"):
        print("[실패] 알 수 없는 응답이야. DRL/네트워크 로그를 확인해 줘.")
        return

    frame = reply[3:]
    if len(frame) < 11 or frame[:4] != b"\xff\xff\xfd\x00":
        print("[미확인] DYNAMIXEL 상태 패킷 헤더가 없거나 데이터가 불완전해.")
        return
    expected_len = 7 + frame[5] + (frame[6] << 8)
    if len(frame) != expected_len:
        print(f"[미확인] 패킷 길이 불일치: 수신 {len(frame)}, 예상 {expected_len}")
        return
    if frame[4] != DYNAMIXEL_ID or frame[7] != 0x55:
        print("[미확인] 장치 ID 또는 Status Instruction 불일치")
        return
    if crc16_dxl(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
        print("[미확인] CRC 검증 실패")
        return
    if frame[8] != 0:
        print(f"[실패] 그리퍼에서 오류 응답: 0x{frame[8]:02X}")
        return
    params = frame[9:-2]
    if len(params) != 2:
        print(f"[미확인] 모델 번호 길이 오류: {len(params)}바이트")
        return
    model = int.from_bytes(params, "little")
    print(f"[검증] 장치 모델 번호 = {model}")
    if model == 35073:
        print("[성공] RH-P12-RN 읽기 통신이 정상적으로 확인됐어!")
    elif model == 35074:
        print("[확인] RH-P12-RN(A) 펌웨어 모델이 응답했어. 제어 테이블을 재검토해야 해.")
    else:
        print("[주의] 예상과 다른 모델. 이동 명령을 보내지 마.")


def main():
    # 앞서 확인한 2023년형 E0509 로봇과 노트북의 직접 연결을 전제로 한다.
    rclpy.init()
    node = rclpy.create_node("aris3_gripper_probe")
    try:
        state = call_read_only(node, GetRobotState, "/dsr01/system/get_robot_state")
        mode = call_read_only(node, GetRobotMode, "/dsr01/system/get_robot_mode")
        print(f"[E0509] robot_state={state.robot_state}, robot_mode={mode.robot_mode}")
        # 실제 전송할 READ 패킷을 사람이 검증할 수 있도록 16진수로 표시한다.
        print("[DYNAMIXEL READ 패킷]", read_model_packet(DYNAMIXEL_ID).hex(" "))
        if not state.success or not mode.success or state.robot_state != 1 or mode.robot_mode != 1:
            raise RuntimeError("STANDBY(1) / AUTO(1) 상태가 아니므로 DRL을 시작하지 않아.")

        # 수신 서버를 먼저 열어야 DRL의 client_socket_open()이 연결될 수 있다.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((PC_IP, PORT))
            server.listen(1)
            server.settimeout(12.0)
            print(f"[수신 대기] {PC_IP}:{PORT} (E0509에서 결과 전송 예정)")

            drl = node.create_client(DrlStart, "/dsr01/drl/drl_start")
            if not drl.wait_for_service(timeout_sec=4.0):
                raise RuntimeError("DRL 시작 서비스를 찾지 못했어.")
            request = DrlStart.Request()
            request.robot_system = 0  # 공식 상수 ROBOT_SYSTEM_REAL
            request.code = build_drl(read_model_packet(DYNAMIXEL_ID))
            future = drl.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
            if not future.done() or future.result() is None or not future.result().success:
                raise RuntimeError("DRL 프로그램 시작 실패. 실물 드라이버 로그 확인 필요")
            print("[DRL] 읽기 전용 진단 프로그램 시작 요청 수락")

            with server.accept()[0] as conn:
                conn.settimeout(5.0)
                chunks = []
                while True:
                    data = conn.recv(256)
                    if not data:
                        break
                    chunks.append(data)
                parse_received(b"".join(chunks))
    except (RuntimeError, OSError, socket.timeout) as exc:
        print(f"[오류] {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

# 실행 흐름 요약:
# 1. E0509가 STANDBY/AUTO인지 조회한다.
# 2. Ubuntu 110.120.1.67:20092에 TCP 수신 서버를 생성한다.
# 3. DRL 스크립트를 로봇 제어기에 전달한다.
# 4. 플랜지 시리얼을 57600bps로 열고, 송신 반환값(TX_RESULT)을 확인한다.
# 5. 송신 성공시에만 모델 번호 2바이트를 요청해 최대 2초 동안 수신한다.
# 6. DRL이 응답 원본을 Ubuntu에 전달하고, CRC 및 모델 번호를 검증한다.
# 7. 토크·목표 위치·전원 출력에 대한 쓰기 요청은 수행하지 않는다.
