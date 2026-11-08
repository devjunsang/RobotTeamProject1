
# ============================================================
# PROJECT 5 - MARK1 TCP CLIENT
# Windows 음성인식 명령을 Ubuntu ROS 2로 전달
# ============================================================

# TCP/IP 통신 라이브러리
import socket

# Python 딕셔너리를 JSON 문자열로 변환하는 라이브러리
import json


# Ubuntu 노트북의 IP 주소 (현재 하드코딩 방식)
HOST = "192.168.204.62"

# Ubuntu TCP 수신 서버 포트
PORT = 5000


# 분석된 음성 명령을 Ubuntu로 전송하는 함수
def send_command(command):

    # ROS 2로 전달할 필드만 선택
    # 원본 문장과 정규화 문장은 전송하지 않음
    data = {
        "target": command["target"],
        "color": command["color"],
        "action": command["action"]
    }

    # 딕셔너리를 JSON 문자열로 변환
    # Ubuntu 서버가 줄 단위로 읽으므로 개행문자 추가
    message = json.dumps(
        data,
        ensure_ascii=False
    ) + "\n"

    try:
        # Ubuntu TCP 서버에 연결
        with socket.create_connection(
            (HOST, PORT),
            timeout=5
        ) as client:

            # 서버 응답 대기시간 설정
            client.settimeout(5)

            # UTF-8로 인코딩한 명령 전송
            client.sendall(message.encode("utf-8"))

            # Ubuntu TCP 서버의 수신 확인 응답 읽기
            response = client.makefile("rb").readline()

            # 응답이 비어 있다면 통신 실패 처리
            if not response:
                print("[TCP ERROR] 서버 응답 없음")
                return False

            # JSON 응답을 Python 딕셔너리로 변환
            result = json.loads(response.decode("utf-8"))

            # 서버에서 정상 수신 응답을 보냈는지 확인
            if result.get("status") == "received":
                print("[TCP SUCCESS] ROS 2 명령 전달 완료")
                return True

            # 서버가 명령을 거부한 경우
            print("[TCP ERROR] 서버 응답:", result)
            return False

    # 네트워크 및 JSON 응답 처리 오류
    except (OSError, ValueError, UnicodeError) as e:
        print(f"[TCP ERROR] {e}")
        return False
