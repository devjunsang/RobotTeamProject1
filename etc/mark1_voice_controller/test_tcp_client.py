
# TCP 네트워크 통신 라이브러리
import socket

# JSON 데이터 변환 라이브러리
import json

# Ubuntu 노트북의 IPv4 주소
HOST = "192.168.219.115"

# mDNS 호스트 이름으로 변경
# HOST = "jun.local"

# Ubuntu TCP 서버 포트
PORT = 5000

# Project 4에서 생성할 음성 명령 예시
command = {
    "target": "triangle",
    "color": "red",
    "action": "pick"
}

# Python 딕셔너리를 JSON 문자열로 변환
# Ubuntu 서버는 줄 단위로 읽으므로 개행문자 추가
message = json.dumps(command, ensure_ascii=False) + "\n"

try:
    # Ubuntu TCP 서버 연결
    with socket.create_connection((HOST, PORT), timeout=5) as client:

        # 서버 응답 대기시간을 5초로 설정
        client.settimeout(5)

        # UTF-8 인코딩 후 JSON 명령 전송
        client.sendall(message.encode("utf-8"))

        # 전송한 명령 출력
        print("[TCP SEND]", command)

        # Ubuntu 서버에서 보낸 응답 수신
        response = client.makefile("rb").readline()

        # 응답 내용을 UTF-8 문자열로 변환해 출력
        print("[TCP RESPONSE]", response.decode("utf-8").strip())

# 네트워크 연결 오류 처리
except OSError as e:
    print("[TCP ERROR]", e)
