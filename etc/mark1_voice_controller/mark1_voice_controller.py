
# ============================================================
# PROJECT 4 - MARK1 VOICE CONTROL SYSTEM
#
# VERSION 1.1
#
# SPACE 누름 : 녹음 시작
# SPACE 유지 : 계속 녹음
# SPACE 해제 : 녹음 종료 및 Whisper 분석
# Q          : 프로그램 종료 (대기 중)
#
# 지원 Action: PICK / PLACE
# ============================================================

import threading

# 키보드 입력 이벤트 감지
from pynput import keyboard

# Whisper 음성인식 모듈
from voice_recognition import (
    create_model,
    record_audio,
    recognize_voice
)

# 명령 분석 모듈
from command_parser import (
    parse_command,
    validate_command
)

# Project 5 - Ubuntu ROS 2 TCP 통신 모듈
from tcp_client import send_command

# ============================================================
# 1. MARK1 시작 배너
# ============================================================

def print_banner():

    banner = r"""
  __  __    _    ____  _  __  _
 |  \/  |  / \  |  _ \| |/ / / |
 | |\/| | / _ \ | |_) | ' /  | |
 | |  | |/ ___ \|  _ <| . \  | |
 |_|  |_/_/   \_\_| \_\_|\_\ |_|

================================================
          MARK1 VOICE CONTROL SYSTEM
------------------------------------------------
  ROBOT     : MARK1
  AI ENGINE : WHISPER
  ACTIONS   : PICK / PLACE
  MODE      : PUSH TO TALK
================================================
"""

    print(banner)


# ============================================================
# 2. 명령 결과 출력
# ============================================================

def print_command_result(command, valid, message):

    print("\n===== MARK1 COMMAND RESULT =====")

    # 실제 Whisper 인식 결과
    print(f"Original   : {command['original_text']}")

    # 오인식 보정 후 결과
    print(f"Normalized : {command['normalized_text']}")

    # 분석된 명령
    print(f"Target     : {command['target']}")
    print(f"Color      : {command['color']}")
    print(f"Action     : {command['action']}")

    if valid:
        print(f"[SUCCESS] {message}")
    else:
        print(f"[WARN] {message}")

    print("================================")


# ============================================================
# 3. 메인 함수
# ============================================================

def main():

    # MARK1 로고 출력
    print_banner()

    # Whisper 모델 초기화
    print("[INFO] Whisper AI 모델 로딩 중...")

    try:
        model = create_model()

    except Exception as e:
        print(f"[ERROR] 모델 로딩 실패: {e}")
        return

    print("[INFO] MARK1 시스템 준비 완료!")


    # --------------------------------------------------------
    # 키보드 이벤트 객체
    # --------------------------------------------------------
    # Event는 스레드 사이에서 신호를 전달하는 객체
    #
    # press_event   : 스페이스바를 눌렀다는 신호
    # release_event : 스페이스바를 뗐다는 신호
    # exit_event    : 프로그램 종료 신호
    # accepting     : 새 녹음 명령을 받을 수 있는 상태
    # --------------------------------------------------------

    press_event = threading.Event()
    release_event = threading.Event()
    exit_event = threading.Event()
    accepting = threading.Event()

    # 스페이스바가 현재 눌린 상태인지 저장
    key_state = {"pressed": False}


    # ========================================================
    # 4. 키보드 누름 이벤트
    # ========================================================

    def on_press(key):

        # 음성 분석 중에는 새 명령을 받지 않음
        if not accepting.is_set():
            return

        # SPACE 키를 처음 누른 경우
        if key == keyboard.Key.space:

            # 키를 계속 누르고 있을 때 발생하는
            # 반복 입력을 무시
            if not key_state["pressed"]:

                key_state["pressed"] = True

                # 녹음 시작 신호 전달
                press_event.set()

        else:

            # Q를 누르면 종료 신호 전달
            try:
                if key.char.lower() == "q":
                    exit_event.set()
            except AttributeError:
                pass


    # ========================================================
    # 5. 키보드 해제 이벤트
    # ========================================================

    def on_release(key):

        if key == keyboard.Key.space:

            # 스페이스바가 눌렸던 상태라면
            if key_state["pressed"]:

                key_state["pressed"] = False

                # 녹음 종료 신호 전달
                release_event.set()


    # ========================================================
    # 6. 키보드 Listener 시작
    # ========================================================

    # Listener는 메인 실행 흐름과 별개로
    # 키보드 누름/해제 이벤트를 감시
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release
    )

    listener.start()

    print("[KEY] SPACE 누르고 말하기")
    print("[KEY] SPACE 떼면 자동 분석")
    print("[KEY] 대기 중 Q를 누르면 종료")


    # ========================================================
    # 7. 메인 반복문
    # ========================================================

    try:

        while not exit_event.is_set():

            # 이전 녹음에서 사용한 이벤트 초기화
            press_event.clear()
            release_event.clear()

            # 새 키보드 명령 수신 허용
            accepting.set()

            print("\n[READY] SPACE 키를 누르세요.")

            # SPACE가 눌리거나 종료 신호가 올 때까지 대기
            while not press_event.wait(timeout=0.05):

                if exit_event.is_set():
                    break

            # 종료 신호가 있다면 반복문 종료
            if exit_event.is_set():
                break

            # 음성인식 중 새로운 녹음 요청 차단
            accepting.clear()

            try:

                # --------------------------------------------
                # STEP 1. 스페이스바 해제까지 녹음
                # --------------------------------------------
                audio = record_audio(release_event)

                # 너무 짧은 녹음은 분석하지 않음
                if audio.size < 1600:
                    print("[WARN] 녹음 시간이 너무 짧습니다.")
                    continue

                # --------------------------------------------
                # STEP 2. Whisper 음성인식
                # --------------------------------------------
                text = recognize_voice(model, audio)

                print(f"\n[STT RESULT] {text}")

                if not text:
                    print("[WARN] 음성이 인식되지 않았습니다.")
                    continue

                # --------------------------------------------
                # STEP 3. Command Parser
                # --------------------------------------------
                command = parse_command(text)

                # --------------------------------------------
                # STEP 4. 명령 검증
                # --------------------------------------------
                valid, message = validate_command(command)

                # --------------------------------------------
                # STEP 5. 명령 분석 결과 출력
                # --------------------------------------------
                print_command_result(
                    command,
                    valid,
                    message
                )

                # --------------------------------------------
                # STEP 6. Ubuntu ROS 2로 명령 전송
                # --------------------------------------------

                # 명령이 유효할 때만 TCP 전송
                if valid:

                    print("[TCP] Ubuntu ROS 2로 명령 전송 중...")

                    # 현재 음성 명령을 Ubuntu에 전송
                    success = send_command(command)

                    # 전송 결과 확인
                    if success:
                        print("[SUCCESS] 음성 명령 전송 성공")
                    else:
                        print("[WARN] 음성 명령 전송 실패")

                else:
                    # 잘못된 명령은 로봇팔에 전달하지 않음
                    print("[TCP] 유효하지 않은 명령 - 전송 취소")

                # 향후 ROS 2 송신 기능 추가 예정

            except Exception as e:
                print(f"[ERROR] 명령 처리 실패: {e}")

    # Ctrl+C로도 종료 가능
    except KeyboardInterrupt:
        print("\n[INFO] 사용자 종료 요청")

    finally:

        # 새 입력 차단
        accepting.clear()

        # 대기 중인 녹음이 있다면 해제
        release_event.set()

        # 키보드 이벤트 감시 종료
        listener.stop()

        print("\n[INFO] MARK1 시스템 종료")


# ============================================================
# 8. 프로그램 실행 진입점
# ============================================================

if __name__ == "__main__":
    main()
