
# ============================================================
# Project 4 - STEP 2
# Whisper AI를 이용한 한국어 음성인식 프로그램
#
# [프로그램 동작 순서]
# 1. Whisper AI 모델 로딩
# 2. Windows 노트북 마이크로 10초 녹음
# 3. 녹음된 음성을 NumPy 배열로 처리
# 4. Whisper AI를 이용해 한국어 텍스트로 변환
# 5. 음성인식 결과를 PowerShell에 출력
# 6. Enter 키를 누르면 다시 녹음, q를 입력하면 종료
# ============================================================


# ============================================================
# 1. 필요한 Python 라이브러리 불러오기
# ============================================================

# sounddevice:
# 마이크에서 소리를 녹음하거나
# 스피커로 소리를 출력할 수 있는 라이브러리
#
# as sd:
# sounddevice라는 긴 이름 대신 sd라는 별칭 사용
import sounddevice as sd


# numpy:
# 숫자 배열을 빠르게 계산하고 처리하는 라이브러리
#
# 녹음된 음성도 실제로는 숫자로 이루어진 배열임
#
# as np:
# numpy를 np라는 별칭으로 사용
import numpy as np


# faster_whisper:
# Whisper AI 음성인식 모델을 효율적으로 실행하는 라이브러리
#
# from A import B:
# A 모듈에서 B라는 클래스 또는 기능을 가져옴
#
# WhisperModel:
# 음성인식 모델을 생성하고 실행하는 클래스
from faster_whisper import WhisperModel


# ============================================================
# 2. 음성 녹음 환경 설정
# ============================================================

# SAMPLE_RATE:
# Sampling Rate(샘플링 주파수)
#
# 16000Hz = 1초 동안 소리를 16,000번 측정
#
# Whisper는 내부적으로 16kHz 오디오를 사용함
SAMPLE_RATE = 16000


# DURATION:
# 마이크로 음성을 녹음할 시간
#
# 단위: 초(second)
DURATION = 10


# MODEL_SIZE:
# 사용할 Whisper AI 모델 크기
#
# tiny   : 매우 작은 모델, 빠르지만 정확도가 낮을 수 있음
# base   : 작은 모델, 초기 음성인식 테스트에 적합
# small  : base보다 큰 모델
# medium : 더 큰 모델, CPU에서는 느릴 수 있음
# large  : 큰 모델, 높은 연산 자원이 필요
#
# 이번 프로젝트에서는 base 사용
MODEL_SIZE = "base"


# ============================================================
# 3. Whisper AI 모델 생성
# ============================================================

# print():
# 화면(터미널)에 문자열을 출력하는 Python 함수
print("[INFO] Whisper AI 모델 로딩 중...")


# WhisperModel():
# 실제 Whisper AI 모델 객체 생성
#
# MODEL_SIZE:
# 사용할 모델 크기(base)
#
# device="cpu":
# AI 연산을 CPU에서 수행
#
# compute_type="int8":
# 모델 연산에 8비트 정수 양자화 방식 사용
# 일반적으로 모델 메모리 사용량을 줄이는 데 도움
#
# 최초 실행 시 모델 다운로드가 필요할 수 있음
model = WhisperModel(
    MODEL_SIZE,
    device="cpu",
    compute_type="int8"
)


print("[INFO] Whisper AI 모델 준비 완료!")


# ============================================================
# 4. 마이크 녹음 함수
# ============================================================

# def:
# Python에서 함수를 정의하는 키워드
#
# record_audio():
# 마이크에서 음성을 녹음하고
# 녹음된 음성 데이터를 반환하는 함수
def record_audio():

    # \n:
    # 줄바꿈을 의미하는 특수 문자
    print("\n[MIC] 10초 동안 말해보세요!")


    # sd.rec():
    # 마이크에서 음성을 녹음하는 함수
    #
    # 첫 번째 인자:
    # 녹음할 전체 샘플 개수
    #
    # 16000 x 5 = 80000개
    #
    # int():
    # 계산 결과를 정수 형태로 변환
    #
    # samplerate:
    # 1초당 녹음할 샘플 개수
    #
    # channels=1:
    # 모노(Mono), 오디오 채널 1개
    #
    # dtype="float32":
    # 각 음성 샘플을 32비트 부동소수점으로 저장
    audio = sd.rec(
        int(SAMPLE_RATE * DURATION),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32"
    )


    # sd.wait():
    # 녹음이 끝날 때까지 프로그램 실행 대기
    #
    # 이 코드가 없으면 녹음 완료 전에
    # 다음 코드가 실행될 수 있음
    sd.wait()


    print("[MIC] 녹음 완료!")


    # np.squeeze():
    # NumPy 배열에서 크기가 1인 차원을 제거
    #
    # 기존 녹음 데이터:
    # (80000, 1)
    #
    # 변환 후:
    # (80000,)
    #
    # Whisper에 1차원 모노 오디오 배열 전달
    audio = np.squeeze(audio)


    # return:
    # 함수에서 처리한 결과를 호출한 곳으로 반환
    #
    # 여기서는 녹음한 음성 배열을 반환
    return audio


# ============================================================
# 5. Whisper AI 음성인식 함수
# ============================================================

# recognize_voice(audio):
#
# audio:
# 앞에서 녹음한 음성 데이터를 전달받는 매개변수
#
# 반환값:
# AI가 인식한 한국어 문자열
def recognize_voice(audio):

    print("[AI] 음성 분석 중...")


    # model.transcribe():
    # Whisper AI를 이용해 음성 데이터를 텍스트로 변환
    #
    # audio:
    # NumPy 형태의 음성 데이터
    #
    # language="ko":
    # 한국어로 음성인식 수행
    #
    # beam_size=5:
    # Beam Search 탐색 폭을 5로 설정
    #
    # 여러 텍스트 후보를 비교하면서
    # 가능성이 높은 인식 결과를 탐색
    #
    # condition_on_previous_text=False:
    # 이전 구간의 인식 텍스트를
    # 다음 구간의 문맥 입력으로 사용하지 않음
    #
    # segments:
    # 인식된 텍스트 구간의 반복 가능한 객체
    #
    # info:
    # 언어 등 음성인식 부가 정보
    segments, info = model.transcribe(
        audio,
        language="ko",
        beam_size=5,
        condition_on_previous_text=False
    )


    # Whisper는 긴 음성을 여러 구간으로 나눠
    # 결과를 반환할 수 있음
    #
    # for segment in segments:
    # segments에 들어 있는 각 구간을 순서대로 처리
    #
    # segment.text:
    # 해당 구간에서 인식된 문자열
    #
    # strip():
    # 문자열 앞뒤의 불필요한 공백 제거
    #
    # " ".join(...):
    # 여러 문자열을 공백 하나로 연결
    #
    # 이 과정에서 음성인식이 실제 수행됨
    # segments는 지연 처리되는 반복 객체이기 때문
    text = " ".join(
        segment.text.strip()
        for segment in segments
    ).strip()


    # 최종 한국어 인식 결과 반환
    return text


# ============================================================
# 6. 프로그램 메인 함수
# ============================================================

# main():
# 프로그램의 전체 실행 흐름을 관리하는 함수
def main():

    print("\n===== Whisper Voice Recognition =====")


    # while True:
    # 조건이 참인 동안 계속 반복
    #
    # 사용자가 q를 입력할 때까지 반복 실행
    while True:


        # input():
        # 키보드 입력을 기다리는 함수
        #
        # Enter를 누르면 빈 문자열("") 반환
        # q를 입력하면 "q" 문자열 반환
        command = input(
            "\n[ENTER] 녹음 시작 / q 종료 : "
        )


        # command.lower():
        # 입력한 영문자를 소문자로 변환
        #
        # 예:
        # "Q" -> "q"
        #
        # == :
        # 두 값이 같은지 비교
        if command.lower() == "q":

            print("[INFO] 프로그램 종료")

            # break:
            # 현재 반복문을 즉시 종료
            break


        # try:
        # 오류가 발생할 수 있는 코드를 실행
        #
        # 오류가 발생하면 except 블록으로 이동
        try:

            # 마이크 녹음 함수 호출
            #
            # 반환된 NumPy 음성 배열을
            # audio 변수에 저장
            audio = record_audio()


            # recognize_voice():
            # 녹음된 오디오를 AI로 분석
            #
            # 반환된 한국어 문장을
            # result 변수에 저장
            result = recognize_voice(audio)


            # 화면에 결과 출력
            print("\n==============================")

            # f-string:
            # 문자열 안에 변수 값을 삽입
            #
            # {result} 위치에 인식된 문장이 들어감
            print(f"[STT RESULT] {result}")

            print("==============================")


            # not result:
            # 문자열이 비어 있으면 True
            #
            # 즉, 아무 문장도 인식하지 못한 경우
            if not result:

                print(
                    "[WARN] 음성이 인식되지 않았습니다."
                )


        # except Exception as e:
        # 프로그램 실행 중 예외가 발생했을 때 실행
        #
        # e에는 발생한 오류 정보가 들어감
        except Exception as e:

            print(f"[ERROR] {e}")


# ============================================================
# 7. 프로그램 시작 지점
# ============================================================

# __name__:
# Python이 자동으로 설정하는 특수 변수
#
# 이 파일을 직접 실행하면:
# __name__ == "__main__"
#
# 다른 Python 파일에서 import하면:
# __name__은 모듈 이름이 됨
#
# 따라서 이 조건은 파일을 직접 실행할 때만
# main() 함수를 호출하도록 만드는 역할
if __name__ == "__main__":

    # main 함수 실행
    main()
