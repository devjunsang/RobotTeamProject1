
# ============================================================
# MARK1 - Voice Recognition
# Push-to-Talk 음성 녹음 + Whisper STT
# ============================================================

import sounddevice as sd
import numpy as np
from faster_whisper import WhisperModel


# Whisper 입력 샘플링 주파수
SAMPLE_RATE = 16000


# ============================================================
# 1. Whisper AI 모델 생성
# ============================================================

def create_model():

    # CPU에서 Whisper base 모델 실행
    model = WhisperModel(
        "base",
        device="cpu",
        compute_type="int8"
    )

    return model


# ============================================================
# 2. 키를 떼기 전까지 음성 녹음
# ============================================================

def record_audio(release_event):

    # 마이크에서 받은 음성 블록 저장 리스트
    audio_chunks = []

    # --------------------------------------------------------
    # Audio Callback
    # --------------------------------------------------------
    # sounddevice가 녹음 데이터를 받을 때마다 호출
    #
    # indata  : 입력된 음성 데이터
    # frames  : 현재 블록의 샘플 수
    # time    : 오디오 시간 정보
    # status  : 녹음 상태 정보
    # --------------------------------------------------------

    def audio_callback(indata, frames, time, status):

        if status:
            print(f"[AUDIO] {status}")

        # 모노 채널 데이터를 복사해서 저장
        # [:, 0]은 첫 번째 채널 선택
        audio_chunks.append(indata[:, 0].copy())


    print("[MIC] RECORDING...")

    # InputStream:
    # 마이크에서 음성을 계속 받아오는 입력 스트림
    #
    # blocksize=1024:
    # 약 64ms 단위로 음성 데이터를 수집
    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
        blocksize=1024,
        callback=audio_callback
    ):

        # 스페이스바를 뗄 때까지 대기
        release_event.wait()


    print("[MIC] 녹음 종료")

    # 녹음된 데이터가 없으면 빈 배열 반환
    if not audio_chunks:
        return np.array([], dtype=np.float32)

    # np.concatenate():
    # 여러 음성 블록을 하나의 1차원 배열로 연결
    audio = np.concatenate(audio_chunks)

    return audio


# ============================================================
# 3. Whisper AI 음성인식
# ============================================================

def recognize_voice(model, audio):

    # 녹음 데이터가 없다면 빈 문자열 반환
    if audio.size == 0:
        return ""

    print("[AI] 음성 인식 중...")

    # 한국어 음성을 텍스트로 변환
    segments, info = model.transcribe(
        audio,
        language="ko",
        beam_size=5,
        condition_on_previous_text=False
    )

    # 여러 구간의 텍스트를 하나의 문장으로 결합
    text = " ".join(
        segment.text.strip()
        for segment in segments
    ).strip()

    return text
