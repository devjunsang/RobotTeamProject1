
import sounddevice as sd
import numpy as np

# ==========================================
# Project 4 - Microphone Test
# ==========================================

SAMPLE_RATE = 16000  # 초당 16,000개 샘플
DURATION = 5         # 녹음 시간: 5초

# 1. 녹음 시작
print("[INFO] 5초 동안 녹음합니다.")
print("[MIC] 지금 말해보세요!")

audio = sd.rec(
    int(SAMPLE_RATE * DURATION),
    samplerate=SAMPLE_RATE,
    channels=1,
    dtype="float32"
)

# 2. 녹음 완료까지 대기
sd.wait()

print("[INFO] 녹음 완료!")

# 3. 녹음된 음성 데이터 분석
volume = np.max(np.abs(audio))

print(f"[INFO] 최대 진폭: {volume:.4f}")
print(f"[INFO] 데이터 형태: {audio.shape}")

# 4. 녹음한 음성 재생
print("[SPEAKER] 녹음한 음성을 재생합니다.")

sd.play(audio, samplerate=SAMPLE_RATE)
sd.wait()

print("[SUCCESS] 마이크 테스트 완료!")
