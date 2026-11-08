
# ============================================================
# PROJECT 4 - MARK1 COMMAND PARSER v1.2
#
# 1. Whisper 오인식 문자열 보정
# 2. 도형 분석
# 3. 색상 분석
# 4. PICK / PLACE 분석
# 5. 명령어 유효성 검사
#
# MOVE 기능은 지원하지 않음
# ============================================================


# ============================================================
# 1. Whisper 오인식 보정 테이블
# ============================================================

# 실제 음성인식 테스트에서 발견된 오인식 표현
CORRECTIONS = {

    # 도형 보정
    "3각형": "삼각형",
    "사닷형": "사각형",

    # 색상 보정
    "바란색": "파란색",
    "허록색": "초록색",
    "크록색": "초록색",

    # PICK 발음 보정
    "지버죠": "집어줘",
    "지버줘": "집어줘",
    "지버조": "집어줘",
    "지보죠": "집어줘",

    # PLACE 발음 보정
    "돌려나": "돌려놔"
}


# ============================================================
# 2. 텍스트 정규화 함수
# ============================================================

def normalize_text(text):

    # 공백 제거
    text = "".join(text.split())

    # 오인식된 문자열을 올바른 표현으로 변환
    for wrong, correct in CORRECTIONS.items():

        text = text.replace(wrong, correct)

    return text


# ============================================================
# 3. 명령어 분석 함수
# ============================================================

def parse_command(text):

    # 원본 음성인식 결과 보관
    original_text = text

    # 오인식 보정
    text = normalize_text(text)

    # 초기값 설정
    target = None
    color = None
    action = None
    ambiguous = False

    # --------------------------------------------------------
    # 도형 분석
    # --------------------------------------------------------

    if "삼각형" in text or "세모" in text:
        target = "triangle"

    elif "사각형" in text or "네모" in text:
        target = "square"

    elif (
        "동그라미" in text
        or "원형" in text
        or "원모양" in text
    ):
        target = "circle"

    # --------------------------------------------------------
    # 색상 분석
    # --------------------------------------------------------

    red_keywords = [
        "빨간색", "빨간", "빨강",
        "붉은색", "붉은", "진빨강"
    ]

    blue_keywords = [
        "파란색", "파란", "파랑",
        "푸른색", "푸른", "진파랑"
    ]

    green_keywords = [
        "초록색", "초록", "녹색",
        "진초록", "연초록"
    ]

    if any(word in text for word in red_keywords):
        color = "red"

    elif any(word in text for word in blue_keywords):
        color = "blue"

    elif any(word in text for word in green_keywords):
        color = "green"

    # --------------------------------------------------------
    # Action 분석
    # --------------------------------------------------------

    place_keywords = [
        "내려놔",
        "내려놓",
        "놓아줘",
        "놔줘",
        "놓으",
        "돌려놔",
        "돌려놓",
        "원래대로"
    ]

    pick_keywords = [
        "집어",
        "집으",
        "집고",
        "잡아",
        "잡으",
        "잡고"
    ]

    # PLACE 우선 검사
    if any(word in text for word in place_keywords):
        action = "place"

    elif any(word in text for word in pick_keywords):
        action = "pick"

    # '지워줘'는 집기 명령의 오인식일 수 있지만
    # 실제 삭제 명령일 수도 있으므로 자동 확정하지 않음
    elif "지워줘" in text:

        if target is not None and color is not None:
            ambiguous = True

    # --------------------------------------------------------
    # 결과 반환
    # --------------------------------------------------------

    return {
        "target": target,
        "color": color,
        "action": action,
        "original_text": original_text,
        "normalized_text": text,
        "ambiguous": ambiguous
    }


# ============================================================
# 4. 명령어 유효성 검사
# ============================================================

def validate_command(command):

    action = command["action"]
    target = command["target"]

    # 의미가 모호한 명령은 자동 실행하지 않음
    if command["ambiguous"]:
        return False, "모호한 명령입니다. 다시 말씀해 주세요."

    # PICK은 도형을 반드시 지정해야 함
    if action == "pick":

        if target is None:
            return False, "집을 물체가 지정되지 않았습니다."

        return True, "PICK 명령 생성 완료"

    # PLACE는 현재 잡고 있는 물체를 내려놓음
    if action == "place":
        return True, "PLACE 명령 생성 완료"

    return False, "지원하지 않는 명령입니다."


# ============================================================
# 5. 단독 테스트
# ============================================================

def main():

    print("MARK1 Command Parser v1.2")

    while True:

        text = input("\n명령 입력 / q 종료 : ")

        if text.lower() == "q":
            break

        command = parse_command(text)

        valid, message = validate_command(command)

        print("\n===== COMMAND RESULT =====")
        print(f"Original   : {command['original_text']}")
        print(f"Normalized : {command['normalized_text']}")
        print(f"Target     : {command['target']}")
        print(f"Color      : {command['color']}")
        print(f"Action     : {command['action']}")

        if valid:
            print(f"[SUCCESS] {message}")
        else:
            print(f"[WARN] {message}")


if __name__ == "__main__":
    main()
