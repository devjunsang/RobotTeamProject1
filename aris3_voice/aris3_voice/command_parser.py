"""한국어 음성 명령 -> 구조화 명령 (action, color, shape).

출처: MARK1 command_parser v1.2 (etc/mark1_voice_controller, 브랜치 feat/mark1-voice-controller)
ARIS3 변경 (2026-11-08, 11-09):
  - 작업 물체가 정육면체뿐이라 물체를 색으로 고른다 -> PICK 은 **색 필수, 도형 선택**
    (MARK1 은 도형 필수·색 선택이었음)
  - 노랑(yellow) 추가, '큐브/정육면체/네모/사각형' -> cube
  - Whisper 오인식 보정 표와 '지워줘'(모호) 처리는 MARK1 그대로
  - 지시어 「이거/그거/저거」(화면에서 고른 물체), 「저기/여기/거기」(화면에서 고른 자리).
    지금은 마우스 클릭, 이후 손짓이 가리킨 것을 쓴다. 실제 물체·자리 확인은 GUI 가 한다

ROS 에 의존하지 않는 순수 함수라서 단위 시험(test/test_command_parser.py)에서 바로 쓴다.
"""

# Whisper 오인식 보정 (MARK1 실측에서 발견된 표현)
CORRECTIONS = {
    "3각형": "삼각형",
    "사닷형": "사각형",
    "바란색": "파란색",
    "허록색": "초록색",
    "크록색": "초록색",
    "지버죠": "집어줘",
    "지버줘": "집어줘",
    "지버조": "집어줘",
    "지보죠": "집어줘",
    "돌려나": "돌려놔",
    # ARIS3 실물 마이크 실측 (2026-11-09)
    "나와줘": "놔줘",
    "피우그": "큐브",
    "고른타리": "고른자리",
    "노키": "놓기",
    "배려나": "내려놔",
    "내려나": "내려놔",
    "내려와": "내려놔",
}

COLOR_WORDS = {
    "red": ["빨간색", "빨간", "빨강", "붉은색", "붉은", "진빨강"],
    "blue": ["파란색", "파란", "파랑", "푸른색", "푸른", "진파랑"],
    "green": ["초록색", "초록", "녹색", "진초록", "연초록"],
    "yellow": ["노란색", "노란", "노랑", "누런", "샛노란"],
}

SHAPE_WORDS = {
    "cube": ["정육면체", "큐브", "블록", "상자", "네모", "사각형"],
    "triangle": ["삼각형", "세모"],
    "circle": ["동그라미", "원형", "원모양"],
}

PLACE_WORDS = ["내려놔", "내려놓", "놓아줘", "놔줘", "놔", "놓으", "놓기", "돌려놔", "돌려놓", "원래대로"]

# 지시어: 화면에서 (지금은 마우스 클릭, 이후 손짓으로) 고른 물체·자리를 가리킨다 (2026-11-09)
OBJECT_DEIXIS = ["이거", "이것", "이걸", "그거", "그것", "그걸", "저거", "저것", "저걸"]
PLACE_DEIXIS = ["저기", "여기", "거기", "그자리", "이자리", "저자리", "그곳", "이곳", "저곳",
                "고른자리", "고른곳"]   # GUI 버튼 이름 「고른 자리에 놓기」 (2026-11-09)
PICK_WORDS = ["집어", "집으", "집고", "잡아", "잡으", "잡고", "들어", "옮겨"]
# 홈 자세로 (2026-11-09): 「집으로 돌아가」 의 「집으」 가 PICK 으로 읽히지 않게 PICK 보다 먼저 본다
HOME_WORDS = ["집으로돌아", "집으로가", "집으로복귀", "홈으로", "홈위치", "원위치", "복귀"]


def normalize_text(text):
    """공백을 없애고 알려진 오인식을 바로잡는다."""
    text = "".join(text.split())
    for wrong, right in CORRECTIONS.items():
        text = text.replace(wrong, right)
    return text


def _first_match(text, table):
    for key, words in table.items():
        if any(w in text for w in words):
            return key
    return ""


def parse_command(text):
    """문장 -> {action, color, shape, ambiguous, original_text, normalized_text}."""
    norm = normalize_text(text)
    color = _first_match(norm, COLOR_WORDS)
    shape = _first_match(norm, SHAPE_WORDS)
    ref_object = any(w in norm for w in OBJECT_DEIXIS)
    ref_place = any(w in norm for w in PLACE_DEIXIS)
    if ref_place and not color:
        # 「저기로 옮겨줘」: 무엇을 말하지 않고 자리만 말하면 화면에서 고른 물체를 옮긴다
        ref_object = True
    action = ""
    ambiguous = False
    if any(w in norm for w in HOME_WORDS):
        # 「집으로 돌아가」「홈으로 가」「원위치」: 물체·자리 말과 상관없이 홈 자세로
        return {"action": "home", "color": "", "shape": "", "ambiguous": False,
                "ref_object": False, "ref_place": False, "original_text": text, "normalized_text": norm}
    if ref_place and (color or ref_object):
        # 「이거 저기로 옮겨줘」「파란 거 저기 놔줘」: 물체와 자리가 다 있으면 옮기기(pick -> place)
        action = "pick"
    elif any(w in norm for w in PLACE_WORDS):      # PLACE 를 먼저 본다 ("들어"보다 "내려놔" 우선)
        action = "place"
    elif any(w in norm for w in PICK_WORDS):
        action = "pick"
    elif "지워줘" in norm and (color or shape or ref_object):
        # '집어줘' 의 오인식일 수도, 진짜 '지워줘' 일 수도 있다 -> 자동으로 정하지 않는다
        ambiguous = True
    return {"action": action, "color": color, "shape": shape, "ambiguous": ambiguous,
            "ref_object": ref_object, "ref_place": ref_place,
            "original_text": text, "normalized_text": norm}


def validate_command(cmd, known_colors=None):
    """(실행 가능한가, 이유). known_colors 를 주면 그 안의 색만 허용한다."""
    if cmd["ambiguous"]:
        return False, "모호한 명령입니다. 다시 말씀해 주세요."
    if cmd["action"] == "pick":
        if cmd.get("ref_object") and not cmd["color"]:
            return True, "PICK 명령 생성 (화면에서 고른 물체" + (" -> 화면에서 고른 자리)" if cmd.get("ref_place") else ")")
        if not cmd["color"]:
            return False, "집을 물체의 색이 지정되지 않았습니다. (「이거」 라고 하려면 화면에서 먼저 고르세요)"
        if known_colors is not None and cmd["color"] not in known_colors:
            return False, f"작업 공간에 없는 색입니다: {cmd['color']}"
        return True, "PICK 명령 생성" + (" (화면에서 고른 자리로)" if cmd.get("ref_place") else "")
    if cmd["action"] == "place":
        return True, "PLACE 명령 생성 (집었던 자리에 놓기)"
    if cmd["action"] == "home":
        return True, "HOME 명령 생성 (홈 자세로 돌아가기)"
    return False, "지원하지 않는 명령입니다."
