"""한국어 음성 명령 -> 구조화 명령 (action, color, shape).

출처: MARK1 command_parser v1.2 (etc/mark1_voice_controller, 브랜치 feat/mark1-voice-controller)
ARIS3 변경 (2026-11-08, 11-09):
  - 작업 물체가 정육면체뿐이라 물체를 색으로 고른다 -> PICK 은 **색 필수, 도형 선택**
    (MARK1 은 도형 필수·색 선택이었음)
  - 노랑(yellow) 추가, '큐브/정육면체/네모/사각형' -> cube
  - Whisper 오인식 보정 표와 '지워줘'(모호) 처리는 MARK1 그대로
  - 「멈춰」(stop, 가장 먼저 봄), 「계속해」(resume: 정지로 끊긴 순서를 이어서),
    「연결해제」(disconnect), 「가동」(power_on: 접속 후 팔을 위로 일자로 펴기) - 2026-11-11
  - 지시어 「이거/그거/저거」(화면에서 고른 물체), 「저기/여기/거기」(화면에서 고른 자리).
    지금은 마우스 클릭, 이후 손짓이 가리킨 것을 쓴다. 실제 물체·자리 확인은 GUI 가 한다
  - 쌓기 (2026-11-13): 「노란 큐브를 파란 큐브 위로 옮겨줘」 -> pick, color=yellow, target_color=blue.
    「위」 바로 앞에 나온 색이 받침, 다른 색이 집을 큐브 (「파란 큐브 위에 노란 큐브 올려」 도 같은 뜻).
    「이거 파란 큐브 위에 올려」 -> 화면에서 고른 물체를 파랑 위로

  - 큐브 번호 (2026-11-14): 같은 색 큐브가 여럿이면 색 검출이 blue_cube_1, blue_cube_2 로 번호를 붙인다.
    「1번 파란색 큐브 집어줘」 -> pick, color=blue, number=1. 번호는 색 바로 앞(「2번 빨간 큐브」)이나
    뒤(「빨간 큐브 2번」)에 말한다. 쌓기는 「1번 파란 큐브를 2번 파란 큐브 위로」 처럼 같은 색끼리도 된다
    (target_number=2). 「이번」 은 「이번에는」 과 헷갈려 색 뒤에 올 때만 2 로 읽는다.

  - 올려놓기 (2026-11-14): 집을 물체를 말하지 않고 받침만 말하면 지금 들고 있는 큐브를 그 위에 놓는다.
    「빨간 큐브 위로 올려놔」 -> place, color="", target_color=red (target_number 도 됨: 「2번 파란 큐브 위에 올려놔」).
    「노란 큐브 위로 들어줘」 처럼 집기·들기 말이 있으면 올려놓기가 아니다. 들고 있는지는 ARIS 화면이 확인한다.

  - 찾기 (2026-11-14): 「파란색 큐브 찾아」 -> find, color=blue (번호도 됨). 집기·놓기 말이 함께 있으면 그쪽이 먼저다
    (「파란 큐브 찾아서 집어줘」 = pick - 집기는 안 보이면 스스로 찾는다). 팔을 돌려 찾는 것은 ARIS 화면이 한다.

ROS 에 의존하지 않는 순수 함수라서 단위 시험(test/test_command_parser.py)에서 바로 쓴다.
"""
import re

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
# 정지·이어서 (2026-11-11): 「멈춰」 는 무엇보다 먼저 본다 (「멈춰 그거 잡지 마」 도 정지)
# 「멈쳐」 「먹쳐」: Whisper 가 실제로 「멈춰」 를 이렇게 적음 (2026-11-11). 정지는 넓게 받는 쪽이 안전
STOP_WORDS = ["멈춰", "멈춰라", "멈추", "멈춤", "멈쳐", "멈처", "먹쳐", "정지", "그만", "스톱", "stop"]
RESUME_WORDS = ["계속해", "계속하", "계속진행", "이어서", "이어해", "다시진행"]
# 접속 (2026-11-11): 「연결해제」 가 「연결해」 를 품고 있으므로 끊기를 먼저 본다
DISCONNECT_WORDS = ["연결해제", "연결끊", "연결을끊", "접속해제", "접속끊", "접속을끊", "전원꺼", "꺼져"]
POWER_ON_WORDS = ["가동", "기동", "작동시작", "깨어나", "일어나"]
# 그리퍼 열기 (2026-11-14): 「그리퍼 열어」「손 펴」「집게 벌려」. 「열어」 만으로는 받지 않는다 (잡담에 흔한 말 -
# 들고 있는 큐브를 떨어뜨릴 수 있으니 그리퍼·손·집게 + 열/펴/벌 이 함께 있을 때만)
GRIPPER_NOUNS = ["그리퍼", "그립퍼", "그리파", "손", "집게"]
OPEN_VERBS = ["열어", "열기", "열어줘", "열자", "펴", "펼쳐", "벌려"]
# 찾기 (2026-11-14): 「파란색 큐브 찾아」 - 그 색 큐브가 카메라에 안 보이면 팔을 좌우로 돌리며 찾는다
FIND_WORDS = ["찾아", "찾기", "찾으", "어디있", "어딨", "어디에있"]
# 쌓기 (2026-11-13): 「~ 위에/위로」 또는 「올려/쌓아」 가 있고 색이 둘이면 한 큐브를 다른 큐브 위에 놓는다
STACK_WORDS = ["위에", "위로", "위쪽", "올려", "올리", "쌓아", "쌓기", "쌓으"]


# 큐브 번호 (2026-11-14): 「1번」「일번」「첫번째」「둘째」 -> 숫자. 긴 낱말을 먼저 본다 (「첫번째」 안의 「번째」 등)
NUMBER_WORDS = {
    1: ["첫번째", "첫째", "일번", "하나번"], 2: ["두번째", "둘째"], 3: ["세번째", "셋째", "삼번"],
    4: ["네번째", "넷째", "사번"], 5: ["다섯번째", "다섯째", "오번"],
}
NUMBER_MAX = 9                      # 「1번」~「9번」 (숫자로 적힌 것)
_NUM_RE = re.compile(r"([1-9])번(?:째)?|" + "|".join(
    f"(?P<n{n}>{'|'.join(sorted(ws, key=len, reverse=True))})" for n, ws in NUMBER_WORDS.items()))
AFTER_COLOR_CHARS = 8               # 색 낱말 뒤 이만큼 안의 번호만 그 색의 번호로 본다 (「파란 큐브 1번」)


def _number_tokens(norm):
    """문장의 번호 낱말 [(시작, 끝, 숫자)] 을 나온 순서대로. 「이번」 은 여기서 찾지 않는다 (cube_numbers 가 색 뒤에서만)."""
    out = []
    for m in _NUM_RE.finditer(norm):
        n = int(m.group(1)) if m.group(1) else next(int(k[1:]) for k, v in m.groupdict().items() if v)
        out.append((m.start(), m.end(), n))
    return out


def cube_numbers(norm, mentions):
    """색 언급 mentions [(시작, 색, 끝)] 마다 그 큐브의 번호 (없으면 0) 를 같은 순서로 돌려준다.

    색 바로 앞에 붙은 번호(「2번빨간」)를 먼저 주고, 남은 번호 중 색 뒤 AFTER_COLOR_CHARS 글자 안(다음 색 앞까지)의
    것을 준다. 번호 낱말 하나는 한 색에만 쓴다.
    """
    toks = _number_tokens(norm)
    nums, used = [0] * len(mentions), set()
    for i, (pos, _key, _end) in enumerate(mentions):                 # 1) 색 바로 앞
        for j, (_a, b, n) in enumerate(toks):
            if b == pos and j not in used:
                nums[i] = n
                used.add(j)
    for i, (_pos, _key, end) in enumerate(mentions):                 # 2) 색 뒤 (다음 색 앞까지)
        if nums[i]:
            continue
        nxt = mentions[i + 1][0] if i + 1 < len(mentions) else len(norm)
        lim = min(nxt, end + AFTER_COLOR_CHARS)
        for j, (a, b, n) in enumerate(toks):
            if j not in used and end <= a and b <= lim:
                nums[i] = n
                used.add(j)
                break
        else:
            k = norm.find("이번", end, lim)                           # 「파란 큐브 이번」 = 2번 (색 앞의 「이번」 은 「이번에는」)
            if k >= 0 and norm[k + 2:k + 3] not in ("에", "엔", "은"):
                nums[i] = 2
    return nums


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


def _colors_in_order(norm):
    """문장에 나온 색을 나온 순서대로 [(위치, 색, 그 색 낱말 끝 위치)]. 같은 색은 처음 한 번만."""
    found = []
    for key, words in COLOR_WORDS.items():
        hits = [(norm.find(w), len(w)) for w in words if w in norm]
        if hits:
            pos, n = min(hits)
            found.append((pos, key, pos + n))
    return sorted(found)


def color_mentions(norm):
    """문장에 나온 색 언급 [(위치, 색, 끝)] 과 그 번호 목록. 같은 색을 또 말한 것은 번호가 서로 다를 때만 남긴다.

    「1번 파란 큐브를 2번 파란 큐브 위로」 = 파랑 두 번 (번호 1, 2). 번호 없이 같은 색을 되풀이하면 처음 것만 (예전과 같음).
    """
    allm, pos = [], 0
    words = sorted(((w, key) for key, ws in COLOR_WORDS.items() for w in ws), key=lambda t: -len(t[0]))
    while pos < len(norm):
        hit = next(((w, key) for w, key in words if norm.startswith(w, pos)), None)   # 그 자리에서 가장 긴 색 낱말
        if hit:
            allm.append((pos, hit[1], pos + len(hit[0])))
            pos += len(hit[0])
        else:
            pos += 1
    nums = cube_numbers(norm, allm)
    keep, seen = [], {}
    for m, n in zip(allm, nums):
        if m[1] in seen and not (n and seen[m[1]] and n != seen[m[1]]):
            continue                            # 같은 색을 다시 말함 (번호로 구분되지 않음) -> 처음 것만
        seen.setdefault(m[1], n)
        keep.append((m, n))
    return [m for m, _n in keep], [n for _m, n in keep]


def stack_parts(norm):
    """쌓기 문장이면 (집을 색, 받침 색, 집을 번호, 받침 번호), 아니면 None. 집을 색이 ""면 「이거」(화면에서 고른 물체).

    「위」 가 바로 뒤따르는 색이 받침이다: 「노란 큐브를 파란 큐브 위로」, 「파란 큐브 위에 노란 거 올려」.
    「위」 가 없으면(「노랑을 파랑에 쌓아」) 먼저 말한 색이 집을 큐브다. 번호는 0 = 말하지 않음.
    """
    if not any(w in norm for w in STACK_WORDS):
        return None
    cols, nums = color_mentions(norm)
    if not cols:
        return None
    under = []                                  # 뒤에 「위」 가 따라오는 색의 순번
    for i, (_pos, _key, end) in enumerate(cols):
        nxt = cols[i + 1][0] if i + 1 < len(cols) else len(norm)
        if norm[end:nxt][:8 + 4].find("위") >= 0:    # 번호가 끼면(「파란 큐브 2번 위에」) 조금 더 멀리 본다
            under.append(i)
    if len(cols) == 1:                          # 「이거 파란 큐브 위에 올려」
        if under and any(w in norm for w in OBJECT_DEIXIS):
            return "", cols[0][1], 0, nums[0]
        return None
    bi = under[0] if under else 1
    oi = next(i for i in range(len(cols)) if i != bi)
    return cols[oi][1], cols[bi][1], nums[oi], nums[bi]


# 올려놓기 (2026-11-14): 받침만 말한 문장에서 「놓기」 로 읽는 말 / 「집기」 라서 올려놓기가 아닌 말
PLACE_ON_VERBS = PLACE_WORDS + ["올려", "올리", "쌓아", "쌓으", "쌓기", "얹어", "옮겨"]
NOT_PLACE_ON = ["집어", "집으", "집고", "잡아", "잡으", "잡고", "들어"]


def place_on_parts(norm):
    """올려놓기 문장이면 (받침 색, 받침 번호), 아니면 None: 색을 하나만 말했고 그 뒤에 「위」 가 오며 놓는 말이 있다.

    「빨간 큐브 위로 올려놔」 = 들고 있는 큐브를 빨강 위에. 「이거 … 위에」 는 쌓기(stack_parts) 라 여기서 받지 않는다.
    """
    cols, nums = color_mentions(norm)
    if len(cols) != 1 or any(w in norm for w in OBJECT_DEIXIS):
        return None
    if norm[cols[0][2]:][:12].find("위") < 0:
        return None
    if any(w in norm for w in NOT_PLACE_ON) or not any(w in norm for w in PLACE_ON_VERBS):
        return None
    return cols[0][1], nums[0]


def stack_colors(norm):
    """쌓기 문장이면 (집을 색, 받침 색), 아니면 None (stack_parts 에서 번호를 뺀 것 - 예전 이름)."""
    st = stack_parts(norm)
    return None if st is None else st[:2]


def parse_command(text):
    """문장 -> {action, color, shape, target_color, number, target_number, ambiguous, ref_object, ref_place,
    original_text, normalized_text}. number·target_number: 큐브 번호 (0 = 말하지 않음)."""
    cmd = _parse(text)
    cmd.update(target_color="", number=0, target_number=0)
    norm = cmd["normalized_text"]
    if cmd["color"] and cmd["action"] in ("pick", "place", "find", ""):
        cols, nums = color_mentions(norm)       # 「1번 파란색 큐브」: 그 색 언급에 붙은 번호
        cmd["number"] = next((n for m, n in zip(cols, nums) if m[1] == cmd["color"]), 0)
    if cmd["action"] in ("pick", "place", ""):
        st = stack_parts(norm)
        if st is not None:                      # 쌓기: 집어서 받침 큐브 위에 놓기 (자리는 화면 클릭 대신 받침 큐브)
            cmd.update(action="pick", color=st[0], target_color=st[1], number=st[2], target_number=st[3],
                       ambiguous=False, ref_object=not st[0], ref_place=False)
        else:
            on = place_on_parts(norm)
            if on is not None:                  # 올려놓기: 들고 있는 큐브를 받침 위에 (집을 물체는 말하지 않음)
                cmd.update(action="place", color="", number=0, target_color=on[0], target_number=on[1],
                           ambiguous=False, ref_object=False, ref_place=False)
    return cmd


def _parse(text):
    """문장 -> {action, color, shape, ambiguous, original_text, normalized_text} (쌓기 전 기본 해석)."""
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
    low = norm.lower()
    if any(w in low for w in STOP_WORDS):
        return {"action": "stop", "color": "", "shape": "", "ambiguous": False,
                "ref_object": False, "ref_place": False, "original_text": text, "normalized_text": norm}
    simple = {"action": "", "color": "", "shape": "", "ambiguous": False,
              "ref_object": False, "ref_place": False, "original_text": text, "normalized_text": norm}
    if any(w in norm for w in DISCONNECT_WORDS):
        return dict(simple, action="disconnect")
    if any(w in norm for w in POWER_ON_WORDS):
        return dict(simple, action="power_on")
    if any(n in norm for n in GRIPPER_NOUNS) and any(v in norm for v in OPEN_VERBS):
        return dict(simple, action="gripper_open")
    if any(w in norm for w in RESUME_WORDS):
        return {"action": "resume", "color": "", "shape": "", "ambiguous": False,
                "ref_object": False, "ref_place": False, "original_text": text, "normalized_text": norm}
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
    elif any(w in norm for w in FIND_WORDS):       # 집기·놓기 말이 없을 때만 (「찾아서 집어줘」 는 pick)
        action = "find"
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
    num, tnum = int(cmd.get("number", 0) or 0), int(cmd.get("target_number", 0) or 0)

    def named(color, n):
        """로그용 이름: 번호가 있으면 「1번 blue」, 없으면 색만."""
        return f"{n}번 {color}" if n else color
    if cmd["action"] == "pick" and cmd.get("target_color"):
        if cmd["target_color"] == cmd["color"] and not (num and tnum and num != tnum):
            # 같은 색끼리는 번호가 서로 달라야 한다 (「1번 파란 큐브를 2번 파란 큐브 위로」)
            return False, "같은 색 큐브 위에 쌓으라는 명령입니다. 받침 큐브의 색이나 번호를 다시 말씀해 주세요."
        if known_colors is not None and cmd["target_color"] not in known_colors:
            return False, f"작업 공간에 없는 색입니다: {cmd['target_color']}"
        if not cmd["color"] and not cmd.get("ref_object"):
            return False, "쌓을 물체의 색이 지정되지 않았습니다."
        what = named(cmd["color"], num) if cmd["color"] else "화면에서 고른 물체"
        return True, f"STACK 명령 생성 ({what} -> {named(cmd['target_color'], tnum)} 위)"
    if cmd["action"] == "pick":
        if cmd.get("ref_object") and not cmd["color"]:
            return True, "PICK 명령 생성 (화면에서 고른 물체" + (" -> 화면에서 고른 자리)" if cmd.get("ref_place") else ")")
        if not cmd["color"]:
            return False, "집을 물체의 색이 지정되지 않았습니다. (「이거」 라고 하려면 화면에서 먼저 고르세요)"
        if known_colors is not None and cmd["color"] not in known_colors:
            return False, f"작업 공간에 없는 색입니다: {cmd['color']}"
        return True, "PICK 명령 생성" + (f" ({num}번 {cmd['color']})" if num else "") + \
            (" (화면에서 고른 자리로)" if cmd.get("ref_place") else "")
    if cmd["action"] == "place" and cmd.get("target_color"):
        if known_colors is not None and cmd["target_color"] not in known_colors:
            return False, f"작업 공간에 없는 색입니다: {cmd['target_color']}"
        return True, f"PLACE_ON 명령 생성 (들고 있는 큐브 -> {named(cmd['target_color'], tnum)} 위)"
    if cmd["action"] == "place":
        return True, "PLACE 명령 생성 (집었던 자리에 놓기)"
    if cmd["action"] == "find":
        if not cmd["color"]:
            return False, "찾을 물체의 색이 지정되지 않았습니다."
        if known_colors is not None and cmd["color"] not in known_colors:
            return False, f"작업 공간에 없는 색입니다: {cmd['color']}"
        return True, f"FIND 명령 생성 ({named(cmd['color'], num)} 찾기)"
    if cmd["action"] == "home":
        return True, "HOME 명령 생성 (홈 자세로 돌아가기)"
    if cmd["action"] == "stop":
        return True, "STOP 명령 생성 (정지)"
    if cmd["action"] == "disconnect":
        return True, "DISCONNECT 명령 생성 (로봇 접속 끊기)"
    if cmd["action"] == "power_on":
        return True, "POWER_ON 명령 생성 (접속 -> 팔을 일자로 펴기)"
    if cmd["action"] == "resume":
        return True, "RESUME 명령 생성 (멈춘 단계부터 이어서)"
    if cmd["action"] == "gripper_open":
        return True, "GRIPPER_OPEN 명령 생성 (그리퍼 열기)"
    return False, "지원하지 않는 명령입니다."
