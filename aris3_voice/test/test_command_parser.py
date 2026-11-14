"""aris3_voice 명령 해석 시험 (pytest). ROS 없이 실행된다."""
import pytest

from aris3_voice.command_parser import parse_command, validate_command

KNOWN = ["red", "blue", "yellow"]

# (문장, action, color, shape, valid)
CASES = [
    ("빨간 거 집어줘", "pick", "red", "", True),
    ("파란 큐브 집어줘", "pick", "blue", "cube", True),
    ("노란색 정육면체 잡아줘", "pick", "yellow", "cube", True),
    ("노란 네모 집어줘", "pick", "yellow", "cube", True),
    ("파란 블록 옮겨줘", "pick", "blue", "cube", True),
    ("바란색 큐브 지버죠", "pick", "blue", "cube", True),       # MARK1 오인식 보정
    ("내려놔", "place", "", "", True),
    ("원래대로 돌려나", "place", "", "", True),
    ("큐브 집어줘", "pick", "", "cube", False),                  # 색 없음 -> 고를 수 없음
    ("빨간 큐브 지워줘", "", "red", "cube", False),              # 모호
    ("초록 큐브 집어줘", "pick", "green", "cube", False),        # 작업 공간에 없는 색
    ("안녕하세요", "", "", "", False),
    # 지시어 (화면 클릭·손짓으로 고른 물체/자리)
    ("이거 저기로 옮겨줘", "pick", "", "", True),
    ("이거 집어줘", "pick", "", "", True),
    ("파란 큐브 저기로 옮겨줘", "pick", "blue", "cube", True),
    ("노란 거 여기 놔줘", "pick", "yellow", "", True),
    ("저기로 옮겨줘", "pick", "", "", True),                 # 무엇을 말하지 않음 -> 화면에서 고른 큐브
    ("여기 놔줘", "pick", "", "", True),
    ("옮겨줘", "pick", "", "", False),                       # 무엇도 어디도 없음
]


@pytest.mark.parametrize("text,action,color,shape,valid", CASES)
def test_parse(text, action, color, shape, valid):
    cmd = parse_command(text)
    ok, _ = validate_command(cmd, KNOWN)
    assert (cmd["action"], cmd["color"], cmd["shape"], ok) == (action, color, shape, valid)


def test_deixis_flags():
    cmd = parse_command("이거 저기로 옮겨줘")
    assert cmd["ref_object"] and cmd["ref_place"]
    cmd = parse_command("저기로 옮겨줘")
    assert cmd["ref_object"] and cmd["ref_place"]
    cmd = parse_command("파란 큐브 저기로 옮겨줘")
    assert not cmd["ref_object"] and cmd["ref_place"]
    cmd = parse_command("파란 큐브 집어줘")
    assert not cmd["ref_object"] and not cmd["ref_place"]


def test_aris3_mic_corrections():
    """2026-11-09 실물 마이크에서 들린 오인식."""
    c = parse_command("저기에 나와줘")
    assert c["normalized_text"] == "저기에놔줘" and c["ref_place"] and c["ref_object"]
    c = parse_command("파란 피우그 잡아줘")
    assert (c["action"], c["color"], c["shape"]) == ("pick", "blue", "cube")
    c = parse_command("나와줘")
    assert c["action"] == "place"


def test_selected_place_phrase():
    """「고른 자리에 놓기」(GUI 버튼 이름)와 그 오인식 「고른타리의 노키」 -> 고른 자리 명령."""
    for text in ("고른 자리에 놓기", "고른타리의 노키", "고른 곳에 놔줘"):
        c = parse_command(text)
        assert c["ref_place"] and c["ref_object"] and c["action"] == "pick", text
        assert validate_command(c)[0], text
    assert parse_command("내려놓기")["action"] == "place"


def test_short_place_word():
    """「놔」 만 말해도 놓기 (2026-11-09 실측 「고른 자리에 놔」)."""
    c = parse_command("고른 자리에 놔")
    assert c["ref_place"] and c["action"] == "pick" and validate_command(c)[0]
    assert parse_command("놔")["action"] == "place"
    c = parse_command("저기 놔")
    assert c["ref_place"] and c["ref_object"]
    assert parse_command("빨간 큐브 집어줘")["action"] == "pick"     # 집기 문장에는 영향 없음


def test_put_down_mishearing():
    """2026-11-09 실측 「배려나?」 = 내려놔."""
    for text in ("배려나?", "내려나", "내려와"):
        assert parse_command(text)["action"] == "place", text


def test_home_command():
    """「집으로 돌아가」 = 홈 자세로 (「집으」 가 집기로 읽히면 안 됨)."""
    for text in ("집으로 돌아가", "집으로 돌아가줘", "집으로 가", "홈으로 가", "원위치", "원위치로 복귀해"):
        c = parse_command(text)
        assert c["action"] == "home", text
        assert validate_command(c) == (True, "HOME 명령 생성 (홈 자세로 돌아가기)"), text
    # 집기 문장은 그대로
    assert parse_command("빨간 큐브 집어줘")["action"] == "pick"
    assert parse_command("파란 거 집으세요")["action"] == "pick"
    assert parse_command("원래대로 돌려놔")["action"] == "place"


# ---- 멈춰 / 계속해 (2026-11-11)
def test_stop_words():
    """「멈춰」「정지」「그만」 -> stop, 다른 말과 섞여도 정지가 먼저."""
    for s in ['멈춰', '멈춰!', '정지', '그만', '멈춰 그거 잡지 마', '빨간 큐브 잡지 말고 멈춰']:
        cmd = parse_command(s)
        assert cmd['action'] == 'stop', s
        assert validate_command(cmd)[0]


def test_resume_words():
    """「계속해」「이어서 해」 -> resume."""
    for s in ['계속해', '계속 해줘', '이어서 해']:
        cmd = parse_command(s)
        assert cmd['action'] == 'resume', s
        assert validate_command(cmd)[0]


def test_pick_not_mistaken_for_stop_or_resume():
    """보통 집기 명령은 그대로 pick."""
    assert parse_command('노란색 큐브 잡아')['action'] == 'pick'


def test_disconnect_and_power_on():
    """「연결해제」 는 「연결해」 를 품어도 끊기, 「가동」 은 power_on."""
    for s, a in [('연결해제', 'disconnect'), ('연결 해제해', 'disconnect'), ('접속 끊어', 'disconnect'),
                 ('가동', 'power_on'), ('가동해', 'power_on'), ('가동 시작', 'power_on')]:
        cmd = parse_command(s)
        assert cmd['action'] == a, s
        assert validate_command(cmd)[0]


def test_gripper_open():
    """그리퍼 열기 (2026-11-14): 그리퍼·손·집게 + 열/펴/벌. 「멈춰」 가 섞이면 정지가 먼저."""
    for s in ['그리퍼 열어', '그리퍼 열어줘', '그리퍼를 열어', '손 펴', '손을 펴줘', '집게 벌려', '그립퍼 열기']:
        cmd = parse_command(s)
        assert cmd['action'] == 'gripper_open', s
        assert validate_command(cmd) == (True, 'GRIPPER_OPEN 명령 생성 (그리퍼 열기)')
    assert parse_command('그리퍼 열지 말고 멈춰')['action'] == 'stop'


def test_open_alone_is_not_gripper_open():
    """「열어」 만(그리퍼·손·집게 없이)은 그리퍼 열기가 아니다 - 잡담으로 큐브를 떨어뜨리지 않게."""
    for s in ['문 열어', '창문 열어줘', '열어']:
        assert parse_command(s)['action'] != 'gripper_open', s


def test_stop_beats_everything():
    """「멈춰」 는 다른 명령 낱말과 함께여도 정지."""
    assert parse_command('가동 멈춰')['action'] == 'stop'


# 쌓기 (2026-11-13): (문장, 집을 색, 받침 색, 「이거」 인가)
STACK_CASES = [
    ("노란색 큐브를 파란색 큐브 위로 옮겨줘", "yellow", "blue", False),
    ("노란 큐브 파란 큐브 위에 올려줘", "yellow", "blue", False),
    ("파란 거 위에 빨간 거 올려", "red", "blue", False),        # 받침을 먼저 말함
    ("빨간 큐브를 노란 큐브에 쌓아줘", "red", "yellow", False),  # 「위」 없음 -> 먼저 말한 색을 집음
    ("이거 파란 큐브 위에 올려줘", "", "blue", True),            # 화면에서 고른 물체를 파랑 위로
]


@pytest.mark.parametrize("text,color,target,ref_obj", STACK_CASES)
def test_stack(text, color, target, ref_obj):
    """「A 를 B 위로」: pick + color=A + target_color=B, 놓을 자리는 화면 클릭이 아니라 받침 큐브."""
    cmd = parse_command(text)
    assert (cmd["action"], cmd["color"], cmd["target_color"], cmd["ref_object"]) == ("pick", color, target, ref_obj)
    assert not cmd["ref_place"]
    ok, why = validate_command(cmd, KNOWN)
    assert ok and "STACK" in why


def test_not_stack():
    """색이 하나뿐인 집기·「위」 없는 옮기기·멈춰는 쌓기가 아니다 (target_color 빈 값)."""
    for text, action in (("노란 큐브 위로 들어줘", "pick"), ("파란 큐브 저기로 옮겨줘", "pick"),
                         ("멈춰 노란 거 파란 거 위에 올리지 마", "stop"), ("홈으로", "home")):
        cmd = parse_command(text)
        assert cmd["action"] == action and cmd["target_color"] == "", text


def test_stack_same_color_invalid():
    """같은 색 위에 쌓으라는 말(받침 = 집을 큐브)은 실행하지 않는다."""
    cmd = dict(parse_command("노란 큐브 집어줘"), target_color="yellow")
    assert not validate_command(cmd, KNOWN)[0]


# ---------------------------------------------------------------- 큐브 번호 (2026-11-14)
NUMBER_CASES = [
    ("1번 파란색 큐브 집어줘", "blue", 1),
    ("파란 큐브 2번 잡아", "blue", 2),                # 번호를 색 뒤에 말함
    ("일번 빨간 큐브 집어", "red", 1),                 # 한글로 적힌 번호
    ("두 번째 노란 큐브 들어", "yellow", 2),
    ("3번 노란색 큐브 저기로 옮겨줘", "yellow", 3),
    ("파란 큐브 이번 집어줘", "blue", 2),              # 색 뒤의 「이번」 = 2번
    ("이번에는 파란 큐브 집어줘", "blue", 0),          # 「이번에는」 은 번호가 아니다
    ("빨간 큐브 집어줘", "red", 0),                    # 번호를 말하지 않음
]


@pytest.mark.parametrize("text,color,number", NUMBER_CASES)
def test_cube_number(text, color, number):
    """「N번 색 큐브」: 색과 큐브 번호를 함께 읽는다 (번호가 없으면 0). 실행할 수 있는 PICK 명령."""
    cmd = parse_command(text)
    assert (cmd["action"], cmd["color"], cmd["number"], cmd["target_number"]) == ("pick", color, number, 0), cmd
    assert validate_command(cmd, KNOWN)[0]


def test_number_without_color_is_not_enough():
    """색 없이 번호만 말하면(「1번 큐브 집어줘」) 어느 색인지 몰라 실행하지 않는다."""
    cmd = parse_command("1번 큐브 집어줘")
    assert cmd["number"] == 0 and not validate_command(cmd, KNOWN)[0]


STACK_NUMBER_CASES = [
    ("1번 파란 큐브를 2번 파란 큐브 위로 옮겨줘", "blue", 1, "blue", 2),     # 같은 색끼리 번호로 구분
    ("노란 큐브를 2번 파란 큐브 위로 옮겨줘", "yellow", 0, "blue", 2),
    ("1번 빨간 큐브를 파란 큐브 위에 올려", "red", 1, "blue", 0),
    ("파란 큐브 위에 1번 빨간 큐브 올려", "red", 1, "blue", 0),             # 받침을 먼저 말함
    ("이거 2번 파란 큐브 위에 올려", "", 0, "blue", 2),                     # 화면에서 고른 물체를 2번 파랑 위로
]


@pytest.mark.parametrize("text,color,number,target,target_number", STACK_NUMBER_CASES)
def test_stack_with_numbers(text, color, number, target, target_number):
    """쌓기에서 집을 큐브·받침 큐브의 번호를 따로 읽는다. 같은 색끼리도 번호가 다르면 쌓기."""
    cmd = parse_command(text)
    assert (cmd["action"], cmd["color"], cmd["number"], cmd["target_color"], cmd["target_number"]) == \
        ("pick", color, number, target, target_number), cmd
    ok, why = validate_command(cmd, KNOWN)
    assert ok and "STACK" in why


def test_stack_same_color_needs_different_numbers():
    """같은 색 위에 쌓기: 번호가 서로 다를 때만 실행한다 (번호가 같거나 한쪽만 있으면 거부)."""
    base = dict(parse_command("파란 큐브 집어줘"), target_color="blue")
    assert not validate_command(dict(base, number=1, target_number=1), KNOWN)[0]
    assert not validate_command(dict(base, number=1, target_number=0), KNOWN)[0]
    assert validate_command(dict(base, number=1, target_number=2), KNOWN)[0]


# ---------------------------------------------------------------- 올려놓기 (2026-11-14)
PLACE_ON_CASES = [
    ("빨간큐브위로 올려놔", "red", 0),
    ("빨간 큐브 위에 올려놔", "red", 0),
    ("2번 파란 큐브 위에 놓아줘", "blue", 2),
    ("노란 큐브 위로 옮겨줘", "yellow", 0),
    ("빨간 큐브 위에 쌓아", "red", 0),
]


@pytest.mark.parametrize("text,target,target_number", PLACE_ON_CASES)
def test_place_on(text, target, target_number):
    """받침만 말하면 지금 들고 있는 큐브를 그 위에 놓는다: place + color 없음 + target_color = 받침."""
    cmd = parse_command(text)
    assert (cmd["action"], cmd["color"], cmd["number"], cmd["target_color"], cmd["target_number"]) == \
        ("place", "", 0, target, target_number), cmd
    assert not cmd["ref_object"] and not cmd["ref_place"]
    ok, why = validate_command(cmd, KNOWN)
    assert ok and "PLACE_ON" in why


def test_not_place_on():
    """집기·들기 말이 있거나(「위로 들어줘」), 「이거」 가 있거나(쌓기), 「위」 가 없으면 올려놓기가 아니다."""
    assert parse_command("노란 큐브 위로 들어줘")["action"] == "pick"
    cmd = parse_command("이거 파란 큐브 위에 올려줘")
    assert cmd["action"] == "pick" and cmd["ref_object"] and cmd["target_color"] == "blue"
    assert parse_command("내려놔")["target_color"] == ""
    assert parse_command("빨간 큐브 집어줘")["action"] == "pick"


# ---------------------------------------------------------------- 찾기 (2026-11-14)
@pytest.mark.parametrize("text,color,number", [
    ("파란색 큐브 찾아", "blue", 0),
    ("빨간 큐브 찾아줘", "red", 0),
    ("2번 파란 큐브 찾아봐", "blue", 2),
    ("노란 큐브 어디 있어", "yellow", 0),
])
def test_find(text, color, number):
    """「색 큐브 찾아」: find + 색 (+ 번호). 집지 않는 명령."""
    cmd = parse_command(text)
    assert (cmd["action"], cmd["color"], cmd["number"], cmd["target_color"]) == ("find", color, number, ""), cmd
    ok, why = validate_command(cmd, KNOWN)
    assert ok and "FIND" in why


def test_find_needs_color_and_pick_wins():
    """색 없이 「찾아」 만 하면 실행하지 않는다. 집기 말이 함께 있으면 찾기가 아니라 집기다."""
    assert not validate_command(parse_command("큐브 찾아줘"), KNOWN)[0]
    cmd = parse_command("파란 큐브 찾아서 집어줘")
    assert (cmd["action"], cmd["color"]) == ("pick", "blue")


# ---------------------------------------------------------------- 탐색 (2026-11-14)
def test_scan_and_scan_with_color_is_find():
    """색 없는 「탐색」 = scan (세 가지 색이 보일 때까지). 색을 말하면 그 색 찾기(find). 집기 말이 있으면 집기."""
    for text in ("탐색", "탐색해", "탐색 시작해줘", "주변 둘러봐"):
        cmd = parse_command(text)
        assert (cmd["action"], cmd["color"]) == ("scan", ""), text
        ok, why = validate_command(cmd, KNOWN)
        assert ok and "SCAN" in why
    cmd = parse_command("파란 큐브 탐색해")
    assert (cmd["action"], cmd["color"]) == ("find", "blue")
    assert parse_command("탐색해서 빨간 큐브 집어줘")["action"] == "pick"

