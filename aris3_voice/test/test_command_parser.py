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


def test_stop_beats_everything():
    """「멈춰」 는 다른 명령 낱말과 함께여도 정지."""
    assert parse_command('가동 멈춰')['action'] == 'stop'
