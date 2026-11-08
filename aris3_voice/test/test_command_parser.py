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
]


@pytest.mark.parametrize("text,action,color,shape,valid", CASES)
def test_parse(text, action, color, shape, valid):
    cmd = parse_command(text)
    ok, _ = validate_command(cmd, KNOWN)
    assert (cmd["action"], cmd["color"], cmd["shape"], ok) == (action, color, shape, valid)
