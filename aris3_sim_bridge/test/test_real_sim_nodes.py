"""실물 비슷한 시뮬레이션 노드의 순수 함수 시험 (ROS 없이)."""
import numpy as np
from sensor_msgs.msg import Image

from aris3_sim_bridge.dsr_joint_mirror import ordered_positions
from aris3_sim_bridge.jpeg_republisher import to_bgr


def test_ordered_positions_reorders_by_joint_name():
    """에뮬레이터가 다른 순서로 보내도 joint_1..6 순서로 바꾼다 (다른 관절은 무시)."""
    names = ['joint_3', 'joint_1', 'joint_2', 'gripper', 'joint_6', 'joint_5', 'joint_4']
    pos = [3.0, 1.0, 2.0, 9.0, 6.0, 5.0, 4.0]
    assert ordered_positions(names, pos) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_ordered_positions_missing_joint_sends_nothing():
    """관절이 하나라도 빠지면 None (반쪽 명령으로 팔이 튀지 않게)."""
    assert ordered_positions(['joint_1', 'joint_2'], [0.0, 0.0]) is None


def test_to_bgr_swaps_rgb_and_rejects_unknown():
    """rgb8 은 BGR 로 바뀌고, 모르는 형식은 None."""
    m = Image(height=1, width=2, encoding='rgb8', step=6, data=bytes([255, 0, 0, 0, 0, 255]))
    bgr = to_bgr(m)
    assert bgr.shape == (1, 2, 3)
    assert list(bgr[0, 0]) == [0, 0, 255] and list(bgr[0, 1]) == [255, 0, 0]
    assert to_bgr(Image(height=1, width=1, encoding='mono8', step=1, data=bytes([7]))) is None
    assert isinstance(bgr, np.ndarray)


# ---- gz_gripper_bridge: 멈춤 판단 (2026-11-11 간헐 실패 재발 방지)
from aris3_sim_bridge.gz_gripper_bridge import SettleTracker, close_result  # noqa: E402


def test_settle_ignores_stale_message():
    """명령 전에 받은 옛 값(열림 0, 그대로)만 계속 보이면 「멈춤」 으로 판단하지 않는다."""
    tr = SettleTracker(start=10.0, tol=0.006)
    # 옛 메시지(시각 9.9)를 1 초 동안 계속 읽음 -> 관절 메시지가 밀린 상황
    assert not any(tr.update(0.0, 9.9, now) for now in [10.3, 10.6, 10.9, 11.0])


def test_settle_ignores_message_too_old():
    """명령 뒤 받았더라도 max_age 보다 오래된 값이면 판단을 미룬다."""
    tr = SettleTracker(start=10.0, tol=0.006)
    assert not any(tr.update(0.0, 10.05, now) for now in [10.3, 10.5, 10.8])


def test_settle_after_fresh_still_messages():
    """새로 받은 메시지가 0.3 초 넘게 같은 위치면 멈춤."""
    tr = SettleTracker(start=10.0, tol=0.006)
    out = [tr.update(0.71, s, s + 0.01) for s in [10.30, 10.40, 10.50, 10.60, 10.65]]
    assert out[-2] is True or out[-1] is True
    assert not any(out[:3])


def test_settle_not_while_moving():
    """움직이는 동안(위치가 tol 넘게 바뀜)은 멈춤이 아니다."""
    tr = SettleTracker(start=10.0, tol=0.006)
    assert not any(tr.update(0.1 * i, 10.3 + 0.1 * i, 10.31 + 0.1 * i) for i in range(8))


def test_close_result_three_ranges():
    """거의 안 닫힘 / 막혀 멈춤 / 끝까지 닫힘 - 실물 브리지와 같은 세 구간."""
    assert close_result(0.0, 0.15, 1.0)[0] is False      # 열림에서 멈춤 = 잡지 못함 (예전에는 「잡음」)
    assert close_result(0.71, 0.15, 1.0)[0] is True      # 50 mm 큐브
    assert close_result(1.1, 0.15, 1.0)[0] is False      # 빈손
