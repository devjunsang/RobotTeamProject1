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
