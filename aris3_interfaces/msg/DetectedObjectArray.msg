# 영상 한 장에서 찾은 물체 목록 (토픽: /aris3/detected_objects)
# 물체가 하나도 없어도 빈 목록을 보낸다. 그래야 "카메라는 살아 있는데 물체가 없다"를 구분할 수 있다.

std_msgs/Header header            # stamp = 영상 시각, frame_id = 아래 pose 들의 좌표계
DetectedObject[] objects
