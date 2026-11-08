#!/bin/bash
# e0509_sim Gazebo 작업 셀의 블록 3개를 처음 자리로 되돌린다 (Task Manager 시험 반복용).
#   bash scripts/reset_sim_blocks.sh
# 그리퍼가 블록을 잡고 있지 않을 때만 쓴다 (잡고 있으면 먼저 /dsr01/gripper/open).
WORLD=${WORLD:-e0509_workcell}
# 시뮬레이션을 GZ_IP=127.0.0.1 로 띄웠다면 여기서도 같아야 Gazebo 서비스를 찾는다.
export GZ_IP=${GZ_IP:-127.0.0.1}
Z=0.7705   # 테이블 0.75 + 블록 반 0.02 + 여유 0.5 mm
for spec in "red_block 0.45 -0.15" "green_block 0.45 0.0" "blue_block 0.45 0.15"; do
  set -- $spec
  gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\", position: {x: $2, y: $3, z: $Z}, orientation: {w: 1}" >/dev/null && echo "$1 -> ($2, $3)"
done
