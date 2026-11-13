#!/bin/bash
# aris3_bringup 시뮬레이션의 큐브 3개를 처음 자리(config/object_layout.csv)로 되돌린다.
#   bash scripts/reset_aris3_cubes.sh
# 그리퍼가 큐브를 잡고 있지 않을 때 쓴다 (잡고 있으면 GUI 에서 「조작 허용」 -> 그리퍼 「열기」 먼저).
# (e0509_sim 작업 셀용은 scripts/reset_sim_blocks.sh)
export GZ_IP=${GZ_IP:-127.0.0.1}
WORLD=${WORLD:-empty}
Z=0.7755   # 탁자 0.75 + 큐브 반 0.025 + 여유 0.5 mm
for spec in "red_cube 0.0 0.45" "blue_cube 0.45 0.0" "yellow_cube -0.45 0.0"; do
  set -- $spec
  gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\", position: {x: $2, y: $3, z: $Z}, orientation: {w: 1}" >/dev/null && echo "$1 -> ($2, $3)"
done
