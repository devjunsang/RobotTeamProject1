#!/bin/bash
# ARIS3 Task Manager 시나리오 시험 (Gazebo 작업 셀 e0509_sim 위에서).
#
# 준비 (각각 다른 터미널):
#   ros2 launch e0509_sim e0509_workcell.launch.py
#   ros2 launch aris3_task_manager task_manager_sim.launch.py
# 실행:
#   bash scripts/test_task_manager_sim.sh
#
# 시험용 설정의 Task Manager 를 /aris3/test_execute_task 이름으로 하나 더 띄워서
# "도달 불가"(아주 먼 구역 FAR)와 "집기 실패"(grasp_offset 을 6 cm 로 크게) 를 일부러 만든다.
# 블록 위치는 시작할 때 scripts/reset_sim_blocks.sh 로 되돌린다.
# Wi-Fi 가 끊겼다 붙으면 Gazebo 내부 통신이 멈출 수 있다 -> 시뮬레이션을 GZ_IP=127.0.0.1 로 띄울 것.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
CFG=$(ros2 pkg prefix aris3_task_manager)/share/aris3_task_manager/config/task_manager.yaml
pass=0; fail=0

# run <이름> <기대 종료코드> <기대 error_code 또는 -> -- send_task 인자...
run() {
  local name=$1 want_exit=$2 want_code=$3; shift 4
  local out; out=$(timeout 240 ros2 run aris3_task_manager send_task "$@" 2>&1); local ex=$?
  local code; code=$(echo "$out" | grep -oE 'error_code=[0-9]+' | head -1 | cut -d= -f2)
  local line; line=$(echo "$out" | grep -A1 '^결과' | tail -1 | sed 's/^ *//')
  if [ "$ex" = "$want_exit" ] && { [ "$want_code" = "-" ] || [ "$code" = "$want_code" ]; }; then
    echo "PASS  $name  (exit $ex, code ${code:--}) $line"; pass=$((pass + 1))
  else
    echo "FAIL  $name  (exit $ex want $want_exit, code ${code:--} want $want_code)"; echo "$out" | tail -4; fail=$((fail + 1))
  fi
}
truth() { timeout 4 ros2 topic echo /model/$1_block/pose --once 2>/dev/null | grep -A2 'position:' | tail -2 | awk '{printf "%.3f ", $2}'; }

bash "$HERE/reset_sim_blocks.sh" > /dev/null && sleep 2
echo "== 기본 동작"
before=$(truth red)
run "dry_run red->A (로봇 안 움직임)" 0 0 -- red A --dry-run
[ "$(truth red)" = "$before" ] && { echo "PASS  dry_run 뒤 블록 그대로 ($before)"; pass=$((pass + 1)); } || { echo "FAIL  dry_run 뒤 블록이 움직임"; fail=$((fail + 1)); }
run "red->A" 0 0 -- red A

echo "== 동시 요청은 거절"
timeout 240 ros2 run aris3_task_manager send_task green B > /tmp/aris3_test_green.log 2>&1 & BG=$!
sleep 4
run "작업 중 두 번째 요청 blue->C" 2 - -- blue C
wait $BG; [ $? = 0 ] && { echo "PASS  첫 작업 green->B 는 계속 진행되어 성공"; pass=$((pass + 1)); } || { echo "FAIL  green->B"; tail -3 /tmp/aris3_test_green.log; fail=$((fail + 1)); }

echo "== 움직이기 전에 거부"
run "blue->A (A 에 red 가 있음)" 1 13 -- blue A
run "purple->C (모르는 색)" 1 1 -- purple C
run "red->Z (모르는 구역)" 1 1 -- red Z

echo "== 취소"
timeout 240 ros2 run aris3_task_manager send_task blue C > /tmp/aris3_test_cancel.log 2>&1 & BG=$!
sleep 5; kill -INT $BG; wait $BG; ex=$?
if [ $ex = 1 ] && grep -q '취소됨' /tmp/aris3_test_cancel.log; then
  echo "PASS  blue->C 5초 뒤 Ctrl-C -> 취소됨 ($(grep -oE '\[.*\] [A-Z]+ ' /tmp/aris3_test_cancel.log | tail -1))"; pass=$((pass + 1))
else echo "FAIL  취소 (exit $ex)"; tail -4 /tmp/aris3_test_cancel.log; fail=$((fail + 1)); fi
st=$(timeout 5 ros2 service call /dsr01/motion/check_motion dsr_msgs2/srv/CheckMotion 2>/dev/null | grep -oE 'status=[0-9]+')
[ "$st" = "status=0" ] && { echo "PASS  취소 뒤 로봇 정지 ($st)"; pass=$((pass + 1)); } || { echo "FAIL  취소 뒤 로봇 상태 $st"; fail=$((fail + 1)); }
run "취소 뒤 다시 blue->C" 0 0 -- blue C

echo "== 시험용 설정: 도달 불가 / 집기 실패"
ros2 run aris3_task_manager task_manager --ros-args --params-file "$CFG" -r __node:=aris3_task_manager_test \
  -p action_name:=/aris3/test_execute_task -p zone_names:="[A,B,C,FAR]" \
  -p zone_xy_world:="[0.30,0.35,0.45,0.35,0.60,0.35,1.20,0.0]" -p grasp_offset_m:=0.06 > /tmp/aris3_test_tm.log 2>&1 & TM=$!
sleep 6
bash "$HERE/reset_sim_blocks.sh" > /dev/null && sleep 2
run "red->FAR (1.2 m, 계획 단계에서 거부)" 1 6 -- red FAR --action /aris3/test_execute_task
run "green->B, 6 cm 위를 집음 (재시도 후 집기 실패, 홈 복귀)" 1 8 -- green B --action /aris3/test_execute_task
kill -TERM $TM; wait $TM 2>/dev/null   # & 로 띄운 프로세스는 SIGINT 를 무시하므로 SIGTERM
grep -q '집기 실패 (1/2)' /tmp/aris3_test_tm.log && { echo "PASS  집기 실패 1회 뒤 다시 관찰하고 재시도함"; pass=$((pass + 1)); } || { echo "FAIL  재시도 기록 없음"; fail=$((fail + 1)); }
grep -q '복구 완료' /tmp/aris3_test_tm.log && { echo "PASS  실패 뒤 위로 피하고 홈 복귀"; pass=$((pass + 1)); } || { echo "FAIL  복구 기록 없음"; fail=$((fail + 1)); }

bash "$HERE/reset_sim_blocks.sh" > /dev/null
echo; echo "결과: PASS $pass / FAIL $fail"
[ $fail = 0 ]
