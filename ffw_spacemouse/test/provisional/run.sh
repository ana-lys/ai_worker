#!/bin/bash
# Provisional teleop tests (MARKER_TELEOP_PLAN.md phase 0). Each test starts
# the nodes it needs on an ISOLATED ROS domain (87, localhost only), drives
# them with scripted inputs and compares the JSON summary with expected/.
#
# usage: [RECORD=1] run.sh <test>      (RECORD=1 rewrites expected/<test>.json)
#   joy_hand        joy_hand alone; diffed against expected/joy_hand_regress.json
#   joy_hand_core   joy_hand -> teleop_core; same expected output
#   marker          joy_hand_marker alone
#   marker_core     joy_hand_marker -> teleop_core
#   base            teleop_core /cmd_vel merge + speed cap
#   health          teleop_core marker_health:=true + joy_hand_marker
#   clamp           teleop_core base clamp, closed loop
#   lift_head       teleop_core lift ramp + head relay
#   all             everything above
D=$(cd "$(dirname "$0")" && pwd)
LOG=${LOG_DIR:-/tmp/ffw_teleop_provisional}; mkdir -p "$LOG"
source /opt/ros/*/setup.bash; source ~/robotis_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TEST:-87} ROS_LOCALHOST_ONLY=1
LIB=$(ros2 pkg prefix ffw_spacemouse)/lib/ffw_spacemouse
JOY_ARGS=(--ros-args -r __ns:=/right -r __node:=joy_hand -p target_arm:=right -p rot_step:=0.025
  -p axis_x:=1 -p axis_y:=0 -p axis_z:=2 -p axis_roll:=4 -p axis_pitch:=3 -p axis_yaw:=5)

run() {  # run <harness> <core args or -> <joy exe or -> [joy extra args...]
  local harness=$1 core=$2 joy=$3; shift 3
  local pids=()
  if [ "$core" != "-" ]; then
    $LIB/teleop_core --ros-args $core > "$LOG/teleop_core.log" 2>&1 & pids+=($!)
  fi
  if [ "$joy" != "-" ]; then
    $LIB/$joy "${JOY_ARGS[@]}" "$@" > "$LOG/joy_hand.log" 2>&1 & pids+=($!)
  fi
  sleep 1
  python3 "$D/$harness" 2>/dev/null
  local rc=$?
  kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null
  return $rc
}

expect() {  # expect <name> : compare stdin JSON with expected/<name>.json (numbers +-0.002)
  local out; out=$(cat)
  if [ -n "${RECORD:-}" ]; then echo "$out" > "$D/expected/$1.json"; echo "RECORDED $1"; return; fi
  if echo "$out" | python3 "$D/compare.py" "$D/expected/$1.json"; then echo "PASS $1"
  else echo "FAIL $1"; fi
}

case "${1:-all}" in
  joy_hand)      run joy_hand_regress.py - joy_hand | expect joy_hand_regress ;;
  joy_hand_core) run joy_hand_regress.py "-p marker_health:=false" joy_hand -p use_teleop_core:=true | expect joy_hand_regress ;;
  marker)        run marker_regress.py - joy_hand_marker | expect marker ;;
  marker_core)   run marker_regress.py "-p marker_health:=false" joy_hand_marker -p use_teleop_core:=true | expect marker_core ;;
  base)          run base_regress.py "-p marker_health:=false" - | expect base ;;
  health)        run health_regress.py "-p marker_health:=true" joy_hand_marker -p use_teleop_core:=true | expect health ;;
  clamp)         run clamp_regress.py "-p marker_health:=false" - | expect clamp ;;
  lift_head)     run lift_head_regress.py "-p marker_health:=false" - | expect lift_head ;;
  all) for t in joy_hand joy_hand_core marker marker_core base health clamp lift_head; do
         echo "== $t"; "$0" "$t"; done ;;
  *) echo "unknown test $1"; exit 2 ;;
esac
