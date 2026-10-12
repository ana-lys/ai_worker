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
#   joy_base        joy_base (use_teleop_core) -> teleop_core: twist, lift, head
#   logitech        logitech_base_teleop (use_teleop_core) -> teleop_core
#   e2e_marker      marker_frame_teleop.launch.py + real solver in sim (opens the
#                   MuJoCo viewer; prints only, not part of "all")
#   e2e_unified     default unified launch keeps the legacy wiring (opens the viewer)
#   all             everything above except e2e_marker
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
  joy_base)      $LIB/joy_base --ros-args -p use_teleop_core:=true -p hardware_mode:=false \
                   > "$LOG/joy_base.log" 2>&1 & JB=$!
                 run joy_base_regress.py "-p marker_health:=false" - | expect joy_base
                 kill $JB 2>/dev/null; wait 2>/dev/null ;;
  logitech)      $LIB/logitech_base_teleop.py --ros-args -p use_teleop_core:=true \
                   > "$LOG/logitech.log" 2>&1 & LG=$!
                 run logitech_regress.py "-p marker_health:=false" - | expect logitech
                 kill $LG 2>/dev/null; wait 2>/dev/null ;;
  e2e_marker)    # real solver + full marker launch in sim; opens the MuJoCo viewer
                 setsid ros2 launch ffw_spacemouse marker_frame_teleop.launch.py hardware_mode:=false \
                   use_logitech:=false use_quest:=false > "$LOG/e2e_launch.log" 2>&1 & LP=$!
                 sleep 5
                 python3 "$D/e2e_marker.py" 2>/dev/null
                 kill -INT -- -$LP 2>/dev/null; sleep 3; kill -- -$LP 2>/dev/null; wait 2>/dev/null ;;
  e2e_unified)   # default unified launch must keep the legacy wiring (no teleop_core)
                 setsid ros2 launch ffw_spacemouse spacemouse_unified_teleop.launch.py hardware_mode:=false \
                   use_logitech:=false use_quest:=false > "$LOG/e2e_unified.log" 2>&1 & LP=$!
                 sleep 10
                 echo "{\"teleop_core_running\": $(ros2 node list 2>/dev/null | grep -qx /teleop_core && echo true || echo false),"
                 echo " \"solver.joy_lift\": \"$(ros2 param get /ffw_ik_solver_teleop joy_lift 2>/dev/null | awk '{print $NF}')\","
                 echo " \"right_mapper.use_teleop_core\": \"$(ros2 param get /right/joy_hand use_teleop_core 2>/dev/null | awk '{print $NF}')\","
                 echo " \"joy_base.use_teleop_core\": \"$(ros2 param get /joy_base use_teleop_core 2>/dev/null | awk '{print $NF}')\","
                 echo " \"cmd_vel_publisher\": \"$(ros2 topic info /cmd_vel -v 2>/dev/null | awk '/Node name:/{n=$NF} /PUBLISHER/{print n}' | head -1)\"}"
                 kill -INT -- -$LP 2>/dev/null; sleep 3; kill -- -$LP 2>/dev/null; wait 2>/dev/null ;;
  all) for t in joy_hand joy_hand_core marker marker_core base health clamp lift_head joy_base logitech; do
         echo "== $t"; "$0" "$t"; done ;;
  *) echo "unknown test $1"; exit 2 ;;
esac
