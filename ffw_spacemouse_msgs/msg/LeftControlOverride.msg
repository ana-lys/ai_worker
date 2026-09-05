# Decoded left Meta Quest controller state, published on
# /quest/left/control_override by ffw_spacemouse/scripts/quest_to_ros2.py.
#
# Consumed by:
#  - ffw_spacemouse/src/joy_hand.cpp (right-arm instance): trigger scales the
#    right SpaceMouse's translation/rotation sensitivity continuously; grip
#    slowly re-levels the right SpaceMouse goal's yaw/roll back to zero while held.
#  - ffw_collision_checker/src/ffw_ik_solver_teleop.cpp: gripper_vel drives
#    the right gripper's open/close velocity.
#  - ffw_zmqinterface/ffw_zmqinterface/gateway_node.py: record is relayed to
#    the remote client as a Record message (protocol.py MSG_RECORD).

float32 gripper_vel    # left thumbstick Y axis, -1..1. Consumer integrates: pos += gripper_vel * k, then clamps to the gripper's min/max.
float32 trigger        # left analog trigger, 0..1. 0 = released (fast/normal speed), 1 = fully held (slowest). Scales SpaceMouse sensitivity continuously between 1.0 and a configured minimum.
float32 grip           # left grip/side button, 0..1. Held (>0.5) decays the right SpaceMouse goal's yaw and roll toward zero each tick (pitch untouched).
int32 record           # left A/X and B/Y buttons, bit-encoded: 0 = none, 1 = A/X, 2 = B/Y, 3 = both
