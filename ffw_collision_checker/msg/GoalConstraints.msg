# Live EE-goal constraints of one joy_hand mapper arm, latched
# (transient_local) and republished on change on
# /spacemouse/<arm>/goal_constraints. ffw_ik_shadow_solver clamps query goals
# with these exactly like the mapper clamps teleop goals.
#
# Box layout (local and global):
#   [px_min px_max py_min py_max pz_min pz_max
#    roll_min roll_max pitch_min pitch_max yaw_min yaw_max]
# meters / radians, extrinsic XYZ RPY (R = Rx(roll) * Ry(pitch) * Rz(yaw)).

string arm          # "left" | "right"
string goal_frame   # native frame of the goal (joy_hand's ee_goal_frame)

# EE orientation locks (/ik_solver/ee_lock): clamp to center +- slack.
bool roll_locked
bool pitch_locked
bool yaw_locked
float64 locked_roll
float64 locked_pitch
float64 locked_yaw
float64 orientation_slack

# Local limit profile box, in goal_frame.
bool local_active
float64[12] local_bounds

# Global limit profile box, in global_frame (e.g. marker_frame).
bool global_active
string global_frame
float64[12] global_bounds
