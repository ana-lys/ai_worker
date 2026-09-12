# ffw_odom

ROS 2 package providing odometry estimation, EKF sensor fusion, and scan-to-map ICP localization correction for the FFW robot.

## Package Overview

This package contains two main executables:
1. **`rf2o_laser_odometry_node`**: Computes fast planar 2D odometry from raw LIDAR scans.
2. **`scan_to_map_icp`**: A fast, robust 2D point-to-line scan matcher that corrects raw odometry drift by registering a live LIDAR scan against a static map (wall features).

---

## Scan-to-Map ICP Matcher (`scan_to_map_icp`)

The `scan_to_map_icp` node implements a point-to-line registration algorithm designed for structured, wall-like indoor environments.

### Core Architecture & Optimizations
* **Precomputed Map Normals**: The static map is loaded once at startup. Local line normals are precomputed using local Principal Component Analysis (PCA) over each point's $k$-nearest neighbors. This eliminates runtime normal calculation overhead.
* **Point-to-Line Residuals**: Residuals are calculated as point-to-line (point-to-plane in 2D) distances, yielding fast and robust convergence on wall structures with minimal overlap bias.
* **Identity Transform Shortcut**: If the incoming `LaserScan`'s `frame_id` is identical to the target `base_frame` (e.g. `base_link`), the static transform lookup is skipped and set to identity, saving TF resources.
* **Robust Convergence & Fallback**:
  * Optimization thresholds are configured at $\mathrm{d}t = 10^{-4}$ meters and $\mathrm{d}\theta = 10^{-5}$ radians to prevent sub-millimeter limit cycle oscillations on sparse maps.
  * In addition to strict convergence criteria, a fallback gate accepts alignments that reach a high-quality local fit (inlier RMS error `< 10` cm with at least 10 matching points), ensuring continuous localization updates.

### Subscribed Topics
* **`/scan`** (`sensor_msgs/msg/LaserScan`): The merged/input laser scan.
* **`/odom`** (`nav_msgs/msg/Odometry`): The raw odometry input.

### Published Topics & Transforms
* **`/odom_corrected`** (`nav_msgs/msg/Odometry`): Corrected odometry in the `map` frame.
* **TF Broadcast (`map` &rarr; `odom`)**: The dynamic correction transform calculated by the ICP matcher.

### Parameters
* `map_file` (string, required): Absolute path to the static wall feature text file (e.g. `all_walls_downsampled_rotated.txt`).
* `scan_topic` (string, default: `/scan`): Input laser scan topic.
* `odom_topic` (string, default: `/odom`): Input raw odometry topic.
* `corrected_odom_topic` (string, default: `/odom_corrected`): Output corrected odometry topic.
* `map_frame` (string, default: `map`): Target map frame ID.
* `odom_frame` (string, default: `odom`): Odometry frame ID.
* `base_frame` (string, default: `base_link`): Robot base frame ID.
* `max_correspondence_dist` (double, default: `0.4`): Maximum distance in meters to match scan points to map features.
* `max_accepted_rms` (double, default: `0.10`): Maximum allowed inlier RMS error to accept non-converged but high-quality alignments.
* `verbose` (boolean, default: `false`): Enables verbose iteration details for debugging.

---

## Marker-Frame EKF (`marker_pose_corrector` + `ekf_node` + `marker_frame_tf_broadcaster`)

Publishes a smooth, high-rate (50 Hz) `base_link -> marker_frame` TF by
fusing continuous wheel/swerve odometry (`/odom`, velocity only) with a
low-rate (~4-5 Hz) AprilTag marker-board absolute pose correction — the same
`robot_localization` `ekf_node` pattern already used above for the map-frame
fusion (odom + ICP correction), just anchored to the marker board instead of
the static wall map, so no map/localization stack is required. Added here as
a convenience TF utility for the teleop "global limit" feature
(`ffw_spacemouse`/`ffw_collision_checker`) even though the rest of this
package is 2D lidar odometry/localization.

Three nodes, in a pipeline:
* **`marker_pose_corrector`**: subscribes `/oakd/marker_board_pose`
  (published by `ffw_depthai`'s AprilTag board detector), applies a fixed
  `board_yaw_offset_rad` correction about the marker's own local Z axis so
  its axes read as ROS front/left/up, inverts it into an absolute
  base_link-in-marker_frame measurement, and publishes it as
  `geometry_msgs/PoseWithCovarianceStamped` on `/oakd/marker_frame_baselink_pose`.
* **`ekf_node`** (`robot_localization`, config `marker_ekf.yaml`, node name
  `marker_ekf_filter_node`): fuses that correction with `/odom` into
  `/marker_ekf_odom`. **`publish_tf` is deliberately `false`** here —
  robot_localization always broadcasts `world_frame -> base_link_frame`
  (base_link as the *child*), but base_link already has a real parent from
  the existing odom/map stack; a second competing parent splits the TF tree
  ("two or more unconnected trees") and broke `scan_to_map_icp`'s
  odom↔base_link lookups when this was first tried with `publish_tf: true`.
* **`marker_frame_tf_broadcaster`**: subscribes `/marker_ekf_odom` and
  broadcasts the *inverse* as TF: `base_link -> marker_frame`. This makes
  `marker_frame` a **leaf** hanging off `base_link` instead of a competing
  root, so it can never conflict with the odom/map tree. TF lookups are
  direction-agnostic, so nothing downstream (`joy_hand.cpp`, the CLI) cares
  which direction was actually broadcast.

### Important: this frame's parameters must match `marker_ekf.yaml`
`marker_pose_corrector`'s `child_frame` (default `marker_frame`) and the
frame names baked into `marker_ekf.yaml` (`odom_frame`/`world_frame`, both
`marker_frame`) must agree by hand — there's no shared config.

### `marker_pose_corrector` parameters
* `source_frame` (string, default: `base_link`): Frame the raw marker pose arrives in.
* `child_frame` (string, default: `marker_frame`): World-anchor frame name used
  downstream (must match `marker_ekf.yaml`'s `odom_frame`/`world_frame`).
* `board_yaw_offset_rad` (double, default: `pi/2`): Fixed yaw correction about
  the marker's own Z axis. Sign/axis convention is unverified — check with
  `ros2 run tf2_ros tf2_echo base_link marker_frame` and adjust if needed.
* `input_topic` (string, default: `/oakd/marker_board_pose`): Input pose topic.
* `output_topic` (string, default: `/oakd/marker_frame_baselink_pose`): Output
  pose-correction topic consumed by `marker_ekf.yaml`'s `pose0`.
* `position_stddev_m` / `yaw_stddev_rad` / `orientation_stddev_rad`: diagonal
  covariance filled on the output measurement.

### `marker_frame_tf_broadcaster` parameters
* `input_topic` (string, default: `/marker_ekf_odom`): The filtered odometry
  topic to republish as TF (inverted).

```bash
ros2 launch ffw_odom marker_ekf.launch.py
```

---

## Launch Instructions

### Launching Odometry Fusion (rf2o + EKF)
To start the planar laser odometry and fuse it with the wheel/swerve odometry:
```bash
ros2 launch ffw_odom odom.launch.py use_ekf:=true
```

### Launching Scan-to-Map ICP Localization
To launch the scan matcher with the default map:
```bash
ros2 launch ffw_odom scan_to_map_icp.launch.py
```
To specify a custom map file path:
```bash
ros2 launch ffw_odom scan_to_map_icp.launch.py map_file:=/path/to/map.txt
```
