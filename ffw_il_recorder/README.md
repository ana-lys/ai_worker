# ffw_il_recorder — Imitation-Learning Episode Recorder

Records teleoperated demos for imitation learning at **30 Hz**: the OAK-D 720p head
view, the right-wrist RGB view (D405 cam1), the full `/joint_states`, and the
controller's **achieved** end-effector poses.

## How the video works

The two video feeds are **not on ROS topics**. The head (OAK-D 720p MJPEG, UDP port
9110) and the right RGB (D405 cam1 "IR" = right view, MJPEG, UDP port 9003) stream
straight to this machine as RTP/MJPEG. The recorder decodes them itself with
`cv2.VideoCapture(GStreamer)`, using pipelines copied **verbatim** from
`ffw_stream/src/realsense_udp_receiver.cpp`.

Because the receiver unicast-binds every port, **the recorder replaces the receiver
during a recording session** — only one process may hold ports 9003/9110.

## Run order

1. **Stop the unified receiver** (it holds the UDP ports):
   ```bash
   # Ctrl-C the receiver terminal / or:
   ros2 launch ffw_stream unified_receiver_launch.py   # <- stop this first
   ```
2. **Start the recorder:**
   ```bash
   cd ~/robotis_ws && source install/setup.bash
   ros2 launch ffw_il_recorder il_recorder_launch.py
   ```

`ros2 launch` runs the node with `output='screen'`, so **keyboard input is live in the
launch terminal**. A live view window also opens (head | right RGB); keys work in
either the terminal or the video window.

## Recording

| Key | Action |
|-----|--------|
| `S` | start an episode → `episode_NNNN/` (refuses until `/joint_states` is seen) |
| `E` | stop + finalize (writes `meta.json`) |
| `Q` / `Ctrl-C` | quit (a still-open episode is finalized on quit) |

Demo while `REC` is shown; press `E` when done.

### Episode layout

```
~/robotis_ws/records/
└── episode_0000/
    ├── state.csv               one row per 30 Hz tick (uniform cadence)
    ├── head/seq_000000.jpg ... every decoded head frame (new decode → new jpg)
    ├── right_rgb/seq_000000.jpg
    └── meta.json               params, timings, counts, joint names, oakd K/D
```

`state.csv` columns: `t` (seconds, 0 at first row), `q_<joint>` per joint in the
start-of-episode `JointState`, `ee_l_x/y/z/qx/qy/qz/qw` and `ee_r_*` (achieved EE
pose xyz + quat), `gripper_l_delta`, `gripper_r_delta` (change of the gripper joint
since the previous `/joint_states` message), then `head_frame`, `head_recv_t`,
`right_frame`, `right_recv_t` (index + receive time of the frame current at that tick).

**Sync model (nearest-sample):** the CSV ticks at a uniform 30 Hz; each row carries the
latest joint/EE values and references the newest frame index received by that tick. A
frame that arrived mid-gap is recorded at the next tick. Frames are never fabricated;
dataloaders nearest-sample `head_recv_t`/`right_recv_t` to `t`.

The right RGB frames are stored **upright** — decoded then rotated 90° CCW (the same
rotation the receiver applies to cam1 "IR").

## Launch arguments

Defaults mirror `ffw_stream/unified_receiver_launch.py` so the recorder drops into the
receiver's slot unchanged:

| Arg | Default | Meaning |
|-----|---------|---------|
| `rgb_source` | `oakd_lite` | informational |
| `oakd_codec` | `mjpeg` | head codec (`mjpeg` \| `h264`) |
| `rs_codec` | `mjpeg` | right RGB codec (`mjpeg` \| `h264`) |
| `oakd_720p_video_port` | `9110` | head stream port |
| `right_rgb_port` | `9003` | D405 cam1 "IR" port (right RGB) |
| `headless` | `false` | set `true` to disable the live window |
| `record_dir` | `~/robotis_ws/records` | parent of `episode_XXXX/` |
| `jpeg_quality` | `95` | JPEG save quality |

Example:
```bash
ros2 launch ffw_il_recorder il_recorder_launch.py headless:=true record_dir:=$HOME/demo_data
```

## Notes

- EE columns are the **achieved** EE poses from `/ik_solver/achieved_ee_pose_{l,r}`
  (same topics `ffw_ee_pose_logger` reads), not the commanded goal.
- The head stream never carries rotation; the OAK-D head is already upright.
- FPS self-report appears on the recorder log every 5 s; a 60 s window averaging
  under 28 fps logs a frame-drop warning.
- Making the receiver and recorder coexist would need multicast or `SO_REUSEPORT`
  on the sender side (a remote-robot streamer change) — out of scope here.
