# ffw_il_recorder — Dataset Definition

What an `episode_NNNN/` actually is: **30 Hz ground-truth state + two on-robot
camera views**, captured while a human teleoperates the dual-arm robot through the
MuJoCo IK stack. This page defines the semantics a learner (or a reader) needs:
which frame the numbers live in, who the observer is, and what the EE columns mean
as an *alternative way to control the arms*.

Reader background: everything below assumes you know the package's run order and
file layout (`README.md`). The control-side details referenced here live in
`ffw_spacemouse/quest_teleop_plan.md` and `ffw_ik_solver_teleop.cpp`.

---

## 1. Quick answer

| Column group | Meaning | Coordinate frame | Observer / source | Units |
|---|---|---|---|---|
| `t`, `*_recv_t` | episode clock | — (time) | recorder wall clock | seconds, 0 at first row |
| `q_<joint>` | joint angles | joint space (per-joint axis) | `/joint_states` | **radians** (raw JointState) |
| `ee_l_*`, `ee_r_*` | **achieved** end-effector pose | **`map`** (fixed robot world frame) | `/ik_solver/achieved_ee_pose_{l,r}` | xyz **metres**, orientation **quaternion xyzw** |
| `gripper_l/r_delta` | gripper motion | gripper joint axis | `/joint_states` (per-arrival diff) | radians change |
| `head/seq_*.jpg` | egocentric view | camera on the robot **head** | OAK-D 720p (UDP 9110) | 1280×720 JPEG |
| `right_rgb/seq_*.jpg` | hand-eye view | camera on the **right wrist/forearm** | D405 cam1 "IR" (UDP 9003) | 1280×720 JPEG (stored upright) |

There is **no external, third-person camera** in the dataset. Every image is taken
by a camera mounted on the robot itself; every state value is the robot/controller's
own ground truth.

---

## 2. The `map` frame — where the EE data lives

All EE poses (`ee_l_*`, `ee_r_*`) are expressed in the ROS frame **`map`**
(`msg.header.frame_id = "map"` on the `/ik_solver/achieved_ee_pose_{l,r}` messages).
The recorder copies those values straight from the messages (rounded to 6
decimals for storage); it does not transform the frame or the convention.

`map` is **the fixed world frame of the MuJoCo IK scene** the solver runs
(`ffw_collision_checker` scene XML). In the plan doc's own words, saved CLI poses
are in the *"Absolute robot world/"map" frame (MuJoCo world, base at origin, ground
z=0)"*, and home is real, meaningful coordinates — e.g. left EE
`(0.0055, 0.2275, 0.513)`, right `(0.0055, -0.2275, 0.513)`.

Concretely:

- **Origin / axes:** `z = 0` is the **ground / floor plane**, `+z` points up;
  `+y` is the robot's **left** side (the *left* arm chain sits on `+y`, the *right*
  arm on `-y`); `+x` runs along the robot's forward axis. Both arms' centerline
  sits at `x ≈ 0.0055`.
- **It is static for the whole episode.** The IK scene has **no freejoint on the
  robot base**, so the base is pinned in the world while the solver runs. Recording
  sessions do not drive the base. A row's EE pose therefore has a fixed, absolute
  meaning: *where the end effector actually is, in room/robot space, at that tick.*
- **Position is metres, orientation is a unit quaternion `(x, y, z, w)`.** No
  Euler angles are stored. If you need RPY, the project's convention is
  extrinsic-XYZ (see `protocol.py` / `ffw_ee_pose_logger`'s saved JSON), but the
  dataset itself keeps quaternions so there is no convention ambiguity.
- `map` is also the frame the **saved CLI poses** (`poses.txt`) and the **Quest**
  absolute goals are expressed in, so EE columns, offline goals, and the solver all
  live in one coordinate universe. The only input that does **not** carry real map
  meaning is the SpaceMouse mapper's raw delta stream (see §4) — its numbers are
  incremental, and it is the *result* the dataset records, not the mapper's internal
  goal.

### Joint columns are frame-free

`q_<joint>` values are raw `sensor_msgs/JointState` positions — **radians** about
each joint's own axis, no common frame. The set of joint columns is fixed at
episode start from the first JointState message, so arm, gripper, head, and lift
joints that are present become columns (`q_arm_l_joint1`, … , `q_gripper_l_joint1`,
`q_head_joint1`, …). A joint absent from a later message renders blank for that row.

---

## 3. The observer(s)

The dataset has two *kinds* of observation, and they come from different
"observers" — know which is which when you fuse them.

### 3.1 Head camera — egocentric, first-person

`head/seq_*.jpg` is the **OAK-D 720p** view. The OAK-D is mounted on the robot's
**head**, and the head moves: the head pan/tilt joints appear in `q_*`. So the
head image is the robot's *own* viewpoint — what it is currently looking at. It is
the natural primary input for a vision policy ("what the robot sees").

- Stored as received from the MJPEG stream (the OAK-D head is already upright; the
  stream never carries rotation metadata).
- The camera's per-frame pose is **not** stored, but is **recoverable**: forward
  kinematics on `q_*` (head joints) plus the known mount transform give the camera
  origin/axes in `map` for any row.

### 3.2 Right-wrist camera — hand-eye

`right_rgb/seq_*.jpg` is the **D405 cam1 "IR"** stream decoded and **rotated 90°
CCW so it is stored upright** (the same rotation the receiver applies). This camera
rides the **right arm**, near the gripper — a hand-eye / wrist view of the
workspace and the manipulated object from the arm's own perspective.

- Rides on a **moving** link: it is *not* fixed in `map`; its pose per row is
  recoverable from FK on the right-arm `q_*` joints (arm chain + wrist mount).
- Only the **OAK-D** head intrinsics (K/D, frame_id) are snapshotted into
  `meta.json`; D405 (right-camera) intrinsics are **not** stored in the episode.

### 3.3 State — ground-truth controller observer

`q_*`, `ee_*`, `gripper_*_delta` are **ground-truth measurements from the robot
itself**:

- `q_*` — real encoder positions published on `/joint_states`.
- `ee_*` — the IK solver's **achieved site pose** (`d->site_xpos`) read every main
  loop and published in `map`.

So the state observer is the **controller, in simulation-aware ground truth** — not
a human-pose estimate, not an external tracker, not a noisy perception stack. That
makes the state columns a *privileged* channel: they are exactly the quantities the
teleop controller used, which is what lets a downstream policy train on
action-consistent supervision (§4) instead of on inferred state.

Nothing in the dataset records the human operator (no body pose, no gaze, no
exo/hand trackers). The human is the *driver*; only its effect on the robot is
recorded.

---

## 4. The EE columns = end-effector control, the "alternative way to control"

The arms can be commanded in **two different spaces**. The EE columns are what the
second space produces.

### The two command spaces into the solver

Everything converges on the one IK solver node (`ffw_ik_solver_teleop.cpp`), which
holds the MuJoCo scene, clamps targets for safety, and solves joints. It has two
goal-update paths:

| Channel | Topic(s) | Command space | Meaning |
|---|---|---|---|
| **Joint space** | `/leader/joint_trajectory_command_broadcaster_{left,right}/joint_trajectory` | joint angles | "put each motor at this angle" — direct joint command, e.g. from a leader arm |
| **End-effector space** (the alternative) | `/spacemouse/{left,right}/ee_target_pose` (delta), `/quest/{left,right}/ee_target_pose` (absolute), saved CLI poses | **Cartesian / task space** | "put the end effector at this position **and** orientation in space" |

End-effector control is the classic *task-space* alternative to joint control: you
command **where the hand should be**, not each motor. The IK solver does the
inverse kinematics to find the joints that realize the target. Because the arms are
redundant, that mapping is non-trivial — which is exactly why the solver is a MuJoCo
scene rather than a per-arm analytic IK.

The three EE sources differ in *how much* they say:

- **SpaceMouse mapper (`joy_hand.cpp`)** — *incremental / delta* control. Its
  internal goal starts at identity and integrates the device deltas; the stream is
  "relative" by nature. The solver integrates these deltas on top of a
  hardware-synced starting pose (`target = initial + accum`), and
  `clip_target()` keeps the target inside a **1 cm / 0.1 rad ball around the
  achieved** site pose every tick (re-syncing `accum`, so the invariant
  `target = initial + accum` holds).
- **Quest (`quest_to_ros2.py`)** — *absolute* control, torso-relative ≈ map:
  "head (x,y) on the ground (z=0), rotation = head yaw, z preserved = ground-
  relative height" — already in the same frame as the saved poses.
- **Saved CLI poses (`poses.txt`)** — absolute `map`-frame goals loaded into the
  solver (the same coordinate universe as `ee_*`).

All paths share the same clamps: `clip_target()` ball (1 cm / 1.5 cm approach /
6 cm track translation, 0.1 rad rotation) and `clamp_target_angle()` (90° absolute
rotation from identity). Head is frozen while arms solve. That is the safety layer
that makes EE-space command practical at teleop speed.

### Why the dataset records the *achieved* pose, and why that is the right EE target

The `ee_*` columns are **not** the commanded goal and **not** the mapper's internal
`ee_goal_`. They are the **achieved** MuJoCo site pose — where the controller
actually ended up, in `map`, each tick — published on
`/ik_solver/achieved_ee_pose_{l,r}` and copied verbatim by the recorder.

That choice is deliberate:

1. **It is control-source-agnostic.** Whether the demonstrator drove the arm with
   the SpaceMouse (deltas), the Quest (absolute), saved poses, or the leader arm in
   joint space, the recorded EE trajectory is the same kind of quantity: the 
   achieved end-effector pose in the fixed `map` frame. One demonstration target,
   regardless of input device.
2. **It is what an autonomous EE-space policy would reproduce.** A Cartesian policy
   trained on these columns outputs a `map`-frame EE pose per tick; feeding that
   pose as the solver target is exactly the absolute path (`/quest/...` absolute,
   or saved-pose load) the stack already executes — the solver finds the joints and
   tracks it under the same clamps. So imitation in EE space is **executable with
   the same controller that recorded the data**, with no sim-to-sim gap on the
   action side.
3. **Joint space is preserved alongside it.** The `q_*` columns give the
   joint-space (leader-arm-style) alternative from the same demonstration, and
   `gripper_*_delta` carries the binary-ish gripper open/close channel separately.

In short: `ee_l_*`/`ee_r_*` are the **demonstrated end-effector behavior in a fixed
world frame** — the dataset's primary *action* ground truth for a Cartesian policy,
and a per-row *state* for anything that needs the hand pose. Joint and image
channels stay available as the other two representations of the same demonstration.

---

## 5. Reading the rows (sync model recap)

- `t` ticks at a **strict uniform 30 Hz** (recorder wall clock, 0 at the first
  row). Each row samples the **latest** joint/EE message (best-effort, depth 1) and
  references the **newest** frame index received by that tick.
- Frames are written **only when a new decode arrives** (nominal ~30 Hz), as
  `head/seq_%06d.jpg` / `right_rgb/seq_%06d.jpg`. A frame that arrived mid-gap is
  referenced by the next tick — nearest-sample. `*_recv_t` is the frame receive
  time in seconds relative to episode start, so a dataloader aligns images to rows
  by nearest `recv_t` to `t`. Frames are never fabricated or interpolated.
- `head_frame` / `right_frame` are the current frame **index** (the `seq_NNNNNN`
  number), so a row maps to exactly one jpg per camera.
- Images decode to **BGR** via `cv2.imread`, 1280×720, JPEG quality 95.
- `meta.json` per episode records: start/end wall times, duration, `sample_hz`,
  codecs/ports, frames written, the joint-name list (maps `q_*` columns → joints),
  the OAK-D camera_info (K/D), and the `state.csv` column list — read it to
  interpret a directory without re-deriving anything.

---

## 6. Things the dataset intentionally does *not* have

- No third-person / external camera view.
- No depth or point clouds (feeds are MJPEG color only).
- No human-operator tracking (body/gaze/exo).
- No per-frame camera pose columns (recoverable by FK, §3 — but not precomputed).
- No commanded-goal columns — only achieved EE. If you need the *goal* trajectory
  (mapper `ee_goal_` before clamping), that is a solver-internal quantity and is
  not recorded; the achieved pose is the closest available and is the executable
  target (§4).
