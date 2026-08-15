# Quest Controller → Manipulator (ARM) Teleop Override — Implementation Plan

Goal: let the Meta Quest Touch controllers **override the SpaceMouse for arm control**
in ARM mode, mirroring exactly how the Logitech Extreme 3D Pro overrides the
SpaceMouse for base control in BASE mode.

---

## 1. The Logitech pattern we are copying (base mode)

```
logitech_base_teleop.py                         joy_base.cpp
─────────────────────────                        ──────────────────────────
subscribes /joy_logitech                         subscribes /logitech/base_active
hold Trigger+Thumb 2s → toggle `active`          if logitech_active_ → return (no cmd_vel)
publishes Bool → /logitech/base_active
```

Key ideas:
1. The **controller-side node owns the hold-gesture detection** and publishes a
   simple `Bool` "active" flag on a dedicated topic.
2. The **base teleop node just consumes that flag** and suppresses its own output
   when it is true. It never knows *why* the other controller is active.

For the Quest we want the same thing in **ARM mode**, except the consumer is the
mapper (`joy_hand.cpp`) and the override does not just *suppress* — it *steers*
the shared 6-DOF end-effector target.

---

## 2. The shared pose variable (how SpaceMouse drives the arm today)

`joy_hand.cpp` owns `ee_goal_` (an `Eigen::Isometry3d`).

- On a 100 Hz `control_timer_callback()` (skipped unless `current_mode_ == "ARM"`):
  - reads latest `/…/joy` axis values,
  - integrates them into `ee_goal_` (velocity → pose stepping, cubic + precision scale),
  - applies EE soft-locks,
  - publishes `PoseStamped` → `/spacemouse/<left|right>/ee_target_pose`.

`ffw_ik_solver_teleop.cpp` consumes that pose **by delta**, not absolutely:

```cpp
// pose_callback_l / pose_callback_r (ffw_ik_solver_teleop.cpp ~line 1306)
delta_trans = trans - last_mapper_l_trans_;
accum_l_trans_ += delta_trans;
target_l_.translation() = initial_l_.translation() + accum_l_trans_;
...
```

So the solver integrates the mapper's *deltas* on top of a hardware-synced
`initial_*` pose. This is why the Quest override must happen **inside the mapper**
(interpolating `ee_goal_` and letting the deltas flow out), NOT by publishing an
absolute pose directly to the solver.

**Clamping already exists and we reuse it for free:**
- `clip_target()` (ffw_ik_solver_teleop.cpp:639–680) is a **ball clamp**, not a
  step leash: each tick it projects the solver target to within
  `max_dist = 0.01 m` and `max_angle = 0.1 rad` of the **achieved** site pose, and
  **re-syncs `accum`** so `target = initial + accum` stays an invariant. So no
  re-base delta injected by the mapper can ever make the solver target leave the
  1 cm / 0.1 rad ball around achieved — arm jumps are bounded by that ball, not by
  any mapper-side step.
- `clamp_target_angle()` (~line 1429) bounds absolute rotation from identity to 90°.
- Solver also freezes `head` and can freeze arm groups / collide-check.

So "still clamped by the solver just like the SpaceMouse" = nothing new needed on
the solver side. A mapper-side leash was considered and skipped — the mapper has
no achieved-pose feedback to leash against (§6).

---

## 3. What the Quest already gives us (`quest_to_ros2.py`, already in package)

Published (torso-relative, ROS frame "map", Unity→ROS converted):

| Topic | Type | Contents |
|---|---|---|
| `/left_controller_pose` | `PoseStamped` | Left Touch controller 6-DOF pose |
| `/right_controller_pose` | `PoseStamped` | Right Touch controller 6-DOF pose |
| `/head_pose` | `PoseStamped` | Head pose (torso reference) |
| `/quest_state` | `Float32MultiArray` | 38 floats: left + right block (pose + analog + button floats) + head pose (§3) |

`/quest_state` layout — 38 floats (per-arm block = pose 6 + analog 4 +
button floats 6; head block = pose 6). **Change (user decision 2026-08-01):
`quest_to_ros2.py` is now modifiable — the button bitmasks become
individual float values** (touches are dropped entirely — nothing reads them),
so the mapper reads button state by index with no bitmask decode:

```
Left :   0-5  pose (x,y,z,roll,pitch,yaw)
         6    trigger   (analog 0-1)
         7    grip      (analog 0-1)
         8    stick_x   (analog -1..1)
         9    stick_y   (analog -1..1)
        10    trigger_btn      (0.0/1.0)
        11    grip_btn         (0.0/1.0)
        12    a_x_btn          (0.0/1.0)
        13    b_y_btn          (0.0/1.0)
        14    thumbstick_btn   (0.0/1.0)
        15    menu_btn         (0.0/1.0)
Right :  16-31  (same block, offset +16)
Head  :  32-37  pose
```

The analog floats (6/7 = trigger/grip) drive the keep-alive and engage gesture;
the button floats are there for any future gesture. The mapper reads poses +
button states from `/quest_state` alone (quest poses are indices 0-5 / 16-21) —
the `/left|right_controller_pose` topics stay for other consumers.
`quest_to_ros2.py` still does **no control logic** — it stays a telemetry bridge;
only its message format is owned by this feature.

> **Pose convention:** `(x, y, z)` metres + `(roll, pitch, yaw)` radians, extrinsic
> XYZ (`R = Rx(roll)·Ry(pitch)·Rz(yaw)`) — the same convention `extract_rpy()`
> uses in `joy_hand.cpp` (joy_hand.cpp:150–156), so a `toIsometry()` built from the
> 6 floats is the exact inverse of the existing decomposition.

---

## 4. Quest override state machine (lives in `joy_hand.cpp`)

Mirror of the Logitech hold-gesture, adapted to ARM mode.

**Analog "thumb" keep-alive:** the thumb is an analog float in [0,1] (1.0 = fully
engaged). Quest mode stays active only while **thumb ≥ 0.5** on **both** hands.
The "thumb" analog **is the grip float** (`quest_thumb_analog_offset` 7),
hardware-verified: grip fires reliably (§10 decision 7). Touches are not in
`/quest_state` at all — nothing in the state machine reads them.

```
         ┌────────────────────────────────────────────────────────────┐
         │                        SM_CONTROL                          │  ← default (SpaceMouse)
         │  ee_goal_ += velocity from SpaceMouse (existing path)      │
         └────────────────────────────────────────────────────────────┘
              ▲                     ▲                        │
              │  thumb<0.5 (either) │  (early release)       │  both hands THUMB≥0.5
              │                     │   trigger<0.5 before    │  AND TRIGGER≥0.5 held 2 s
              │                     │   threshold → re-arm    ▼
         ┌────┴───────┐        ┌──────────────────────────┐   ┌──────────────────────────┐
         │ QUEST_TRACK│◀───────│      QUEST_READY         │◀──│     QUEST_APPROACH (slow) │
         │ w = w_fast │ trigger │ (fast armed, waiting for │   │   w = w_slow              │
         │            │ release │  trigger release)        │   │   ee_goal_ → quest target │
         └────┬───────┘        └──────────────────────────┘   └──────────────────────────┘
              │  thumb<0.5                                     │
              └──────────────────▶ SM_CONTROL ◀────────────────┘
```

### States

| State | Enter condition | Behavior | Exit condition |
|---|---|---|---|
| `SM_CONTROL` | default / any exit below | SpaceMouse velocity integration (unchanged) | all four floats at zero, then both hands **thumb ≥ 0.5 AND trigger ≥ 0.5** held **2 s** (edge), **gated on a fresh achieved pose** (§4 Gestures) → engage |
| `QUEST_APPROACH` | engage edge (2 s hold complete) | weak interpolation `w_slow` toward quest target; still clamped by solver | **error < threshold** → fire notification → `QUEST_READY`; **either trigger < 0.5 early** (before threshold) → `SM_CONTROL` (fast locked, must re-arm from all-zero); **thumb < 0.5** (either) → `SM_CONTROL`; **no error progress for `quest_no_progress_s`** → `SM_CONTROL` (§10) |
| `QUEST_READY` | error < threshold reached | hold current interpolated pose; notification already sent ("release trigger to go fast") | **either trigger < 0.5 (thumbs kept ≥ 0.5)** → `QUEST_TRACK`; **thumb < 0.5** (either) → `SM_CONTROL` |
| `QUEST_TRACK` | trigger released after READY | strong interpolation `w_fast`; 1:1 tracking; still clamped | **thumb < 0.5** (either) → `SM_CONTROL` |

> **Exits not repeated per row:** every QUEST state also returns to `SM_CONTROL`
> on the watchdog timeout (§9) and on `/teleop_mode` flipping to BASE (§9); the
> §4 "Abort" freeze applies to all of them.

### Gestures (parameterized)

- **Engage:** the 2 s countdown only **arms** when all four floats are fully
  released — both thumbs AND both triggers below `quest_release_min` (default 0.1)
  — then requires an uninterrupted **THUMB ≥ 0.5 + TRIGGER ≥ 0.5** on **both** hands
  for **2.0 s** (`quest_hold_engage_s`). A partial or carried-over hold never
  engages: after any interruption the four floats must return to zero before a new
  countdown may start. The "4 buttons" = 2 hands × (thumb + trigger).
- **Where the state machine runs:** the Quest state machine — including the engage
  detection that leaves `SM_CONTROL` — runs on **every ARM tick** in
  `control_timer_callback()`, whether or not the override is currently active (§7).
  In `SM_CONTROL` the engage check runs and then the normal SpaceMouse path
  proceeds; the instant engage completes, the Quest branch takes over and
  SpaceMouse is suppressed.
- **Keep-alive:** THUMB ≥ 0.5 must be maintained on **both** hands in every QUEST
  state. Either thumb dropping below 0.5 returns to `SM_CONTROL` immediately
  (publishes inactive flag + log).
- **Go-fast confirmation:** in `QUEST_READY`, releasing **either** TRIGGER (keeping
  both thumbs ≥ 0.5) enters `QUEST_TRACK`.
- **Early-release lockout:** if **either** TRIGGER is released while still in
  `QUEST_APPROACH` (error ≥ threshold), fast mode is NOT granted — the user must
  release all four to zero, then re-do the full 4-button, 2 s hold to re-arm and
  try again.
- **Engage requires a live achieved pose:** the engage edge only completes when an
  achieved pose has been received **and** is fresh (`header.stamp` age <
  `quest_achieved_max_age_s`, default 0.5 s). On startup, or after a long dropout,
  the first engage attempt is ignored (log + notification) until the achieved
  topic is live — a re-base from a default-constructed pose would produce exactly
  the engage jump §5 exists to prevent.
- **Hold timer reset / re-arm:** the 2 s engage countdown restarts if any of the
  four analogs drops below threshold mid-hold, `/quest_state` goes stale within
  `quest_timeout_s`, or the mode leaves ARM. On any such interruption the count is
  **disarmed** — all four must return below `quest_release_min` before a new
  countdown can start. The hold is an uninterrupted 2 s, not a cumulative one.
- **Threshold hysteresis:** `quest_hold_hysteresis` (default 0.05). Once engaged,
  the keep-alive / early-release comparisons use `≥ 0.5 − 0.05` (or `< 0.5 − 0.05`
  to exit) instead of the raw 0.5 edge, so a thumb hovering on the 0.5 line does
  not tremble the state machine in and out of QUEST_TRACK.
- **Per-tick evaluation order** (in a QUEST state, each 100 Hz tick): (1) either
  thumb < keep-alive → `SM_CONTROL`; (2) still in `QUEST_APPROACH` and error <
  threshold → `QUEST_READY` (notification); (3) still in `QUEST_APPROACH` and
  **either** trigger < keep-alive → early release → `SM_CONTROL`; (4) in
  `QUEST_READY` and **either** trigger < keep-alive → `QUEST_TRACK`. If the error
  crosses threshold on the same tick the trigger releases, (2) fires first and the
  release immediately enters `QUEST_TRACK` — no spurious early-release lockout.

### Abort → SM_CONTROL: freeze the goal, reset the count

Every path back to `SM_CONTROL` (thumb < 0.5, early trigger release, watchdog
timeout, mode switch) does exactly the same thing (user decision 2026-08-01):

1. **Stop listening to the quest** — the mapper stops updating `ee_goal_` from
   `/quest_state`; the goal **stays where it is** (no re-anchor, no snap, no jump).
2. **Push the state back to its pre-count state** — `SM_CONTROL`, count **disarmed**:
   the next engage must start from all four floats at zero again (§4 Gestures). No
   partial hold and no latched override survives into the next session.

Why freezing is enough (no re-anchor needed): under the ball clamp (§2) the solver
target is always within the 1 cm / 0.1 rad ball around the achieved pose, so the
instant the mapper stops moving the goal the arm holds where it is — the solver
does not chase a frozen goal, because its target is `initial + accum` and `accum`
has stopped changing. If the goal ran ahead of the arm during TRACK, the arm simply
stays put until the SpaceMouse is commanded again, then closes the gap at leash
speed; the §11 drift re-base snaps the reference back to `achieved` once an idle
gap exceeds its threshold. No exit depends on measuring the real arm, so none of
these paths carries a sim-vs-real caveat.

### Threshold-reached notification

The transition `QUEST_APPROACH → QUEST_READY` is the "you can go fast now" moment.
The user has confirmed this should be an explicit event, so it:
1. **prints** a line (e.g. `"APPROACH COMPLETE — release TRIGGER to go fast"`), and
2. **publishes** `std_msgs/String` on **`/ffw_control/notification`** (payload =
   `"quest_approach_reached"` or similar) — fired by **one** `joy_hand` instance
   only (§7: gated on `quest_publish_notifications`), so two arms approaching
   together do not double-fire the notification.

Future hooks the user mentioned: play a sound on this event, or rumble the
SpaceMouse, so the operator knows "if you release trigger now we go fast". The
topic is the stable interface those will plug into.

Per the user: **the event fires only when the error is below threshold** — the
2 s engage hold is complete by construction, since `QUEST_APPROACH` only exists
after it.

---

## 5. Frame mapping — verified 2026-07-31 (user's point 3)

### Verified finding: the SpaceMouse teleop does NOT share the saved-pose frame

I checked all three frames directly:

| Source | Frame | Evidence |
|---|---|---|
| **Saved CLI poses** (`poses.txt`) | **Absolute robot world/"map" frame** (MuJoCo world, base at origin, ground z=0) | Saved from the solver's `target_l_/target_r_ = initial_* + accum`, where `initial_*` = MuJoCo `site_xpos`. Home: left EE `(0.0055, 0.2275, 0.513)`, right `(0.0055, -0.2275, 0.513)` — real, meaningful coordinates |
| **SpaceMouse mapper** (`joy_hand.cpp`) | **Arbitrary delta frame** — NOT the real frame | `ee_goal_` starts at `Eigen::Isometry3d::Identity()` = `(0,0,0)` and integrates SpaceMouse deltas; `frame_id="map"` is just a label. The absolute numbers are meaningless |
| **Quest** (`quest_to_ros2.py`) | **Torso-relative ≈ robot map frame** (user is right) | `torso_relative()`: origin = head (x,y) on ground (z=0), rotation = head yaw, **z preserved = ground-relative height**. Exactly "x, y, orientation relative to torso, z relative to ground" |

Why the SpaceMouse mapper can get away with a fake frame: the solver re-bases on the
FIRST mapper message and integrates only deltas —

```cpp
if (first_msg_l_) { last_mapper_l_trans_ = trans; first_msg_l_ = false; }
delta_trans = trans - last_mapper_l_trans_;   // only deltas matter
accum_l_trans_ += delta_trans;
target_l_ = initial_l_ + accum_l_trans_;       // re-based onto real robot pose
```

### The catch for the Quest (this is the key subtlety)

The user's intuition is correct: **the Quest torso-relative pose is in the same
coordinate universe as the saved poses** (robot map frame, z from ground), provided
the operator stands aligned with the robot base. The saved poses reaching z ≈
1.3–1.8 m (Messi / grappler) are exactly operator-hand height, which is why they
"feel" like they match the Quest output.

**BUT** the interpolation target goes into `ee_goal_`, which lives in the mapper's
fake delta frame. If we feed the Quest pose `Q` directly as the interpolation target
for `ee_goal_`:

```
robot lands at  initial + (Q − P_engage)      // NOT at Q
```

i.e. the robot moves by a **relative vector** (the difference between the
engage-time pose and the quest pose), not to the absolute quest location. It would
mirror hand motion but not land where the hand is.

### Fix: re-base `ee_goal_` onto the robot's achieved pose at engage

To get **absolute** positioning ("arm goes to where my hand is"), the mapper must
know the robot's achieved EE pose in map frame at the moment of engage, then
interpolate toward the (map-frame) Quest pose:

```
at engage:   ee_goal_  = achieved_pose_map      (no jump — arm is already there)
             target    = quest_pose_map         (absolute, already in map frame)
per tick:    ee_goal_  = slerp(ee_goal_, target, w)   // w_slow then w_fast
```

`target` is **re-read from the latest `/quest_state` every tick** — it is never
latched at engage. The engage-time re-base only sets the *starting reference*; a
moving hand keeps the error up and delays READY (correct), and TRACK follows the
live hand, not the pose that happened to be current at engage.

Because `ee_goal_` is now expressed in the real map frame, the deltas the solver
receives (`Q − achieved`) drive the solver target toward the Quest pose,
ball-projected each tick — so the arm converges smoothly, and the *rate* of
convergence is bounded by the 1 cm / 0.1 rad ball clamp (§2): the arm reaches the
Quest pose, it does not teleport. `clamp_target_angle` (90° absolute cap) still
applies on the solver side.

> **Engage freshness gate:** the engage re-base reads the latest cached achieved
> pose, so the engage edge is gated on that cache being populated AND fresh — age
> measured from the mapper's **own receive time** (an `rclcpp::Time` recorded in
> the achieved-pose callback), NOT the message `header.stamp` (which is stamped by
> the solver's publisher clock and can be old under load). Age <
> `quest_achieved_max_age_s` (§4) — never re-base from an uninitialized/
> default-constructed pose.

The solver already knows the achieved pose (`target_l_/target_r_` and the MuJoCo
site pose each tick) but **does not publish it** — so we add one small topic:

- **`ffw_ik_solver_teleop.cpp`**: publish the achieved EE pose per arm as two
  `PoseStamped` topics, `/ik_solver/achieved_ee_pose_l` / `_r` (§10).
- **`joy_hand.cpp`**: subscribe; at engage, re-base `ee_goal_` from it.

> **Which pose is "achieved"?** The **MuJoCo site pose** (`d->site_xpos`/
> `site_xmat`, where the sim IK actually converged this tick) — NOT the
> `target_l_/target_r_` commanded setpoint. The two diverge under joint limits,
> soft-locks, and collision constraints (the site lags the target). Every
> "no jump / land exactly where the hand is" argument here re-bases onto where the
> arm *is*; re-basing onto the commanded setpoint silently re-anchors at a place
> the arm is not, defeating the §5 fix. The site pose is what the solver already
> computes each tick (`:1917–1934`). Publish that, unambiguously.

Because `ee_goal_` re-bases onto the achieved pose at engage, no relative
baseline is needed. `quest_frame_rot_rpy` / `quest_pos_scale` become optional
tuning params (only if the operator's standing alignment is slightly off), not
mandatory.

> **Operator-alignment assumption:** "torso-relative ≈ map frame" holds only while
> the operator stands with the torso's ground projection near the robot base,
> facing +x. A lateral standing offset adds a constant translation offset to every
> Quest pose — the arm then converges to a pose offset from the hand by that
> lateral distance. The §4 error threshold sees convergence to the *shifted*
> target, so approach still completes; the offset is a constant reach error, never
> parameterized away. Head yaw rotates the mapping. That residual is an accepted
> trade-off
> (user decision 3) — we do **not** add a full head-pose transform. If tuning is
> needed, `quest_frame_rot_rpy` / `quest_pos_scale` apply in the mapper *before*
> the pose becomes the interpolation target: `Q′ = R(quest_frame_rot_rpy) · Q`,
> with translation scaled by `quest_pos_scale` (defaults: identity / 1.0). These
> correct rotation and scale only — a pure translation offset is not parameterized,
> which is exactly what the standing-alignment assumption covers.

---

## 6. Interpolation math (in `control_timer_callback`)

Same 100 Hz tick. When in a QUEST state:

```cpp
// translation: linear blend
ee_goal_.translation() = (1.0 - w) * ee_goal_.translation() + w * target.translation();

// rotation: quaternion slerp
Eigen::Quaterniond q_cur(ee_goal_.linear());
Eigen::Quaterniond q_tgt(target.linear());
Eigen::Quaterniond q_new = q_cur.slerp(w, q_tgt);
ee_goal_.linear() = q_new.toRotationMatrix();
```

Weight (parameter, per-tick at 100 Hz — equivalent to time-constant):
- `quest_w_slow`  default `0.15`  → ~15 % of remaining error per tick (approach).

TRACK is **raw pose**: `ee_goal_ = quest_target_` directly, no blend, no slerp
(§7) — 1:1 with the hand. `quest_w_fast` was removed when TRACK went raw.

The TRACK/APPROACH result is then run through the **existing** EE soft-lock
(`apply_ee_locks()`) and the publish path is unchanged — `apply_ee_locks()` is
called explicitly in the Quest branch (§7), same as the SpaceMouse path. The
solver's `clip_target` leash (1 cm/tick in ffw_ik_solver_teleop) keeps the actual
arm velocity bounded regardless of the target; raw TRACK hits that leash at fast
hand speeds, so the observed top speed in TRACK is 1 m/s (1 cm × 100 Hz), not the
hand speed.

**Lock centers vs the engage re-base:** a soft-lock's center is captured from
`ee_goal_` when the lock engages (joy_hand.cpp:114–132). The engage re-base
overwrites `ee_goal_`'s orientation, so if a roll/yaw lock is already engaged at
engage time, re-capture its locked center from the (new) achieved orientation —
otherwise `apply_ee_locks()` immediately clamps the re-based orientation back to
the old center and fights the re-base.

**Mapper-side leash — skipped:** a leash on `ee_goal_` toward the last achieved
pose is NOT directly available in the mapper (it has no achieved-pose feedback).
The solver's leash already covers this.

---

## 7. Code changes — file by file

### `src/joy_hand.cpp` (main work, ~all new logic)

All new behavior is implemented here in C++ (`quest_to_ros2.py` remains a pure
telemetry bridge — modifiable only in message format, §3 / §10 decision 4).

New parameters:

```cpp
// quest override
declare_parameter("quest_state_topic", "/quest_state");
declare_parameter("quest_achieved_pose_topic_l", "/ik_solver/achieved_ee_pose_l");
declare_parameter("quest_achieved_pose_topic_r", "/ik_solver/achieved_ee_pose_r");
declare_parameter("quest_active_topic_prefix", "/quest");       // publishes Bool on /quest/<arm>/active
declare_parameter("quest_publish_notifications", true);          // only ONE instance may fire /ffw_control/notification
declare_parameter("quest_notification_topic", "/ffw_control/notification"); // publishes String
declare_parameter("quest_hold_engage_s", 2.0);
declare_parameter("quest_release_min", 0.1);         // all 4 floats < this → engage count arms (§4)
declare_parameter("quest_thumb_analog_offset", 7);   // index in arm block: 7 = grip (float 0-1) (§3)
declare_parameter("quest_trigger_analog_offset", 6); // index in arm block: 6 = trigger (§3)
declare_parameter("quest_thumb_engage_min", 0.5);    // thumb >= 0.5 keeps mode alive
declare_parameter("quest_trigger_engage_min", 0.5);
declare_parameter("quest_w_slow", 0.15);
declare_parameter("quest_approach_pos_m", 0.05);
declare_parameter("quest_approach_ang_rad", 0.10);
declare_parameter("quest_timeout_s", 0.5);       // no /quest_state → auto-return
declare_parameter("quest_achieved_max_age_s", 0.5); // engage re-base: achieved pose age limit
declare_parameter("quest_hold_hysteresis", 0.05);   // threshold band for keep-alive/early-release (§4)
declare_parameter("quest_frame_rot_rpy", {0.0, 0.0, 0.0}); // optional alignment tuning
declare_parameter("quest_pos_scale", 1.0);
```

New subscriptions:
- `/quest_state` (Float32MultiArray) → cache per-hand **analog thumb/trigger floats**
  (6/7 left, 22/23 right — arm block offset +16, §3) + the button floats + quest
  poses (map frame), and record `quest_last_msg_time_` = now (the §9 arrival
  watchdog reads it). Block selection by this instance:
  `int block_offset = (target_arm_ == "right") ? 16 : 0;` — the **control pose**
  comes from this instance's block (§3 indices 0-5 / 16-21); the four **engage**
  floats (6/7 left, 22/23 right) are read globally at fixed indices regardless of
  `target_arm_` (§4 — the gesture is 2-handed).
- `/ik_solver/achieved_ee_pose_l` / `_r` (PoseStamped, map frame) → cache the
  robot's current achieved EE pose for the engage-time re-base.

> **Thread model (explicit):** the default single-threaded executor serializes all
> callbacks, so the quest/achieved caches written in callbacks and read in the
> 100 Hz timer need **no new mutex** — same pattern as `precision_mode_` today
> (joy_hand.cpp:59–62, read at :227 without a lock). State this in the code so a
> future multi-threaded executor knows to add the mutex.

New publishers:
- `/quest/<arm>/active` (Bool, `arm` = `target_arm_`) — the "override active"
  flag, **per arm**, exactly parallel to `/logitech/base_active`. Per-arm is
  required because two `joy_hand` instances must not both publish one shared Bool:
  a last-writer-wins race would leave `/quest/active` False when one hand drops
  out a few ms before the other.
- `/ffw_control/notification` (String) — fired once on APPROACH → READY
  ("release trigger to go fast"). Published by **one** instance only (gated on
  `quest_publish_notifications`; the right-hand `joy_hand` instance is launched
  with it `false`) so two arms approaching together do not double-fire. Future
  sound/rumble hooks plug into this.

New members + logic:
- enum `QuestState { SM_CONTROL, QUEST_APPROACH, QUEST_READY, QUEST_TRACK }` with
  member `quest_state_` (initial `SM_CONTROL`); `quest_active_` ⇔
  `quest_state_ != SM_CONTROL`,
- `engage_armed_` (bool, §4) — the count only starts once all four floats are at
  zero (`quest_release_min`), and any interruption disarms it,
- `target_arm_` ("left"/"right", from a param/launch arg) — selects this
  instance's `/quest_state` block and the `/quest/<arm>/active` topic name,
- `quest_last_msg_time_` (rclcpp::Time) — set in the `/quest_state` callback;
  the §9 arrival watchdog reads it,
- `achieved_pose_valid_` (bool) — set when the first achieved pose arrives;
  guards the re-bases and §11 step 3's publish guard,
- hold-engagement timer (`hold_start_`, per-hand thumb/trigger analog latches),
- engage-time **re-base**: at engage (gated on a fresh achieved pose, §4),
  `ee_goal_ = achieved_ee_pose_map` (from the solver topic), then
  `target = quest_pose_map`; if a roll/yaw soft-lock is already engaged,
  **re-capture its locked center from the new achieved orientation** first, else
  `apply_ee_locks()` clamps the re-based orientation back and fights the re-base
  (§6),
- `detect_engage()` (§4) — runs **every ARM tick** while in `SM_CONTROL` (guarded:
  `if (quest_state_ != SM_CONTROL) return;` so it is inert in the QUEST states),
- `step_quest_state_machine()` (§4) — per-tick keep-alive / go-fast / arrival
  watchdog / APPROACH no-progress-guard evaluation in the active states; the
  no-progress guard tracks `quest_last_err_` + `quest_last_progress_time_`
  (§10),
- `step_quest_interpolation()` (§6) — slerp `ee_goal_` toward `target`,
- optional alignment: apply `quest_frame_rot_rpy` / `quest_pos_scale` to the quest
  pose **before** it becomes the interpolation target: `Q′ = R(rot_rpy)·Q`,
  translation scaled by `quest_pos_scale` (§5; defaults identity/1.0 = no-op),
- `quest_thumb_held()` / `quest_trigger_held()` helpers (analog ≥ 0.5, both hands),
- in `control_timer_callback()`: immediately after the `current_mode_ == "BASE"`
  gate (joy_hand.cpp:193) and **before** the `if (!joy_received_) return;` gate
  (joy_hand.cpp:198) — Quest must run even if no SpaceMouse message has ever
  arrived (its input is `/quest_state`, not `/joy`); only then comes the SpaceMouse
  velocity branch,
  ```
  if (current_mode_ == "BASE") return;   // quest path fully ignored in BASE (§9)

  // Quest state machine — runs every ARM tick, active or not (§4)
  if (quest_state_ == SM_CONTROL) {
    detect_engage();                  // all-4-at-zero → 2 s hold → engage (§4)
                                      //   (on engage: re-base ee_goal_ onto achieved)
  }
  if (quest_state_ != SM_CONTROL) {   // override active → suppress SpaceMouse
    step_quest_state_machine();       // §4 per-tick order (keep-alive / go-fast / watchdog)
    step_quest_interpolation();       // slerp ee_goal_ toward quest target (§6)
    apply_ee_locks();                 // EE soft-locks still enforced in quest mode
    publish_pose();
    return;
  }
  ```
  (spacemouse velocity path is bypassed while quest is active — same "suppress"
  concept as `if (logitech_active_) return;` in `joy_base.cpp`; `apply_ee_locks()`
  is called explicitly so the §6 safety claim matches the code),
- **Mode switch = the emergency stop, and it lives in the `mode_sub_` callback,
  not the timer** — on leaving ARM (`msg->data != "ARM"` while currently ARM),
  `mode_sub_` **stops listening to the quest, freezes `ee_goal_` where it is, and
  resets the state machine to its pre-count state** — `quest_state_ = SM_CONTROL`,
  `engage_armed_ = false`, `/quest/<arm>/active = False` (§4 "Abort", §9):
  ```cpp
  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/teleop_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (current_mode_ == "ARM" && msg->data != "ARM") {
          // emergency stop: freeze goal, reset quest to pre-count state (§4/§9)
          quest_state_ = SM_CONTROL;
          engage_armed_ = false;
          quest_active_pub_->publish(false);   // /quest/<arm>/active = False
        } else if (msg->data == "ARM" && current_mode_ != "ARM") {
          if (achieved_pose_valid_)            // never re-base from default pose (§5)
            ee_goal_ = achieved_pose_map;      // BASE→ARM: re-base into world frame (§11 step 2)
        }
        current_mode_ = msg->data;
    });
  ```
  Because the reset runs in the callback, it executes even though the timer
  early-returns in BASE — the panic abort can never be dead code. `joy_hand.cpp`
  has no button/double-click handler (verified); the only quest-time suppression
  in the timer is the SpaceMouse velocity branch above. If a separate gesture
  node triggers the mode switch, its *non-emergency* actions may gate on
  `!quest_active_`, but the mode-switch path itself must remain available as the
  abort channel — thumb release must not be the sole manual stop.

### `ffw_collision_checker/src/ffw_ik_solver_teleop.cpp` — small addition (new)

Publish the robot's achieved EE pose per arm in map frame so the mapper can re-base
at engage. The solver already computes `target_l_`/`target_r_` and the MuJoCo site
pose every tick — expose them:

- New publishers: `/ik_solver/achieved_ee_pose_l` and `/ik_solver/achieved_ee_pose_r`
  (`geometry_msgs::PoseStamped`, frame_id `"map"`), updated each control-loop tick
  from the **MuJoCo achieved site pose** (`d->site_xpos`/`site_xmat` after
  `mj_forward`) — NOT from `target_l_`/`target_r_` (the commanded setpoint, which
  can lag the site under limits/soft-locks; see §5 note).
- Purely additive telemetry here (the achieved-pose publisher). The one behavioral
  solver change is Task 1 step 2 (`last_mapper_*` reset in `apply_hardware_sync()`),
  explicitly carved out under contract C7 — see §11 and the acceptance test.

### `src/joy_base.cpp` — minimal / no change
Quest override is ARM-only; base teleop is BASE-only. No conflict.

### `scripts/quest_to_ros2.py` — format change only (user decision 2026-08-01)
The button bitmasks become individual float values in `/quest_state` (§3 / §10
decision 4). Touches are dropped entirely — nothing reads them. Still **no
control logic** — it remains a pure telemetry bridge, only its message format is
owned by this feature.

### `launch/spacemouse_unified_teleop.launch.py`
- Add `use_quest` (`false` default) + `quest_device` args, mirroring `use_logitech`.
- When enabled, include the quest pipeline:
  `ExecuteProcess python3 quest_to_ros2.py --tui` (or a new
  `quest_teleop.launch.py` that runs `quest_to_ros2.py` + optional rviz).
- Pass the new `joy_hand` params through `spacemouse_mapper.launch.py` (or set them
  directly in the unified launch's mapper include args). Set
  `quest_publish_notifications=false` on the **right-hand** mapper instance (only
  the left one fires `/ffw_control/notification`; both still publish their own
  `/quest/<arm>/active`).

### `launch/spacemouse_mapper.launch.py`
- Accept and forward the new quest params to the `joy_hand` node.

### `CMakeLists.txt`
- Already installs `scripts/quest_to_ros2.py`. No change needed beyond that.

---

## 8. Behavior walkthrough (what the operator does)

1. Robot in **BASE** mode with both SpaceMice. User double-clicks the SpaceMouse
   buttons → mode → **ARM** (existing path — the double-click lives in a gesture
   node that publishes `/teleop_mode`, not in `joy_hand.cpp`).
2. SpaceMouse drives both EE targets as today (velocity integration).
3. User **releases all four floats to zero** (both thumbs + both triggers below
   `quest_release_min`, 0.1) — this **arms** the engage count (§4) — then grips
   both Quest controllers and **holds THUMB ≥ 0.5 + TRIGGER ≥ 0.5 on both hands
   for 2 s** → each `joy_hand` records its baseline, enters `QUEST_APPROACH`,
   publishes `/quest/<arm>/active = True`. SpaceMouse velocity is suppressed for
   that arm.
4. Arm slowly interpolates from the SpaceMouse-held pose toward the mapped Quest
   pose (`w_slow`), solver leash keeps it smooth/safe.
5. When error < threshold → **print + publish `/ffw_control/notification`**
   ("release trigger to go fast") → `QUEST_READY`. Future: sound / SpaceMouse
   rumble on this event.
6. User **releases TRIGGER (keeps THUMB ≥ 0.5)** → `QUEST_TRACK`: the EE follows
   the hand 1:1 (`w_fast`).
7. If the user released TRIGGER **too early** (still in `QUEST_APPROACH`) → back
   to `SM_CONTROL`; fast mode is locked until a fresh 4-button, 2 s hold re-arms it.
8. User releases **either THUMB** (below 0.5) at any point → override ends →
   `SM_CONTROL`, `/quest/<arm>/active = False`. On exit the mapper **freezes
   `ee_goal_` where it is and resets the count to disarmed** (§4 "Abort") — the
   arm holds (the ball clamp pins the solver target to the achieved pose, so it
   stops within the leash ball), and a fresh engage must start again from all four
   floats at zero.

---

## 9. Safety / edge cases

- **Release thumb while still in APPROACH** → the §4 abort freeze applies:
  `ee_goal_` stays where it is, count disarmed, arm holds (ball clamp, §2).
- **Quest packet loss / timeout** → add a watchdog: if no `/quest_state` for
  `quest_timeout_s` (default 0.5 s) while in a QUEST state, auto-return to
  `SM_CONTROL` — via the §4 abort freeze (goal stays put, count disarmed). (Marks
  a design gap in the Logitech pattern — Logitech is wired, Quest is wireless, so
  this watchdog is mandatory.)
- **Frozen-stream blind spot (accepted)** → the arrival watchdog above only
  detects a *dead* bridge; a bridge re-publishing a cached `/quest_state` defeats
  it. A TRACK data-freshness guard was considered (cycle-3 #10) and **removed**:
  the un-stamped `Float32MultiArray` cannot distinguish a genuinely-still hand
  from a frozen stream, so it would abort on legitimate stillness, and it was
  disabled by default anyway. Accepted — a frozen-alive stream just reads as a
  still hand (arm holds; `quest_no_progress_s` covers a frozen stream in APPROACH).
- **Mode switch during quest active = the emergency stop** → if `/teleop_mode`
  flips away from ARM while quest override is on, the `mode_sub_` callback
  **stops listening to the quest, freezes `ee_goal_` where it is, and resets the
  state machine to its pre-count state** (`SM_CONTROL`, count disarmed,
  `/quest/<arm>/active = False`). This runs in the callback, not the timer, so it
  executes even though the timer early-returns in BASE (§7).
- **In BASE, the quest path is completely ignored** — the BASE gate returns before
  any quest code runs (§7), and the mode-switch reset guarantees no quest state
  carries across an ARM → BASE → ARM cycle.
- **Both arms together** → each `joy_hand` instance is independent (left arm ←
  left hand, right arm ← right hand). The 2-hand gesture is **global**: both
  hands' floats come from the shared `/quest_state`, so both instances evaluate
  the same gesture on the same message and engage/disengage **simultaneously by
  construction** — inter-process receipt jitter is inherent scheduling, no
  tolerance param (removed 2026-08-01).
- **Gripper/thumbstick conflicts** → keep the analog mapping params
  (`quest_thumb_analog_offset`, `quest_trigger_analog_offset` — 7 = grip /
  6 = trigger, §7) parameterized so no future gripper mapping collides with the
  thumb/trigger analogs (the engage gesture uses the analogs, not the button
  floats).

---

## 10. Decisions & open questions

### Resolved by user (2026-07-31 → 2026-08-01)

1. **"Thumb" is an analog float** (1.0 = fully engaged). Keep-alive condition is
   **thumb ≥ 0.5** on both hands (parameter `quest_thumb_engage_min`). The exact
   analog source is parameterized (`quest_thumb_analog_offset`, default 7 = grip)
   — confirmed as the grip analog; see decision 7.
2. **Trigger release → fast** is confirmed as the go-fast gesture. A new
   `QUEST_READY` state waits between "approached" and "trigger released".
3. **Frame mapping** — user's point 3 verified: the **Quest torso-relative frame ≈
   robot map frame** (x/y/yaw from torso, z from ground), which is the same
   universe as the saved CLI poses. The SpaceMouse teleop does **NOT** share that
   frame (it's a fake delta frame re-based by the solver). Resolution: re-base
   `ee_goal_` onto the solver's achieved pose at engage, then interpolate toward
   the (map-frame) Quest pose → absolute positioning. See §5.
4. **`quest_to_ros2.py` is now modifiable** (2026-08-01): the button bitmasks
   become individual float values in `/quest_state`; **touches are dropped
   entirely** (nothing reads them) (§3). It still does **no control logic** — it
   remains a telemetry bridge; the C++ behavior lives in `joy_hand.cpp` (+ a tiny
   additive publisher in the solver).
5. **Engage arms only from all-four-zero** (2026-08-01): the 2 s countdown only
   starts when both thumbs AND both triggers are below `quest_release_min` (0.1),
   then requires an uninterrupted THUMB ≥ 0.5 + TRIGGER ≥ 0.5 on both hands for
   2.0 s (§4). Any interruption disarms the count.
6. **Emergency stop = freeze + reset** (2026-08-01): leaving ARM stops listening
   to the quest, freezes `ee_goal_` where it is, and resets the state machine to
   its pre-count state — no re-anchor, no snap (§4 "Abort", §7, §9).
7. **"Thumb" = the analog grip** (2026-08-01, hardware-verified): `--debug` on
   `receiver_quest.py` showed grip and trigger are real, independent analog
   signals (both reach 1.00 with working button bits), while the thumbrest and
   thumbstick **touch bits never fired once** — the HTS stream does not populate
   the touches mask. So the keep-alive source is **final: analog grip**
   (`quest_thumb_analog_offset` 7), and **touches are removed from `/quest_state`
   entirely** (no remaining use once the thumbrest fallback was dead). Resolves
   #11 (§13).

### Decided during review (2026-08-01)

- **Error metric for APPROACH→READY**: combined pos + rot, measured on the
  **achieved site pose** (from `/ik_solver/achieved_ee_pose_*`, §5) vs the live
  quest target, on **unlocked axes only**:
  `err_pos = |achieved.translation() − target.translation()|` and
  `err_rot = angle(achieved.linear(), target.linear())`, with **roll excluded**
  when `soft_lock_ee_roll_` is engaged and **yaw excluded** when `soft_lock_ee_yaw_`
  is. READY when `err_pos < quest_approach_pos_m` (0.05 m) **AND**
  `err_rot < quest_approach_ang_rad` (0.10 rad).
  - Locked axes MUST be excluded: `apply_ee_locks()` clamps them to center ± slack
    (joy_hand.cpp:158–180), so their error is irreducible — including them would
    strand the operator in APPROACH forever.
  - A target beyond the 90° `clamp_target_angle` cap (or a joint limit) is
    physically unreachable → READY never fires → the no-progress guard (below)
    returns control instead of stranding the operator.
  - Pos-only was considered and rejected: approach is 6-DOF, and the locked-roll /
    locked-yaw exclusions already make the combined metric well-posed.
- **No-progress stuck guard**: `quest_no_progress_s` (default 5.0 s) — if the
  APPROACH error metric shows no progress toward threshold for that long (target
  unreachable, hand held still mid-approach, stream frozen), fire the notification
  with a `"quest_no_progress"` payload and return to `SM_CONTROL`. Without it, an
  unreachable target (90° cap / joint limit / soft-lock) strands the operator in
  APPROACH with the READY threshold never met.
- **Achieved-pose topic shape**: two `PoseStamped` topics
  (`/ik_solver/achieved_ee_pose_l` / `_r`) — already assumed throughout §5/§7/§11.

### Still to confirm before coding

- **Notification payload string** for `/ffw_control/notification` (e.g.
  `"quest_approach_reached"`) and whether it should also fire on TRACK entry.

---

## 11. Task 1 (prerequisite) — make the SpaceMouse teleop publish in the robot world frame

User decision 2026-07-31: before the Quest work, fix the mapper so its published
`ee_target_pose` is a **real world/map-frame pose** (same frame as MuJoCo and the
saved CLI poses), instead of the current fake delta frame starting at `(0,0,0)`.

**Design principle (user-confirmed): the mapper should know the world result and
clamp.** World-pose knowledge and clamping should live in the mapper, not be
hidden inside the solver's delta re-base. The solver already clamps (`clip_target`
leash + `clamp_target_angle`); with the mapper in world frame it can clamp too, and
the two become composable instead of the mapper being a blind delta source.

Benefits:
- The mapper's absolute poses become meaningful — debuggable in rviz, comparable
  to `/ik_solver/achieved_ee_pose_*` and saved poses.
- The mapper can clamp against the achieved pose (mapper-side leash / workspace
  limits), not just the solver.
- The Quest feature becomes trivial on top: `ee_goal_` is *already* in map frame,
  so `target = quest_pose` directly (no extra engage re-base needed — though we
  keep the achieved-pose re-base at engage as a safety re-sync).

### Why the solver must change too (the one subtlety)

Today the solver re-bases the mapper's fake frame on the FIRST message and
integrates only deltas:

```cpp
if (first_msg_l_) { last_mapper_l_trans_ = trans; first_msg_l_ = false; }
delta_trans = trans - last_mapper_l_trans_;   // deltas only
```

If the mapper suddenly re-bases `ee_goal_` to the world pose on ARM switch but the
solver's `last_mapper_*` keeps the old fake-frame value, the first delta after the
switch is huge → phantom jump. So the solver must **reset its mapper baseline** to
the same world pose at the same moment. (`apply_hardware_sync()` currently resets
`initial/target/accum` but NOT `last_mapper_*` — verified in code.)

### How a CLI move (load pose / home reset) is handled — today vs after Task 1

**Today (mapper is blind, solver re-bases):** the CLI moves the arm without the
mapper knowing. It copes automatically:

1. CLI calls `/ik_solver/load_pose` (or `reset_to_home`) → solver sets
   `solving_to_home_ = true`, `initial_* = loaded/home pose`, `target_* = same`,
   `accum_* = 0`, **`first_msg_* = true`** (`apply_pose_load`, `apply_home_reset`).
2. While `solving_to_home_`, `pose_callback_l/r` return early → **mapper deltas are
   ignored** during the move.
3. Arm reaches the loaded pose. User touches the SpaceMouse → mapper publishes its
   next pose; the solver sees `first_msg_* = true` → **re-bases** `last_mapper_*`
   on it, so no phantom delta.
4. Subsequent deltas accumulate on top of the loaded pose → the arm moves
   **relative to the loaded pose**. No jump.

So the delta design *works* — but only because the solver silently re-bases. The
mapper stays **unaware and its absolute output is now a lie** (it still claims the
old fake pose while the arm is at the loaded pose). That's fine for SpaceMouse
deltas, but **breaks the Quest's absolute targeting** — which is exactly why Task 1
exists.

**After Task 1 (mapper knows world pose):** the mapper detects the CLI move via the
achieved-pose topic — `|achieved − ee_goal_|` becomes large. The drift-based
re-base (step 3 below) snaps `ee_goal_ = achieved`, so the mapper stays truthful
and the Quest target stays correct.

> **Side note (user-confirmed):** in the current code the SpaceMouse mapper is
> **completely blind** — it never knows nor cares about the world target. The
> world-frame target you see on the MuJoCo viewer is **exclusive to the solver's
> MuJoCo simulation**; the mapper is just a dumb delta source that the solver
> re-bases onto whatever MuJoCo believes. The whole Task-1 premise is fixing this
> blindness.

### Changes

**`ffw_ik_solver_teleop.cpp`**
1. Add publishers `/ik_solver/achieved_ee_pose_l` / `_r` (`PoseStamped`, frame
   `"map"`), published each control tick from the **MuJoCo achieved site pose**
   (`d->site_xpos`/`site_xmat` after `mj_forward`) — NOT `target_l_`/`target_r_`
   (the commanded setpoint can lag the site under limits/soft-locks; see §5).
   — This is the same additive topic the Quest feature needs.
2. In `apply_hardware_sync()` (and/or the `mode_sub_` BASE→ARM branch): after
   `target_l_ = initial_l_`, also set
   ```cpp
   last_mapper_l_trans_ = target_l_.translation();
   last_mapper_l_rot_   = target_l_.linear();
   // same for r
   ```
   so the mapper's world-frame re-base does not inject a phantom delta.
   (Residual risk: a mapper pose arriving between the mode message and the sync
   tick; bounded by the 1 cm/0.1 rad leash — acceptable.)

**`src/joy_hand.cpp`**
1. Subscribe to `/ik_solver/achieved_ee_pose_l` / `_r`, cache — no new mutex, same
   single-threaded-executor argument as §7 (the timer and the callbacks share one
   thread, exactly like `precision_mode_` today).
2. On startup (first achieved pose received while ARM) and on every
   `BASE → ARM` mode switch:
   ```cpp
   ee_goal_ = achieved_pose_map;   // re-base the mapper into the world frame
   ```
   Execution locations: the first-achieved re-base runs in the **achieved-pose
   subscription callback** (when `current_mode_ == "ARM"` and the cache is
   valid); the BASE→ARM re-base is a branch of the `mode_sub_` callback (§7),
   gated on `achieved_pose_valid_` — never re-base from a default pose (§5).
   (No new mutex — `mode_sub_` and the timer share the single executor thread,
   §7.)
3. Do not publish until the first achieved pose is received: skip `publish_pose()`
   until `achieved_pose_map` has data (the solver's first-message re-base already
   masks the Identity pose, but this keeps the mapper's output truthful).
4. Optional robustness: while in ARM, if the **translation distance**
   `|achieved.translation() − ee_goal_.translation()|` exceeds a drift threshold
   (e.g. 10 cm — the arm was homed / pose-loaded / manually moved), re-base
   `ee_goal_ = achieved_pose` to stay truthful. Orientation is excluded from the
   drift metric (a pure rotation gap does not indicate an external move worth
   re-basing). Three guardrails:
   - **Never while Quest is active** (`!quest_active_`): in a QUEST state the gap
     legitimately grows to the tracking lead, and re-base would snap the reference
     mid-approach and fight the §4 state machine.
   - **Re-base only while the user is NOT commanding** (both SpaceMouse axes ~0):
     a gap also grows from the operator pushing into a limit/singularity at leash
     speed, and re-basing then erases their progress. A deadband on gap size CANNOT
     distinguish "user outpaces arm" from "arm moved externally" — gate on joystick
     idle, not on gap size. (At max SpaceMouse command velocity the arm keeps up —
     mapper step 0.75 cm/tick vs the 1 cm leash — so an idle-time gap > 10 cm
     implies an external move or a limit-push, both safe to re-base on.)
   - **This path injects a delta into the solver** (`achieved − last_mapper_*` can
     be the full gap), so it is safe ONLY because the ball clamp projects the
     solver target inside the 1 cm / 0.1 rad ball around `achieved` the same tick
     (delta-then-clamp ordering + `accum` re-sync, §2): the solver target lands
     inside the leash ball of `achieved`, no arm jump. If that ordering ever
     changes, route the drift re-base through a solver-aware reset of
     `last_mapper_*` (same as the hardware-sync path, step 2).
5. SpaceMouse delta integration is otherwise unchanged — it now accumulates in the
   world frame.

**No change to the solver's control math** — it already operates in the world
frame; only the mapper's absolute output changes (fake → real). The solver's
`clamp_target_angle` / `clip_target` are frame-agnostic on the solver side.

### Acceptance test
1. Start the unified launch (`hardware_mode:=false` for sim).
2. Switch to ARM, move the SpaceMouse.
3. Echo `/spacemouse/left/ee_target_pose` — values should now be sane world
   coordinates (e.g. near the Home pose `(0.0055, ±0.2275, 0.513)` region), not
   tiny deltas near `(0,0,0)`.
4. Verify no jump at mode switch, and that `/ik_solver/achieved_ee_pose_l` tracks
   the arm's real pose.

---

## 12. Task 2 — Quest override (the main feature, §4–§10)

After Task 1 lands, the Quest work in `joy_hand.cpp` simplifies:
- `ee_goal_` is already in map frame → `target = quest_pose_map` at engage.
- The achieved-pose topic already exists (Task 1) → still re-base `ee_goal_ =
  achieved` at engage as a safety re-sync, then interpolate with `w_slow`/`w_fast`.
- Everything else (§4 state machine, §6 interpolation, §7 params) is unchanged.

---

## 13. Review status (doubt-driven development) — 2026-08-01

Doubt cycles run on this plan: **3** (the skill's bound). Cross-model offer was
made every cycle: **cycle 1** = manual external review (10 findings; H1–H6 edits
applied), **cycle 2** = user chose skip (18 findings reconciled, edits applied),
**cycle 3** = user chose skip (16 findings; RECONCILE was **interrupted** at the
user's request, then **superseded by the user's 2026-08-01 redesign** below — the
redesign's decisions are applied in §3/§4/§7/§9/§10).

### Cycle-3 findings — classification snapshot (RECONCILE superseded by the redesign below)

| # | Sev | Finding | Class |
|---|---|---|---|
| 1 | HIGH | Engage edge has no execution location (§7 sketch only runs when `quest_active_` already true) | valid+actionable, unapplied |
| 2 | HIGH | §9 mode-switch abort can't execute — quest code sits after the BASE gate | valid+actionable, unapplied |
| 3 | HIGH/MED | Re-anchor proven against sim site pose, not measured joints | valid trade-off (needs scope note), unapplied |
| 4 | MED | Abort re-anchor doesn't re-capture soft-lock centers (§6 only covers engage) | noise/low note |
| 5 | MED | Trigger-release predicate never says either vs both hands | valid+actionable, unapplied |
| 6 | MED | Per-arm engage freshness gate breaks §9 "engage/disengage together" | valid trade-off (low) |
| 7 | MED | Contradiction: §7 "no new mutex" vs §11 "cache under mutex" | valid+actionable, unapplied |
| 8 | MED | §4 "every exit re-anchors" vs §10 no-progress exit (no re-anchor stated) | valid+actionable, unapplied |
| 9 | MED/LOW | Notification double-fire is the shipped default (param true + vague launch bullet) | valid+actionable, unapplied |
| 10 | MED/LOW | TRACK frozen-stream guard aborts on legitimate stillness | noise (default 0.0 = disabled) → removed 2026-08-01 |
| 11 | LOW/MED | "thumb = grip" keep-alive is a spurious-abort source during hand motion | valid trade-off (already "to confirm" in §10) |
| 12 | LOW | Mapper may publish fake Identity pose before first achieved pose | noise (solver re-bases first msg) + cheap guard |
| 13 | LOW | §5 "lateral offset partially absorbed by error threshold" is mechanically wrong | valid+actionable (wording), unapplied |
| 14 | LOW | Hysteresis keeps mode alive at 0.45, below C3's literal 0.5 | contract-misread (contract lacked hysteresis) |
| 15 | LOW | `quest_frame_rot_rpy`/`quest_pos_scale` defined in §5 but their use absent from §7 enumeration | valid+actionable (add §7 bullet), unapplied |
| 16 | LOW | Solver `last_mapper_*` reset location hedged "and/or" | **resolved** (see below) |

### Resolved this session

- **Finding 16 (solver `last_mapper_*` reset location).** Verified in code:
  `mode_sub_` sets `hardware_sync_requested_ = true` on every BASE→ARM switch
  (ffw_ik_solver_teleop.cpp:542–543), and the main loop calls `apply_hardware_sync()`
  every tick, early-returning unless requested (:823–826, :1908). So resetting
  `last_mapper_*` inside `apply_hardware_sync()` covers **both** the explicit
  hardware-sync command and the plain BASE→ARM switch — the plan's "and/or" resolves
  to "inside `apply_hardware_sync()` is sufficient". Residual timing gap (a mapper
  pose arriving between the mode message and the sync tick) remains bounded by the
  1 cm / 0.1 rad ball clamp, as the plan already notes.

### Redesign (user, 2026-08-01) — applied; supersedes the cycle-3 RECONCILE

The user directed a **total overhaul** of the broken parts instead of further
iteration. Four decisions, all applied to the plan:

1. **Quest logic runs only when the unified launch has already passed control to
   the manipulator (ARM) mode.** In BASE mode the quest path is ignored
   completely — the BASE gate returns before any quest code runs (§7, §9).
2. **Emergency stop = stop listening to the quest, freeze the goal where it is,
   and push the state back to its pre-count state.** Lives in the `mode_sub_`
   callback, not the timer, so it executes even though the timer early-returns in
   BASE (§4 "Abort", §7, §9).
3. **The engage countdown only starts counting when all 4 buttons (2 hands ×
   thumb+trigger) are fully at zero** — a clean baseline. New `quest_release_min`
   (0.1) arms the count; any interruption disarms it (§4, §7, §8 step 3).
4. **`quest_to_ros2.py` is modifiable** — the button bitmasks become individual
   float values in `/quest_state` (38-float layout, §3); touches are dropped
   entirely. It stays a telemetry bridge with no control logic (§7, §10 decision 4).

Effect on the cycle-3 findings:

| # | Cycle-3 class | After redesign |
|---|---|---|
| 1 | HIGH — engage edge has no execution location | **fixed** — state machine (incl. `detect_engage()`) runs every ARM tick (§7) |
| 2 | HIGH — mode-switch abort is dead code | **fixed** — reset lives in `mode_sub_`, runs even when the timer early-returns in BASE (§7/§9) |
| 3 | HIGH/MED — re-anchor proven vs sim, not joints | **moot** — abort freeze replaces the sim re-anchor; no exit depends on measuring the real arm |
| 4 | MED — abort doesn't re-capture lock centers | **moot** for the abort path (uniform freeze, §4); the engage-side requirement is placed in §7's engage re-base bullet (§6) |
| 5 | MED — trigger-release either/both unspecified | **fixed** — "either trigger" everywhere (§3/§4) |
| 6 | MED — per-arm freshness gate breaks together-engage | still pending (trade-off note only) |
| 7 | MED — §7 "no new mutex" vs §11 "cache under mutex" | **fixed** — §11 wording now matches §7 (single-threaded executor) |
| 8 | MED — every-exit-re-anchor contradiction | **moot** — uniform abort freeze covers ALL exits (§4) |
| 9 | MED/LOW — notification double-fire default | **fixed** — right-hand instance sets `quest_publish_notifications=false` (§7 launch) |
| 10 | MED/LOW — TRACK frozen-stream aborts on stillness | **removed** — dead param (default 0.0); frozen-alive stream reads as a still hand, arm holds (§9) |
| 11 | LOW/MED — "thumb = grip" spurious-abort source | **resolved (hardware)** — grip is a clean independent analog signal; touches dropped from the layout, so grip stays the keep-alive source (§10 decision 7) |
| 12 | LOW — Identity pose before first achieved pose | **fixed** — §11 step 3 publish guard |
| 13 | LOW — §5 "partially absorbed" wording | **fixed** — §5 rewritten to "constant reach error" |
| 14 | LOW — hysteresis below literal 0.5 | contract-misread; unchanged |
| 15 | LOW — `quest_frame_rot_rpy`/`quest_pos_scale` not in §7 | **fixed** — §7 members alignment bullet |
| 16 | LOW — solver `last_mapper_*` reset location | **resolved** — unchanged (apply_hardware_sync covers both) |

Still open (no behavior): #6 (trade-off note only), #14 (contract-misread).
