# Plan: two callable IKSolver functions — dual-arm EE-velocity→dq map & collision-bounded qpos rail override

Status: **planning complete — awaiting approval to implement.** This document is the executable spec
for the implementation; it also records the full design conversation (grill rounds, decisions, and
premise corrections) so a reader — or a future session after context loss — can reconstruct *why*
every choice was made.

---

## Context

`ffw_ik::IKSolver` (package `ffw_collision_checker`, lib `ffw_ik_solver_lib`) is the dual-arm MuJoCo
IK solver **and** the effective kinematics library (all FK is inline MuJoCo — there is **no**
`ff_kinematic`/`ffw_kinematics` package in the live tree). Consumers: `TeleopNode`
(`ffw_ik_solver_teleop.cpp`), `RadioControlMapper` (`ffw_radio`), and the solver test executables.

We add two callable functions for an external-controller / qpos-rail workflow:

1. **f1 — convert EE velocity into joint velocity** (`dq ≈ J⁺·v`). A pure, stateless *diagnostic*
   mapping: which motors would move, and by how much, for a commanded dual-arm motion.
2. **f2 — override the MuJoCo state with a commanded qpos list, collision-bounded.** Write the
   commanded qpos, adopt the FK-of-qpos pose as the goal (ignoring any EE goal), and — if the
   written state violates the collision margin — run a bounded corrective solve that **returns the
   nearest collision-safe pose and stops the moment clearance is restored**, instead of driving to
   equilibrium.

### Confirmed semantics (three grill rounds, all user answers)

- f1 = **dual-arm 12D stacked**: one call takes BOTH EE twists as a single 12D world-frame vector
  `[v_l; ω_l; v_r; ω_r]`, builds ONE stacked 12×nv Jacobian (the same task-row shape solveStep
  builds for the two EE sites), solves once for the full-nv `dq`. Output `dq` only, contribution
  judged by `|dq_i|`. A single-arm query = pass zeros for the other arm's six rows.
- f2 min distance = **body collision clearance** (≥ margin is safe; MuJoCo reports positive signed
  gaps up to geom margin — see Verified facts).
- qpos source = **continuous external rail**: an external controller streams a desired qpos every
  tick; the FK-of-qpos goal replaces the EE goal each tick.
- f2's normal path = **snap + FK-adopt** (reuse `apply_hardware_sync` re-anchor); the snap updates
  the goal, then re-clamps and re-solves for the adapted goal.
- **Branched solve**: one solve function for the continuous spacemouse/quest goal (unchanged),
  one for goal-from-correction. The correction solve has **minimum solve time and exits as soon as
  its critical check (all-contact min_dist ≥ margin) is true** — never runs to equilibrium.
- **Scope (user decision)**: solver + TeleopNode mode-B wiring — a new ROS qpos subscription and a
  main-loop branch that snaps→FK-adopts→(re-clamps)→clears while the rail is active, suppressing
  the spacemouse/quest goal chase. ZMQ ControlCmd qpos extension is **OUT of scope**.

---

## Conversation summary

The design was produced through three structured "grill me" rounds plus a premise correction and a
design-agent reconciliation pass. Everything below that is still load-bearing appears in the body of
this plan; this section is the narrative record.

**Controlling request (verbatim intent):** *"we need 2 callable function in iksolver, one is convert
ee velocity into joint velocity (inverse jacobian matrix?) one is override the mujoco state as a
list of qpos, update the goal using ff_kinematic (i think that we should already have this, but it
did not check for the for minimum distance right, update it so it return to the closet pose that
match this, ignore the ee goal if any) grill me for anything unclear."*

**Round A — component + f1/f2 semantics + branching:**
- Component: **C++ IKSolver library** (not a new node, not the Python CLI).
- f1 semantics: *"pure mapping at first what i want is to know which motor movement would contribute
  most to that movement"* — a diagnostic map, `dq` only, judged by `|dq_i|` ranking.
- f2 semantics: *"yeah kind of snap + fk adapt, reuse anything. but if something that violate
  mindistance then solve for the targeted ee pose (from fk) stop right when it out of min dist
  instead of move to equilibrium like normal"* — snap to the commanded qpos, adopt FK-of-qpos as the
  goal, and when it violates the minimum distance, run a bounded correction that stops at the
  clearance boundary rather than running to equilibrium.
- Branched solve: *"the snap function also update goal right ? make it trigger reclamp and solve
  again for the adapted goal (we really should have a branched solve function, one get called when
  we using coninuous spacemouse goal, and one is goal from correction. goal correction should have
  minimum solve time, just exit as soon as the critical check got true)"* — two disjoint callables,
  mode A (continuous goal) untouched, mode B (correction) exits on its critical check.

**Round B — f2's min-distance meaning, qpos source, f1 output:**
- f2 min distance = **Collision clearance** (≥ margin is safe, not "some target margin to reach").
- qpos source = **Continuous external rail** (an external controller streams qpos every tick; this
  is why the goal must re-adopt every tick, and why freshness/staleness arbitration exists).
- f1 output = **dq only** (rank by `|dq|`).

**Round C — f1 shape + scope:**
- f1 = **Dual-arm 12D stacked**: BOTH EE twists in one call, ONE stacked 12×nv Jacobian, one dq
  solve (this is what makes `lift_joint` participation and the residual test meaningful — see
  Verification item 3).
- Scope = **Also wire the TeleopNode branch** (not solver-library-only).

**Premise correction (accepted):** the user believed a goal-update-by-`ff_kinematic` already existed
in the tree. It does **not** — no `ff_kinematic`/`ffw_kinematics` package exists; FK is inline MuJoCo
(`mj_forward` + site read), exactly as `TeleopNode::apply_hardware_sync` does it. f2 therefore
reuses that inline pattern; no external kinematics dependency is introduced.

**Design-agent reconciliation (all applied to the final design below):**
1. The agent's first `buildClearQP` draft used the raw uncapped `lb = -cbf_alpha·(dist − margin)` for
   *every* active contact. A near-safe band contact (`margin ≤ dist ≤ margin + band`) would get a
   **negative** bound (allowed to approach), so the push-out could trade one violation for another.
   Fixed with `lb_ext(nv_+k) = max(-cbf_alpha·(dist − margin), 0.0)`: violators get a **positive**
   bound (forced to separate), band contacts get **lb = 0** (a wall). This is exactly the difference
   from solveStep's `min(repulsion, 0.0)` clamp at line 613, which zeroes even a violator's bound —
   that clamp is what makes solveStep a pure "wall" and f2 a genuine "clear".
2. `max_steps` 60 → **300**: the violation deficit decays geometrically (per-step factor
   ≈ `1 − cbf_alpha·h`, h capped at 0.1), so a deep 0.1 m violation needs up to ~200 steps; 60 was
   too tight and could stall a legitimate deep command.
3. Internal h-cap 0.05 → **0.1** in `clearToMargin`'s loop (the §D rail `clear_cfg` still passes
   `step_size = min(cfg.step_size, 0.05)` so the *returned pose lands close to the boundary*).
4. §D arbitration restructured: **rail mode owns the tick** while active (EE callbacks early-return,
   head-sync/soft-locks/home-load/hardware-sync gated `if (!rail_gs)`, `solveStep` runs only
   `!rail_gs`), with explicit EE→rail / rail→EE transition handling and a
   `/teleop_goal_source` topic as the natural flip channel for a continuous external controller
   (the param stays for launch-time defaulting and manual `ros2 param set`).
5. Verification item 3 was **wrong** in the first draft: for a pure left-arm x-translation,
   `lift_joint` is ancestral to **BOTH** EE sites, so the right site *legitimately* moves and the
   right-site rows do NOT stay ~0. The correct assertion is driven-row residual small + |dq|-energy
   concentrated in the left arm and `lift_joint`, `head` < 1e-9, qpos unchanged, zero-twist → dq≈0.

**Final consistency check (before this document was written):** a grep over the plan for stale
values (`60`, `250 ms`, `rail_applied`, `max_clear`, `no min(·,0)`, stray `0.05`) returned exactly
one match — the intended `step_size = min(cfg.step_size, 0.05)` in the §D rail `clear_cfg`. The plan
is internally consistent.

---

## Verified facts (grounding the design)

- **CBF construction is a pure "wall"** — `solveStep` (~589-616): CBF rows active only for pairs with
  `ci.dist ≤ collision_margin` (599), `repulsion = -cbf_alpha·(dist − margin)` (609), and
  `lb_ext_(nv_+n_within) = std::min(repulsion, 0.0)` (613, comment: "solid wall, never force AWAY").
  Active rows therefore always have `lb = 0` → `Jdist·dq + slack ≥ 0` blocks *further approach* but
  never clears an existing violation. **f2's correction needs the UNCAPPED `lb = repulsion`**
  (positive when `dist < margin` → forces separation at `cbf_alpha·(margin − dist)`).
- **`d->contact` is a genuine pre-touch clearance source.** Model default `<geom margin="0.30"
  gap="0.0"/>` (`3rd_party/robotis_ffw/ffw_bg2_large_margin_no_gripper_joint.xml`) emits contacts
  with positive signed gaps up to 0.30 m — so contact distance is real-meter clearance comparable to
  the 0.10/0.155 QP margins. **smtm caveat**: verify its geom margin > margin before using rail with
  `robot_model:=smtm`.
- **Top-k blind spot (load-bearing for f2).** solveStep's CBF rows and `StepResult.min_dist` only
  see the `cfg.topk_contacts` (=5) closest contacts. f2's critical check and stop condition must use
  **ALL `d->ncon` contacts**: `computeContacts(d, max(1, d->ncon), out)` — the function partial-sorts
  ascending, so `out.closest.front().dist` is the global minimum. A 6th-closest violating pair must
  not be invisible. (MuJoCo caps `d->ncon` at `nconmax` by evicting the *farthest* pairs first, so
  the global-minimum pair always survives the cap — the all-contact scan is exact.)
- **`computeContacts`** (155-205) already fills per-contact `Jdist_row` (world
  `normalᵀ·(J_body2 − J_body1)` via `mj_jac`) in `ContactInfo`; `ContactResult{total_contacts,
  closest}`; `StepResult.min_dist` = closest gap, `0.30` sentinel when empty.
- **No SVD / pinv / `J.inverse()` anywhere** in the solver today. f1's solve is net-new; DLS via the
  symmetric normal equations (LDLT) needs no SVD and is deterministic (see Design §B).
- **solveStep is monolithic** (211-687) with the QP inline and four private helpers
  `buildJacobian/buildCollisionGradient/buildJointBounds/solveQP` declared (header ~234-239) but
  never defined. **We do NOT refactor solveStep or touch it**; new code is additive and self-contained.
- **Goal state is caller-owned.** `IKSolver` takes targets as args and mutates `d->qpos` in place
  (`mj_integratePos` at 685). Leash/accum/initial/target/last_mapper state lives in `TeleopNode`
  (~1803-1866). The node owns snap/adopt/re-anchor; the solver library owns the pure
  correction primitive.
- **Re-anchor template** = `TeleopNode::apply_hardware_sync` (964-1029): write qpos per joint name →
  `mj_forward` → read `left_gripper_site`/`right_gripper_site` → `target_*_ = initial_*_ = FK` → zero
  `accum_*_` → park `last_mapper_*_` on the pose. Name→qposadr resolution via `mj_name2id` +
  `mj_jnt_qposadr` — never hardcode (bg2: nq=nv=17, `lift(0), head_joint1(1), head_joint2(2),
  arm joints 3-16`, no free joint).
- **Main loop** (2323-2581): obstacle mocap 2328-2353, `apply_continuous_head_sync` 2355,
  `get_targets` 2358, err-hist clear on target move 2361-2366, `solveStep` 2375-2377,
  `apply_soft_joint_locks` 2380, home/pose load 2452-2453, `apply_hardware_sync` 2456,
  `mj_forward` 2462, read achieved 2465-2485, `clip_target` 2566 (leash; accum never rebased).
  Teleop config: `collision_margin = 0.10`, `step_size 0.15` (~2212-2233).

---

## Design

### A. Branched solve (function A continuous / function B correction)

Two **disjoint public callables**; solveStep (mode A) is untouched. Rail mode (mode B) calls the
correction path, which **exits as soon as its critical check is true** (all-contact min_dist ≥
margin − tol) — it never chases equilibrium, per the user's branched-solve requirement. The main-loop
EE `solveStep` is **not run on ticks that applied a fresh rail command** (the correction already is
that tick's solve; state = the cleared command).

### B. f1 — `mapEeTwistToDq` (dual-arm 12D stacked, pure DLS, no mutation)

```cpp
// Pure DLS map: dual-arm endpoint twist -> joint velocity. Stateless: does NOT
// mutate d->qpos (no write, no mj_forward; d must be kinematically consistent).
//   ee_twist: 12D world-frame [v_l(3); w_l(3); v_r(3); w_r(3)] — the same stacked
//             task-row shape solveStep builds for the two EE sites.
//             Query ONE arm: pass zeros for the other arm's six rows.
// Returns the FULL nv_ dq; joints not ancestral to the driven site(s) come out
// ~0, so ranking |dq_i| shows which motor contributes most.
Eigen::VectorXd mapEeTwistToDq(mjData* d, const Eigen::VectorXd& ee_twist /* 12 */,
                               const SolverConfig& cfg = SolverConfig{}) const;

// Pure math core (testable without MuJoCo): dq = (JᵀWJ + λ²I)⁻¹ JᵀW v, W = row-weight diag.
static Eigen::VectorXd dlsMap(const Eigen::MatrixXd& J, const Eigen::VectorXd& v,
                              double damping, const Eigen::VectorXd& weights);
```

- Build one **12×nv stacked Jacobian** exactly as solveStep does (mj_jacSite into jacp/jacr for
  `id_l_` and `id_r_`; rows = pos-l, rot-l, pos-r, rot-r), left and right arm blocks for `id_l_` /
  `id_r_` (single-arm query = caller zeroes the other block → give it a zero cost row).
- Row weights mirror solveStep's task weighting: `pos_weight` on the three position rows and
  `ori_weight` on the three rotation rows of each arm block (×0 if `track_orientation` false), scaled
  by `left_weight_scale`/`right_weight_scale` per arm.
- Solve **DLS via the symmetric normal equations**: `A = JᵀWJ + λ²I` (λ = `cfg.damping`, squared so
  the map is scale-stable at a singular J), `rhs = JᵀWv`, `dq = A⁻¹rhs` via **LDLT** with a
  `fullPivLu` fallback if LDLT fails. No SVD needed. (Rationale for DLS over the ProxSuite QP: pure
  diagnostic map, no constraints to bind, deterministic, and it reuses none of solveStep's mutable
  scratch — the member QP stays warm for mode A.)
- Zero columns of J whose dof's joint name matches any `cfg.frozen_joints` substring (default
  `{"head"}`) so frozen joints stay out of the ranking; non-ancestor joints are already ~0.
- Output is **unscaled by `joint_vel_limit`** (scaling would not change relative `|dq_i|` ranking;
  the caller scales if it ever drives from this map). dq only, no state change.

### C. f2 — library: `clearToMargin` (bounded push-out to the nearest safe pose)

```cpp
// Bounded clearance-correction ("goal-from-correction" solve, function B). From the
// current d->qpos (already = the clamped commanded qpos), iteratively integrates a
// push-out least-norm QP (no EE task, no nullspace, no arm-fold attractor) driving
// every contact to >= col.collision_margin, and STOPS as soon as all-contact
// min_dist >= margin - tol (critical check true) or max_steps is hit.
// Mutates d->qpos (mj_forward + mj_integratePos). Returns true iff a safe pose was
// reached; *final_min_dist = clearance of the pose it leaves d in.
bool clearToMargin(mjData* d, const SolverConfig& cfg, const CollisionCostConfig& col,
                   int max_steps = 300, double* final_min_dist = nullptr);
// private, extend the 234-239 block:
bool buildClearQP(mjData* d, const SolverConfig& cfg, const CollisionCostConfig& col,
                  const ContactResult& contacts /* ALL contacts, ascending */,
                  double margin, double band, double h, Eigen::VectorXd& dq_out);
```

Loop per step (`h = min(0.1, max(1e-3, cfg.step_size))` — capped so the returned pose lands close
to the boundary; `tol = 1e-3`; `band = 0.02`):
1. `mj_forward(m_, d)`; `computeContacts(d, max(1, d->ncon), all)` (ALL contacts, ascending).
2. If `all.empty() || all.closest.front().dist >= margin - tol`: stop, report, return true.
3. `buildClearQP(...)` → dq; on QP failure return false. `mj_integratePos(m_, d->qpos, dq, h)`.
4. On budget exhaustion: one final `mj_forward` + all-contact query, write `*final_min_dist`, return
   whether it cleared.

`buildClearQP`: active set = prefix scan of contacts with `dist ≤ margin + band` and
`Jdist_row.size() == nv_`. `n_var = nv_ + n_active`, `n_ineq = nv_ + 2·n_active`. Cost
`H = diag(I_nv, slack_penalty·I)` (pure least-norm displacement — explicitly not mass-weighted, no
EE JᵀWJ, `g = 0`). Joint-bound rows mirror solveStep's arithmetic (jnt_range → per-step clamp) with
`h` in place of `cfg.step_size`; frozen joints (see §D coverage) get fixed rows. CBF rows mirror
solveStep's 599-616 form (`C_ext(nv_+k,·) = [Jdist_row, e_k]`, slack ≥ 0, `slack_penalty` cost) with
**exactly one difference: `lb_ext(nv_+k) = max(-cbf_alpha·(dist − margin), 0.0)`** — a violator
(`dist < margin`) gets a **positive** bound forcing separation at `cbf_alpha·(margin − dist)`, while
a near-safe band contact (`margin ≤ dist ≤ margin + band`) gets `lb = 0`, a wall that keeps the
push-out from trading one violation for another. (solveStep's `min(repulsion, 0.0)` at 613 would
zero the violator's bound too — that clamp is the entire wall/clear difference.) Slack ≥ 0 rows as
solveStep 591-594. Solve with a **function-local** `proxsuite::proxqp::dense::QP<double>` per step —
never the member `qp_`, so mode A's warm start is undisturbed. Return `results.x.head(nv_)`.

Why this matches the user's spec: the state starts AT the commanded qpos (FK-of-command pose is
where we are), so the correction is purely the minimum displacement that restores the margin; it
stops at the boundary (geometric deficit shrink with `cbf_alpha`), never drives on to equilibrium,
and returns the **nearest collision-safe pose to the command**. Iteration cost is bounded: the
deficit decays geometrically (≈ per-step factor `1 − cbf_alpha·h`), so a deep 0.1 m violation needs
≤ ~200 steps — a rare one-tick stall — and a near-safe command clears in a handful of steps.

### D. f2 — TeleopNode rail path (mode B, in scope)

New members (near 1803-1866): `enum class GoalSource { EE, RAIL }`; `std::atomic<GoalSource>
goal_source_{EE}`; `sensor_msgs::msg::JointState latest_rail_joints_` + `rail_seen_/rail_dirty_/
rail_stamp_` (message received at t0; `rail_timeout_ = 0.5`s); rail subscription handle; two
arbiters — `bool rail_goal_source() const { return goal_source_ == GoalSource::RAIL; }` (**mode
only**: which branch owns the tick) and `bool rail_active() const { return rail_goal_source() &&
rail_seen_ && (now() − rail_stamp_) < rail_timeout_; }` (**mode AND freshness**: gates the
mapper-affecting reads / stale-rail hold).

New subscription (near the existing `real_joint_sub_`, ~638-647): **`/qpos_rail`,
`sensor_msgs/msg/JointState`** — keep-newest: store `latest_rail_joints_`, set `rail_seen_` +
`rail_stamp_`, flag `rail_dirty_`. JointState-by-name is chosen because it reuses the node's
existing name→qposadr loop verbatim (apply_hardware_sync 969-990), allows partial commands
(head/arms/lift optionally), and needs no new message definition. No ZMQ protocol change (out of
scope).

New param `goal_source` (`ee` default), declared ~428, **runtime-flippable two ways**: the existing
`on_set_parameters` callback (437-461) AND a **`/teleop_goal_source` std_msgs/String** subscription
(`"ee"`/`"rail"`, keep-newest) — both write `goal_source_` (the topic is the natural channel for a
continuous external controller; the param for launch-time defaulting and manual flipping). Launch
file passes the static default through so `goal_source:=rail` works without code edits.

New method (place after `apply_continuous_head_sync`, ~1051):

```cpp
bool apply_rail_sync(mjModel* m, mjData* d, ffw_ik::IKSolver& solver,
                     const ffw_ik::SolverConfig& cfg,
                     const ffw_ik::CollisionCostConfig& col);
```

Body (the snap → FK-adopt → re-clamp → clear sequence; returns **true exactly when a fresh rail
message was applied** this tick):
1. Guard: if `!rail_goal_source()` return false; if `!rail_dirty_` return false (no fresh command →
   hold — nothing moves `d->qpos` between rail ticks).
2. Consume: copy `latest_rail_joints_`, clear `rail_dirty_` (one-shot per message — a rejected
   message WARNs once, not every tick).
3. Build `q_cmd` (init from current `d->qpos`): mirror the apply_hardware_sync name loop — known
   joint names → qpos via `mj_name2id`+`mj_jnt_qposadr`; clamp every limited joint to `jnt_range`.
   **Coverage rule**: require every arm joint of each driven arm (weight_scale > 0) + the lift joint
   present, else WARN and ignore (return false, hold). Joints absent from the message are treated
   as frozen at their current value.
4. Snapshot `prev_qpos` (rollback). Write `q_cmd` into `d->qpos`, `mj_forward` — **the snap**.
5. **Critical check** (ALL contacts — a 6th-closest violator must not be invisible):
   `computeContacts(d, max(1, d->ncon), all)`.
   - If `min_dist ≥ col.collision_margin`: safe → keep the commanded state verbatim, skip correction.
   - Else run `clearToMargin(d, clear_cfg, col, 300, &final_min)` where `clear_cfg` carries
     `frozen_joints` = exactly the joints the rail did NOT command (+ disabled groups), plus
     `nullspace_weight = 0`, `inertia_weight = 0`, `step_size = min(cfg.step_size, 0.05)` (land
     close to the boundary). On failure, restore `prev_qpos` + `mj_forward`, WARN, hold the last
     safe pose.
6. **FK-adopt** under `pose_mutex_` (same reads as apply_hardware_sync 994-1024): read the two
   gripper sites into `initial_*_`/`target_*_`, zero `accum_*_`, park `last_mapper_*_` on the pose,
   set `last_goal_from_quest_*_ = false`. **(the snap updated the goal — any EE goal is ignored)**
7. Return true. Re-clamp (`clip_target`, 2566) is then trivially a no-op: adopted target == achieved.

Main-loop wiring (2323-2581): read `rail_gs = rail_goal_source()` once per tick. **Rail mode owns
the tick** — while `rail_gs`, the entire EE goal chase is suppressed:
- The four EE-goal callbacks (`update_goal_delta_l/r` 1453/1512, `update_goal_pose_l/r`
  1611/1663) early-return on `rail_goal_source()` — "ignore the EE goal if any".
- 2355 `apply_continuous_head_sync`, 2380 soft-joint-locks, 2452-2453 home/pose load, and 2456
  `apply_hardware_sync` are all gated `if (!rail_gs)` (a real `/joint_states` snapshot or a homing
  trigger must not fight the rail).
- `apply_rail_sync(...)` runs every rail tick; its own `rail_dirty_` gate makes it **return true
  only on a fresh applied command**, false (hold) on stale ticks.
- `solveStep` 2375-2377 runs **only when `!rail_gs`** — on a fresh-rail tick the correction *was*
  the solve (function B, exited on its critical check); never stack an equilibrium solveStep on top.
- Transitions: EE→rail on a param/topic flip sets `hardware_sync_requested_ = false` and disables
  homing; rail→EE re-arms `hardware_sync_requested_` when in hardware mode.
- Stale-rail hold: if no message arrives within `rail_timeout_`, log once and keep holding the last
  parked state; mode A resumes from the parked/adopted pose without a jump (mapper re-bases on
  `/ik_solver/achieved_ee_pose_*`).

---

## Files to touch

- `ffw_collision_checker/include/ffw_ik_solver.h` — declare `dlsMap` + `mapEeTwistToDq` +
  `clearToMargin` (after line 188, before `private:`), `buildClearQP` (extend the 234-239 block).
- `ffw_collision_checker/src/ffw_ik_solver.cpp` — define all four in a new section after solveStep
  (after 687). **Do not touch solveStep.**
- `ffw_collision_checker/src/ffw_ik_solver_teleop.cpp` — rail members + `apply_rail_sync` +
  `/qpos_rail` + `/teleop_goal_source` subscriptions + `goal_source` param (runtime-flippable) +
  EE-callback guards + main-loop branch. (No CMakeLists change: the solver header and this file
  already build into `ffw_ik_solver_lib` and the teleop binary.)
- `ffw_collision_checker/src/ffw_ik_solver_test.cpp` — add f1/f2 unit checks (below).
- `ffw_spacemouse/launch/spacemouse_unified_teleop.launch.py` — pass `goal_source` (default `ee`)
  to the teleop node (302-307).
- `ffw_collision_checker/iksolver_functions_plan.md` — this document.

---

## Verification

1. **Build + regression**: `colcon build --packages-select ffw_collision_checker`; run
   `ffw_ik_solver_test` before/after step 2 — convergence count and objective profile identical
   (proves solveStep untouched).
2. **`dlsMap` unit** (pure math): identity J → `dq = v/(1+λ²)`; rank-deficient J → finite dq,
   least-squares residual; zero rotational weights → translational-only response.
3. **`mapEeTwistToDq`**: at bg2 home config, command a pure x-translation for the left block (right
   zeros); assert the driven-row residual `‖W_l·(J·dq − v)‖` is small, |dq|-energy concentrates in
   the left arm + `lift_joint` (ancestral to BOTH EE sites — it legitimately participates, so the
   right-site rows do NOT stay ~0), `head_joint1/2` dq < 1e-9 (non-ancestor), `d->qpos` unchanged,
   and a zero 12D twist maps to `dq ≈ 0`.
4. **`clearToMargin`** (headless): take a safe config, then write a `q_cmd` that moves one arm into
   the static pole so all-contact min < 0.10; assert pre-clear min < margin → after clear,
   min ≥ margin − 1e-3, `‖q_corr − q_start‖` small (nearest-safe), and it stopped at the boundary,
   not in mid-free-space.
5. **Teleop rail integration** (hardware_mode:=false, `goal_source:=rail`): publish home-qpos
   `/qpos_rail` at 100 Hz → robot parks; publish an unsafe command (lift into the pole) → published
   sim distance never drops below margin and the arm holds at the boundary, not at the raw command;
   flip `goal_source` back to `ee` — via `ros2 param set` OR `ros2 topic pub /teleop_goal_source
   std_msgs/String "{data: ee}"` — → spacemouse delta resumes from the parked pose with no jump.
6. **Default-mode regression** (`goal_source:=ee`): existing spacemouse/quest flows unchanged
   (only executed difference is the new param read + four early-return guards that are false).

---

## Out of scope

- ZMQ gateway/ControlCmd qpos channel (no protocol change this pass).
- RadioControlMapper feed-forward use of f1 (library callable is ready for it later).
- Refactoring solveStep / defining the four legacy private helpers.
