#pragma once

#include <deque>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <proxsuite/proxqp/dense/dense.hpp>

// ---------------------------------------------------------------------------
// FFW IK Solver
// Damped-least-squares QP-based inverse kinematics for the FFW dual-arm robot.
//
// Usage:
//   IKSolver solver(model);
//   auto traj = solver.solve(data, target_left, target_right, cfg, col_cfg);
//   // traj[i] is the joint position vector (nq) at step i
// ---------------------------------------------------------------------------

namespace ffw_ik {

// ============================================================
// Configuration
// ============================================================

struct SolverConfig {
  double damping = 4e-3;
  double step_size = 0.2;
  double tolerance = 5e-3;
  double joint_vel_limit = 3.1;
  int max_steps = 100;
  int topk_contacts = 5;
  bool qp_verbose = false;
  bool track_orientation =
      true; // If true, tracks 6D pose. If false, tracks only 3D position.

  // Task-space weights
  double pos_weight = 1.0; // Weight for translational error (meters)
  double ori_weight =
      0.075; // Weight for rotational error (radians). Increased by 50%.
  double left_weight_scale = 1.0;
  double right_weight_scale = 1.0;

  // Dynamic Orientation Weights ("Reach then Align")
  double dynamic_ori_far_thresh =
      0.3; // Distance (m) beyond which ori_weight is heavily discounted
  double dynamic_ori_close_thresh =
      0.1; // Distance (m) under which ori_weight is applied 100%
  double dynamic_ori_far_discount =
      0.20; // Scale factor applied to ori_weight when far away (80% discount)

  // Adaptive Damping
  double adaptive_damping_drop_thresh =
      0.10; // Error under which damping scales down proportionally
  double adaptive_damping_min_scale =
      0.01; // Minimum damping scale to avoid numerical instability

  // Mass-weighted H: λ * M added to J^T J before damping.
  // Heavier DOFs (torso) become proportionally more expensive to move,
  // so the solver naturally reaches targets via lighter distal joints first.
  // Tune in the range [1e-4, 1e-2]; set to 0.0 to disable.
  double inertia_weight = 5e-3;

  // Sliding-window early-stop: end-effector improvement
  int ee_window = 5;
  double ee_improvement_rate = 0.02;

  // Sliding-window early-stop: minimum contact distance stability
  int dist_window = 5;
  double dist_stability_thresh = 0.002;
  double dist_safe_ratio = 0.98;

  // Minimum objective to accept an early-stop as "converged"
  double early_convergence_obj = 0.87;

  // Dynamic Null-Space Injection
  double nullspace_weight = 1e-2;   // Priority of the null-space cost
  double nullspace_amplitude = 0.5; // Max velocity amplitude (rad/s)
  double nullspace_frequency = 0.2; // Frequency of the sine wave (Hz)
  int nullspace_type = 0;           // 0 = Sinusoidal, 1 = Joint Centering

  // Joints to freeze during IK (e.g. head, neck)
  std::vector<std::string> frozen_joints = {"head"};
};

struct CollisionCostConfig {
  double collision_margin = 0.155; // Used as d_safe
  double cbf_alpha = 0.5; // How aggressively to enforce the safe margin
  double slack_penalty =
      1e6; // Massive penalty for getting too close (infeasibility fallback)

  // Legacy soft repulsion params (kept for backwards compatibility if needed)
  double weight_scale = 0.01;
  double epsilon = 1e-1;

  double arm_dist_weight = 0.025;
};

// ============================================================
// Internal data types (exposed for inspection / logging)
// ============================================================

struct ContactInfo {
  int contact_index = -1;
  double dist = std::numeric_limits<double>::infinity();
  int geom1 = -1, geom2 = -1;
  int body1 = -1, body2 = -1;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p2 = Eigen::Vector3d::Zero();
  std::vector<mjtNum> Jdist_row;
};

struct ContactResult {
  int total_contacts = 0;
  std::vector<ContactInfo> closest;
};

struct StepResult {
  double error = std::numeric_limits<double>::infinity();
  double objective_ee = 0.0;
  double objective_collision = 0.0;
  double objective_total = 0.0;
  double min_dist = std::numeric_limits<double>::quiet_NaN();
  bool converged = false;
  bool early_converged = false;
  bool stalled = false;
  ContactResult contacts;
};

// ============================================================
// IKSolver
// ============================================================

class IKSolver {
public:
  explicit IKSolver(mjModel *model);

  // ----------------------------------------------------------
  // High-level API
  //
  // Runs IK from the current d->qpos toward (target_l, target_r).
  // Returns a variable-length trajectory: one qpos snapshot per step,
  // including the final (converged or stopped) pose.
  //
  // d->qpos is mutated in-place; callers should save/restore if needed.
  // ----------------------------------------------------------
  // ----------------------------------------------------------
  // High-level API (Pose Target)
  // ----------------------------------------------------------
  std::vector<Eigen::VectorXd>
  solve(mjData *d, const Eigen::Isometry3d &target_l,
        const Eigen::Isometry3d &target_r,
        const SolverConfig &cfg = SolverConfig{},
        const CollisionCostConfig &col = CollisionCostConfig{});

  // ----------------------------------------------------------
  // High-level API (Position Target)
  // ----------------------------------------------------------
  std::vector<Eigen::VectorXd>
  solve(mjData *d, const Eigen::Vector3d &target_l,
        const Eigen::Vector3d &target_r,
        const SolverConfig &cfg = SolverConfig{},
        const CollisionCostConfig &col = CollisionCostConfig{});

  // ----------------------------------------------------------
  // Low-level API — single optimisation step.
  // ----------------------------------------------------------
  StepResult solveStep(mjData *d, const Eigen::Isometry3d &target_l,
                       const Eigen::Isometry3d &target_r,
                       const SolverConfig &cfg, const CollisionCostConfig &col,
                       std::deque<double> &error_history,
                       std::deque<double> &dist_history);

  // Scalar objective functions (exposed for external logging)
  static double eeObjective(double error);
  static double collisionObjective(double min_dist);

  // Contact query
  void computeContacts(mjData *d, int topk, ContactResult &out);

  // Pretty-print one step to stdout
  void printStep(int step, const StepResult &r) const;

  // Kinematic queries (baked into core class)
  std::string getKinematicTree(mjData *d) const;
  std::vector<std::string> getJointNames() const;

private:
  mjModel *m_;
  int nv_;
  int id_l_;   // site id: left_gripper_site
  int id_r_;   // site id: right_gripper_site
  int id_lUA_; // site id: left_UA_site
  int id_lUB_; // site id: left_UB_site
  int id_rUA_; // site id: right_UA_site
  int id_rUB_; // site id: right_UB_site

  // Reusable Eigen workspace
  Eigen::MatrixXd J_;       // (6, nv)
  Eigen::MatrixXd H_;       // (nv, nv)
  Eigen::MatrixXd C_;       // identity (nv, nv) — QP inequality lhs
  Eigen::MatrixXd M_;       // (nv, nv) — dense inertia matrix
  Eigen::VectorXd g_;       // (nv)
  Eigen::VectorXd err_;     // (6)
  Eigen::VectorXd lb_, ub_; // (nv)

  // Collision gradient workspace
  Eigen::MatrixXd Jdist_all_; // (max_contacts, nv)
  Eigen::VectorXd dist_all_;
  Eigen::VectorXd w_all_;

  // MuJoCo Jacobian scratch buffers (row-major, mjtNum)
  std::vector<mjtNum> jacp_l_, jacp_r_, jacp_c1_, jacp_c2_;
  std::vector<mjtNum> jacr_l_, jacr_r_;
  std::vector<mjtNum> jacp_lUA_, jacp_lUB_, jacp_rUA_, jacp_rUB_;
  // Contact index scratch (avoids per-step allocation in computeContacts)
  std::vector<int> scratch_;

  // Expanded Eigen workspace for Slack variables (nv + topk_contacts)
  Eigen::MatrixXd H_ext_;
  Eigen::VectorXd g_ext_;
  Eigen::MatrixXd C_ext_;
  Eigen::VectorXd lb_ext_;
  Eigen::VectorXd ub_ext_;

  // Cached QP — warm-started across steps
  // Using pointer to allow lazy initialization based on cfg.topk_contacts
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  int qp_topk_ = -1; // track the allocated size
  bool qp_initialized_ = false;

  void buildJacobian(mjData *d);
  void buildCollisionGradient(const ContactResult &contacts,
                              const CollisionCostConfig &col,
                              int &n_within_out);
  void buildJointBounds(mjData *d, const SolverConfig &cfg);
  void solveQP(const SolverConfig &cfg);

  // Dynamic Null-Space Injection State
  double nullspace_time_ = 0.0;
  std::vector<double> nullspace_phase_offsets_;
  std::vector<double> nullspace_frequencies_;
  double last_active_nullspace_weight_ = 0.0;

  // Interleaved Manipulability Gradient State
  Eigen::VectorXd manipulability_grad_;
  int manipulability_idx_ = 0;

public:
  double getLastActiveNullSpaceWeight() const {
    return last_active_nullspace_weight_;
  }
};

// ============================================================
// MultiIKSolver
// ============================================================

class MultiIKSolver {
public:
  explicit MultiIKSolver(mjModel *m);
  ~MultiIKSolver();

  // Evaluates 3 solvers concurrently from the same initial `d` state.
  // Overwrites `d` with the winning solver's next state based on EE error.
  StepResult solveStepMulti(mjData *d, const Eigen::Isometry3d &target_l,
                            const Eigen::Isometry3d &target_r, 
                            const SolverConfig &cfg, 
                            const CollisionCostConfig &col_cfg,
                            std::deque<double> &err_hist,
                            std::deque<double> &dist_hist);

private:
  mjModel *m_;
  std::unique_ptr<IKSolver> solver0_, solver1_, solver2_;
  mjData *d0_, *d1_, *d2_;
  int current_best_idx_;
};

} // namespace ffw_ik