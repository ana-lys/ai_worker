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
    double damping              = 4e-3;
    double step_size            = 0.2;
    double tolerance            = 5e-3;
    double joint_vel_limit      = 3.1;
    int    max_steps            = 100;
    int    topk_contacts        = 5;
    bool   qp_verbose           = false;

    // Sliding-window early-stop: end-effector improvement
    int    ee_window            = 5;
    double ee_improvement_rate  = 0.02;

    // Sliding-window early-stop: minimum contact distance stability
    int    dist_window          = 5;
    double dist_stability_thresh = 0.002;
    double dist_safe_ratio      = 0.98;

    // Minimum objective to accept an early-stop as "converged"
    double early_convergence_obj = 0.87;
};

struct CollisionCostConfig {
    double collision_margin = 0.155;
    double weight_scale     = 0.01;
    double epsilon          = 1e-1;
};

// ============================================================
// Internal data types (exposed for inspection / logging)
// ============================================================

struct ContactInfo {
    int             contact_index = -1;
    double          dist          = std::numeric_limits<double>::infinity();
    int             geom1 = -1, geom2 = -1;
    int             body1 = -1, body2 = -1;
    Eigen::Vector3d normal        = Eigen::Vector3d::Zero();
    Eigen::Vector3d p1            = Eigen::Vector3d::Zero();
    Eigen::Vector3d p2            = Eigen::Vector3d::Zero();
    std::vector<mjtNum> Jdist_row;
};

struct ContactResult {
    int                      total_contacts = 0;
    std::vector<ContactInfo> closest;
};

struct StepResult {
    double error               = std::numeric_limits<double>::infinity();
    double objective_ee        = 0.0;
    double objective_collision = 0.0;
    double objective_total     = 0.0;
    double min_dist            = std::numeric_limits<double>::quiet_NaN();
    bool   converged           = false;
    bool   early_converged     = false;
    bool   stalled             = false;
    ContactResult contacts;
};

// ============================================================
// IKSolver
// ============================================================

class IKSolver {
public:
    explicit IKSolver(mjModel* model);

    // ----------------------------------------------------------
    // High-level API
    //
    // Runs IK from the current d->qpos toward (target_l, target_r).
    // Returns a variable-length trajectory: one qpos snapshot per step,
    // including the final (converged or stopped) pose.
    //
    // d->qpos is mutated in-place; callers should save/restore if needed.
    // ----------------------------------------------------------
    std::vector<Eigen::VectorXd> solve(
        mjData*                 d,
        const Eigen::Vector3d&  target_l,
        const Eigen::Vector3d&  target_r,
        const SolverConfig&     cfg     = SolverConfig{},
        const CollisionCostConfig& col  = CollisionCostConfig{});

    // ----------------------------------------------------------
    // Low-level API — single optimisation step.
    // Advances d->qpos by one QP solve and returns diagnostics.
    // ----------------------------------------------------------
    StepResult solveStep(
        mjData*                 d,
        const Eigen::Vector3d&  target_l,
        const Eigen::Vector3d&  target_r,
        const SolverConfig&     cfg,
        const CollisionCostConfig& col,
        std::deque<double>&     error_history,
        std::deque<double>&     dist_history);

    // Scalar objective functions (exposed for external logging)
    static double eeObjective(double error);
    static double collisionObjective(double min_dist);

    // Contact query
    void computeContacts(mjData* d, int topk, ContactResult& out);

    // Pretty-print one step to stdout
    void printStep(int step, const StepResult& r) const;

private:
    mjModel*        m_;
    int             nv_;
    int             id_l_;   // site id: left_gripper_site
    int             id_r_;   // site id: right_gripper_site

    // Reusable Eigen workspace
    Eigen::MatrixXd J_;         // (6, nv)
    Eigen::MatrixXd H_;         // (nv, nv)
    Eigen::MatrixXd C_;         // identity (nv, nv) — QP inequality lhs
    Eigen::VectorXd g_;         // (nv)
    Eigen::VectorXd err_;       // (6)
    Eigen::VectorXd lb_, ub_;   // (nv)

    // Collision gradient workspace
    Eigen::MatrixXd Jdist_all_; // (max_contacts, nv)
    Eigen::VectorXd dist_all_;
    Eigen::VectorXd w_all_;

    // MuJoCo Jacobian scratch buffers (row-major, mjtNum)
    std::vector<mjtNum> jacp_l_, jacp_r_, jacp_c1_, jacp_c2_;

    // Contact index scratch (avoids per-step allocation in computeContacts)
    std::vector<int> scratch_;

    // Cached QP — warm-started across steps
    proxsuite::proxqp::dense::QP<double> qp_;
    bool qp_initialized_ = false;

    void buildJacobian(mjData* d);
    void buildCollisionGradient(const ContactResult& contacts,
                                const CollisionCostConfig& col,
                                int& n_within_out);
    void buildJointBounds(mjData* d, const SolverConfig& cfg);
    void solveQP(const SolverConfig& cfg);
};

} // namespace ffw_ik