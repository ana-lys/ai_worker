#include "ffw_ik_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <numeric>
#include <stdexcept>
#include <future>
#include <iostream>

namespace ffw_ik {

static double computeManipulability(mjModel* m, mjData* d, int id_l, int id_r, int nv, std::vector<mjtNum>& jacp_l, std::vector<mjtNum>& jacr_l, std::vector<mjtNum>& jacp_r, std::vector<mjtNum>& jacr_r, bool track_ori) {
    int task_dim = track_ori ? 12 : 6;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(task_dim, nv);
    using RowMat3xN = Eigen::Matrix<mjtNum, 3, Eigen::Dynamic, Eigen::RowMajor>;
    
    if (track_ori) {
        mj_jacSite(m, d, jacp_l.data(), jacr_l.data(), id_l);
        mj_jacSite(m, d, jacp_r.data(), jacr_r.data(), id_r);
        J.block(0, 0, 3, nv) = Eigen::Map<const RowMat3xN>(jacp_l.data(), 3, nv).cast<double>();
        J.block(3, 0, 3, nv) = Eigen::Map<const RowMat3xN>(jacr_l.data(), 3, nv).cast<double>();
        J.block(6, 0, 3, nv) = Eigen::Map<const RowMat3xN>(jacp_r.data(), 3, nv).cast<double>();
        J.block(9, 0, 3, nv) = Eigen::Map<const RowMat3xN>(jacr_r.data(), 3, nv).cast<double>();
    } else {
        mj_jacSite(m, d, jacp_l.data(), nullptr, id_l);
        mj_jacSite(m, d, jacp_r.data(), nullptr, id_r);
        J.block(0, 0, 3, nv) = Eigen::Map<const RowMat3xN>(jacp_l.data(), 3, nv).cast<double>();
        J.block(3, 0, 3, nv) = Eigen::Map<const RowMat3xN>(jacp_r.data(), 3, nv).cast<double>();
    }
    
    Eigen::MatrixXd JJt = J * J.transpose();
    double det = JJt.determinant();
    if (det < 0.0) det = 0.0;
    return std::sqrt(det);
}

// ============================================================
// Construction
// ============================================================

IKSolver::IKSolver(mjModel* model)
    : m_(model)
    , nv_(model->nv)
{
    J_.resize(12, nv_); // Max size 12
    err_.resize(12);    // Max size 12
    g_.resize(nv_);
    C_ = Eigen::MatrixXd::Identity(nv_, nv_);
    lb_.resize(nv_);
    ub_.resize(nv_);
    M_.resize(nv_, nv_);

    jacp_l_.resize(3 * nv_);
    jacp_r_.resize(3 * nv_);
    jacr_l_.resize(3 * nv_);
    jacr_r_.resize(3 * nv_);
    jacp_c1_.resize(3 * nv_);
    jacp_c2_.resize(3 * nv_);
    jacp_lUA_.resize(3 * nv_);
    jacp_lUB_.resize(3 * nv_);
    jacp_rUA_.resize(3 * nv_);
    jacp_rUB_.resize(3 * nv_);

    const int max_contacts = (model->nconmax > 0) ? model->nconmax : 512;
    Jdist_all_.resize(max_contacts, nv_);
    dist_all_.resize(max_contacts);
    w_all_.resize(max_contacts);
    scratch_.reserve(max_contacts);

    id_l_   = mj_name2id(m_, mjOBJ_SITE, "left_gripper_site");
    id_r_   = mj_name2id(m_, mjOBJ_SITE, "right_gripper_site");
    id_lUA_ = mj_name2id(m_, mjOBJ_SITE, "left_UA_site");
    id_lUB_ = mj_name2id(m_, mjOBJ_SITE, "left_UB_site");
    id_rUA_ = mj_name2id(m_, mjOBJ_SITE, "right_UA_site");
    id_rUB_ = mj_name2id(m_, mjOBJ_SITE, "right_UB_site");

    nullspace_phase_offsets_.resize(nv_);
    nullspace_frequencies_.resize(nv_);
    for (int i = 0; i < nv_; ++i) {
        // Pseudo-random phase offsets so joints don't swing in perfect unison
        nullspace_phase_offsets_[i] = std::sin(i * 1.37) * M_PI;
        // Pseudo-random frequency multipliers so joints oscillate at different speeds
        nullspace_frequencies_[i] = 1.0 + 0.3 * std::sin(i * 2.11);
    }
}

// ============================================================
// High-level solve
// ============================================================

std::vector<Eigen::VectorXd> IKSolver::solve(
    mjData*                    d,
    const Eigen::Vector3d&     target_l,
    const Eigen::Vector3d&     target_r,
    const SolverConfig&        cfg,
    const CollisionCostConfig& col)
{
    Eigen::Isometry3d iso_l = Eigen::Isometry3d::Identity();
    iso_l.translation() = target_l;
    Eigen::Isometry3d iso_r = Eigen::Isometry3d::Identity();
    iso_r.translation() = target_r;
    
    SolverConfig pos_cfg = cfg;
    pos_cfg.track_orientation = false;
    
    return solve(d, iso_l, iso_r, pos_cfg, col);
}

std::vector<Eigen::VectorXd> IKSolver::solve(
    mjData*                    d,
    const Eigen::Isometry3d&   target_l,
    const Eigen::Isometry3d&   target_r,
    const SolverConfig&        cfg,
    const CollisionCostConfig& col)
{
    std::vector<Eigen::VectorXd> trajectory;
    trajectory.reserve(cfg.max_steps);

    std::deque<double> error_history;
    std::deque<double> dist_history;

    for (int step = 0; step < cfg.max_steps; ++step) {
        StepResult r = solveStep(d, target_l, target_r, cfg, col,
                                 error_history, dist_history);

        trajectory.emplace_back(
            Eigen::Map<const Eigen::VectorXd>(d->qpos, m_->nq));

        if (r.converged || r.early_converged || r.stalled)
            break;
    }

    return trajectory;
}

// ============================================================
// Objective functions
// ============================================================

double IKSolver::eeObjective(double error) {
    return 0.9 * std::exp(-std::max(0.0, error) / 0.5);
}

double IKSolver::collisionObjective(double min_dist) {
    if (!std::isfinite(min_dist)) return 0.0;
    return 0.1 * (1.0 - std::exp(-std::max(0.0, min_dist) / 0.05));
}

// ============================================================
// Contact query
// ============================================================

void IKSolver::computeContacts(mjData* d, int topk, ContactResult& out) {
    out.total_contacts = d ? d->ncon : 0;
    out.closest.clear();
    if (!d || d->ncon <= 0 || topk <= 0) return;

    const int k = std::min(topk, d->ncon);

    scratch_.resize(d->ncon);
    std::iota(scratch_.begin(), scratch_.end(), 0);

    auto cmp = [d](int a, int b){ return d->contact[a].dist < d->contact[b].dist; };
    std::nth_element(scratch_.begin(), scratch_.begin() + k, scratch_.end(), cmp);
    scratch_.resize(k);
    std::sort(scratch_.begin(), scratch_.end(), cmp);

    out.closest.reserve(k);
    for (int ci : scratch_) {
        const mjContact& c = d->contact[ci];
        ContactInfo info;
        info.contact_index = ci;
        info.dist   = c.dist;
        info.geom1  = c.geom1;
        info.geom2  = c.geom2;
        info.body1  = (c.geom1 >= 0) ? m_->geom_bodyid[c.geom1] : -1;
        info.body2  = (c.geom2 >= 0) ? m_->geom_bodyid[c.geom2] : -1;
        info.normal = Eigen::Vector3d(c.frame[0], c.frame[1], c.frame[2]);

        Eigen::Vector3d mid(c.pos[0], c.pos[1], c.pos[2]);
        info.p1 = mid - 0.5 * c.dist * info.normal;
        info.p2 = mid + 0.5 * c.dist * info.normal;

        if (info.body1 >= 0 && info.body2 >= 0) {
            const mjtNum pt1[3] = {info.p1.x(), info.p1.y(), info.p1.z()};
            const mjtNum pt2[3] = {info.p2.x(), info.p2.y(), info.p2.z()};
            mj_jac(m_, d, jacp_c1_.data(), nullptr, pt1, info.body1);
            mj_jac(m_, d, jacp_c2_.data(), nullptr, pt2, info.body2);

            using RowMat = Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
            Eigen::Map<const RowMat> Jc1(jacp_c1_.data(), 3, nv_);
            Eigen::Map<const RowMat> Jc2(jacp_c2_.data(), 3, nv_);

            Eigen::Matrix<mjtNum, 1, Eigen::Dynamic> Jdist =
                info.normal.transpose().cast<mjtNum>() * (Jc2 - Jc1);

            info.Jdist_row.resize(nv_);
            Eigen::Map<Eigen::Matrix<mjtNum, 1, Eigen::Dynamic>>(
                info.Jdist_row.data(), nv_) = Jdist;
        }
        out.closest.push_back(std::move(info));
    }
}

// ============================================================
// Single IK step
// ============================================================

StepResult IKSolver::solveStep(
    mjData*                    d,
    const Eigen::Isometry3d&   target_l,
    const Eigen::Isometry3d&   target_r,
    const SolverConfig&        cfg,
    const CollisionCostConfig& col,
    std::deque<double>&        error_history,
    std::deque<double>&        dist_history)
{
    StepResult result;

    mj_forward(m_, d);

    // End-effector error
    Eigen::Vector3d pos_l = Eigen::Vector3d::Map(d->site_xpos + 3 * id_l_);
    Eigen::Vector3d pos_r = Eigen::Vector3d::Map(d->site_xpos + 3 * id_r_);

    int task_dim = cfg.track_orientation ? 12 : 6;
    
    // Create mapped views of the resized arrays without calling expensive allocation methods
    Eigen::VectorXd err_view = Eigen::VectorXd::Zero(task_dim);
    Eigen::MatrixXd J_view = Eigen::MatrixXd::Zero(task_dim, nv_);

    if (cfg.track_orientation) {
        Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> R_curr_l(d->site_xmat + 9 * id_l_);
        Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> R_curr_r(d->site_xmat + 9 * id_r_);

        // Angular error using log map / AngleAxis
        Eigen::AngleAxisd aa_l(target_l.rotation() * R_curr_l.cast<double>().transpose());
        Eigen::AngleAxisd aa_r(target_r.rotation() * R_curr_r.cast<double>().transpose());

        err_view.segment<3>(0) = pos_l - target_l.translation();
        err_view.segment<3>(3) = - (aa_l.axis() * aa_l.angle());
        err_view.segment<3>(6) = pos_r - target_r.translation();
        err_view.segment<3>(9) = - (aa_r.axis() * aa_r.angle());
    } else {
        err_view.segment<3>(0) = pos_l - target_l.translation();
        err_view.segment<3>(3) = pos_r - target_r.translation();
    }
    
    err_ = err_view;

    // Dynamic Orientation Weight ("Reach then Align")
    double current_ori_weight = cfg.ori_weight;
    if (cfg.track_orientation) {
        double pos_err_l = (pos_l - target_l.translation()).norm();
        double pos_err_r = (pos_r - target_r.translation()).norm();
        double max_pos_err = std::max(pos_err_l, pos_err_r);

        double alpha = 1.0;
        if (max_pos_err >= cfg.dynamic_ori_far_thresh) {
            alpha = cfg.dynamic_ori_far_discount;
        } else if (max_pos_err <= cfg.dynamic_ori_close_thresh) {
            alpha = 1.0;
        } else {
            // Linear interpolation between the two thresholds
            double range = cfg.dynamic_ori_far_thresh - cfg.dynamic_ori_close_thresh;
            double progress = (cfg.dynamic_ori_far_thresh - max_pos_err) / range;
            alpha = cfg.dynamic_ori_far_discount + (1.0 - cfg.dynamic_ori_far_discount) * progress;
        }
        current_ori_weight *= alpha;
    }

    // Apply task-space weights to balance position (meters) vs orientation (radians)
    Eigen::VectorXd task_weight = Eigen::VectorXd::Ones(task_dim);
    if (cfg.track_orientation) {
        task_weight.segment<3>(0).setConstant(cfg.pos_weight * cfg.left_weight_scale);
        task_weight.segment<3>(3).setConstant(current_ori_weight * cfg.left_weight_scale);
        task_weight.segment<3>(6).setConstant(cfg.pos_weight * cfg.right_weight_scale);
        task_weight.segment<3>(9).setConstant(current_ori_weight * cfg.right_weight_scale);
    } else {
        task_weight.segment<3>(0).setConstant(cfg.pos_weight * cfg.left_weight_scale);
        task_weight.segment<3>(3).setConstant(cfg.pos_weight * cfg.right_weight_scale);
    }

    // Compute weighted error norm for early stopping detection
    result.error = (task_weight.asDiagonal() * err_).norm();

    // Contact query
    computeContacts(d, cfg.topk_contacts, result.contacts);
    result.min_dist = result.contacts.closest.empty()
        ? 0.30
        : result.contacts.closest.front().dist;

    result.objective_ee        = eeObjective(result.error);
    result.objective_collision = collisionObjective(result.min_dist);
    result.objective_total     = result.objective_ee + result.objective_collision;

    // Hard convergence check
    if (result.error < cfg.tolerance) {
        result.converged = true;
        return result;
    }

    // Sliding-window early-stop
    error_history.push_back(result.error);
    if ((int)error_history.size() > cfg.ee_window)
        error_history.pop_front();

    if (std::isfinite(result.min_dist)) {
        dist_history.push_back(result.min_dist);
        if ((int)dist_history.size() > cfg.dist_window)
            dist_history.pop_front();
    }

    if ((int)error_history.size() == cfg.ee_window) {
        double old_err  = error_history.front();
        double rate     = (old_err - result.error) / std::max(old_err, 1e-9);
        bool ee_stalled = rate < cfg.ee_improvement_rate;

        bool dist_stable_and_safe = false;
        if (result.contacts.total_contacts == 0) {
            result.objective_collision = 0.1;
            result.min_dist            = col.collision_margin;
            result.objective_total     = result.objective_ee + result.objective_collision;
            dist_stable_and_safe       = true;
        } else if ((int)dist_history.size() == cfg.dist_window) {
            double mean = 0.0;
            for (double v : dist_history) mean += v;
            mean /= dist_history.size();
            double var = 0.0;
            for (double v : dist_history) var += (v - mean) * (v - mean);
            var /= dist_history.size();
            bool stable = var < cfg.dist_stability_thresh;
            bool safe   = std::isfinite(result.min_dist) &&
                          result.min_dist > col.collision_margin * cfg.dist_safe_ratio;
            dist_stable_and_safe = stable && safe;
        }

        if (ee_stalled && dist_stable_and_safe) {
            result.early_converged = result.objective_total > cfg.early_convergence_obj;
            result.stalled         = !result.early_converged;
            return result;
        }
    }

    // Jacobians — mj_jacSite fills row-major (3, nv) blocks
    using RowMat3xN = Eigen::Matrix<mjtNum, 3, Eigen::Dynamic, Eigen::RowMajor>;
    
    if (cfg.track_orientation) {
        mj_jacSite(m_, d, jacp_l_.data(), jacr_l_.data(), id_l_);
        mj_jacSite(m_, d, jacp_r_.data(), jacr_r_.data(), id_r_);
        J_view.block(0, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacp_l_.data(), 3, nv_).cast<double>();
        J_view.block(3, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacr_l_.data(), 3, nv_).cast<double>();
        J_view.block(6, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacp_r_.data(), 3, nv_).cast<double>();
        J_view.block(9, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacr_r_.data(), 3, nv_).cast<double>();
    } else {
        mj_jacSite(m_, d, jacp_l_.data(), nullptr, id_l_);
        mj_jacSite(m_, d, jacp_r_.data(), nullptr, id_r_);
        J_view.block(0, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacp_l_.data(), 3, nv_).cast<double>();
        J_view.block(3, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacp_r_.data(), 3, nv_).cast<double>();
    }
    
    J_ = J_view;

    // Build mass-weighted Hessian: H = J^T W J + λ M
    // mj_fullM densifies MuJoCo's sparse upper-triangular d->qM (already
    // populated by mj_forward above) into a flat row-major array. M_ is
    // symmetric so the column/row-major mismatch between MuJoCo and Eigen
    // is harmless.
    mj_fullM(m_, M_.data(), d->qM);
    H_ = J_.transpose() * task_weight.asDiagonal() * J_;
    if (cfg.inertia_weight > 0.0)
        H_.noalias() += cfg.inertia_weight * M_;
        
    // Adaptive Damping (scale down damping as we converge to final millimeters)
    double damp_scale = std::min(1.0, result.error / std::max(1e-9, cfg.adaptive_damping_drop_thresh));
    double adaptive_damping = cfg.damping * std::max(cfg.adaptive_damping_min_scale, damp_scale);
    H_.diagonal().array() += adaptive_damping;

    g_ = J_.transpose() * task_weight.asDiagonal() * err_;

    // ── Arm-fold attractive gradient: minimise lUA↔lUB and rUA↔rUB ──────────
    {
        using RowMat = Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

        // Left pair
        Eigen::Vector3d pa = Eigen::Vector3d::Map(d->site_xpos + 3 * id_lUA_);
        Eigen::Vector3d pb = Eigen::Vector3d::Map(d->site_xpos + 3 * id_lUB_);
        Eigen::Vector3d diff = pb - pa;
        double dist = diff.norm();
        if (dist >= 1e-9) {
            Eigen::Vector3d n = diff / dist;
            mj_jacSite(m_, d, jacp_lUA_.data(), nullptr, id_lUA_);
            mj_jacSite(m_, d, jacp_lUB_.data(), nullptr, id_lUB_);
            Eigen::Map<const RowMat> Ja(jacp_lUA_.data(), 3, nv_);
            Eigen::Map<const RowMat> Jb(jacp_lUB_.data(), 3, nv_);
            g_.noalias() += col.arm_dist_weight * dist *
                            (n.transpose().cast<double>() * (Jb - Ja).cast<double>()).transpose();
        }

        // Right pair
        pa   = Eigen::Vector3d::Map(d->site_xpos + 3 * id_rUA_);
        pb   = Eigen::Vector3d::Map(d->site_xpos + 3 * id_rUB_);
        diff = pb - pa;
        dist = diff.norm();
        if (dist >= 1e-9) {
            Eigen::Vector3d n = diff / dist;
            mj_jacSite(m_, d, jacp_rUA_.data(), nullptr, id_rUA_);
            mj_jacSite(m_, d, jacp_rUB_.data(), nullptr, id_rUB_);
            Eigen::Map<const RowMat> Ja(jacp_rUA_.data(), 3, nv_);
            Eigen::Map<const RowMat> Jb(jacp_rUB_.data(), 3, nv_);
            g_.noalias() += col.arm_dist_weight * dist *
                            (n.transpose().cast<double>() * (Jb - Ja).cast<double>()).transpose();
        }
    }

    // ============================================================
    // Setup Extended QP matrices (Variables + Slacks)
    // ============================================================
    int num_slacks = cfg.topk_contacts;
    int n_var = nv_ + num_slacks;
    int n_ineq = nv_ + 2 * num_slacks; // nv (joint bounds) + slacks (CBF) + slacks (slack >= 0)

    if (!qp_ || qp_topk_ != num_slacks) {
        qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(n_var, 0, n_ineq);
        qp_topk_ = num_slacks;
        
        H_ext_.resize(n_var, n_var);
        g_ext_.resize(n_var);
        C_ext_.resize(n_ineq, n_var);
        lb_ext_.resize(n_ineq);
        ub_ext_.resize(n_ineq);
    }

    // 1. Cost Function (Hessian and Gradient)
    H_ext_.setZero();
    H_ext_.topLeftCorner(nv_, nv_) = H_;
    H_ext_.bottomRightCorner(num_slacks, num_slacks) = Eigen::MatrixXd::Identity(num_slacks, num_slacks) * col.slack_penalty;
    
    g_ext_.setZero();
    g_ext_.head(nv_) = g_;
    g_ext_.tail(num_slacks).setConstant(col.slack_penalty); // Linear penalty to push slacks to 0

    // 1b. Dynamic Null-Space Injection
    if (cfg.nullspace_weight > 0.0) {
        nullspace_time_ += cfg.step_size;
        
        // Quadratically fade out the null-space weight as the EE error approaches zero.
        // We use raw, unweighted errors to ensure null-space stays active 
        // if orientation is still far off, even if translation is perfect!
        double max_raw_pos = std::max(err_.segment<3>(0).norm(), err_.segment<3>(6).norm());
        double max_raw_ori = cfg.track_orientation ? std::max(err_.segment<3>(3).norm(), err_.segment<3>(9).norm()) : 0.0;
        
        // 5cm translation or 0.2 rad (~11 deg) orientation keeps it fully active
        double pos_scale = std::min(1.0, (max_raw_pos * max_raw_pos) / (0.05 * 0.05));
        double ori_scale = std::min(1.0, (max_raw_ori * max_raw_ori) / (0.20 * 0.20));
        double scale = std::max(pos_scale, ori_scale);
        
        double active_weight = cfg.nullspace_weight * scale;

        double w_curr = 0.0;
        if (cfg.nullspace_type == 3 && active_weight > 1e-9) {
            mj_kinematics(m_, d);
            w_curr = computeManipulability(m_, d, id_l_, id_r_, nv_, jacp_l_, jacr_l_, jacp_r_, jacr_r_, cfg.track_orientation);
            
            if (manipulability_grad_.size() != nv_) {
                manipulability_grad_ = Eigen::VectorXd::Zero(nv_);
                manipulability_idx_ = 0;
            }
            
            // Interleaved gradient descent: evaluate subset of joints per step
            int group_size = (nv_ + 4) / 5; // roughly divide into 5 groups
            int end_idx = std::min(nv_, manipulability_idx_ + group_size);
            
            for (int i = manipulability_idx_; i < end_idx; ++i) {
                int jid = m_->dof_jntid[i];
                const char* jname = mj_id2name(m_, mjOBJ_JOINT, jid);
                if (jname && strstr(jname, "head") != nullptr) {
                    manipulability_grad_(i) = 0.0;
                    continue;
                }
                
                double delta = 1e-4;
                int q_adr = m_->jnt_qposadr[jid];
                double q_orig = d->qpos[q_adr];
                
                d->qpos[q_adr] += delta;
                mj_kinematics(m_, d);
                // Note: mj_comPos removed for speed
                double w_new = computeManipulability(m_, d, id_l_, id_r_, nv_, jacp_l_, jacr_l_, jacp_r_, jacr_r_, cfg.track_orientation);
                
                d->qpos[q_adr] = q_orig;
                manipulability_grad_(i) = (w_new - w_curr) / delta;
            }
            
            manipulability_idx_ = end_idx;
            if (manipulability_idx_ >= nv_) {
                manipulability_idx_ = 0;
            }
            
            // Restore clean state for the rest of the solver
            mj_kinematics(m_, d);
        }

        // Apply velocities to gradients
        if (active_weight > 1e-9) {
            last_active_nullspace_weight_ = active_weight;
            for (int i = 0; i < nv_; ++i) {
                double v_null_i = 0.0;
                
                if (cfg.nullspace_type == 0) {
                    double phase = (i < (int)nullspace_phase_offsets_.size()) ? nullspace_phase_offsets_[i] : 0.0;
                    double freq_mult = (i < (int)nullspace_frequencies_.size()) ? nullspace_frequencies_[i] : 1.0;
                    double ramp_up = std::min(1.0, nullspace_time_ / (3.0 * cfg.step_size)); // Ramp up over exactly 3 steps
                    v_null_i = ramp_up * cfg.nullspace_amplitude * std::sin(2.0 * M_PI * cfg.nullspace_frequency * freq_mult * nullspace_time_ + phase);
                } else if (cfg.nullspace_type == 1) {
                    int jid = m_->dof_jntid[i];
                    if (m_->jnt_limited[jid]) {
                        double q_min = m_->jnt_range[jid * 2];
                        double q_max = m_->jnt_range[jid * 2 + 1];
                        double q_curr = d->qpos[m_->jnt_qposadr[jid]];
                        double q_mid = (q_min + q_max) / 2.0;
                        double range = q_max - q_min;
                        if (range > 1e-6) {
                            double normalized_err = (q_mid - q_curr) / (range / 2.0);
                            v_null_i = cfg.nullspace_amplitude * normalized_err;
                        }
                    }
                } else if (cfg.nullspace_type == 2) {
                    double phase = (i < (int)nullspace_phase_offsets_.size()) ? nullspace_phase_offsets_[i] : 0.0;
                    double freq_mult = (i < (int)nullspace_frequencies_.size()) ? nullspace_frequencies_[i] : 1.0;
                    double ramp_up = std::min(1.0, nullspace_time_ / (3.0 * cfg.step_size)); // Ramp up over exactly 3 steps
                    v_null_i = -ramp_up * cfg.nullspace_amplitude * std::sin(2.0 * M_PI * cfg.nullspace_frequency * freq_mult * nullspace_time_ + phase);
                } else if (cfg.nullspace_type == 3) {
                    double grad = 0.0;
                    if (manipulability_grad_.size() == nv_) {
                        grad = manipulability_grad_(i);
                    }
                    double ramp_up = std::min(1.0, nullspace_time_ / (3.0 * cfg.step_size));
                    v_null_i = ramp_up * cfg.nullspace_amplitude * grad;
                }
                // Add w * I to Hessian
                H_ext_(i, i) += active_weight;
                // Subtract w * v_null from gradient
                g_ext_(i) -= active_weight * v_null_i;
            }
        }
    } else {
        last_active_nullspace_weight_ = 0.0;
    }

    // 2. Inequality Constraints
    C_ext_.setZero();
    lb_ext_.setConstant(-1e20); // Default no lower bound
    ub_ext_.setConstant(1e20);  // Default no upper bound

    // 2a. Joint velocity bounds (first nv_ rows)
    C_ext_.topLeftCorner(nv_, nv_) = Eigen::MatrixXd::Identity(nv_, nv_);
    // 2b. Joint velocity bound values
    for (int i = 0; i < nv_; ++i) {
        int jid   = m_->dof_jntid[i];
        double min_vel = -cfg.joint_vel_limit;
        double max_vel =  cfg.joint_vel_limit;
        if (m_->jnt_limited[jid]) {
            double q_min  = m_->jnt_range[jid * 2];
            double q_max  = m_->jnt_range[jid * 2 + 1];
            double q_curr = d->qpos[m_->jnt_qposadr[jid]];
            min_vel = std::max(min_vel, (q_min - q_curr) / cfg.step_size);
            max_vel = std::min(max_vel, (q_max - q_curr) / cfg.step_size);
        }

        const char* jname = mj_id2name(m_, mjOBJ_JOINT, jid);
        if (jname) {
            std::string name_str(jname);
            for (const auto& frozen_name : cfg.frozen_joints) {
                if (name_str.find(frozen_name) != std::string::npos) {
                    min_vel = 0.0;
                    max_vel = 0.0;
                    break;
                }
            }
        }

        lb_ext_(i) = min_vel;
        ub_ext_(i) = max_vel;
    }

    // 2c. Control Barrier Functions (CBF) & Slack Non-negativity
    // slack >= 0 (final num_slacks rows)
    C_ext_.bottomRightCorner(num_slacks, num_slacks) = Eigen::MatrixXd::Identity(num_slacks, num_slacks);
    for(int i = 0; i < num_slacks; ++i) {
        lb_ext_(nv_ + num_slacks + i) = 0.0;
    }

    // CBF Constraints: Jdist * dq + slack >= -alpha * (dist - safe_dist)
    int n_within = 0;
    for (const ContactInfo& ci : result.contacts.closest) {
        if (ci.dist > col.collision_margin) continue;
        if ((int)ci.Jdist_row.size() != nv_) continue;
        if (n_within >= num_slacks) break;

        Eigen::VectorXd Jdist = Eigen::Map<const Eigen::Matrix<mjtNum, 1, Eigen::Dynamic>>(ci.Jdist_row.data(), nv_).cast<double>();
        
        C_ext_.block(nv_ + n_within, 0, 1, nv_) = Jdist.transpose();
        C_ext_(nv_ + n_within, nv_ + n_within) = 1.0; // Add slack variable
        
        // Calculate the theoretical repulsion
        double repulsion = -col.cbf_alpha * (ci.dist - col.collision_margin);
        // Cap the repulsion at 0.0. 
        // This transforms the CBF from a "violent bouncy spring" into a "solid wall".
        // It will prevent velocity TOWARD the collision, but will NEVER force the arm to move AWAY.
        lb_ext_(nv_ + n_within) = std::min(repulsion, 0.0);
        
        ++n_within;
    }

    // QP solve
    qp_->settings.verbose = cfg.qp_verbose;

    // -- CBF DEBUG LOGGING --
    for (int i = 0; i < n_within; ++i) {
        double required_bound = lb_ext_(nv_ + i);
        if (required_bound > 0.05) {
            double max_jdist = 0;
            int max_jdist_idx = -1;
            for (int j = 0; j < nv_; ++j) {
                if (std::abs(C_ext_(nv_ + i, j)) > std::abs(max_jdist)) {
                    max_jdist = C_ext_(nv_ + i, j);
                    max_jdist_idx = j;
                }
            }
            std::printf("[IK DEBUG] MASSIVE CBF ACTIVE: constraint %d, bound=%.3f\n", i, required_bound);
            std::printf("   -> Max Jdist is %.3f on jid=%d\n", max_jdist, max_jdist_idx);
        }
    }
    // -----------------------
    qp_->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT;

    // Use a flag to track if we've initialized this specific qp_ instance
    if (!qp_initialized_ || qp_topk_ != num_slacks) {
        qp_->init(H_ext_, g_ext_, std::nullopt, std::nullopt, C_ext_, lb_ext_, ub_ext_);
        qp_initialized_ = true;
    } else {
        qp_->update(H_ext_, g_ext_, std::nullopt, std::nullopt, C_ext_, lb_ext_, ub_ext_);
    }
    qp_->solve();

    // -- DEBUG LOGGING --
    if (qp_->results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
        std::printf("[IK DEBUG] QP Solver failed! Status: %d\n", (int)qp_->results.info.status);
        for(int i = 0; i < nv_; ++i) {
            if (lb_ext_(i) > ub_ext_(i) + 1e-6) {
                std::printf("[IK DEBUG] -> INFEASIBLE JOINT LIMIT: id=%d, min_v=%.3f, max_v=%.3f\n", 
                            i, lb_ext_(i), ub_ext_(i));
            }
        }
    } else {
        double max_v = 0;
        int max_j = -1;
        for(int i = 0; i < nv_; ++i) {
            if (std::abs(qp_->results.x(i)) > max_v) {
                max_v = std::abs(qp_->results.x(i));
                max_j = i;
            }
        }
        // If joint moves more than 0.1 rad in a single step (approx 5.7 degrees)
        if (max_v * cfg.step_size > 0.1) {
            int jnt_id = m_->dof_jntid[max_j];
            const char* jname = mj_id2name(m_, mjOBJ_JOINT, jnt_id);
            double q_curr = d->qpos[m_->jnt_qposadr[jnt_id]];
            double q_min = m_->jnt_limited[jnt_id] ? m_->jnt_range[jnt_id * 2] : -999.0;
            double q_max = m_->jnt_limited[jnt_id] ? m_->jnt_range[jnt_id * 2 + 1] : 999.0;
            
            std::printf("[IK DEBUG] HUGE JUMP: jid=%d (%s), v=%.3f\n", 
                        max_j, jname ? jname : "unknown", max_v);
            std::printf("   -> q_curr: %.3f (limits: [%.3f, %.3f])\n", q_curr, q_min, q_max);
            std::printf("   -> QP Bounds: lb=%.3f, ub=%.3f\n", lb_ext_(max_j), ub_ext_(max_j));
            std::printf("   -> QP Grad: g=%.3f, H_diag=%.3f\n", g_ext_(max_j), H_ext_(max_j, max_j));
        }
    }
    // -------------------

    // Extract only the joint velocities (head of x) and integrate
    mj_integratePos(m_, d->qpos, qp_->results.x.head(nv_).data(), cfg.step_size);
    return result;
}

// ============================================================
// f1 — dual-arm EE-velocity -> joint-velocity diagnostic map
// ============================================================

Eigen::VectorXd IKSolver::dlsMap(const Eigen::MatrixXd& J, const Eigen::VectorXd& v,
                                  double damping, const Eigen::VectorXd& weights) {
    Eigen::MatrixXd JtW = J.transpose() * weights.asDiagonal();
    Eigen::MatrixXd A = JtW * J;
    A.diagonal().array() += damping * damping;
    Eigen::VectorXd rhs = JtW * v;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    if (ldlt.info() == Eigen::Success) {
        Eigen::VectorXd dq = ldlt.solve(rhs);
        if (dq.allFinite()) return dq;
    }
    return Eigen::FullPivLU<Eigen::MatrixXd>(A).solve(rhs);
}

Eigen::VectorXd IKSolver::mapEeTwistToDq(mjData* d, const Eigen::VectorXd& ee_twist,
                                          const SolverConfig& cfg) const {
    if (ee_twist.size() != 12) {
        throw std::invalid_argument(
            "mapEeTwistToDq: ee_twist must be 12D [v_l(3);w_l(3);v_r(3);w_r(3)]");
    }

    // Function-local scratch (not the member jacp_/jacr_ buffers): this call
    // must stay pure/stateless and reentrant, unlike solveStep's warm-started
    // mode-A path.
    std::vector<mjtNum> jacp_l(3 * nv_), jacr_l(3 * nv_);
    std::vector<mjtNum> jacp_r(3 * nv_), jacr_r(3 * nv_);
    mj_jacSite(m_, d, jacp_l.data(), jacr_l.data(), id_l_);
    mj_jacSite(m_, d, jacp_r.data(), jacr_r.data(), id_r_);

    using RowMat3xN = Eigen::Matrix<mjtNum, 3, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::MatrixXd J(12, nv_);
    J.block(0, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacp_l.data(), 3, nv_).cast<double>();
    J.block(3, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacr_l.data(), 3, nv_).cast<double>();
    J.block(6, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacp_r.data(), 3, nv_).cast<double>();
    J.block(9, 0, 3, nv_) = Eigen::Map<const RowMat3xN>(jacr_r.data(), 3, nv_).cast<double>();

    // Row weights mirror solveStep's task weighting (no dynamic "reach then
    // align" modulation — there is no target/error for a one-shot velocity
    // query).
    const double ori_w = cfg.track_orientation ? cfg.ori_weight : 0.0;
    Eigen::VectorXd w(12);
    w.segment<3>(0).setConstant(cfg.pos_weight * cfg.left_weight_scale);
    w.segment<3>(3).setConstant(ori_w * cfg.left_weight_scale);
    w.segment<3>(6).setConstant(cfg.pos_weight * cfg.right_weight_scale);
    w.segment<3>(9).setConstant(ori_w * cfg.right_weight_scale);

    // Zero columns for frozen joints so they never appear in the ranking.
    for (int i = 0; i < nv_; ++i) {
        int jid = m_->dof_jntid[i];
        const char* jname = mj_id2name(m_, mjOBJ_JOINT, jid);
        if (!jname) continue;
        std::string name_str(jname);
        for (const auto& frozen_name : cfg.frozen_joints) {
            if (name_str.find(frozen_name) != std::string::npos) {
                J.col(i).setZero();
                break;
            }
        }
    }

    return dlsMap(J, ee_twist, cfg.damping, w);
}

// ============================================================
// f2 — bounded push-out to the nearest collision-safe pose
// ============================================================

bool IKSolver::buildClearQP(mjData* d, const SolverConfig& cfg, const CollisionCostConfig& col,
                             const ContactResult& contacts, double margin, double band,
                             double h, Eigen::VectorXd& dq_out) {
    // Active set: prefix scan (contacts are ascending by dist) of contacts
    // within the band, with a usable Jacobian row.
    int n_active = 0;
    for (const ContactInfo& ci : contacts.closest) {
        if (ci.dist > margin + band) break;
        if ((int)ci.Jdist_row.size() != nv_) continue;
        ++n_active;
    }
    if (n_active <= 0) return false;

    const int n_var = nv_ + n_active;
    const int n_ineq = nv_ + 2 * n_active;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_var, n_var);
    H.topLeftCorner(nv_, nv_) = Eigen::MatrixXd::Identity(nv_, nv_); // pure least-norm displacement
    H.bottomRightCorner(n_active, n_active) =
        Eigen::MatrixXd::Identity(n_active, n_active) * col.slack_penalty;

    Eigen::VectorXd g = Eigen::VectorXd::Zero(n_var); // no EE task, no nullspace, no attractor

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n_ineq, n_var);
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_ineq, -1e20);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_ineq, 1e20);

    // Joint-bound rows: solveStep's arithmetic (558-587), with h in place of
    // cfg.step_size; frozen joints get fixed (min=max=0) rows.
    C.topLeftCorner(nv_, nv_) = Eigen::MatrixXd::Identity(nv_, nv_);
    for (int i = 0; i < nv_; ++i) {
        int jid = m_->dof_jntid[i];
        double min_vel = -cfg.joint_vel_limit;
        double max_vel =  cfg.joint_vel_limit;
        if (m_->jnt_limited[jid]) {
            double q_min  = m_->jnt_range[jid * 2];
            double q_max  = m_->jnt_range[jid * 2 + 1];
            double q_curr = d->qpos[m_->jnt_qposadr[jid]];
            min_vel = std::max(min_vel, (q_min - q_curr) / h);
            max_vel = std::min(max_vel, (q_max - q_curr) / h);
        }

        const char* jname = mj_id2name(m_, mjOBJ_JOINT, jid);
        if (jname) {
            std::string name_str(jname);
            for (const auto& frozen_name : cfg.frozen_joints) {
                if (name_str.find(frozen_name) != std::string::npos) {
                    min_vel = 0.0;
                    max_vel = 0.0;
                    break;
                }
            }
        }
        lb(i) = min_vel;
        ub(i) = max_vel;
    }

    // Slack >= 0 rows.
    C.bottomRightCorner(n_active, n_active) = Eigen::MatrixXd::Identity(n_active, n_active);
    for (int k = 0; k < n_active; ++k) lb(nv_ + n_active + k) = 0.0;

    // CBF rows — same form as solveStep (599-616), but UNCAPPED: a violator
    // (dist < margin) gets a positive lb forcing separation at
    // cbf_alpha*(margin-dist); a near-safe band contact (margin <= dist <=
    // margin+band) gets lb = 0, a wall that stops the push-out from trading
    // one violation for another. This is the one-line difference from
    // solveStep's std::min(repulsion, 0.0) "solid wall, never force away".
    int k = 0;
    for (const ContactInfo& ci : contacts.closest) {
        if (ci.dist > margin + band) break;
        if ((int)ci.Jdist_row.size() != nv_) continue;

        Eigen::VectorXd Jdist =
            Eigen::Map<const Eigen::Matrix<mjtNum, 1, Eigen::Dynamic>>(ci.Jdist_row.data(), nv_)
                .cast<double>();
        C.block(nv_ + k, 0, 1, nv_) = Jdist.transpose();
        C(nv_ + k, nv_ + k) = 1.0;

        double repulsion = -col.cbf_alpha * (ci.dist - margin);
        lb(nv_ + k) = std::max(repulsion, 0.0);
        ++k;
    }

    proxsuite::proxqp::dense::QP<double> qp(n_var, 0, n_ineq);
    qp.settings.verbose = cfg.qp_verbose;
    qp.init(H, g, std::nullopt, std::nullopt, C, lb, ub);
    qp.solve();

    if (qp.results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
        return false;

    dq_out = qp.results.x.head(nv_);
    return true;
}

bool IKSolver::clearToMargin(mjData* d, const SolverConfig& cfg, const CollisionCostConfig& col,
                              int max_steps, double* final_min_dist) {
    const double h = std::min(0.1, std::max(1e-3, cfg.step_size));
    constexpr double tol = 1e-3;
    constexpr double band = 0.02;
    constexpr double stall_eps = 1e-4;  // min per-step improvement to not count as a plateau
    constexpr int stall_patience = 3;   // consecutive plateaued steps before early exit

    double prev_dist = -std::numeric_limits<double>::infinity();
    int stalled_steps = 0;

    for (int step = 0; step < max_steps; ++step) {
        mj_forward(m_, d);
        ContactResult all;
        computeContacts(d, std::max(1, d->ncon), all);

        double min_dist = all.closest.empty() ? 0.30 : all.closest.front().dist;
        if (final_min_dist) *final_min_dist = min_dist;

        if (all.closest.empty() || min_dist >= col.collision_margin - tol)
            return true;

        // Plateau check: don't grind out the full step budget once progress
        // stalls (e.g. QP-infeasible corner) — hand back to the caller early
        // so it can fall back (interpolation toward a known-safe pose)
        // instead of burning the tick on a correction that won't converge.
        if (min_dist - prev_dist < stall_eps) {
            if (++stalled_steps >= stall_patience) return false;
        } else {
            stalled_steps = 0;
        }
        prev_dist = min_dist;

        Eigen::VectorXd dq;
        if (!buildClearQP(d, cfg, col, all, col.collision_margin, band, h, dq))
            return false;
        mj_integratePos(m_, d->qpos, dq.data(), h);
    }

    mj_forward(m_, d);
    ContactResult all;
    computeContacts(d, std::max(1, d->ncon), all);
    double min_dist = all.closest.empty() ? 0.30 : all.closest.front().dist;
    if (final_min_dist) *final_min_dist = min_dist;
    return all.closest.empty() || min_dist >= col.collision_margin - tol;
}

// ============================================================
// Logging
// ============================================================

void IKSolver::printStep(int step, const StepResult& r) const {
    std::printf(
        "Step %3d | error=%.6f | obj=%.6f (ee=%.6f coll=%.6f)"
        " | contacts=%d | min_dist=%.6f\n",
        step,
        r.error,
        r.objective_total,
        r.objective_ee,
        r.objective_collision,
        r.contacts.total_contacts,
        std::isfinite(r.min_dist) ? r.min_dist : -1.0);
}

// ============================================================
// Kinematic queries
// ============================================================

std::string IKSolver::getKinematicTree(mjData *d) const {
    std::stringstream ss;
    ss << "=== MuJoCo Kinematic Tree ===\n";
    for (int i = 1; i < m_->nbody; ++i) { // skip worldbody (0)
        const char* name = mj_id2name(m_, mjOBJ_BODY, i);
        if (!name) continue;
        
        Eigen::Vector3d pos = Eigen::Vector3d::Map(d->xpos + 3 * i);
        Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> rot(d->xmat + 9 * i);
        
        // Convert to euler for readability
        double pitch = std::asin(std::clamp(rot(0, 2), -1.0, 1.0));
        double yaw = std::atan2(-rot(0, 1), rot(0, 0));
        double roll = std::atan2(-rot(1, 2), rot(2, 2));

        ss << "Body [" << i << "]: " << name << "\n";
        ss << "  Pos (XYZ): " << pos.x() << ", " << pos.y() << ", " << pos.z() << "\n";
        ss << "  Rot (RPY): " << roll << ", " << pitch << ", " << yaw << "\n";
    }
    return ss.str();
}

std::vector<std::string> IKSolver::getJointNames() const {
    std::vector<std::string> names;
    for (int i = 0; i < nv_; ++i) {
        int jid = m_->dof_jntid[i];
        const char* jname = mj_id2name(m_, mjOBJ_JOINT, jid);
        if (jname) {
            std::string name_str(jname);
            if (std::find(names.begin(), names.end(), name_str) == names.end()) {
                names.push_back(name_str);
            }
        }
    }
    return names;
}

// ============================================================
// MultiIKSolver Implementation
// ============================================================

MultiIKSolver::MultiIKSolver(mjModel *m)
    : m_(m), current_best_idx_(0) {
    solver0_ = std::make_unique<IKSolver>(m_);
    solver1_ = std::make_unique<IKSolver>(m_);
    solver2_ = std::make_unique<IKSolver>(m_);
    d0_ = mj_makeData(m_);
    d1_ = mj_makeData(m_);
    d2_ = mj_makeData(m_);
}

MultiIKSolver::~MultiIKSolver() {
    mj_deleteData(d0_);
    mj_deleteData(d1_);
    mj_deleteData(d2_);
}

StepResult MultiIKSolver::solveStepMulti(mjData *d, const Eigen::Isometry3d &target_l,
                                         const Eigen::Isometry3d &target_r, 
                                         const SolverConfig &base_cfg, 
                                         const CollisionCostConfig &col_cfg,
                                         std::deque<double> &err_hist,
                                         std::deque<double> &dist_hist) {
    mj_copyData(d0_, m_, d);
    mj_copyData(d1_, m_, d);
    mj_copyData(d2_, m_, d);

    SolverConfig cfg0 = base_cfg; cfg0.nullspace_type = 0;
    SolverConfig cfg1 = base_cfg; cfg1.nullspace_type = 1;
    SolverConfig cfg2 = base_cfg; cfg2.nullspace_type = 2; // Anti-Sinusoidal

    std::deque<double> err0 = err_hist, err1 = err_hist, err2 = err_hist;
    std::deque<double> dist0 = dist_hist, dist1 = dist_hist, dist2 = dist_hist;

    auto f0 = std::async(std::launch::async, [&]() {
        return solver0_->solveStep(d0_, target_l, target_r, cfg0, col_cfg, err0, dist0);
    });
    auto f1 = std::async(std::launch::async, [&]() {
        return solver1_->solveStep(d1_, target_l, target_r, cfg1, col_cfg, err1, dist1);
    });
    auto f2 = std::async(std::launch::async, [&]() {
        return solver2_->solveStep(d2_, target_l, target_r, cfg2, col_cfg, err2, dist2);
    });

    StepResult r0 = f0.get();
    StepResult r1 = f1.get();
    StepResult r2 = f2.get();

    auto evaluate_error = [&](mjData* data) {
        int left_id = mj_name2id(m_, mjOBJ_SITE, "left_gripper_site");
        int right_id = mj_name2id(m_, mjOBJ_SITE, "right_gripper_site");
        
        double total = 0.0;
        if (left_id >= 0) {
            Eigen::Isometry3d curr_l = Eigen::Isometry3d::Identity();
            curr_l.translation() = Eigen::Vector3d::Map(data->site_xpos + 3 * left_id);
            curr_l.linear() = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(data->site_xmat + 9 * left_id).cast<double>();
            total += (curr_l.translation() - target_l.translation()).norm();
            Eigen::AngleAxisd aa_l(curr_l.linear().transpose() * target_l.linear());
            total += aa_l.angle();
        }
        if (right_id >= 0) {
            Eigen::Isometry3d curr_r = Eigen::Isometry3d::Identity();
            curr_r.translation() = Eigen::Vector3d::Map(data->site_xpos + 3 * right_id);
            curr_r.linear() = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(data->site_xmat + 9 * right_id).cast<double>();
            total += (curr_r.translation() - target_r.translation()).norm();
            Eigen::AngleAxisd aa_r(curr_r.linear().transpose() * target_r.linear());
            total += aa_r.angle();
        }
        return total;
    };

    double e0 = evaluate_error(d0_);
    double e1 = evaluate_error(d1_);
    double e2 = evaluate_error(d2_);

    double errors[3] = {e0, e1, e2};
    int best_idx = current_best_idx_;
    double best_err = errors[current_best_idx_];

    for (int i = 0; i < 3; i++) {
        // Use hysteresis of 1e-4 rad/m to prevent floating point jitter
        if (i != current_best_idx_ && errors[i] < best_err - 1e-4) {
            best_idx = i;
            best_err = errors[i];
        }
    }

    current_best_idx_ = best_idx;
    StepResult best_res = r0;

    if (best_idx == 0) {
        mj_copyData(d, m_, d0_);
        err_hist = err0; dist_hist = dist0;
        best_res = r0;
    } else if (best_idx == 1) {
        mj_copyData(d, m_, d1_);
        err_hist = err1; dist_hist = dist1;
        best_res = r1;
    } else {
        mj_copyData(d, m_, d2_);
        err_hist = err2; dist_hist = dist2;
        best_res = r2;
    }

    return best_res;
}

} // namespace ffw_ik