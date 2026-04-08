#include "ffw_ik_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <stdexcept>

namespace ffw_ik {

// ============================================================
// Construction
// ============================================================

IKSolver::IKSolver(mjModel* model)
    : m_(model)
    , nv_(model->nv)
    , qp_(model->nv, 0, model->nv)
{
    J_.resize(6, nv_);
    H_.resize(nv_, nv_);
    g_.resize(nv_);
    err_.resize(6);
    C_ = Eigen::MatrixXd::Identity(nv_, nv_);
    lb_.resize(nv_);
    ub_.resize(nv_);

    jacp_l_.resize(3 * nv_);
    jacp_r_.resize(3 * nv_);
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

    id_l_ = mj_name2id(m_, mjOBJ_SITE, "left_gripper_site");
    id_r_ = mj_name2id(m_, mjOBJ_SITE, "right_gripper_site");
    id_lUA_ = mj_name2id(m_, mjOBJ_SITE, "left_UA_site");
    id_lUB_ = mj_name2id(m_, mjOBJ_SITE, "left_UB_site");
    id_rUA_ = mj_name2id(m_, mjOBJ_SITE, "right_UA_site");
    id_rUB_ = mj_name2id(m_, mjOBJ_SITE, "right_UB_site");
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
    const Eigen::Vector3d&     target_l,
    const Eigen::Vector3d&     target_r,
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
    err_.segment<3>(0) = pos_l - target_l;
    err_.segment<3>(3) = pos_r - target_r;
    result.error = err_.norm();

    // Contact query
    computeContacts(d, cfg.topk_contacts, result.contacts);
    result.min_dist = result.contacts.closest.empty()
        ? std::numeric_limits<double>::quiet_NaN()
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
    mj_jacSite(m_, d, jacp_l_.data(), nullptr, id_l_);
    mj_jacSite(m_, d, jacp_r_.data(), nullptr, id_r_);
    J_.block(0, 0, 3, nv_) =
        Eigen::Map<const RowMat3xN>(jacp_l_.data(), 3, nv_).cast<double>();
    J_.block(3, 0, 3, nv_) =
        Eigen::Map<const RowMat3xN>(jacp_r_.data(), 3, nv_).cast<double>();

    H_ = J_.transpose() * J_;
    H_.diagonal().array() += cfg.damping;
    g_ = J_.transpose() * err_;

    // Collision repulsion gradient
    int n_within = 0;
    for (const ContactInfo& ci : result.contacts.closest) {
        if (ci.dist > col.collision_margin)    continue;
        if ((int)ci.Jdist_row.size() != nv_)  continue;

        Jdist_all_.row(n_within) =
            Eigen::Map<const Eigen::Matrix<mjtNum, 1, Eigen::Dynamic>>(
                ci.Jdist_row.data(), nv_).cast<double>();
        dist_all_(n_within) = ci.dist;
        w_all_(n_within)    = col.weight_scale / (ci.dist * ci.dist + col.epsilon);
        ++n_within;
    }

    if (n_within > 0) {
        g_ -= Jdist_all_.topRows(n_within).transpose() *
              w_all_.head(n_within).asDiagonal() *
              dist_all_.head(n_within);
    }

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
            g_ += col.arm_dist_weight * dist *
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
            g_ += col.arm_dist_weight * dist *
                  (n.transpose().cast<double>() * (Jb - Ja).cast<double>()).transpose();
        }
    }

    // Joint velocity bounds from joint limits
    for (int i = 0; i < nv_; ++i) {
        int jid   = m_->dof_jntid[i];
        lb_[i]    = -cfg.joint_vel_limit;
        ub_[i]    =  cfg.joint_vel_limit;
        if (m_->jnt_limited[jid]) {
            double q_min  = m_->jnt_range[jid * 2];
            double q_max  = m_->jnt_range[jid * 2 + 1];
            double q_curr = d->qpos[m_->jnt_qposadr[jid]];
            lb_[i] = std::max(lb_[i], (q_min - q_curr) / cfg.step_size);
            ub_[i] = std::min(ub_[i], (q_max - q_curr) / cfg.step_size);
        }
    }
    
    // QP solve (warm-started after first call)
    qp_.settings.verbose = cfg.qp_verbose;
    if (!qp_initialized_) {
        qp_.init(H_, g_, std::nullopt, std::nullopt, C_, lb_, ub_);
        qp_initialized_ = true;
    } else {
        qp_.update(H_, g_, std::nullopt, std::nullopt, C_, lb_, ub_);
    }
    qp_.solve();

    mj_integratePos(m_, d->qpos, qp_.results.x.data(), cfg.step_size);
    return result;
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

} // namespace ffw_ik