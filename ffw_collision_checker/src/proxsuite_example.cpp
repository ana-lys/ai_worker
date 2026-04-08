#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <deque>
#include <random>
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ---------------------------------------------------------------------------
// SimpleViewer
// ---------------------------------------------------------------------------
class SimpleViewer {
public:
    explicit SimpleViewer(mjModel* model) : m_(model) {
        if (!glfwInit()) {
            std::cerr << "[Viewer] GLFW init failed. Running headless." << std::endl;
            return;
        }
        window_ = glfwCreateWindow(1200, 900, "IK Step Viewer", nullptr, nullptr);
        if (!window_) {
            std::cerr << "[Viewer] GLFW window creation failed. Running headless." << std::endl;
            glfwTerminate();
            return;
        }
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        mjv_defaultCamera(&cam_);
        cam_.distance  = 3.0;
        cam_.azimuth   = 0.0;
        cam_.elevation = -20.0;
        cam_.type      = mjCAMERA_FREE;
        cam_.lookat[0] = 0.0;
        cam_.lookat[1] = 0.0;
        cam_.lookat[2] = 1.0;

        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);
        enabled_ = true;
    }

    ~SimpleViewer() {
        if (enabled_) {
            mjv_freeScene(&scn_);
            mjr_freeContext(&con_);
        }
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }

    bool render(mjData* d) {
        if (!enabled_) return true;
        glfwPollEvents();
        if (glfwWindowShouldClose(window_)) return false;

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
        mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

        if (debug_spheres_enabled_ && scn_.ngeom + 1 < scn_.maxgeom) {
            const mjtNum size[3] = {debug_sphere_radius_, debug_sphere_radius_, debug_sphere_radius_};
            const mjtNum mat[9]  = {1,0,0, 0,1,0, 0,0,1};

            mjvGeom* gl = &scn_.geoms[scn_.ngeom];
            mjv_initGeom(gl, mjGEOM_SPHERE, size, debug_sphere_pos_left_, mat, debug_sphere_rgba_);
            gl->category = mjCAT_DECOR;
            scn_.ngeom++;

            mjvGeom* gr = &scn_.geoms[scn_.ngeom];
            mjv_initGeom(gr, mjGEOM_SPHERE, size, debug_sphere_pos_right_, mat, debug_sphere_rgba_);
            gr->category = mjCAT_DECOR;
            scn_.ngeom++;
        }

        mjr_render(viewport, &scn_, &con_);
        glfwSwapBuffers(window_);
        return true;
    }

    bool enabled() const { return enabled_; }

    void setDebugSpheres(const Eigen::Vector3d& left_pos,
                         const Eigen::Vector3d& right_pos,
                         double diameter = 0.1,
                         float r = 1.0f, float g = 0.5f,
                         float b = 0.0f, float a = 0.8f) {
        debug_spheres_enabled_ = true;
        for (int i = 0; i < 3; ++i) {
            debug_sphere_pos_left_[i]  = left_pos[i];
            debug_sphere_pos_right_[i] = right_pos[i];
        }
        debug_sphere_radius_   = std::max(0.0, 0.5 * diameter);
        debug_sphere_rgba_[0]  = r;
        debug_sphere_rgba_[1]  = g;
        debug_sphere_rgba_[2]  = b;
        debug_sphere_rgba_[3]  = a;
    }

private:
    mjModel*    m_      = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera   cam_;
    mjvOption   opt_;
    mjvScene    scn_;
    mjrContext  con_;
    bool        enabled_               = false;
    bool        debug_spheres_enabled_ = false;
    mjtNum      debug_sphere_pos_left_[3]  = {};
    mjtNum      debug_sphere_pos_right_[3] = {};
    mjtNum      debug_sphere_radius_       = 0.05;
    float       debug_sphere_rgba_[4]      = {0.2f, 0.9f, 0.2f, 0.8f};
};

// ---------------------------------------------------------------------------
// IKSolver
// ---------------------------------------------------------------------------
class IKSolver {
public:
    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------
    struct SolverConfig {
        double damping           = 4e-3;
        double step_size         = 0.2;
        double tolerance         = 5e-3;
        double joint_vel_limit   = 3.1;
        bool   qp_verbose        = false;

        // topk contacts evaluated per step (was buried in DebugConfig before)
        int    topk_contacts          = 5;

        // Early-stop: sliding window on EE error rate of improvement
        int    ee_window              = 5;
        double ee_improvement_rate    = 0.02;

        // Early-stop: min_dist stability
        int    dist_window            = 5;
        double dist_stability_thresh  = 0.002;
        double dist_safe_ratio        = 0.98;

        double early_convergence_obj  = 0.87;

        int    max_steps         = 100;
    };

    struct CollisionCostConfig {
        double collision_margin = 0.155;
        double weight_scale     = 0.01;
        double epsilon          = 1e-1;
    };

    struct DebugConfig {
        bool   print_summary          = true;
        bool   visualize_goal_spheres = false;
        double goal_sphere_diameter   = 0.05;
        int    frame_sleep_ms         = 0;
    };

    // ------------------------------------------------------------------
    // Data types
    // ------------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------
    // FIX (inefficiency): QP object created once here rather than every step.
    // nv_ inequality constraints come from per-DOF box bounds (C = I).
    explicit IKSolver(mjModel* model)
        : m_(model)
        , nv_(model->nv)
        , qp_(model->nv, 0, model->nv)   // (n_vars, n_eq, n_ineq)
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

        // FIX (inefficiency): pre-size to worst case so no per-step allocations.
        // model->nconmax == -1 means "unlimited" in MuJoCo — guard against it.
        // In practice topk_contacts (default 5) is the real ceiling on rows we
        // ever fill, so 512 is a safe, generous upper bound for scratch_.
        const int max_contacts = (model->nconmax > 0) ? model->nconmax : 512;
        Jdist_all_.resize(max_contacts, nv_);
        dist_all_.resize(max_contacts);
        w_all_.resize(max_contacts);

        // FIX (inefficiency): reserve scratch_ once.
        scratch_.reserve(max_contacts);

        id_l_ = mj_name2id(m_, mjOBJ_SITE, "left_gripper_site");
        id_r_ = mj_name2id(m_, mjOBJ_SITE, "right_gripper_site");
        if (id_l_ == -1 || id_r_ == -1)
            std::cerr << "[IKSolver] Warning: gripper sites not found.\n";
    }

    // ------------------------------------------------------------------
    // Objective functions
    // ------------------------------------------------------------------
    static double eeObjective(double error) {
        return 0.9 * std::exp(-std::max(0.0, error) / 0.5);
    }

    static double collisionObjective(double min_dist) {
        if (!std::isfinite(min_dist)) return 0.0;
        return 0.1 * (1.0 - std::exp(-std::max(0.0, min_dist) / 0.05));
    }

    // ------------------------------------------------------------------
    // Contact query
    // ------------------------------------------------------------------
    void computeContacts(mjData* d, int topk, ContactResult& out) {
        out.total_contacts = d ? d->ncon : 0;
        out.closest.clear();
        if (!d || d->ncon <= 0 || topk <= 0) return;

        const int k = std::min(topk, d->ncon);

        // FIX (inefficiency): scratch_ was resized every call; now just filled.
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
                Eigen::Map<Eigen::Matrix<mjtNum,1,Eigen::Dynamic>>(info.Jdist_row.data(), nv_) = Jdist;
            }
            out.closest.push_back(std::move(info));
        }
    }

    // ------------------------------------------------------------------
    // One IK step
    // ------------------------------------------------------------------
    StepResult solveStep(mjData* d,
                         const Eigen::Vector3d& target_l,
                         const Eigen::Vector3d& target_r,
                         const SolverConfig& cfg,
                         const CollisionCostConfig& col_cfg,
                         std::deque<double>& error_history,
                         std::deque<double>& dist_history) {
        StepResult result;

        mj_forward(m_, d);

        // EE error
        Eigen::Vector3d pos_l = Eigen::Vector3d::Map(d->site_xpos + 3 * id_l_);
        Eigen::Vector3d pos_r = Eigen::Vector3d::Map(d->site_xpos + 3 * id_r_);
        err_.segment<3>(0) = pos_l - target_l;
        err_.segment<3>(3) = pos_r - target_r;
        result.error = err_.norm();

        // Contacts — topk now driven by SolverConfig (FIX: was hardcoded 5)
        computeContacts(d, cfg.topk_contacts, result.contacts);
        result.min_dist = result.contacts.closest.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : result.contacts.closest.front().dist;

        result.objective_ee        = eeObjective(result.error);
        result.objective_collision = collisionObjective(result.min_dist);
        result.objective_total     = result.objective_ee + result.objective_collision;

        // ----------------------------------------------------------------
        // Hard convergence
        // ----------------------------------------------------------------
        if (result.error < cfg.tolerance) {
            result.converged = true;
            return result;
        }

        // ----------------------------------------------------------------
        // Sliding-window early stop
        // ----------------------------------------------------------------
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
                // No contacts: collision objective is at its ideal constant value.
                result.objective_collision = 0.1;
                result.min_dist            = col_cfg.collision_margin;
                // FIX (Bug 2): objective_total was stale after patching the two
                // sub-objectives above. Recompute it for a consistent StepResult.
                result.objective_total = result.objective_ee + result.objective_collision;
                dist_stable_and_safe   = true;
            } else if ((int)dist_history.size() == cfg.dist_window) {
                double mean = 0.0;
                for (double v : dist_history) mean += v;
                mean /= dist_history.size();
                double var = 0.0;
                for (double v : dist_history) var += (v - mean) * (v - mean);
                var /= dist_history.size();
                bool stable = var < cfg.dist_stability_thresh;
                bool safe   = std::isfinite(result.min_dist) &&
                              result.min_dist > col_cfg.collision_margin * cfg.dist_safe_ratio;
                dist_stable_and_safe = stable && safe;
            }

            if (ee_stalled && dist_stable_and_safe) {
                if (result.objective_total > cfg.early_convergence_obj)
                    result.early_converged = true;
                else
                    result.stalled = true;
                return result;
            }
        }

        // ----------------------------------------------------------------
        // Build EE Jacobian
        // ----------------------------------------------------------------
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

        // ----------------------------------------------------------------
        // Collect contacts within margin and add gradient repulsion term
        // ----------------------------------------------------------------
        int n_within = 0;
        for (const ContactInfo& ci : result.contacts.closest) {
            if (ci.dist > col_cfg.collision_margin)   continue;
            if ((int)ci.Jdist_row.size() != nv_)      continue;

            // FIX (inefficiency): write directly into pre-allocated members
            Jdist_all_.row(n_within) =
                Eigen::Map<const Eigen::Matrix<mjtNum,1,Eigen::Dynamic>>(
                    ci.Jdist_row.data(), nv_).cast<double>();
            dist_all_(n_within) = ci.dist;
            w_all_(n_within)    = col_cfg.weight_scale /
                                   (ci.dist * ci.dist + col_cfg.epsilon);
            ++n_within;
        }

        if (n_within > 0) {
            g_ -= Jdist_all_.topRows(n_within).transpose() *
                  w_all_.head(n_within).asDiagonal() *
                  dist_all_.head(n_within);
        }

        // ----------------------------------------------------------------
        // Joint velocity bounds from joint limits
        // ----------------------------------------------------------------
        for (int i = 0; i < nv_; ++i) {
            int jid = m_->dof_jntid[i];
            lb_[i]  = -cfg.joint_vel_limit;
            ub_[i]  =  cfg.joint_vel_limit;
            if (m_->jnt_limited[jid]) {
                double q_min  = m_->jnt_range[jid * 2];
                double q_max  = m_->jnt_range[jid * 2 + 1];
                double q_curr = d->qpos[m_->jnt_qposadr[jid]];
                lb_[i] = std::max(lb_[i], (q_min - q_curr) / cfg.step_size);
                ub_[i] = std::min(ub_[i], (q_max - q_curr) / cfg.step_size);
            }
        }

        // ----------------------------------------------------------------
        // Solve QP
        // ----------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // Debug print
    // ------------------------------------------------------------------
    void printStep(int step, const StepResult& r) const {
        std::printf(
            "Step %3d | Error: %.6f | Obj: %.6f (ee=%.6f coll=%.6f)"
            " | Contacts: %d | min_dist: %.6f\n",
            step,
            r.error,
            r.objective_total,
            r.objective_ee,
            r.objective_collision,
            r.contacts.total_contacts,
            std::isfinite(r.min_dist) ? r.min_dist : -1.0);
    }

private:
    mjModel*        m_;
    int             nv_, id_l_, id_r_;
    Eigen::MatrixXd J_, H_, C_;
    Eigen::VectorXd g_, err_, lb_, ub_;

    // Pre-allocated collision arrays (FIX: were local per-step in solveStep)
    Eigen::MatrixXd Jdist_all_;
    Eigen::VectorXd dist_all_, w_all_;

    std::vector<mjtNum> jacp_l_, jacp_r_, jacp_c1_, jacp_c2_;
    std::vector<int>    scratch_;

    // Cached QP — allocated once at construction, warm-started each solve
    proxsuite::proxqp::dense::QP<double> qp_;
    bool qp_initialized_ = false;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {

double computeMinContactDistance(const mjData* d) {
    if (!d || d->ncon <= 0)
        return std::numeric_limits<double>::infinity();
    double min_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < d->ncon; ++i)
        min_dist = std::min(min_dist, static_cast<double>(d->contact[i].dist));
    return min_dist;
}

bool isPoseLegalByCheck(mjModel* m,
                        mjData* d,
                        const Eigen::VectorXd& q,
                        double min_clearance,
                        double* out_min_dist = nullptr) {
    std::vector<mjtNum> qpos_backup(m->nq);
    std::vector<mjtNum> qvel_backup(m->nv);
    mju_copy(qpos_backup.data(), d->qpos, m->nq);
    mju_copy(qvel_backup.data(), d->qvel, m->nv);

    mju_copy(d->qpos, q.data(), m->nq);
    mju_zero(d->qvel, m->nv);
    mj_forward(m, d);

    const double min_dist = computeMinContactDistance(d);
    const bool legal = !std::isfinite(min_dist) || min_dist >= min_clearance;
    if (out_min_dist) *out_min_dist = min_dist;

    mju_copy(d->qpos, qpos_backup.data(), m->nq);
    mju_copy(d->qvel, qvel_backup.data(), m->nv);
    mj_forward(m, d);
    return legal;
}

Eigen::VectorXd sampleRandomPose(mjModel* m,
                                 const Eigen::VectorXd& ref_q,
                                 std::mt19937& rng) {
    Eigen::VectorXd q = ref_q;
    for (int jid = 0; jid < m->njnt; ++jid) {
        const int jtype = m->jnt_type[jid];
        const int qadr  = m->jnt_qposadr[jid];

        if (jtype == mjJNT_HINGE || jtype == mjJNT_SLIDE) {
            double lower, upper;
            if (m->jnt_limited[jid]) {
                lower = m->jnt_range[2 * jid];
                upper = m->jnt_range[2 * jid + 1];
                const double span = upper - lower;
                if (span > 1e-6) {
                    const double margin = 0.02 * span;
                    lower += margin;
                    upper -= margin;
                }
                if (lower > upper) {
                    lower = m->jnt_range[2 * jid];
                    upper = m->jnt_range[2 * jid + 1];
                }
            } else {
                lower = (jtype == mjJNT_HINGE)
                    ? ref_q[qadr] - M_PI  : ref_q[qadr] - 0.25;
                upper = (jtype == mjJNT_HINGE)
                    ? ref_q[qadr] + M_PI  : ref_q[qadr] + 0.25;
            }
            std::uniform_real_distribution<double> dist(lower, upper);
            q[qadr] = dist(rng);
        }
    }
    return q;
}

bool findRandomLegalPose(mjModel* m,
                         mjData* d,
                         const Eigen::VectorXd& ref_q,
                         std::mt19937& rng,
                         double min_clearance,
                         int max_tries,
                         Eigen::VectorXd& out_q,
                         double& out_min_dist) {
    for (int attempt = 0; attempt < max_tries; ++attempt) {
        Eigen::VectorXd candidate = sampleRandomPose(m, ref_q, rng);
        double min_dist = std::numeric_limits<double>::infinity();
        if (isPoseLegalByCheck(m, d, candidate, min_clearance, &min_dist)) {
            out_q       = std::move(candidate);
            out_min_dist = min_dist;
            return true;
        }
    }
    return false;
}

bool computeGripperSitePositionsAtPose(mjModel* m,
                                       mjData* d,
                                       const Eigen::VectorXd& q,
                                       int left_site_id,
                                       int right_site_id,
                                       Eigen::Vector3d& out_left,
                                       Eigen::Vector3d& out_right) {
    if (left_site_id < 0 || right_site_id < 0) return false;

    std::vector<mjtNum> qpos_backup(m->nq);
    std::vector<mjtNum> qvel_backup(m->nv);
    mju_copy(qpos_backup.data(), d->qpos, m->nq);
    mju_copy(qvel_backup.data(), d->qvel, m->nv);

    mju_copy(d->qpos, q.data(), m->nq);
    mju_zero(d->qvel, m->nv);
    mj_forward(m, d);

    out_left  = Eigen::Vector3d::Map(d->site_xpos + 3 * left_site_id);
    out_right = Eigen::Vector3d::Map(d->site_xpos + 3 * right_site_id);

    mju_copy(d->qpos, qpos_backup.data(), m->nq);
    mju_copy(d->qvel, qvel_backup.data(), m->nv);
    mj_forward(m, d);
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    (void)argc; (void)argv;

    using ament_index_cpp::get_package_share_directory;
    std::string xml_path = get_package_share_directory("ffw_collision_checker")
                         + "/3rd_party/robotis_ffw/scene_inverse_kinematic.xml";

    char error[1000];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!m) { std::cerr << "MuJoCo load error: " << error << "\n"; return 1; }
    mjData* d = mj_makeData(m);

    mju_zero(d->qpos, m->nq);
    mju_zero(d->qvel, m->nv);
    if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE) d->qpos[3] = 1.0;
    mj_forward(m, d);

    IKSolver     solver(m);
    SimpleViewer viewer(m);

    IKSolver::SolverConfig solver_cfg;
    solver_cfg.damping               = 1e-3;
    solver_cfg.step_size             = 0.15;
    solver_cfg.tolerance             = 5e-3;
    solver_cfg.joint_vel_limit       = 3.1;
    solver_cfg.max_steps             = 100;
    solver_cfg.topk_contacts         = 5;  // now lives here, not in DebugConfig
    solver_cfg.ee_window             = 5;
    solver_cfg.ee_improvement_rate   = 0.02;
    solver_cfg.dist_window           = 5;
    solver_cfg.dist_stability_thresh = 0.002;
    solver_cfg.dist_safe_ratio       = 0.98;
    solver_cfg.early_convergence_obj = 0.87;

    IKSolver::CollisionCostConfig col_cfg;
    col_cfg.collision_margin = 0.10;  // lowered from 0.155 — see kMinClearanceMeters note
    col_cfg.weight_scale     = 0.005;
    col_cfg.epsilon          = 1e-1;

    IKSolver::DebugConfig dbg_cfg;
    dbg_cfg.print_summary          = true;
    dbg_cfg.visualize_goal_spheres = true;
    dbg_cfg.goal_sphere_diameter   = 0.09;
    dbg_cfg.frame_sleep_ms         = 0;

    constexpr double kMinClearanceMeters = 0.10;  // must be >= col_cfg.collision_margin
    if (kMinClearanceMeters < col_cfg.collision_margin) {
        std::cerr << "[main] FATAL: kMinClearanceMeters (" << kMinClearanceMeters
                  << ") < col_cfg.collision_margin (" << col_cfg.collision_margin
                  << "). Legal goal poses can land inside the repulsion zone.\n";
        mj_deleteData(d); mj_deleteModel(m);
        return 1;
    }

    constexpr int kMaxPoseSampleTries = 1500;
    constexpr int kEndFlashFrames     = 5;
    constexpr int kEndFlashSleepMs    = 10;

    const int left_site_id  = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
    const int right_site_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");

    std::random_device rd;
    std::mt19937 rng(rd());

    if (viewer.enabled() && !viewer.render(d)) {
        std::cerr << "Viewer closed before start.\n";
        mj_deleteData(d); mj_deleteModel(m);
        return 0;
    }

    // FIX (dead code): removed startup_done flag — the first iteration is
    // identical to all others; a different log line is not worth the confusion.
    int  motion_index = 0;
    bool viewer_open  = true;

    while (viewer_open) {
        mj_forward(m, d);

        Eigen::Map<const Eigen::VectorXd> start_q_map(d->qpos, m->nq);
        Eigen::VectorXd start_q = start_q_map;

        const double start_min_dist = computeMinContactDistance(d);
        const bool start_legal = !std::isfinite(start_min_dist) ||
                                 start_min_dist >= kMinClearanceMeters;

        Eigen::VectorXd goal_q(m->nq);
        double goal_min_dist = std::numeric_limits<double>::infinity();
        const bool found_goal = findRandomLegalPose(
            m, d, start_q, rng, kMinClearanceMeters,
            kMaxPoseSampleTries, goal_q, goal_min_dist);

        if (!found_goal) {
            std::cerr << "Failed to find a legal random pose after "
                      << kMaxPoseSampleTries << " tries.\n";
            break;
        }

        ++motion_index;
        std::cout << "\n=== Motion " << motion_index << " ===\n"
                  << "  start min distance: " << start_min_dist << " m\n"
                  << "  goal  min distance: " << goal_min_dist  << " m\n";

        Eigen::Vector3d goal_left_marker  = Eigen::Vector3d::Zero();
        Eigen::Vector3d goal_right_marker = Eigen::Vector3d::Zero();
        const bool goal_marker_ok = computeGripperSitePositionsAtPose(
            m, d, goal_q, left_site_id, right_site_id,
            goal_left_marker, goal_right_marker);

        if (goal_marker_ok && viewer.enabled()) {
            viewer.setDebugSpheres(goal_left_marker, goal_right_marker,
                                   0.09, 1.0f, 0.65f, 0.0f, 0.85f);
            if (!viewer.render(d)) {
                std::cerr << "Viewer closed.\n";
                viewer_open = false;
                continue;
            }
        }

        if (!start_legal) {
            std::cerr << "Result: FAILED (start pose below "
                      << kMinClearanceMeters << " m; skip motion).\n";
            continue;
        }

        bool task_done  = false;
        bool success    = false;
        bool failed     = false;
        std::string fail_reason;
        int steps_done  = 0;

        std::deque<double> error_history;
        std::deque<double> dist_history;

        for (int step = 0; step < solver_cfg.max_steps && !task_done; ++step) {
            IKSolver::StepResult r = solver.solveStep(
                d, goal_left_marker, goal_right_marker,
                solver_cfg, col_cfg,
                error_history, dist_history);

            steps_done = step + 1;

            if (dbg_cfg.print_summary)
                solver.printStep(step, r);

            if (dbg_cfg.visualize_goal_spheres && goal_marker_ok && viewer.enabled()) {
                viewer.setDebugSpheres(goal_left_marker, goal_right_marker,
                                       dbg_cfg.goal_sphere_diameter,
                                       1.0f, 0.65f, 0.0f, 0.85f);
            }

            if (r.converged || r.early_converged) {
                success   = true;
                task_done = true;
            } else if (r.stalled) {
                failed      = true;
                fail_reason = "stalled";
                task_done   = true;
            } else if (step == solver_cfg.max_steps - 1) {
                failed      = true;
                fail_reason = "max steps reached";
                task_done   = true;
            }

            if (viewer.enabled() && !viewer.render(d)) {
                std::cerr << "Viewer closed.\n";
                viewer_open = false;
                task_done   = true;
            }

            if (dbg_cfg.frame_sleep_ms > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(dbg_cfg.frame_sleep_ms));
        }

        if (!viewer_open) continue;

        // Post-motion safety check: the IK integrator can leave the robot at a
        // pose whose min_dist is just below kMinClearanceMeters (e.g. 0.0995 m
        // with a 0.10 floor).  Without this guard every subsequent iteration
        // fails the start-pose check and the robot is stuck forever.
        // Restore start_q whenever the final pose is below clearance.
        mj_forward(m, d);
        const double final_min_dist = computeMinContactDistance(d);
        if (std::isfinite(final_min_dist) && final_min_dist < kMinClearanceMeters) {
            std::cerr << "Result: FAILED (final pose below clearance: "
                      << final_min_dist << " m — restoring start pose)\n";
            mju_copy(d->qpos, start_q.data(), m->nq);
            mju_zero(d->qvel, m->nv);
            mj_forward(m, d);
            success = false;
            failed  = true;
            fail_reason = "final pose below clearance";
        }

        if (success)
            std::cout << "Result: SUCCESS (IK reached legal target, steps=" << steps_done << ")\n";
        else if (failed)
            std::cerr << "Result: FAILED (" << fail_reason << ")\n";

        if (goal_marker_ok && viewer.enabled()) {
            const float rr = success ? 0.2f : 0.95f;
            const float gg = success ? 0.95f : 0.2f;
            viewer.setDebugSpheres(goal_left_marker, goal_right_marker,
                                   0.10, rr, gg, 0.2f, 0.90f);
            for (int i = 0; i < kEndFlashFrames; ++i) {
                if (!viewer.render(d)) { viewer_open = false; break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(kEndFlashSleepMs));
            }
        }
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}