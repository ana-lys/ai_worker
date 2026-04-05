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
        cam_.distance  = 2.0;
        cam_.azimuth   = 140.0;
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

    // Returns false if the window was closed.
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
        debug_sphere_radius_    = std::max(0.0, 0.5 * diameter);
        debug_sphere_rgba_[0]  = r;
        debug_sphere_rgba_[1]  = g;
        debug_sphere_rgba_[2]  = b;
        debug_sphere_rgba_[3]  = a;
    }

private:
    mjModel*   m_      = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera  cam_;
    mjvOption  opt_;
    mjvScene   scn_;
    mjrContext con_;
    bool       enabled_              = false;
    bool       debug_spheres_enabled_ = false;
    mjtNum     debug_sphere_pos_left_[3]  = {};
    mjtNum     debug_sphere_pos_right_[3] = {};
    mjtNum     debug_sphere_radius_       = 0.05;
    float debug_sphere_rgba_[4] = {0.2f, 0.9f, 0.2f, 0.8f};
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
        double tolerance         = 5e-3;   // hard EE error stop
        double joint_vel_limit   = 3.1;
        bool   qp_verbose        = false;

        // Early-stop: sliding window on EE error rate of improvement
        int    ee_window              = 5;     // steps to look back
        double ee_improvement_rate    = 0.02;  // <2% improvement → stalled

        // Early-stop: min_dist stability
        int    dist_window            = 5;
        double dist_stability_thresh  = 0.002; // variance threshold (m)
        double dist_safe_ratio        = 0.98;  // min_dist > margin * ratio

        // Objective plateau early-convergence threshold
        // Set to a reachable value given your EE(0.9) + collision(0.1) weights.
        double early_convergence_obj  = 0.87;

        int    max_steps         = 100;
    };

    struct CollisionCostConfig {
        double collision_margin = 0.155; // contacts detected within this distance (m)
        double weight_scale     = 0.01; // global scale on inverse-square weight
        double epsilon          = 1e-1; // regularization: keeps max weight ~10
    };

    struct DebugConfig {
        int    topk_contacts          = 5;
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
        std::vector<mjtNum> Jdist_row; // (nv,) signed distance Jacobian
    };

    struct ContactResult {
        int                      total_contacts = 0;
        std::vector<ContactInfo> closest;       // sorted nearest-first, up to topk
    };

    struct StepResult {
        double error              = std::numeric_limits<double>::infinity();
        double objective_ee       = 0.0;
        double objective_collision = 0.0;
        double objective_total    = 0.0;
        double min_dist           = std::numeric_limits<double>::quiet_NaN();
        bool   converged          = false; // hard tolerance met
        bool   early_converged    = false; // plateau at good objective
        bool   stalled            = false; // plateau at bad objective
        ContactResult contacts;            // cached — no double computation
    };

    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------
    explicit IKSolver(mjModel* model) : m_(model), nv_(model->nv) {
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
    // Contact query (called once per step, result stored in StepResult)
    // ------------------------------------------------------------------
    void computeContacts(mjData* d, int topk, ContactResult& out) {
        out.total_contacts = d ? d->ncon : 0;
        out.closest.clear();
        if (!d || d->ncon <= 0 || topk <= 0) return;

        // Partial sort: find topk nearest contacts
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

            // Contact midpoint; p1/p2 on each body surface
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

                // Jdist = n^T (Jc2 - Jc1): gradient of signed distance w.r.t. qvel
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
                         // Early-stop state (maintained by caller across steps):
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

        // Contacts (computed once, cached in result)
        computeContacts(d, 5, result.contacts);
        result.min_dist = result.contacts.closest.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : result.contacts.closest.front().dist;

        // Objectives
        result.objective_ee        = eeObjective(result.error);
        result.objective_collision = collisionObjective(result.min_dist);
        result.objective_total     = result.objective_ee + result.objective_collision;

        // ----------------------------------------------------------------
        // Hard convergence: EE error below tolerance
        // ----------------------------------------------------------------
        if (result.error < cfg.tolerance) {
            result.converged = true;
            return result;
        }

        // ----------------------------------------------------------------
        // Sliding-window early stop
        //   Condition A: EE improvement rate < threshold over last N steps
        //   Condition B: min_dist is stable (variance < thresh) and safe
        // Both must be true to stop early.
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

            // Safe if: no contacts exist at all (trivially clear),
            // OR the dist window is full and min_dist is stable and above margin.
            bool dist_stable_and_safe = false;
            if (result.contacts.total_contacts == 0) {
                result.objective_collision = 0.1;      // desired constant
                result.min_dist = col_cfg.collision_margin; // optional, for logging/early-stop
                dist_stable_and_safe = true;
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
        // Build QP: min 0.5 dq^T H dq + g^T dq  s.t. lb <= dq <= ub
        //
        // Two branches depending on whether any contact is within margin:
        //
        //   NO contacts within margin → robot is safe, do pure EE tracking.
        //     H = J^T J + damping*I,  g = J^T * err
        //     No collision term needed; collision objective is already maximal.
        //
        //   Contacts within margin → blend EE tracking with collision repulsion.
        //     H = J^T J + damping*I  (kept PD — no Hessian subtraction)
        //     g = J^T * err  −  Jdist^T * W * d   (gradient repulsion only)
        // ----------------------------------------------------------------

        // Collect contacts that are within the collision margin
        int n_within = 0;
        Eigen::MatrixXd Jdist_all;
        Eigen::VectorXd dist_all;
        Eigen::VectorXd w_all;

        if (!result.contacts.closest.empty()) {
            // Pre-size to worst case
            Jdist_all.resize(result.contacts.closest.size(), nv_);
            dist_all.resize(result.contacts.closest.size());
            w_all.resize(result.contacts.closest.size());

            for (const ContactInfo& ci : result.contacts.closest) {
                if (ci.dist > col_cfg.collision_margin) continue;      // outside margin
                if ((int)ci.Jdist_row.size() != nv_)        continue;  // no Jacobian
                Jdist_all.row(n_within) =
                    Eigen::Map<const Eigen::Matrix<mjtNum,1,Eigen::Dynamic>>(
                        ci.Jdist_row.data(), nv_).cast<double>();
                dist_all(n_within) = ci.dist;
                w_all(n_within)    = col_cfg.weight_scale /
                                     (ci.dist * ci.dist + col_cfg.epsilon);
                ++n_within;
            }
        }

        const bool contacts_within_margin = n_within > 0;

        if (!contacts_within_margin) {
            // Safe: pure EE tracking
            mj_jacSite(m_, d, jacp_l_.data(), nullptr, id_l_);
            mj_jacSite(m_, d, jacp_r_.data(), nullptr, id_r_);
            J_.block(0, 0, 3, nv_) =
                Eigen::Map<Eigen::MatrixXd>(jacp_l_.data(), nv_, 3).transpose();
            J_.block(3, 0, 3, nv_) =
                Eigen::Map<Eigen::MatrixXd>(jacp_r_.data(), nv_, 3).transpose();

            H_ = J_.transpose() * J_;
            H_.diagonal().array() += cfg.damping;
            g_ = J_.transpose() * err_;
        } else {
            // Contacts within margin: EE tracking + gradient collision repulsion
            mj_jacSite(m_, d, jacp_l_.data(), nullptr, id_l_);
            mj_jacSite(m_, d, jacp_r_.data(), nullptr, id_r_);
            J_.block(0, 0, 3, nv_) =
                Eigen::Map<Eigen::MatrixXd>(jacp_l_.data(), nv_, 3).transpose();
            J_.block(3, 0, 3, nv_) =
                Eigen::Map<Eigen::MatrixXd>(jacp_r_.data(), nv_, 3).transpose();

            H_ = J_.transpose() * J_;
            H_.diagonal().array() += cfg.damping;
            g_ = J_.transpose() * err_;

            // Subtract repulsion from gradient: pushes joints to increase distance.
            // Sign: minimizing g^T dq with g -= Jdist^T W d means we prefer
            // dq in the +Jdist direction (increasing distance).
            g_ -= Jdist_all.topRows(n_within).transpose() *
                  w_all.head(n_within).asDiagonal() *
                  dist_all.head(n_within);
        }

        // Joint limits → velocity bounds
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

        proxsuite::proxqp::dense::QP<double> qp(nv_, 0, nv_);
        qp.settings.verbose = cfg.qp_verbose;
        qp.init(H_, g_, std::nullopt, std::nullopt, C_, lb_, ub_);
        qp.solve();

        mj_integratePos(m_, d->qpos, qp.results.x.data(), cfg.step_size);
        return result;
    }

    // ------------------------------------------------------------------
    // Debug print (uses cached contacts from StepResult — no recompute)
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
    std::vector<mjtNum> jacp_l_, jacp_r_, jacp_c1_, jacp_c2_;
    std::vector<int>    scratch_;
};

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

    // Base targets from original config — random goals are sampled near these.
    const Eigen::Vector3d base_left (0.5, -0.3, 1.0);
    const Eigen::Vector3d base_right(0.6, -0.3, 1.4);
    // Uniform random perturbation radius (m) applied independently per axis.
    const double goal_radius = 0.25;

    IKSolver      solver(m);
    SimpleViewer  viewer(m);

    IKSolver::SolverConfig solver_cfg;
    solver_cfg.damping                = 1e-3;
    solver_cfg.step_size              = 0.15;
    solver_cfg.tolerance              = 5e-3;
    solver_cfg.joint_vel_limit        = 3.1;
    solver_cfg.max_steps              = 100;
    solver_cfg.ee_window              = 5;
    solver_cfg.ee_improvement_rate    = 0.02;
    solver_cfg.dist_window            = 5;
    solver_cfg.dist_stability_thresh  = 0.002;
    solver_cfg.dist_safe_ratio        = 0.98;
    solver_cfg.early_convergence_obj  = 0.87;

    IKSolver::CollisionCostConfig col_cfg;
    col_cfg.collision_margin = 0.15;
    col_cfg.weight_scale     = 0.005;
    col_cfg.epsilon          = 1e-1;

    IKSolver::DebugConfig dbg_cfg;
    dbg_cfg.topk_contacts          = 5;
    dbg_cfg.print_summary          = true;
    dbg_cfg.visualize_goal_spheres = true;
    dbg_cfg.frame_sleep_ms         = 0;

    // Random number generator seeded from hardware entropy.
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<double> perturb(-goal_radius, goal_radius);

    auto randomGoal = [&](const Eigen::Vector3d& base) -> Eigen::Vector3d {
        return base + Eigen::Vector3d(perturb(rng), perturb(rng), perturb(rng));
    };

    // Initial render before first task.
    if (viewer.enabled() && !viewer.render(d)) {
        std::cerr << "Viewer closed before start.\n";
        mj_deleteData(d); mj_deleteModel(m);
        return 0;
    }

    int task_index = 0;
    bool viewer_open = true;

    while (viewer_open) {
        // Sample new random targets.
        Eigen::Vector3d target_left  = randomGoal(base_left);
        Eigen::Vector3d target_right = randomGoal(base_right);

        ++task_index;
        std::cout << "\n=== Task " << task_index << " ===\n"
                  << "  left  target: ["
                  << target_left.x()  << ", "
                  << target_left.y()  << ", "
                  << target_left.z()  << "]\n"
                  << "  right target: ["
                  << target_right.x() << ", "
                  << target_right.y() << ", "
                  << target_right.z() << "]\n";

        // Reset robot to zero pose for each new task.
        mju_zero(d->qpos, m->nq);
        mju_zero(d->qvel, m->nv);
        if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE) d->qpos[3] = 1.0;

        // Reset sliding-window state for the new task.
        std::deque<double> error_history;
        std::deque<double> dist_history;

        bool task_done = false;

        for (int step = 0; step < solver_cfg.max_steps && !task_done; ++step) {
            IKSolver::StepResult r = solver.solveStep(
                d, target_left, target_right,
                solver_cfg, col_cfg,
                error_history, dist_history);

            if (dbg_cfg.print_summary)
                solver.printStep(step, r);

            if (dbg_cfg.visualize_goal_spheres && viewer.enabled()) {
                viewer.setDebugSpheres(target_left, target_right,
                                       dbg_cfg.goal_sphere_diameter,
                                       0.2f, 0.9f, 0.2f, 0.85f);
            }

            if (r.converged) {
                std::cout << "Result: SUCCESS (converged, error=" << r.error << ")\n";
                task_done = true;
            } else if (r.early_converged) {
                std::cout << "Result: SUCCESS (early convergence, obj="
                          << r.objective_total << ", error=" << r.error << ")\n";
                task_done = true;
            } else if (r.stalled) {
                std::cerr << "Result: FAILED (stalled, obj="
                          << r.objective_total << ", error=" << r.error << ")\n";
                task_done = true;
            } else if (step == solver_cfg.max_steps - 1) {
                std::cerr << "Result: FAILED (max steps reached, error=" << r.error << ")\n";
                task_done = true;
            }

            if (viewer.enabled() && !viewer.render(d)) {
                std::cerr << "Viewer closed.\n";
                viewer_open = false;
                task_done   = true;
            }

            if (dbg_cfg.frame_sleep_ms > 0)
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(dbg_cfg.frame_sleep_ms));
        }
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}