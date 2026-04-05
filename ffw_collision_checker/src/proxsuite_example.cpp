#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <limits>
#include <numeric>
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
        cam_.distance = 2.0;
        cam_.azimuth = 140.0;
        cam_.elevation = -20.0;
        cam_.type = mjCAMERA_FREE;
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
        if (window_) {
            glfwDestroyWindow(window_);
        }
        glfwTerminate();
    }

    bool render(mjData* d) {
        if (!enabled_) {
            return true;
        }
        glfwPollEvents();
        if (glfwWindowShouldClose(window_)) {
            return false;
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
        mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        if (debug_sphere_enabled_ && scn_.ngeom < scn_.maxgeom) {
            mjvGeom* g = &scn_.geoms[scn_.ngeom];
            const mjtNum size[3] = {debug_sphere_radius_, debug_sphere_radius_, debug_sphere_radius_};
            const mjtNum pos[3] = {debug_sphere_pos_[0], debug_sphere_pos_[1], debug_sphere_pos_[2]};
            const mjtNum mat[9] = {
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            };
            mjv_initGeom(g, mjGEOM_SPHERE, size, pos, mat, debug_sphere_rgba_);
            g->category = mjCAT_DECOR;
            scn_.ngeom++;
        }
        mjr_render(viewport, &scn_, &con_);
        glfwSwapBuffers(window_);
        return true;
    }

    bool enabled() const {
        return enabled_;
    }

    void setDebugSphere(const Eigen::Vector3d& pos,
                        double diameter = 0.1,
                        float r = 1.0f,
                        float g = 0.5f,
                        float b = 0.0f,
                        float a = 0.8f) {
        debug_sphere_enabled_ = true;
        debug_sphere_pos_[0] = pos.x();
        debug_sphere_pos_[1] = pos.y();
        debug_sphere_pos_[2] = pos.z();
        debug_sphere_radius_ = std::max(0.0, 0.5 * diameter);
        debug_sphere_rgba_[0] = r;
        debug_sphere_rgba_[1] = g;
        debug_sphere_rgba_[2] = b;
        debug_sphere_rgba_[3] = a;
    }

    void clearDebugSphere() {
        debug_sphere_enabled_ = false;
    }

private:
    mjModel* m_ = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    bool enabled_ = false;
    bool debug_sphere_enabled_ = false;
    mjtNum debug_sphere_pos_[3] = {0.0, 0.0, 0.0};
    mjtNum debug_sphere_radius_ = 0.05;
    float debug_sphere_rgba_[4] = {1.0f, 0.5f, 0.0f, 0.8f};
};

class IKSolver {
public:
    struct SolverConfig {
        double damping = 1e-3;
        double step_size = 0.15;
        double tolerance = 5e-3;
        double practical_tolerance = 7e-2;
        double min_progress = 1e-3;
        int stagnation_window = 10;
        int max_steps = 100;
        double joint_vel_limit = 3.1;
        bool qp_verbose = false;
    };

    struct ContactDebugConfig {
        int topk_contacts = 5;
        bool print_contacts = true;
        bool print_contact_jacobians = true;
        bool print_full_contact_jacobians = false;
        bool visualize_min_contact_sphere = false;
        double min_contact_sphere_diameter = 0.1;
        int frame_sleep_ms = 40;
    };

    struct RunConfig {
        SolverConfig solver;
        ContactDebugConfig debug;
    };

    struct ContactInfo {
        int contact_index = -1;
        double dist = std::numeric_limits<double>::infinity();
        int geom1 = -1;
        int geom2 = -1;
        int body1 = -1;
        int body2 = -1;
        Eigen::Vector3d mid = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d p2 = Eigen::Vector3d::Zero();
        double Jc1_fro = 0.0;
        double Jc2_fro = 0.0;
        std::vector<mjtNum> Jc1_full;
        std::vector<mjtNum> Jc2_full;
    };

    struct ContactDebugResult {
        int total_contacts = 0;
        std::vector<ContactInfo> closest_contacts;
    };

    struct StepResult {
        double error = std::numeric_limits<double>::infinity();
        int stagnation_steps = 0;
        bool converged = false;
        bool early_converged = false;
        bool stalled = false;
    };

    IKSolver(mjModel* model) : m(model) {
        nv = m->nv;
        // Pre-allocate Eigen matrices
        J.resize(6, nv);
        H.resize(nv, nv);
        g.resize(nv);
        err.resize(6);
        C = Eigen::MatrixXd::Identity(nv, nv);
        lb.resize(nv);
        ub.resize(nv);

        // Pre-allocate MuJoCo Jacobian buffers
        jacp_l.resize(3 * nv);
        jacp_r.resize(3 * nv);
        jacp_c1_.resize(3 * nv);
        jacp_c2_.resize(3 * nv);
        top_contact_indices_.reserve(5);

        // Find Site IDs once
        id_l = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
        id_r = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");
        
        if (id_l == -1 || id_r == -1) {
            std::cerr << "Warning: Gripper sites not found in model!" << std::endl;
        }
    }

    static const char* geomName(const mjModel* model, int geom_id) {
        if (!model || geom_id < 0 || geom_id >= model->ngeom) {
            return "<invalid-geom>";
        }
        int adr = model->name_geomadr[geom_id];
        return (adr >= 0) ? (model->names + adr) : "<unnamed-geom>";
    }

    static const char* geomBodyName(const mjModel* model, int geom_id) {
        if (!model || geom_id < 0 || geom_id >= model->ngeom) {
            return "<invalid-body>";
        }
        int body_id = model->geom_bodyid[geom_id];
        if (body_id < 0 || body_id >= model->nbody) {
            return "<invalid-body>";
        }
        int adr = model->name_bodyadr[body_id];
        return (adr >= 0) ? (model->names + adr) : "<unnamed-body>";
    }

    void fillTopKClosestContacts(const mjData* d, int k, std::vector<int>& out_indices) {
        out_indices.clear();
        if (!d || d->ncon <= 0 || k <= 0) {
            return;
        }

        const int kk = std::min(k, d->ncon);
        scratch_contact_indices_.resize(d->ncon);
        std::iota(scratch_contact_indices_.begin(), scratch_contact_indices_.end(), 0);

        auto cmp = [d](int a, int b) {
            return d->contact[a].dist < d->contact[b].dist;
        };

        if (kk < d->ncon) {
            std::nth_element(
                scratch_contact_indices_.begin(),
                scratch_contact_indices_.begin() + kk,
                scratch_contact_indices_.end(),
                cmp);
            scratch_contact_indices_.resize(kk);
        }

        std::sort(scratch_contact_indices_.begin(), scratch_contact_indices_.end(), cmp);
        out_indices = scratch_contact_indices_;
    }

    void computeContactDebugResult(mjData* d,
                                   const ContactDebugConfig& cfg,
                                   ContactDebugResult& result) {
        result.total_contacts = d ? d->ncon : 0;
        result.closest_contacts.clear();
        if (!d || d->ncon <= 0 || cfg.topk_contacts <= 0) {
            return;
        }

        fillTopKClosestContacts(d, cfg.topk_contacts, top_contact_indices_);
        result.closest_contacts.reserve(top_contact_indices_.size());

        for (int ci : top_contact_indices_) {
            const mjContact& c = d->contact[ci];
            ContactInfo info;
            info.contact_index = ci;
            info.dist = c.dist;
            info.geom1 = c.geom1;
            info.geom2 = c.geom2;
            info.body1 = (info.geom1 >= 0 && info.geom1 < m->ngeom) ? m->geom_bodyid[info.geom1] : -1;
            info.body2 = (info.geom2 >= 0 && info.geom2 < m->ngeom) ? m->geom_bodyid[info.geom2] : -1;

            info.normal = Eigen::Vector3d(c.frame[0], c.frame[1], c.frame[2]);
            info.mid = Eigen::Vector3d(c.pos[0], c.pos[1], c.pos[2]);
            info.p1 = info.mid - 0.5 * c.dist * info.normal;
            info.p2 = info.mid + 0.5 * c.dist * info.normal;

            if (cfg.print_contact_jacobians && info.body1 >= 0 && info.body2 >= 0) {
                const mjtNum point1[3] = {
                    static_cast<mjtNum>(info.p1.x()),
                    static_cast<mjtNum>(info.p1.y()),
                    static_cast<mjtNum>(info.p1.z())
                };
                const mjtNum point2[3] = {
                    static_cast<mjtNum>(info.p2.x()),
                    static_cast<mjtNum>(info.p2.y()),
                    static_cast<mjtNum>(info.p2.z())
                };

                mj_jac(m, d, jacp_c1_.data(), nullptr, point1, info.body1);
                mj_jac(m, d, jacp_c2_.data(), nullptr, point2, info.body2);

                Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jc1(jacp_c1_.data(), 3, nv);
                Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jc2(jacp_c2_.data(), 3, nv);
                info.Jc1_fro = Jc1.norm();
                info.Jc2_fro = Jc2.norm();

                if (cfg.print_full_contact_jacobians) {
                    info.Jc1_full.assign(jacp_c1_.begin(), jacp_c1_.end());
                    info.Jc2_full.assign(jacp_c2_.begin(), jacp_c2_.end());
                }
            }

            result.closest_contacts.push_back(std::move(info));
        }
    }

    void printContactDebug(int step,
                           double current_error,
                           int stagnation_steps,
                           const ContactDebugConfig& cfg,
                           const ContactDebugResult& result) const {
        if (!cfg.print_contacts) {
            return;
        }

        std::printf("Step %3d | Error: %.6f | Contacts: %d | topk: %zu | stall:%2d\n",
                    step,
                    current_error,
                    result.total_contacts,
                    result.closest_contacts.size(),
                    stagnation_steps);

        for (size_t rank = 0; rank < result.closest_contacts.size(); ++rank) {
            const ContactInfo& info = result.closest_contacts[rank];
            std::printf("  #%zu idx=%d dist=%.6f | g1=%d (%s/%s) | g2=%d (%s/%s)\n",
                        rank + 1,
                        info.contact_index,
                        info.dist,
                        info.geom1,
                        geomBodyName(m, info.geom1),
                        geomName(m, info.geom1),
                        info.geom2,
                        geomBodyName(m, info.geom2),
                        geomName(m, info.geom2));
            std::printf("     p1=[%.4f %.4f %.4f] p2=[%.4f %.4f %.4f]\n",
                        info.p1.x(), info.p1.y(), info.p1.z(),
                        info.p2.x(), info.p2.y(), info.p2.z());

            if (cfg.print_contact_jacobians && info.body1 >= 0 && info.body2 >= 0) {
                std::printf("     Jc1(3x%d) fro=%.6f | Jc2(3x%d) fro=%.6f\n",
                            nv,
                            info.Jc1_fro,
                            nv,
                            info.Jc2_fro);
                if (cfg.print_full_contact_jacobians &&
                    info.Jc1_full.size() == static_cast<size_t>(3 * nv) &&
                    info.Jc2_full.size() == static_cast<size_t>(3 * nv)) {
                    Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jc1(info.Jc1_full.data(), 3, nv);
                    Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jc2(info.Jc2_full.data(), 3, nv);
                    std::cout << "     Jc1 =\n" << Jc1 << std::endl;
                    std::cout << "     Jc2 =\n" << Jc2 << std::endl;
                }
            }
        }
    }

    void updateContactVisualization(const ContactDebugConfig& cfg,
                                    const ContactDebugResult& result,
                                    SimpleViewer* viewer) const {
        if (!viewer || !cfg.visualize_min_contact_sphere) {
            return;
        }

        if (result.closest_contacts.empty()) {
            viewer->clearDebugSphere();
            return;
        }

        viewer->setDebugSphere(
            result.closest_contacts.front().mid,
            cfg.min_contact_sphere_diameter,
            1.0f,
            0.5f,
            0.0f,
            0.85f);
    }

    StepResult solveStep(mjData* d,
                         const Eigen::Vector3d& target_l,
                         const Eigen::Vector3d& target_r,
                         const SolverConfig& cfg,
                         double& best_error,
                         int& stagnation_steps) {
        StepResult result;

        mj_forward(m, d);  // FK + collisions in one call

        Eigen::Vector3d pos_l = Eigen::Vector3d::Map(d->site_xpos + 3 * id_l);
        Eigen::Vector3d pos_r = Eigen::Vector3d::Map(d->site_xpos + 3 * id_r);
        err.segment<3>(0) = (pos_l - target_l);
        err.segment<3>(3) = (pos_r - target_r);
        result.error = err.norm();

        if ((best_error - result.error) > cfg.min_progress) {
            best_error = result.error;
            stagnation_steps = 0;
        } else {
            stagnation_steps++;
        }
        result.stagnation_steps = stagnation_steps;

        if (result.error < cfg.tolerance) {
            result.converged = true;
            return result;
        }

        if (stagnation_steps >= cfg.stagnation_window) {
            if (result.error < cfg.practical_tolerance) {
                result.early_converged = true;
            } else {
                result.stalled = true;
            }
            return result;
        }

        mj_jacSite(m, d, jacp_l.data(), nullptr, id_l);
        mj_jacSite(m, d, jacp_r.data(), nullptr, id_r);
        J.block(0, 0, 3, nv) = Eigen::Map<Eigen::MatrixXd>(jacp_l.data(), nv, 3).transpose();
        J.block(3, 0, 3, nv) = Eigen::Map<Eigen::MatrixXd>(jacp_r.data(), nv, 3).transpose();

        H = J.transpose() * J;
        H.diagonal().array() += cfg.damping;
        g = J.transpose() * err;

        for (int i = 0; i < nv; ++i) {
            int jnt_id = m->dof_jntid[i];
            double v_min = -cfg.joint_vel_limit;
            double v_max = cfg.joint_vel_limit;

            if (m->jnt_limited[jnt_id]) {
                double q_min = m->jnt_range[jnt_id * 2];
                double q_max = m->jnt_range[jnt_id * 2 + 1];
                double q_curr = d->qpos[m->jnt_qposadr[jnt_id]];
                v_min = std::max(v_min, (q_min - q_curr) / cfg.step_size);
                v_max = std::min(v_max, (q_max - q_curr) / cfg.step_size);
            }
            lb[i] = v_min;
            ub[i] = v_max;
        }

        proxsuite::proxqp::dense::QP<double> qp(nv, 0, nv);
        qp.settings.verbose = cfg.qp_verbose;
        qp.init(H, g, std::nullopt, std::nullopt, C, lb, ub);
        qp.solve();

        mj_integratePos(m, d->qpos, qp.results.x.data(), cfg.step_size);
        return result;
    }

    bool solve(mjData* d,
               const Eigen::Vector3d& target_l,
               const Eigen::Vector3d& target_r,
               const SolverConfig& cfg,
               StepResult* out_last_step = nullptr) {
        double best_error = std::numeric_limits<double>::infinity();
        int stagnation_steps = 0;

        for (int n = 0; n < cfg.max_steps; ++n) {
            StepResult step = solveStep(d, target_l, target_r, cfg, best_error, stagnation_steps);
            if (out_last_step) {
                *out_last_step = step;
            }

            if (step.converged || step.early_converged) {
                return true;
            }
            if (step.stalled) {
                return false;
            }
        }
        return false;
    }

private:
    mjModel* m;
    int nv, id_l, id_r;
    Eigen::MatrixXd J, H, C;
    Eigen::VectorXd g, err, lb, ub;
    std::vector<mjtNum> jacp_l, jacp_r;
    std::vector<mjtNum> jacp_c1_, jacp_c2_;
    std::vector<int> top_contact_indices_;
    std::vector<int> scratch_contact_indices_;
};

// --- Main ---
int main(int argc, char** argv) {
    using ament_index_cpp::get_package_share_directory;
    std::string collision_pkg = get_package_share_directory("ffw_collision_checker");
    std::string mujoco_xml_path = collision_pkg + "/3rd_party/robotis_ffw/scene.xml";

    char error[1000];
    mjModel* m = mj_loadXML(mujoco_xml_path.c_str(), 0, error, 1000);
    if (!m) {
        std::cerr << "MuJoCo Load Error: " << error << std::endl;
        return 1;
    }
    mjData* d = mj_makeData(m);

    // Initial State Reset
    mju_zero(d->qpos, m->nq);
    mju_zero(d->qvel, m->nv);
    if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE) d->qpos[3] = 1.0;
    mj_forward(m, d);

    // Specific Manual Targets
    Eigen::Vector3d target_left(0.5, -0.3, 1.0); 
    Eigen::Vector3d target_right(0.6, -0.3, 1.4);

    // Create Solver Instance (Pre-allocates memory)
    IKSolver solver(m);
    SimpleViewer viewer(m);

    IKSolver::RunConfig run_cfg;
    run_cfg.solver.damping = 1e-3;
    run_cfg.solver.step_size = 0.15;
    run_cfg.solver.tolerance = 5e-3;
    run_cfg.solver.practical_tolerance = 7e-2;
    run_cfg.solver.min_progress = 1e-3;
    run_cfg.solver.stagnation_window = 10;
    run_cfg.solver.max_steps = 100;
    run_cfg.solver.joint_vel_limit = 3.1;
    run_cfg.solver.qp_verbose = false;

    run_cfg.debug.topk_contacts = 5;
    run_cfg.debug.print_contacts = false;
    run_cfg.debug.print_contact_jacobians = true;
    run_cfg.debug.print_full_contact_jacobians = true;
    run_cfg.debug.visualize_min_contact_sphere = false;
    run_cfg.debug.min_contact_sphere_diameter = 0.1;
    run_cfg.debug.frame_sleep_ms = 80;

    std::cout << "--- Starting IK ---" << std::endl;
    bool success = false;
    double best_error = std::numeric_limits<double>::infinity();
    int stagnation_steps = 0;

    if (viewer.enabled() && !viewer.render(d)) {
        std::cerr << "IK interrupted: viewer window closed." << std::endl;
    } else {
        for (int step = 0; step < run_cfg.solver.max_steps; ++step) {
            IKSolver::StepResult step_result = solver.solveStep(
                d,
                target_left,
                target_right,
                run_cfg.solver,
                best_error,
                stagnation_steps);

            IKSolver::ContactDebugResult debug_result;
            solver.computeContactDebugResult(d, run_cfg.debug, debug_result);
            solver.printContactDebug(
                step,
                step_result.error,
                step_result.stagnation_steps,
                run_cfg.debug,
                debug_result);
            solver.updateContactVisualization(run_cfg.debug, debug_result, &viewer);

            if (step_result.converged) {
                std::cout << "IK Success: Converged." << std::endl;
                success = true;
                break;
            }
            if (step_result.early_converged) {
                std::cout << "IK Early convergence: plateau near practical tolerance ("
                          << step_result.error << ")" << std::endl;
                success = true;
                break;
            }
            if (step_result.stalled) {
                std::cerr << "IK Stalled: no meaningful improvement for "
                          << step_result.stagnation_steps
                          << " steps (error=" << step_result.error << ")"
                          << std::endl;
                break;
            }

            if (viewer.enabled()) {
                if (!viewer.render(d)) {
                    std::cerr << "IK interrupted: viewer window closed." << std::endl;
                    break;
                }
            }

            if (run_cfg.debug.frame_sleep_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(run_cfg.debug.frame_sleep_ms));
            }

            if (step == run_cfg.solver.max_steps - 1) {
                std::cerr << "IK Failure: Max steps reached." << std::endl;
            }
        }
    }

    if (success) {
        std::cout << "IK Result: SUCCESS" << std::endl;
    }

    if (viewer.enabled()) {
        std::cout << "Close the viewer window to exit." << std::endl;
        while (viewer.render(d)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}