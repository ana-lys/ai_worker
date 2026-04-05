#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <limits>
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
        mjr_render(viewport, &scn_, &con_);
        glfwSwapBuffers(window_);
        return true;
    }

    bool enabled() const {
        return enabled_;
    }

private:
    mjModel* m_ = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    bool enabled_ = false;
};

class IKSolver {
public:
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

        // Find Site IDs once
        id_l = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
        id_r = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");
        
        if (id_l == -1 || id_r == -1) {
            std::cerr << "Warning: Gripper sites not found in model!" << std::endl;
        }
    }

    bool solve(mjData* d,
               const Eigen::Vector3d& target_l,
               const Eigen::Vector3d& target_r,
               SimpleViewer* viewer = nullptr) {
        // Hyperparameters
        const double damping = 1e-3;
        const double step_size = 0.15;
        const double tolerance = 5e-3;
        const double practical_tolerance = 7e-2;
        const double min_progress = 1e-3;
        const int stagnation_window = 10;
        const int max_steps = 100;
        const double joint_vel_limit = 3.1;
        const auto frame_sleep = std::chrono::milliseconds(40);

        double best_error = std::numeric_limits<double>::infinity();
        int stagnation_steps = 0;

        if (viewer && !viewer->render(d)) {
            return false;
        }

        for (int n = 0; n < max_steps; ++n) {
            mj_forward(m, d);  // FK + collisions in one call

            // 1. Calculate Error
            Eigen::Vector3d pos_l = Eigen::Vector3d::Map(d->site_xpos + 3 * id_l);
            Eigen::Vector3d pos_r = Eigen::Vector3d::Map(d->site_xpos + 3 * id_r);
            
            err.segment<3>(0) = 1.0 * (pos_l - target_l); // left
            err.segment<3>(3) = 1.0 * (pos_r - target_r); // right

            double current_error = err.norm();

            if ((best_error - current_error) > min_progress) {
                best_error = current_error;
                stagnation_steps = 0;
            } else {
                stagnation_steps++;
            }
            
            // Collision detection and distance stats
            int n_contacts = d->ncon;
            double max_dist = -1e6, min_dist = 1e6;
            if (n_contacts > 0) {
                for (int i = 0; i < n_contacts; ++i) {
                    double dist = d->contact[i].dist;
                    max_dist = std::max(max_dist, dist);
                    min_dist = std::min(min_dist, dist);
                }
            }
            std::printf("Step %3d | Error: %.6f | Contacts: %d | max_dist: %.4f | min_dist: %.4f | stall:%2d\n",
                        n, current_error, n_contacts, max_dist, min_dist, stagnation_steps);

            if (current_error < tolerance) {
                std::cout << "IK Success: Converged." << std::endl;
                return true;
            }

            if (stagnation_steps >= stagnation_window) {
                if (current_error < practical_tolerance) {
                    std::cout << "IK Early convergence: plateau near practical tolerance ("
                              << current_error << ")" << std::endl;
                    return true;
                }
                std::cerr << "IK Stalled: no meaningful improvement for "
                          << stagnation_steps << " steps (error=" << current_error << ")"
                          << std::endl;
                return false;
            }

            // 2. Compute Jacobians
            mj_jacSite(m, d, jacp_l.data(), nullptr, id_l);
            mj_jacSite(m, d, jacp_r.data(), nullptr, id_r);
            // Explicitly use Map with RowMajor or verify dimensions
            J.block(0, 0, 3, nv) = Eigen::Map<Eigen::MatrixXd>(jacp_l.data(), nv, 3).transpose();
            J.block(3, 0, 3, nv) = Eigen::Map<Eigen::MatrixXd>(jacp_r.data(), nv, 3).transpose();

            // 3. Build QP
            H = J.transpose() * J;
            H.diagonal().array() += damping;
            g = J.transpose() * err;

            // 4. Update Constraints (Joint Limits & Velocity)
            for (int i = 0; i < nv; ++i) {
                int jnt_id = m->dof_jntid[i];
                double v_min = -joint_vel_limit;
                double v_max = joint_vel_limit;

                if (m->jnt_limited[jnt_id]) {
                    double q_min = m->jnt_range[jnt_id * 2];
                    double q_max = m->jnt_range[jnt_id * 2 + 1];
                    double q_curr = d->qpos[m->jnt_qposadr[jnt_id]];
                    v_min = std::max(v_min, (q_min - q_curr) / step_size);
                    v_max = std::min(v_max, (q_max - q_curr) / step_size);
                }
                lb[i] = v_min;
                ub[i] = v_max;
            }

            // 5. Solve QP
            proxsuite::proxqp::dense::QP<double> qp(nv, 0, nv);
            qp.settings.verbose = false;
            qp.init(H, g, std::nullopt, std::nullopt, C, lb, ub);
            qp.solve();

            // 6. Integrate
            mj_integratePos(m, d->qpos, qp.results.x.data(), step_size);
            // Next loop iteration calls mj_forward(), no need to duplicate here

            if (viewer) {
                if (!viewer->render(d)) {
                    std::cerr << "IK interrupted: viewer window closed." << std::endl;
                    return false;
                }
            }
            std::this_thread::sleep_for(frame_sleep);
        }

        std::cerr << "IK Failure: Max steps reached." << std::endl;
        return false;
    }

private:
    mjModel* m;
    int nv, id_l, id_r;
    Eigen::MatrixXd J, H, C;
    Eigen::VectorXd g, err, lb, ub;
    std::vector<mjtNum> jacp_l, jacp_r;
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

    std::cout << "--- Starting IK ---" << std::endl;
    bool success = solver.solve(d, target_left, target_right, &viewer);

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