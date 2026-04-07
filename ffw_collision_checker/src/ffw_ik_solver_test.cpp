//
// ffw_ik_solver_test.cpp
//
// Stand-alone test driver for ffw_ik::IKSolver.
//
// Workflow (loops indefinitely until the viewer is closed):
//   1. Sample a random, collision-free joint pose as the IK goal.
//   2. Forward-compute the gripper site positions at that pose.
//   3. Run IK from the current robot state toward those EE targets.
//   4. Visualize the trajectory and print pass/fail.
//   5. Use the reached pose as the next starting state.
//

#include "ffw_ik_solver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ============================================================
// Minimal GLFW / MuJoCo viewer
// ============================================================

class SimpleViewer {
public:
    explicit SimpleViewer(mjModel* model) : m_(model) {
        if (!glfwInit()) { std::cerr << "[Viewer] GLFW init failed.\n"; return; }

        window_ = glfwCreateWindow(1200, 900, "FFW IK Test", nullptr, nullptr);
        if (!window_) {
            std::cerr << "[Viewer] Window creation failed.\n";
            glfwTerminate();
            return;
        }
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        mjv_defaultCamera(&cam_);
        cam_.distance   = 3.0;
        cam_.azimuth    = 0.0;
        cam_.elevation  = -20.0;
        cam_.type       = mjCAMERA_FREE;
        cam_.lookat[0]  = 0.0;
        cam_.lookat[1]  = 0.0;
        cam_.lookat[2]  = 1.0;

        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);
        enabled_ = true;
    }

    ~SimpleViewer() {
        if (enabled_) { mjv_freeScene(&scn_); mjr_freeContext(&con_); }
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }

    bool render(mjData* d) {
        if (!enabled_) return true;
        glfwPollEvents();
        if (glfwWindowShouldClose(window_)) return false;

        mjrRect vp = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &vp.width, &vp.height);
        mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

        if (spheres_enabled_ && scn_.ngeom + 2 <= scn_.maxgeom) {
            const mjtNum size[3] = {sphere_r_, sphere_r_, sphere_r_};
            const mjtNum mat[9]  = {1,0,0, 0,1,0, 0,0,1};

            mjvGeom* gl = &scn_.geoms[scn_.ngeom++];
            mjv_initGeom(gl, mjGEOM_SPHERE, size, pos_l_, mat, rgba_);
            gl->category = mjCAT_DECOR;

            mjvGeom* gr = &scn_.geoms[scn_.ngeom++];
            mjv_initGeom(gr, mjGEOM_SPHERE, size, pos_r_, mat, rgba_);
            gr->category = mjCAT_DECOR;
        }

        mjr_render(vp, &scn_, &con_);
        glfwSwapBuffers(window_);
        return true;
    }

    void setGoalSpheres(const Eigen::Vector3d& l, const Eigen::Vector3d& r,
                        double diameter = 0.09,
                        float fr = 1.0f, float fg = 0.65f,
                        float fb = 0.0f, float fa = 0.85f) {
        spheres_enabled_ = true;
        sphere_r_        = std::max(0.0, 0.5 * diameter);
        for (int i = 0; i < 3; ++i) { pos_l_[i] = l[i]; pos_r_[i] = r[i]; }
        rgba_[0] = fr; rgba_[1] = fg; rgba_[2] = fb; rgba_[3] = fa;
    }

    bool enabled() const { return enabled_; }

private:
    mjModel*    m_      = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera   cam_;
    mjvOption   opt_;
    mjvScene    scn_;
    mjrContext  con_;
    bool        enabled_         = false;
    bool        spheres_enabled_ = false;
    mjtNum      pos_l_[3]        = {};
    mjtNum      pos_r_[3]        = {};
    mjtNum      sphere_r_        = 0.045;
    float       rgba_[4]         = {1.f, 0.65f, 0.f, 0.85f};
};

// ============================================================
// Pose sampling helpers
// ============================================================

static double minContactDist(const mjData* d) {
    if (!d || d->ncon <= 0) return std::numeric_limits<double>::infinity();
    double m = std::numeric_limits<double>::infinity();
    for (int i = 0; i < d->ncon; ++i)
        m = std::min(m, static_cast<double>(d->contact[i].dist));
    return m;
}

static bool isPoseLegal(mjModel* m, mjData* d,
                        const Eigen::VectorXd& q,
                        double min_clearance,
                        double* out_dist = nullptr) {
    std::vector<mjtNum> qbak(m->nq), vbak(m->nv);
    mju_copy(qbak.data(), d->qpos, m->nq);
    mju_copy(vbak.data(), d->qvel, m->nv);

    mju_copy(d->qpos, q.data(), m->nq);
    mju_zero(d->qvel, m->nv);
    mj_forward(m, d);
    const double dist = minContactDist(d);
    const bool ok = !std::isfinite(dist) || dist >= min_clearance;
    if (out_dist) *out_dist = dist;

    mju_copy(d->qpos, qbak.data(), m->nq);
    mju_copy(d->qvel, vbak.data(), m->nv);
    mj_forward(m, d);
    return ok;
}

static Eigen::VectorXd sampleRandomPose(mjModel* m,
                                        const Eigen::VectorXd& ref,
                                        std::mt19937& rng) {
    Eigen::VectorXd q = ref;
    for (int jid = 0; jid < m->njnt; ++jid) {
        const int jtype = m->jnt_type[jid];
        if (jtype != mjJNT_HINGE && jtype != mjJNT_SLIDE) continue;

        const int qadr = m->jnt_qposadr[jid];
        double lo, hi;
        if (m->jnt_limited[jid]) {
            lo = m->jnt_range[2 * jid];
            hi = m->jnt_range[2 * jid + 1];
            const double span   = hi - lo;
            const double margin = 0.02 * span;
            lo = (span > 1e-6) ? lo + margin : lo;
            hi = (span > 1e-6) ? hi - margin : hi;
            if (lo > hi) { lo = m->jnt_range[2 * jid]; hi = m->jnt_range[2 * jid + 1]; }
        } else {
            lo = ref[qadr] - (jtype == mjJNT_HINGE ? M_PI : 0.25);
            hi = ref[qadr] + (jtype == mjJNT_HINGE ? M_PI : 0.25);
        }
        q[qadr] = std::uniform_real_distribution<double>(lo, hi)(rng);
    }
    return q;
}

static bool findLegalRandomPose(mjModel* m, mjData* d,
                                const Eigen::VectorXd& ref,
                                std::mt19937& rng,
                                double min_clearance,
                                int max_tries,
                                Eigen::VectorXd& out_q,
                                double& out_dist) {
    for (int t = 0; t < max_tries; ++t) {
        Eigen::VectorXd candidate = sampleRandomPose(m, ref, rng);
        double dist = std::numeric_limits<double>::infinity();
        if (isPoseLegal(m, d, candidate, min_clearance, &dist)) {
            out_q    = std::move(candidate);
            out_dist = dist;
            return true;
        }
    }
    return false;
}

static bool gripperSitesAtPose(mjModel* m, mjData* d,
                               const Eigen::VectorXd& q,
                               int left_id, int right_id,
                               Eigen::Vector3d& out_l,
                               Eigen::Vector3d& out_r) {
    if (left_id < 0 || right_id < 0) return false;

    std::vector<mjtNum> qbak(m->nq), vbak(m->nv);
    mju_copy(qbak.data(), d->qpos, m->nq);
    mju_copy(vbak.data(), d->qvel, m->nv);

    mju_copy(d->qpos, q.data(), m->nq);
    mju_zero(d->qvel, m->nv);
    mj_forward(m, d);
    out_l = Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
    out_r = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);

    mju_copy(d->qpos, qbak.data(), m->nq);
    mju_copy(d->qvel, vbak.data(), m->nv);
    mj_forward(m, d);
    return true;
}

// ============================================================
// main
// ============================================================

int main(int, char**) {
    using ament_index_cpp::get_package_share_directory;
    const std::string xml_path =
        get_package_share_directory("ffw_collision_checker")
        + "/3rd_party/robotis_ffw/scene_inverse_kinematic.xml";

    char error[1000];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!m) { std::cerr << "MuJoCo load error: " << error << "\n"; return 1; }
    mjData* d = mj_makeData(m);

    mju_zero(d->qpos, m->nq);
    mju_zero(d->qvel, m->nv);
    if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE) d->qpos[3] = 1.0;
    mj_forward(m, d);

    // IK configuration
    ffw_ik::SolverConfig solver_cfg;
    solver_cfg.damping               = 1e-3;
    solver_cfg.step_size             = 0.15;
    solver_cfg.tolerance             = 5e-3;
    solver_cfg.joint_vel_limit       = 3.1;
    solver_cfg.max_steps             = 100;
    solver_cfg.topk_contacts         = 5;
    solver_cfg.ee_window             = 5;
    solver_cfg.ee_improvement_rate   = 0.02;
    solver_cfg.dist_window           = 5;
    solver_cfg.dist_stability_thresh = 0.002;
    solver_cfg.dist_safe_ratio       = 0.98;
    solver_cfg.early_convergence_obj = 0.87;

    ffw_ik::CollisionCostConfig col_cfg;
    col_cfg.collision_margin = 0.10;
    col_cfg.weight_scale     = 0.005;
    col_cfg.epsilon          = 1e-1;

    constexpr double kMinClearance    = 0.10;
    constexpr int    kMaxSampleTries  = 1500;
    constexpr int    kPlaybackSleepMs = 20;
    constexpr int    kFlashFrames     = 10;
    constexpr int    kFlashSleepMs    = 10;

    if (kMinClearance < col_cfg.collision_margin) {
        std::cerr << "[test] FATAL: kMinClearance < col_cfg.collision_margin\n";
        mj_deleteData(d); mj_deleteModel(m);
        return 1;
    }

    ffw_ik::IKSolver solver(m);
    SimpleViewer     viewer(m);

    const int left_id  = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
    const int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");

    std::mt19937 rng(std::random_device{}());

    if (viewer.enabled() && !viewer.render(d)) {
        mj_deleteData(d); mj_deleteModel(m);
        return 0;
    }

    int  motion = 0;
    bool running = true;

    while (running) {
        mj_forward(m, d);

        const Eigen::Map<const Eigen::VectorXd> start_q_map(d->qpos, m->nq);
        const Eigen::VectorXd start_q = start_q_map;
        const double start_dist       = minContactDist(d);
        const bool   start_legal      = !std::isfinite(start_dist) || start_dist >= kMinClearance;

        // Sample a collision-free goal pose
        Eigen::VectorXd goal_q(m->nq);
        double goal_dist = std::numeric_limits<double>::infinity();
        if (!findLegalRandomPose(m, d, start_q, rng, kMinClearance,
                                 kMaxSampleTries, goal_q, goal_dist)) {
            std::cerr << "Could not find a legal goal after " << kMaxSampleTries << " tries.\n";
            break;
        }

        ++motion;
        std::cout << "\n=== Motion " << motion << " ==="
                  << " | start_dist=" << start_dist
                  << " | goal_dist="  << goal_dist << "\n";

        // Forward-compute EE targets at the goal pose
        Eigen::Vector3d goal_l, goal_r;
        const bool have_markers = gripperSitesAtPose(m, d, goal_q,
                                                     left_id, right_id,
                                                     goal_l, goal_r);

        if (have_markers && viewer.enabled()) {
            viewer.setGoalSpheres(goal_l, goal_r, 0.09, 1.0f, 0.65f, 0.0f, 0.85f);
            if (!viewer.render(d)) { running = false; continue; }
        }

        if (!start_legal) {
            std::cerr << "Skipping motion: start pose below clearance.\n";
            continue;
        }

        // Run IK — returns one qpos per step
        const auto trajectory = solver.solve(d, goal_l, goal_r, solver_cfg, col_cfg);
        // d->qpos now holds the final pose of the trajectory

        if (have_markers && viewer.enabled() && !trajectory.empty()) {
            for (const auto& q : trajectory) {
                mju_copy(d->qpos, q.data(), m->nq);
                mju_zero(d->qvel, m->nv);
                mj_forward(m, d);
                if (!viewer.render(d)) {
                    running = false;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(kPlaybackSleepMs));
            }
        }

        // Post-motion clearance guard
        mj_forward(m, d);
        const double final_dist = minContactDist(d);
        bool success = !trajectory.empty();

        if (std::isfinite(final_dist) && final_dist < kMinClearance) {
            std::cerr << "Final pose below clearance (" << final_dist
                      << " m); restoring start pose.\n";
            mju_copy(d->qpos, start_q.data(), m->nq);
            mju_zero(d->qvel, m->nv);
            mj_forward(m, d);
            success = false;
        }

        std::cout << "Result: " << (success ? "SUCCESS" : "FAILED")
                  << " | steps=" << trajectory.size() << "\n";

        // Flash success/failure colour in viewer
        if (have_markers && viewer.enabled()) {
            const float fr = success ? 0.2f : 0.95f;
            const float fg = success ? 0.95f : 0.2f;
            viewer.setGoalSpheres(goal_l, goal_r, 0.10, fr, fg, 0.2f, 0.90f);
            for (int i = 0; i < kFlashFrames && running; ++i) {
                if (!viewer.render(d)) running = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(kFlashSleepMs));
            }
        }
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}