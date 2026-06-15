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

#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mujoco/mujoco.h>
#include <random>
#include <string>
#include <thread>
#include <vector>

// ============================================================
// Minimal GLFW / MuJoCo viewer
// ============================================================

class SimpleViewer {
public:
  explicit SimpleViewer(mjModel *model, bool headless = false) : m_(model) {
    if (headless) {
      enabled_ = false;
      return;
    }
    if (!glfwInit()) {
      std::cerr << "[Viewer] GLFW init failed.\n";
      return;
    }

    window_ = glfwCreateWindow(1200, 900, "FFW IK Test", nullptr, nullptr);
    if (!window_) {
      std::cerr << "[Viewer] Window creation failed.\n";
      glfwTerminate();
      return;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam_);
    cam_.distance = 3.0;
    cam_.azimuth = 180.0;
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
    if (window_)
      glfwDestroyWindow(window_);
    glfwTerminate();
  }

  bool render(mjData *d) {
    if (!enabled_)
      return true;
    glfwPollEvents();
    if (glfwWindowShouldClose(window_))
      return false;

    mjrRect vp = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &vp.width, &vp.height);
    mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

    if (spheres_enabled_ && scn_.ngeom + 12 <= scn_.maxgeom) {
      if (!track_orientation_) {
        const mjtNum size[3] = {sphere_r_, sphere_r_, sphere_r_};
        const mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        mjvGeom *gl = &scn_.geoms[scn_.ngeom++];
        mjv_initGeom(gl, mjGEOM_SPHERE, size, pos_l_, mat, rgba_);
        gl->category = mjCAT_DECOR;

        mjvGeom *gr = &scn_.geoms[scn_.ngeom++];
        mjv_initGeom(gr, mjGEOM_SPHERE, size, pos_r_, mat, rgba_);
        gr->category = mjCAT_DECOR;
      } else {
        drawAxes(pose_l_, 0.5f);
        drawAxes(pose_r_, 0.5f);

        int id_l = mj_name2id(m_, mjOBJ_SITE, "left_gripper_site");
        int id_r = mj_name2id(m_, mjOBJ_SITE, "right_gripper_site");
        if (id_l >= 0 && id_r >= 0) {
          Eigen::Isometry3d curr_l = Eigen::Isometry3d::Identity();
          Eigen::Isometry3d curr_r = Eigen::Isometry3d::Identity();
          curr_l.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * id_l);
          curr_r.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * id_r);
          curr_l.linear() =
              Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
                  d->site_xmat + 9 * id_l)
                  .cast<double>();
          curr_r.linear() =
              Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
                  d->site_xmat + 9 * id_r)
                  .cast<double>();
          drawAxes(curr_l, 1.0f);
          drawAxes(curr_r, 1.0f);
        }
      }
    }

    mjr_render(vp, &scn_, &con_);
    glfwSwapBuffers(window_);
    return true;
  }

  void drawAxes(const Eigen::Isometry3d &pose, float alpha) {
    if (scn_.ngeom + 3 > scn_.maxgeom)
      return;

    double length = 0.08;
    double radius = 0.003;
    mjtNum mat[9];
    Eigen::Map<Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> mat_map(mat);
    mat_map = pose.linear().cast<mjtNum>();

    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3d offset = Eigen::Vector3d::Zero();
      offset[i] = length / 2.0;
      Eigen::Vector3d center = pose.translation() + pose.linear() * offset;

      mjtNum cpos[3] = {center.x(), center.y(), center.z()};
      mjtNum size[3] = {radius, radius, radius};
      size[i] = length / 2.0;

      float rgba[4] = {0.f, 0.f, 0.f, alpha};
      rgba[i] = 1.0f; // RGB for XYZ

      mjvGeom *g = &scn_.geoms[scn_.ngeom++];
      mjv_initGeom(g, mjGEOM_BOX, size, cpos, mat, rgba);
      g->category = mjCAT_DECOR;
    }
  }

  void setGoalSpheres(const Eigen::Vector3d &l, const Eigen::Vector3d &r,
                      double diameter = 0.09, float fr = 1.0f, float fg = 0.65f,
                      float fb = 0.0f, float fa = 0.85f) {
    spheres_enabled_ = true;
    track_orientation_ = false;
    sphere_r_ = std::max(0.0, 0.5 * diameter);
    for (int i = 0; i < 3; ++i) {
      pos_l_[i] = l[i];
      pos_r_[i] = r[i];
    }
    rgba_[0] = fr;
    rgba_[1] = fg;
    rgba_[2] = fb;
    rgba_[3] = fa;
  }

  void setGoalPose(const Eigen::Isometry3d &l, const Eigen::Isometry3d &r,
                   bool track_ori) {
    track_orientation_ = track_ori;
    if (!track_ori) {
      setGoalSpheres(l.translation(), r.translation());
    } else {
      pose_l_ = l;
      pose_r_ = r;
      spheres_enabled_ = true;
    }
  }

  bool enabled() const { return enabled_; }

private:
  mjModel *m_ = nullptr;
  GLFWwindow *window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  bool enabled_ = false;
  bool spheres_enabled_ = false;
  bool track_orientation_ = false;
  mjtNum pos_l_[3] = {};
  mjtNum pos_r_[3] = {};
  Eigen::Isometry3d pose_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_r_ = Eigen::Isometry3d::Identity();
  mjtNum sphere_r_ = 0.045;
  float rgba_[4] = {1.f, 0.65f, 0.f, 0.85f};
};

// ============================================================
// Pose sampling helpers
// ============================================================

static double minContactDist(const mjData *d) {
  if (!d || d->ncon <= 0)
    return 0.30;
  double m = 0.30;
  for (int i = 0; i < d->ncon; ++i)
    m = std::min(m, static_cast<double>(d->contact[i].dist));
  return m;
}

static bool isPoseLegal(mjModel *m, mjData *d, const Eigen::VectorXd &q,
                        double min_clearance, double *out_dist = nullptr) {
  std::vector<mjtNum> qbak(m->nq), vbak(m->nv);
  mju_copy(qbak.data(), d->qpos, m->nq);
  mju_copy(vbak.data(), d->qvel, m->nv);

  mju_copy(d->qpos, q.data(), m->nq);
  mju_zero(d->qvel, m->nv);
  mj_forward(m, d);
  const double dist = minContactDist(d);
  const bool ok = !std::isfinite(dist) || dist >= min_clearance;
  if (out_dist)
    *out_dist = dist;

  mju_copy(d->qpos, qbak.data(), m->nq);
  mju_copy(d->qvel, vbak.data(), m->nv);
  mj_forward(m, d);
  return ok;
}

static Eigen::VectorXd sampleRandomPose(mjModel *m, const Eigen::VectorXd &ref,
                                        std::mt19937 &rng) {
  Eigen::VectorXd q = ref;
  for (int jid = 0; jid < m->njnt; ++jid) {
    const int jtype = m->jnt_type[jid];
    if (jtype != mjJNT_HINGE && jtype != mjJNT_SLIDE)
      continue;

    const int qadr = m->jnt_qposadr[jid];
    double lo, hi;
    if (m->jnt_limited[jid]) {
      lo = m->jnt_range[2 * jid];
      hi = m->jnt_range[2 * jid + 1];
      const double span = hi - lo;
      const double margin = 0.02 * span;
      lo = (span > 1e-6) ? lo + margin : lo;
      hi = (span > 1e-6) ? hi - margin : hi;
      if (lo > hi) {
        lo = m->jnt_range[2 * jid];
        hi = m->jnt_range[2 * jid + 1];
      }
    } else {
      lo = ref[qadr] - (jtype == mjJNT_HINGE ? M_PI : 0.25);
      hi = ref[qadr] + (jtype == mjJNT_HINGE ? M_PI : 0.25);
    }
    q[qadr] = std::uniform_real_distribution<double>(lo, hi)(rng);
  }
  return q;
}

static bool findLegalRandomPose(mjModel *m, mjData *d,
                                const Eigen::VectorXd &ref, std::mt19937 &rng,
                                double min_clearance, int max_tries,
                                Eigen::VectorXd &out_q, double &out_dist) {
  for (int t = 0; t < max_tries; ++t) {
    Eigen::VectorXd candidate = sampleRandomPose(m, ref, rng);
    double dist = std::numeric_limits<double>::infinity();
    if (isPoseLegal(m, d, candidate, min_clearance, &dist)) {
      out_q = std::move(candidate);
      out_dist = dist;
      return true;
    }
  }
  return false;
}

static bool gripperSitesAtPose(mjModel *m, mjData *d, const Eigen::VectorXd &q,
                               int left_id, int right_id,
                               Eigen::Isometry3d &out_l,
                               Eigen::Isometry3d &out_r) {
  if (left_id < 0 || right_id < 0)
    return false;

  std::vector<mjtNum> qbak(m->nq), vbak(m->nv);
  mju_copy(qbak.data(), d->qpos, m->nq);
  mju_copy(vbak.data(), d->qvel, m->nv);

  out_l.setIdentity();
  out_r.setIdentity();
  mju_copy(d->qpos, q.data(), m->nq);
  mju_zero(d->qvel, m->nv);
  mj_forward(m, d);
  out_l.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
  out_r.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
  out_l.linear() =
      Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
          d->site_xmat + 9 * left_id)
          .cast<double>();
  out_r.linear() =
      Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
          d->site_xmat + 9 * right_id)
          .cast<double>();

  mju_copy(d->qpos, qbak.data(), m->nq);
  mju_copy(d->qvel, vbak.data(), m->nv);
  mj_forward(m, d);
  return true;
}

struct SolverThreadResult {
  int type;
  std::vector<Eigen::VectorXd> trajectory;
  double pos_err = 0.0;
  double ori_err = 0.0;
  double ns_weight = 0.0;
  bool success = false;
  double final_dist = 0.0;
  double time_ms = 0.0;
  double path_length = 0.0;
  std::string stop_reason;
};

void run_benchmark_parallel(mjModel* m, int num_tests, const ffw_ik::SolverConfig& solver_cfg, const ffw_ik::CollisionCostConfig& col_cfg, double kMinClearance, int kMaxSampleTries) {
  std::cout << "Starting parallel benchmark with " << num_tests << " motions across 14 threads...\n";
  std::ofstream csv_file("benchmark_results.csv");
  csv_file << "motion_idx,solver_type,success,steps,pos_err,ori_err,ns_weight,min_dist,path_len,time_ms,reason\n";

  std::atomic<int> motion_counter{0};
  std::mutex csv_mutex;
  
  int num_threads = 14;
  std::vector<std::thread> workers;

  const int left_id = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
  const int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");

  for (int t = 0; t < num_threads; ++t) {
    workers.emplace_back([&]() {
      mjData* d = mj_makeData(m);
      std::mt19937 rng(std::random_device{}() + std::hash<std::thread::id>{}(std::this_thread::get_id()));

      while (true) {
        int motion = motion_counter.fetch_add(1);
        if (motion >= num_tests) break;

        Eigen::VectorXd start_q(m->nq);
        mju_zero(d->qpos, m->nq);
        if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE) d->qpos[3] = 1.0;
        mj_forward(m, d);
        const Eigen::Map<const Eigen::VectorXd> zero_q(d->qpos, m->nq);
        
        double start_dist = 0.0;
        if (!findLegalRandomPose(m, d, zero_q, rng, kMinClearance, kMaxSampleTries, start_q, start_dist)) {
            continue;
        }

        Eigen::VectorXd goal_q(m->nq);
        double goal_dist = 0.0;
        if (!findLegalRandomPose(m, d, start_q, rng, kMinClearance, kMaxSampleTries, goal_q, goal_dist)) {
            continue;
        }

        Eigen::Isometry3d goal_l = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d goal_r = Eigen::Isometry3d::Identity();
        gripperSitesAtPose(m, d, goal_q, left_id, right_id, goal_l, goal_r);

        auto run_solver_seq = [&](int type) -> SolverThreadResult {
          SolverThreadResult res;
          res.type = type;

          mjData *thread_d = mj_makeData(m);
          mju_copy(thread_d->qpos, start_q.data(), m->nq);
          mj_forward(m, thread_d);

          ffw_ik::SolverConfig thread_cfg = solver_cfg;
          thread_cfg.nullspace_type = type;

          ffw_ik::IKSolver thread_solver(m);
          auto t_start = std::chrono::high_resolution_clock::now();
          res.trajectory = thread_solver.solve(thread_d, goal_l, goal_r, thread_cfg, col_cfg);
          auto t_end = std::chrono::high_resolution_clock::now();
          res.time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

          if (!res.trajectory.empty()) {
            res.path_length = 0.0;
            Eigen::VectorXd curr = start_q;
            for (const auto &q : res.trajectory) {
              res.path_length += (q - curr).norm();
              curr = q;
            }
          }

          res.final_dist = minContactDist(thread_d);
          res.success = !res.trajectory.empty();
          res.ns_weight = thread_solver.getLastActiveNullSpaceWeight();

          if (res.trajectory.empty()) {
            res.stop_reason = "IMMEDIATE_FAIL";
          } else if (res.trajectory.size() == thread_cfg.max_steps) {
            res.stop_reason = "MAX_STEPS";
          } else {
            res.stop_reason = "HALTED";
          }

          if (std::isfinite(res.final_dist) && res.final_dist < kMinClearance) {
            res.success = false;
            res.stop_reason = "COLLISION";
          }

          if (!res.trajectory.empty()) {
            Eigen::Isometry3d final_l, final_r;
            gripperSitesAtPose(m, thread_d, Eigen::Map<Eigen::VectorXd>(thread_d->qpos, m->nq), left_id, right_id, final_l, final_r);
            res.pos_err = (final_l.translation() - goal_l.translation()).norm() + (final_r.translation() - goal_r.translation()).norm();
            Eigen::AngleAxisd aa_l(goal_l.rotation() * final_l.rotation().transpose());
            Eigen::AngleAxisd aa_r(goal_r.rotation() * final_r.rotation().transpose());
            res.ori_err = std::abs(aa_l.angle()) + std::abs(aa_r.angle());

            if (res.pos_err > 0.05 || (thread_cfg.track_orientation && res.ori_err > 0.35)) {
              res.success = false;
              if (res.stop_reason == "HALTED") res.stop_reason = "STALLED_LOCAL_MINIMA";
              if (res.stop_reason == "MAX_STEPS") res.stop_reason = "MAX_STEPS_LOCAL_MINIMA";
            } else if (res.success) {
              if (res.stop_reason == "HALTED") res.stop_reason = "CONVERGED";
              if (res.stop_reason == "MAX_STEPS") res.stop_reason = "CONVERGED_AT_MAX";
            }
          }
          mj_deleteData(thread_d);
          return res;
        };

        std::vector<SolverThreadResult> results;
        results.push_back(run_solver_seq(0));
        results.push_back(run_solver_seq(1));
        results.push_back(run_solver_seq(2));
        results.push_back(run_solver_seq(3));

        {
          std::lock_guard<std::mutex> lock(csv_mutex);
          for (const auto& r : results) {
            csv_file << motion << "," << r.type << "," << (r.success ? 1 : 0) << ","
                     << r.trajectory.size() << "," << r.pos_err << "," << r.ori_err
                     << "," << r.ns_weight << "," << r.final_dist << ","
                     << r.path_length << "," << r.time_ms << "," << r.stop_reason << "\n";
          }
          if ((motion + 1) % 100 == 0) {
            std::cout << "\r[Benchmark] Completed " << (motion + 1) << " / " << num_tests << "..." << std::flush;
          }
        }
      }
      mj_deleteData(d);
    });
  }

  for (auto& w : workers) {
    w.join();
  }
  std::cout << "\n[Benchmark] Finished " << num_tests << " tests entirely!\n";
}

// ============================================================
// main
// ============================================================

int main(int argc, char **argv) {
  bool is_benchmark = false;
  int num_tests = 1000;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--benchmark")
      is_benchmark = true;
    else if (arg.find("--num-tests=") == 0) {
      num_tests = std::stoi(arg.substr(12));
    }
  }
  using ament_index_cpp::get_package_share_directory;
  const std::string xml_path =
      get_package_share_directory("ffw_collision_checker") +
      "/3rd_party/robotis_ffw/scene_inverse_kinematic.xml";

  char error[1000];
  mjModel *m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    std::cerr << "MuJoCo load error: " << error << "\n";
    return 1;
  }
  mjData *d = mj_makeData(m);

  mju_zero(d->qpos, m->nq);
  mju_zero(d->qvel, m->nv);
  if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE)
    d->qpos[3] = 1.0;
  mj_forward(m, d);

  // IK configuration
  ffw_ik::SolverConfig solver_cfg;
  solver_cfg.damping = 1e-3;
  solver_cfg.step_size = 0.15;
  solver_cfg.tolerance = 5e-3;
  solver_cfg.joint_vel_limit = 3.1;
  solver_cfg.max_steps = 150;
  solver_cfg.topk_contacts = 5;
  solver_cfg.ee_window = 10;
  solver_cfg.ee_improvement_rate = 0.02;
  solver_cfg.dist_window = 10;
  solver_cfg.dist_stability_thresh = 0.002;
  solver_cfg.dist_safe_ratio = 0.98;
  solver_cfg.early_convergence_obj = 0.87;
  solver_cfg.nullspace_amplitude = 0.8;

  ffw_ik::CollisionCostConfig col_cfg;
  col_cfg.collision_margin = 0.10;
  col_cfg.cbf_alpha = 1.0;
  col_cfg.weight_scale = 0.025;
  col_cfg.epsilon = 1e-1;

  constexpr double kMinClearance = 0.10;
  constexpr int kMaxSampleTries = 1500;
  constexpr int kPlaybackSleepMs = 20;
  constexpr int kFlashFrames = 10;
  constexpr int kFlashSleepMs = 10;

  if (kMinClearance < col_cfg.collision_margin) {
    std::cerr << "[test] FATAL: kMinClearance < col_cfg.collision_margin\n";
    mj_deleteData(d);
    mj_deleteModel(m);
    return 1;
  }

  if (is_benchmark) {
    run_benchmark_parallel(m, num_tests, solver_cfg, col_cfg, kMinClearance, kMaxSampleTries);
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
  }

  ffw_ik::IKSolver solver(m);
  SimpleViewer viewer(m, false);

  const int left_id = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
  const int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");

  std::mt19937 rng(std::random_device{}());

  if (viewer.enabled() && !viewer.render(d)) {
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
  }

  int motion = 0;
  bool running = true;

  while (running) {
    mj_forward(m, d);

    const Eigen::Map<const Eigen::VectorXd> start_q_map(d->qpos, m->nq);
    const Eigen::VectorXd start_q = start_q_map;
    const double start_dist = minContactDist(d);
    const bool start_legal =
        !std::isfinite(start_dist) || start_dist >= kMinClearance;

    // Sample a collision-free goal pose
    Eigen::VectorXd goal_q(m->nq);
    double goal_dist = std::numeric_limits<double>::infinity();
    if (!findLegalRandomPose(m, d, start_q, rng, kMinClearance, kMaxSampleTries,
                             goal_q, goal_dist)) {
      std::cerr << "Could not find a legal goal after " << kMaxSampleTries
                << " tries.\n";
      break;
    }

    ++motion;
    std::cout << "\n=== Motion " << motion << " ==="
              << " | start_dist=" << start_dist << " | goal_dist=" << goal_dist
              << "\n";

    // Forward-compute EE targets at the goal pose
    Eigen::Isometry3d goal_l = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d goal_r = Eigen::Isometry3d::Identity();
    const bool have_markers =
        gripperSitesAtPose(m, d, goal_q, left_id, right_id, goal_l, goal_r);

    if (have_markers && viewer.enabled()) {
      viewer.setGoalPose(goal_l, goal_r, solver_cfg.track_orientation);
      if (!viewer.render(d)) {
        running = false;
        continue;
      }
    }

    if (!start_legal) {
      std::cerr << "Skipping motion: start pose below clearance.\n";
      continue;
    }


    auto run_solver_thread = [&](int type) -> SolverThreadResult {
      SolverThreadResult res;
      res.type = type;

      mjData *thread_d = mj_makeData(m);
      mju_copy(thread_d->qpos, start_q.data(), m->nq);
      mj_forward(m, thread_d);

      ffw_ik::SolverConfig thread_cfg = solver_cfg;
      thread_cfg.nullspace_type = type;

      ffw_ik::IKSolver thread_solver(m);
      auto t_start = std::chrono::high_resolution_clock::now();
      res.trajectory =
          thread_solver.solve(thread_d, goal_l, goal_r, thread_cfg, col_cfg);
      auto t_end = std::chrono::high_resolution_clock::now();
      res.time_ms =
          std::chrono::duration<double, std::milli>(t_end - t_start).count();

      if (!res.trajectory.empty()) {
        res.path_length = 0.0;
        Eigen::VectorXd curr = start_q;
        for (const auto &q : res.trajectory) {
          res.path_length += (q - curr).norm();
          curr = q;
        }
      }

      res.final_dist = minContactDist(thread_d);
      res.success = !res.trajectory.empty();
      res.ns_weight = thread_solver.getLastActiveNullSpaceWeight();

      if (res.trajectory.empty()) {
        res.stop_reason = "IMMEDIATE_FAIL";
      } else if (res.trajectory.size() == thread_cfg.max_steps) {
        res.stop_reason = "MAX_STEPS";
      } else {
        res.stop_reason = "HALTED"; // Either early converged or stalled
      }

      if (std::isfinite(res.final_dist) && res.final_dist < kMinClearance) {
        res.success = false;
        res.stop_reason = "COLLISION";
      }

      if (!res.trajectory.empty()) {
        Eigen::Isometry3d final_l, final_r;
        gripperSitesAtPose(m, thread_d,
                           Eigen::Map<Eigen::VectorXd>(thread_d->qpos, m->nq),
                           left_id, right_id, final_l, final_r);
        res.pos_err = (final_l.translation() - goal_l.translation()).norm() +
                      (final_r.translation() - goal_r.translation()).norm();

        Eigen::AngleAxisd aa_l(goal_l.rotation() *
                               final_l.rotation().transpose());
        Eigen::AngleAxisd aa_r(goal_r.rotation() *
                               final_r.rotation().transpose());
        res.ori_err = std::abs(aa_l.angle()) + std::abs(aa_r.angle());

        if (res.pos_err > 0.05 ||
            (thread_cfg.track_orientation && res.ori_err > 0.35)) {
          res.success = false;
          if (res.stop_reason == "HALTED")
            res.stop_reason = "STALLED_LOCAL_MINIMA";
          if (res.stop_reason == "MAX_STEPS")
            res.stop_reason = "MAX_STEPS_LOCAL_MINIMA";
        } else if (res.success) {
          if (res.stop_reason == "HALTED")
            res.stop_reason = "CONVERGED";
          if (res.stop_reason == "MAX_STEPS")
            res.stop_reason = "CONVERGED_AT_MAX";
        }
      }

      mj_deleteData(thread_d);
      return res;
    };

    std::vector<int> solver_types_to_run = {
        3}; // Expandable list of solver configs!
    std::vector<std::future<SolverThreadResult>> futures;
    for (int type : solver_types_to_run) {
      futures.push_back(
          std::async(std::launch::async, run_solver_thread, type));
    }

    std::vector<SolverThreadResult> results;
    for (auto &f : futures) {
      results.push_back(f.get());
    }

    // Print comparative results
    auto print_res = [&](const SolverThreadResult &r) {
      std::cout << "Solver " << r.type
                << (r.type == 0
                        ? " (Sinusoidal): "
                        : (r.type == 1 ? " (Centering) : " 
                        : (r.type == 2 ? " (Anti-Sin)  : " 
                        : (r.type == 3 ? " (Manipulab) : " : " (Unknown)   : "))))
                << (r.success ? "SUCCESS" : "FAILED ")
                << " | steps=" << r.trajectory.size()
                << " | pos_err=" << std::fixed << std::setprecision(4)
                << r.pos_err << " | ori_err=" << std::fixed
                << std::setprecision(4) << r.ori_err
                << " | ns_weight=" << std::fixed << std::setprecision(4)
                << r.ns_weight << " | min_dist=" << std::fixed
                << std::setprecision(4) << r.final_dist
                << " | path_len=" << std::fixed << std::setprecision(4)
                << r.path_length << " | time_ms=" << std::fixed
                << std::setprecision(2) << r.time_ms
                << " | reason=" << r.stop_reason << "\n";

    };

    SolverThreadResult *best_res = nullptr;
    for (auto &res : results) {
      print_res(res);
      if (!best_res) {
        best_res = &res;
        continue;
      }
      if (res.success && !best_res->success) {
        best_res = &res; // Success always beats failure
      } else if (res.success && best_res->success) {
        // Both success: pick lowest combined error
        if (res.pos_err + res.ori_err < best_res->pos_err + best_res->ori_err) {
          best_res = &res;
        }
      } else if (!res.success && !best_res->success) {
        // Both failure: pick lowest combined error (now includes orientation!)
        if (res.pos_err + res.ori_err < best_res->pos_err + best_res->ori_err) {
          best_res = &res;
        }
      }
    }

    std::cout << "Best Solver: " << best_res->type << "\n";

    const auto &trajectory = best_res->trajectory;
    bool success = best_res->success;

    if (have_markers && viewer.enabled() && !trajectory.empty()) {
      for (const auto &q : trajectory) {
        mju_copy(d->qpos, q.data(), m->nq);
        mju_zero(d->qvel, m->nv);
        mj_forward(m, d);
        if (!viewer.render(d)) {
          running = false;
          break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kPlaybackSleepMs));
      }
    }

    // Post-motion clearance guard for the main state
    mj_forward(m, d);
    const double main_final_dist = minContactDist(d);
    if (!success ||
        (std::isfinite(main_final_dist) && main_final_dist < kMinClearance)) {
      std::cerr << "Restoring start pose (failed or below clearance).\n";
      mju_copy(d->qpos, start_q.data(), m->nq);
      mju_zero(d->qvel, m->nv);
      mj_forward(m, d);
    }

    // Flash success/failure colour in viewer
    if (have_markers && viewer.enabled()) {
      if (!solver_cfg.track_orientation) {
        const float fr = success ? 0.2f : 0.95f;
        const float fg = success ? 0.95f : 0.2f;
        viewer.setGoalSpheres(goal_l.translation(), goal_r.translation(), 0.10,
                              fr, fg, 0.2f, 0.90f);
      }
      for (int i = 0; i < kFlashFrames && running; ++i) {
        if (!viewer.render(d))
          running = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(kFlashSleepMs));
      }
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}