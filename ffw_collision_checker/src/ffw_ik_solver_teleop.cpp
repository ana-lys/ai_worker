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
#include <deque>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mujoco/mujoco.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <unordered_set>
#include <fstream>
#include <sstream>

// ============================================================
// Voxel Map Definitions
// ============================================================
const double VOXEL_SIZE = 0.05;

struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash {
    std::size_t operator()(const VoxelKey& k) const {
        std::size_t h1 = std::hash<int>{}(k.x);
        std::size_t h2 = std::hash<int>{}(k.y);
        std::size_t h3 = std::hash<int>{}(k.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// ============================================================
// Minimal GLFW / MuJoCo viewer
// ============================================================

class SimpleViewer {
public:
  explicit SimpleViewer(mjModel *model) : m_(model) {
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
    cam_.distance = 2.0;
    cam_.azimuth = 0.0; // Changed from 180 to view from back
    cam_.elevation = -60.0;
    cam_.type = mjCAMERA_FREE;
    cam_.lookat[0] = 0.75;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.65;

    mjv_defaultCamera(&cam_top_);
    cam_top_.distance = 2.0;
    cam_top_.azimuth = 180.0; // View from the front
    cam_top_.elevation = -15.0; // Straight on
    cam_top_.type = mjCAMERA_FREE;
    cam_top_.lookat[0] = 0.75;
    cam_top_.lookat[1] = 0.0;
    cam_top_.lookat[2] = 0.65;

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

  void addCustomGeoms(mjData *d) {
    // 1. Draw orange skeleton overlay
    for (int i = 1; i < m_->nbody; ++i) { // Skip world body
      int parent = m_->body_parentid[i];
      if (parent >= 0) {
        if (scn_.ngeom >= scn_.maxgeom) break;

        const mjtNum* p1 = d->xpos + 3 * parent;
        const mjtNum* p2 = d->xpos + 3 * i;

        // Skip drawing if the two points are exactly identical to avoid zero-length geom errors
        if (std::abs(p1[0]-p2[0]) < 1e-4 && std::abs(p1[1]-p2[1]) < 1e-4 && std::abs(p1[2]-p2[2]) < 1e-4) {
            continue;
        }

        mjvGeom *g = &scn_.geoms[scn_.ngeom++];
        mjv_connector(g, mjGEOM_CAPSULE, 0.015, p1, p2); // 1.5cm thickness
        
        // Vibrant Orange, mostly opaque
        g->rgba[0] = 1.0f;
        g->rgba[1] = 0.45f;
        g->rgba[2] = 0.0f;
        g->rgba[3] = 0.85f;
        g->category = mjCAT_DECOR;
      }
    }

    // 2. Draw target spheres and axes
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
  }

  bool render(mjData *d) {
    if (!enabled_)
      return true;
    glfwPollEvents();
    if (glfwWindowShouldClose(window_))
      return false;

    mjrRect vp_full = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &vp_full.width, &vp_full.height);

    int h_half = vp_full.height / 2;
    mjrRect vp_top = {0, h_half, vp_full.width, vp_full.height - h_half};
    mjrRect vp_bot = {0, 0, vp_full.width, h_half};

    // --- Render Top View ---
    mjv_updateScene(m_, d, &opt_, nullptr, &cam_top_, mjCAT_ALL, &scn_);
    addCustomGeoms(d);
    mjr_render(vp_top, &scn_, &con_);

    // --- Render Bottom View ---
    mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    addCustomGeoms(d);
    mjr_render(vp_bot, &scn_, &con_);

    // --- Overlay EE Positions ---
    int id_l = mj_name2id(m_, mjOBJ_SITE, "left_gripper_site");
    int id_r = mj_name2id(m_, mjOBJ_SITE, "right_gripper_site");
    if (id_l >= 0 && id_r >= 0) {
      char text_l[256];
      char text_r[256];
      const mjtNum* pl = d->site_xpos + 3 * id_l;
      const mjtNum* pr = d->site_xpos + 3 * id_r;
      snprintf(text_l, sizeof(text_l), "LEFT EE XYZ:\nX: %7.3f\nY: %7.3f\nZ: %7.3f", pl[0], pl[1], pl[2]);
      snprintf(text_r, sizeof(text_r), "RIGHT EE XYZ:\nX: %7.3f\nY: %7.3f\nZ: %7.3f", pr[0], pr[1], pr[2]);
      mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, vp_full, text_l, nullptr, &con_);
      mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, vp_full, text_r, nullptr, &con_);
    }

    glfwSwapBuffers(window_);
    return true;
  }

  void drawAxes(const Eigen::Isometry3d &pose, float alpha) {
    if (scn_.ngeom + 3 > scn_.maxgeom)
      return;

    double length = 0.15;
    double radius = 0.008;
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
  mjvCamera cam_top_;
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
// TeleopNode
// ============================================================

using std::placeholders::_1;

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode() : Node("ffw_ik_solver_teleop") {

    pose_sub_l_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/spacemouse/left/ee_target_pose", 10,
        std::bind(&TeleopNode::pose_callback_l, this, _1));

    pose_sub_r_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/spacemouse/right/ee_target_pose", 10,
        std::bind(&TeleopNode::pose_callback_r, this, _1));

    joy_sub_l_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/left/joy", 10, std::bind(&TeleopNode::joy_callback_l, this, _1));

    joy_sub_r_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/right/joy", 10, std::bind(&TeleopNode::joy_callback_r, this, _1));

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    swap_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TeleopNode::swap_check_callback, this));

    std::vector<double> ori_vals = {-M_PI/2, -M_PI/4, 0, M_PI/4, M_PI/2};
    for (double r : ori_vals) {
        for (double p : ori_vals) {
            for (double yw : ori_vals) {
                Eigen::Quaterniond q_r = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
                                       * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(yw, Eigen::Vector3d::UnitZ());
                target_quats_r_.push_back(q_r);
                
                Eigen::Quaterniond q_l = Eigen::AngleAxisd(-r, Eigen::Vector3d::UnitX())
                                       * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(-yw, Eigen::Vector3d::UnitZ());
                target_quats_l_.push_back(q_l);
            }
        }
    }

    // Load Voxel Map
    std::string bin_path = ament_index_cpp::get_package_share_directory("ffw_collision_checker") + "/../../../../src/ai_worker/ffw_collision_checker/explore/pareto_boundary_voxels.bin";
    std::ifstream in(bin_path, std::ios::binary);
    if (!in.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open pareto_boundary_voxels.bin at %s. Try running ffw_workspace_explorer first.", bin_path.c_str());
    } else {
        std::vector<float> row(129);
        while (in.read(reinterpret_cast<char*>(row.data()), row.size() * sizeof(float))) {
            float x = row[0];
            float y = row[1];
            float z = row[2];
            float versatility = row[3];
            
            if (versatility > 0.0f) {
                int ix = static_cast<int>(std::floor(x / VOXEL_SIZE));
                int iy = static_cast<int>(std::floor(y / VOXEL_SIZE));
                int iz = static_cast<int>(std::floor(z / VOXEL_SIZE));
                
                min_ix_ = std::min(min_ix_, ix);
                max_ix_ = std::max(max_ix_, ix);
                min_iy_ = std::min(min_iy_, iy);
                max_iy_ = std::max(max_iy_, iy);
                min_iz_ = std::min(min_iz_, iz);
                max_iz_ = std::max(max_iz_, iz);
                
                std::vector<float> scores(row.begin() + 4, row.end());
                voxel_scores_r_[{ix, iy, iz}] = scores;
                voxel_scores_l_[{ix, -iy, iz}] = scores;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded Voxel Map (Binary): %zu valid right voxels. Bounds: X[%d, %d] Y[%d, %d] Z[%d, %d]", voxel_scores_r_.size(), min_ix_, max_ix_, min_iy_, max_iy_, min_iz_, max_iz_);
    }

    RCLCPP_INFO(this->get_logger(),
                "SpaceMouse IK Teleop started! Listening to both arms.");
  }

  void set_initial_poses(const Eigen::Isometry3d &l,
                         const Eigen::Isometry3d &r) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    initial_l_ = l;
    initial_r_ = r;
    target_l_ = l;
    target_r_ = r;
  }

  void get_targets(Eigen::Isometry3d &l, Eigen::Isometry3d &r) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    l = target_l_;
    r = target_r_;
  }

  void publish_joints(mjModel *m, mjData *d) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    for (int i = 0; i < m->njnt; ++i) {
      if (m->jnt_type[i] == mjJNT_HINGE || m->jnt_type[i] == mjJNT_SLIDE) {
        const char *name = mj_id2name(m, mjOBJ_JOINT, i);
        if (name) {
          msg.name.push_back(name);
          msg.position.push_back(d->qpos[m->jnt_qposadr[i]]);
        }
      }
    }
    joint_pub_->publish(msg);
  }

private:
  void pose_callback_l(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z);
    Eigen::Quaterniond rot(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rot_mat = rot.toRotationMatrix();

    if (first_msg_l_) {
      last_mapper_l_trans_ = trans;
      last_mapper_l_rot_ = rot_mat;
      first_msg_l_ = false;
    }

    Eigen::Vector3d delta_trans = trans - last_mapper_l_trans_;
    Eigen::Matrix3d delta_rot = rot_mat * last_mapper_l_rot_.transpose();

    last_mapper_l_trans_ = trans;
    last_mapper_l_rot_ = rot_mat;

    if (!swapped_) {
      Eigen::Vector3d prospective_trans = initial_l_.translation() + accum_l_trans_ + delta_trans;
      int ix = static_cast<int>(std::floor(prospective_trans.x() / VOXEL_SIZE));
      int iy = static_cast<int>(std::floor(prospective_trans.y() / VOXEL_SIZE));
      int iz = static_cast<int>(std::floor(prospective_trans.z() / VOXEL_SIZE));
      VoxelKey key = {ix, iy, iz};
      
      Eigen::Matrix3d prospective_rot_mat = delta_rot * accum_l_rot_ * initial_l_.linear();
      Eigen::Quaterniond prospective_quat(prospective_rot_mat);
      
      bool rejected = false;
      auto it = voxel_scores_l_.find(key);
      Eigen::Matrix3d relative_rot = delta_rot * accum_l_rot_;
      Eigen::AngleAxisd angle_axis(relative_rot);

      bool trans_rejected = false;
      bool rot_rejected = false;

      if (!l_in_safe_region_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Left Arm searching safe region. RelPos: %.3f %.3f %.3f", prospective_trans.x(), prospective_trans.y(), prospective_trans.z());
          if (it != voxel_scores_l_.end()) {
              l_in_safe_region_ = true;
              RCLCPP_INFO(this->get_logger(), "Left arm entered safe region. XYZ Limiter ENABLED.");
          }
      } else {
          // Hard boundary check for XYZ
          if (ix < min_ix_ || ix > max_ix_ || iy > -min_iy_ || iy < -max_iy_ || iz < min_iz_ || iz > max_iz_) {
              trans_rejected = true;
          }
          // Always apply angle limit
          if (std::abs(angle_axis.angle()) > M_PI / 2.0) {
              rot_rejected = true;
          }
      }
      
      if (!trans_rejected) {
          accum_l_trans_ += delta_trans;
          target_l_.translation() = prospective_trans;
      }
      if (!rot_rejected) {
          accum_l_rot_ = delta_rot * accum_l_rot_;
          target_l_.linear() = prospective_rot_mat;
      }
    } else {
      Eigen::Vector3d prospective_trans = initial_r_.translation() + accum_r_trans_ + delta_trans;
      int ix = static_cast<int>(std::floor(prospective_trans.x() / VOXEL_SIZE));
      int iy = static_cast<int>(std::floor(prospective_trans.y() / VOXEL_SIZE));
      int iz = static_cast<int>(std::floor(prospective_trans.z() / VOXEL_SIZE));
      VoxelKey key = {ix, iy, iz};
      
      Eigen::Matrix3d prospective_rot_mat = delta_rot * accum_r_rot_ * initial_r_.linear();
      Eigen::Quaterniond prospective_quat(prospective_rot_mat);
      
      bool rejected = false;
      auto it = voxel_scores_r_.find(key);
      Eigen::Matrix3d relative_rot = delta_rot * accum_r_rot_;
      Eigen::AngleAxisd angle_axis(relative_rot);

      bool trans_rejected = false;
      bool rot_rejected = false;

      if (!r_in_safe_region_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Right Arm searching safe region. RelPos: %.3f %.3f %.3f", prospective_trans.x(), prospective_trans.y(), prospective_trans.z());
          if (it != voxel_scores_r_.end()) {
              r_in_safe_region_ = true;
              RCLCPP_INFO(this->get_logger(), "Right arm entered safe region. XYZ Limiter ENABLED.");
          }
      } else {
          // Hard boundary check for XYZ
          if (ix < min_ix_ || ix > max_ix_ || iy < min_iy_ || iy > max_iy_ || iz < min_iz_ || iz > max_iz_) {
              trans_rejected = true;
          }
          // Always apply angle limit
          if (std::abs(angle_axis.angle()) > M_PI / 2.0) {
              rot_rejected = true;
          }
      }
      
      if (!trans_rejected) {
          accum_r_trans_ += delta_trans;
          target_r_.translation() = prospective_trans;
      }
      if (!rot_rejected) {
          accum_r_rot_ = delta_rot * accum_r_rot_;
          target_r_.linear() = prospective_rot_mat;
      }
    }
  }

  void pose_callback_r(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z);
    Eigen::Quaterniond rot(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rot_mat = rot.toRotationMatrix();

    if (first_msg_r_) {
      last_mapper_r_trans_ = trans;
      last_mapper_r_rot_ = rot_mat;
      first_msg_r_ = false;
    }

    Eigen::Vector3d delta_trans = trans - last_mapper_r_trans_;
    Eigen::Matrix3d delta_rot = rot_mat * last_mapper_r_rot_.transpose();

    last_mapper_r_trans_ = trans;
    last_mapper_r_rot_ = rot_mat;

    if (!swapped_) {
      Eigen::Vector3d prospective_trans = initial_r_.translation() + accum_r_trans_ + delta_trans;
      int ix = static_cast<int>(std::floor(prospective_trans.x() / VOXEL_SIZE));
      int iy = static_cast<int>(std::floor(prospective_trans.y() / VOXEL_SIZE));
      int iz = static_cast<int>(std::floor(prospective_trans.z() / VOXEL_SIZE));
      VoxelKey key = {ix, iy, iz};
      
      Eigen::Matrix3d prospective_rot_mat = delta_rot * accum_r_rot_ * initial_r_.linear();
      Eigen::Quaterniond prospective_quat(prospective_rot_mat);
      
      bool rejected = false;
      auto it = voxel_scores_r_.find(key);
      Eigen::Matrix3d relative_rot = delta_rot * accum_r_rot_;
      Eigen::AngleAxisd angle_axis(relative_rot);

      bool trans_rejected = false;
      bool rot_rejected = false;

      if (!r_in_safe_region_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Right Arm searching safe region. RelPos: %.3f %.3f %.3f", prospective_trans.x(), prospective_trans.y(), prospective_trans.z());
          if (it != voxel_scores_r_.end()) {
              r_in_safe_region_ = true;
              RCLCPP_INFO(this->get_logger(), "Right arm entered safe region. XYZ Limiter ENABLED.");
          }
      } else {
          // Hard boundary check for XYZ
          if (ix < min_ix_ || ix > max_ix_ || iy < min_iy_ || iy > max_iy_ || iz < min_iz_ || iz > max_iz_) {
              trans_rejected = true;
          }
          // Always apply angle limit
          if (std::abs(angle_axis.angle()) > M_PI / 2.0) {
              rot_rejected = true;
          }
      }
      
      if (!trans_rejected) {
          accum_r_trans_ += delta_trans;
          target_r_.translation() = prospective_trans;
      }
      if (!rot_rejected) {
          accum_r_rot_ = delta_rot * accum_r_rot_;
          target_r_.linear() = prospective_rot_mat;
      }
    } else {
      Eigen::Vector3d prospective_trans = initial_l_.translation() + accum_l_trans_ + delta_trans;
      int ix = static_cast<int>(std::floor(prospective_trans.x() / VOXEL_SIZE));
      int iy = static_cast<int>(std::floor(prospective_trans.y() / VOXEL_SIZE));
      int iz = static_cast<int>(std::floor(prospective_trans.z() / VOXEL_SIZE));
      VoxelKey key = {ix, iy, iz};
      
      Eigen::Matrix3d prospective_rot_mat = delta_rot * accum_l_rot_ * initial_l_.linear();
      Eigen::Quaterniond prospective_quat(prospective_rot_mat);
      
      bool rejected = false;
      auto it = voxel_scores_l_.find(key);
      Eigen::Matrix3d relative_rot = delta_rot * accum_l_rot_;
      Eigen::AngleAxisd angle_axis(relative_rot);

      bool trans_rejected = false;
      bool rot_rejected = false;

      if (!l_in_safe_region_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Left Arm searching safe region. RelPos: %.3f %.3f %.3f", prospective_trans.x(), prospective_trans.y(), prospective_trans.z());
          if (it != voxel_scores_l_.end()) {
              l_in_safe_region_ = true;
              RCLCPP_INFO(this->get_logger(), "Left arm entered safe region. XYZ Limiter ENABLED.");
          }
      } else {
          // Hard boundary check for XYZ
          if (ix < min_ix_ || ix > max_ix_ || iy > -min_iy_ || iy < -max_iy_ || iz < min_iz_ || iz > max_iz_) {
              trans_rejected = true;
          }
          // Always apply angle limit
          if (std::abs(angle_axis.angle()) > M_PI / 2.0) {
              rot_rejected = true;
          }
      }
      
      if (!trans_rejected) {
          accum_l_trans_ += delta_trans;
          target_l_.translation() = prospective_trans;
      }
      if (!rot_rejected) {
          accum_l_rot_ = delta_rot * accum_l_rot_;
          target_l_.linear() = prospective_rot_mat;
      }
    }
  }

  void joy_callback_l(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() >= 2) {
      left_btn0_ = (msg->buttons[0] == 1);
      left_btn1_ = (msg->buttons[1] == 1);
    }
  }

  void joy_callback_r(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() >= 2) {
      right_btn0_ = (msg->buttons[0] == 1);
      right_btn1_ = (msg->buttons[1] == 1);
    }
  }

  void swap_check_callback() {
    auto now = this->now();
    if (left_btn0_ && right_btn0_) {
      if (swap_timer_start_.nanoseconds() == 0) {
        swap_timer_start_ = now;
        RCLCPP_INFO(this->get_logger(),
                    "Both Left buttons held. Starting 5s swap timer...");
      } else if ((now - swap_timer_start_).seconds() >= 5.0) {
        if ((now - last_swap_time_).seconds() >= 5.0) {
          swapped_ = !swapped_;
          last_swap_time_ = now;
          RCLCPP_INFO(this->get_logger(),
                      "SWAP TRIGGERED! ARMS HAVE BEEN SWAPPED.");
        }
      }
    } else {
      if (swap_timer_start_.nanoseconds() != 0 &&
          (now - swap_timer_start_).seconds() < 5.0) {
        RCLCPP_INFO(this->get_logger(), "Swap cancelled. Buttons released.");
      }
      swap_timer_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }

  Eigen::Isometry3d initial_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d initial_r_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_r_ = Eigen::Isometry3d::Identity();
  std::mutex pose_mutex_;

  bool first_msg_l_ = true;
  bool first_msg_r_ = true;
  Eigen::Vector3d last_mapper_l_trans_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d last_mapper_l_rot_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d last_mapper_r_trans_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d last_mapper_r_rot_ = Eigen::Matrix3d::Identity();

  Eigen::Vector3d accum_l_trans_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d accum_l_rot_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d accum_r_trans_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d accum_r_rot_ = Eigen::Matrix3d::Identity();

  bool l_in_safe_region_ = false;
  bool r_in_safe_region_ = false;
  bool swapped_ = false;
  bool left_btn0_ = false, left_btn1_ = false;
  bool right_btn0_ = false, right_btn1_ = false;
  rclcpp::Time swap_timer_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_swap_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  std::unordered_map<VoxelKey, std::vector<float>, VoxelHash> voxel_scores_l_;
  
  int min_ix_ = 9999, max_ix_ = -9999;
  int min_iy_ = 9999, max_iy_ = -9999;
  int min_iz_ = 9999, max_iz_ = -9999;
  std::unordered_map<VoxelKey, std::vector<float>, VoxelHash> voxel_scores_r_;
  
  std::vector<Eigen::Quaterniond> target_quats_r_;
  std::vector<Eigen::Quaterniond> target_quats_l_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_l_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_r_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_l_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_r_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr swap_check_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopNode>();

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

  // Make the robot body extremely transparent so the skeleton is visible
  for (int i = 0; i < m->ngeom; ++i) {
    if (m->geom_bodyid[i] > 0) { // > 0 means it's not the static world body
      float alpha = 0.15f; // 15% opacity for normal arm links
      const char *body_name = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[i]);
      if (body_name) {
        std::string name(body_name);
        // Lower opacity almost completely for head, neck/torso equivalents to avoid blocking the view
        if (name.find("head") != std::string::npos ||
            name.find("arm_base_link") != std::string::npos ||
            name.find("pole") != std::string::npos ||
            name.find("drive") != std::string::npos) {
          alpha = 0.02f; // 2% opacity (basically invisible)
        }
      }
      m->geom_rgba[4 * i + 3] = alpha;
    }
  }

  mju_zero(d->qpos, m->nq);
  mju_zero(d->qvel, m->nv);
  if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE)
    d->qpos[3] = 1.0;
  mj_forward(m, d);

  int left_id = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
  int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");

  Eigen::Isometry3d init_l = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d init_r = Eigen::Isometry3d::Identity();
  if (left_id >= 0) {
    init_l.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
    init_l.linear() =
        Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
            d->site_xmat + 9 * left_id)
            .cast<double>();
  }
  if (right_id >= 0) {
    init_r.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
    init_r.linear() =
        Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
            d->site_xmat + 9 * right_id)
            .cast<double>();
  }
  node->set_initial_poses(init_l, init_r);

  ffw_ik::SolverConfig solver_cfg;
  solver_cfg.damping = 2e-3;
  solver_cfg.step_size = 0.15;
  solver_cfg.tolerance = 5e-3;
  solver_cfg.track_orientation = true;

  // Loosen stall conditions to prevent giving up during slow teleop
  solver_cfg.ee_improvement_rate = 1e-5;
  solver_cfg.ee_window = 15;

  // Increase orientation weight significantly so it doesn't get suppressed by
  // damping
  solver_cfg.ori_weight = 0.5;

  ffw_ik::CollisionCostConfig col_cfg;
  col_cfg.collision_margin = 0.10;
  col_cfg.weight_scale = 0.005;

  ffw_ik::IKSolver solver(m);
  SimpleViewer viewer(m);

  std::deque<double> err_hist, dist_hist;

  // Start ROS thread
  std::thread ros_thread([&node]() { rclcpp::spin(node); });

  // Main interaction loop
  int step_counter = 0;
  Eigen::Isometry3d prev_target_l = init_l;
  Eigen::Isometry3d prev_target_r = init_r;

  while (viewer.enabled() && rclcpp::ok()) {
    Eigen::Isometry3d current_target_l, current_target_r;
    node->get_targets(current_target_l, current_target_r);

    // Clear early-stopping history if the target has moved
    if (!current_target_l.isApprox(prev_target_l, 1e-4) ||
        !current_target_r.isApprox(prev_target_r, 1e-4)) {
      err_hist.clear();
    }
    prev_target_l = current_target_l;
    prev_target_r = current_target_r;

    // Run one gradient step of the IK solver
    ffw_ik::StepResult res =
        solver.solveStep(d, current_target_l, current_target_r, solver_cfg,
                         col_cfg, err_hist, dist_hist);

    // Print solver status every 20 ticks or if stalled
    // if (++step_counter % 20 == 0 || res.stalled) {
    //   std::cout << "\n--- Step " << step_counter << " ---" << std::endl;
    //   solver.printStep(step_counter, res);
    // 
    //   Eigen::Isometry3d curr_r = Eigen::Isometry3d::Identity();
    //   curr_r.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
    //   double pos_err =
    //       (curr_r.translation() - current_target_r.translation()).norm();
    //   std::cout << "Target R pos: "
    //             << current_target_r.translation().transpose() << std::endl;
    //   std::cout << "Current R pos: " << curr_r.translation().transpose()
    //             << std::endl;
    //   std::cout << "Pos error: " << pos_err << std::endl;
    //   if (res.stalled) {
    //     std::cout << "[WARNING] Solver STALLED!" << std::endl;
    //   }
    // }

    // Ensure simulation state is updated for rendering
    mj_forward(m, d);

    viewer.setGoalPose(current_target_l, current_target_r,
                       solver_cfg.track_orientation);
    if (!viewer.render(d)) {
      rclcpp::shutdown();
      break;
    }

    node->publish_joints(m, d);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(10)); // 100 Hz simulation/render loop
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  ros_thread.join();

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}