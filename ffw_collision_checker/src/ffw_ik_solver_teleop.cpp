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
#include <iomanip>
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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ffw_collision_checker/srv/toggle_joint_group.hpp"
#include "ffw_collision_checker/srv/save_load_pose.hpp"
#include "ffw_collision_checker/msg/collision_debug.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

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
    cam_top_.azimuth = 180.0;   // View from the front
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
    // 1. (Disabled) Orange skeleton overlay
    // The skeleton is no longer drawn so we can see the solid robot model instead.

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

    // 3. Draw collision spheres (small red spheres)
    for (const auto& ci : active_collisions_) {
        if (ci.dist > 0.155) continue; // Only draw if within collision margin
        if (scn_.ngeom + 2 > scn_.maxgeom) break;
        
        mjtNum size[3] = {0.02, 0.02, 0.02}; // small sphere (2cm)
        mjtNum mat[9] = {1,0,0, 0,1,0, 0,0,1};
        float rgba[4] = {1.0f, 0.0f, 0.0f, 0.8f}; // Red, slightly transparent (0.8 alpha)
        
        mjtNum p1[3] = {ci.p1.x(), ci.p1.y(), ci.p1.z()};
        mjvGeom *g1 = &scn_.geoms[scn_.ngeom++];
        mjv_initGeom(g1, mjGEOM_SPHERE, size, p1, mat, rgba);
        g1->category = mjCAT_DECOR;
        
        mjtNum p2[3] = {ci.p2.x(), ci.p2.y(), ci.p2.z()};
        mjvGeom *g2 = &scn_.geoms[scn_.ngeom++];
        mjv_initGeom(g2, mjGEOM_SPHERE, size, p2, mat, rgba);
        g2->category = mjCAT_DECOR;
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
      Eigen::Vector3d rpy_l = extract_rpy(pose_l_.linear());
      Eigen::Vector3d rpy_r = extract_rpy(pose_r_.linear());
      const mjtNum *pl = d->site_xpos + 3 * id_l;
      const mjtNum *pr = d->site_xpos + 3 * id_r;
      snprintf(text_l, sizeof(text_l),
               "LEFT EE XYZ:\nX: %7.3f\nY: %7.3f\nZ: %7.3f\nRoll:  "
               "%7.3f\nPitch: %7.3f\nYaw:   %7.3f",
               pl[0], pl[1], pl[2], rpy_l[0], rpy_l[1], rpy_l[2]);
      snprintf(text_r, sizeof(text_r),
               "RIGHT EE XYZ:\nX: %7.3f\nY: %7.3f\nZ: %7.3f\nRoll:  "
               "%7.3f\nPitch: %7.3f\nYaw:   %7.3f",
               pr[0], pr[1], pr[2], rpy_r[0], rpy_r[1], rpy_r[2]);
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
    pos_l_[0] = l.x();
    pos_l_[1] = l.y();
    pos_l_[2] = l.z();

    pos_r_[0] = r.x();
    pos_r_[1] = r.y();
    pos_r_[2] = r.z();

    sphere_r_ = std::max(0.0, 0.5 * diameter);
    rgba_[0] = fr;
    rgba_[1] = fg;
    rgba_[2] = fb;
    rgba_[3] = fa;
  }

  void setCollisions(const std::vector<ffw_ik::ContactInfo>& contacts) {
      active_collisions_ = contacts;
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

  static Eigen::Vector3d extract_rpy(const Eigen::Matrix3d &R) {
    double pitch = std::asin(std::clamp(R(0, 2), -1.0, 1.0));
    double yaw = std::atan2(-R(0, 1), R(0, 0));
    double roll = std::atan2(-R(1, 2), R(2, 2));
    return Eigen::Vector3d(roll, pitch, yaw);
  }

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
  std::vector<ffw_ik::ContactInfo> active_collisions_;
};

struct PoseState {
  Eigen::Vector3d target_l_pos = Eigen::Vector3d::Zero();
  Eigen::Matrix3d target_l_rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d target_r_pos = Eigen::Vector3d::Zero();
  Eigen::Matrix3d target_r_rot = Eigen::Matrix3d::Identity();
  double gripper_l_pos = 0.0;
  double gripper_r_pos = 0.0;
  double head_joint1_pos = 0.0;
  double head_joint2_pos = 0.0;
};

using std::placeholders::_1;

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode() : Node("ffw_ik_solver_teleop") {

    // Two goal-update functions (quest_teleop_plan §10 fix): the delta one
    // consumes spacemouse absolute goals as deltas (accum/rebase); the pose
    // one consumes quest absolute goals directly (no accum/rebase). joy_hand
    // publishes on whichever topic matches the current mode, so which function
    // runs is decided by which topic delivered the last goal.
    pose_sub_l_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/spacemouse/left/ee_target_pose", 10,
        std::bind(&TeleopNode::update_goal_delta_l, this, _1));

    pose_sub_r_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/spacemouse/right/ee_target_pose", 10,
        std::bind(&TeleopNode::update_goal_delta_r, this, _1));

    quest_pose_sub_l_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/quest/left/ee_target_pose", 10,
        std::bind(&TeleopNode::update_goal_pose_l, this, _1));

    quest_pose_sub_r_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/quest/right/ee_target_pose", 10,
        std::bind(&TeleopNode::update_goal_pose_r, this, _1));

    // Per-arm quest state feed (joy_hand publishes on every state change): lets
    // the leash pick 1.5cm during APPROACH vs 3cm in TRACK (§8b).
    quest_state_sub_l_ = this->create_subscription<std_msgs::msg::String>(
        "/quest/left/state", 10,
        std::bind(&TeleopNode::quest_state_cb_l, this, _1));

    quest_state_sub_r_ = this->create_subscription<std_msgs::msg::String>(
        "/quest/right/state", 10,
        std::bind(&TeleopNode::quest_state_cb_r, this, _1));

    // §11: live per-arm trigger feed — absolute gripper command in TRACK mode
    // (1 → close, 0 → open). Published every active quest tick by joy_hand.
    quest_trigger_sub_l_ = this->create_subscription<std_msgs::msg::Float32>(
        "/quest/left/trigger", 10,
        std::bind(&TeleopNode::quest_trigger_cb_l, this, _1));

    quest_trigger_sub_r_ = this->create_subscription<std_msgs::msg::Float32>(
        "/quest/right/trigger", 10,
        std::bind(&TeleopNode::quest_trigger_cb_r, this, _1));

    joy_sub_l_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/left/joy", 10, std::bind(&TeleopNode::joy_callback_l, this, _1));

    joy_sub_r_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/right/joy", 10, std::bind(&TeleopNode::joy_callback_r, this, _1));

    // hardware_mode=true  (default): subscribe to real robot /joint_states for
    // sync. hardware_mode=false (sim-only): drive MuJoCo internally only;
    // publish nothing to ROS.
    this->declare_parameter("hardware_mode", true);
    hardware_mode_ = this->get_parameter("hardware_mode").as_bool();

    // Dynamic joint group locking
    this->declare_parameter<bool>("left_arm_enabled", true);
    this->declare_parameter<bool>("right_arm_enabled", true);
    this->declare_parameter<bool>("lift_enabled", true);
    this->declare_parameter<bool>("collision_debug", true);
    // §10 slow->fast debug: print target_l/achieved/error each 0.2s during quest
    this->declare_parameter<bool>("quest_debug_solver", false);

    this->declare_parameter<std::string>("robot_model", "bg2");
    robot_model_ = this->get_parameter("robot_model").as_string();

    // Qpos-rail mode: "ee" (default) keeps spacemouse/quest control; "rail"
    // hands the tick to /qpos_rail. Launch-time default here; runtime flip via
    // either `ros2 param set` (below) or /teleop_goal_source (near real_joint_sub_).
    this->declare_parameter<std::string>("goal_source", "ee");
    if (this->get_parameter("goal_source").as_string() == "rail") {
      goal_source_ = GoalSource::RAIL;
    }

    left_arm_enabled_ = true;
    right_arm_enabled_ = true;
    lift_enabled_ = true;
    collision_debug_ = true;
    quest_debug_solver_ = this->get_parameter("quest_debug_solver").as_bool();

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &param : parameters) {
            if (param.get_name() == "left_arm_enabled") {
              std::lock_guard<std::mutex> lock(pose_mutex_);
              left_arm_enabled_ = param.as_bool();
              RCLCPP_INFO(this->get_logger(), "left_arm_enabled set to %s", left_arm_enabled_ ? "true" : "false");
            } else if (param.get_name() == "right_arm_enabled") {
              std::lock_guard<std::mutex> lock(pose_mutex_);
              right_arm_enabled_ = param.as_bool();
              RCLCPP_INFO(this->get_logger(), "right_arm_enabled set to %s", right_arm_enabled_ ? "true" : "false");
            } else if (param.get_name() == "lift_enabled") {
              std::lock_guard<std::mutex> lock(pose_mutex_);
              lift_enabled_ = param.as_bool();
              RCLCPP_INFO(this->get_logger(), "lift_enabled set to %s", lift_enabled_ ? "true" : "false");
            } else if (param.get_name() == "collision_debug") {
              std::lock_guard<std::mutex> lock(pose_mutex_);
              collision_debug_ = param.as_bool();
              RCLCPP_INFO(this->get_logger(), "collision_debug set to %s", collision_debug_ ? "true" : "false");
            } else if (param.get_name() == "goal_source") {
              const std::string v = param.as_string();
              if (v == "rail") {
                goal_source_ = GoalSource::RAIL;
              } else if (v == "ee") {
                goal_source_ = GoalSource::EE;
              } else {
                result.successful = false;
                result.reason = "goal_source must be 'ee' or 'rail'";
                continue;
              }
              RCLCPP_INFO(this->get_logger(), "goal_source set to %s", v.c_str());
            }
          }
          return result;
        });

    collision_debug_pub_ = this->create_publisher<ffw_collision_checker::msg::CollisionDebug>(
        "/ik_solver/collision_debug", 10);

    // Achieved EE pose (MuJoCo site pose) in map frame, per arm — published each
    // control tick so the SpaceMouse mapper can re-base its delta accumulation
    // onto the real pose at engage/switch time (see quest_teleop_plan.md §11).
    achieved_pose_pub_l_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/ik_solver/achieved_ee_pose_l", 10);
    achieved_pose_pub_r_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/ik_solver/achieved_ee_pose_r", 10);

    // If hardware_mode=false, base teleop is disabled, so there's no mode
    // switch. Default to ARM.
    current_mode_ = hardware_mode_ ? "BASE" : "ARM";

    toggle_group_srv_ = this->create_service<ffw_collision_checker::srv::ToggleJointGroup>(
        "/ik_solver/toggle_joint_group",
        [this](const std::shared_ptr<ffw_collision_checker::srv::ToggleJointGroup::Request> req,
               std::shared_ptr<ffw_collision_checker::srv::ToggleJointGroup::Response> res) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          std::string group = req->group_name;
          if (group == "left_arm") {
            left_arm_enabled_ = !left_arm_enabled_;
            res->success = true;
            res->message = "Left arm toggled. Now " + std::string(left_arm_enabled_ ? "ENABLED" : "DISABLED");
          } else if (group == "right_arm") {
            right_arm_enabled_ = !right_arm_enabled_;
            res->success = true;
            res->message = "Right arm toggled. Now " + std::string(right_arm_enabled_ ? "ENABLED" : "DISABLED");
          } else if (group == "lift") {
            lift_enabled_ = !lift_enabled_;
            res->success = true;
            res->message = "Lift toggled. Now " + std::string(lift_enabled_ ? "ENABLED" : "DISABLED");
          } else {
            res->success = false;
            res->message = "Unknown group name: " + group + ". Valid groups: left_arm, right_arm, lift";
          }
          RCLCPP_INFO(this->get_logger(), "Toggle service: %s", res->message.c_str());
        });

    reset_to_home_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/ik_solver/reset_to_home",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          (void)req;
          std::lock_guard<std::mutex> lock(pose_mutex_);
          home_reset_requested_ = true;
          res->success = true;
          res->message = "Reset to home configuration requested.";
          RCLCPP_INFO(this->get_logger(), "Home reset service triggered");
        });

    save_pose_srv_ = this->create_service<ffw_collision_checker::srv::SaveLoadPose>(
        "/ik_solver/save_pose",
        [this](const std::shared_ptr<ffw_collision_checker::srv::SaveLoadPose::Request> req,
               std::shared_ptr<ffw_collision_checker::srv::SaveLoadPose::Response> res) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          std::string name = req->pose_name;
          if (name.empty()) {
            res->success = false;
            res->message = "Pose name cannot be empty.";
            return;
          }

          PoseState pose;
          pose.target_l_pos = target_l_.translation();
          pose.target_l_rot = target_l_.linear();
          pose.target_r_pos = target_r_.translation();
          pose.target_r_rot = target_r_.linear();
          pose.gripper_l_pos = gripper_l_pos_;
          pose.gripper_r_pos = gripper_r_pos_;
          pose.head_joint1_pos = latest_head_pos_[0];
          pose.head_joint2_pos = latest_head_pos_[1];

          if (req->to_file) {
            auto file_poses = read_poses_from_file();
            file_poses[name] = pose;
            if (write_poses_to_file(file_poses)) {
              res->success = true;
              res->message = "Pose '" + name + "' saved successfully to file.";
              RCLCPP_INFO(this->get_logger(), "Pose '%s' saved to poses file.", name.c_str());
            } else {
              res->success = false;
              res->message = "Failed to write to poses file.";
            }
          } else {
            memory_poses_[name] = pose;
            res->success = true;
            res->message = "Pose saved successfully to memory as '" + name + "'";
            RCLCPP_INFO(this->get_logger(), "Pose '%s' saved to memory.", name.c_str());
          }
        });

    load_pose_srv_ = this->create_service<ffw_collision_checker::srv::SaveLoadPose>(
        "/ik_solver/load_pose",
        [this](const std::shared_ptr<ffw_collision_checker::srv::SaveLoadPose::Request> req,
               std::shared_ptr<ffw_collision_checker::srv::SaveLoadPose::Response> res) {
          std::string name = req->pose_name;
          if (name.empty()) {
            res->success = false;
            res->message = "Pose name cannot be empty.";
            return;
          }

          PoseState pose;
          bool found = false;

          if (req->to_file) {
            auto file_poses = read_poses_from_file();
            auto it = file_poses.find(name);
            if (it != file_poses.end()) {
              pose = it->second;
              found = true;
            } else {
              res->success = false;
              res->message = "Pose '" + name + "' not found in poses file.";
              return;
            }
          } else {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            auto it = memory_poses_.find(name);
            if (it != memory_poses_.end()) {
              pose = it->second;
              found = true;
            } else {
              res->success = false;
              res->message = "Pose '" + name + "' not found in memory.";
              return;
            }
          }

          if (found) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            target_to_load_ = pose;
            active_pose_name_ = name;
            pose_load_requested_ = true;
            res->success = true;
            res->message = "Pose '" + name + "' load requested safely.";
            RCLCPP_INFO(this->get_logger(), "Pose load requested for '%s'", name.c_str());
          }
        });

    list_saved_poses_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/ik_solver/list_saved_poses",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          (void)req;
          std::lock_guard<std::mutex> lock(pose_mutex_);
          std::string msg = "";
          for (auto const& [name, val] : memory_poses_) {
            if (!msg.empty()) msg += "\n";
            msg += name;
          }
          res->success = true;
          res->message = msg;
        });

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/teleop_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          if (msg->data == "ARM" && current_mode_ == "BASE") {
            hardware_sync_requested_ = true;
          }
          current_mode_ = msg->data;
        });

    // Always subscribe to real/Gazebo robot joint states for sync on mode
    // switch.
    real_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          latest_real_joints_ = *msg;
          joint_msg_count_++;
        });

    // Qpos rail: external controller streams a desired joint-state every
    // tick while goal_source_ == RAIL. JointState-by-name (not a new message
    // type) so apply_rail_sync can reuse the same name->qposadr loop as
    // apply_hardware_sync, and partial commands (head/arms/lift optionally)
    // just work. Keep-newest; consumed (and rail_dirty_ cleared) once per
    // fresh message by apply_rail_sync on the main-loop thread — protected by
    // rail_mutex_, not pose_mutex_, since it's written on this (ROS spin)
    // thread and read on the main-loop thread.
    qpos_rail_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/qpos_rail", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(rail_mutex_);
          latest_rail_joints_ = *msg;
          rail_seen_ = true;
          rail_dirty_ = true;
          rail_stamp_ = this->now();
        });

    // Runtime goal-source flip via topic (the natural channel for a
    // continuous external controller); `ros2 param set goal_source` (above)
    // covers manual/launch-time flipping.
    goal_source_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/teleop_goal_source", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == "rail") {
            goal_source_ = GoalSource::RAIL;
            RCLCPP_INFO(this->get_logger(), "goal_source (topic) set to rail");
          } else if (msg->data == "ee") {
            goal_source_ = GoalSource::EE;
            RCLCPP_INFO(this->get_logger(), "goal_source (topic) set to ee");
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "/teleop_goal_source: ignoring unknown value '%s' (want 'ee' or 'rail')",
                        msg->data.c_str());
          }
        });

    // Full machine-state resync request (fired by joy_base on joint-state
    // recovery): re-run apply_hardware_sync to snap MuJoCo back to the real
    // robot joints and re-base the arm targets — same primitive as the
    // BASE→ARM switch, so teleop resumes from where the robot actually is.
    resync_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/teleop_resync", 10, [this](const std_msgs::msg::Empty::SharedPtr) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          hardware_sync_requested_ = true;
        });
    // Pose-change notification: fired when a pose load / home reset homing
    // completes, so joy_hand re-bases its ee_goal_ onto the new achieved pose.
    // Without it the mapper's goal stays anchored at the pre-load pose and the
    // limit profile (a world-frame clamp on that goal) applies relative to the
    // stale reference — the effective range shifts by the pose displacement.
    pose_changed_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/teleop/pose_changed", 10);
    // When hardware_mode=false: no joint_states publisher — MuJoCo is internal
    // only.

    left_traj_pub_ = this->create_publisher<
        trajectory_msgs::msg::JointTrajectory>(
        "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
        10);
    right_traj_pub_ = this->create_publisher<
        trajectory_msgs::msg::JointTrajectory>(
        "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
        10);
    lift_traj_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joystick_controller_right/joint_trajectory", 10);
    head_traj_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joystick_controller_left/joint_trajectory", 10);

    locks_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/teleop_locks", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          std::stringstream ss(msg->data);
          std::string item;
          while (std::getline(ss, item, ',')) {
            item.erase(item.begin(), std::find_if(item.begin(), item.end(), [](unsigned char ch) { return !std::isspace(ch); }));
            item.erase(std::find_if(item.rbegin(), item.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), item.end());
            if (item.empty()) continue;
            // Toggle soft lock: add if not present, remove if present
            auto it = std::find(soft_locked_joints_.begin(), soft_locked_joints_.end(), item);
            if (it != soft_locked_joints_.end()) {
              soft_locked_joints_.erase(it);
            } else {
              soft_locked_joints_.push_back(item);
            }
          }
          // Capture center positions on next apply_soft_joint_locks call
          soft_lock_capture_pending_ = true;
          RCLCPP_INFO(this->get_logger(), "Updated soft-locked joints: %s", msg->data.c_str());
        });

    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/dynamic_obstacle_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          latest_obstacle_pose_ = *msg;
          obstacle_pose_received_ = true;
        });

    head_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/leader/joystick_controller_left/joint_trajectory", 10,
        [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (solving_to_home_) return;
            if (!msg->points.empty()) {
                for (size_t i = 0; i < msg->joint_names.size() && i < msg->points[0].positions.size(); ++i) {
                    if (msg->joint_names[i] == "head_joint1") latest_head_pos_[0] = msg->points[0].positions[i];
                    else if (msg->joint_names[i] == "head_joint2") latest_head_pos_[1] = msg->points[0].positions[i];
                }
            }
        });

    RCLCPP_INFO(this->get_logger(),
                "SpaceMouse IK Teleop started! Hardware Mode: %s",
                hardware_mode_ ? "TRUE" : "FALSE");
  }

  void set_initial_poses(const Eigen::Isometry3d &l,
                         const Eigen::Isometry3d &r) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    initial_l_ = l;
    initial_r_ = r;
    target_l_ = l;
    target_r_ = r;
    xml_home_l_ = l;
    xml_home_r_ = r;
  }

  void get_targets(Eigen::Isometry3d &l, Eigen::Isometry3d &r) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    l = target_l_;
    r = target_r_;
  }

  // §10 quest slow->fast debug: pre-clip target as last seen by the pose callback
  void get_raw_targets(Eigen::Isometry3d &l, Eigen::Isometry3d &r) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    l = raw_target_l_;
    r = raw_target_r_;
  }

  void clip_target(const Eigen::Isometry3d &achieved_l,
                   const Eigen::Isometry3d &achieved_r, double max_dist = 0.01,
                   double max_angle = 0.1) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (solving_to_home_) {
      return;
    }

    // Per-arm, per-state leash: the quest pose path runs 1.5cm during APPROACH
    // (before the triggers release) and 3cm in TRACK so the QP sees a larger
    // error (lower adaptive damping, less lag); the spacemouse delta path keeps
    // the tight 1cm default.
    const double max_dist_l = last_goal_from_quest_l_ ? quest_leash(quest_state_l_) : max_dist;
    const double max_dist_r = last_goal_from_quest_r_ ? quest_leash(quest_state_r_) : max_dist;

    // Left arm clipping. The leash is transient: it bounds only the target
    // handed to the next solveStep. accum (delta path) is deliberately NOT
    // rebased onto the leashed value — accum must track only the mapper's
    // published deltas, so the next update_goal_delta re-asserts the mapper's
    // absolute goal (which the mapper keeps clamped inside the limit profile).
    // Rebasing accum here pinned the target at achieved + leash while the
    // clamped goal was stationary (delta == 0), so the arm stopped short of
    // the profile boundary — or stayed outside it — instead of tracking the
    // in-box goal. The quest path never writes accum for the same reason.
    Eigen::Vector3d err_l = target_l_.translation() - achieved_l.translation();
    if (err_l.norm() > max_dist_l) {
      target_l_.translation() =
          achieved_l.translation() + err_l.normalized() * max_dist_l;
    }
    Eigen::AngleAxisd err_rot_l(target_l_.linear() *
                                achieved_l.linear().transpose());
    if (std::abs(err_rot_l.angle()) > max_angle) {
      Eigen::AngleAxisd clamped_rot_l(
          max_angle * (err_rot_l.angle() > 0 ? 1 : -1), err_rot_l.axis());
      target_l_.linear() =
          clamped_rot_l.toRotationMatrix() * achieved_l.linear();
    }

    // Right arm clipping
    Eigen::Vector3d err_r = target_r_.translation() - achieved_r.translation();
    if (err_r.norm() > max_dist_r) {
      target_r_.translation() =
          achieved_r.translation() + err_r.normalized() * max_dist_r;
    }
    Eigen::AngleAxisd err_rot_r(target_r_.linear() *
                                achieved_r.linear().transpose());
    if (std::abs(err_rot_r.angle()) > max_angle) {
      Eigen::AngleAxisd clamped_rot_r(
          max_angle * (err_rot_r.angle() > 0 ? 1 : -1), err_rot_r.axis());
      target_r_.linear() =
          clamped_rot_r.toRotationMatrix() * achieved_r.linear();
    }
  }

  // Store the achieved EE pose read from site_xpos each main-loop tick so the
  // quest pose callbacks (ROS thread) can leash the absolute goal against it.
  void set_achieved_poses(const Eigen::Isometry3d &achieved_l,
                          const Eigen::Isometry3d &achieved_r,
                          bool valid_l, bool valid_r) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (valid_l) {
      achieved_l_ = achieved_l;
      achieved_valid_l_ = true;
    }
    if (valid_r) {
      achieved_r_ = achieved_r;
      achieved_valid_r_ = true;
    }
  }

  void publish_joints(mjModel *m, mjData *d) {
    // Publish arm and lift trajectories continuously.
    // In BASE mode, the goals are locked horizontally but can ascend/descend
    // synchronously.
    publish_arm_trajectory(m, d, "l");
    publish_arm_trajectory(m, d, "r");
    publish_lift_trajectory(m, d);
  }

  void publish_arm_trajectory(mjModel *m, mjData *d,
                              const std::string &prefix) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = rclcpp::Time(0); // instant execution

    std::vector<std::string> joint_names = {
        "arm_" + prefix + "_joint1", "arm_" + prefix + "_joint2",
        "arm_" + prefix + "_joint3", "arm_" + prefix + "_joint4",
        "arm_" + prefix + "_joint5", "arm_" + prefix + "_joint6",
        "arm_" + prefix + "_joint7", "gripper_" + prefix + "_joint1"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;

    for (const auto &name : joint_names) {
      traj.joint_names.push_back(name);
      if (name == "gripper_l_joint1") {
        point.positions.push_back(gripper_l_pos_);
      } else if (name == "gripper_r_joint1") {
        point.positions.push_back(gripper_r_pos_);
      } else {
        int jnt_id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
        if (jnt_id >= 0) {
          point.positions.push_back(d->qpos[m->jnt_qposadr[jnt_id]]);
        } else {
          point.positions.push_back(0.0);
        }
      }
      point.velocities.push_back(0.0);
    }

    static std::vector<double> prev_positions_l;
    static std::vector<double> prev_positions_r;
    std::vector<double> &prev_positions =
        (prefix == "l") ? prev_positions_l : prev_positions_r;

    bool changed = false;
    if (prev_positions.size() != point.positions.size()) {
      changed = true;
    } else {
      for (size_t i = 0; i < point.positions.size(); ++i) {
        if (std::abs(point.positions[i] - prev_positions[i]) > 1e-4) {
          changed = true;
          break;
        }
      }
    }

    if (!changed)
      return;
    prev_positions = point.positions;

    traj.points.push_back(point);
    if (prefix == "l")
      left_traj_pub_->publish(traj);
    else
      right_traj_pub_->publish(traj);
  }

  void publish_lift_trajectory(mjModel *m, mjData *d) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = rclcpp::Time(0); // instant execution

    traj.joint_names.push_back("lift_joint");

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;

    int jnt_id = mj_name2id(m, mjOBJ_JOINT, "lift_joint");
    if (jnt_id >= 0) {
      point.positions.push_back(d->qpos[m->jnt_qposadr[jnt_id]]);
    } else {
      point.positions.push_back(0.0);
    }
    point.velocities.push_back(0.0);

    traj.points.push_back(point);
    if (lift_traj_pub_) {
      lift_traj_pub_->publish(traj);
    }
  }

  void publish_head_trajectory(mjModel *m, mjData *d) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = rclcpp::Time(0); // instant execution

    std::vector<std::string> joint_names = {"head_joint1", "head_joint2"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 20000000; // 20 ms

    for (const auto &name : joint_names) {
      traj.joint_names.push_back(name);
      int jnt_id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
      if (jnt_id >= 0) {
        point.positions.push_back(d->qpos[m->jnt_qposadr[jnt_id]]);
      } else {
        if (name == "head_joint1") {
          point.positions.push_back(latest_head_pos_[0]);
        } else {
          point.positions.push_back(latest_head_pos_[1]);
        }
      }
      point.velocities.push_back(0.0);
    }

    static std::vector<double> prev_head_positions;
    bool changed = false;
    if (prev_head_positions.size() != point.positions.size()) {
      changed = true;
    } else {
      for (size_t i = 0; i < point.positions.size(); ++i) {
        if (std::abs(point.positions[i] - prev_head_positions[i]) > 1e-4) {
          changed = true;
          break;
        }
      }
    }

    if (!changed)
      return;
    prev_head_positions = point.positions;

    traj.points.push_back(point);
    if (head_traj_pub_) {
      head_traj_pub_->publish(traj);
    }
  }

  void apply_hardware_sync(mjModel *m, mjData *d) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!hardware_sync_requested_)
      return;

    for (size_t i = 0; i < latest_real_joints_.name.size(); ++i) {
      std::string name = latest_real_joints_.name[i];
      if (name == "gripper_l_joint1") {
        gripper_l_pos_ = latest_real_joints_.position[i];
        continue;
      }
      if (name == "gripper_r_joint1") {
        gripper_r_pos_ = latest_real_joints_.position[i];
        continue;
      }
      if (name.find("gripper") != std::string::npos) {
        int jnt_id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
        if (jnt_id < 0) {
          continue; // Skip grippers with no MuJoCo joint (BG2 model)
        }
      }

      int jnt_id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
      if (jnt_id >= 0) {
        d->qpos[m->jnt_qposadr[jnt_id]] = latest_real_joints_.position[i];
      }
    }

    mj_forward(m, d);

    int left_id = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
    int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");

    if (left_id >= 0) {
      initial_l_.translation() =
          Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
      initial_l_.linear() =
          Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
              d->site_xmat + 9 * left_id)
              .cast<double>();
      target_l_ = initial_l_;
      accum_l_trans_.setZero();
      accum_l_rot_.setIdentity();
      // Re-base the mapper's world-frame reference so its next delta starts
      // from the freshly-snapped pose (no phantom jump on mode switch).
      last_mapper_l_trans_ = target_l_.translation();
      last_mapper_l_rot_ = target_l_.linear();
    }
    if (right_id >= 0) {
      initial_r_.translation() =
          Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
      initial_r_.linear() =
          Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
              d->site_xmat + 9 * right_id)
              .cast<double>();
      target_r_ = initial_r_;
      accum_r_trans_.setZero();
      accum_r_rot_.setIdentity();
      last_mapper_r_trans_ = target_r_.translation();
      last_mapper_r_rot_ = target_r_.linear();
    }

    hardware_sync_requested_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Hardware sync complete! Snapped MuJoCo to real robot.");
  }

  void apply_continuous_head_sync(mjModel *m, mjData *d) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    int h1 = mj_name2id(m, mjOBJ_JOINT, "head_joint1");
    int h2 = mj_name2id(m, mjOBJ_JOINT, "head_joint2");

    if (hardware_mode_ && joint_msg_count_ > 0) {
      for (size_t i = 0; i < latest_real_joints_.name.size(); ++i) {
        if (latest_real_joints_.name[i] == "head_joint1") {
          latest_head_pos_[0] = latest_real_joints_.position[i];
        } else if (latest_real_joints_.name[i] == "head_joint2") {
          latest_head_pos_[1] = latest_real_joints_.position[i];
        }
      }
    } else {
      latest_head_pos_[0] = head_target_pos_[0];
      latest_head_pos_[1] = head_target_pos_[1];
    }

    if (h1 >= 0) d->qpos[m->jnt_qposadr[h1]] = latest_head_pos_[0];
    if (h2 >= 0) d->qpos[m->jnt_qposadr[h2]] = latest_head_pos_[1];
  }

  // Qpos-rail mode (mode B): snap -> FK-adopt -> (re-clamp) -> clear for one
  // /qpos_rail tick. Returns true exactly when a fresh rail message was
  // applied this tick (false = hold — nothing moved between rail ticks).
  bool apply_rail_sync(mjModel *m, mjData *d, ffw_ik::IKSolver &solver,
                       const ffw_ik::SolverConfig &cfg,
                       const ffw_ik::CollisionCostConfig &col) {
    if (!rail_goal_source()) return false;

    sensor_msgs::msg::JointState msg;
    {
      std::lock_guard<std::mutex> lock(rail_mutex_);
      if (!rail_dirty_) return false;  // no fresh command -> hold
      msg = latest_rail_joints_;
      rail_dirty_ = false;  // one-shot: a rejected message WARNs once, not every tick
    }

    // Coverage rule: the message must name every joint the robot model
    // expects (solver.getJointNames()), and every name in the message must
    // be a recognized joint — or the gripper scalars, which (per
    // apply_hardware_sync) have no MuJoCo joint in this model and are simply
    // not applicable to a qpos rail. A single missing or unrecognized name
    // rejects the whole message; the external controller is responsible for
    // always sending a complete, matching set.
    std::map<std::string, double> rail_vals;
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
      rail_vals[msg.name[i]] = msg.position[i];
    }
    for (const auto &n : msg.name) {
      if (n == "gripper_l_joint1" || n == "gripper_r_joint1") continue;
      if (mj_name2id(m, mjOBJ_JOINT, n.c_str()) < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "/qpos_rail: rejecting message - unrecognized joint '%s'",
                    n.c_str());
        return false;
      }
    }
    std::vector<std::string> expected = solver.getJointNames();
    for (const auto &jname : expected) {
      if (rail_vals.find(jname) == rail_vals.end()) {
        RCLCPP_WARN(this->get_logger(),
                    "/qpos_rail: rejecting message - missing joint '%s'",
                    jname.c_str());
        return false;
      }
    }

    // Build q_cmd from the current state so anything outside `expected`
    // (there is none on bg2 — no free joint) is left untouched.
    std::vector<mjtNum> q_cmd(m->nq);
    mju_copy(q_cmd.data(), d->qpos, m->nq);
    for (const auto &jname : expected) {
      int jid = mj_name2id(m, mjOBJ_JOINT, jname.c_str());
      int qadr = m->jnt_qposadr[jid];
      double v = rail_vals[jname];
      if (m->jnt_limited[jid]) {
        v = std::clamp(v, static_cast<double>(m->jnt_range[jid * 2]),
                       static_cast<double>(m->jnt_range[jid * 2 + 1]));
      }
      q_cmd[qadr] = v;
    }

    // Snapshot the last known-good state — the rollback / interpolation
    // fallback baseline below.
    std::vector<mjtNum> prev_qpos(m->nq);
    mju_copy(prev_qpos.data(), d->qpos, m->nq);

    // The snap.
    mju_copy(d->qpos, q_cmd.data(), m->nq);
    mj_forward(m, d);

    // Critical check over ALL contacts (not just top-k) — a 6th-closest
    // violator must not be invisible.
    ffw_ik::ContactResult all;
    solver.computeContacts(d, std::max(1, d->ncon), all);
    double min_dist = all.closest.empty() ? 0.30 : all.closest.front().dist;

    if (all.closest.empty() || min_dist >= col.collision_margin) {
      // Safe as commanded — keep it verbatim, skip correction.
    } else {
      ffw_ik::SolverConfig clear_cfg = cfg;
      clear_cfg.nullspace_weight = 0.0;
      clear_cfg.inertia_weight = 0.0;
      clear_cfg.step_size = std::min(cfg.step_size, 0.05);  // land close to the boundary

      double final_min = 0.0;
      bool cleared = solver.clearToMargin(d, clear_cfg, col, 50, &final_min);

      if (!cleared) {
        // Interpolation fallback: walk from the (partially) corrected pose
        // back toward the last known-safe pose at decreasing fractions
        // (90%, 80%, ... 0%), taking the first (highest) fraction that
        // clears. Each candidate is one cheap mj_forward + contact check, no
        // QP. Always terminates at a provably safe pose (frac=0 ==
        // prev_qpos is always tested), instead of throwing away whatever
        // partial progress clearToMargin made.
        std::vector<mjtNum> corrected(m->nq);
        mju_copy(corrected.data(), d->qpos, m->nq);
        bool found_safe = false;
        for (int step = 9; step >= 0 && !found_safe; --step) {
          double frac = step / 10.0;
          for (int i = 0; i < m->nq; ++i)
            d->qpos[i] = prev_qpos[i] + frac * (corrected[i] - prev_qpos[i]);
          mj_forward(m, d);
          ffw_ik::ContactResult probe;
          solver.computeContacts(d, std::max(1, d->ncon), probe);
          double probe_dist = probe.closest.empty() ? 0.30 : probe.closest.front().dist;
          if (probe.closest.empty() || probe_dist >= col.collision_margin) {
            found_safe = true;
          }
        }
        if (!found_safe) {
          // Even prev_qpos (frac=0, already tested above) failed the check —
          // nothing better is available; hold it anyway, same as any other
          // rail failure mode.
          mju_copy(d->qpos, prev_qpos.data(), m->nq);
          mj_forward(m, d);
        }
        RCLCPP_WARN(this->get_logger(),
                    "/qpos_rail: correction did not fully converge "
                    "(final_min_dist=%.4f) - interpolated toward last-safe pose",
                    final_min);
      }
    }

    // FK-adopt (same reads as apply_hardware_sync) — the snap updated the
    // goal, so any pending EE goal is ignored.
    std::lock_guard<std::mutex> lock(pose_mutex_);
    int left_id = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
    int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");
    if (left_id >= 0) {
      initial_l_.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
      initial_l_.linear() =
          Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
              d->site_xmat + 9 * left_id)
              .cast<double>();
      target_l_ = initial_l_;
      accum_l_trans_.setZero();
      accum_l_rot_.setIdentity();
      last_mapper_l_trans_ = target_l_.translation();
      last_mapper_l_rot_ = target_l_.linear();
      last_goal_from_quest_l_ = false;
    }
    if (right_id >= 0) {
      initial_r_.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
      initial_r_.linear() =
          Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
              d->site_xmat + 9 * right_id)
              .cast<double>();
      target_r_ = initial_r_;
      accum_r_trans_.setZero();
      accum_r_rot_.setIdentity();
      last_mapper_r_trans_ = target_r_.translation();
      last_mapper_r_rot_ = target_r_.linear();
      last_goal_from_quest_r_ = false;
    }

    return true;
  }

  bool get_obstacle_pose(Eigen::Vector3d& pos, Eigen::Quaterniond& quat) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!obstacle_pose_received_) return false;
    pos = Eigen::Vector3d(latest_obstacle_pose_.pose.position.x,
                          latest_obstacle_pose_.pose.position.y,
                          latest_obstacle_pose_.pose.position.z);
    quat = Eigen::Quaterniond(latest_obstacle_pose_.pose.orientation.w,
                              latest_obstacle_pose_.pose.orientation.x,
                              latest_obstacle_pose_.pose.orientation.y,
                              latest_obstacle_pose_.pose.orientation.z);
    return true;
  }

  std::vector<std::string> get_frozen_joints() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    std::vector<std::string> frozen = frozen_joints_;
    if (!left_arm_enabled_) {
      if (std::find(frozen.begin(), frozen.end(), "arm_l") == frozen.end()) {
        frozen.push_back("arm_l");
      }
    }
    if (!right_arm_enabled_) {
      if (std::find(frozen.begin(), frozen.end(), "arm_r") == frozen.end()) {
        frozen.push_back("arm_r");
      }
    }
    if (!lift_enabled_) {
      if (std::find(frozen.begin(), frozen.end(), "lift") == frozen.end()) {
        frozen.push_back("lift");
      }
    }
    if (std::find(frozen.begin(), frozen.end(), "head") == frozen.end()) {
      frozen.push_back("head");
    }
    return frozen;
  }

  void apply_home_reset(mjModel *m, mjData *d) {
    (void)m;
    (void)d;
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!home_reset_requested_)
      return;

    // 1. Re-enable all joint groups so they can move
    left_arm_enabled_ = true;
    right_arm_enabled_ = true;
    lift_enabled_ = true;
    solving_to_home_ = true;
    homing_ticks_ = 0;

    // 2. Reset targets, reference frames and accumulators to pure XML home poses
    initial_l_ = xml_home_l_;
    target_l_ = xml_home_l_;
    accum_l_trans_.setZero();
    accum_l_rot_.setIdentity();
    first_msg_l_ = true;

    initial_r_ = xml_home_r_;
    target_r_ = xml_home_r_;
    accum_r_trans_.setZero();
    accum_r_rot_.setIdentity();
    first_msg_r_ = true;

    head_target_pos_ = {0.0, 0.0};

    // Publish a single trajectory message to move head to home slowly (1.5s)
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"head_joint1", "head_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.0};
    point.velocities = {0.0, 0.0};
    point.time_from_start.sec = 1;
    point.time_from_start.nanosec = 500000000; // 1.5s
    traj.points.push_back(point);
    if (head_traj_pub_) {
      head_traj_pub_->publish(traj);
    }

    home_reset_requested_ = false;
    RCLCPP_INFO(this->get_logger(), "Home reset triggered: re-enabled all groups and solving back to XML home posture safely!");
  }

  void apply_pose_load(mjModel *m, mjData *d) {
    (void)m;
    (void)d;
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!pose_load_requested_)
      return;

    // 1. Re-enable all joint groups so they can move
    left_arm_enabled_ = true;
    right_arm_enabled_ = true;
    lift_enabled_ = true;
    solving_to_home_ = true;
    homing_ticks_ = 0;

    // 2. Set targets to the loaded pose
    Eigen::Isometry3d loaded_l = Eigen::Isometry3d::Identity();
    loaded_l.translation() = target_to_load_.target_l_pos;
    loaded_l.linear() = target_to_load_.target_l_rot;

    Eigen::Isometry3d loaded_r = Eigen::Isometry3d::Identity();
    loaded_r.translation() = target_to_load_.target_r_pos;
    loaded_r.linear() = target_to_load_.target_r_rot;

    initial_l_ = loaded_l;
    target_l_ = loaded_l;
    accum_l_trans_.setZero();
    accum_l_rot_.setIdentity();
    first_msg_l_ = true;

    initial_r_ = loaded_r;
    target_r_ = loaded_r;
    accum_r_trans_.setZero();
    accum_r_rot_.setIdentity();
    first_msg_r_ = true;

    gripper_l_pos_ = target_to_load_.gripper_l_pos;
    gripper_r_pos_ = target_to_load_.gripper_r_pos;

    head_target_pos_[0] = target_to_load_.head_joint1_pos;
    head_target_pos_[1] = target_to_load_.head_joint2_pos;

    // Publish a single trajectory message to move head to loaded pose slowly (1.5s)
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"head_joint1", "head_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {target_to_load_.head_joint1_pos, target_to_load_.head_joint2_pos};
    point.velocities = {0.0, 0.0};
    point.time_from_start.sec = 1;
    point.time_from_start.nanosec = 500000000; // 1.5s
    traj.points.push_back(point);
    if (head_traj_pub_) {
      head_traj_pub_->publish(traj);
    }

    pose_load_requested_ = false;
    RCLCPP_INFO(this->get_logger(), "Loaded pose '%s' safely! Homing solver started.", active_pose_name_.c_str());
  }

  void apply_gripper_sync(mjModel *m, mjData *d) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    
    // Left gripper bodies
    int gl_r1 = mj_name2id(m, mjOBJ_BODY, "gripper_l_rh_p12_rn_r1");
    int gl_r2 = mj_name2id(m, mjOBJ_BODY, "gripper_l_rh_p12_rn_r2");
    int gl_l1 = mj_name2id(m, mjOBJ_BODY, "gripper_l_rh_p12_rn_l1");
    int gl_l2 = mj_name2id(m, mjOBJ_BODY, "gripper_l_rh_p12_rn_l2");

    Eigen::Quaterniond q0(0.0, 0.0, 1.0, 0.0); // Default quat [0, 0, 1, 0]

    if (gl_r1 >= 0) {
      Eigen::Quaterniond q_local(Eigen::AngleAxisd(gripper_l_pos_, Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond q = q0 * q_local;
      m->body_quat[4 * gl_r1 + 0] = q.w();
      m->body_quat[4 * gl_r1 + 1] = q.x();
      m->body_quat[4 * gl_r1 + 2] = q.y();
      m->body_quat[4 * gl_r1 + 3] = q.z();
    }
    if (gl_r2 >= 0) {
      Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(-gripper_l_pos_, Eigen::Vector3d::UnitX()));
      m->body_quat[4 * gl_r2 + 0] = q.w();
      m->body_quat[4 * gl_r2 + 1] = q.x();
      m->body_quat[4 * gl_r2 + 2] = q.y();
      m->body_quat[4 * gl_r2 + 3] = q.z();
    }
    if (gl_l1 >= 0) {
      Eigen::Quaterniond q_local(Eigen::AngleAxisd(-gripper_l_pos_, Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond q = q0 * q_local;
      m->body_quat[4 * gl_l1 + 0] = q.w();
      m->body_quat[4 * gl_l1 + 1] = q.x();
      m->body_quat[4 * gl_l1 + 2] = q.y();
      m->body_quat[4 * gl_l1 + 3] = q.z();
    }
    if (gl_l2 >= 0) {
      Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(gripper_l_pos_, Eigen::Vector3d::UnitX()));
      m->body_quat[4 * gl_l2 + 0] = q.w();
      m->body_quat[4 * gl_l2 + 1] = q.x();
      m->body_quat[4 * gl_l2 + 2] = q.y();
      m->body_quat[4 * gl_l2 + 3] = q.z();
    }

    // Right gripper bodies
    int gr_r1 = mj_name2id(m, mjOBJ_BODY, "gripper_r_rh_p12_rn_r1");
    int gr_r2 = mj_name2id(m, mjOBJ_BODY, "gripper_r_rh_p12_rn_r2");
    int gr_l1 = mj_name2id(m, mjOBJ_BODY, "gripper_r_rh_p12_rn_l1");
    int gr_l2 = mj_name2id(m, mjOBJ_BODY, "gripper_r_rh_p12_rn_l2");

    if (gr_r1 >= 0) {
      Eigen::Quaterniond q_local(Eigen::AngleAxisd(gripper_r_pos_, Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond q = q0 * q_local;
      m->body_quat[4 * gr_r1 + 0] = q.w();
      m->body_quat[4 * gr_r1 + 1] = q.x();
      m->body_quat[4 * gr_r1 + 2] = q.y();
      m->body_quat[4 * gr_r1 + 3] = q.z();
    }
    if (gr_r2 >= 0) {
      Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(-gripper_r_pos_, Eigen::Vector3d::UnitX()));
      m->body_quat[4 * gr_r2 + 0] = q.w();
      m->body_quat[4 * gr_r2 + 1] = q.x();
      m->body_quat[4 * gr_r2 + 2] = q.y();
      m->body_quat[4 * gr_r2 + 3] = q.z();
    }
    if (gr_l1 >= 0) {
      Eigen::Quaterniond q_local(Eigen::AngleAxisd(-gripper_r_pos_, Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond q = q0 * q_local;
      m->body_quat[4 * gr_l1 + 0] = q.w();
      m->body_quat[4 * gr_l1 + 1] = q.x();
      m->body_quat[4 * gr_l1 + 2] = q.y();
      m->body_quat[4 * gr_l1 + 3] = q.z();
    }
    if (gr_l2 >= 0) {
      Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(gripper_r_pos_, Eigen::Vector3d::UnitX()));
      m->body_quat[4 * gr_l2 + 0] = q.w();
      m->body_quat[4 * gr_l2 + 1] = q.x();
      m->body_quat[4 * gr_l2 + 2] = q.y();
      m->body_quat[4 * gr_l2 + 3] = q.z();
    }

    // Right gripper joint (XM430-W350) - directly set qpos if joint exists
    // in the MuJoCo model (SMTM variant only; BG2 returns -1 harmlessly)
    int gr_jnt = mj_name2id(m, mjOBJ_JOINT, "gripper_r_joint1");
    if (gr_jnt >= 0) {
      d->qpos[m->jnt_qposadr[gr_jnt]] = gripper_r_pos_;
    }
  }

  void toggle_frozen_joint(const std::string& jname) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    // We treat the jname as an exact prefix or string to toggle
    auto it = std::find(frozen_joints_.begin(), frozen_joints_.end(), jname);
    if (it != frozen_joints_.end()) {
        frozen_joints_.erase(it);
    } else {
        frozen_joints_.push_back(jname);
    }
    if (std::find(frozen_joints_.begin(), frozen_joints_.end(), "head") == frozen_joints_.end()) {
        frozen_joints_.push_back("head");
    }
  }

  void register_callbacks(std::function<std::string()> tree_cb, std::function<std::string()> list_cb) {
    tree_cb_ = tree_cb;
    list_cb_ = list_cb;
    tree_srv_ = this->create_service<std_srvs::srv::Trigger>("/ik_solver/get_tree",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            if (tree_cb_) {
                res->success = true;
                res->message = "\n" + tree_cb_();
            }
        });
    list_srv_ = this->create_service<std_srvs::srv::Trigger>("/ik_solver/list_joints",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            if (list_cb_) {
                res->success = true;
                res->message = "\n" + list_cb_();
            }
        });
  }

  // ── Soft locking helpers ─────────────────────────────────────────

  /** Return the current set of soft-locked joint names. */
  const std::vector<std::string>& get_soft_locked_joints() const {
    return soft_locked_joints_;
  }

  /** Apply soft joint locks by projecting joint positions back within
   *  the slack deadband from their locked center positions.
   *  Called after solver.solveStep() in the main loop. */
  void apply_soft_joint_locks(mjModel *m, mjData *d) {
    if (soft_locked_joints_.empty()) return;

    // Capture center positions on first call after lock engagement
    if (soft_lock_capture_pending_) {
      for (const auto &jname : soft_locked_joints_) {
        int jnt_id = mj_name2id(m, mjOBJ_JOINT, jname.c_str());
        if (jnt_id >= 0) {
          soft_locked_joint_centers_[jname] = d->qpos[m->jnt_qposadr[jnt_id]];
        }
      }
      soft_lock_capture_pending_ = false;
    }

    // Apply deadband clamping
    for (const auto &jname : soft_locked_joints_) {
      int jnt_id = mj_name2id(m, mjOBJ_JOINT, jname.c_str());
      if (jnt_id < 0) continue;
      int qposadr = m->jnt_qposadr[jnt_id];
      auto it = soft_locked_joint_centers_.find(jname);
      if (it == soft_locked_joint_centers_.end()) continue;
      double center = it->second;
      double slack = (jname.find("lift") != std::string::npos) ? lift_soft_slack_ : joint_soft_slack_;

      double diff = d->qpos[qposadr] - center;
      if (diff > slack) {
        d->qpos[qposadr] = center + slack;
      } else if (diff < -slack) {
        d->qpos[qposadr] = center - slack;
      }
    }
  }

private:
  std::map<std::string, PoseState> read_poses_from_file() {
    std::map<std::string, PoseState> file_poses;
    std::string filepath = "/home/lys/robotis_ws/src/ai_worker/ffw_collision_checker/config/poses.txt";
    std::ifstream in(filepath);
    if (!in.is_open()) {
      return file_poses;
    }
    std::string line;
    std::string current_pose_name = "";
    PoseState current_pose;
    bool has_pose = false;

    while (std::getline(in, line)) {
      if (line.empty()) continue;
      if (line[0] == '[' && line[line.size() - 1] == ']') {
        if (has_pose && !current_pose_name.empty()) {
          file_poses[current_pose_name] = current_pose;
        }
        current_pose_name = line.substr(1, line.size() - 2);
        current_pose = PoseState();
        has_pose = true;
      } else {
        std::stringstream ss(line);
        std::string key;
        ss >> key;
        if (key == "target_l_pos") {
          double x, y, z;
          if (ss >> x >> y >> z) {
            current_pose.target_l_pos = Eigen::Vector3d(x, y, z);
          }
        } else if (key == "target_l_rot") {
          for (int i = 0; i < 9; ++i) {
            ss >> current_pose.target_l_rot.data()[i];
          }
        } else if (key == "target_r_pos") {
          double x, y, z;
          if (ss >> x >> y >> z) {
            current_pose.target_r_pos = Eigen::Vector3d(x, y, z);
          }
        } else if (key == "target_r_rot") {
          for (int i = 0; i < 9; ++i) {
            ss >> current_pose.target_r_rot.data()[i];
          }
        } else if (key == "gripper_l_pos") {
          ss >> current_pose.gripper_l_pos;
        } else if (key == "gripper_r_pos") {
          ss >> current_pose.gripper_r_pos;
        } else if (key == "head_joint1_pos") {
          ss >> current_pose.head_joint1_pos;
        } else if (key == "head_joint2_pos") {
          ss >> current_pose.head_joint2_pos;
        }
      }
    }
    if (has_pose && !current_pose_name.empty()) {
      file_poses[current_pose_name] = current_pose;
    }
    in.close();
    return file_poses;
  }

  bool write_poses_to_file(const std::map<std::string, PoseState> &file_poses) {
    std::string filepath = "/home/lys/robotis_ws/src/ai_worker/ffw_collision_checker/config/poses.txt";
    try {
      std::filesystem::path p(filepath);
      if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
      }
      std::ofstream out(filepath);
      if (!out.is_open()) {
        return false;
      }
      for (const auto &[name, pose] : file_poses) {
        out << "[" << name << "]\n";
        out << "target_l_pos " << pose.target_l_pos.x() << " " << pose.target_l_pos.y() << " " << pose.target_l_pos.z() << "\n";
        out << "target_l_rot";
        for (int i = 0; i < 9; ++i) out << " " << pose.target_l_rot.data()[i];
        out << "\n";
        out << "target_r_pos " << pose.target_r_pos.x() << " " << pose.target_r_pos.y() << " " << pose.target_r_pos.z() << "\n";
        out << "target_r_rot";
        for (int i = 0; i < 9; ++i) out << " " << pose.target_r_rot.data()[i];
        out << "\n";
        out << "gripper_l_pos " << pose.gripper_l_pos << "\n";
        out << "gripper_r_pos " << pose.gripper_r_pos << "\n";
        out << "head_joint1_pos " << pose.head_joint1_pos << "\n";
        out << "head_joint2_pos " << pose.head_joint2_pos << "\n\n";
      }
      out.close();
      return true;
    } catch (...) {
      return false;
    }
  }
  void update_goal_delta_l(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (solving_to_home_ || rail_goal_source()) {
      return; // rail mode owns the tick — ignore any EE goal
    }
    Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z);
    Eigen::Quaterniond rot(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rot_mat = rot.toRotationMatrix();

    last_goal_from_quest_l_ = false;

    if (first_msg_l_) {
      last_mapper_l_trans_ = trans;
      last_mapper_l_rot_ = rot_mat;
      first_msg_l_ = false;
    }

    Eigen::Vector3d delta_trans = trans - last_mapper_l_trans_;
    Eigen::Matrix3d delta_rot = rot_mat * last_mapper_l_rot_.transpose();

    // Re-base detection: a single SpaceMouse tick moves the goal by at most
    // ~7.5mm / ~3° (pos_step × sensitivity, cubic-scaled), so a larger jump
    // means the mapper re-anchored ee_goal_ onto a new achieved pose (pose
    // load / home reset / engage). Treat it as a new baseline instead of a
    // command — otherwise the arm lurches by the full pose displacement.
    Eigen::AngleAxisd jump_aa(delta_rot);
    if (delta_trans.norm() > kMapperRebaseDist ||
        std::abs(jump_aa.angle()) > kMapperRebaseAng) {
      // Re-base: the mapper re-anchored ee_goal_ onto a new achieved pose (ARM
      // switch / override release / resync / pose load — the mapper always
      // re-bases onto achieved, never into free space). Adopt the fresh pose as
      // the new target too: leaving the old leashed target stale would park the
      // arm off by the quest→delta residual and effectively shift the limit box.
      // Safe to adopt — the arm is already at the pose the mapper re-based onto.
      last_mapper_l_trans_ = trans;
      last_mapper_l_rot_ = rot_mat;
      initial_l_.translation() = trans;
      initial_l_.linear() = rot_mat;
      accum_l_trans_ = Eigen::Vector3d::Zero();
      accum_l_rot_ = Eigen::Quaterniond::Identity();
      target_l_.translation() = trans;
      target_l_.linear() = rot_mat;
      raw_target_l_ = target_l_;  // §10: pre-clip target for quest slow->fast debug
      return; // re-base adopted as the new target
    }

    last_mapper_l_trans_ = trans;
    last_mapper_l_rot_ = rot_mat;

    accum_l_trans_ += delta_trans;
    accum_l_rot_ = delta_rot * accum_l_rot_;
    target_l_.translation() = initial_l_.translation() + accum_l_trans_;
    target_l_.linear() = accum_l_rot_ * initial_l_.linear();
    clamp_target_angle(target_l_, initial_l_, accum_l_rot_);
    raw_target_l_ = target_l_;  // §10: pre-clip target for quest slow->fast debug
  }

  void update_goal_delta_r(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (solving_to_home_ || rail_goal_source()) {
      return; // rail mode owns the tick — ignore any EE goal
    }
    Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z);
    Eigen::Quaterniond rot(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rot_mat = rot.toRotationMatrix();

    last_goal_from_quest_r_ = false;

    if (first_msg_r_) {
      last_mapper_r_trans_ = trans;
      last_mapper_r_rot_ = rot_mat;
      first_msg_r_ = false;
    }

    Eigen::Vector3d delta_trans = trans - last_mapper_r_trans_;
    Eigen::Matrix3d delta_rot = rot_mat * last_mapper_r_rot_.transpose();

    // Re-base detection: see update_goal_delta_l — a goal jump larger than any
    // single SpaceMouse tick means the mapper re-anchored ee_goal_ onto a new
    // achieved pose; treat it as a new baseline, never as a command.
    Eigen::AngleAxisd jump_aa(delta_rot);
    if (delta_trans.norm() > kMapperRebaseDist ||
        std::abs(jump_aa.angle()) > kMapperRebaseAng) {
      // Re-base: the mapper re-anchored ee_goal_ onto a new achieved pose (ARM
      // switch / override release / resync / pose load — the mapper always
      // re-bases onto achieved, never into free space). Adopt the fresh pose as
      // the new target too: leaving the old leashed target stale would park the
      // arm off by the quest→delta residual and effectively shift the limit box.
      // Safe to adopt — the arm is already at the pose the mapper re-based onto.
      last_mapper_r_trans_ = trans;
      last_mapper_r_rot_ = rot_mat;
      initial_r_.translation() = trans;
      initial_r_.linear() = rot_mat;
      accum_r_trans_ = Eigen::Vector3d::Zero();
      accum_r_rot_ = Eigen::Quaterniond::Identity();
      target_r_.translation() = trans;
      target_r_.linear() = rot_mat;
      raw_target_r_ = target_r_;  // §10: pre-clip target for quest slow->fast debug
      return; // re-base adopted as the new target
    }

    last_mapper_r_trans_ = trans;
    last_mapper_r_rot_ = rot_mat;

    accum_r_trans_ += delta_trans;
    accum_r_rot_ = delta_rot * accum_r_rot_;
    target_r_.translation() = initial_r_.translation() + accum_r_trans_;
    target_r_.linear() = accum_r_rot_ * initial_r_.linear();
    clamp_target_angle(target_r_, initial_r_, accum_r_rot_);
    raw_target_r_ = target_r_;  // §10: pre-clip target for quest slow->fast debug
  }

  // Quest state machine mirror, fed by joy_hand's /quest/<arm>/state (published
  // on every transition). Declared here so the callbacks below can use it as a
  // return/parameter type; CTRL also means "quest not active".
  enum class QuestState { CTRL, APPROACH, TRACK };

  // Per-arm quest state feed (§8b): joy_hand publishes /quest/<arm>/state on
  // every transition. Reads under pose_mutex_ like the pose callbacks.
  void quest_state_cb_l(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    quest_state_l_ = parse_quest_state(msg->data);
  }
  void quest_state_cb_r(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    quest_state_r_ = parse_quest_state(msg->data);
  }
  static QuestState parse_quest_state(const std::string &s) {
    if (s == "APPROACH") return QuestState::APPROACH;
    if (s == "TRACK") return QuestState::TRACK;
    return QuestState::CTRL;
  }

  // §11: live per-arm trigger feed (joy_hand publishes every active quest tick).
  // Only consumed in TRACK; the held trigger during APPROACH is ignored so the
  // 4-way engage hold never moves the gripper.
  void quest_trigger_cb_l(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    quest_trigger_l_ = msg->data;
  }
  void quest_trigger_cb_r(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    quest_trigger_r_ = msg->data;
  }

  // Quest absolute-goal path: the mapper's interpolated ee_goal_ IS the target,
  // no accum/rebase/delta chain. The goal is leashed against the stored achieved
  // pose (achieved + per-state leash) so a far or unreachable goal still yields a
  // bounded per-tick velocity — the main-loop clip_target alone would be defeated
  // here because the next quest message overwrites its snap each tick. The
  // mapper's absolute orientation is authoritative, so clamp_target_angle is NOT
  // applied. last_mapper_* is parked on the applied (leashed) target — not the
  // raw goal — so the first spacemouse delta after a quest→delta handoff starts
  // from the pose the solver actually holds, with no leash-slack residual.
  void update_goal_pose_l(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (solving_to_home_ || rail_goal_source()) {
      return; // rail mode owns the tick — ignore any EE goal
    }
    Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z);
    Eigen::Quaterniond rot(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rot_mat = rot.toRotationMatrix();

    first_msg_l_ = false;
    last_goal_from_quest_l_ = true;

    target_l_.translation() = trans;
    target_l_.linear() = rot_mat;
    if (achieved_valid_l_) {
      const double leash_l = quest_leash(quest_state_l_);   // 1.5cm APPROACH, 6cm TRACK
      Eigen::Vector3d err_l = trans - achieved_l_.translation();
      if (err_l.norm() > leash_l) {
        target_l_.translation() =
            achieved_l_.translation() + err_l.normalized() * leash_l;
      }
      Eigen::AngleAxisd err_rot_l(target_l_.linear() *
                                  achieved_l_.linear().transpose());
      if (std::abs(err_rot_l.angle()) > kGoalMaxAngle) {
        Eigen::AngleAxisd clamped_rot_l(
            kGoalMaxAngle * (err_rot_l.angle() > 0 ? 1 : -1),
            err_rot_l.axis());
        target_l_.linear() =
            clamped_rot_l.toRotationMatrix() * achieved_l_.linear();
      }
    }

    // Quest → delta handoff: same re-anchor as _r — the quest path never writes
    // accum/initial, so park the delta baseline on the applied target so a resume
    // continues from the quest pose instead of a stale pre-quest one.
    initial_l_.translation() = target_l_.translation();
    initial_l_.linear() = target_l_.linear();
    accum_l_trans_ = Eigen::Vector3d::Zero();
    accum_l_rot_ = Eigen::Quaterniond::Identity();

    // last_mapper tracks the APPLIED (leashed) target, not the raw quest goal —
    // at the quest→delta handoff the first mapper delta is compared against this
    // baseline, so leaving it on the raw goal would inject the leash slack as a
    // phantom residual (or, if over kMapperRebaseDist, a stale parked target).
    last_mapper_l_trans_ = target_l_.translation();
    last_mapper_l_rot_ = target_l_.linear();

    raw_target_l_ = target_l_;  // §10: pre-clip (leashed) target for quest slow->fast debug
  }

  void update_goal_pose_r(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (solving_to_home_ || rail_goal_source()) {
      return; // rail mode owns the tick — ignore any EE goal
    }
    Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z);
    Eigen::Quaterniond rot(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rot_mat = rot.toRotationMatrix();

    first_msg_r_ = false;
    last_goal_from_quest_r_ = true;

    target_r_.translation() = trans;
    target_r_.linear() = rot_mat;
    if (achieved_valid_r_) {
      const double leash_r = quest_leash(quest_state_r_);   // 1.5cm APPROACH, 6cm TRACK
      Eigen::Vector3d err_r = trans - achieved_r_.translation();
      if (err_r.norm() > leash_r) {
        target_r_.translation() =
            achieved_r_.translation() + err_r.normalized() * leash_r;
      }
      Eigen::AngleAxisd err_rot_r(target_r_.linear() *
                                  achieved_r_.linear().transpose());
      if (std::abs(err_rot_r.angle()) > kGoalMaxAngle) {
        Eigen::AngleAxisd clamped_rot_r(
            kGoalMaxAngle * (err_rot_r.angle() > 0 ? 1 : -1),
            err_rot_r.axis());
        target_r_.linear() =
            clamped_rot_r.toRotationMatrix() * achieved_r_.linear();
      }
    }

    // Quest → delta handoff: the quest path writes target but never accum/initial,
    // so on resume update_goal_delta_r would rebuild a stale pre-quest baseline.
    // Re-anchor the delta integrator onto the applied (leashed) target here — one
    // quest message now updates BOTH the absolute target AND the delta baseline,
    // so no stale record is left for the switch-back to resurrect.
    initial_r_.translation() = target_r_.translation();
    initial_r_.linear() = target_r_.linear();
    accum_r_trans_ = Eigen::Vector3d::Zero();
    accum_r_rot_ = Eigen::Quaterniond::Identity();

    // last_mapper tracks the APPLIED (leashed) target, not the raw quest goal —
    // at the quest→delta handoff the first mapper delta is compared against this
    // baseline, so leaving it on the raw goal would inject the leash slack as a
    // phantom residual (or, if over kMapperRebaseDist, a stale parked target).
    last_mapper_r_trans_ = target_r_.translation();
    last_mapper_r_rot_ = target_r_.linear();

    raw_target_r_ = target_r_;  // §10: pre-clip (leashed) target for quest slow->fast debug
  }

  void joy_callback_l(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.empty())
      return;
    double now = this->now().seconds();

    bool new_btn0 = (msg->buttons[0] == 1);
    if (new_btn0 && !left_btn0_)
      left_btn0_press_time_ = now;
    left_btn0_ = new_btn0;

    bool new_btn1 = false;
    for (size_t i = 1; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i] == 1)
        new_btn1 = true;
    }
    if (new_btn1 && !left_btn1_)
      left_btn1_press_time_ = now;
    left_btn1_ = new_btn1;

    if (current_mode_ == "BASE" && msg->axes.size() > 2) {
      // In BASE mode, left mouse Z axis controls synchronized lift of both IK
      // targets
      double z = msg->axes[2];
      if (std::abs(z) > 0.01) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        // Speed: 0.005 per tick at 100Hz. Apply cubic scaling and 0.2
        // multiplier.
        double z_cubed = z * z * z;
        double z_delta = z_cubed * 0.005 * 0.2;

        accum_l_trans_.z() += z_delta;
        accum_r_trans_.z() += z_delta;

        target_l_.translation() = initial_l_.translation() + accum_l_trans_;
        target_r_.translation() = initial_r_.translation() + accum_r_trans_;
      }
    }
  }

  void joy_callback_r(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.empty())
      return;
    double now = this->now().seconds();

    bool new_btn0 = (msg->buttons[0] == 1);
    if (new_btn0 && !right_btn0_)
      right_btn0_press_time_ = now;
    right_btn0_ = new_btn0;

    bool new_btn1 = false;
    for (size_t i = 1; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i] == 1)
        new_btn1 = true;
    }
    if (new_btn1 && !right_btn1_)
      right_btn1_press_time_ = now;
    right_btn1_ = new_btn1;
  }

  void clamp_target_angle(Eigen::Isometry3d &target, Eigen::Isometry3d &initial,
                          Eigen::Matrix3d &accum) {
    // 1. Find the absolute rotation from the World Identity frame to the target
    // pose Since the reference is Identity, the relative rotation is just
    // target.linear()
    Eigen::Matrix3d abs_rot = target.linear();

    // 2. Convert to Angle-Axis to get the pure 3D rotation angle from World
    // Identity
    Eigen::AngleAxisd angle_axis(abs_rot);
    double angle = angle_axis.angle();

    // Eigen AngleAxisd returns an angle in [0, pi].
    // If the absolute angle from World Identity exceeds pi/2 (90 degrees),
    // scale it back!
    double limit = M_PI / 2.0;
    if (angle > limit) {
      Eigen::AngleAxisd clamped_abs_rot(limit, angle_axis.axis());

      // Reconstruct the target using the clamped absolute rotation
      target.linear() = clamped_abs_rot.toRotationMatrix();

      // Update the accumulator so the teleop doesn't wind up while clamped
      accum = target.linear() * initial.linear().transpose();
    }
  }

  Eigen::Isometry3d initial_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d initial_r_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_r_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d xml_home_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d xml_home_r_ = Eigen::Isometry3d::Identity();
  std::mutex pose_mutex_;

  std::map<std::string, PoseState> memory_poses_;
  std::atomic<bool> pose_load_requested_{false};
  PoseState target_to_load_;
  std::string active_pose_name_;

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

  // §10 quest slow->fast debug: pre-clip target as last seen by the pose callback
  // (before clip_target's leash and before any solveStep-side modifications).
  Eigen::Isometry3d raw_target_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d raw_target_r_ = Eigen::Isometry3d::Identity();

  // Two goal-update paths: true when the last goal came from /quest/<arm>/ee_target_pose
  // (pose path), false when it came from /spacemouse/<arm>/ee_target_pose (delta
  // path). Used to keep clip_target's accum rebases out of the pose path.
  bool last_goal_from_quest_l_ = false;
  bool last_goal_from_quest_r_ = false;

  // Stored achieved EE pose for the pose path's per-state leash (1.5cm APPROACH,
  // 6cm TRACK).
  // The main loop reads site_xpos every tick and pushes it here under pose_mutex_
  // so update_goal_pose_* can clamp the absolute goal even though the next quest
  // message overwrites the target before the loop's clip_target would run.
  Eigen::Isometry3d achieved_l_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d achieved_r_ = Eigen::Isometry3d::Identity();
  bool achieved_valid_l_ = false;
  bool achieved_valid_r_ = false;
  // Per-path, per-state translation leash. The spacemouse delta path keeps the
  // tight 1cm clamp; the quest pose path is state-dependent: 1.5cm during
  // APPROACH (before the triggers release, so the arm previews slowly) and 6cm
  // once TRACK begins. A bigger leash means a bigger QP error and thus lower
  // adaptive damping, so the arm chases the accumulator faster and lags less (§6).
  // [TEST] 3cm -> 6cm: is the leash binding the TRACK chase? If the ~1s tail
  // survives the doubled clip, the leash was never the limiter — the tail is
  // the lead decay / solver response instead (revert or tune from there).
  static constexpr double kSpaceMouseMaxDist = 0.01;      // 1cm — matches clip_target default
  static constexpr double kQuestApproachMaxDist = 0.015;  // 1.5cm — before trigger release
  static constexpr double kQuestMaxDist = 0.06;           // 6cm — after release (TRACK) [TEST]
  static constexpr double kGoalMaxAngle = 0.1;            // ~5.7°, both paths

  // Goal-jump thresholds for the delta-path re-base detection: a single
  // SpaceMouse tick moves the goal by at most ~7.5mm / ~3°, so anything
  // larger is a mapper re-anchor (pose load / home reset / engage) and must
  // be treated as a new baseline, never as a commanded motion.
  static constexpr double kMapperRebaseDist = 0.03;  // 3cm
  static constexpr double kMapperRebaseAng = 0.1;    // ~5.7°

  // Quest state mirrors (see enum above); used to pick the per-state leash.
  QuestState quest_state_l_ {QuestState::CTRL};
  QuestState quest_state_r_ {QuestState::CTRL};

  // §11: latest /quest/<arm>/trigger analog — absolute gripper command in TRACK.
  double quest_trigger_l_ = 0.0;
  double quest_trigger_r_ = 0.0;

  double quest_leash(QuestState st) const {
    return st == QuestState::APPROACH ? kQuestApproachMaxDist : kQuestMaxDist;
  }

  bool left_btn0_ = false, left_btn1_ = false;
  bool right_btn0_ = false, right_btn1_ = false;

  double left_btn0_press_time_ = 0.0;
  double left_btn1_press_time_ = 0.0;
  double right_btn0_press_time_ = 0.0;
  double right_btn1_press_time_ = 0.0;

  // Gripper position ranges — larger = closed. Left RH-P12-RN-A, right XM430
  // (MinPosLimit 130 ≈ 0.199 rad, floor 0.175). Used by the button increments
  // and the §11 quest-trigger absolute mapping.
  static constexpr double kGripperLMin = 0.0;
  static constexpr double kGripperLMax = 1.2;
  static constexpr double kGripperRMin = 0.175;
  static constexpr double kGripperRMax = 1.2;

  double gripper_l_pos_ = 1.1;
  double gripper_r_pos_ = 1.1;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_l_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_r_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr quest_pose_sub_l_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr quest_pose_sub_r_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr quest_state_sub_l_;  // /quest/left/state (§8b)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr quest_state_sub_r_;  // /quest/right/state (§8b)
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr quest_trigger_sub_l_;  // /quest/left/trigger (§11)
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr quest_trigger_sub_r_;  // /quest/right/trigger (§11)
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_l_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_r_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resync_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr locks_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_traj_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tree_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr list_srv_;
  rclcpp::Service<ffw_collision_checker::srv::ToggleJointGroup>::SharedPtr toggle_group_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_to_home_srv_;
  rclcpp::Service<ffw_collision_checker::srv::SaveLoadPose>::SharedPtr save_pose_srv_;
  rclcpp::Service<ffw_collision_checker::srv::SaveLoadPose>::SharedPtr load_pose_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr list_saved_poses_srv_;
  std::function<std::string()> tree_cb_;
  std::function<std::string()> list_cb_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      left_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      right_traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pose_changed_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      lift_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      head_traj_pub_;
  rclcpp::Publisher<ffw_collision_checker::msg::CollisionDebug>::SharedPtr collision_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr achieved_pose_pub_l_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr achieved_pose_pub_r_;

  bool hardware_mode_ = true;
  std::string robot_model_ = "bg2";
  std::atomic<bool> hardware_sync_requested_{false};
  std::string current_mode_ = "BASE";
  sensor_msgs::msg::JointState latest_real_joints_;
  int joint_msg_count_ = 0;

  std::atomic<bool> home_reset_requested_{false};
  std::atomic<bool> solving_to_home_{false};
  std::atomic<int> homing_ticks_{0};
  bool left_arm_enabled_ = true;
  bool right_arm_enabled_ = true;
  bool lift_enabled_ = true;
  bool collision_debug_ = true;
  bool quest_debug_solver_ = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::vector<std::string> frozen_joints_{"head"};

  // Soft locking — individual joints with slack deadband
  std::vector<std::string> soft_locked_joints_;
  std::map<std::string, double> soft_locked_joint_centers_;
  bool soft_lock_capture_pending_ = false;
  double joint_soft_slack_ = 3.0 * M_PI / 180.0;  // ±3 degrees for arm joints
  double lift_soft_slack_ = 0.05;                   // ±5 cm for lift joint

  geometry_msgs::msg::PoseStamped latest_obstacle_pose_;
  bool obstacle_pose_received_{false};
  std::vector<double> latest_head_pos_{0.0, 0.0};
  std::vector<double> head_target_pos_{0.0, 0.0};

  // ----------------------------------------------------------
  // Qpos-rail mode (goal_source == RAIL): an external controller streams a
  // desired qpos every tick; the FK-of-qpos goal replaces the EE goal each
  // tick. goal_source_ is atomic (read from EE-callback threads without
  // locking); everything else the /qpos_rail callback touches is a separate
  // rail_mutex_ (not pose_mutex_, which is already heavily contended) since
  // it's written on the ROS spin thread and read on the main-loop thread.
  // ----------------------------------------------------------
  enum class GoalSource { EE, RAIL };
  std::atomic<GoalSource> goal_source_{GoalSource::EE};

  std::mutex rail_mutex_;
  sensor_msgs::msg::JointState latest_rail_joints_;
  bool rail_seen_ = false;   // at least one /qpos_rail message ever received
  bool rail_dirty_ = false;  // a fresh message is waiting to be consumed
  rclcpp::Time rail_stamp_;  // receipt time of latest_rail_joints_
  static constexpr double kRailTimeout = 0.5;  // seconds

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr qpos_rail_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_source_sub_;

  // mode only: which branch owns the tick.
  bool rail_goal_source() const { return goal_source_.load() == GoalSource::RAIL; }
  // mode AND freshness: gates the mapper-affecting reads / stale-rail hold.
  bool rail_active() {
    if (!rail_goal_source()) return false;
    std::lock_guard<std::mutex> lock(rail_mutex_);
    if (!rail_seen_) return false;
    return (this->now() - rail_stamp_).seconds() < kRailTimeout;
  }

public:
  bool is_hardware_mode() const { return hardware_mode_; }
  bool quest_debug_solver() const { return quest_debug_solver_; }
  bool is_sync_requested() const { return hardware_sync_requested_; }
  const std::string &get_robot_model() const { return robot_model_; }
  void request_hardware_sync() { hardware_sync_requested_ = true; }
  // Tell joy_hand the arm settled at a new pose (pose load / home reset) so it
  // re-bases ee_goal_ onto the achieved pose before the next mapper delta.
  void publish_pose_changed() {
    std_msgs::msg::Empty msg;
    pose_changed_pub_->publish(msg);
  }
  bool is_solving_to_home() const { return solving_to_home_; }
  void stop_solving_to_home() { solving_to_home_ = false; }
  int get_homing_ticks() const { return homing_ticks_; }
  void increment_homing_ticks() { homing_ticks_++; }
  void reset_homing_ticks() { homing_ticks_ = 0; }

  bool is_rail_mode() const { return rail_goal_source(); }
  // EE -> rail transition: a pending hardware-sync/homing must not fight the
  // rail's snap this tick.
  void on_rail_engage() {
    hardware_sync_requested_ = false;
    stop_solving_to_home();
  }

  bool is_left_arm_enabled() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return left_arm_enabled_;
  }
  bool is_right_arm_enabled() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return right_arm_enabled_;
  }

  bool is_collision_debug_enabled() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return collision_debug_;
  }

  void publish_collision_debug(const ffw_collision_checker::msg::CollisionDebug &msg) {
    collision_debug_pub_->publish(msg);
  }

  // Publish each arm's achieved EE pose (MuJoCo site pose) in map frame, so
  // the SpaceMouse mapper can re-base its world-frame accumulation onto the
  // real pose at engage/switch time (see quest_teleop_plan.md §11).
  void publish_achieved_poses(const Eigen::Isometry3d &achieved_l,
                              const Eigen::Isometry3d &achieved_r,
                              bool publish_l, bool publish_r) {
    auto publish_one = [this](const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &pub,
                              const Eigen::Isometry3d &achieved) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      msg.header.frame_id = "map";
      msg.pose.position.x = achieved.translation().x();
      msg.pose.position.y = achieved.translation().y();
      msg.pose.position.z = achieved.translation().z();
      Eigen::Quaterniond q(achieved.linear());
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      pub->publish(msg);
    };
    if (publish_l) publish_one(achieved_pose_pub_l_, achieved_l);
    if (publish_r) publish_one(achieved_pose_pub_r_, achieved_r);
  }

  int get_and_reset_joint_msg_count() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    int count = joint_msg_count_;
    joint_msg_count_ = 0;
    return count;
  }

  std::string get_current_mode() const { return current_mode_; }
  void update_grippers(mjModel *m, mjData *d) {
    if (current_mode_ != "ARM")
      return;

    double now = this->now().seconds();
    auto get_step = [now](bool is_pressed, double press_time) {
      if (!is_pressed)
        return 0.0;
      double hold_time = now - press_time;
      if (hold_time <= 0.0)
        return 0.004;
      // increase up to x3.0 after 2 seconds (base doubled vs 0.002)
      double mult = 1.0 + (hold_time / 2.0) * 2.0;
      return 0.004 * std::min(mult, 3.0);
    };

    // Quest TRACK (§11): the released trigger is repurposed as an absolute
    // gripper command — 1 → close, 0 → open. Only TRACK obeys it: the 4-way
    // engage hold keeps the trigger pulled during APPROACH, so obeying it there
    // would slam the gripper closed. Thumbs release → CTRL → spacemouse buttons
    // resume; trigger grip re-arms only on the next engage → release cycle.
    // Reads under pose_mutex_ (writers: quest state/trigger callbacks).
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (quest_state_l_ == QuestState::TRACK) {
        gripper_l_pos_ = std::clamp(quest_trigger_l_, 0.0, 1.0) * kGripperLMax;
      } else {
        gripper_l_pos_ += get_step(left_btn0_, left_btn0_press_time_); // close
        gripper_l_pos_ -= get_step(left_btn1_, left_btn1_press_time_); // open
        gripper_l_pos_ = std::clamp(gripper_l_pos_, kGripperLMin, kGripperLMax);
      }
      if (quest_state_r_ == QuestState::TRACK) {
        gripper_r_pos_ = kGripperRMin +
            std::clamp(quest_trigger_r_, 0.0, 1.0) * (kGripperRMax - kGripperRMin);
      } else {
        gripper_r_pos_ += get_step(right_btn0_, right_btn0_press_time_); // close
        gripper_r_pos_ -= get_step(right_btn1_, right_btn1_press_time_); // open
        gripper_r_pos_ = std::clamp(gripper_r_pos_, kGripperRMin, kGripperRMax);  // XM430 MinPosLimit=130
      }
    }

    static int log_counter = 0;
    if ((left_btn0_ || left_btn1_ || right_btn0_ || right_btn1_) &&
        log_counter++ % 50 == 0) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Grippers -> L: %.2f (btns: %d, %d) | R: %.2f (btns: %d, %d)",
                  gripper_l_pos_, left_btn0_, left_btn1_, gripper_r_pos_,
                  right_btn0_, right_btn1_);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopNode>();

  using ament_index_cpp::get_package_share_directory;
  const std::string model = node->get_robot_model();
  const std::string scene_file = (model == "smtm")
      ? "scene_inverse_kinematic_smtm.xml"
      : "scene_inverse_kinematic.xml";
  const std::string xml_path =
      get_package_share_directory("ffw_collision_checker") +
      "/3rd_party/robotis_ffw/" + scene_file;

  char error[1000];
  mjModel *m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    std::cerr << "MuJoCo load error: " << error << "\n";
    return 1;
  }
  mjData *d = mj_makeData(m);

  // Restored original robot alpha (no longer forcing transparency)
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
  solver_cfg.tolerance = 2.5e-3; // 0.25cm
  solver_cfg.track_orientation = true;

  // Loosen stall conditions to prevent giving up during slow teleop
  solver_cfg.ee_improvement_rate = 1e-5;
  solver_cfg.ee_window = 15;

  // Increase orientation weight significantly so it doesn't get suppressed by
  // damping
  solver_cfg.ori_weight = 0.5;

  // Revert back to the default nullspace behavior (Type 0) to prevent the 
  // Manipulability solver from violently jerking the arm backwards (which feels like a teleport).
  solver_cfg.nullspace_type = 3;
  solver_cfg.nullspace_amplitude = 0.1;

  ffw_ik::CollisionCostConfig col_cfg;
  col_cfg.collision_margin = 0.10;
  col_cfg.weight_scale = 0.025;

  ffw_ik::IKSolver solver(m);
  SimpleViewer viewer(m);

  std::deque<double> err_hist, dist_hist;

  // Start ROS thread
  std::thread ros_thread([&node]() { rclcpp::spin(node); });

  node->register_callbacks(
    [&solver, d]() { return solver.getKinematicTree(d); },
    [&solver, node_ptr = node.get()]() {
       auto joints = solver.getJointNames();
       auto frozen = node_ptr->get_frozen_joints();
       auto &soft = node_ptr->get_soft_locked_joints();
       std::stringstream ss;
       ss << "=== Joints ===\n";
       for (size_t i = 0; i < joints.size(); ++i) {
          bool is_frozen = false;
          for (const auto& f : frozen) {
             if (joints[i].find(f) != std::string::npos) is_frozen = true;
          }
          bool is_soft = false;
          for (const auto& s : soft) {
             if (joints[i].find(s) != std::string::npos) is_soft = true;
          }
          std::string status = is_frozen ? "[LOCKED] " : (is_soft ? "[SOFT  ] " : "[      ] ");
          ss << "[" << (i+1) << "] " << status << joints[i] << "\n";
       }
       ss << "\nLOCKED = hard-frozen (group), SOFT = soft-locked (individual, ±3° slack)";
       ss << "\nTo toggle a joint, publish its name to /teleop_locks";
       return ss.str();
    }
  );

  if (node->is_hardware_mode()) {
    RCLCPP_INFO(
        node->get_logger(),
        "Waiting up to 30s for stable /joint_states (>=95Hz for 1s)...");
    auto wait_start = std::chrono::steady_clock::now();
    auto window_start = wait_start;
    node->get_and_reset_joint_msg_count();
    bool stable = false;

    while (rclcpp::ok() && !stable && viewer.enabled()) {
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - wait_start)
              .count() > 30) {
        RCLCPP_WARN(
            node->get_logger(),
            "Timeout waiting for stable /joint_states! Starting anyway.");
        break;
      }
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                window_start)
              .count() >= 1000) {
        int count = node->get_and_reset_joint_msg_count();
        RCLCPP_INFO(node->get_logger(), "/joint_states rate: %d Hz", count);
        if (count >= 95) {
          RCLCPP_INFO(
              node->get_logger(),
              "Stable /joint_states achieved! Performing initial sync...");
          stable = true;
          node->request_hardware_sync();
        }
        window_start = now;
      }

      // Render the viewer so the OS doesn't kill the unresponsive window
      viewer.render(d);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Wait for sync to actually complete
    while (rclcpp::ok() && node->is_sync_requested() && viewer.enabled()) {
      node->apply_hardware_sync(m, d);
      viewer.render(d);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Update targets to reflect the synced state
    node->get_targets(init_l, init_r);
  }

  // Main interaction loop
  int step_counter = 0;
  Eigen::Isometry3d prev_target_l = init_l;
  Eigen::Isometry3d prev_target_r = init_r;
  bool prev_rail_gs = node->is_rail_mode();

  while (viewer.enabled() && rclcpp::ok()) {
    // Rail mode owns the tick: read once, gate everything below on it.
    bool rail_gs = node->is_rail_mode();
    if (rail_gs != prev_rail_gs) {
      if (rail_gs) {
        node->on_rail_engage();  // EE -> rail: don't let a pending sync/homing fight the snap
      } else if (node->is_hardware_mode()) {
        node->request_hardware_sync();  // rail -> EE: resume synced to the real robot
      }
      prev_rail_gs = rail_gs;
    }

    solver_cfg.frozen_joints = node->get_frozen_joints();
    solver_cfg.left_weight_scale = node->is_left_arm_enabled() ? 1.0 : 0.0;
    solver_cfg.right_weight_scale = node->is_right_arm_enabled() ? 1.0 : 0.0;

    Eigen::Vector3d obs_pos;
    Eigen::Quaterniond obs_quat;
    if (node->get_obstacle_pose(obs_pos, obs_quat)) {
      int body_id = mj_name2id(m, mjOBJ_BODY, "dynamic_obstacle");
      if (body_id >= 0) {
        int mocap_id = m->body_mocapid[body_id];
        if (mocap_id >= 0) {
          d->mocap_pos[3 * mocap_id + 0] = obs_pos.x();
          d->mocap_pos[3 * mocap_id + 1] = obs_pos.y();
          d->mocap_pos[3 * mocap_id + 2] = obs_pos.z();
          
          double q_norm = obs_quat.norm();
          if (std::abs(q_norm - 1.0) < 0.1) {
            d->mocap_quat[4 * mocap_id + 0] = obs_quat.w();
            d->mocap_quat[4 * mocap_id + 1] = obs_quat.x();
            d->mocap_quat[4 * mocap_id + 2] = obs_quat.y();
            d->mocap_quat[4 * mocap_id + 3] = obs_quat.z();
          } else {
            d->mocap_quat[4 * mocap_id + 0] = 1.0;
            d->mocap_quat[4 * mocap_id + 1] = 0.0;
            d->mocap_quat[4 * mocap_id + 2] = 0.0;
            d->mocap_quat[4 * mocap_id + 3] = 0.0;
          }
        }
      }
    }

    // A real /joint_states-driven head snap must not fight the rail.
    if (!rail_gs) node->apply_continuous_head_sync(m, d);

    Eigen::Isometry3d current_target_l, current_target_r;
    node->get_targets(current_target_l, current_target_r);

    // Clear early-stopping history if the target has moved
    if (!current_target_l.isApprox(prev_target_l, 1e-6) ||
        !current_target_r.isApprox(prev_target_r, 1e-6)) {
      err_hist.clear();
    }
    prev_target_l = current_target_l;
    prev_target_r = current_target_r;

    // Run one gradient step continuously in ALL modes.
    // In BASE mode, targets are stationary except for synchronized Z movement.
    ffw_ik::SolverConfig active_cfg = solver_cfg;
    if (node->is_solving_to_home()) {
      active_cfg.joint_vel_limit = 0.5; // limit to 0.5 rad/s for slow/safe homing/solving
    }

    ffw_ik::StepResult res;
    if (!rail_gs) {
      res = solver.solveStep(d, current_target_l, current_target_r, active_cfg,
                             col_cfg, err_hist, dist_hist);
    } else {
      // Rail mode owns the tick: apply_rail_sync's correction (function B,
      // exits on its own critical check) *is* this tick's solve — never
      // stack solveStep's equilibrium chase on top of it. apply_rail_sync
      // gates itself on rail_dirty_ (hold on a stale tick, nothing moves).
      // Still refresh contacts for the viewer/collision-debug code below.
      node->apply_rail_sync(m, d, solver, active_cfg, col_cfg);
      solver.computeContacts(d, active_cfg.topk_contacts, res.contacts);
      res.min_dist = res.contacts.closest.empty() ? 0.30 : res.contacts.closest.front().dist;
    }

    // Apply soft joint locks (project joints within slack deadband). Rail
    // overrides an active lock by design (confirmed with user) — skip it
    // while rail owns the tick.
    if (!rail_gs) node->apply_soft_joint_locks(m, d);

    if (node->is_solving_to_home()) {
      node->increment_homing_ticks();
      bool arrived = (res.error < 2.0 * active_cfg.tolerance);

      static int stall_counter = 0;
      if (res.stalled) {
        stall_counter++;
      } else {
        stall_counter = 0;
      }

      bool timeout = (node->get_homing_ticks() > 500); // 5 seconds timeout
      bool stalled = (stall_counter > 50); // 0.5s solver stall/stagnation limit

      if (arrived || timeout || stalled) {
        node->stop_solving_to_home();
        stall_counter = 0;
        // The arm just settled at a loaded pose / home-reset posture that the
        // mapper knows nothing about: tell joy_hand to re-base its ee_goal_
        // onto the new achieved pose, or the limit profile clamps the stale
        // pre-load goal and the effective range shifts by the displacement.
        node->publish_pose_changed();
        if (arrived) {
          RCLCPP_INFO(node->get_logger(), "Robot has successfully arrived at target posture. Teleop resumed.");
        } else if (timeout) {
          RCLCPP_WARN(node->get_logger(), "Homing/Target-seeking timed out (5s limit). Resuming teleop at current state.");
        } else {
          RCLCPP_WARN(node->get_logger(), "Homing/Target-seeking stagnated/stalled. Resuming teleop at current state.");
        }
      }
    }

    viewer.setCollisions(res.contacts.closest);

    if (node->is_collision_debug_enabled()) {
      ffw_collision_checker::msg::CollisionDebug debug_msg;
      bool found_close_contact = false;
      for (const auto &c : res.contacts.closest) {
        if (c.dist < col_cfg.collision_margin * 1.5 && c.dist > -0.3) {
          found_close_contact = true;
          const char *g1_name = mj_id2name(m, mjOBJ_GEOM, c.geom1);
          const char *g2_name = mj_id2name(m, mjOBJ_GEOM, c.geom2);
          const char *b1_name = mj_id2name(m, mjOBJ_BODY, c.body1);
          const char *b2_name = mj_id2name(m, mjOBJ_BODY, c.body2);
          
          debug_msg.geom1_names.push_back(g1_name ? g1_name : "unnamed");
          debug_msg.geom2_names.push_back(g2_name ? g2_name : "unnamed");
          debug_msg.body1_names.push_back(b1_name ? b1_name : "unnamed");
          debug_msg.body2_names.push_back(b2_name ? b2_name : "unnamed");
          debug_msg.distances.push_back(c.dist);
          debug_msg.p1_x.push_back(c.p1.x());
          debug_msg.p1_y.push_back(c.p1.y());
          debug_msg.p1_z.push_back(c.p1.z());
          debug_msg.p2_x.push_back(c.p2.x());
          debug_msg.p2_y.push_back(c.p2.y());
          debug_msg.p2_z.push_back(c.p2.z());
        }
      }
      if (found_close_contact) {
        node->publish_collision_debug(debug_msg);
      }
    }

    // Print solver status every 20 ticks or if stalled
    if (++step_counter % 20 == 0 || res.stalled) {
      // Reduced verbosity
    }

    node->update_grippers(m, d);

    // A homing trigger or a real /joint_states snapshot must not fight the
    // rail; a request made during rail mode stays pending and is processed
    // once rail mode is exited.
    if (!rail_gs) {
      node->apply_home_reset(m, d);
      node->apply_pose_load(m, d);

      // Process any hardware sync requests before mj_forward
      node->apply_hardware_sync(m, d);
    }

    // Sync grippers and mimics to MuJoCo qpos
    node->apply_gripper_sync(m, d);

    // Ensure simulation state is updated for rendering
    mj_forward(m, d);

    // Dynamic error-bounding clamp (Leash)
    Eigen::Isometry3d achieved_l = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d achieved_r = Eigen::Isometry3d::Identity();
    if (left_id >= 0) {
      achieved_l.translation() =
          Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
      achieved_l.linear() =
          Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
              d->site_xmat + 9 * left_id)
              .cast<double>();
    }
    if (right_id >= 0) {
      achieved_r.translation() =
          Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
      achieved_r.linear() =
          Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
              d->site_xmat + 9 * right_id)
              .cast<double>();
    }
    // Feed the achieved pose to the quest pose callbacks (ROS thread) so they
    // can leash the absolute goal against it.
    node->set_achieved_poses(achieved_l, achieved_r, left_id >= 0, right_id >= 0);
    // §10 quest slow->fast debug: ground truth on why APPROACH error never
    // falls below threshold. raw target = pre-clip (as the mapper delta
    // landed), current target = what solveStep chased this tick, achieved =
    // MuJoCo site. Throttled to 0.2s.
    if (node->quest_debug_solver()) {
      static auto last_qs_print = std::chrono::steady_clock::now();
      auto qs_now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(qs_now - last_qs_print).count() >= 0.2) {
        last_qs_print = qs_now;
        Eigen::Isometry3d raw_l, raw_r;
        node->get_raw_targets(raw_l, raw_r);
        RCLCPP_INFO(
            node->get_logger(),
            "[SOLVER] L tgt=(%.3f %.3f %.3f) raw=(%.3f %.3f %.3f) ach=(%.3f %.3f %.3f) |clip-ach|L=%.4f |raw-ach|L=%.4f |raw-clip|L=%.4f | R tgt=(%.3f %.3f %.3f) raw=(%.3f %.3f %.3f) ach=(%.3f %.3f %.3f) |clip-ach|R=%.4f |raw-ach|R=%.4f err=%.4f conv=%d stalled=%d dist=%.4f nC=%d soft=%zu",
            current_target_l.translation().x(), current_target_l.translation().y(),
            current_target_l.translation().z(), raw_l.translation().x(),
            raw_l.translation().y(), raw_l.translation().z(),
            achieved_l.translation().x(), achieved_l.translation().y(),
            achieved_l.translation().z(),
            (current_target_l.translation() - achieved_l.translation()).norm(),
            (raw_l.translation() - achieved_l.translation()).norm(),
            (raw_l.translation() - current_target_l.translation()).norm(),
            current_target_r.translation().x(), current_target_r.translation().y(),
            current_target_r.translation().z(), raw_r.translation().x(),
            raw_r.translation().y(), raw_r.translation().z(),
            achieved_r.translation().x(), achieved_r.translation().y(),
            achieved_r.translation().z(),
            (current_target_r.translation() - achieved_r.translation()).norm(),
            (raw_r.translation() - achieved_r.translation()).norm(),
            res.error, res.converged, res.stalled, res.min_dist,
            res.contacts.total_contacts,
            node->get_soft_locked_joints().size());
        // §10 CBF wall: the body pair at the closest contact and how many CBF
        // rows the QP enforced (contacts within collision_margin). dist is the
        // same closest gap as above; the names discriminate a central
        // torso/head wall from a self-collision or the opposite arm.
        int n_within = 0;
        for (const auto &c : res.contacts.closest) {
          if (c.dist > col_cfg.collision_margin) break;
          ++n_within;
        }
        const char *b1 = nullptr, *b2 = nullptr;
        double cdist = res.min_dist;
        if (!res.contacts.closest.empty()) {
          const auto &c0 = res.contacts.closest.front();
          b1 = mj_id2name(m, mjOBJ_BODY, c0.body1);
          b2 = mj_id2name(m, mjOBJ_BODY, c0.body2);
          cdist = c0.dist;
        }
        RCLCPP_INFO(node->get_logger(),
          "[SOLVER] wall dist=%.4f within=%d block=%s<->%s",
          cdist, n_within,
          b1 ? b1 : "?", b2 ? b2 : "?");
      }
    }

    // Publish the achieved EE pose in map frame (not the commanded target —
    // the setpoint can lag the site after clipping/IK).
    node->publish_achieved_poses(achieved_l, achieved_r,
                                 left_id >= 0, right_id >= 0);
    node->clip_target(achieved_l, achieved_r);

    // Update target variables so the viewer spheres reflect the clipped target
    node->get_targets(current_target_l, current_target_r);

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