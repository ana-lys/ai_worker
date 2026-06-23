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
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
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
      Eigen::Vector3d rpy_l = extract_rpy(pose_l_.linear());
      Eigen::Vector3d rpy_r = extract_rpy(pose_r_.linear());
      const mjtNum* pl = d->site_xpos + 3 * id_l;
      const mjtNum* pr = d->site_xpos + 3 * id_r;
      snprintf(text_l, sizeof(text_l), "LEFT EE XYZ:\nX: %7.3f\nY: %7.3f\nZ: %7.3f\nRoll:  %7.3f\nPitch: %7.3f\nYaw:   %7.3f", 
               pl[0], pl[1], pl[2], rpy_l[0], rpy_l[1], rpy_l[2]);
      snprintf(text_r, sizeof(text_r), "RIGHT EE XYZ:\nX: %7.3f\nY: %7.3f\nZ: %7.3f\nRoll:  %7.3f\nPitch: %7.3f\nYaw:   %7.3f", 
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

  static Eigen::Vector3d extract_rpy(const Eigen::Matrix3d& R) {
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

    // hardware_mode=true  (default): subscribe to real robot /joint_states for sync.
    // hardware_mode=false (sim-only): drive MuJoCo internally only; publish nothing to ROS.
    this->declare_parameter("hardware_mode", true);
    hardware_mode_ = this->get_parameter("hardware_mode").as_bool();

    // If hardware_mode=false, base teleop is disabled, so there's no mode switch. Default to ARM.
    current_mode_ = hardware_mode_ ? "BASE" : "ARM";

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/teleop_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (msg->data == "ARM" && current_mode_ == "BASE") {
                hardware_sync_requested_ = true;
            }
            current_mode_ = msg->data;
        });

    // Always subscribe to real/Gazebo robot joint states for sync on mode switch.
    real_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            latest_real_joints_ = *msg;
            joint_msg_count_++;
        });
    // When hardware_mode=false: no joint_states publisher — MuJoCo is internal only.

    left_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory", 10);
    right_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);
    lift_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/leader/joystick_controller_right/joint_trajectory", 10);

    RCLCPP_INFO(this->get_logger(),
                "SpaceMouse IK Teleop started! Hardware Mode: %s", hardware_mode_ ? "TRUE" : "FALSE");
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

  void clip_target(const Eigen::Isometry3d &achieved_l,
                   const Eigen::Isometry3d &achieved_r,
                   double max_dist = 0.015, double max_angle = 0.2) {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    // Left arm clipping
    Eigen::Vector3d err_l = target_l_.translation() - achieved_l.translation();
    if (err_l.norm() > max_dist) {
      target_l_.translation() = achieved_l.translation() + err_l.normalized() * max_dist;
      accum_l_trans_ = target_l_.translation() - initial_l_.translation();
    }
    Eigen::AngleAxisd err_rot_l(target_l_.linear() * achieved_l.linear().transpose());
    if (std::abs(err_rot_l.angle()) > max_angle) {
      Eigen::AngleAxisd clamped_rot_l(max_angle * (err_rot_l.angle() > 0 ? 1 : -1), err_rot_l.axis());
      target_l_.linear() = clamped_rot_l.toRotationMatrix() * achieved_l.linear();
      accum_l_rot_ = target_l_.linear() * initial_l_.linear().transpose();
    }

    // Right arm clipping
    Eigen::Vector3d err_r = target_r_.translation() - achieved_r.translation();
    if (err_r.norm() > max_dist) {
      target_r_.translation() = achieved_r.translation() + err_r.normalized() * max_dist;
      accum_r_trans_ = target_r_.translation() - initial_r_.translation();
    }
    Eigen::AngleAxisd err_rot_r(target_r_.linear() * achieved_r.linear().transpose());
    if (std::abs(err_rot_r.angle()) > max_angle) {
      Eigen::AngleAxisd clamped_rot_r(max_angle * (err_rot_r.angle() > 0 ? 1 : -1), err_rot_r.axis());
      target_r_.linear() = clamped_rot_r.toRotationMatrix() * achieved_r.linear();
      accum_r_rot_ = target_r_.linear() * initial_r_.linear().transpose();
    }
  }

  void publish_joints(mjModel *m, mjData *d) {
    // Publish arm and lift trajectories continuously.
    // In BASE mode, the goals are locked horizontally but can ascend/descend synchronously.
    publish_arm_trajectory(m, d, "l");
    publish_arm_trajectory(m, d, "r");
    publish_lift_trajectory(m, d);
  }

  void publish_arm_trajectory(mjModel *m, mjData *d, const std::string& prefix) {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.header.stamp = rclcpp::Time(0); // instant execution
      
      std::vector<std::string> joint_names = {
          "arm_" + prefix + "_joint1", "arm_" + prefix + "_joint2", "arm_" + prefix + "_joint3",
          "arm_" + prefix + "_joint4", "arm_" + prefix + "_joint5", "arm_" + prefix + "_joint6",
          "arm_" + prefix + "_joint7", "gripper_" + prefix + "_joint1"
      };
      
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start.sec = 0;
      point.time_from_start.nanosec = 0;
      
      for (const auto& name : joint_names) {
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
      std::vector<double>& prev_positions = (prefix == "l") ? prev_positions_l : prev_positions_r;
      
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
      
      if (!changed) return;
      prev_positions = point.positions;
      
      traj.points.push_back(point);
      if (prefix == "l") left_traj_pub_->publish(traj);
      else right_traj_pub_->publish(traj);
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

  void apply_hardware_sync(mjModel *m, mjData *d) {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (!hardware_sync_requested_) return;

      for (size_t i = 0; i < latest_real_joints_.name.size(); ++i) {
          std::string name = latest_real_joints_.name[i];
          if (name == "gripper_l_joint1") gripper_l_pos_ = latest_real_joints_.position[i];
          if (name == "gripper_r_joint1") gripper_r_pos_ = latest_real_joints_.position[i];
          if (name.find("gripper") != std::string::npos) continue; // Skip grippers
          
          int jnt_id = mj_name2id(m, mjOBJ_JOINT, name.c_str());
          if (jnt_id >= 0) {
              d->qpos[m->jnt_qposadr[jnt_id]] = latest_real_joints_.position[i];
          }
      }
      
      mj_forward(m, d);
      
      int left_id = mj_name2id(m, mjOBJ_SITE, "left_gripper_site");
      int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");
      
      if (left_id >= 0) {
          initial_l_.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
          initial_l_.linear() = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(d->site_xmat + 9 * left_id).cast<double>();
          target_l_ = initial_l_;
          accum_l_trans_.setZero();
          accum_l_rot_.setIdentity();
      }
      if (right_id >= 0) {
          initial_r_.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
          initial_r_.linear() = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(d->site_xmat + 9 * right_id).cast<double>();
          target_r_ = initial_r_;
          accum_r_trans_.setZero();
          accum_r_rot_.setIdentity();
      }

      hardware_sync_requested_ = false;
      RCLCPP_INFO(this->get_logger(), "Hardware sync complete! Snapped MuJoCo to real robot.");
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

    accum_l_trans_ += delta_trans;
    accum_l_rot_ = delta_rot * accum_l_rot_;
    target_l_.translation() = initial_l_.translation() + accum_l_trans_;
    target_l_.linear() = accum_l_rot_ * initial_l_.linear();
    clamp_target_angle(target_l_, initial_l_, accum_l_rot_);
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

    accum_r_trans_ += delta_trans;
    accum_r_rot_ = delta_rot * accum_r_rot_;
    target_r_.translation() = initial_r_.translation() + accum_r_trans_;
    target_r_.linear() = accum_r_rot_ * initial_r_.linear();
    clamp_target_angle(target_r_, initial_r_, accum_r_rot_);
  }

  void joy_callback_l(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.empty()) return;
    left_btn0_ = (msg->buttons[0] == 1);
    left_btn1_ = false;
    for (size_t i = 1; i < msg->buttons.size(); ++i) {
        if (msg->buttons[i] == 1) left_btn1_ = true;
    }

    if (current_mode_ == "BASE" && msg->axes.size() > 2) {
        // In BASE mode, left mouse Z axis controls synchronized lift of both IK targets
        double z = msg->axes[2];
        if (std::abs(z) > 0.01) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            // Speed: 0.005 per tick at 100Hz = 0.5 m/s max velocity
            double z_delta = z * 0.005;
            
            accum_l_trans_.z() += z_delta;
            accum_r_trans_.z() += z_delta;
            
            target_l_.translation() = initial_l_.translation() + accum_l_trans_;
            target_r_.translation() = initial_r_.translation() + accum_r_trans_;
        }
    }
  }

  void joy_callback_r(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.empty()) return;
    right_btn0_ = (msg->buttons[0] == 1);
    right_btn1_ = false;
    for (size_t i = 1; i < msg->buttons.size(); ++i) {
        if (msg->buttons[i] == 1) right_btn1_ = true;
    }
  }

  void clamp_target_angle(Eigen::Isometry3d& target, Eigen::Isometry3d& initial, Eigen::Matrix3d& accum) {
      // 1. Find the absolute rotation from the World Identity frame to the target pose
      // Since the reference is Identity, the relative rotation is just target.linear()
      Eigen::Matrix3d abs_rot = target.linear();
      
      // 2. Convert to Angle-Axis to get the pure 3D rotation angle from World Identity
      Eigen::AngleAxisd angle_axis(abs_rot);
      double angle = angle_axis.angle();
      
      // Eigen AngleAxisd returns an angle in [0, pi]. 
      // If the absolute angle from World Identity exceeds pi/2 (90 degrees), scale it back!
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

  bool left_btn0_ = false, left_btn1_ = false;
  bool right_btn0_ = false, right_btn1_ = false;
  
  double gripper_l_pos_ = 1.1;
  double gripper_r_pos_ = 1.1;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_l_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_r_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_l_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_r_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_joint_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lift_traj_pub_;

  bool hardware_mode_ = true;
  std::atomic<bool> hardware_sync_requested_{false};
  std::string current_mode_ = "BASE";
  sensor_msgs::msg::JointState latest_real_joints_;
  int joint_msg_count_ = 0;

public:
  bool is_hardware_mode() const { return hardware_mode_; }
  bool is_sync_requested() const { return hardware_sync_requested_; }
  void request_hardware_sync() { hardware_sync_requested_ = true; }
  
  int get_and_reset_joint_msg_count() {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      int count = joint_msg_count_;
      joint_msg_count_ = 0;
      return count;
  }

  std::string get_current_mode() const { return current_mode_; }
  void update_grippers(mjModel *m, mjData *d) {
      if (current_mode_ != "ARM") return;
      
      if (left_btn0_) gripper_l_pos_ += 0.01; // close
      if (left_btn1_) gripper_l_pos_ -= 0.01; // open
      gripper_l_pos_ = std::clamp(gripper_l_pos_, 0.0, 1.2);

      if (right_btn0_) gripper_r_pos_ += 0.01; // close
      if (right_btn1_) gripper_r_pos_ -= 0.01; // open
      gripper_r_pos_ = std::clamp(gripper_r_pos_, 0.0, 1.2);

      static int log_counter = 0;
      if ((left_btn0_ || left_btn1_ || right_btn0_ || right_btn1_) && log_counter++ % 50 == 0) {
          RCLCPP_INFO(this->get_logger(), "Grippers -> L: %.2f (btns: %d, %d) | R: %.2f (btns: %d, %d)",
                      gripper_l_pos_, left_btn0_, left_btn1_, gripper_r_pos_, right_btn0_, right_btn1_);
      }
  }
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
  solver_cfg.tolerance = 2.5e-3; // 0.25cm
  solver_cfg.track_orientation = true;

  // Loosen stall conditions to prevent giving up during slow teleop
  solver_cfg.ee_improvement_rate = 1e-5;
  solver_cfg.ee_window = 15;

  // Increase orientation weight significantly so it doesn't get suppressed by
  // damping
  solver_cfg.ori_weight = 0.5;

  // Use the Manipulability solver!
  solver_cfg.nullspace_type = 3;
  solver_cfg.nullspace_amplitude = 0.8;

  ffw_ik::CollisionCostConfig col_cfg;
  col_cfg.collision_margin = 0.10;
  col_cfg.weight_scale = 0.025;

  ffw_ik::IKSolver solver(m);
  SimpleViewer viewer(m);

  std::deque<double> err_hist, dist_hist;

  // Start ROS thread
  std::thread ros_thread([&node]() { rclcpp::spin(node); });

  if (node->is_hardware_mode()) {
      RCLCPP_INFO(node->get_logger(), "Waiting up to 30s for stable /joint_states (>=95Hz for 1s)...");
      auto wait_start = std::chrono::steady_clock::now();
      auto window_start = wait_start;
      node->get_and_reset_joint_msg_count();
      bool stable = false;

      while (rclcpp::ok() && !stable && viewer.enabled()) {
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::seconds>(now - wait_start).count() > 30) {
              RCLCPP_WARN(node->get_logger(), "Timeout waiting for stable /joint_states! Starting anyway.");
              break;
          }
          if (std::chrono::duration_cast<std::chrono::milliseconds>(now - window_start).count() >= 1000) {
              int count = node->get_and_reset_joint_msg_count();
              RCLCPP_INFO(node->get_logger(), "/joint_states rate: %d Hz", count);
              if (count >= 95) {
                  RCLCPP_INFO(node->get_logger(), "Stable /joint_states achieved! Performing initial sync...");
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

  while (viewer.enabled() && rclcpp::ok()) {
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
    ffw_ik::StepResult res =
        solver.solveStep(d, current_target_l, current_target_r, solver_cfg,
                         col_cfg, err_hist, dist_hist);

    // Print solver status every 20 ticks or if stalled
    if (++step_counter % 20 == 0 || res.stalled) {
      // Reduced verbosity
    }
    
    node->update_grippers(m, d);

    // Process any hardware sync requests before mj_forward
    node->apply_hardware_sync(m, d);

    // Ensure simulation state is updated for rendering
    mj_forward(m, d);

    // Dynamic error-bounding clamp (Leash)
    Eigen::Isometry3d achieved_l = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d achieved_r = Eigen::Isometry3d::Identity();
    if (left_id >= 0) {
      achieved_l.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * left_id);
      achieved_l.linear() = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(d->site_xmat + 9 * left_id).cast<double>();
    }
    if (right_id >= 0) {
      achieved_r.translation() = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
      achieved_r.linear() = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(d->site_xmat + 9 * right_id).cast<double>();
    }
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