#pragma once

#include <memory>
#include <vector>
#include <string>
#include <optional>
#include <utility>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>
#include <chrono>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace ffw_vr_control
{

class VRHandControlNode : public rclcpp::Node
{
public:
  VRHandControlNode();
  ~VRHandControlNode();

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_pose_l_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_pose_r_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_l_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_r_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_axes_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_ee_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_ee_pose_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_l_;
  std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_r_;
  KDL::Chain kdl_chain_l_, kdl_chain_r_;
  KDL::JntArray joint_min_, joint_max_, joint_min_r_, joint_max_r_;
  std::vector<std::string> joint_names_l_, joint_names_r_;

  KDL::JntArray last_solution_l_, last_solution_r_;
  bool has_last_solution_l_, has_last_solution_r_;

  geometry_msgs::msg::Pose last_target_pose_l_, last_target_pose_r_;

  void post_init();
  void initialize_ik_solver_common(
    const std::string& base_link,
    const std::string& tip_link,
    std::unique_ptr<TRAC_IK::TRAC_IK>& solver,
    KDL::Chain& chain,
    KDL::JntArray& joint_min,
    KDL::JntArray& joint_max,
    std::vector<std::string>& joint_names,
    const std::string& side);

  void hand_pose_l_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void hand_pose_r_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  bool compute_and_publish_ik_l(const geometry_msgs::msg::Pose& pose);
  bool compute_and_publish_ik_r(const geometry_msgs::msg::Pose& pose);

  bool compute_ik_common(
    const geometry_msgs::msg::Pose& pose,
    const std::unique_ptr<TRAC_IK::TRAC_IK>& solver,
    const KDL::JntArray& joint_min,
    const KDL::JntArray& joint_max,
    const std::vector<std::string>& joint_names,
    const KDL::Chain& kdl_chain,
    KDL::JntArray& last_solution,
    bool& has_last_solution,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub,
    bool is_right_arm);

  std::vector<KDL::JntArray> get_seed_candidates(
    bool is_right_arm,
    const KDL::JntArray& joint_min,
    const KDL::JntArray& last_solution,
    bool has_last_solution);

  bool is_pose_error_acceptable(
    const KDL::Frame& fk_result,
    const KDL::Frame& target,
    double pos_tol,
    double rot_tol);

  void publish_fk_pose(
    const KDL::Chain& chain,
    const std::string& frame_id,
    const std::string& side,
    const std::vector<std::string>& joint_names);

  visualization_msgs::msg::MarkerArray create_axes_arrows(
    const geometry_msgs::msg::Pose& pose,
    const std::string& side);

  geometry_msgs::msg::Pose clamp_rpy_and_update_pose(
    const geometry_msgs::msg::Pose& input_pose,
    std::optional<std::pair<double, double>> roll_deg = std::nullopt,
    std::optional<std::pair<double, double>> pitch_deg = std::nullopt,
    std::optional<std::pair<double, double>> yaw_deg = std::nullopt,
    std::optional<std::pair<double, double>> x_range = std::nullopt,
    std::optional<std::pair<double, double>> y_range = std::nullopt,
    std::optional<std::pair<double, double>> z_range = std::nullopt);

  trajectory_msgs::msg::JointTrajectory create_trajectory(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<std::string>& joint_names,
    const KDL::JntArray& last_solution);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  KDL::JntArray get_jnt_array_from_joint_state(
    const sensor_msgs::msg::JointState& msg,
    const std::vector<std::string>& kdl_joint_names);

  sensor_msgs::msg::JointState latest_joint_state_;

  double lift_position_;
  double ee_offset_;
};

}  // namespace ffw_vr_control
