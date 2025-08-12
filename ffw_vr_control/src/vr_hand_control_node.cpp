#include "ffw_vr_control/vr_hand_control_node.hpp"


using namespace std::chrono_literals;

namespace ffw_vr_control
{

VRHandControlNode::~VRHandControlNode() = default;

VRHandControlNode::VRHandControlNode()
: Node("vr_hand_control_node"),
  has_last_solution_l_(false), has_last_solution_r_(false)
{
  this->declare_parameter<std::string>("robot_description", "");
  RCLCPP_INFO(this->get_logger(), "VR Hand Control Node initializing...");

  trajectory_l_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_l_controller/joint_trajectory", 10);
  trajectory_r_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_r_controller/joint_trajectory", 10);
  marker_axes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/fk_axes_markers", 10);
  left_ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/left_ee_pose", 10);
  right_ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/right_ee_pose", 10);
  init_timer_ = this->create_wall_timer(100ms, std::bind(&VRHandControlNode::post_init, this));

  RCLCPP_INFO(this->get_logger(), "VR Hand Control Node initialized.");
  ee_offset_ = -0.21;
  lift_position_ = 0.0;

}

void VRHandControlNode::post_init()
{
  static bool initialized = false;
  if (initialized) return;

  try {
    initialize_ik_solver_common("base_link", "arm_l_link7", ik_solver_l_, kdl_chain_l_, joint_min_, joint_max_, joint_names_l_, "left");
    initialize_ik_solver_common("base_link", "arm_r_link7", ik_solver_r_, kdl_chain_r_, joint_min_r_, joint_max_r_, joint_names_r_, "right");
    initialized = true;

    hand_pose_l_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/vr_hand_pose_l", 10, std::bind(&VRHandControlNode::hand_pose_l_callback, this, std::placeholders::_1));
    hand_pose_r_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/vr_hand_pose_r", 10, std::bind(&VRHandControlNode::hand_pose_r_callback, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&VRHandControlNode::joint_state_callback, this, std::placeholders::_1));
    init_timer_->cancel();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize IK: %s", e.what());
  }
}

KDL::JntArray VRHandControlNode::get_jnt_array_from_joint_state(
  const sensor_msgs::msg::JointState& msg,
  const std::vector<std::string>& kdl_joint_names)
{
  std::unordered_map<std::string, double> joint_position_map;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (i < msg.position.size()) {
      joint_position_map[msg.name[i]] = msg.position[i];
    }
  }

  KDL::JntArray jnt_array(kdl_joint_names.size());
  for (size_t i = 0; i < kdl_joint_names.size(); ++i) {
    const auto& name = kdl_joint_names[i];
    if (joint_position_map.find(name) != joint_position_map.end()) {
      jnt_array(i) = joint_position_map[name];
    } else {
      throw std::runtime_error("Joint " + name + " not found in joint_state");
    }
  }

  return jnt_array;
}

void VRHandControlNode::initialize_ik_solver_common(
  const std::string& base_link,
  const std::string& tip_link,
  std::unique_ptr<TRAC_IK::TRAC_IK>& solver,
  KDL::Chain& chain,
  KDL::JntArray& joint_min,
  KDL::JntArray& joint_max,
  std::vector<std::string>& joint_names,
  const std::string& side)
{
  double timeout = 0.02;
  double eps = 3e-2;

  solver = std::make_unique<TRAC_IK::TRAC_IK>(
    this->shared_from_this(), base_link, tip_link,
    "robot_description", timeout, eps, TRAC_IK::SolveType::Distance);

  if (!solver->getKDLChain(chain)) {
    throw std::runtime_error("Failed to get KDL chain from URDF (" + side + ")");
  }

  joint_names.clear();
  for (size_t i = 0; i < chain.getNrOfSegments(); ++i) {
    const auto& joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      joint_names.push_back(joint.getName());
    }
  }

  if (!solver->getKDLLimits(joint_min, joint_max)) {
    throw std::runtime_error("Failed to get joint limits (" + side + ")");
  }

  RCLCPP_INFO(this->get_logger(), "TRAC-IK initialized with %ld joints (%s)", joint_names.size(), side.c_str());
}

void VRHandControlNode::hand_pose_l_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  last_target_pose_l_ = *msg;

  tf2::Quaternion quat;
  tf2::fromMsg(last_target_pose_l_.orientation, quat);

  tf2::Matrix3x3 rot(quat);

  tf2::Vector3 x_dir = rot * tf2::Vector3(1, 0, 0);

  last_target_pose_l_.position.x += ee_offset_ * x_dir.x();
  last_target_pose_l_.position.y += ee_offset_ * x_dir.y();
  last_target_pose_l_.position.z += ee_offset_ * x_dir.z();

  last_target_pose_l_ = clamp_rpy_and_update_pose(
    last_target_pose_l_,
    std::make_optional(std::make_pair(-70.0, 70.0)),  // roll
    std::make_optional(std::make_pair(-25.0, 85.0)),  // pitch
    std::make_optional(std::make_pair(-70.0, 70.0)),  // yaw
    std::make_pair(0.3, 0.9),   // x range
    std::make_pair(-0.1, 0.5),  // y range
    std::make_pair(0.4 + lift_position_, 1.5 + lift_position_)    // z range
  );

  if (!compute_and_publish_ik_l(last_target_pose_l_)) {
    RCLCPP_WARN(this->get_logger(), "Left IK failed at Z: %.3f", last_target_pose_l_.position.z);
  }
}

void VRHandControlNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  latest_joint_state_ = *msg;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "lift_joint") {
      lift_position_ = msg->position[i];
      break;
    }
  }
}

void VRHandControlNode::hand_pose_r_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{

  last_target_pose_r_ = *msg;

  tf2::Quaternion quat;
  tf2::fromMsg(last_target_pose_r_.orientation, quat);

  tf2::Matrix3x3 rot(quat);

  tf2::Vector3 x_dir = rot * tf2::Vector3(1, 0, 0);

  last_target_pose_r_.position.x += ee_offset_ * x_dir.x();
  last_target_pose_r_.position.y += ee_offset_ * x_dir.y();
  last_target_pose_r_.position.z += ee_offset_ * x_dir.z();

  last_target_pose_r_ = clamp_rpy_and_update_pose(
    last_target_pose_r_,
    std::make_optional(std::make_pair(-70.0, 70.0)),  // roll
    std::make_optional(std::make_pair(-25.0, 85.0)),  // pitch
    std::make_optional(std::make_pair(-70.0, 70.0)),  // yaw
    std::make_pair(0.3, 0.9),   // x range
    std::make_pair(-0.5, 0.1),  // y range
    std::make_pair(0.4 + lift_position_, 1.5 + lift_position_)    // z range
  );

  if (!compute_and_publish_ik_r(last_target_pose_r_)) {
    RCLCPP_WARN(this->get_logger(), "Right IK failed at Z: %.3f", last_target_pose_r_.position.z);
  }
}

bool VRHandControlNode::compute_and_publish_ik_l(const geometry_msgs::msg::Pose& pose)
{
  return compute_ik_common(pose, ik_solver_l_, joint_min_, joint_max_, joint_names_l_, kdl_chain_l_, last_solution_l_, has_last_solution_l_, trajectory_l_pub_, false);
}

bool VRHandControlNode::compute_and_publish_ik_r(const geometry_msgs::msg::Pose& pose)
{
  return compute_ik_common(pose, ik_solver_r_, joint_min_r_, joint_max_r_, joint_names_r_, kdl_chain_r_, last_solution_r_, has_last_solution_r_, trajectory_r_pub_, true);
}

std::vector<KDL::JntArray> VRHandControlNode::get_seed_candidates(
  bool is_right_arm,
  const KDL::JntArray& joint_min,
  const KDL::JntArray& last_solution,
  bool has_last_solution)
{
  std::vector<std::vector<double>> presets = is_right_arm ?
    std::vector<std::vector<double>>{
      {0.0, -0.5, -0.2,  1.0, 0.0, -1.5, 0.0},
      {0.0, -0.3, -0.3, -1.1, -0.2, -1.3, 0.1},
      {0.0, -0.7,  0.2, -1.5, 0.0, -1.8, -0.3}
    } :
    std::vector<std::vector<double>>{
      {0.0, -0.5,  0.2, -1.0, 0.0, 1.5, 0.0},
      {0.0, -0.3,  0.3, -1.1, 0.2, 1.3, -0.1},
      {0.0, -0.7, -0.2, -1.5, 0.0, 1.8, 0.3}
    };

  std::vector<KDL::JntArray> seeds;
  for (const auto& posture : presets) {
    KDL::JntArray seed(joint_min.rows());
    for (size_t i = 0; i < posture.size(); ++i)
      seed(i) = posture[i];
    seeds.push_back(seed);
  }

  if (has_last_solution && last_solution.rows() == joint_min.rows())
    seeds.push_back(last_solution);

  return seeds;
}

bool VRHandControlNode::compute_ik_common(
  const geometry_msgs::msg::Pose& pose,
  const std::unique_ptr<TRAC_IK::TRAC_IK>& solver,
  const KDL::JntArray& joint_min, const KDL::JntArray& joint_max,
  const std::vector<std::string>& joint_names, const KDL::Chain& kdl_chain,
  KDL::JntArray& last_solution, bool& has_last_solution,
  const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub,
  bool is_right_arm)
{
  constexpr double POSITION_TOLERANCE = 0.1;        // meters
  constexpr double ORIENTATION_TOLERANCE = 0.15;    // radians
  constexpr double JOINT_DELTA_LIMIT = 0.6;          // rad (per joint max step)

  KDL::Frame target_frame;
  tf2::fromMsg(pose, target_frame);
  target_frame.p.z(target_frame.p.z() - lift_position_);
  target_frame.M = target_frame.M * KDL::Rotation::RotY(-M_PI_2);

  auto seeds = get_seed_candidates(is_right_arm, joint_min, last_solution, has_last_solution);

  for (const auto& seed : seeds) {
    KDL::JntArray result;
    if (solver->CartToJnt(seed, target_frame, result) >= 0) {
      // Forward Kinematics to verify IK accuracy
      KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
      KDL::Frame fk_result;
      if (fk_solver.JntToCart(result, fk_result) < 0) continue;

      if (!is_pose_error_acceptable(fk_result, target_frame, POSITION_TOLERANCE, ORIENTATION_TOLERANCE)) {
        RCLCPP_WARN(this->get_logger(), "FK vs Target pose mismatch too large. Skipping trajectory.");
        continue;
      }

      // Joint delta 검증
      if (has_last_solution && result.rows() == last_solution.rows()) {
        bool excessive_jump = false;
        for (size_t i = 0; i < result.rows(); ++i) {
          double delta = std::abs(result(0) - last_solution(0));
          if (delta > JOINT_DELTA_LIMIT) {
            RCLCPP_WARN(this->get_logger(), "Joint %zu delta too large: %.3f rad. Skipping trajectory.", i, delta);
            excessive_jump = true;
            break;
          }
        }
        if (excessive_jump) continue;

        // Optional: smoothing
        for (size_t i = 0; i < result.rows(); ++i) {
          result(i) = 0.1 * result(i) + 0.9 * last_solution(i);
        }
      }

      last_solution = result;
      has_last_solution = true;

      std::string side = is_right_arm ? "right" : "left";
      publish_fk_pose(kdl_chain, "base_link", side, joint_names);

      std::vector<std::string> names(joint_names.begin() + 1, joint_names.end());
      std::vector<double> positions;
      for (size_t i = 1; i < result.rows(); ++i)
        positions.push_back(result(i));

      auto traj = create_trajectory(positions, {}, names, last_solution);
      pub->publish(traj);
      return true;
    }
  }

  return false;
}

void VRHandControlNode::publish_fk_pose(
  const KDL::Chain& chain,
  const std::string& frame_id,
  const std::string& side,
  const std::vector<std::string>& joint_names)
{
  try {
    KDL::JntArray actual_joint_array = get_jnt_array_from_joint_state(latest_joint_state_, joint_names);

    if (actual_joint_array.rows() != chain.getNrOfJoints()) return;

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame ee_frame;
    if (fk_solver.JntToCart(actual_joint_array, ee_frame) < 0) return;

    // ee_frame.p.z(ee_frame.p.z() - lift_position_);

    const KDL::Vector offset_local(0.0, 0.0, ee_offset_);
    const KDL::Vector offset_world = ee_frame.M * offset_local;
    ee_frame.p = ee_frame.p + offset_world;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    msg.pose = tf2::toMsg(ee_frame);

    if (side == "left") {
      left_ee_pose_pub_->publish(msg);
    } else if (side == "right") {
      right_ee_pose_pub_->publish(msg);
    }

    auto marker_array = create_axes_arrows(msg.pose, side);
    marker_axes_pub_->publish(marker_array);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "FK pose publish failed: %s", e.what());
  }
}

bool VRHandControlNode::is_pose_error_acceptable(
  const KDL::Frame& fk_result,
  const KDL::Frame& target,
  double pos_tol,
  double rot_tol)
{
  // Position error
  double pos_error = (fk_result.p - target.p).Norm();

  // Orientation error
  KDL::Rotation R_err = fk_result.M.Inverse() * target.M;
  double rx, ry, rz;
  R_err.GetRPY(rx, ry, rz);
  double rot_error = std::sqrt(rx * rx + ry * ry + rz * rz);

  return pos_error < pos_tol && rot_error < rot_tol;
}

trajectory_msgs::msg::JointTrajectory VRHandControlNode::create_trajectory(
  const std::vector<double>& positions,
  const std::vector<double>& velocities,
  const std::vector<std::string>& joint_names,
  const KDL::JntArray& last_solution)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities = velocities.empty() ? std::vector<double>(positions.size(), 0.0) : velocities;
  point.accelerations = std::vector<double>(positions.size(), 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(0.01);

  traj.points.push_back(point);
  return traj;
}

visualization_msgs::msg::MarkerArray VRHandControlNode::create_axes_arrows(
  const geometry_msgs::msg::Pose& pose,
  const std::string& side)
{
  visualization_msgs::msg::MarkerArray marker_array;

  int base_id = (side == "left") ? 10 : 20;
  std::map<std::string, std::tuple<float, float, float>> color_map = {
    {"x", {1.0f, 0.0f, 0.0f}},
    {"y", {0.0f, 1.0f, 0.0f}},
    {"z", {0.0f, 0.0f, 1.0f}},
  };

  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 rot(q);
  geometry_msgs::msg::Point start = pose.position;
  double length = 0.15;

  for (int i = 0; i < 3; ++i) {
    tf2::Vector3 axis_vec = rot.getColumn(i);
    char axis_char = "xyz"[i];
    std::string axis(1, axis_char);

    geometry_msgs::msg::Point end;
    end.x = start.x + axis_vec.x() * length;
    end.y = start.y + axis_vec.y() * length;
    end.z = start.z + axis_vec.z() * length;

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "base_link";
    arrow.header.stamp = this->now();
    arrow.ns = side + "_axes";
    arrow.id = base_id + i;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.02;
    arrow.scale.y = 0.03;
    arrow.color.r = std::get<0>(color_map[axis]);
    arrow.color.g = std::get<1>(color_map[axis]);
    arrow.color.b = std::get<2>(color_map[axis]);
    arrow.color.a = 1.0;
    arrow.points = {start, end};

    marker_array.markers.push_back(arrow);
  }

  return marker_array;
}

geometry_msgs::msg::Pose VRHandControlNode::clamp_rpy_and_update_pose(
  const geometry_msgs::msg::Pose& input_pose,
  std::optional<std::pair<double, double>> roll_deg,
  std::optional<std::pair<double, double>> pitch_deg,
  std::optional<std::pair<double, double>> yaw_deg,
  std::optional<std::pair<double, double>> x_range,
  std::optional<std::pair<double, double>> y_range,
  std::optional<std::pair<double, double>> z_range)
{
  auto deg2rad = [](double deg) { return deg * M_PI / 180.0; };

  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(input_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  if (roll_deg)  roll = std::clamp(roll, deg2rad(roll_deg->first), deg2rad(roll_deg->second));
  if (pitch_deg) pitch = std::clamp(pitch, deg2rad(pitch_deg->first), deg2rad(pitch_deg->second));
  if (yaw_deg)   yaw = std::clamp(yaw, deg2rad(yaw_deg->first), deg2rad(yaw_deg->second));

  tf2::Quaternion q_clamped;
  q_clamped.setRPY(roll, pitch, yaw);
  q_clamped.normalize();

  geometry_msgs::msg::Pose clamped_pose = input_pose;
  clamped_pose.orientation = tf2::toMsg(q_clamped);

  if (x_range) clamped_pose.position.x = std::clamp(clamped_pose.position.x, x_range->first, x_range->second);
  if (y_range) clamped_pose.position.y = std::clamp(clamped_pose.position.y, y_range->first, y_range->second);
  if (z_range) clamped_pose.position.z = std::clamp(clamped_pose.position.z, z_range->first, z_range->second);

  return clamped_pose;
}


} // namespace ffw_vr_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ffw_vr_control::VRHandControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
