#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <memory>
#include <vector>
#include <map>
#include <algorithm>

class DualArmTrajectoryCommander : public rclcpp::Node
{
public:
    DualArmTrajectoryCommander() : Node("dual_arm_trajectory_commander"),
                                   left_squeeze_value_(0.0),
                                   right_squeeze_value_(0.0),
                                   has_right_ik_solution_(false),
                                   has_left_ik_solution_(false)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 Dual-Arm Trajectory Commander starting...");

        // Parameters for trajectory timing
        this->declare_parameter<double>("trajectory_duration", 0.1);  // 100ms for responsive control
        this->declare_parameter<bool>("enable_gripper_control", true);

        trajectory_duration_ = this->get_parameter("trajectory_duration").as_double();
        enable_gripper_control_ = this->get_parameter("enable_gripper_control").as_bool();

        RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.3f seconds", trajectory_duration_);
        RCLCPP_INFO(this->get_logger(), "Gripper control: %s", enable_gripper_control_ ? "enabled" : "disabled");

        // Subscribers for IK solutions
        right_ik_solution_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/right_arm_ik_solution", 10,
            std::bind(&DualArmTrajectoryCommander::rightIKSolutionCallback, this, std::placeholders::_1));

        left_ik_solution_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/left_arm_ik_solution", 10,
            std::bind(&DualArmTrajectoryCommander::leftIKSolutionCallback, this, std::placeholders::_1));

        // Subscribers for gripper control (VR squeeze values)
        if (enable_gripper_control_) {
            left_squeeze_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "/vr_hand/left_squeeze", 10,
                std::bind(&DualArmTrajectoryCommander::leftSqueezeCallback, this, std::placeholders::_1));

            right_squeeze_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "/vr_hand/right_squeeze", 10,
                std::bind(&DualArmTrajectoryCommander::rightSqueezeCallback, this, std::placeholders::_1));
        }

        // Publishers for joint trajectories
        right_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

        left_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "✅ Dual-arm trajectory commander initialized");
        RCLCPP_INFO(this->get_logger(), "Waiting for IK solutions from dual_arm_ik_solver...");
    }

private:
    void rightIKSolutionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty() || msg->position.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty right IK solution");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "🎯 Received RIGHT arm IK solution with %zu joints", msg->position.size());

        // Store the latest IK solution
        right_ik_solution_ = *msg;
        has_right_ik_solution_ = true;

        // Create and send joint trajectory
        sendRightArmTrajectory();
    }

    void leftIKSolutionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty() || msg->position.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty left IK solution");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "🎯 Received LEFT arm IK solution with %zu joints", msg->position.size());

        // Store the latest IK solution
        left_ik_solution_ = *msg;
        has_left_ik_solution_ = true;

        // Create and send joint trajectory
        sendLeftArmTrajectory();
    }

    void leftSqueezeCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        left_squeeze_value_ = msg->data;

        // Calculate corresponding gripper position
        // VR squeeze: 0.035=closed (fist), 0.095=open (palm open)
        // Gripper: 1.2=closed, 0=open
        // Map squeeze value to gripper position (invert mapping)
        double gripper_position = 1.2 - ((left_squeeze_value_ - 0.035) * 1.2 / (0.095 - 0.035));
        gripper_position = std::max(0.0, std::min(1.2, gripper_position));

        RCLCPP_DEBUG(this->get_logger(), "🤏 Left squeeze: %.3f → gripper: %.3f", left_squeeze_value_, gripper_position);

        // If we have a recent IK solution, update the trajectory with new gripper value
        if (has_left_ik_solution_) {
            sendLeftArmTrajectory();
        }
    }

    void rightSqueezeCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        right_squeeze_value_ = msg->data;

        // Calculate corresponding gripper position
        // VR squeeze: 0.035=closed (fist), 0.095=open (palm open)
        // Gripper: 1.2=closed, 0=open
        // Map squeeze value to gripper position (invert mapping)
        double gripper_position = 1.2 - ((right_squeeze_value_ - 0.035) * 1.2 / (0.095 - 0.035));
        gripper_position = std::max(0.0, std::min(1.2, gripper_position));

        RCLCPP_DEBUG(this->get_logger(), "🤏 Right squeeze: %.3f → gripper: %.3f", right_squeeze_value_, gripper_position);

        // If we have a recent IK solution, update the trajectory with new gripper value
        if (has_right_ik_solution_) {
            sendRightArmTrajectory();
        }
    }

    void sendRightArmTrajectory()
    {
        if (!has_right_ik_solution_) {
            RCLCPP_WARN(this->get_logger(), "No right IK solution available");
            return;
        }

        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        // trajectory_msg.header.stamp = this->get_clock()->now();
        trajectory_msg.header.frame_id = "base_link";

        // Set joint names (arm joints + gripper joints)
        trajectory_msg.joint_names = right_ik_solution_.name;

        if (enable_gripper_control_) {
            // Add gripper joint names
            trajectory_msg.joint_names.push_back("gripper_r_joint1");
        }

        // Create trajectory point
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();

        // Set arm joint positions from IK solution
        point.positions = right_ik_solution_.position;

        if (enable_gripper_control_) {
            // Calculate gripper position from squeeze value
            double gripper_position = calculateGripperPosition(right_squeeze_value_);

            // Add gripper position (only one joint)
            point.positions.push_back(gripper_position);
        }

        // Set velocities (all zeros for position control)
        point.velocities.resize(point.positions.size(), 0.0);

        // Set accelerations (all zeros)
        point.accelerations.resize(point.positions.size(), 0.0);

        // Set time from start
        // point.time_from_start = rclcpp::Duration::from_nanoseconds(
        //     static_cast<int64_t>(trajectory_duration_ * 1e9));

        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 0;

        trajectory_msg.points.push_back(point);

        // Publish trajectory
        right_joint_trajectory_pub_->publish(trajectory_msg);

        RCLCPP_DEBUG(this->get_logger(), "📤 Published RIGHT arm trajectory with %zu joints",
                    trajectory_msg.joint_names.size());
    }

    void sendLeftArmTrajectory()
    {
        if (!has_left_ik_solution_) {
            RCLCPP_WARN(this->get_logger(), "No left IK solution available");
            return;
        }

        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        // trajectory_msg.header.stamp = this->get_clock()->now();
        trajectory_msg.header.frame_id = "base_link";

        // Set joint names (arm joints + gripper joints)
        trajectory_msg.joint_names = left_ik_solution_.name;

        if (enable_gripper_control_) {
            // Add gripper joint names
            trajectory_msg.joint_names.push_back("gripper_l_joint1");
        }

        // Create trajectory point
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();

        // Set arm joint positions from IK solution
        point.positions = left_ik_solution_.position;

        if (enable_gripper_control_) {
            // Calculate gripper position from squeeze value
            double gripper_position = calculateGripperPosition(left_squeeze_value_);

            // Add gripper position (only one joint)
            point.positions.push_back(gripper_position);
        }

        // Set velocities (all zeros for position control)
        point.velocities.resize(point.positions.size(), 0.0);

        // Set accelerations (all zeros)
        point.accelerations.resize(point.positions.size(), 0.0);

        // Set time from start
        // point.time_from_start = rclcpp::Duration::from_nanoseconds(
        //     static_cast<int64_t>(trajectory_duration_ * 1e9));
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 0;

        trajectory_msg.points.push_back(point);

        // Publish trajectory
        left_joint_trajectory_pub_->publish(trajectory_msg);

        RCLCPP_DEBUG(this->get_logger(), "📤 Published LEFT arm trajectory with %zu joints",
                    trajectory_msg.joint_names.size());
    }

    double calculateGripperPosition(double squeeze_value)
    {
        // VR squeeze mapping to gripper position
        // VR squeeze: 0.035=closed (fist), 0.095=open (palm open)
        // Gripper: 1.2=closed, 0=open
        // Invert the mapping since squeeze increases when closing, but gripper position decreases when opening
        double gripper_position = 1.2 - ((squeeze_value - 0.035) * 1.2 / (0.095 - 0.035));

        // Clamp to valid range
        return std::max(0.0, std::min(1.2, gripper_position));
    }

private:
    // Parameters
    double trajectory_duration_;
    bool enable_gripper_control_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_ik_solution_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_ik_solution_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_squeeze_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_squeeze_sub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_joint_trajectory_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_joint_trajectory_pub_;

    // State variables
    sensor_msgs::msg::JointState right_ik_solution_;
    sensor_msgs::msg::JointState left_ik_solution_;
    double left_squeeze_value_;
    double right_squeeze_value_;
    bool has_right_ik_solution_;
    bool has_left_ik_solution_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DualArmTrajectoryCommander>();

    RCLCPP_INFO(node->get_logger(), "🚀 Dual-Arm Trajectory Commander node started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
