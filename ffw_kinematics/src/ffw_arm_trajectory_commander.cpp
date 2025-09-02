#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <memory>
#include <vector>
#include <map>
#include <algorithm>

class FfwArmTrajectoryCommander : public rclcpp::Node
{
public:
    FfwArmTrajectoryCommander() : Node("ffw_arm_trajectory_commander"),
                                   left_squeeze_value_(0.0),
                                   right_squeeze_value_(0.0),
                                   has_right_ik_solution_(false),
                                   has_left_ik_solution_(false)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 Dual-Arm Trajectory Commander starting...");

        // Parameters for trajectory timing
        this->declare_parameter<double>("trajectory_duration", 0.01);  // 10ms for responsive control
        this->declare_parameter<bool>("enable_gripper_control", true);

        // Gripper mapping parameters
        this->declare_parameter<double>("vr_squeeze_closed", 0.035);   // VR squeeze value when closed (fist)
        this->declare_parameter<double>("vr_squeeze_open", 0.095);     // VR squeeze value when open (palm open)
        this->declare_parameter<double>("gripper_pos_closed", 1.2);    // Gripper position when closed
        this->declare_parameter<double>("gripper_pos_open", 0.0);      // Gripper position when open

        trajectory_duration_ = this->get_parameter("trajectory_duration").as_double();
        enable_gripper_control_ = this->get_parameter("enable_gripper_control").as_bool();

        vr_squeeze_closed_ = this->get_parameter("vr_squeeze_closed").as_double();
        vr_squeeze_open_ = this->get_parameter("vr_squeeze_open").as_double();
        gripper_pos_closed_ = this->get_parameter("gripper_pos_closed").as_double();
        gripper_pos_open_ = this->get_parameter("gripper_pos_open").as_double();

        RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.3f seconds", trajectory_duration_);
        RCLCPP_INFO(this->get_logger(), "Gripper control: %s", enable_gripper_control_ ? "enabled" : "disabled");

        // Subscribers for IK solutions
        right_ik_solution_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/right_arm_ik_solution", 10,
            std::bind(&FfwArmTrajectoryCommander::rightIKSolutionCallback, this, std::placeholders::_1));

        left_ik_solution_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/left_arm_ik_solution", 10,
            std::bind(&FfwArmTrajectoryCommander::leftIKSolutionCallback, this, std::placeholders::_1));

        // Subscribers for gripper control (VR squeeze values)
        if (enable_gripper_control_) {
            left_squeeze_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "/vr_hand/left_squeeze", 10,
                std::bind(&FfwArmTrajectoryCommander::leftSqueezeCallback, this, std::placeholders::_1));

            right_squeeze_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "/vr_hand/right_squeeze", 10,
                std::bind(&FfwArmTrajectoryCommander::rightSqueezeCallback, this, std::placeholders::_1));
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

        // Validate joint names and positions have same size
        if (msg->name.size() != msg->position.size()) {
            RCLCPP_ERROR(this->get_logger(), "Right IK solution: joint names (%zu) and positions (%zu) size mismatch",
                        msg->name.size(), msg->position.size());
            return;
        }

        // Check for NaN or infinite values
        for (const auto& pos : msg->position) {
            if (!std::isfinite(pos)) {
                RCLCPP_ERROR(this->get_logger(), "Right IK solution contains invalid joint position: %f", pos);
                return;
            }
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

        // Validate joint names and positions have same size
        if (msg->name.size() != msg->position.size()) {
            RCLCPP_ERROR(this->get_logger(), "Left IK solution: joint names (%zu) and positions (%zu) size mismatch",
                        msg->name.size(), msg->position.size());
            return;
        }

        // Check for NaN or infinite values
        for (const auto& pos : msg->position) {
            if (!std::isfinite(pos)) {
                RCLCPP_ERROR(this->get_logger(), "Left IK solution contains invalid joint position: %f", pos);
                return;
            }
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

        // Calculate corresponding gripper position using parameterized mapping
        double gripper_position = calculateGripperPosition(left_squeeze_value_);

        RCLCPP_DEBUG(this->get_logger(), "🤏 Left squeeze: %.3f → gripper: %.3f", left_squeeze_value_, gripper_position);

        // If we have a recent IK solution, update the trajectory with new gripper value
        if (has_left_ik_solution_) {
            sendLeftArmTrajectory();
        }
    }

    void rightSqueezeCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        right_squeeze_value_ = msg->data;

        // Calculate corresponding gripper position using parameterized mapping
        double gripper_position = calculateGripperPosition(right_squeeze_value_);

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
        // Validate input range
        if (squeeze_value < vr_squeeze_closed_ || squeeze_value > vr_squeeze_open_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "VR squeeze value %.3f out of expected range [%.3f, %.3f]",
                                squeeze_value, vr_squeeze_closed_, vr_squeeze_open_);
        }

        // Normalize squeeze value to [0, 1] range
        double normalized = (squeeze_value - vr_squeeze_closed_) / (vr_squeeze_open_ - vr_squeeze_closed_);
        normalized = std::max(0.0, std::min(1.0, normalized));

        // Map to gripper position (inverted: squeeze increases -> gripper closes)
        double gripper_position = gripper_pos_closed_ - (normalized * (gripper_pos_closed_ - gripper_pos_open_));

        // Clamp to valid range
        return std::max(gripper_pos_open_, std::min(gripper_pos_closed_, gripper_position));
    }

private:
    // Parameters
    double trajectory_duration_;
    bool enable_gripper_control_;

    // Gripper mapping parameters
    double vr_squeeze_closed_;
    double vr_squeeze_open_;
    double gripper_pos_closed_;
    double gripper_pos_open_;

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

    auto node = std::make_shared<FfwArmTrajectoryCommander>();

    RCLCPP_INFO(node->get_logger(), "🚀 Dual-Arm Trajectory Commander node started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
