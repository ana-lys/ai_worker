#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>

struct PoseData {
    double x, y, z;
    double qx, qy, qz, qw;
};

class TargetPosePublisher : public rclcpp::Node
{
public:
    TargetPosePublisher() : Node("target_pose_publisher"), pose_index_(0)
    {
        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10);

        // Timer to publish poses every 0.3 seconds
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000),
            std::bind(&TargetPosePublisher::publishTargetPose, this));

        // Initialize pose sequence for demo
        poses_ = {
            // Position 1: Forward
            {0.5, -0.5, 1.3, 0, -0.70710678, 0, 0.70710678},
            // Position 2: Right
            {0.4, -0.7, 1.3, 0, -0.70710678, 0, 0.70710678},
            // Position 3: Left
            {0.4, -0.5, 1.3, 0, -0.70710678, 0, 0.70710678},
            // Position 4: Up
            {0.4, -0.3, 1.5, 0, -0.70710678, 0, 0.70710678},
            // Position 5: Down
            {0.4, -0.2, 1.2, 0, -0.70710678, 0, 0.70710678}

            // {0.7, -0.5, 1.3, 0, -0.70710678, 0, 0.70710678},

            // {0.3, -0.4, 0.7, 0, -0.70710678, 0, 0.70710678},

            // {0.3, -0.4, 0.8, 0, -0.70710678, 0, 0.70710678}

        };

        // poses_ = {
        //     // Position 1: Forward
        //     {0.5, -0.5, 1.3, 0, 0, 0, 1},
        //     // Position 2: Right
        //     {0.4, -0.7, 1.3, 0, 0, 0, 1},
        //     // Position 3: Left
        //     {0.4, -0.5, 1.3, 0, 0, 0, 1},
        //     // Position 4: Up
        //     {0.4, -0.3, 1.5, 0, 0, 0, 1},
        //     // Position 5: Down
        //     {0.4, -0.1, 1.2, 0, 0, 0, 1}
        // };

        RCLCPP_INFO(this->get_logger(), "🚀 Target Pose Publisher started!");
        RCLCPP_INFO(this->get_logger(), "Will publish %zu different target poses in sequence", poses_.size());
        RCLCPP_INFO(this->get_logger(), "Publishing poses every 3 seconds on /target_pose");
    }

private:
    void publishTargetPose()
    {
        if (poses_.empty()) {
            return;
        }

        // Get current pose from sequence
        const PoseData& pose_data = poses_[pose_index_];

        // Create PoseStamped message
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";

        msg.pose.position.x = pose_data.x;
        msg.pose.position.y = pose_data.y;
        msg.pose.position.z = pose_data.z;

        msg.pose.orientation.x = pose_data.qx;
        msg.pose.orientation.y = pose_data.qy;
        msg.pose.orientation.z = pose_data.qz;
        msg.pose.orientation.w = pose_data.qw;

        // Publish the pose
        pose_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "📤 Published target pose %zu/%zu:",
                   pose_index_ + 1, poses_.size());
        RCLCPP_INFO(this->get_logger(), "   Position: [%.3f, %.3f, %.3f]",
                   pose_data.x, pose_data.y, pose_data.z);

        // Move to next pose in sequence
        pose_index_ = (pose_index_ + 1) % poses_.size();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<PoseData> poses_;
    size_t pose_index_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TargetPosePublisher>();

    RCLCPP_INFO(node->get_logger(), "🎯 Target Pose Publisher node started");

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "Target Pose Publisher shutting down...");
    rclcpp::shutdown();
    return 0;
}
