#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class VRHandPoseTransformer : public rclcpp::Node
{
public:
    VRHandPoseTransformer() : Node("vr_hand_pose_transformer")
    {
        // Declare and get VR scale parameter
        // 로봇 작업공간(836mm) vs 성인남성 팔길이(~650mm) 비율 고려
        // 로봇이 더 크므로 VR 움직임을 확대: 836/650 ≈ 1.29,
        this->declare_parameter<double>("vr_scale", 1.2);
        vr_scale_ = this->get_parameter("vr_scale").as_double();

        // Subscribe to VR hand poses for both hands
        right_hand_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/vr_hand/right_poses", 10,
            std::bind(&VRHandPoseTransformer::right_hand_pose_callback, this, std::placeholders::_1));
            
        left_hand_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/vr_hand/left_poses", 10,
            std::bind(&VRHandPoseTransformer::left_hand_pose_callback, this, std::placeholders::_1));

        // Publishers for target poses to IK solvers
        right_target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/right_target_pose", 10);
            
        left_target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/left_target_pose", 10);

        RCLCPP_INFO(this->get_logger(), "VR Hand Pose Transformer node started - supporting both hands (vr_scale=%.3f)", vr_scale_);
    }

private:
    void right_hand_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        transform_and_publish_pose(msg, right_target_pose_pub_, "right");
    }
    
    void left_hand_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        transform_and_publish_pose(msg, left_target_pose_pub_, "left");
    }

    void transform_and_publish_pose(
        const geometry_msgs::msg::PoseArray::SharedPtr msg,
        const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
        const std::string& hand_name)
    {
        if (msg->poses.empty())
        {
            return;
        }

        // Assume the first pose in the array is the wrist pose
        // (or find the wrist pose based on your VR system's convention)
        const auto& wrist_pose_vr = msg->poses[0];  // Adjust index if needed

        // Extract position from VR pose (Meta Quest/Unity 좌표계)
        Eigen::Vector3d vr_position(
            wrist_pose_vr.position.x,
            wrist_pose_vr.position.y,
            wrist_pose_vr.position.z
        );

        // Extract orientation from VR pose
        Eigen::Quaterniond vr_quaternion(
            wrist_pose_vr.orientation.w,  // w comes first in Eigen
            wrist_pose_vr.orientation.x,
            wrist_pose_vr.orientation.y,
            wrist_pose_vr.orientation.z
        );

        // === VR(Meta Quest/Unity) → ROS 변환 (position, with scale) ===
        // ROS.x = -VR.z, ROS.y = -VR.x, ROS.z = VR.y
        Eigen::Vector3d ros_position;
        ros_position.x() = -vr_position.z() * vr_scale_;
        ros_position.y() = -vr_position.x() * vr_scale_;
        ros_position.z() =  vr_position.y() * vr_scale_;

        // 기준 좌표: zedm_camera_center에서 base_link로 이동 (전체 변환 체인 적용)
        // zedm_camera_center → zedm_camera_link: (0, 0, -0.01325)
        // zedm_camera_link → head_link2: (-0.0238122, 0.00651797, 0.0242094)
        // head_link2 → head_link1: (-0.040, 0, -0.054)
        // head_link1 → arm_base_link: (-0.049483, 0, -0.102130)
        // arm_base_link → base_link: (-0.0055, 0, -1.4316) + lift_joint_position

        // 전체 변환: zedm_camera_center → base_link
        Eigen::Vector3d zedm_to_base_offset(
            0.0 - 0.0238122 - 0.040 - 0.049483 - 0.0055,           // x: -0.1132952
            0.0 + 0.0 + 0.0 + 0.0 + 0.0,                    // y: 0.00651797
            -0.01325 + 0.0242094 - 0.054 - 0.102130 - 1.4316       // z: -1.5766
        );

        Eigen::Vector3d base_position = ros_position;
        base_position.x() -= zedm_to_base_offset.x();
        base_position.y() -= zedm_to_base_offset.y();
        base_position.z() -= zedm_to_base_offset.z();
        
        RCLCPP_INFO(this->get_logger(), "[DEBUG] %s VR raw pos: [%.4f, %.4f, %.4f]  ROS pos: [%.4f, %.4f, %.4f]  Offset: [%.4f, %.4f, %.4f]  base_link pos: [%.4f, %.4f, %.4f]",
            hand_name.c_str(),
            vr_position.x(), vr_position.y(), vr_position.z(),
            ros_position.x(), ros_position.y(), ros_position.z(),
            zedm_to_base_offset.x(), zedm_to_base_offset.y(), zedm_to_base_offset.z(),
            base_position.x(), base_position.y(), base_position.z());

        // === VR(Meta Quest/Unity) → ROS 변환 (orientation) ===
        // 변환 행렬: (VR → ROS)
        //   [ 0,  0, -1 ]
        //   [ -1, 0,  0 ]
        //   [ 0,  1,  0 ]
        Eigen::Matrix3d vr_to_ros;
        vr_to_ros <<  0, 0, -1,
                 -1, 0, 0,
                  0, 1, 0;

        Eigen::Matrix3d vr_rot = vr_quaternion.toRotationMatrix();
        Eigen::Matrix3d ros_rot = vr_to_ros * vr_rot;

        // 추가: Z축으로 180도 회전 행렬 곱하기 (오른손만)
        if (hand_name == "right") {
            Eigen::AngleAxisd rot_z_180(M_PI, Eigen::Vector3d::UnitZ());
            ros_rot = ros_rot * rot_z_180.toRotationMatrix();
        }

        Eigen::Quaterniond arm_quaternion(ros_rot);

        // Create target pose message
        auto target_pose = geometry_msgs::msg::PoseStamped();
        target_pose.header.stamp = this->get_clock()->now();
        target_pose.header.frame_id = "base_link";

        target_pose.pose.position.x = base_position.x();
        target_pose.pose.position.y = base_position.y();
        target_pose.pose.position.z = base_position.z();

        target_pose.pose.orientation.x = arm_quaternion.x();
        target_pose.pose.orientation.y = arm_quaternion.y();
        target_pose.pose.orientation.z = arm_quaternion.z();
        target_pose.pose.orientation.w = arm_quaternion.w();
        
        // Publish target pose
        publisher->publish(target_pose);

        RCLCPP_DEBUG(this->get_logger(),
            "Transformed %s VR pose: pos=[%.3f, %.3f, %.3f]",
            hand_name.c_str(), base_position.x(), base_position.y(), base_position.z());
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr right_hand_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr left_hand_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_pose_pub_;
    double vr_scale_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VRHandPoseTransformer>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
