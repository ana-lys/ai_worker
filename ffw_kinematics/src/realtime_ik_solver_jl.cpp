#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>
#include <memory>
#include <vector>
#include <map>
#include <algorithm>

class RealtimeIKSolverJL : public rclcpp::Node
{
public:
    RealtimeIKSolverJL() : Node("realtime_ik_solver_jl"),
                           lift_joint_index_(-1),
                           setup_complete_(false),
                           has_joint_states_(false),
                           left_pinch_value_(0.0),
                           right_pinch_value_(0.0)
    {
        // Parameters
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("arm_base_link", "arm_base_link");
        this->declare_parameter<std::string>("right_end_effector_link", "arm_r_link7");
        this->declare_parameter<std::string>("left_end_effector_link", "arm_l_link7");
        this->declare_parameter<std::string>("right_target_pose_topic", "/right_target_pose");
        this->declare_parameter<std::string>("left_target_pose_topic", "/left_target_pose");

        base_link_ = this->get_parameter("base_link").as_string();
        arm_base_link_ = this->get_parameter("arm_base_link").as_string();
        right_end_effector_link_ = this->get_parameter("right_end_effector_link").as_string();
        left_end_effector_link_ = this->get_parameter("left_end_effector_link").as_string();
        std::string right_target_pose_topic = this->get_parameter("right_target_pose_topic").as_string();
        std::string left_target_pose_topic = this->get_parameter("left_target_pose_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "🚀 Dual-Arm Realtime IK Solver with Joint Limits starting...");
        RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Arm base link: %s", arm_base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right end effector link: %s", right_end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Left end effector link: %s", left_end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right target pose topic: %s", right_target_pose_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Left target pose topic: %s", left_target_pose_topic.c_str());

        // Subscribers
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&RealtimeIKSolverJL::robotDescriptionCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RealtimeIKSolverJL::jointStateCallback, this, std::placeholders::_1));

        // Subscribe to target pose topics for real-time IK solving
        right_target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            right_target_pose_topic, 10,
            std::bind(&RealtimeIKSolverJL::rightTargetPoseCallback, this, std::placeholders::_1));

        left_target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            left_target_pose_topic, 10,
            std::bind(&RealtimeIKSolverJL::leftTargetPoseCallback, this, std::placeholders::_1));

        // Subscribe to VR pinch values
        left_pinch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vr_hand/left_pinch", 10,
            std::bind(&RealtimeIKSolverJL::leftPinchCallback, this, std::placeholders::_1));

        right_pinch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vr_hand/right_pinch", 10,
            std::bind(&RealtimeIKSolverJL::rightPinchCallback, this, std::placeholders::_1));

        // Publishers
        right_current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/right_current_end_effector_pose", 10);

        left_current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/left_current_end_effector_pose", 10);

        right_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

        left_joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory", 10);

        // Timer for publishing current poses at 10Hz
        pose_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RealtimeIKSolverJL::publishCurrentPoses, this));

        // Try to get robot_description from parameter server
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");

        if (param_client->wait_for_service(std::chrono::seconds(2))) {
            try {
                auto parameters = param_client->get_parameters({"robot_description"});
                if (!parameters.empty() && parameters[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    std::string robot_description = parameters[0].as_string();
                    processRobotDescription(robot_description);
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to get robot_description from parameter server: %s", e.what());
            }
        }

        RCLCPP_INFO(this->get_logger(), "✅ Dual-arm node initialized. Waiting for target poses on:");
        RCLCPP_INFO(this->get_logger(), "   Right arm: %s", right_target_pose_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "   Left arm: %s", left_target_pose_topic.c_str());
    }

private:
    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received robot_description via topic");
        processRobotDescription(msg->data);
    }

    void processRobotDescription(const std::string& robot_description)
    {
        RCLCPP_INFO(this->get_logger(), "Processing robot_description (%zu bytes)", robot_description.size());

        try {
            // Parse URDF
            urdf::Model model;
            if (!model.initString(robot_description)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "URDF parsed successfully: %s", model.getName().c_str());

            // Create KDL tree
            KDL::Tree tree;
            if (!kdl_parser::treeFromUrdfModel(model, tree)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create KDL tree from URDF");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "KDL tree created with %d segments", tree.getNrOfSegments());

            // Extract chains from tree: arm_base_link -> end_effector_links
            if (!tree.getChain(arm_base_link_, right_end_effector_link_, right_chain_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to extract right arm chain from %s to %s",
                           arm_base_link_.c_str(), right_end_effector_link_.c_str());

                // Print available segments for debugging
                auto segments = tree.getSegments();
                RCLCPP_INFO(this->get_logger(), "Available segments:");
                for (const auto& seg : segments) {
                    RCLCPP_INFO(this->get_logger(), "  - %s", seg.first.c_str());
                }
                return;
            }

            if (!tree.getChain(arm_base_link_, left_end_effector_link_, left_chain_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to extract left arm chain from %s to %s",
                           arm_base_link_.c_str(), left_end_effector_link_.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Right KDL chain extracted with %d joints", right_chain_.getNrOfJoints());
            RCLCPP_INFO(this->get_logger(), "Left KDL chain extracted with %d joints", left_chain_.getNrOfJoints());

            // Extract joint names for both arms
            extractJointNames();

            // Setup joint limits from URDF for both arms
            setupJointLimits(model);

            // Create solvers with joint limits for both arms
            right_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(right_chain_);
            right_ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(right_chain_);
            right_ik_solver_jl_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
                right_chain_, right_q_min_, right_q_max_, *right_fk_solver_, *right_ik_vel_solver_, 20000, 0.03);

            left_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(left_chain_);
            left_ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(left_chain_);
            left_ik_solver_jl_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
                left_chain_, left_q_min_, left_q_max_, *left_fk_solver_, *left_ik_vel_solver_, 20000, 0.03);

            setup_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ Dual-arm KDL setup with Joint Limits completed successfully! Ready for real-time IK.");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing robot description: %s", e.what());
        }
    }

    void extractJointNames()
    {
        // Extract right arm joint names
        right_joint_names_.clear();
        for (unsigned int i = 0; i < right_chain_.getNrOfSegments(); i++) {
            KDL::Segment segment = right_chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None) {
                std::string joint_name = segment.getJoint().getName();
                right_joint_names_.push_back(joint_name);
                RCLCPP_INFO(this->get_logger(), "Found right arm joint: %s", joint_name.c_str());
            }
        }

        // Extract left arm joint names
        left_joint_names_.clear();
        for (unsigned int i = 0; i < left_chain_.getNrOfSegments(); i++) {
            KDL::Segment segment = left_chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None) {
                std::string joint_name = segment.getJoint().getName();
                left_joint_names_.push_back(joint_name);
                RCLCPP_INFO(this->get_logger(), "Found left arm joint: %s", joint_name.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Right arm joint names extracted:");
        for (size_t i = 0; i < right_joint_names_.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, right_joint_names_[i].c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Left arm joint names extracted:");
        for (size_t i = 0; i < left_joint_names_.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, left_joint_names_[i].c_str());
        }
    }

    void setupJointLimits(const urdf::Model& model)
    {
        // Setup right arm joint limits
        unsigned int right_num_joints = right_chain_.getNrOfJoints();
        right_q_min_.resize(right_num_joints);
        right_q_max_.resize(right_num_joints);

        RCLCPP_INFO(this->get_logger(), "🔒 Setting up right arm joint limits:");
        for (size_t i = 0; i < right_joint_names_.size(); i++) {
            const std::string& joint_name = right_joint_names_[i];
            auto joint_ptr = model.getJoint(joint_name);

            if (joint_ptr && joint_ptr->limits) {
                right_q_min_(i) = joint_ptr->limits->lower;
                right_q_max_(i) = joint_ptr->limits->upper;
                RCLCPP_INFO(this->get_logger(), "  %s: [%.3f, %.3f] rad ([%.1f°, %.1f°])",
                           joint_name.c_str(),
                           right_q_min_(i), right_q_max_(i),
                           right_q_min_(i) * 180.0 / M_PI, right_q_max_(i) * 180.0 / M_PI);
            } else {
                right_q_min_(i) = -3.14159;
                right_q_max_(i) = 3.14159;
                RCLCPP_WARN(this->get_logger(), "  %s: Using default limits [%.3f, %.3f] rad ([%.1f°, %.1f°])",
                           joint_name.c_str(),
                           right_q_min_(i), right_q_max_(i),
                           right_q_min_(i) * 180.0 / M_PI, right_q_max_(i) * 180.0 / M_PI);
            }
        }

        // Setup left arm joint limits
        unsigned int left_num_joints = left_chain_.getNrOfJoints();
        left_q_min_.resize(left_num_joints);
        left_q_max_.resize(left_num_joints);

        RCLCPP_INFO(this->get_logger(), "🔒 Setting up left arm joint limits:");
        for (size_t i = 0; i < left_joint_names_.size(); i++) {
            const std::string& joint_name = left_joint_names_[i];
            auto joint_ptr = model.getJoint(joint_name);

            if (joint_ptr && joint_ptr->limits) {
                left_q_min_(i) = joint_ptr->limits->lower;
                left_q_max_(i) = joint_ptr->limits->upper;
                RCLCPP_INFO(this->get_logger(), "  %s: [%.3f, %.3f] rad ([%.1f°, %.1f°])",
                           joint_name.c_str(),
                           left_q_min_(i), left_q_max_(i),
                           left_q_min_(i) * 180.0 / M_PI, left_q_max_(i) * 180.0 / M_PI);
            } else {
                left_q_min_(i) = -3.14159;
                left_q_max_(i) = 3.14159;
                RCLCPP_WARN(this->get_logger(), "  %s: Using default limits [%.3f, %.3f] rad ([%.1f°, %.1f°])",
                           joint_name.c_str(),
                           left_q_min_(i), left_q_max_(i),
                           left_q_min_(i) * 180.0 / M_PI, left_q_max_(i) * 180.0 / M_PI);
            }
        }

        RCLCPP_INFO(this->get_logger(), "✅ Joint limits configured for both arms");
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!setup_complete_) {
            return;
        }

        // Extract current lift_joint position for coordinate transformation
        lift_joint_position_ = 0.0;
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "lift_joint" && i < msg->position.size()) {
                lift_joint_position_ = msg->position[i];
                break;
            }
        }

        // Extract right arm joint positions
        right_current_joint_positions_.assign(right_joint_names_.size(), 0.0);
        bool right_all_joints_found = true;
        for (size_t i = 0; i < right_joint_names_.size(); i++) {
            bool found = false;
            for (size_t j = 0; j < msg->name.size(); j++) {
                if (msg->name[j] == right_joint_names_[i] && j < msg->position.size()) {
                    right_current_joint_positions_[i] = msg->position[j];
                    found = true;
                    break;
                }
            }
            if (!found) {
                right_all_joints_found = false;
                break;
            }
        }

        // Extract left arm joint positions
        left_current_joint_positions_.assign(left_joint_names_.size(), 0.0);
        bool left_all_joints_found = true;
        for (size_t i = 0; i < left_joint_names_.size(); i++) {
            bool found = false;
            for (size_t j = 0; j < msg->name.size(); j++) {
                if (msg->name[j] == left_joint_names_[i] && j < msg->position.size()) {
                    left_current_joint_positions_[i] = msg->position[j];
                    found = true;
                    break;
                }
            }
            if (!found) {
                left_all_joints_found = false;
                break;
            }
        }

        if (right_all_joints_found && left_all_joints_found && !has_joint_states_) {
            has_joint_states_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ Joint states received. Dual-arm IK system ready!");
            RCLCPP_INFO(this->get_logger(), "   Lift joint position: %.3f m", lift_joint_position_);

            // Check if current joint positions are within limits
            checkCurrentJointLimits();
        }
    }

    void checkCurrentJointLimits()
    {
        RCLCPP_INFO(this->get_logger(), "🔍 Checking current joint positions against limits:");

        // Check right arm joints
        bool right_all_within_limits = true;
        RCLCPP_INFO(this->get_logger(), "Right arm joints:");
        for (size_t i = 0; i < right_current_joint_positions_.size(); i++) {
            double pos = right_current_joint_positions_[i];
            bool within_limits = (pos >= right_q_min_(i) && pos <= right_q_max_(i));
            if (!within_limits) {
                right_all_within_limits = false;
            }
            RCLCPP_INFO(this->get_logger(), "  %s: %.3f rad (%.1f°) %s [%.3f, %.3f]",
                       right_joint_names_[i].c_str(),
                       pos, pos * 180.0 / M_PI,
                       within_limits ? "✅" : "⚠️ OUTSIDE",
                       right_q_min_(i), right_q_max_(i));
        }

        // Check left arm joints
        bool left_all_within_limits = true;
        RCLCPP_INFO(this->get_logger(), "Left arm joints:");
        for (size_t i = 0; i < left_current_joint_positions_.size(); i++) {
            double pos = left_current_joint_positions_[i];
            bool within_limits = (pos >= left_q_min_(i) && pos <= left_q_max_(i));
            if (!within_limits) {
                left_all_within_limits = false;
            }
            RCLCPP_INFO(this->get_logger(), "  %s: %.3f rad (%.1f°) %s [%.3f, %.3f]",
                       left_joint_names_[i].c_str(),
                       pos, pos * 180.0 / M_PI,
                       within_limits ? "✅" : "⚠️ OUTSIDE",
                       left_q_min_(i), left_q_max_(i));
        }

        if (right_all_within_limits && left_all_within_limits) {
            RCLCPP_INFO(this->get_logger(), "✅ All joints are within limits");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Some joints are outside their limits!");
        }
    }

    void rightTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!setup_complete_ || !has_joint_states_) {
            RCLCPP_WARN(this->get_logger(), "🚫 Right arm IK solver not ready yet, ignoring target pose");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🎯 Received RIGHT arm target pose (base_link frame):");
        RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "   Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                   msg->pose.orientation.x, msg->pose.orientation.y,
                   msg->pose.orientation.z, msg->pose.orientation.w);

        // Transform pose from base_link to arm_base_link frame
        geometry_msgs::msg::PoseStamped arm_base_pose = *msg;

        // URDF lift_joint origin: xyz="0.0055 0 1.4316"  
        // Transform: base_link -> arm_base_link
        arm_base_pose.pose.position.x -= 0.0055;  // lift_joint x offset
        arm_base_pose.pose.position.y -= 0.0;     // lift_joint y offset
        arm_base_pose.pose.position.z -= (1.4316 + lift_joint_position_); // lift_joint z offset + current lift position

        RCLCPP_INFO(this->get_logger(), "🔄 Transformed to arm_base_link frame (RIGHT):");
        RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f (lift: %.3f m)",
                   arm_base_pose.pose.position.x, arm_base_pose.pose.position.y,
                   arm_base_pose.pose.position.z, lift_joint_position_);

        // Calculate distance for reachability check
        double distance = sqrt(arm_base_pose.pose.position.x * arm_base_pose.pose.position.x +
                              arm_base_pose.pose.position.y * arm_base_pose.pose.position.y +
                              arm_base_pose.pose.position.z * arm_base_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "📐 RIGHT arm target distance from arm_base: %.3f m", distance);

        // Solve IK for the transformed target
        solveIKAndMove(arm_base_pose, "right");
    }

    void leftTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!setup_complete_ || !has_joint_states_) {
            RCLCPP_WARN(this->get_logger(), "🚫 Left arm IK solver not ready yet, ignoring target pose");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🎯 Received LEFT arm target pose (base_link frame):");
        RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "   Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                   msg->pose.orientation.x, msg->pose.orientation.y,
                   msg->pose.orientation.z, msg->pose.orientation.w);

        // Transform pose from base_link to arm_base_link frame
        geometry_msgs::msg::PoseStamped arm_base_pose = *msg;

        // URDF lift_joint origin: xyz="0.0055 0 1.4316"  
        // Transform: base_link -> arm_base_link
        arm_base_pose.pose.position.x -= 0.0055;  // lift_joint x offset
        arm_base_pose.pose.position.y -= 0.0;     // lift_joint y offset
        arm_base_pose.pose.position.z -= (1.4316 + lift_joint_position_); // lift_joint z offset + current lift position

        RCLCPP_INFO(this->get_logger(), "🔄 Transformed to arm_base_link frame (LEFT):");
        RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f (lift: %.3f m)",
                   arm_base_pose.pose.position.x, arm_base_pose.pose.position.y,
                   arm_base_pose.pose.position.z, lift_joint_position_);

        // Calculate distance for reachability check
        double distance = sqrt(arm_base_pose.pose.position.x * arm_base_pose.pose.position.x +
                              arm_base_pose.pose.position.y * arm_base_pose.pose.position.y +
                              arm_base_pose.pose.position.z * arm_base_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "📐 LEFT arm target distance from arm_base: %.3f m", distance);

        // Solve IK for the transformed target
        solveIKAndMove(arm_base_pose, "left");
    }

    void leftPinchCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        left_pinch_value_ = msg->data;
        // Calculate corresponding gripper position for logging
        double gripper_position = 1.2 - (left_pinch_value_ * 1.2 / 0.1);
        gripper_position = std::max(0.0, std::min(1.2, gripper_position));
        RCLCPP_INFO(this->get_logger(), "🤏 Left pinch: %.3f → gripper: %.3f", left_pinch_value_, gripper_position);
    }

    void rightPinchCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        right_pinch_value_ = msg->data;
        // Calculate corresponding gripper position for logging
        double gripper_position = 1.2 - (right_pinch_value_ * 1.2 / 0.1);
        gripper_position = std::max(0.0, std::min(1.2, gripper_position));
        RCLCPP_INFO(this->get_logger(), "🤏 Right pinch: %.3f → gripper: %.3f", right_pinch_value_, gripper_position);
    }

    void solveIKAndMove(const geometry_msgs::msg::PoseStamped& target_pose, const std::string& arm)
    {
        RCLCPP_INFO(this->get_logger(), "🔧 Solving %s arm IK with Joint Limits...", arm.c_str());

        // Convert target pose to KDL Frame
        KDL::Frame target_frame;
        target_frame.p.x(target_pose.pose.position.x);
        target_frame.p.y(target_pose.pose.position.y);
        target_frame.p.z(target_pose.pose.position.z);

        // Convert quaternion to rotation matrix
        KDL::Rotation rot = KDL::Rotation::Quaternion(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        );
        target_frame.M = rot;

        // Select arm-specific variables
        KDL::Chain* chain_ptr;
        std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>* ik_solver_ptr;
        std::vector<std::string>* joint_names_ptr;
        std::vector<double>* current_positions_ptr;
        KDL::JntArray* q_min_ptr;
        KDL::JntArray* q_max_ptr;

        if (arm == "right") {
            chain_ptr = &right_chain_;
            ik_solver_ptr = &right_ik_solver_jl_;
            joint_names_ptr = &right_joint_names_;
            current_positions_ptr = &right_current_joint_positions_;
            q_min_ptr = &right_q_min_;
            q_max_ptr = &right_q_max_;
        } else {
            chain_ptr = &left_chain_;
            ik_solver_ptr = &left_ik_solver_jl_;
            joint_names_ptr = &left_joint_names_;
            current_positions_ptr = &left_current_joint_positions_;
            q_min_ptr = &left_q_min_;
            q_max_ptr = &left_q_max_;
        }

        // Get current arm joint positions as initial guess
        KDL::JntArray q_init(chain_ptr->getNrOfJoints());
        for (size_t i = 0; i < current_positions_ptr->size(); i++) {
            q_init(i) = (*current_positions_ptr)[i];
        }

        // Log current joint positions
        std::string current_log = "🔧 Current " + arm + " arm joints: [";
        for (size_t i = 0; i < current_positions_ptr->size(); i++) {
            current_log += (*joint_names_ptr)[i] + "=" + std::to_string((*current_positions_ptr)[i]);
            if (i < current_positions_ptr->size() - 1) current_log += ", ";
        }
        current_log += "]";
        RCLCPP_INFO(this->get_logger(), "%s", current_log.c_str());

        // Clamp initial guess to joint limits
        for (unsigned int i = 0; i < q_init.rows(); i++) {
            if (q_init(i) < (*q_min_ptr)(i)) {
                q_init(i) = (*q_min_ptr)(i);
                RCLCPP_DEBUG(this->get_logger(), "Clamped %s arm initial guess for joint %d to min limit", arm.c_str(), i);
            }
            if (q_init(i) > (*q_max_ptr)(i)) {
                q_init(i) = (*q_max_ptr)(i);
                RCLCPP_DEBUG(this->get_logger(), "Clamped %s arm initial guess for joint %d to max limit", arm.c_str(), i);
            }
        }

        KDL::JntArray q_result(chain_ptr->getNrOfJoints());
        int ik_result = (*ik_solver_ptr)->CartToJnt(q_init, target_frame, q_result);

        // If IK failed, try with different initial guesses within limits
        if (ik_result < 0) {
            RCLCPP_WARN(this->get_logger(), "%s arm IK failed with current guess (error: %d), trying alternatives...", arm.c_str(), ik_result);

            // Try with center of joint limits as initial guess
            KDL::JntArray q_center(chain_ptr->getNrOfJoints());
            for (unsigned int i = 0; i < chain_ptr->getNrOfJoints(); i++) {
                q_center(i) = ((*q_min_ptr)(i) + (*q_max_ptr)(i)) / 2.0;
            }
            ik_result = (*ik_solver_ptr)->CartToJnt(q_center, target_frame, q_result);

            if (ik_result < 0) {
                // Try with home position (zeros)
                KDL::JntArray q_home(chain_ptr->getNrOfJoints());
                for (unsigned int i = 0; i < q_home.rows(); i++) {
                    q_home(i) = 0.0;
                }
                ik_result = (*ik_solver_ptr)->CartToJnt(q_home, target_frame, q_result);
            }
        }

        if (ik_result >= 0) {
            // Verify all joints are within limits
            bool all_within_limits = true;
            for (unsigned int i = 0; i < q_result.rows(); i++) {
                if (q_result(i) < (*q_min_ptr)(i) || q_result(i) > (*q_max_ptr)(i)) {
                    all_within_limits = false;
                    RCLCPP_WARN(this->get_logger(), "%s arm joint %d solution %.3f is outside limits [%.3f, %.3f]",
                               arm.c_str(), i, q_result(i), (*q_min_ptr)(i), (*q_max_ptr)(i));
                }
            }

            if (!all_within_limits) {
                RCLCPP_ERROR(this->get_logger(), "❌ %s arm IK solution violates joint limits! Skipping movement.", arm.c_str());
                return;
            }

            // Clamp joint movement to max step (e.g., 20 degrees per cycle)
            const double max_joint_step = 20.0 * M_PI / 180.0; // 20 degrees in radians
            bool clamped = false;
            for (unsigned int i = 0; i < q_result.rows(); i++) {
                double delta = q_result(i) - (*current_positions_ptr)[i];
                if (std::abs(delta) > max_joint_step) {
                    clamped = true;
                    if (delta > 0)
                        q_result(i) = (*current_positions_ptr)[i] + max_joint_step;
                    else
                        q_result(i) = (*current_positions_ptr)[i] - max_joint_step;
                }
            }
            if (clamped) {
                RCLCPP_WARN(this->get_logger(), "⚠️ %s arm joint movement clamped to max %.1f deg per cycle for safety.", arm.c_str(), max_joint_step * 180.0 / M_PI);
            }

            RCLCPP_INFO(this->get_logger(), "✅ %s arm IK solution found with Joint Limits. Moving robot...", arm.c_str());

            // Log joint solution
            std::string solution_log = "🎯 " + arm + " arm joint solution: [";
            for (size_t i = 0; i < joint_names_ptr->size(); i++) {
                solution_log += (*joint_names_ptr)[i] + "=" + std::to_string(q_result(i));
                if (i < joint_names_ptr->size() - 1) solution_log += ", ";
            }
            solution_log += "]";
            RCLCPP_INFO(this->get_logger(), "%s", solution_log.c_str());

            // Send joint trajectory command to move the robot
            sendJointTrajectory(q_result, arm);

        } else {
            // Provide detailed error information
            std::string error_msg;
            switch(ik_result) {
                case -1: error_msg = "Failed to converge"; break;
                case -2: error_msg = "Undefined problem"; break;
                case -3: error_msg = "Degraded gradient"; break;
                case -4: error_msg = "Singularity detected"; break;
                case -5: error_msg = "Maximum iterations exceeded"; break;
                default: error_msg = "Unknown error"; break;
            }

            RCLCPP_ERROR(this->get_logger(), "❌ %s arm IK with Joint Limits failed: %d (%s)", arm.c_str(), ik_result, error_msg.c_str());

            // Additional failure analysis
            if (ik_result == -5) {
                RCLCPP_ERROR(this->get_logger(), "  → Target may be unreachable for %s arm configuration", arm.c_str());
            } else if (ik_result == -3 || ik_result == -4) {
                RCLCPP_ERROR(this->get_logger(), "  → %s arm is in or near a singularity", arm.c_str());
            }
        }
    }

    void sendJointTrajectory(const KDL::JntArray& joint_positions, const std::string& arm)
    {
        // Create joint trajectory message for specified arm joints
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.header.frame_id = "";

        // Select arm-specific variables
        std::vector<std::string>* joint_names_ptr;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr* publisher_ptr;

        if (arm == "right") {
            joint_names_ptr = &right_joint_names_;
            publisher_ptr = &right_joint_trajectory_pub_;
        } else {
            joint_names_ptr = &left_joint_names_;
            publisher_ptr = &left_joint_trajectory_pub_;
        }

        // Set arm joint names (including gripper)
        std::vector<std::string> arm_joint_names = *joint_names_ptr;
        std::vector<double> target_arm_positions;

        // Add arm joint positions
        for (size_t i = 0; i < joint_names_ptr->size(); i++) {
            target_arm_positions.push_back(joint_positions(i));
        }

        // Add gripper joint and position
        if (arm == "right") {
            arm_joint_names.push_back("gripper_r_joint1");
            // Map VR pinch value (0=close, 0.1=open) to gripper position (1.2=close, 0=open)
            // Invert the mapping: closed pinch (0) -> closed gripper (1.2), open pinch (0.1) -> open gripper (0)
            double gripper_position = 1.2 - (right_pinch_value_ * 1.2 / 0.1);
            // Clamp to valid range [0, 1.2]
            gripper_position = std::max(0.0, std::min(1.2, gripper_position));
            target_arm_positions.push_back(gripper_position);
        } else {
            arm_joint_names.push_back("gripper_l_joint1");
            // Map VR pinch value (0=close, 0.1=open) to gripper position (1.2=close, 0=open)
            // Invert the mapping: closed pinch (0) -> closed gripper (1.2), open pinch (0.1) -> open gripper (0)
            double gripper_position = 1.2 - (left_pinch_value_ * 1.2 / 0.1);
            // Clamp to valid range [0, 1.2]
            gripper_position = std::max(0.0, std::min(1.2, gripper_position));
            target_arm_positions.push_back(gripper_position);
        }

        traj_msg.joint_names = arm_joint_names;

        // Create single trajectory point with smooth timing
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = target_arm_positions;
        point.velocities = {};  // Empty velocities array
        point.accelerations = {};  // Empty accelerations array
        point.effort = {};  // Empty effort array
        point.time_from_start.sec = 0;    // Immediate execution
        point.time_from_start.nanosec = 0;

        traj_msg.points.push_back(point);

        // Publish the trajectory
        (*publisher_ptr)->publish(traj_msg);

        // Log sent trajectory
        std::string sent_log = "📤 Sent " + arm + " arm trajectory: [";
        for (size_t i = 0; i < arm_joint_names.size(); i++) {
            sent_log += arm_joint_names[i] + "=" + std::to_string(target_arm_positions[i]);
            if (i < arm_joint_names.size() - 1) sent_log += ", ";
        }
        sent_log += "]";
        RCLCPP_INFO(this->get_logger(), "%s", sent_log.c_str());
        RCLCPP_INFO(this->get_logger(), "📤 %s arm joint trajectory sent! Robot should move immediately.", arm.c_str());
    }

    void publishCurrentPoses()
    {
        if (!setup_complete_ || !has_joint_states_) {
            return;
        }

        // Publish right arm current pose
        KDL::JntArray right_q(right_chain_.getNrOfJoints());
        for (size_t i = 0; i < right_current_joint_positions_.size(); i++) {
            right_q(i) = right_current_joint_positions_[i];
        }

        KDL::Frame right_end_effector_frame;
        int right_fk_result = right_fk_solver_->JntToCart(right_q, right_end_effector_frame);

        if (right_fk_result >= 0) {
            // Extract position and orientation (FK result is in arm_base_link frame)
            KDL::Vector right_pos = right_end_effector_frame.p;
            double right_qx, right_qy, right_qz, right_qw;
            right_end_effector_frame.M.GetQuaternion(right_qx, right_qy, right_qz, right_qw);

            // Transform from arm_base_link to base_link frame
            double right_base_x = right_pos.x() + 0.0055;
            double right_base_y = right_pos.y() + 0.0;
            double right_base_z = right_pos.z() + (1.4316 + lift_joint_position_);

            // Publish right arm current pose (in base_link frame)
            auto right_pose_msg = geometry_msgs::msg::PoseStamped();
            right_pose_msg.header.stamp = this->get_clock()->now();
            right_pose_msg.header.frame_id = base_link_;
            right_pose_msg.pose.position.x = right_base_x;
            right_pose_msg.pose.position.y = right_base_y;
            right_pose_msg.pose.position.z = right_base_z;
            right_pose_msg.pose.orientation.x = right_qx;
            right_pose_msg.pose.orientation.y = right_qy;
            right_pose_msg.pose.orientation.z = right_qz;
            right_pose_msg.pose.orientation.w = right_qw;
            right_current_pose_pub_->publish(right_pose_msg);
        }

        // Publish left arm current pose
        KDL::JntArray left_q(left_chain_.getNrOfJoints());
        for (size_t i = 0; i < left_current_joint_positions_.size(); i++) {
            left_q(i) = left_current_joint_positions_[i];
        }

        KDL::Frame left_end_effector_frame;
        int left_fk_result = left_fk_solver_->JntToCart(left_q, left_end_effector_frame);

        if (left_fk_result >= 0) {
            // Extract position and orientation (FK result is in arm_base_link frame)
            KDL::Vector left_pos = left_end_effector_frame.p;
            double left_qx, left_qy, left_qz, left_qw;
            left_end_effector_frame.M.GetQuaternion(left_qx, left_qy, left_qz, left_qw);

            // Transform from arm_base_link to base_link frame
            double left_base_x = left_pos.x() + 0.0055;
            double left_base_y = left_pos.y() + 0.0;
            double left_base_z = left_pos.z() + (1.4316 + lift_joint_position_);

            // Publish left arm current pose (in base_link frame)
            auto left_pose_msg = geometry_msgs::msg::PoseStamped();
            left_pose_msg.header.stamp = this->get_clock()->now();
            left_pose_msg.header.frame_id = base_link_;
            left_pose_msg.pose.position.x = left_base_x;
            left_pose_msg.pose.position.y = left_base_y;
            left_pose_msg.pose.position.z = left_base_z;
            left_pose_msg.pose.orientation.x = left_qx;
            left_pose_msg.pose.orientation.y = left_qy;
            left_pose_msg.pose.orientation.z = left_qz;
            left_pose_msg.pose.orientation.w = left_qw;
            left_current_pose_pub_->publish(left_pose_msg);
        }
    }

private:
    // Parameters
    std::string base_link_;
    std::string arm_base_link_;
    std::string right_end_effector_link_;
    std::string left_end_effector_link_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_current_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_current_pose_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_joint_trajectory_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_joint_trajectory_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_pinch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_pinch_sub_;
    rclcpp::TimerBase::SharedPtr pose_timer_;

    // KDL objects for right arm
    KDL::Chain right_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> right_fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> right_ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> right_ik_solver_jl_;

    // KDL objects for left arm
    KDL::Chain left_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> left_fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> left_ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> left_ik_solver_jl_;

    // Joint limits for right arm
    KDL::JntArray right_q_min_;
    KDL::JntArray right_q_max_;

    // Joint limits for left arm
    KDL::JntArray left_q_min_;
    KDL::JntArray left_q_max_;

    // Joint information for right arm
    std::vector<std::string> right_joint_names_;
    std::vector<double> right_current_joint_positions_;

    // Joint information for left arm
    std::vector<std::string> left_joint_names_;
    std::vector<double> left_current_joint_positions_;

    // Common joint information
    int lift_joint_index_;  // Not used in arm-only chain
    double lift_joint_position_;  // Current lift joint position for coordinate transformation

    // VR pinch values for gripper control
    double left_pinch_value_;
    double right_pinch_value_;

    // Status flags
    bool setup_complete_;
    bool has_joint_states_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RealtimeIKSolverJL>();

    RCLCPP_INFO(node->get_logger(), "🚀 Dual-Arm Realtime IK Solver with Joint Limits node started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
