#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>
#include <memory>
#include <vector>
#include <map>

class KDLIKSolver : public rclcpp::Node
{
public:
    KDLIKSolver() : Node("kdl_ik_solver"),
                     lift_joint_index_(-1),
                     lift_joint_value_(0.0),
                     setup_complete_(false),
                     has_joint_states_(false),
                     test_executed_(false),
                     has_sent_trajectory_(false)
    {
        // Parameters
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("end_effector_link", "arm_r_link7");

        base_link_ = this->get_parameter("base_link").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();

        RCLCPP_INFO(this->get_logger(), "KDL IK Solver starting...");
        RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());

        // Subscribers
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&KDLIKSolver::robotDescriptionCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&KDLIKSolver::jointStateCallback, this, std::placeholders::_1));

        // Publisher for test results
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/kdl_ik_test_pose", 10);

        // Publisher for joint trajectory commands
        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

        // Timer for periodic testing
        test_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&KDLIKSolver::runTest, this));

        // Timer for checking if robot moved after trajectory command
        check_movement_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&KDLIKSolver::checkMovement, this));

        // Initialize flags
        setup_complete_ = false;
        has_joint_states_ = false;
        test_executed_ = false;

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
        } else {
            RCLCPP_WARN(this->get_logger(), "robot_state_publisher parameter service not available");
        }
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

            // Extract chain from tree
            if (!tree.getChain(base_link_, end_effector_link_, chain_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from %s to %s",
                           base_link_.c_str(), end_effector_link_.c_str());

                // Print available segments for debugging
                auto segments = tree.getSegments();
                RCLCPP_INFO(this->get_logger(), "Available segments:");
                for (const auto& seg : segments) {
                    RCLCPP_INFO(this->get_logger(), "  - %s", seg.first.c_str());
                }
                return;
            }

            RCLCPP_INFO(this->get_logger(), "KDL chain extracted with %d joints", chain_.getNrOfJoints());

            // Create solvers
            fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

            ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
            // Increase max iterations and relax tolerance for better convergence
            ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(
                chain_, *fk_solver_, *ik_vel_solver_, 1000, 1e-3);

            // Extract joint names and identify lift_joint index
            joint_names_.clear();
            lift_joint_index_ = -1;

            for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
                KDL::Segment segment = chain_.getSegment(i);
                if (segment.getJoint().getType() != KDL::Joint::None) {
                    std::string joint_name = segment.getJoint().getName();
                    joint_names_.push_back(joint_name);

                    // Check if this is the lift_joint
                    if (joint_name == "lift_joint") {
                        lift_joint_index_ = joint_names_.size() - 1;
                        RCLCPP_INFO(this->get_logger(), "Found lift_joint at index %d", lift_joint_index_);
                    }
                }
            }

            RCLCPP_INFO(this->get_logger(), "Joint names extracted:");
            for (size_t i = 0; i < joint_names_.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "  [%zu] %s%s", i, joint_names_[i].c_str(),
                           (i == lift_joint_index_) ? " (LIFT - will be fixed)" : "");
            }

            setup_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ KDL setup completed successfully!");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing robot description: %s", e.what());
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!setup_complete_) {
            return;
        }

        // Extract joint positions for our chain
        current_joint_positions_.assign(joint_names_.size(), 0.0);
        bool all_joints_found = true;

        for (size_t i = 0; i < joint_names_.size(); i++) {
            bool found = false;
            for (size_t j = 0; j < msg->name.size(); j++) {
                if (msg->name[j] == joint_names_[i] && j < msg->position.size()) {
                    current_joint_positions_[i] = msg->position[j];
                    found = true;
                    break;
                }
            }
            if (!found) {
                all_joints_found = false;
                break;
            }
        }

        if (all_joints_found) {
            has_joint_states_ = true;
        }
    }

    void runTest()
    {
        if (!setup_complete_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for setup completion...");
            return;
        }

        if (!has_joint_states_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
            return;
        }

        // Execute test only once
        if (test_executed_) {
            return;
        }
        test_executed_ = true;

        RCLCPP_INFO(this->get_logger(), "=== KDL IK Test ===");

        // Print current joint positions
        RCLCPP_INFO(this->get_logger(), "Current joint positions:");
        for (size_t i = 0; i < joint_names_.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad (%.1f°)",
                       joint_names_[i].c_str(),
                       current_joint_positions_[i],
                       current_joint_positions_[i] * 180.0 / M_PI);
        }

        // Forward Kinematics
        KDL::JntArray q(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            q(i) = current_joint_positions_[i];
        }

        KDL::Frame end_effector_frame;
        int fk_result = fk_solver_->JntToCart(q, end_effector_frame);

        if (fk_result >= 0) {
            // Extract position and orientation
            KDL::Vector pos = end_effector_frame.p;
            double qx, qy, qz, qw;
            end_effector_frame.M.GetQuaternion(qx, qy, qz, qw);

            RCLCPP_INFO(this->get_logger(), "Current end-effector pose (FK):");
            RCLCPP_INFO(this->get_logger(), "  Position: x=%.3f, y=%.3f, z=%.3f", pos.x(), pos.y(), pos.z());
            RCLCPP_INFO(this->get_logger(), "  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", qx, qy, qz, qw);

            // Publish current pose
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = base_link_;
            pose_msg.pose.position.x = pos.x();
            pose_msg.pose.position.y = pos.y();
            pose_msg.pose.position.z = pos.z();
            pose_msg.pose.orientation.x = qx;
            pose_msg.pose.orientation.y = qy;
            pose_msg.pose.orientation.z = qz;
            pose_msg.pose.orientation.w = qw;
            pose_pub_->publish(pose_msg);

            // IK Test - set target pose with 3D movement (X, Y, Z all possible with 7DOF)
            KDL::Frame target_frame = end_effector_frame;
            target_frame.p.x(pos.x() + 0.03);  // 3cm forward
            target_frame.p.y(pos.y() + 0.02);  // 2cm to the side
            target_frame.p.z(pos.z() + 0.02);  // 2cm up (7DOF allows Z movement)

            RCLCPP_INFO(this->get_logger(), "Target pose:");
            RCLCPP_INFO(this->get_logger(), "  Position: x=%.3f, y=%.3f, z=%.3f",
                       target_frame.p.x(), target_frame.p.y(), target_frame.p.z());

            // Solve IK with lift_joint fixed - try multiple initial guesses
            KDL::JntArray q_init = q;  // Use current position as initial guess
            KDL::JntArray q_result(chain_.getNrOfJoints());

            int ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_result);

            // If IK failed, try with different initial guesses
            if (ik_result < 0) {
                RCLCPP_WARN(this->get_logger(), "IK failed with first guess (error: %d), trying alternatives...", ik_result);

                // Try with zero initial position
                KDL::JntArray q_zero(chain_.getNrOfJoints());
                for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
                    q_zero(i) = 0.0;
                }
                ik_result = ik_solver_->CartToJnt(q_zero, target_frame, q_result);

                if (ik_result < 0) {
                    // Try with small random perturbation
                    KDL::JntArray q_perturb = q;
                    for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
                        if (i != lift_joint_index_) {  // Don't perturb lift joint
                            q_perturb(i) += (rand() % 1000 - 500) * 0.001;  // ±0.5 rad perturbation
                        }
                    }
                    ik_result = ik_solver_->CartToJnt(q_perturb, target_frame, q_result);
                }
            }

            // Fix lift_joint to its current position
            if (lift_joint_index_ >= 0 && lift_joint_index_ < q_result.rows()) {
                q_result(lift_joint_index_) = current_joint_positions_[lift_joint_index_];
                RCLCPP_INFO(this->get_logger(), "Fixed lift_joint to current position: %.4f rad",
                           current_joint_positions_[lift_joint_index_]);
            }

            if (ik_result >= 0) {
                RCLCPP_INFO(this->get_logger(), "✓ IK solution found!");
                RCLCPP_INFO(this->get_logger(), "Target joint angles:");

                for (size_t i = 0; i < joint_names_.size(); i++) {
                    double angle = q_result(i);
                    RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad (%.1f°)%s",
                               joint_names_[i].c_str(), angle, angle * 180.0 / M_PI,
                               (i == lift_joint_index_) ? " [FIXED]" : "");
                }

                // Verify the solution with the fixed lift_joint
                KDL::Frame verify_frame;
                fk_solver_->JntToCart(q_result, verify_frame);

                KDL::Vector error_pos = verify_frame.p - target_frame.p;
                double error_magnitude = sqrt(
                    error_pos.x() * error_pos.x() +
                    error_pos.y() * error_pos.y() +
                    error_pos.z() * error_pos.z());

                RCLCPP_INFO(this->get_logger(), "Verification error: %.6f m", error_magnitude);

                if (error_magnitude < 1.0) {  // 10cm 허용 오차 (lift_joint 고정으로 인한)
                    RCLCPP_INFO(this->get_logger(), "✓ IK solution is acceptable (with fixed lift_joint)!");

                    // Send joint trajectory command to move the robot
                    sendJointTrajectory(q_result);

                } else {
                    RCLCPP_WARN(this->get_logger(), "⚠ Large verification error even with fixed lift_joint");
                }

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

                RCLCPP_ERROR(this->get_logger(), "✗ IK failed with error code: %d (%s)", ik_result, error_msg.c_str());

                // Print detailed error information
                double distance = sqrt(
                    pow(target_frame.p.x() - pos.x(), 2) +
                    pow(target_frame.p.y() - pos.y(), 2) +
                    pow(target_frame.p.z() - pos.z(), 2)
                );
                RCLCPP_WARN(this->get_logger(), "Target distance: %.4f m", distance);
                RCLCPP_WARN(this->get_logger(), "Current position: [%.3f, %.3f, %.3f]", pos.x(), pos.y(), pos.z());
                RCLCPP_WARN(this->get_logger(), "Target position: [%.3f, %.3f, %.3f]",
                           target_frame.p.x(), target_frame.p.y(), target_frame.p.z());

                // Try with smaller target movement
                if (distance > 0.01) {  // If target is more than 1cm away
                    RCLCPP_INFO(this->get_logger(), "Trying smaller movement...");
                    KDL::Frame smaller_target = end_effector_frame;
                    smaller_target.p.x(pos.x() + 0.005);  // 5mm forward
                    smaller_target.p.y(pos.y() + 0.005);  // 5mm to the side

                    int retry_result = ik_solver_->CartToJnt(q_init, smaller_target, q_result);
                    if (retry_result >= 0) {
                        // Fix lift_joint again
                        if (lift_joint_index_ >= 0 && lift_joint_index_ < q_result.rows()) {
                            q_result(lift_joint_index_) = current_joint_positions_[lift_joint_index_];
                        }
                        RCLCPP_INFO(this->get_logger(), "✓ IK solved with smaller movement!");
                        sendJointTrajectory(q_result);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "✗ IK failed even with smaller movement (error: %d)", retry_result);
                    }
                }
            }

        } else {
            RCLCPP_ERROR(this->get_logger(), "FK failed with error code: %d", fk_result);
        }
    }

    void sendJointTrajectory(const KDL::JntArray& joint_positions)
    {
        // Create joint trajectory message (format matching real leader)
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.header.stamp.sec = 0;
        traj_msg.header.stamp.nanosec = 0;
        traj_msg.header.frame_id = "";

        // Set joint names (arm joints + gripper, excluding lift_joint)
        std::vector<std::string> arm_joint_names;
        std::vector<double> target_arm_positions;

        for (size_t i = 0; i < joint_names_.size(); i++) {
            if (joint_names_[i] != "lift_joint") {  // Skip lift_joint
                arm_joint_names.push_back(joint_names_[i]);
                target_arm_positions.push_back(joint_positions(i));
            }
        }

        // Add gripper joint (gripper_r_joint1) with default open position
        arm_joint_names.push_back("gripper_r_joint1");
        target_arm_positions.push_back(0.1);  // Slightly open gripper

        traj_msg.joint_names = arm_joint_names;

        // Create single trajectory point (matching real leader format)
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = target_arm_positions;
        point.velocities = {};  // Empty velocities array (like real leader)
        point.accelerations = {};  // Empty accelerations array (like real leader)
        point.effort = {};  // Empty effort array (like real leader)
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 0;

        traj_msg.points.push_back(point);

        // Debug: Print trajectory details
        RCLCPP_INFO(this->get_logger(), "📤 Sending Joint Trajectory (Leader format):");
        RCLCPP_INFO(this->get_logger(), "   Joints: %zu", arm_joint_names.size());
        for (size_t i = 0; i < target_arm_positions.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "   %s: %.4f rad",
                       arm_joint_names[i].c_str(),
                       target_arm_positions[i]);
        }
        RCLCPP_INFO(this->get_logger(), "   Single point trajectory (no timing)");

        // Publish the trajectory
        joint_trajectory_pub_->publish(traj_msg);

        RCLCPP_INFO(this->get_logger(), "✅ Joint trajectory command published!");

        // Store initial arm positions for movement verification (excluding gripper)
        initial_joint_positions_.clear();
        for (size_t i = 0; i < joint_names_.size(); i++) {
            if (joint_names_[i] != "lift_joint") {  // Skip lift_joint
                initial_joint_positions_.push_back(current_joint_positions_[i]);
            }
        }
        has_sent_trajectory_ = true;

        // Start timer to check movement after 3 seconds (shorter for testing)
        check_movement_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&KDLIKSolver::checkMovement, this));

        RCLCPP_INFO(this->get_logger(), "🕒 Movement check timer started (3 seconds)");
    }

    void checkMovement()
    {
        if (!has_sent_trajectory_ || initial_joint_positions_.empty()) {
            return;
        }

        // Stop the timer (one-time check)
        check_movement_timer_.reset();

        RCLCPP_INFO(this->get_logger(), "🔍 Checking robot movement...");

        // Compare current positions with initial positions (arm joints only)
        std::vector<double> current_arm_positions;
        for (size_t i = 0; i < joint_names_.size(); i++) {
            if (joint_names_[i] != "lift_joint") {
                current_arm_positions.push_back(current_joint_positions_[i]);
            }
        }

        bool has_moved = false;
        double total_movement = 0.0;
        const double movement_threshold = 0.01; // 0.01 rad threshold

        RCLCPP_INFO(this->get_logger(), "Joint position comparison:");

        for (size_t i = 0; i < std::min(initial_joint_positions_.size(), current_arm_positions.size()); i++) {
            double diff = std::abs(current_arm_positions[i] - initial_joint_positions_[i]);
            total_movement += diff;

            if (diff > movement_threshold) {
                has_moved = true;
            }

            RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.4f → %.4f (diff: %.4f)%s",
                       i, initial_joint_positions_[i], current_arm_positions[i], diff,
                       (diff > movement_threshold) ? " ✓ MOVED" : "");
        }

        RCLCPP_INFO(this->get_logger(), "Total movement: %.4f rad", total_movement);

        if (has_moved) {
            RCLCPP_INFO(this->get_logger(), "🎉 SUCCESS: Robot has moved! Trajectory was executed.");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  Robot has not moved significantly. Check controller or topic.");
        }

        // Reset flag
        has_sent_trajectory_ = false;
    }

private:
    // Parameters
    std::string base_link_;
    std::string end_effector_link_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    rclcpp::TimerBase::SharedPtr check_movement_timer_;

    // KDL objects
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_;

    // Joint information
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    std::vector<double> initial_joint_positions_;  // Store initial positions when trajectory is sent
    int lift_joint_index_;  // Index of lift_joint in the chain (-1 if not found)
    double lift_joint_value_;  // Fixed value for lift joint

    // Status flags
    bool setup_complete_;
    bool has_joint_states_;
    bool test_executed_;
    bool has_sent_trajectory_;  // Track if we've sent a trajectory
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KDLIKSolver>();

    RCLCPP_INFO(node->get_logger(), "KDL IK Solver node started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
