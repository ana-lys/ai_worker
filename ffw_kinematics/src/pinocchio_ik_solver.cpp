#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Dense>
#include <chrono>

class PinocchioIKSolver : public rclcpp::Node
{
public:
    PinocchioIKSolver() : Node("pinocchio_ik_solver")
    {
        // Parameters
        this->declare_parameter("use_robot_description", true);
        this->declare_parameter("urdf_path", "/root/ros2_ws/src/ai_worker/ffw_description/urdf/ffw_bg2_rev4_follower/ffw_bg2_follower.urdf");
        this->declare_parameter("base_link", "base_link");
        this->declare_parameter("end_effector_link", "arm_r_link7");
        this->declare_parameter("max_iterations", 1000);
        this->declare_parameter("tolerance", 0.3);  // Relaxed tolerance
        this->declare_parameter("step_size", 0.01);

        use_robot_description_ = this->get_parameter("use_robot_description").as_bool();
        urdf_path_ = this->get_parameter("urdf_path").as_string();
        base_link_ = this->get_parameter("base_link").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        tolerance_ = this->get_parameter("tolerance").as_double();
        step_size_ = this->get_parameter("step_size").as_double();

        // Initialize Pinocchio model
        if (!initializePinocchioModel()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Pinocchio model");
            return;
        }

        // Publishers and subscribers
        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10,
            std::bind(&PinocchioIKSolver::targetPoseCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&PinocchioIKSolver::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Pinocchio IK Solver initialized");
        RCLCPP_INFO(this->get_logger(), "Model has %ld DOFs", model_.nv);
        RCLCPP_INFO(this->get_logger(), "End effector frame ID: %ld", ee_frame_id_);
    }

private:
    bool initializePinocchioModel()
    {
        try {
            if (use_robot_description_) {
                // Try to get robot_description parameter
                if (!this->has_parameter("robot_description")) {
                    RCLCPP_WARN(this->get_logger(), "robot_description parameter not found, trying to declare it...");
                    this->declare_parameter("robot_description", "");
                }

                std::string robot_description;
                if (this->get_parameter("robot_description", robot_description) && !robot_description.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Loading URDF from robot_description parameter");
                    // Load URDF from string
                    pinocchio::urdf::buildModelFromXML(robot_description, model_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "robot_description parameter is empty, falling back to file path");
                    // Fallback to file path
                    pinocchio::urdf::buildModel(urdf_path_, model_);
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Loading URDF from file: %s", urdf_path_.c_str());
                // Load URDF from file
                pinocchio::urdf::buildModel(urdf_path_, model_);
            }
            
            data_ = pinocchio::Data(model_);

            RCLCPP_INFO(this->get_logger(), "Loaded URDF model with %ld joints", model_.njoints);

            // Find end effector frame
            if (model_.existFrame(end_effector_link_)) {
                ee_frame_id_ = model_.getFrameId(end_effector_link_);
                RCLCPP_INFO(this->get_logger(), "Found end effector frame: %s (ID: %ld)",
                           end_effector_link_.c_str(), ee_frame_id_);
            } else {
                RCLCPP_ERROR(this->get_logger(), "End effector frame '%s' not found in model",
                            end_effector_link_.c_str());
                return false;
            }

            // Print joint names for debugging
            for (size_t i = 1; i < model_.names.size(); ++i) {  // Skip universe joint (index 0)
                RCLCPP_INFO(this->get_logger(), "Joint %ld: %s", i, model_.names[i].c_str());
            }

            // Initialize joint configuration
            q_ = pinocchio::neutral(model_);

            // Define joints to exclude from IK (only use right arm joints for IK)
            excluded_joints_ = {
                "lift_joint", 
                "wheel_fl_joint", "wheel_fr_joint", "wheel_bl_joint", "wheel_br_joint",
                "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4", "arm_l_joint5", "arm_l_joint6", "arm_l_joint7",
                "gripper_l_joint1", "gripper_l_joint2", "gripper_l_joint3", "gripper_l_joint4",
                "gripper_r_joint1", "gripper_r_joint2", "gripper_r_joint3", "gripper_r_joint4",
                "head_joint1", "head_joint2"
            };

            // Build mapping from joint names to indices
            for (size_t i = 1; i < model_.names.size(); ++i) {  // Skip universe joint
                joint_name_to_index_[model_.names[i]] = i;

                // Check if this joint should be excluded from IK
                if (std::find(excluded_joints_.begin(), excluded_joints_.end(), model_.names[i]) == excluded_joints_.end()) {
                    active_joint_indices_.push_back(i);
                }
            }

            RCLCPP_INFO(this->get_logger(), "Active joints for IK: %ld", active_joint_indices_.size());
            for (auto idx : active_joint_indices_) {
                RCLCPP_INFO(this->get_logger(), "  - %s (index %ld)", model_.names[idx].c_str(), idx);
            }

            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing Pinocchio model: %s", e.what());
            return false;
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update current joint configuration
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = joint_name_to_index_.find(msg->name[i]);
            if (it != joint_name_to_index_.end() && it->second < q_.size()) {
                q_[it->second] = msg->position[i];
            }
        }
        
        current_joint_state_ = *msg;
        has_joint_state_ = true;
        
        // Debug: Print received joint state for right arm joints
        static int count = 0;
        if (++count % 50 == 0) {  // Print every 50 messages to avoid spam
            RCLCPP_INFO(this->get_logger(), "Received joint states. Right arm positions:");
            for (const std::string& joint_name : {"arm_r_joint1", "arm_r_joint2", "arm_r_joint3", 
                                                 "arm_r_joint4", "arm_r_joint5", "arm_r_joint6", "arm_r_joint7"}) {
                auto it = joint_name_to_index_.find(joint_name);
                if (it != joint_name_to_index_.end()) {
                    RCLCPP_INFO(this->get_logger(), "  %s: %.6f", joint_name.c_str(), q_[it->second]);
                }
            }
        }
    }    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!has_joint_state_) {
            RCLCPP_WARN(this->get_logger(), "No joint state received yet, ignoring target pose");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Joint state received: %s", has_joint_state_ ? "YES" : "NO");
        
        // First, compute forward kinematics with current joint state to see where end effector is
        pinocchio::forwardKinematics(model_, data_, q_);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::SE3 current_ee_pose = data_.oMf[ee_frame_id_];
        
        RCLCPP_INFO(this->get_logger(), "Current end effector pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]", 
                   current_ee_pose.translation().x(), current_ee_pose.translation().y(), current_ee_pose.translation().z());
        
        Eigen::Quaterniond current_quat(current_ee_pose.rotation());
        RCLCPP_INFO(this->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
                   current_quat.w(), current_quat.x(), current_quat.y(), current_quat.z());
        
        auto start_time = std::chrono::high_resolution_clock::now();

        // Convert ROS pose to Pinocchio SE3
        pinocchio::SE3 target_pose;
        target_pose.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        target_pose.rotation() = Eigen::Quaterniond(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        ).toRotationMatrix();

        RCLCPP_INFO(this->get_logger(), "Received target pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
                   msg->pose.orientation.w, msg->pose.orientation.x,
                   msg->pose.orientation.y, msg->pose.orientation.z);

        // Solve IK
        Eigen::VectorXd solution = q_;  // Start from current configuration
        bool success = solveIK(target_pose, solution);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "IK solved successfully in %ld µs", duration.count());
            
            // Update current joint state with the solution
            q_ = solution;
            
            publishJointTrajectory(solution);

            // Verify the solution
            verifyIKSolution(solution, target_pose);
        } else {
            RCLCPP_ERROR(this->get_logger(), "IK failed to converge in %ld µs", duration.count());
        }
    }

    bool solveIK(const pinocchio::SE3& target_pose, Eigen::VectorXd& q_solution)
    {
        const double eps = tolerance_;
        const int max_iter = max_iterations_;

        Eigen::VectorXd q_iter = q_solution;
        double prev_error_norm = std::numeric_limits<double>::max();
        int stagnation_count = 0;
        const int max_stagnation = 10;  // Allow 10 iterations without improvement
        const double min_progress = 1e-6;  // Minimum progress to avoid stagnation

        for (int i = 0; i < max_iter; ++i) {
            // Update kinematics
            pinocchio::forwardKinematics(model_, data_, q_iter);
            pinocchio::updateFramePlacements(model_, data_);

            // Get current end effector pose
            pinocchio::SE3 current_pose = data_.oMf[ee_frame_id_];

            // Compute pose error
            pinocchio::SE3 error_se3 = target_pose.inverse() * current_pose;
            Eigen::VectorXd error = pinocchio::log6(error_se3).toVector();

            double error_norm = error.norm();
            
            // Debug: Print error information for first few iterations
            if (i < 5 || i % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Iteration %d: error norm = %.6f", i + 1, error_norm);
                RCLCPP_INFO(this->get_logger(), "  Position error: [%.6f, %.6f, %.6f]", 
                           error[0], error[1], error[2]);
                RCLCPP_INFO(this->get_logger(), "  Orientation error: [%.6f, %.6f, %.6f]", 
                           error[3], error[4], error[5]);
            }
            
            if (error_norm < eps) {
                q_solution = q_iter;
                RCLCPP_INFO(this->get_logger(), "🎯 IK CONVERGED in %d iterations! Final error: %.6f (tolerance: %.6f)", i + 1, error_norm, eps);
                return true;
            }

            // Check for stagnation (not making progress)
            if (std::abs(prev_error_norm - error_norm) < min_progress) {
                stagnation_count++;
                if (stagnation_count >= max_stagnation) {
                    RCLCPP_WARN(this->get_logger(), "IK stagnated after %d iterations, error norm: %.6f", i + 1, error_norm);
                    return false;  // Failed due to stagnation
                }
            } else {
                stagnation_count = 0;  // Reset stagnation counter
            }
            prev_error_norm = error_norm;

            // Compute Jacobian for the end effector frame
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_.nv);
            pinocchio::computeFrameJacobian(model_, data_, q_iter, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);

            // Debug: Print Jacobian size and some values
            if (i == 0) {
                RCLCPP_INFO(this->get_logger(), "Jacobian shape: %ld x %ld", J.rows(), J.cols());
                RCLCPP_INFO(this->get_logger(), "Active joints count: %ld", active_joint_indices_.size());
                for (size_t j = 0; j < active_joint_indices_.size(); ++j) {
                    size_t joint_idx = active_joint_indices_[j];
                    size_t q_idx = model_.joints[joint_idx].idx_q();
                    size_t v_idx = model_.joints[joint_idx].idx_v();
                    RCLCPP_INFO(this->get_logger(), "Joint %s (idx %ld) -> q_idx %ld, v_idx %ld", 
                               model_.names[joint_idx].c_str(), joint_idx, q_idx, v_idx);
                }
            }

            // Only use active joints (use velocity indices from joint model)
            Eigen::MatrixXd J_active(6, active_joint_indices_.size());
            for (size_t j = 0; j < active_joint_indices_.size(); ++j) {
                size_t joint_idx = active_joint_indices_[j];
                // Get the velocity index for this joint
                size_t v_idx = model_.joints[joint_idx].idx_v();
                if (v_idx < J.cols()) {
                    J_active.col(j) = J.col(v_idx);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Velocity index %ld out of bounds (Jacobian cols: %ld)", 
                                v_idx, J.cols());
                    return false;
                }
            }

            // Compute pseudo-inverse using SVD with damping
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_active, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const double svd_threshold = 1e-6;
            const double damping = 1e-6;  // Add damping for stability
            
            Eigen::VectorXd singular_values = svd.singularValues();
            Eigen::VectorXd singular_values_inv = singular_values;

            for (int k = 0; k < singular_values.size(); ++k) {
                if (singular_values(k) > svd_threshold) {
                    singular_values_inv(k) = singular_values(k) / (singular_values(k) * singular_values(k) + damping * damping);
                } else {
                    singular_values_inv(k) = 0.0;
                }
            }

            Eigen::MatrixXd J_pinv = svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose();

            // Compute joint velocity with smaller step size
            Eigen::VectorXd dq_active = -step_size_ * J_pinv * error;
            
            // Limit the joint velocity to prevent large jumps
            const double max_joint_velocity = 0.5;  // rad per iteration
            for (int k = 0; k < dq_active.size(); ++k) {
                dq_active[k] = std::max(-max_joint_velocity, std::min(max_joint_velocity, dq_active[k]));
            }

            // Update only active joints using their configuration indices
            for (size_t j = 0; j < active_joint_indices_.size(); ++j) {
                size_t joint_idx = active_joint_indices_[j];
                // Get the configuration index for this joint
                size_t q_idx = model_.joints[joint_idx].idx_q();
                if (q_idx < q_iter.size()) {
                    q_iter[q_idx] += dq_active[j];
                }
            }

            // Apply joint limits (simple clamping) to active joints
            for (size_t j = 0; j < active_joint_indices_.size(); ++j) {
                size_t joint_idx = active_joint_indices_[j];
                size_t q_idx = model_.joints[joint_idx].idx_q();
                if (q_idx < q_iter.size() && q_idx < model_.lowerPositionLimit.size() && q_idx < model_.upperPositionLimit.size()) {
                    q_iter[q_idx] = std::max(model_.lowerPositionLimit[q_idx],
                                           std::min(model_.upperPositionLimit[q_idx], q_iter[q_idx]));
                }
            }

            if (i % 100 == 0) {
                RCLCPP_DEBUG(this->get_logger(), "Iteration %d: error norm = %.6f", i + 1, error_norm);
            }
        }

        RCLCPP_WARN(this->get_logger(), "IK failed to converge after %d iterations", max_iter);
        return false;
    }

    void verifyIKSolution(const Eigen::VectorXd& q_solution, const pinocchio::SE3& target_pose)
    {
        // Forward kinematics with the solution
        pinocchio::forwardKinematics(model_, data_, q_solution);
        pinocchio::updateFramePlacements(model_, data_);

        pinocchio::SE3 achieved_pose = data_.oMf[ee_frame_id_];

        // Compute position and orientation errors
        Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();

        // Convert rotations to quaternions for easier comparison
        Eigen::Quaterniond target_quat(target_pose.rotation());
        Eigen::Quaterniond achieved_quat(achieved_pose.rotation());

        // Compute angular error
        Eigen::Quaterniond quat_error = target_quat.inverse() * achieved_quat;
        double angle_error = 2.0 * std::acos(std::abs(quat_error.w()));

        RCLCPP_INFO(this->get_logger(), "IK Solution Verification:");
        RCLCPP_INFO(this->get_logger(), "  Target pos: [%.6f, %.6f, %.6f]",
                   target_pose.translation().x(), target_pose.translation().y(), target_pose.translation().z());
        RCLCPP_INFO(this->get_logger(), "  Achieved pos: [%.6f, %.6f, %.6f]",
                   achieved_pose.translation().x(), achieved_pose.translation().y(), achieved_pose.translation().z());
        RCLCPP_INFO(this->get_logger(), "  Position error: [%.6f, %.6f, %.6f] (norm: %.6f)",
                   pos_error.x(), pos_error.y(), pos_error.z(), pos_error.norm());
        RCLCPP_INFO(this->get_logger(), "  Angular error: %.6f rad (%.2f deg)", angle_error, angle_error * 180.0 / M_PI);

        // Print joint values for active joints
        RCLCPP_INFO(this->get_logger(), "Joint solution:");
        for (auto idx : active_joint_indices_) {
            RCLCPP_INFO(this->get_logger(), "  %s: %.6f", model_.names[idx].c_str(), q_solution[idx]);
        }
    }

    void publishJointTrajectory(const Eigen::VectorXd& q_solution)
    {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        // traj_msg.header.stamp = this->now();
        traj_msg.header.frame_id = "";

        // Add only the right arm joints that we actually control (excluding lift_joint)
        std::vector<std::string> controlled_joints = {
            "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
            "arm_r_joint5", "arm_r_joint6", "arm_r_joint7"
        };

        for (const auto& joint_name : controlled_joints) {
            auto it = joint_name_to_index_.find(joint_name);
            if (it != joint_name_to_index_.end()) {
                traj_msg.joint_names.push_back(joint_name);
            }
        }

        // Add gripper joint with default open position
        traj_msg.joint_names.push_back("gripper_r_joint1");

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start.sec = 0;    // Execute immediately like realtime_ik_solver_jl
        point.time_from_start.nanosec = 0;

        // Add arm joint positions
        for (const auto& joint_name : controlled_joints) {
            auto it = joint_name_to_index_.find(joint_name);
            if (it != joint_name_to_index_.end()) {
                point.positions.push_back(q_solution[it->second]);
            }
        }

        // Add gripper position (slightly open)
        point.positions.push_back(0.1);

        // Leave velocities, accelerations, and effort empty like realtime_ik_solver_jl
        point.velocities = {};
        point.accelerations = {};
        point.effort = {};

        traj_msg.points.push_back(point);
        joint_trajectory_pub_->publish(traj_msg);

        RCLCPP_INFO(this->get_logger(), "📤 Joint trajectory sent to leader! Robot should move immediately.");
        RCLCPP_INFO(this->get_logger(), "Published trajectory with %ld arm joints + gripper", controlled_joints.size());
    }

private:
    // ROS2 interfaces
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Pinocchio model and data
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_;

    // Configuration
    bool use_robot_description_;
    std::string urdf_path_;
    std::string base_link_;
    std::string end_effector_link_;
    int max_iterations_;
    double tolerance_;
    double step_size_;

    // State
    Eigen::VectorXd q_;  // Current joint configuration
    sensor_msgs::msg::JointState current_joint_state_;
    bool has_joint_state_ = false;

    // Joint management
    std::map<std::string, size_t> joint_name_to_index_;
    std::vector<size_t> active_joint_indices_;  // Joints that participate in IK
    std::vector<std::string> excluded_joints_;  // Joints excluded from IK
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PinocchioIKSolver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
