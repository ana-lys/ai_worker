#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>
#include <memory>
#include <vector>
#include <chrono>
#include <iomanip>

struct IKSolverResult {
    std::string solver_name;
    int error_code;
    double computation_time_ms;
    double position_error;
    double orientation_error;
    bool success;
    int iterations;
    std::vector<double> joint_solution;
};

class IKSolverComparison : public rclcpp::Node
{
public:
    IKSolverComparison() : Node("ik_solver_comparison"),
                           test_count_(0),
                           lift_joint_index_(-1),
                           setup_complete_(false),
                           has_joint_states_(false)
    {
        // Parameters
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("end_effector_link", "arm_r_link7");
        this->declare_parameter<int>("num_tests", 10);

        base_link_ = this->get_parameter("base_link").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        num_tests_ = this->get_parameter("num_tests").as_int();

        RCLCPP_INFO(this->get_logger(), "🔬 IK Solver Comparison Tool starting...");
        RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Number of tests: %d", num_tests_);

        // Subscribers
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&IKSolverComparison::robotDescriptionCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&IKSolverComparison::jointStateCallback, this, std::placeholders::_1));

        // Publishers
        result_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ik_comparison_result", 10);

        // Timer for running tests
        test_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&IKSolverComparison::runComparison, this));

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
                return;
            }

            RCLCPP_INFO(this->get_logger(), "KDL chain extracted with %d joints", chain_.getNrOfJoints());

            // Create FK solver
            fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

            // Create velocity solvers
            ik_vel_solver_pinv_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
            ik_vel_solver_wdls_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(chain_, 1e-6);

            // Create IK solvers
            setupIKSolvers();

            // Extract joint names and identify lift_joint index
            extractJointNames();

            setup_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ All IK solvers setup completed successfully!");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing robot description: %s", e.what());
        }
    }

    void setupIKSolvers()
    {
        // Newton-Raphson solvers with different parameters
        ik_solver_nr_fast_ = std::make_unique<KDL::ChainIkSolverPos_NR>(
            chain_, *fk_solver_, *ik_vel_solver_pinv_, 100, 1e-2);  // Fast but less accurate

        ik_solver_nr_accurate_ = std::make_unique<KDL::ChainIkSolverPos_NR>(
            chain_, *fk_solver_, *ik_vel_solver_pinv_, 1000, 1e-6);  // Slow but accurate

        ik_solver_nr_balanced_ = std::make_unique<KDL::ChainIkSolverPos_NR>(
            chain_, *fk_solver_, *ik_vel_solver_pinv_, 500, 1e-3);   // Balanced

        ik_solver_nr_wdls_ = std::make_unique<KDL::ChainIkSolverPos_NR>(
            chain_, *fk_solver_, *ik_vel_solver_wdls_, 500, 1e-3);   // With WDLS

        // Joint limits solver (set reasonable limits)
        KDL::JntArray q_min(chain_.getNrOfJoints());
        KDL::JntArray q_max(chain_.getNrOfJoints());
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            q_min(i) = -3.14159;  // -180 degrees
            q_max(i) = 3.14159;   // +180 degrees
        }

        ik_solver_nr_jl_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
            chain_, q_min, q_max, *fk_solver_, *ik_vel_solver_pinv_, 500, 1e-3);

        // Levenberg-Marquardt solver
        ik_solver_lma_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, 1e-5, 500, 1e-15);

        RCLCPP_INFO(this->get_logger(), "Created 6 different IK solvers for comparison");
    }

    void extractJointNames()
    {
        joint_names_.clear();
        lift_joint_index_ = -1;

        for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
            KDL::Segment segment = chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None) {
                std::string joint_name = segment.getJoint().getName();
                joint_names_.push_back(joint_name);

                if (joint_name == "lift_joint") {
                    lift_joint_index_ = joint_names_.size() - 1;
                    RCLCPP_INFO(this->get_logger(), "Found lift_joint at index %d", lift_joint_index_);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Extracted %zu joint names", joint_names_.size());
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

        if (all_joints_found && !has_joint_states_) {
            has_joint_states_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ Joint states received. Ready for comparison tests!");
        }
    }

    void runComparison()
    {
        if (!setup_complete_ || !has_joint_states_ || test_count_ >= num_tests_) {
            return;
        }

        test_count_++;

        RCLCPP_INFO(this->get_logger(), "\n🔬 ========== IK Solver Comparison Test %d/%d ==========",
                   test_count_, num_tests_);

        // Generate random target pose
        KDL::Frame target_frame = generateRandomTarget();

        RCLCPP_INFO(this->get_logger(), "🎯 Target pose:");
        RCLCPP_INFO(this->get_logger(), "   Position: [%.3f, %.3f, %.3f]",
                   target_frame.p.x(), target_frame.p.y(), target_frame.p.z());

        // Test all solvers
        std::vector<IKSolverResult> results;

        results.push_back(testSolver("NR_Fast", ik_solver_nr_fast_.get(), target_frame));
        results.push_back(testSolver("NR_Accurate", ik_solver_nr_accurate_.get(), target_frame));
        results.push_back(testSolver("NR_Balanced", ik_solver_nr_balanced_.get(), target_frame));
        results.push_back(testSolver("NR_WDLS", ik_solver_nr_wdls_.get(), target_frame));
        results.push_back(testSolver("NR_JointLimits", ik_solver_nr_jl_.get(), target_frame));
        results.push_back(testSolver("LMA", ik_solver_lma_.get(), target_frame));

        // Print comparison results
        printComparisonResults(results, target_frame);

        if (test_count_ >= num_tests_) {
            RCLCPP_INFO(this->get_logger(), "\n🏁 All comparison tests completed!");
            test_timer_.reset();
        }
    }

    KDL::Frame generateRandomTarget()
    {
        KDL::Frame target;

        // Generate random position around current end-effector position
        KDL::JntArray q_current(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            q_current(i) = current_joint_positions_[i];
        }

        KDL::Frame current_frame;
        fk_solver_->JntToCart(q_current, current_frame);

        // Random offset within reasonable range
        double offset_range = 0.2; // 20cm range
        target.p.x(current_frame.p.x() + (rand() % 1000 - 500) * offset_range / 1000.0);
        target.p.y(current_frame.p.y() + (rand() % 1000 - 500) * offset_range / 1000.0);
        target.p.z(current_frame.p.z() + (rand() % 1000 - 500) * offset_range / 1000.0);

        // Keep orientation simple for now
        target.M = KDL::Rotation::Identity();

        return target;
    }

    IKSolverResult testSolver(const std::string& name, KDL::ChainIkSolverPos* solver,
                             const KDL::Frame& target)
    {
        IKSolverResult result;
        result.solver_name = name;

        // Initial guess - current joint positions
        KDL::JntArray q_init(chain_.getNrOfJoints());
        KDL::JntArray q_result(chain_.getNrOfJoints());

        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            q_init(i) = current_joint_positions_[i];
        }

        // Measure computation time
        auto start = std::chrono::high_resolution_clock::now();

        int error_code = solver->CartToJnt(q_init, target, q_result);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        result.error_code = error_code;
        result.computation_time_ms = duration.count() / 1000.0;
        result.success = (error_code >= 0);
        result.iterations = std::max(0, error_code);  // Positive error_code = iterations

        if (result.success) {
            // Fix lift_joint if needed
            if (lift_joint_index_ >= 0 && lift_joint_index_ < (int)q_result.rows()) {
                q_result(lift_joint_index_) = current_joint_positions_[lift_joint_index_];
            }

            // Store joint solution
            result.joint_solution.clear();
            for (unsigned int i = 0; i < q_result.rows(); i++) {
                result.joint_solution.push_back(q_result(i));
            }

            // Calculate errors
            KDL::Frame verify_frame;
            fk_solver_->JntToCart(q_result, verify_frame);

            KDL::Vector pos_error = verify_frame.p - target.p;
            result.position_error = sqrt(
                pos_error.x() * pos_error.x() +
                pos_error.y() * pos_error.y() +
                pos_error.z() * pos_error.z()
            );

            // Orientation error (simplified)
            KDL::Rotation rot_error = target.M * verify_frame.M.Inverse();
            KDL::Vector axis;
            double angle = rot_error.GetRotAngle(axis);
            result.orientation_error = std::abs(angle);
        } else {
            result.position_error = -1.0;
            result.orientation_error = -1.0;
        }

        return result;
    }

    void printComparisonResults(const std::vector<IKSolverResult>& results,
                               const KDL::Frame& /* target */)
    {
        RCLCPP_INFO(this->get_logger(), "\n📊 Results Summary:");
        RCLCPP_INFO(this->get_logger(), "%-15s | %8s | %8s | %10s | %10s | %6s",
                   "Solver", "Success", "Time(ms)", "Pos Error", "Rot Error", "Iters");
        RCLCPP_INFO(this->get_logger(), "%s", std::string(75, '-').c_str());

        for (const auto& result : results) {
            std::string success_str = result.success ? "✅ YES" : "❌ NO";
            std::string pos_error_str = result.success ?
                (std::to_string(result.position_error * 1000.0).substr(0, 6) + "mm") : "N/A";
            std::string rot_error_str = result.success ?
                (std::to_string(result.orientation_error * 180.0 / M_PI).substr(0, 6) + "°") : "N/A";
            std::string iter_str = result.success ? std::to_string(result.iterations) :
                ("ERR:" + std::to_string(result.error_code));

            RCLCPP_INFO(this->get_logger(), "%-15s | %8s | %8.2f | %10s | %10s | %6s",
                       result.solver_name.c_str(),
                       success_str.c_str(),
                       result.computation_time_ms,
                       pos_error_str.c_str(),
                       rot_error_str.c_str(),
                       iter_str.c_str());
        }

        // Find best performing solver
        auto best_solver = std::min_element(results.begin(), results.end(),
            [](const IKSolverResult& a, const IKSolverResult& b) {
                if (!a.success && !b.success) return false;
                if (!a.success) return false;
                if (!b.success) return true;
                return (a.position_error + a.computation_time_ms / 100.0) <
                       (b.position_error + b.computation_time_ms / 100.0);
            });

        if (best_solver != results.end() && best_solver->success) {
            RCLCPP_INFO(this->get_logger(), "\n🏆 Best solver this round: %s",
                       best_solver->solver_name.c_str());
        }
    }

private:
    // Parameters
    std::string base_link_;
    std::string end_effector_link_;
    int num_tests_;
    int test_count_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr result_pub_;
    rclcpp::TimerBase::SharedPtr test_timer_;

    // KDL objects
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    // Velocity solvers
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_pinv_;
    std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_wdls_;

    // Position IK solvers for comparison
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_nr_fast_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_nr_accurate_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_nr_balanced_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_nr_wdls_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_nr_jl_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_lma_;

    // Joint information
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    int lift_joint_index_;

    // Status flags
    bool setup_complete_;
    bool has_joint_states_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IKSolverComparison>();

    RCLCPP_INFO(node->get_logger(), "🔬 IK Solver Comparison Tool started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
