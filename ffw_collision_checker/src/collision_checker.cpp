#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <ffw_collision_checker/msg/collision_check.hpp>
#include <ffw_collision_checker/srv/solve_collision_naive.hpp>

class CollisionCheckerPublisher : public rclcpp::Node
{
public:
  CollisionCheckerPublisher() : Node("collision_checker_publisher")
  {
    declare_parameter("use_gui", false);
    declare_parameter("gui_update_rate", 20.0);
    
    use_gui_ = get_parameter("use_gui").as_bool();
    gui_update_rate_ = get_parameter("gui_update_rate").as_double();
    
    loadMujocoModel("robotis_ffw/scene.xml");
    
    if (use_gui_) {
      initializeGui();
    }
  }

  void init()
  {
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CollisionCheckerPublisher::jointStateCallback, this, std::placeholders::_1));

    // Create publisher for collision check results
    collision_pub_ = this->create_publisher<ffw_collision_checker::msg::CollisionCheck>(
      "/collision/collision_check", 10);

    // Create service client for collision solver
    solve_collision_client_ = this->create_client<ffw_collision_checker::srv::SolveCollisionNaive>(
      "/collision/solve_collision_naive");

    // Create GUI update timer if GUI is enabled
    if (use_gui_) {
      auto period = std::chrono::duration<double>(1.0 / gui_update_rate_);
      gui_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CollisionCheckerPublisher::guiUpdateCallback, this));
    }

    RCLCPP_INFO(get_logger(), "Collision checker publisher ready!");
    if (use_gui_) {
      RCLCPP_INFO(get_logger(), "GUI update rate: %.1f Hz", gui_update_rate_);
    }
  }

  bool useGui() const { return use_gui_; }

  ~CollisionCheckerPublisher()
  {
    if (use_gui_ && window_) {
      mjv_freeScene(&scn_);
      mjr_freeContext(&con_);
      glfwDestroyWindow(window_);
      glfwTerminate();
    }
    
    if (mj_data_) mj_deleteData(mj_data_);
    if (mj_model_) mj_deleteModel(mj_model_);
  }

  void spin()
  {
    if (!use_gui_ || !window_) {
      // No GUI → normal single-threaded spin
      rclcpp::spin(shared_from_this());
      return;
    }

    // GUI mode: use MultiThreadedExecutor for ROS callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(shared_from_this());

    RCLCPP_INFO(get_logger(), "Starting GUI loop (GLFW event polling in main thread)");

    // Main thread handles GLFW events
    while (rclcpp::ok() && !glfwWindowShouldClose(window_)) {
      glfwPollEvents();  // Process window close, resize, input, etc.
      executor.spin_some();  // Non-blocking: process any pending ROS callbacks
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Avoid 100% CPU
    }

    RCLCPP_INFO(get_logger(), "GUI window closed or shutdown requested.");
  }

private:
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;

  // Separate data for collision solver computations
  mjData* mj_data_solver_ = nullptr;
  
  // Visualization data
  bool use_gui_ = false;
  double gui_update_rate_ = 10.0;
  bool solver_needed_ = false;     // set when collision detected
  bool solver_solved_ = false;    // prevent reentry

  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<ffw_collision_checker::msg::CollisionCheck>::SharedPtr collision_pub_;
  rclcpp::TimerBase::SharedPtr gui_timer_;
  rclcpp::Client<ffw_collision_checker::srv::SolveCollisionNaive>::SharedPtr solve_collision_client_;
  
  std::vector<double> latest_qpos_;
  std::vector<int> msg_idx_to_qpos_;
  std::vector<int> solution_;

  std::deque<int> left_default_solver_deque_;
  std::deque<int> right_default_solver_deque_;

  std::vector<std::pair<int, int>> mimic_pos_adr_pairs_;
  std::vector<std::string> joint_adr_to_name_;

  bool state_map_initialized_ = false;

  std::map<std::string, int> joint_name_to_qpos_adr_;
  
  // Mutex to protect mj_data_ access between callback and GUI update
  std::mutex data_mutex_;

  void loadMujocoModel(const std::string& xml_name)
  { 
    using ament_index_cpp::get_package_share_directory;

    std::string collision_pkg = get_package_share_directory("ffw_collision_checker");
    std::string mujoco_xml_path = collision_pkg + "/3rd_party/" + xml_name;

    char error[1000] = "Could not load binary model";
    
    mj_model_ = mj_loadXML(mujoco_xml_path.c_str(), 0, error, 1000);
    if (mj_model_) {
      // Build joint name to qpos address map
      for (int i = 0; i < mj_model_->njnt; ++i) {
        const char* name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
        if (name) {
          joint_name_to_qpos_adr_[std::string(name)] = mj_model_->jnt_qposadr[i];
        }
      }
      joint_adr_to_name_.clear();
      joint_adr_to_name_.resize(mj_model_->njnt);
      for (int i = 0; i < mj_model_->njnt; ++i) {
      const char* name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
      if (name) {
          // Map ONLY the starting address to the name
          joint_adr_to_name_[mj_model_->jnt_qposadr[i]] = name;
        }
      }
      mimic_pos_adr_pairs_.clear();

      // Parse equality constraints for mimic joints
      for (int i = 0; i < mj_model_->neq; ++i) {
        if (mj_model_->eq_type[i] == mjEQ_JOINT) {
          int slave_id = mj_model_->eq_obj1id[i];
          int master_id = mj_model_->eq_obj2id[i];
          int slave_pos_adr = mj_model_->jnt_qposadr[slave_id];
          int master_pos_adr = mj_model_->jnt_qposadr[master_id];

          mimic_pos_adr_pairs_.emplace_back(slave_pos_adr, master_pos_adr);
          
          const char* slave_name = mj_id2name(mj_model_, mjOBJ_JOINT, slave_id);
          const char* master_name = mj_id2name(mj_model_, mjOBJ_JOINT, master_id);
          RCLCPP_INFO(get_logger(), "Mimic joint found: %s follows %s", 
                      slave_name ? slave_name : "unknown",
                      master_name ? master_name : "unknown");
        }
      }
      
      mj_data_ = mj_makeData(mj_model_);
      mj_data_solver_ = mj_data_; // Initialize solver data

      // Initialize latest_qpos_ with default values
      latest_qpos_.resize(mj_model_->nq, 0.0);
      if (mj_model_->qpos0) {
        for(int i = 0; i < mj_model_->nq; ++i) {
          latest_qpos_[i] = mj_model_->qpos0[i];
        }
      }   

    left_default_solver_deque_ = std::deque<int>{0 , 10 , 9 , 8 ,7 , 6 , 5 , 4 , 3 };
    right_default_solver_deque_ = std::deque<int>{21 , 20 , 19 , 18 ,17 ,16 ,15 , 14 , 1 , 2 };
    
    for( int i =0 ; i < left_default_solver_deque_.size(); ++i )
    {
      RCLCPP_INFO(get_logger(), "Solver deque left: %d %s", left_default_solver_deque_[i] , joint_adr_to_name_[left_default_solver_deque_[i]].c_str() );
    } 

    for ( int i =0 ; i < right_default_solver_deque_.size(); ++i )
    {
      RCLCPP_INFO(get_logger(), "Solver deque right: %d %s", right_default_solver_deque_[i] , joint_adr_to_name_[right_default_solver_deque_[i]].c_str() );
    }

    // lift_joint: qpos_adr=0
    // gripper_l_joint1: qpos_adr=10
    // arm_l_joint1: qpos_adr=3
    // arm_l_joint2: qpos_adr=4
    // arm_l_joint3: qpos_adr=5
    // arm_l_joint4: qpos_adr=6
    // arm_l_joint5: qpos_adr=7
    // arm_l_joint6: qpos_adr=8
    // arm_l_joint7: qpos_adr=9

    // gripper_r_joint1: qpos_adr=21
    // arm_r_joint1: qpos_adr=14
    // arm_r_joint2: qpos_adr=15
    // arm_r_joint3: qpos_adr=16
    // arm_r_joint4: qpos_adr=17
    // arm_r_joint5: qpos_adr=18
    // arm_r_joint6: qpos_adr=19
    // arm_r_joint7: qpos_adr=20
    // head_joint1: qpos_adr=1
    // head_joint2: qpos_adr=2

      RCLCPP_INFO(get_logger(), "MuJoCo model loaded successfully from: %s", mujoco_xml_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to load MuJoCo model: %s", error);
    }
  }

  void initializeGui()
  {
    if (!mj_model_) return;

    if (!glfwInit()) {
      RCLCPP_ERROR(get_logger(), "Could not initialize GLFW");
      use_gui_ = false;
      return;
    }

    window_ = glfwCreateWindow(1200, 900, "Collision Checker", NULL, NULL);
    if (!window_) {
      glfwTerminate();
      use_gui_ = false;
      return;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam_);
    
    // Set initial camera view
    cam_.distance = 3.0;
    cam_.azimuth = 180.0;
    cam_.elevation = -20.0;
    cam_.type = mjCAMERA_FREE;
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 1.25;

    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    mjv_makeScene(mj_model_, &scn_, 2000);
    mjr_makeContext(mj_model_, &con_, mjFONTSCALE_150);
    
    RCLCPP_INFO(get_logger(), "GUI initialized successfully");
  }

  void guiUpdateCallback()
  {
    if (!use_gui_ || !window_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // Update scene and render
    if (!solver_needed_) {
      mjv_updateScene(mj_model_, mj_data_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    } else {
      mjv_updateScene(mj_model_, mj_data_solver_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    }
    mjr_render(viewport, &scn_, &con_);

    glfwSwapBuffers(window_);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!mj_model_ || !mj_data_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MuJoCo model or data not initialized yet.");
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    // Initialize mapping on first message
    if (!state_map_initialized_) {
      msg_idx_to_qpos_.clear();
      msg_idx_to_qpos_.resize(msg->name.size(), -1);
      
      for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it_q = joint_name_to_qpos_adr_.find(msg->name[i]);
        if (it_q != joint_name_to_qpos_adr_.end()) {
          msg_idx_to_qpos_[i] = it_q->second;
        }
        
        RCLCPP_INFO(get_logger(), "Mapping joint %s: qpos_adr=%d",
                    msg->name[i].c_str(),
                    msg_idx_to_qpos_[i]);
      }
      state_map_initialized_ = true;
    }

    // Update positions
    for (size_t i = 0; i < msg->position.size(); ++i) {
      if (i < msg_idx_to_qpos_.size()) {
        int qpos_adr = msg_idx_to_qpos_[i];
        if (qpos_adr >= 0 && qpos_adr < (int)latest_qpos_.size()) {
          latest_qpos_[qpos_adr] = msg->position[i];
        }
      }
    }

    // Update qpos in mujoco data
    for (int i = 0; i < mj_model_->nq; ++i) {
      mj_data_->qpos[i] = latest_qpos_[i];
    }

    // Handle mimic joints
    for (size_t i = 0; i < mimic_pos_adr_pairs_.size(); ++i) {
      int slave_pos_adr = mimic_pos_adr_pairs_[i].first;
      int master_pos_adr = mimic_pos_adr_pairs_[i].second;
      mj_data_->qpos[slave_pos_adr] = mj_data_->qpos[master_pos_adr];
    }

    // Forward kinematics
    mj_forward(mj_model_, mj_data_);

    // Check collisions and prepare message
    auto collision_msg = ffw_collision_checker::msg::CollisionCheck();
    collision_msg.in_collision = false;

    if (mj_data_->ncon > 0) {
      for (int i = 0; i < mj_data_->ncon; ++i) {
        double dist = mj_data_->contact[i].dist;

        // Get geometry IDs
        int geom1_id = mj_data_->contact[i].geom1;
        int geom2_id = mj_data_->contact[i].geom2;

        // Get geometry names
        const char* geom1 = mj_id2name(mj_model_, mjOBJ_GEOM, geom1_id);
        const char* geom2 = mj_id2name(mj_model_, mjOBJ_GEOM, geom2_id);

        // Get parent body (link) for each geometry
        int body1_id = mj_model_->geom_bodyid[geom1_id];
        int body2_id = mj_model_->geom_bodyid[geom2_id];
      
        const char* body1 = mj_id2name(mj_model_, mjOBJ_BODY, body1_id);
        const char* body2 = mj_id2name(mj_model_, mjOBJ_BODY, body2_id);
      
        // Create descriptive names: prefer body name, fallback to geom name or ID
        std::string name1, name2;
        if (body1) {
          name1 = std::string(body1);
          if (geom1) name1 += "/" + std::string(geom1);
        } else {
          name1 = geom1 ? std::string(geom1) : "geom_" + std::to_string(geom1_id);
        }
      
        if (body2) {
          name2 = std::string(body2);
          if (geom2) name2 += "/" + std::string(geom2);
        } else {
          name2 = geom2 ? std::string(geom2) : "geom_" + std::to_string(geom2_id);
        }
      
        // Add to message arrays
        collision_msg.distances.push_back(dist);
        collision_msg.geom1_names.push_back(name1);
        collision_msg.geom2_names.push_back(name2);

        // Check if actual collision (negative distance)
        if (dist < 0) {
          collision_msg.in_collision = true;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Collision: %s <-> %s | dist: %.4f",
                               name1.c_str(),
                               name2.c_str(),
                               dist);
        }
      }
    }

    // Publish collision message
    collision_pub_->publish(collision_msg);
    if (collision_msg.in_collision && !solver_solved_) {
      solver_needed_ = true;
    }
    if (!collision_msg.in_collision) {
      mj_data_solver_ = mj_data_;
    }
    solveCollision();
  }

  bool solveJointGradually(int joint_qpos_adr, const std::string& joint_name, const char* side_name)
{
  if (!mj_data_solver_) return false;
  
  // Store original value
  double original_value = mj_data_solver_->qpos[joint_qpos_adr];
  double target_value = 0.0;
  
  const int num_steps = 20;
  bool all_steps_valid = true;
  
  // Try moving in 10 steps from current to zero
  for (int step = 1; step <= num_steps; ++step) {
    double alpha = static_cast<double>(step) / num_steps;
    double interpolated_value = original_value * (1.0 - alpha) + target_value * alpha;
    
    // Update solver data
    mj_data_solver_->qpos[joint_qpos_adr] = interpolated_value;
    
    // Handle mimic joints
    for (size_t i = 0; i < mimic_pos_adr_pairs_.size(); ++i) {
      int slave_pos_adr = mimic_pos_adr_pairs_[i].first;
      int master_pos_adr = mimic_pos_adr_pairs_[i].second;
      mj_data_solver_->qpos[slave_pos_adr] = mj_data_solver_->qpos[master_pos_adr];
    }
    // Check collision at this step
    mj_forward(mj_model_, mj_data_solver_);
    
    bool has_collision = false;
    if (mj_data_solver_->ncon > 0) {
      for (int i = 0; i < mj_data_solver_->ncon; ++i) {
        if (mj_data_solver_->contact[i].dist < 0) {
          has_collision = true;
          break;
        }
      }
    }
    
    if (has_collision) {
      RCLCPP_WARN(get_logger(), "[%s] Joint %s failed at step %d/%d (value=%.3f)", 
                  side_name, joint_name.c_str(), step, num_steps, interpolated_value);
      all_steps_valid = false;
      break;
    }
  }
  
  if (all_steps_valid) {
    // Success! Keep the joint at zero
    mj_data_solver_->qpos[joint_qpos_adr] = target_value;
    RCLCPP_INFO(get_logger(), "[%s] ✓ Solved joint %s (%.3f -> 0.0)", 
                side_name, joint_name.c_str(), original_value);
    return true;
  } else {
    // Rollback to original value
    mj_data_solver_->qpos[joint_qpos_adr] = original_value;
    mj_forward(mj_model_, mj_data_solver_);
    return false;
  }
}

void solveCollision()
{
  if (solver_solved_ || !solver_needed_ || !mj_data_ || !mj_model_ || !mj_data_solver_) return;
  
  RCLCPP_INFO(get_logger(), "Starting collision solver...");
  
  // Create solver data copy
  if (!mj_data_solver_) {
    mj_data_solver_ = mj_makeData(mj_model_);
  }
  
  // Copy current state to solver data
  for (int i = 0; i < mj_model_->nq; ++i) {
    mj_data_solver_->qpos[i] = mj_data_->qpos[i];
  }
  
  // Clear solution
  solution_.clear();
  
  // Make working copies of deques
  std::deque<int> left_work = left_default_solver_deque_;
  std::deque<int> right_work = right_default_solver_deque_;
  
  bool use_left = true;
  int iteration = 0;
  const int max_iterations = 100;
  
  while ((!left_work.empty() || !right_work.empty()) && iteration < max_iterations) {
    iteration++;
    std::deque<int>& current_deque = use_left ? left_work : right_work;
    const char* side_name = use_left ? "LEFT" : "RIGHT";
    
    if (current_deque.empty()) {
      use_left = !use_left;
      continue;
    }
    
    // Get front joint
    int joint_qpos_adr = current_deque.front();
    current_deque.pop_front();
    
    // Get joint name for logging
    std::string joint_name = "unknown";
    for (const auto& pair : joint_name_to_qpos_adr_) {
      if (pair.second == joint_qpos_adr) {
        joint_name = pair.first;
        break;
      }
    }
    
    // Try to solve this joint gradually
    if (solveJointGradually(joint_qpos_adr, joint_name, side_name)) {
      // Success! Add to solution
      solution_.push_back(joint_qpos_adr);
    } else {
      // Failed, move to back
      current_deque.push_back(joint_qpos_adr);

      // Switch sides
      use_left = !use_left;
    }
    
   
  }
  
  RCLCPP_INFO(get_logger(), "Solver finished: %zu joints solved in %d iterations", 
              solution_.size(), iteration);
  
  // Check final collision state
  mj_forward(mj_model_, mj_data_solver_);
  bool final_collision = false;
  if (mj_data_solver_->ncon > 0) {
    for (int i = 0; i < mj_data_solver_->ncon; ++i) {
      if (mj_data_solver_->contact[i].dist < 0) {
        final_collision = true;
        break;
      }
    }
  }
  
  
  RCLCPP_INFO(get_logger(), final_collision ? "✗ Collision still present" : "✓ Collision resolved!");
  if(!final_collision)
  { 
    sendSolverServiceRequest();
  }
  // Send service request with solution
  
}

  void sendSolverServiceRequest()
  {
    if (!solve_collision_client_) {
      RCLCPP_ERROR(get_logger(), "Service client not initialized");
      return;
    }

    // Wait for service to be available
    if (!solve_collision_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Service /solve_collision_naive not available, sending empty request");
      // Send empty message - service not available
      return;
    }

    // Prepare request with joint names from solution
    auto request = std::make_shared<ffw_collision_checker::srv::SolveCollisionNaive::Request>();
    
    for (int qpos_adr : solution_) {
      std::string joint_name = joint_adr_to_name_[qpos_adr];
      request->joint_names.push_back(joint_name);
    }

    RCLCPP_INFO(get_logger(), "Sending solver service request with %zu joint names", request->joint_names.size());

    // Send async request
    auto future = solve_collision_client_->async_send_request(request,
      [this](rclcpp::Client<ffw_collision_checker::srv::SolveCollisionNaive>::SharedFuture response_future) {
        try {
          auto response = response_future.get();
          if (response->accept) {
            RCLCPP_INFO(get_logger(), "✓ Solver service accepted solution");
                solver_solved_ = false;
                solver_needed_ = false;
          } else {
            RCLCPP_WARN(get_logger(), "✗ Solver service rejected solution (error: %d)", response->error);
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s, sending empty message", e.what());
        }
      });
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionCheckerPublisher>();
  node->init();

  node->spin();

  rclcpp::shutdown();
  return 0;
}