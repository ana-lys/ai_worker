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

class DynamicReplay : public rclcpp::Node
{
public:
  DynamicReplay() : Node("collision_checker_publisher")
  {
    declare_parameter("use_gui", true);
    declare_parameter("gui_update_rate", 20.0);
    declare_parameter("simulation_mode", false);
    
    use_gui_ = get_parameter("use_gui").as_bool();
    gui_update_rate_ = get_parameter("gui_update_rate").as_double();
    simulation_mode_ = get_parameter("simulation_mode").as_bool();
    
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
      std::bind(&DynamicReplay::jointStateCallback, this, std::placeholders::_1));
      
    leader_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/leader/joint_states", 10,
      std::bind(&DynamicReplay::leaderJointStateCallback, this, std::placeholders::_1));
      
    // Create publisher for simulated state
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states_ref", 10);

    // Create GUI update timer if GUI is enabled
    if (use_gui_) {
      auto period = std::chrono::duration<double>(1.0 / gui_update_rate_);
      gui_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&DynamicReplay::guiUpdateCallback, this));
    }

    RCLCPP_INFO(get_logger(), "Collision checker publisher ready!");
    if (use_gui_) {
      RCLCPP_INFO(get_logger(), "GUI update rate: %.1f Hz", gui_update_rate_);
    }
  }

  bool useGui() const { return use_gui_; }

  ~DynamicReplay()
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
  
  // Visualization data
  bool use_gui_ = false;
  double gui_update_rate_ = 10.0;
  bool simulation_mode_ = false;
  bool sim_initialized_ = false;

  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr gui_timer_;
  
  std::vector<double> latest_qpos_;
  std::vector<double> latest_qvel_;
  std::vector<double> latest_effort_;
  std::vector<int> msg_idx_to_qpos_;
  std::vector<int> msg_idx_to_dof_;
  std::vector<int> leader_msg_idx_to_actuator_;

  std::vector<std::pair<int, int>> mimic_pos_adr_pairs_;
  std::vector<std::pair<int, int>> mimic_dof_adr_pairs_;
  std::vector<std::string> joint_adr_to_name_;

  bool state_map_initialized_ = false;
  bool leader_state_map_initialized_ = false;

  std::map<std::string, int> joint_name_to_qpos_adr_;
  std::map<std::string, int> joint_name_to_dof_adr_;
  std::map<std::string, int> joint_name_to_actuator_id_;
  
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
      // Disable warmstart to prevent history from affecting "teleportation" steps
      mj_model_->opt.disableflags |= mjDSBL_WARMSTART;
      
      // Fixed 100Hz
      mj_model_->opt.timestep = 0.01;

      // Build joint name to qpos address map
      for (int i = 0; i < mj_model_->njnt; ++i) {
        const char* name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
        if (name) {
          joint_name_to_qpos_adr_[std::string(name)] = mj_model_->jnt_qposadr[i];
          joint_name_to_dof_adr_[std::string(name)] = mj_model_->jnt_dofadr[i];
        }
      }

      // Map joint names to actuator IDs
      joint_name_to_actuator_id_.clear();
      for (int i = 0; i < mj_model_->nu; ++i) {
        int trnid = mj_model_->actuator_trnid[2*i];
        int trntype = mj_model_->actuator_trntype[i];
        if (trntype == mjTRN_JOINT) {
           const char* name = mj_id2name(mj_model_, mjOBJ_JOINT, trnid);
           if (name) {
             joint_name_to_actuator_id_[std::string(name)] = i;
           }
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
      mimic_dof_adr_pairs_.clear();

      // Parse equality constraints for mimic joints
      for (int i = 0; i < mj_model_->neq; ++i) {
        if (mj_model_->eq_type[i] == mjEQ_JOINT) {
          int slave_id = mj_model_->eq_obj1id[i];
          int master_id = mj_model_->eq_obj2id[i];
          int slave_pos_adr = mj_model_->jnt_qposadr[slave_id];
          int master_pos_adr = mj_model_->jnt_qposadr[master_id];
          int slave_dof_adr = mj_model_->jnt_dofadr[slave_id];
          int master_dof_adr = mj_model_->jnt_dofadr[master_id];

          mimic_pos_adr_pairs_.emplace_back(slave_pos_adr, master_pos_adr);
          mimic_dof_adr_pairs_.emplace_back(slave_dof_adr, master_dof_adr);
          
          const char* slave_name = mj_id2name(mj_model_, mjOBJ_JOINT, slave_id);
          const char* master_name = mj_id2name(mj_model_, mjOBJ_JOINT, master_id);
          RCLCPP_INFO(get_logger(), "Mimic joint found: %s follows %s", 
                      slave_name ? slave_name : "unknown",
                      master_name ? master_name : "unknown");
        }
      }
      
      mj_data_ = mj_makeData(mj_model_);
      // Initialize latest_qpos_ with default values
      latest_qpos_.resize(mj_model_->nq, 0.0);
      latest_qvel_.resize(mj_model_->nv, 0.0);
      latest_effort_.resize(mj_model_->nv, 0.0);

      if (mj_model_->qpos0) {
        for(int i = 0; i < mj_model_->nq; ++i) {
          latest_qpos_[i] = mj_model_->qpos0[i];
        }
      }   

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
    mjv_updateScene(mj_model_, mj_data_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
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
      msg_idx_to_dof_.clear();
      msg_idx_to_dof_.resize(msg->name.size(), -1);
      leader_msg_idx_to_actuator_.clear();
      leader_msg_idx_to_actuator_.resize(msg->name.size(), -1);
      
      for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it_q = joint_name_to_qpos_adr_.find(msg->name[i]);
        if (it_q != joint_name_to_qpos_adr_.end()) {
          msg_idx_to_qpos_[i] = it_q->second;
        }

        auto it_d = joint_name_to_dof_adr_.find(msg->name[i]);
        if (it_d != joint_name_to_dof_adr_.end()) {
          msg_idx_to_dof_[i] = it_d->second;
        }
        
        auto it_a = joint_name_to_actuator_id_.find(msg->name[i]);
        if (it_a != joint_name_to_actuator_id_.end()) {
          leader_msg_idx_to_actuator_[i] = it_a->second;
        }
        
        RCLCPP_INFO(get_logger(), "Mapping joint %s: qpos_adr=%d qvel_adr=%d",
                    msg->name[i].c_str(),
                    msg_idx_to_qpos_[i],
                    msg_idx_to_dof_[i]);
      }
      state_map_initialized_ = true;
    }

    // Update positions,velocities,efforts
    for (size_t i = 0; i < msg->position.size(); ++i) {
      if (i < msg_idx_to_qpos_.size()) {
        int qpos_adr = msg_idx_to_qpos_[i];
        if (qpos_adr >= 0 && qpos_adr < (int)latest_qpos_.size()) {
          latest_qpos_[qpos_adr] = msg->position[i];
        }
      }
    }

    for (size_t i = 0; i < msg->velocity.size(); ++i) {
      if (i < msg_idx_to_dof_.size()) {
        int dof_adr = msg_idx_to_dof_[i];
        if (dof_adr >= 0 && dof_adr < (int)latest_qvel_.size()) {
          latest_qvel_[dof_adr] = msg->velocity[i];
        }
      }
    }

    for (size_t i = 0; i < msg->effort.size(); ++i) {
      if (i < msg_idx_to_dof_.size()) {
        int dof_adr = msg_idx_to_dof_[i];
        if (dof_adr >= 0 && dof_adr < (int)latest_effort_.size()) {
          latest_effort_[dof_adr] = msg->effort[i];
        }
      }
    }

    // If NOT in simulation mode, update visualization immediately using teleport
    if (!simulation_mode_) {
       // Update mujoco data for visualization immediately
       for (int i = 0; i < mj_model_->nq; ++i) {
         mj_data_->qpos[i] = latest_qpos_[i];
       }
       for (int i = 0; i < mj_model_->nv; ++i) {
         mj_data_->qvel[i] = latest_qvel_[i];
       }
       // Handle mimic joints
       for (size_t i = 0; i < mimic_pos_adr_pairs_.size(); ++i) {
         mj_data_->qpos[mimic_pos_adr_pairs_[i].first] = mj_data_->qpos[mimic_pos_adr_pairs_[i].second];
       }
       // Forward kinematics/dynamics for visualization
       mj_forward(mj_model_, mj_data_);
    }
  }

  void leaderJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
     if (!mj_model_ || !mj_data_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Initialize leader mapping
    if (!leader_state_map_initialized_) {
      leader_msg_idx_to_actuator_.clear();
      leader_msg_idx_to_actuator_.resize(msg->name.size(), -1);
      
      for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it_a = joint_name_to_actuator_id_.find(msg->name[i]);
        if (it_a != joint_name_to_actuator_id_.end()) {
          leader_msg_idx_to_actuator_[i] = it_a->second;
        }
        RCLCPP_INFO(get_logger(), "Mapping leader joint %s: actuator_id=%d",
                    msg->name[i].c_str(),
                    leader_msg_idx_to_actuator_[i]);
      }
      leader_state_map_initialized_ = true;
    }
    // 1. Update control from leader (ACTION)
    for (size_t i = 0; i < msg->position.size(); ++i) {
      if (i < leader_msg_idx_to_actuator_.size()) {
        int act_id = leader_msg_idx_to_actuator_[i];
        if (act_id >= 0 && act_id < mj_model_->nu) {
          mj_data_->ctrl[act_id] = msg->position[i];
        }
      }
    }

    if (simulation_mode_) {
       // --- SIMULATION MODE ---
       // Initialize simulation state from robot state ONLY ONCE
       if (!sim_initialized_) {
         for (int i = 0; i < mj_model_->nq; ++i) mj_data_->qpos[i] = latest_qpos_[i];
         for (int i = 0; i < mj_model_->nv; ++i) mj_data_->qvel[i] = latest_qvel_[i];
         // Handle mimics
         for (size_t i = 0; i < mimic_pos_adr_pairs_.size(); ++i) {
           mj_data_->qpos[mimic_pos_adr_pairs_[i].first] = mj_data_->qpos[mimic_pos_adr_pairs_[i].second];
         }
         for (size_t i = 0; i < mimic_dof_adr_pairs_.size(); ++i) {
           mj_data_->qvel[mimic_dof_adr_pairs_[i].first] = mj_data_->qvel[mimic_dof_adr_pairs_[i].second];
         }
         sim_initialized_ = true;
         RCLCPP_INFO(get_logger(), "Simulation initialized from robot state.");
       }
       
       // Step simulation forward
       mj_step(mj_model_, mj_data_);
       
    } else {
      // --- TELEPORT MODE (Original Behavior) ---
      
      // Override State from latest robot state
      for (int i = 0; i < mj_model_->nq; ++i) {
        mj_data_->qpos[i] = latest_qpos_[i];
      }
      for (int i = 0; i < mj_model_->nv; ++i) {
        mj_data_->qvel[i] = latest_qvel_[i];
      }
      
      // Handle mimic joints
      for (size_t i = 0; i < mimic_pos_adr_pairs_.size(); ++i) {
        mj_data_->qpos[mimic_pos_adr_pairs_[i].first] = mj_data_->qpos[mimic_pos_adr_pairs_[i].second];
      }
      for (size_t i = 0; i < mimic_dof_adr_pairs_.size(); ++i) {
        mj_data_->qvel[mimic_dof_adr_pairs_[i].first] = mj_data_->qvel[mimic_dof_adr_pairs_[i].second];
      }

      // Forward Dynamics (No Time Integration)
      mj_forward(mj_model_, mj_data_);
    }


    // 4. Publish
    sensor_msgs::msg::JointState ref_msg;
    ref_msg.header.stamp = this->now();
    for (int i = 0; i < mj_model_->njnt; ++i) {
       const char* name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
       if (name) {
         ref_msg.name.push_back(name);
         int qpos_adr = mj_model_->jnt_qposadr[i];
         int dof_adr = mj_model_->jnt_dofadr[i];
         
         ref_msg.position.push_back(mj_data_->qpos[qpos_adr]);
         ref_msg.velocity.push_back(mj_data_->qvel[dof_adr]);
         ref_msg.effort.push_back(mj_data_->qfrc_actuator[dof_adr]);
       }
    }
    joint_state_pub_->publish(ref_msg);
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicReplay>();
  node->init();

  node->spin();

  rclcpp::shutdown();
  return 0;
}