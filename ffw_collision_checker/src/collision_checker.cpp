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

// Custom message for collision check results
// You'll need to create this message type in your package:
// bool in_collision
// float64[] distances
// string[] geom1_names
// string[] geom2_names
#include <ffw_collision_checker/msg/collision_check.hpp>

class CollisionCheckerPublisher : public rclcpp::Node
{
public:
  CollisionCheckerPublisher() : Node("collision_checker_publisher")
  {
    declare_parameter("use_gui", false);
    
    use_gui_ = get_parameter("use_gui").as_bool();
    
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
      "/collision_check", 10);

    RCLCPP_INFO(get_logger(), "Collision checker publisher ready!");
  }

  bool useGui() const { return use_gui_; }

  void spinGui()
  {
    if (!use_gui_ || !window_) return;
    
    while (!glfwWindowShouldClose(window_) && rclcpp::ok()) {
      glfwPollEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

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

    // Main thread handles GLFW events and rendering
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
  bool collision_solver = false;

  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<ffw_collision_checker::msg::CollisionCheck>::SharedPtr collision_pub_;
  
  std::vector<double> latest_qpos_;
  std::vector<double> latest_qvel_;

  std::vector<int> msg_idx_to_qpos_;
  std::vector<int> msg_idx_to_qvel_;

  std::vector<std::pair<int, int>> mimic_pos_adr_pairs_;
  std::vector<std::pair<int, int>> mimic_vel_adr_pairs_;

  bool state_map_initialized_ = false;

  std::map<std::string, int> joint_name_to_qpos_adr_;
  std::map<std::string, int> joint_name_to_qvel_adr_;

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
          joint_name_to_qvel_adr_[std::string(name)] = mj_model_->jnt_dofadr[i];
        }
      }
      
      mimic_pos_adr_pairs_.clear();
      mimic_vel_adr_pairs_.clear();
      
      // Parse equality constraints for mimic joints
      for (int i = 0; i < mj_model_->neq; ++i) {
        if (mj_model_->eq_type[i] == mjEQ_JOINT) {
          int slave_id = mj_model_->eq_obj1id[i];
          int master_id = mj_model_->eq_obj2id[i];
          int slave_pos_adr = mj_model_->jnt_qposadr[slave_id];
          int master_pos_adr = mj_model_->jnt_qposadr[master_id];

          int slave_vel_adr = mj_model_->jnt_dofadr[slave_id];
          int master_vel_adr = mj_model_->jnt_dofadr[master_id];

          mimic_pos_adr_pairs_.emplace_back(slave_pos_adr, master_pos_adr);
          mimic_vel_adr_pairs_.emplace_back(slave_vel_adr, master_vel_adr);
          
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
      // Initialize latest_qvel_
      latest_qvel_.resize(mj_model_->nv, 0.0);

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

  void updateGui( bool collision_solver)
  {
    if (!use_gui_ || !window_) return;

    // Get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // Update scene and render
    if (!collision_solver){
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
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                           "MuJoCo model not loaded");
      return;
    }

    // Initialize mapping on first message
    if (!state_map_initialized_) {
      msg_idx_to_qpos_.clear();
      msg_idx_to_qvel_.clear();
      msg_idx_to_qpos_.resize(msg->name.size(), -1);
      msg_idx_to_qvel_.resize(msg->name.size(), -1);
      
      for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it_q = joint_name_to_qpos_adr_.find(msg->name[i]);
        if (it_q != joint_name_to_qpos_adr_.end()) {
          msg_idx_to_qpos_[i] = it_q->second;
        }
        
        auto it_d = joint_name_to_qvel_adr_.find(msg->name[i]);
        if (it_d != joint_name_to_qvel_adr_.end()) {
          msg_idx_to_qvel_[i] = it_d->second;
        }
        RCLCPP_INFO(get_logger(), "Mapping joint %s: qpos_adr=%d, qvel_adr=%d",
                    msg->name[i].c_str(),
                    msg_idx_to_qpos_[i],
                    msg_idx_to_qvel_[i]);
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

    // Update velocities
    for (size_t i = 0; i < msg->velocity.size(); ++i) {
      if (i < msg_idx_to_qvel_.size()) {
        int dof_adr = msg_idx_to_qvel_[i];
        if (dof_adr >= 0 && dof_adr < (int)latest_qvel_.size()) {
          latest_qvel_[dof_adr] = msg->velocity[i];
        }
      }
    }

    // Update qpos and qvel in mujoco data
    for (int i = 0; i < mj_model_->nq; ++i) {
      mj_data_->qpos[i] = latest_qpos_[i];
    }
    for (int i = 0; i < mj_model_->nv; ++i) {
      mj_data_->qvel[i] = latest_qvel_[i];
    }

    // Handle mimic joints
    for (size_t i = 0; i < mimic_pos_adr_pairs_.size(); ++i) {
      int slave_pos_adr = mimic_pos_adr_pairs_[i].first;
      int master_pos_adr = mimic_pos_adr_pairs_[i].second;
      mj_data_->qpos[slave_pos_adr] = mj_data_->qpos[master_pos_adr];

      int slave_vel_adr = mimic_vel_adr_pairs_[i].first;
      int master_vel_adr = mimic_vel_adr_pairs_[i].second;
      mj_data_->qvel[slave_vel_adr] = mj_data_->qvel[master_vel_adr];
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
    collision_solver = collision_msg.in_collision;
    if (!collision_solver){
      mj_data_solver_ = mj_data_;
    }
    // Update GUI if enabled
    if (use_gui_) {
      updateGui(collision_solver);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionCheckerPublisher>();
  node->init();

  // This single line handles BOTH cases cleanly
  node->spin();

  rclcpp::shutdown();
  return 0;
}