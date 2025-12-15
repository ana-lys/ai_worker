#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <ffw_collision_checker/srv/check_collision.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>

class CollisionCheckerServer : public rclcpp::Node
{
public:
  CollisionCheckerServer() : Node("collision_checker_server")
  {
    declare_parameter("use_gui", false);
    use_gui_ = get_parameter("use_gui").as_bool();
    
    loadMujocoModel("robotis_ffw/scene.xml");
  }

  void init()
  {
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CollisionCheckerServer::jointStateCallback, this, std::placeholders::_1));

    // Create service
    service_ = this->create_service<ffw_collision_checker::srv::CheckCollision>(
        "check_collision",
        std::bind(&CollisionCheckerServer::serviceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Collision checker ready!");
  }

  bool useGui() const { return use_gui_; }

  void runGuiLoop()
  {
    if (!use_gui_ || !mj_model_) return;

    if (!glfwInit()) {
      RCLCPP_ERROR(get_logger(), "Could not initialize GLFW");
      return;
    }

    window_ = glfwCreateWindow(1200, 900, "Collision Checker", NULL, NULL);
    if (!window_) {
      glfwTerminate();
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
    
    while (!glfwWindowShouldClose(window_) && rclcpp::ok()) {
      
      // Get framebuffer viewport
      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        mj_step(mj_model_, mj_data_);
        // Update scene and render
        mjv_updateScene(mj_model_, mj_data_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
      }

      mjr_render(viewport, &scn_, &con_);

      glfwSwapBuffers(window_);
      glfwPollEvents();
    }
    
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    glfwDestroyWindow(window_);
    glfwTerminate();
  }

  ~CollisionCheckerServer()
  {
    if (mj_data_) mj_deleteData(mj_data_);
    if (mj_model_) mj_deleteModel(mj_model_);
  }

private:
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;
  
  // Visualization data
  bool use_gui_ = false;
  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<ffw_collision_checker::srv::CheckCollision>::SharedPtr service_;
  
  std::vector<double> latest_qpos_;
  std::vector<double> latest_qvel_;

  std::vector<int> msg_idx_to_qpos_;
  std::vector<int> msg_idx_to_qvel_;

  std::vector<int> req_idex_to_actuator_;

  std::vector<std::pair<int ,int>> mimic_pos_adr_pairs_;
  std::vector<std::pair<int ,int>> mimic_vel_adr_pairs_;

  bool state_map_initialized_ = false;
  bool actuator_map_initialized_ = false;

  std::map<std::string, int> joint_name_to_qpos_adr_;
  std::map<std::string, int> joint_name_to_qvel_adr_;
  std::map<std::string, int> joint_name_to_actuator_adr_;

  std::mutex joint_mutex_;

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
        RCLCPP_INFO(get_logger(), "Mimic joint found: slave=%d master=%d", slave_id, master_id);
      }
    }
    // Clear the map before populating
    joint_name_to_actuator_adr_.clear();

    RCLCPP_INFO(get_logger(), "Model has %d actuators", mj_model_->nu);

    // Iterate over all actuators (from 0 to nu-1)
    for (int i = 0; i < mj_model_->nu; ++i) {
        // Check the transmission type
        int trn_type = mj_model_->actuator_trntype[i];

        if (trn_type == mjTRN_JOINT) {  // mjTRN_JOINT = 0
            // For joint transmission, trnid[0] is the joint ID
            int joint_id = mj_model_->actuator_trnid[i*2];
            const char* joint_name = mj_id2name(mj_model_, mjOBJ_JOINT, joint_id);

            if (joint_name) {
                joint_name_to_actuator_adr_[std::string(joint_name)] = i; 
                RCLCPP_INFO(get_logger(), "Actuator %d controls joint %s (id=%d)", 
                           i, joint_name, joint_id);          
            }
        } else {
            RCLCPP_INFO(get_logger(), "Actuator %d has transmission type %d (not direct joint)", 
                       i, trn_type);
        }
    }
    
    mj_data_ = mj_makeData(mj_model_);

    // Initialize latest_qpos_ with default values
    latest_qpos_.resize(mj_model_->nq, 0.0);
    if (mj_model_->qpos0) {
      for(int i=0; i<mj_model_->nq; ++i) latest_qpos_[i] = mj_model_->qpos0[i];
    }
    // Initialize latest_qvel_
    latest_qvel_.resize(mj_model_->nv, 0.0);


    RCLCPP_INFO(get_logger(), "MuJoCo model loaded successfully from: %s", mujoco_xml_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to load MuJoCo model: %s", error);
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
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
      }
      state_map_initialized_ = true;
    }

    std::lock_guard<std::mutex> lock(joint_mutex_);
    
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
  }
  void serviceCallback(
    const std::shared_ptr<ffw_collision_checker::srv::CheckCollision::Request> req,
    std::shared_ptr<ffw_collision_checker::srv::CheckCollision::Response> res)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (!mj_model_ || !mj_data_) {
      RCLCPP_ERROR(get_logger(), "MuJoCo model not loaded");
      res->in_collision = true;
      return;
    }

    // // Build request index to actuator address map
    if (!actuator_map_initialized_) {
      req_idex_to_actuator_.clear();
      req_idex_to_actuator_.resize(req->joint_names.size(), -1);
      for (size_t i = 0; i < req->joint_names.size(); ++i) {
        req_idex_to_actuator_[i] = -1;
        auto it = joint_name_to_actuator_adr_.find(req->joint_names[i]);
        if (it != joint_name_to_actuator_adr_.end()) {
           if (it->second < mj_model_->nu) {
              req_idex_to_actuator_[i] = it->second;
              RCLCPP_INFO(get_logger(), "Mapping request joint %s to actuator %d , request index %zu", req->joint_names[i].c_str(), it->second, i);
           }
        } else {
          RCLCPP_WARN(get_logger(), "Joint %s not found in MuJoCo Actuator model", req->joint_names[i].c_str());
          res->in_collision = true;
          return;
        }
      }
      actuator_map_initialized_ = true;
    }
    // // Update qpos from current joint states
    // {
    //   std::lock_guard<std::mutex> lock(joint_mutex_);
    //   for (int i = 0; i < mj_model_->nq; ++i) {
    //     mj_data_->qpos[i] = latest_qpos_[i];
    //   }
    //   for (int i = 0; i < mj_model_->nv; ++i) {
    //     mj_data_->qvel[i] = latest_qvel_[i];
    //   }
    // }
    // RCLCPP_INFO(get_logger(), "Checking mimics pos %lf joints", mj_data_->qpos[mimic_pos_adr_pairs_[0].second]);
    // mj_data_->qpos[mimic_pos_adr_pairs_[0].first] = -mj_data_->qpos[mimic_pos_adr_pairs_[0].second];
    // Handle mimic joinsts
    // for ( int i = 0; i < mimic_pos_adr_pairs_.size(); ++i) {
    //     int slave_pos_adr = mimic_pos_adr_pairs_[i].first;
    //     int master_pos_adr = mimic_pos_adr_pairs_[i].second;
    //     mj_data_->qpos[slave_pos_adr] = mj_data_->qpos[master_pos_adr];

    //     int slave_vel_adr = mimic_vel_adr_pairs_[i].first;
    //     int master_vel_adr = mimic_vel_adr_pairs_[i].second;
    //     mj_data_->qvel[slave_vel_adr] = mj_data_->qvel[master_vel_adr];   
    // }
    // Set actuator controls from request
    for (size_t i = 0; i < req->joint_positions.size(); ++i) {
      if (i < req_idex_to_actuator_.size()) {
        int actuator_adr = req_idex_to_actuator_[i];
        if (actuator_adr >= 0 && actuator_adr < mj_model_->nu) {
          // Set control input for the actuator
          mj_data_->ctrl[actuator_adr] = req->joint_positions[i];
        }
      }
    }

    // Forward kinematics and collision
    mj_forward(mj_model_, mj_data_);

    // Check collisions
    bool actual_collision = false;
    double min_dist = 1000.0;

    if (mj_data_->ncon > 0) {
      for (int i = 0; i < mj_data_->ncon; ++i) {
        double dist = mj_data_->contact[i].dist;
        if (dist < min_dist) {
            min_dist = dist;
        }
        if (dist < 0) {
            actual_collision = true;
            
            const char* geom1 = mj_id2name(mj_model_, mjOBJ_GEOM, mj_data_->contact[i].geom1);
            const char* geom2 = mj_id2name(mj_model_, mjOBJ_GEOM, mj_data_->contact[i].geom2);
            
            RCLCPP_WARN(get_logger(), "Collision: %s <-> %s | dist: %.4f",
                        geom1 ? geom1 : "unknown",
                        geom2 ? geom2 : "unknown",
                        dist);
        }
      }
    }
    
    res->in_collision = actual_collision;
    res->min_distance = min_dist;
    
    // if (!actual_collision) {
    //     RCLCPP_INFO(get_logger(), "No collision | min_dist=%.4f", min_dist);
    // }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionCheckerServer>();
  node->init();

  if (node->useGui()) {
    // Run ROS spin in a separate thread
    std::thread spin_thread([node]() {
      rclcpp::spin(node);
    });
    
    // Run GUI loop in main thread
    node->runGuiLoop();
    
    // When GUI closes, shutdown ROS
    rclcpp::shutdown();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
  } else {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  
  return 0;
}
