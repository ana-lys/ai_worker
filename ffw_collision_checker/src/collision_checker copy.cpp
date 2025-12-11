#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/collision_detection/collision_tools.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "ffw_collision_checker/srv/check_collision.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>

std::string loadFile1(const std::string& path)
  {
    std::ifstream f(path);
    if (!f.is_open())
      throw std::runtime_error("Failed to open file: " + path);
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  }

class CollisionCheckerServer : public rclcpp::Node
{
public:
  CollisionCheckerServer() : Node("collision_checker_server")
  {
    loadRobotDescription();   // <-- MUST happen before planning scene creation
    init();
  }

private:

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  rclcpp::Service<ffw_collision_checker::srv::CheckCollision>::SharedPtr service_;

  void init()
  {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(),
        "robot_description",
        "robot_description_semantic");

    if (!psm_->getPlanningScene())
    {
      RCLCPP_ERROR(get_logger(), "PlanningSceneMonitor failed to load planning scene!");
      return;
    }

    psm_->startSceneMonitor();
    psm_->startStateMonitor();
    psm_->startWorldGeometryMonitor();

    service_ = this->create_service<ffw_collision_checker::srv::CheckCollision>(
        "check_collision",
        std::bind(&CollisionCheckerServer::callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Collision checker ready!");
  }

  void callback(
      const std::shared_ptr<ffw_collision_checker::srv::CheckCollision::Request> req,
      std::shared_ptr<ffw_collision_checker::srv::CheckCollision::Response> res)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    if (!scene)
    {
      RCLCPP_ERROR(get_logger(), "Failed to lock planning scene.");
      res->in_collision = true;
      res->min_distance = 0.0;
      return;
    }

    moveit::core::RobotState robot_state(scene->getCurrentState());
    const auto model = robot_state.getRobotModel();

    for (size_t i = 0; i < req->joint_names.size(); ++i)
    {
      if (!model->hasJointModel(req->joint_names[i]))
      {
        RCLCPP_WARN(get_logger(), "Joint %s not in robot model", req->joint_names[i].c_str());
        continue;
      }
      robot_state.setJointPositions(req->joint_names[i], {req->joint_positions[i]});
    }

    robot_state.update();

    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    creq.distance = true;
    creq.contacts  = true;

    scene->checkSelfCollision(creq, cres, robot_state);

    res->in_collision = cres.collision;
    res->min_distance = cres.distance;

    RCLCPP_INFO(get_logger(), "Collision=%d | min_dist=%.4f",
                res->in_collision, res->min_distance);
  }

  void loadRobotDescription()
  {
    using namespace ament_index_cpp;

    const std::string desc_pkg   = get_package_share_directory("ffw_description");
    const std::string moveit_pkg = get_package_share_directory("ffw_moveit_config");

    const std::string urdf_path = desc_pkg + "/urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf";
    const std::string srdf_path = moveit_pkg + "/config/ffw.srdf";

    this->declare_parameter("robot_description", loadFile(urdf_path));
    this->declare_parameter("robot_description_semantic", loadFile(srdf_path));

    RCLCPP_INFO(get_logger(), "Loaded URDF: %s", urdf_path.c_str());
    RCLCPP_INFO(get_logger(), "Loaded SRDF: %s", srdf_path.c_str());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CollisionCheckerServer>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
