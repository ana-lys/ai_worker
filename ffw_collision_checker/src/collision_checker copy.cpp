#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/collision_detection/collision_tools.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "ffw_collision_checker/srv/check_collision.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <regex>
#include <fstream>
#include <sstream>

class CollisionCheckerServer : public rclcpp::Node
{
public:
  CollisionCheckerServer() : Node("collision_checker_server")
  {
    loadRobotDescription();   // <-- MUST happen before planning scene creation
  }

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

private:

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  rclcpp::Service<ffw_collision_checker::srv::CheckCollision>::SharedPtr service_;

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
  creq.contacts = true;
  creq.max_contacts = 10;  // Get up to 10 contact points
  creq.max_contacts_per_pair = 1;  // One contact per link pair

  scene->checkSelfCollision(creq, cres, robot_state);

  res->in_collision = cres.collision;
  res->min_distance = cres.distance;

  // Log collision status
  if (cres.collision)
  {
    RCLCPP_WARN(get_logger(), "COLLISION DETECTED!");
    
    // Print all contact pairs
    for (const auto& contact_pair : cres.contacts)
    {
      const std::string& link1 = contact_pair.first.first;
      const std::string& link2 = contact_pair.first.second;
      
      for (const auto& contact : contact_pair.second)
      {
        RCLCPP_WARN(get_logger(), "  Collision: %s <-> %s | depth: %.4f",
                    link1.c_str(), link2.c_str(), contact.depth);
      }
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "No collision | min_dist=%.4f", res->min_distance);
    
    // Find closest link pair (even when not colliding)
    // For distance queries, we need to check collision detector's distance field
    collision_detection::CollisionRequest dist_req;
    collision_detection::CollisionResult dist_res;
    dist_req.distance = true;
    dist_req.contacts = false;
    
    scene->getCollisionEnv()->checkSelfCollision(dist_req, dist_res, robot_state,
                                                   scene->getAllowedCollisionMatrix());
    
    // Get nearest points info if available
    if (dist_res.distance > 0.0)
    {
      // Unfortunately, MoveIt doesn't directly expose which links are closest
      // when not in collision. We can estimate by checking contacts at very small threshold
      collision_detection::CollisionRequest near_req;
      collision_detection::CollisionResult near_res;
      near_req.distance = true;
      near_req.contacts = true;
      near_req.max_contacts = 1;
      near_req.max_contacts_per_pair = 1;
      
      // Temporarily modify ACM to get nearest pair info
      collision_detection::AllowedCollisionMatrix acm_temp = scene->getAllowedCollisionMatrix();
      acm_temp.setDefaultEntry("", false);  // Allow all collision checks temporarily
      
      scene->getCollisionEnv()->checkSelfCollision(near_req, near_res, robot_state, acm_temp);
      
      if (!near_res.contacts.empty())
      {
        const auto& closest_pair = *near_res.contacts.begin();
        const std::string& link1 = closest_pair.first.first;
        const std::string& link2 = closest_pair.first.second;
        
        RCLCPP_INFO(get_logger(), "  Closest pair: %s <-> %s | distance: %.4f",
                    link1.c_str(), link2.c_str(), dist_res.distance);
      }
    }
  }
}

  std::string loadFile(const std::string &path)
  {
    std::ifstream file(path);
    if (!file.is_open())
      throw std::runtime_error("Cannot open: " + path);

    std::stringstream ss;
    ss << file.rdbuf();
    return ss.str();
  }

 // Remove SRDF entries referencing joints/links not in URDF
std::string sanitizeSRDF(const std::string &srdf, const std::string &urdf)
{
  std::string cleaned = srdf;

  // Extract robot name from URDF
  std::smatch m;
  std::regex_search(urdf, m, std::regex(R"delim(<robot\s+name="([^"]+)")delim"));
  std::string real_name = m[1];

  // Force SRDF robot name to match
  cleaned = std::regex_replace(cleaned, std::regex(R"delim(<robot\s+name="[^"]+">)delim"),
                               "<robot name=\"" + real_name + "\">");

  // Extract all valid joint names from URDF
  std::set<std::string> valid_joints;
  std::regex joint_regex(R"delim(<joint\s+name="([^"]+)")delim");
  auto joints_begin = std::sregex_iterator(urdf.begin(), urdf.end(), joint_regex);
  auto joints_end = std::sregex_iterator();
  for (std::sregex_iterator i = joints_begin; i != joints_end; ++i) {
    valid_joints.insert((*i)[1]);
  }

  // Extract all valid link names from URDF
  std::set<std::string> valid_links;
  std::regex link_regex(R"delim(<link\s+name="([^"]+)")delim");
  auto links_begin = std::sregex_iterator(urdf.begin(), urdf.end(), link_regex);
  auto links_end = std::sregex_iterator();
  for (std::sregex_iterator i = links_begin; i != links_end; ++i) {
    valid_links.insert((*i)[1]);
  }

  // Remove invalid joints from groups
  std::regex group_joint_regex(R"delim(<joint\s+name="([^"]+)"\s*/>)delim");
  std::string temp;
  auto begin = std::sregex_iterator(cleaned.begin(), cleaned.end(), group_joint_regex);
  auto end = std::sregex_iterator();
  size_t last_pos = 0;
  for (std::sregex_iterator i = begin; i != end; ++i) {
    temp += cleaned.substr(last_pos, i->position() - last_pos);
    std::string joint_name = (*i)[1];
    if (valid_joints.count(joint_name)) {
      temp += i->str();  // Keep valid joint
    }
    last_pos = i->position() + i->length();
  }
  temp += cleaned.substr(last_pos);
  cleaned = temp;

  // Remove invalid end_effector entries
  cleaned = std::regex_replace(cleaned,
                               std::regex(R"delim(<end_effector\s+name="[^"]+"\s+parent_link="[^"]+"\s+group="[^"]+"\s*/>)delim"),
                               "");

  // Remove invalid disable_collisions with links not in URDF
  std::regex disable_collision_regex(R"delim(<disable_collisions\s+link1="([^"]+)"\s+link2="([^"]+)"\s+reason="[^"]+"\s*/>)delim");
  temp.clear();
  begin = std::sregex_iterator(cleaned.begin(), cleaned.end(), disable_collision_regex);
  last_pos = 0;
  for (std::sregex_iterator i = begin; i != end; ++i) {
    temp += cleaned.substr(last_pos, i->position() - last_pos);
    std::string link1 = (*i)[1];
    std::string link2 = (*i)[2];
    if (valid_links.count(link1) && valid_links.count(link2)) {
      temp += i->str();  // Keep valid collision pair
    }
    last_pos = i->position() + i->length();
  }
  temp += cleaned.substr(last_pos);
  cleaned = temp;

  return cleaned;
}

  void loadRobotDescription()
  {
    using ament_index_cpp::get_package_share_directory;

    std::string desc_pkg = get_package_share_directory("ffw_description");
    std::string moveit_pkg = get_package_share_directory("ffw_moveit_config");

    std::string urdf_path = desc_pkg + "/urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf";
    std::string srdf_path = moveit_pkg + "/config/ffw.srdf";

    std::string urdf = loadFile(urdf_path);
    std::string srdf = sanitizeSRDF(loadFile(srdf_path), urdf);

    declare_parameter("robot_description", urdf);
    declare_parameter("robot_description_semantic", srdf);

    RCLCPP_INFO(get_logger(), "Loaded URDF: %s", urdf_path.c_str());
    RCLCPP_INFO(get_logger(), "Loaded SRDF (sanitized): %s", srdf_path.c_str());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CollisionCheckerServer>();
  node->init();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
