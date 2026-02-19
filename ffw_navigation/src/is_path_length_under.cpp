#include "ffw_navigation/is_path_length_under.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include <cmath> 
#include <limits>

namespace nav2_behavior_tree
{
IsPathLengthUnder::IsPathLengthUnder(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

BT::PortsList IsPathLengthUnder::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Path to calculate distance for"),
    BT::InputPort<double>("distance_threshold", 1.0, "Distance threshold in meters")
  };
}

BT::NodeStatus IsPathLengthUnder::tick()
{
  nav_msgs::msg::Path path;
  double threshold;

  if (!getInput("path", path)) {
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("distance_threshold", threshold)) {
    return BT::NodeStatus::FAILURE;
  }

  if (path.poses.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  double path_length = 0.0;
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i].pose.position;
    const auto & p2 = path.poses[i+1].pose.position;
    path_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }

  if (path_length <= threshold) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathLengthUnder>("IsPathLengthUnder");
}
