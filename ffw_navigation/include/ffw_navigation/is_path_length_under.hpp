#ifndef FFW_NAVIGATION__IS_PATH_LENGTH_UNDER_HPP_
#define FFW_NAVIGATION__IS_PATH_LENGTH_UNDER_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/path.hpp"

namespace nav2_behavior_tree
{
  class IsPathLengthUnder : public BT::ConditionNode
  {
  public:
    IsPathLengthUnder(const std::string & name, const BT::NodeConfiguration & conf);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
  };
}

#endif
