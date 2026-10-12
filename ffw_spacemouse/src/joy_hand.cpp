// joy_hand: SpaceMouse EE teleop with goals in base_link (unified launch).
#include "ffw_spacemouse/ee_teleop_base.hpp"

class BaseLinkTeleop : public EeTeleopBase
{
public:
  BaseLinkTeleop() : EeTeleopBase("base_link") {}

protected:
  // Goals are already in the solver's base frame.
  bool lookup_base_from_native(Eigen::Isometry3d &T_base_native) override
  {
    T_base_native.setIdentity();
    return true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseLinkTeleop>());
  rclcpp::shutdown();
  return 0;
}
