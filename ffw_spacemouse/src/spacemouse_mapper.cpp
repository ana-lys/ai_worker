#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class SpaceMouseMapper : public rclcpp::Node
{
public:
  SpaceMouseMapper() : Node("spacemouse_mapper")
  {
    this->declare_parameter("pos_step", 0.005);
    this->declare_parameter("rot_step", 0.05); // radians
    this->declare_parameter("publish_rate_hz", 100.0);
    this->declare_parameter("trans_sensitivity", 1.5);
    this->declare_parameter("rot_sensitivity", 1.0);
    
    // Axes mapping for SpaceMouse
    this->declare_parameter("reference_frame", "global"); // "global" or "local"
    this->declare_parameter("axis_x", 1); 
    this->declare_parameter("axis_y", 0); 
    this->declare_parameter("axis_z", 2); 
    this->declare_parameter("axis_roll", 3);  
    this->declare_parameter("axis_pitch", 4); 
    this->declare_parameter("axis_yaw", 5);   

    // Invert flags
    this->declare_parameter("invert_x", false);
    this->declare_parameter("invert_y", false);
    this->declare_parameter("invert_z", false);
    this->declare_parameter("invert_roll", false);
    this->declare_parameter("invert_pitch", false);
    this->declare_parameter("invert_yaw", false);
    this->declare_parameter("target_arm", "arm");
    target_arm_ = this->get_parameter("target_arm").as_string();

    std::string topic_name = "/spacemouse/" + target_arm_ + "/ee_target_pose";
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&SpaceMouseMapper::joy_callback, this, _1));

    // Initialize pose at origin
    ee_goal_ = Eigen::Isometry3d::Identity();

    const double rate = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    command_dt_ = 1.0 / rate;
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(command_dt_)),
      std::bind(&SpaceMouseMapper::control_timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "SpaceMouse pose mapper started. Publishing to /spacemouse/ee_target_pose");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    latest_joy_ = *msg;
    joy_received_ = true;
  }

  void control_timer_callback()
  {
    sensor_msgs::msg::Joy joy;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      if (!joy_received_) return;
      joy = latest_joy_;
    }

    double pos_step = this->get_parameter("pos_step").as_double();
    double rot_step = this->get_parameter("rot_step").as_double();

    auto get_axis = [&joy](int index, bool invert) -> double {
      if (index >= 0 && index < static_cast<int>(joy.axes.size())) {
        double val = joy.axes[index];
        return invert ? -val : val;
      }
      return 0.0;
    };

    Eigen::Vector3d trans_raw(
      get_axis(this->get_parameter("axis_x").as_int(), this->get_parameter("invert_x").as_bool()),
      get_axis(this->get_parameter("axis_y").as_int(), this->get_parameter("invert_y").as_bool()),
      get_axis(this->get_parameter("axis_z").as_int(), this->get_parameter("invert_z").as_bool())
    );

    double trans_sens = this->get_parameter("trans_sensitivity").as_double();
    trans_raw *= trans_sens;
    double trans_norm = trans_raw.norm();
    if (trans_norm > 1.0) {
      trans_raw /= trans_norm;
      trans_norm = 1.0;
    }
    Eigen::Vector3d trans_scaled = trans_raw * (trans_norm * trans_norm); // Cubic magnitude
    double dx = trans_scaled.x();
    double dy = trans_scaled.y();
    double dz = trans_scaled.z();

    Eigen::Vector3d rot_raw(
      get_axis(this->get_parameter("axis_roll").as_int(), this->get_parameter("invert_roll").as_bool()),
      get_axis(this->get_parameter("axis_pitch").as_int(), this->get_parameter("invert_pitch").as_bool()),
      get_axis(this->get_parameter("axis_yaw").as_int(), this->get_parameter("invert_yaw").as_bool())
    );

    double rot_sens = this->get_parameter("rot_sensitivity").as_double();
    rot_raw *= rot_sens;
    double rot_norm = rot_raw.norm();
    if (rot_norm > 1.0) {
      rot_raw /= rot_norm;
      rot_norm = 1.0;
    }
    Eigen::Vector3d rot_scaled = rot_raw * (rot_norm * rot_norm); // Cubic magnitude
    double drx = rot_scaled.x();
    double dry = rot_scaled.y();
    double drz = rot_scaled.z();

    if (joy.axes.size() >= 3) {
      // Print removed to reduce terminal spam
    }

    std::string ref_frame = this->get_parameter("reference_frame").as_string();

    Eigen::Vector3d trans_delta(dx * pos_step, dy * pos_step, dz * pos_step);

    // Apply translation
    if (ref_frame == "local") {
      ee_goal_.translation() += ee_goal_.linear() * trans_delta;
    } else {
      ee_goal_.translation() += trans_delta;
    }
    
    // Map straight to angle axis for quaternion
    Eigen::Vector3d rot_axis(drx, dry, drz);
    double angle = rot_axis.norm() * rot_step;
    
    Eigen::Quaterniond q_delta = Eigen::Quaterniond::Identity();
    if (angle > 1e-6) {
      q_delta = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot_axis.normalized()));
    }

    // Multiply old quat vs this for the new value
    Eigen::Quaterniond q_old(ee_goal_.linear());
    Eigen::Quaterniond q_new;
    
    if (ref_frame == "local") {
      q_new = q_old * q_delta;
    } else {
      q_new = q_delta * q_old;
    }
    
    q_new.normalize();
    ee_goal_.linear() = q_new.toRotationMatrix();

    publish_pose();
  }

  void publish_pose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map"; 
    msg.pose.position.x = ee_goal_.translation().x();
    msg.pose.position.y = ee_goal_.translation().y();
    msg.pose.position.z = ee_goal_.translation().z();
    Eigen::Quaterniond q(ee_goal_.linear());
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pose_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string target_arm_;
  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy latest_joy_;
  bool joy_received_ {false};

  Eigen::Isometry3d ee_goal_;
  double command_dt_ {0.01};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpaceMouseMapper>());
  rclcpp::shutdown();
  return 0;
}
