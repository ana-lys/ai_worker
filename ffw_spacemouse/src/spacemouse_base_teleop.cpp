#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

struct ButtonClickState {
    double last_press_time_sec = 0.0;
    int count = 0;
    bool prev_pressed = false;
};

class SpaceMouseBaseTeleop : public rclcpp::Node
{
public:
  SpaceMouseBaseTeleop() : Node("spacemouse_base_teleop")
  {
    // Left Joy Parameters (Base)
    this->declare_parameter("left_joy_topic", "/spacemouse/left/joy");
    this->declare_parameter("max_linear_vel", 1.0);
    this->declare_parameter("max_angular_vel", 1.5);
    this->declare_parameter("axis_x", 1);
    this->declare_parameter("axis_y", 0);
    this->declare_parameter("axis_yaw", 5);
    this->declare_parameter("invert_x", false);
    this->declare_parameter("invert_y", false);
    this->declare_parameter("invert_yaw", false);

    // Right Joy Parameters (Auxiliary)
    this->declare_parameter("right_joy_topic", "/spacemouse/right/joy");
    this->declare_parameter("axis_z", 2);
    this->declare_parameter("axis_pitch", 3);
    this->declare_parameter("axis_head_pan", 5);
    this->declare_parameter("invert_z", false);
    this->declare_parameter("invert_pitch", false);
    this->declare_parameter("invert_head_pan", false);
    this->declare_parameter("lift_step", 0.005);
    this->declare_parameter("head_step", 0.05);

    // Joint Topics
    this->declare_parameter("lift_topic", "/leader/joystick_controller_right/joint_trajectory");
    this->declare_parameter("head_topic", "/leader/joystick_controller_left/joint_trajectory");
    this->declare_parameter("joint_state_topic", "/joint_states");
    
    // Limits
    this->declare_parameter<std::vector<double>>("lift_lower_limits", {-0.5});
    this->declare_parameter<std::vector<double>>("lift_upper_limits", {0.0});
    this->declare_parameter<std::vector<double>>("head_lower_limits", {-0.2317, -0.35});
    this->declare_parameter<std::vector<double>>("head_upper_limits", {0.6951, 0.35});

    lift_lower_limits_ = this->get_parameter("lift_lower_limits").as_double_array();
    lift_upper_limits_ = this->get_parameter("lift_upper_limits").as_double_array();
    head_lower_limits_ = this->get_parameter("head_lower_limits").as_double_array();
    head_upper_limits_ = this->get_parameter("head_upper_limits").as_double_array();

    // Subscribers
    left_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      this->get_parameter("left_joy_topic").as_string(), 10, std::bind(&SpaceMouseBaseTeleop::left_joy_callback, this, _1));
      
    right_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      this->get_parameter("right_joy_topic").as_string(), 10, std::bind(&SpaceMouseBaseTeleop::right_joy_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      this->get_parameter("joint_state_topic").as_string(), 20, std::bind(&SpaceMouseBaseTeleop::joint_state_callback, this, _1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    lift_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("lift_topic").as_string(), 10);
    head_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("head_topic").as_string(), 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/teleop_mode", 10);

    // Initial state
    current_lift_pos_ = 0.0;
    current_head_pos_ = {0.0, 0.0};
    current_mode_ = "BASE";
    
    // Publish initial mode after a short delay
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std_msgs::msg::String mode_msg;
        mode_msg.data = current_mode_;
        mode_pub_->publish(mode_msg);
    }).detach();

    RCLCPP_INFO(this->get_logger(), "SpaceMouse Base Teleop Started. Mode: %s", current_mode_.c_str());
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "lift_joint") {
        current_lift_pos_ = msg->position[i];
      } else if (msg->name[i] == "head_joint1") {
        current_head_pos_[0] = msg->position[i];
      } else if (msg->name[i] == "head_joint2") {
        current_head_pos_[1] = msg->position[i];
      }
    }
  }

  double get_axis(const sensor_msgs::msg::Joy& joy, int index, bool invert) {
    if (index >= 0 && index < static_cast<int>(joy.axes.size())) {
      double val = joy.axes[index];
      // Apply a small deadband
      if (std::abs(val) < 0.05) return 0.0;
      return invert ? -val : val;
    }
    return 0.0;
  }

  void left_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Check buttons for double-click mode switch
    {
        std::lock_guard<std::mutex> lock(button_mutex_);
        for (size_t i = 0; i < msg->buttons.size() && i < 2; ++i) {
            bool pressed = msg->buttons[i] > 0;
            if (pressed && !left_clicks_[i].prev_pressed) {
                double now = this->now().seconds();
                if ((now - left_clicks_[i].last_press_time_sec) > 0.5) {
                    left_clicks_[i].count = 1;
                } else {
                    left_clicks_[i].count++;
                }
                left_clicks_[i].last_press_time_sec = now;
            }
            left_clicks_[i].prev_pressed = pressed;
        }
        check_four_button_double_click();
    }

    // Base control (Swerve)
    if (current_mode_ != "BASE") return;

    double max_linear_vel = this->get_parameter("max_linear_vel").as_double();
    double max_angular_vel = this->get_parameter("max_angular_vel").as_double();

    double x = get_axis(*msg, this->get_parameter("axis_x").as_int(), this->get_parameter("invert_x").as_bool());
    double y = get_axis(*msg, this->get_parameter("axis_y").as_int(), this->get_parameter("invert_y").as_bool());
    double yaw = get_axis(*msg, this->get_parameter("axis_yaw").as_int(), this->get_parameter("invert_yaw").as_bool());

    geometry_msgs::msg::Twist twist_msg;
    // Cubic scaling for finer control at low speeds
    twist_msg.linear.x = (x * x * x) * max_linear_vel;
    twist_msg.linear.y = (y * y * y) * max_linear_vel;
    twist_msg.angular.z = (yaw * yaw * yaw) * max_angular_vel;

    cmd_vel_pub_->publish(twist_msg);
  }

  void right_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Check buttons for double-click mode switch
    {
        std::lock_guard<std::mutex> lock(button_mutex_);
        for (size_t i = 0; i < msg->buttons.size() && i < 2; ++i) {
            bool pressed = msg->buttons[i] > 0;
            if (pressed && !right_clicks_[i].prev_pressed) {
                double now = this->now().seconds();
                if ((now - right_clicks_[i].last_press_time_sec) > 0.5) {
                    right_clicks_[i].count = 1;
                } else {
                    right_clicks_[i].count++;
                }
                right_clicks_[i].last_press_time_sec = now;
            }
            right_clicks_[i].prev_pressed = pressed;
        }
        check_four_button_double_click();
    }

    // Auxiliary control (Lift & Head)
    if (current_mode_ != "BASE") return;

    double lift_step = this->get_parameter("lift_step").as_double();
    double head_step = this->get_parameter("head_step").as_double();

    double z = get_axis(*msg, this->get_parameter("axis_z").as_int(), this->get_parameter("invert_z").as_bool());
    double pitch = get_axis(*msg, this->get_parameter("axis_pitch").as_int(), this->get_parameter("invert_pitch").as_bool());
    double pan = get_axis(*msg, this->get_parameter("axis_head_pan").as_int(), this->get_parameter("invert_head_pan").as_bool());

    // Only process and publish if there is input
    if (std::abs(z) > 0.01) {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"lift_joint"};
      
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start.sec = 0;
      point.time_from_start.nanosec = 0;
      point.positions = {current_lift_pos_ + (z * lift_step)};
      point.velocities = {0.0};
      
      traj.points.push_back(point);
      lift_pub_->publish(traj);
    }

    if (std::abs(pitch) > 0.01 || std::abs(pan) > 0.01) {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"head_joint1", "head_joint2"};
      
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start.sec = 0;
      point.time_from_start.nanosec = 0;
      
      // We map pan to joint1, pitch to joint2
      point.positions = {
          current_head_pos_[0] + (pan * head_step),
          current_head_pos_[1] + (pitch * head_step)
      };
      point.velocities = {0.0, 0.0};
      
      traj.points.push_back(point);
      head_pub_->publish(traj);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr right_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lift_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;

  std::mutex joint_mutex_;
  double current_lift_pos_;
  std::vector<double> current_head_pos_;
  std::string current_mode_;
  
  std::vector<double> lift_lower_limits_;
  std::vector<double> lift_upper_limits_;
  std::vector<double> head_lower_limits_;
  std::vector<double> head_upper_limits_;

  std::mutex button_mutex_;
  std::vector<ButtonClickState> left_clicks_{2};
  std::vector<ButtonClickState> right_clicks_{2};

  void check_four_button_double_click() {
      double now = this->now().seconds();
      auto is_dc = [now](const ButtonClickState& state) {
          return state.count >= 2 && (now - state.last_press_time_sec) < 1.0;
      };
      
      if (is_dc(left_clicks_[0]) && is_dc(left_clicks_[1]) && 
          is_dc(right_clicks_[0]) && is_dc(right_clicks_[1])) {
          
          current_mode_ = (current_mode_ == "BASE") ? "ARM" : "BASE";
          RCLCPP_INFO(this->get_logger(), "ALL FOUR BUTTONS DOUBLE CLICKED! SWITCHING MODE TO: %s", current_mode_.c_str());
          
          std_msgs::msg::String mode_msg;
          mode_msg.data = current_mode_;
          mode_pub_->publish(mode_msg);
          
          left_clicks_[0].count = 0;
          left_clicks_[1].count = 0;
          right_clicks_[0].count = 0;
          right_clicks_[1].count = 0;
      }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpaceMouseBaseTeleop>());
  rclcpp::shutdown();
  return 0;
}
