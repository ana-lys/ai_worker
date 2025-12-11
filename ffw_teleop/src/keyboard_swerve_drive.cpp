#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode() : Node("keyboard_teleop")
  {
    // Declare parameters
    this->declare_parameter("topic", "/cmd_vel");
    this->declare_parameter("update_rate", 100.0);  // Hz
    this->declare_parameter("max_vx", 1.0);
    this->declare_parameter("min_vx", -0.5);
    this->declare_parameter("max_wz", 1.5);
    this->declare_parameter("min_wz", -1.5);
    this->declare_parameter("accel_rate", 2.0);     // m/s² for linear
    this->declare_parameter("accel_rate_angular", 3.0);  // rad/s² for angular
    this->declare_parameter("vx_discount", 0.02);   // Discount factor for deceleration
    this->declare_parameter("wz_discount", 0.03);   // Discount factor for angular deceleration
    
    std::string topic = this->get_parameter("topic").as_string();
    update_rate_ = this->get_parameter("update_rate").as_double();
    max_vx_ = this->get_parameter("max_vx").as_double();
    min_vx_ = this->get_parameter("min_vx").as_double();
    max_wz_ = this->get_parameter("max_wz").as_double();
    min_wz_ = this->get_parameter("min_wz").as_double();
    accel_rate_ = this->get_parameter("accel_rate").as_double();
    accel_rate_angular_ = this->get_parameter("accel_rate_angular").as_double();
    vx_discount_ = this->get_parameter("vx_discount").as_double();
    wz_discount_ = this->get_parameter("wz_discount").as_double();
    
    // Calculate time step and velocity steps
    dt_ = 1.0 / update_rate_;
    vx_step_ = accel_rate_ * dt_;
    wz_step_ = accel_rate_angular_ * dt_;
    
    // Create publisher
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    
    // Setup terminal for non-blocking input
    setupTerminal();
    
    // Initialize state
    current_vx_ = 0.0;
    current_wz_ = 0.0;
    enabled_ = false;
    
    // Create timer at specified rate
    auto timer_interval = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_interval),
      std::bind(&KeyboardTeleopNode::updateLoop, this));
    
    printInstructions();
  }
  
  ~KeyboardTeleopNode()
  {
    restoreTerminal();
  }

private:
  void setupTerminal()
  {
    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &old_tio_);
    
    // Set terminal to non-canonical mode (no line buffering)
    struct termios new_tio = old_tio_;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    
    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }
  
  void restoreTerminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
  }
  
  void printInstructions()
  {
    RCLCPP_INFO(this->get_logger(), "\n"
      "=================================================\n"
      "     Swerve Drive Keyboard Teleop Control\n"
      "=================================================\n"
      "   Arrow Keys:\n"
      "     ↑  : Forward  (vx+)\n"
      "     ↓  : Backward (vx-)\n"
      "     ←  : Rotate Left  (wz+)\n"
      "     →  : Rotate Right (wz-)\n"
      "\n"
      "   SPACE : Toggle ON/OFF (currently: OFF)\n"
      "   ESC   : Quit\n"
      "\n"
      "   Hold key: Gradually increase velocity\n"
      "   Release : Gradually return to zero\n"
      "\n"
      "   Limits: vx[%.1f, %.1f] wz[%.1f, %.1f] m/s\n"
      "   Rate: %.0f Hz\n"
      "=================================================\n",
      min_vx_, max_vx_, min_wz_, max_wz_, update_rate_);
  }
  
  void updateLoop()
  {
    // Read keyboard input and update masks
    processKeyboard();
    
    // Update velocities using masking approach
    updateVelocities();
    
    // Publish command
    publishCommand();
  }
  
  void processKeyboard()
  {
    char c;
    bool U = false, D = false, L = false, R = false;
    
    // Read all available input
    while (read(STDIN_FILENO, &c, 1) > 0) {
      // Check for escape sequences (arrow keys)
      if (c == 27) {  // ESC
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) > 0) {
          if (seq[0] == '[') {
            if (read(STDIN_FILENO, &seq[1], 1) > 0) {
              switch(seq[1]) {
                case 'A':  // Up arrow
                  U = true;
                  break;
                case 'B':  // Down arrow
                  D = true;
                  break;
                case 'D':  // Left arrow
                  L = true;
                  break;
                case 'C':  // Right arrow
                  R = true;
                  break;
              }
            }
          }
        } else {
          // Just ESC key - quit
          RCLCPP_INFO(this->get_logger(), "Shutting down...");
          rclcpp::shutdown();
          return;
        }
      }
      else if (c == ' ') {  // Space - toggle
        enabled_ = !enabled_;
        if (enabled_) {
          RCLCPP_INFO(this->get_logger(), ">>> ENABLED <<<");
        } else {
          RCLCPP_INFO(this->get_logger(), ">>> DISABLED <<<");
          // Immediately stop when disabled
          current_vx_ = 0.0;
          current_wz_ = 0.0;
        }
      }
    }
    
    // Update masks only if enabled
    if (enabled_) {
      mask_U_ = U;
      mask_D_ = D;
      mask_L_ = L;
      mask_R_ = R;
    } else {
      // Disabled - clear all masks
      mask_U_ = mask_D_ = mask_L_ = mask_R_ = false;
    }
  }
  
  void updateVelocities()
  {
    if (!enabled_) {
      current_vx_ = current_wz_ = 0.0;
      return;
    }
    
    // Linear velocity masking: vx += (U-D) * vx_step
    int8_t linear_mask = (mask_U_ ? 1 : 0) - (mask_D_ ? 1 : 0);
    if (linear_mask != 0) {
      current_vx_ += linear_mask * vx_step_;
      current_vx_ = std::clamp(current_vx_, min_vx_, max_vx_);
    } else {
      // No linear input: vx *= 1 - (!U & !D)*vx_discount
      bool no_linear_input = !mask_U_ && !mask_D_;
      if (no_linear_input && std::abs(current_vx_) > 0.001) {
        current_vx_ *= (1.0 - vx_discount_);
        if (std::abs(current_vx_) < 0.001) current_vx_ = 0.0;
      }
    }
    
    // Angular velocity masking: wz += (L-R) * wz_step  
    int8_t angular_mask = (mask_L_ ? 1 : 0) - (mask_R_ ? 1 : 0);
    if (angular_mask != 0) {
      current_wz_ += angular_mask * wz_step_;
      current_wz_ = std::clamp(current_wz_, min_wz_, max_wz_);
    } else {
      // No angular input: wz *= 1 - (!L & !R)*wz_discount
      bool no_angular_input = !mask_L_ && !mask_R_;
      if (no_angular_input && std::abs(current_wz_) > 0.001) {
        current_wz_ *= (1.0 - wz_discount_);
        if (std::abs(current_wz_) < 0.001) current_wz_ = 0.0;
      }
    }
  }
  
  void publishCommand()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = current_vx_;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = current_wz_;
    
    pub_->publish(msg);
    
    // Print status every 1 second (at 100Hz = every 100 cycles)
    static int counter = 0;
    counter++;
    if (counter >= static_cast<int>(update_rate_)) {
      counter = 0;
      RCLCPP_INFO(this->get_logger(), 
        "[%s] vx: %.2f | wz: %.2f | masks: %d%d%d%d", 
        enabled_ ? "ON " : "OFF",
        current_vx_, current_wz_, 
        mask_U_, mask_D_, mask_L_, mask_R_);
    }
  }
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  struct termios old_tio_;
  
  // Parameters
  double update_rate_;
  double max_vx_;
  double min_vx_;
  double max_wz_;
  double min_wz_;
  double accel_rate_;
  double accel_rate_angular_;
  double vx_discount_;
  double wz_discount_;
  
  // Computed values
  double dt_;
  double vx_step_;
  double wz_step_;
  
  // State
  double current_vx_;
  double current_wz_;
  bool enabled_;
  
  // Key masks
  bool mask_U_, mask_D_, mask_L_, mask_R_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}