#include <iterator>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "loki_hw_interface/local_nucleo_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

constexpr double DEADZONE_THRESHOLD = 0.2;

constexpr double apply_deadzone(double x) { return std::abs(x) < DEADZONE_THRESHOLD ? 0.0 : x; };

constexpr double SPEED_GAINS[] = {10, 80, 300};
constexpr double THETA_DOT_GAIN = 5;

constexpr double WHEEL_RADIUS = 65e-3; // m
constexpr double WHEEL_BASE = 200e-3;  // m
constexpr double SIN_PI_3 = 0.8660254037844386;
constexpr double SQRT_3 = 1.7320508075688772;

constexpr double map_trigger_to_servo(double x, double new_min, double new_max) {
  // Normalize x from [-1, 1] to [0, 1] and then scale to [new_min, new_max]
  return ((x + 1.0) / 2.0) * (new_max - new_min) + new_min;
}

class JoystickControllerNode : public rclcpp::Node {
public:
  JoystickControllerNode() : Node("joystick_controller_node") {
    comms_ = std::make_unique<LocalNucleoInterface>(1000);

    sub_left_x_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/left_x", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
          left_x_ = apply_deadzone(msg->data);
        });

    sub_left_y_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/left_y", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
          left_y_ = apply_deadzone(msg->data);
        });

    sub_third_axis_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/third_axis", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
          third_axis_ = apply_deadzone(msg->data);
        });

    sub_lt_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/lt", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { lt_ = apply_deadzone(msg->data); });

    sub_rt_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/rt", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { rt_ = apply_deadzone(msg->data); });

    sub_start_ = this->create_subscription<std_msgs::msg::Bool>(
        "joystick/start", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { start_ = msg->data; });

    sub_select_ = this->create_subscription<std_msgs::msg::Bool>(
        "joystick/select", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { select_ = msg->data; });

    // Timer to send wheel speeds at a regular interval
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     [this]() { this->update_wheel_speeds(); });
  }

private:
  void update_wheel_speeds() {
    // Allow speed selection via start and select.
    if (start_ && !last_start_) {
      if (speed_gain_idx_ != std::size(SPEED_GAINS) - 1)
        speed_gain_idx_ += 1;
      comms_->print_lcd(1, "Speed up :)  ");
    } else if (select_ && !last_select_) {
      if (speed_gain_idx_ != 0)
        speed_gain_idx_ -= 1;
      comms_->print_lcd(1, "Speed down :(");
    }
    auto speed_gain_ = SPEED_GAINS[speed_gain_idx_];

    // Compute (x_dot, y_dot, theta_dot) with appropriate convention conversions (e.g. y reversal)
    // and gains in the theta case.
    double x_dot = -left_x_;
    double y_dot = left_y_;
    double theta_dot = THETA_DOT_GAIN * (-third_axis_);

    // Compute inverse kinematics via inverse Jacobian, with speed gain.
    double u1 = speed_gain_ * (-WHEEL_BASE * theta_dot + x_dot) / WHEEL_RADIUS;
    double u2 =
        speed_gain_ * (-WHEEL_BASE * theta_dot - x_dot / 2 - y_dot * SIN_PI_3) / WHEEL_RADIUS;
    double u3 =
        speed_gain_ * (-WHEEL_BASE * theta_dot - x_dot / 2 + y_dot * SIN_PI_3) / WHEEL_RADIUS;

    // Send wheel speeds.
    comms_->set_wheel_speeds({u1, u2, u3});

    // Compute servo angles based on triggers and send them.
    double theta0 = map_trigger_to_servo(lt_, 70, 90);
    comms_->set_servo_angle(0, theta0);
    double theta1 = map_trigger_to_servo(rt_, 0, 110);
    comms_->set_servo_angle(1, theta1);

    // Debug logging, yay!
    RCLCPP_DEBUG(this->get_logger(), "u=(%.2f, %.2f, %.2f) s=(%.2f°, %.2f) b=(%d %d)", u1, u2, u3,
                 theta0, theta1, start_, select_);

    // Update button state for "just pressed" check.
    last_start_ = start_;
    last_select_ = select_;
  }

  std::unique_ptr<LocalNucleoInterface> comms_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_x_, sub_left_y_, sub_third_axis_,
      sub_lt_, sub_rt_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_, sub_select_;

  double left_x_{}, left_y_{}, third_axis_{}, lt_{}, rt_{};
  bool start_{}, select_{}, last_start_{}, last_select_{};

  std::size_t speed_gain_idx_ = 1;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
