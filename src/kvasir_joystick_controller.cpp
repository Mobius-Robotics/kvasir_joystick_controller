#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "kvasir_hw_interface/local_nucleo_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

constexpr double DEADZONE_THRESHOLD = 0.2;

constexpr double apply_deadzone(double x) {
  return std::abs(x) < DEADZONE_THRESHOLD ? 0.0 : x;
};

constexpr double SPEED_GAINS[] = {10, 80, 300};
constexpr double THETA_DOT_GAIN = 5;

constexpr double map_trigger_to_servo(double x, double new_min,
                                      double new_max) {
  // Normalize x from [-1, 1] to [0, 1] and then scale to [new_min, new_max]
  return ((x + 1.0) / 2.0) * (new_max - new_min) + new_min;
}

class Button {
public:
  void update(bool current_state) {
    prev_state_ = current_state_;
    current_state_ = current_state;
  }

  bool just_pressed() const { return current_state_ && !prev_state_; }

private:
  bool current_state_ = false;
  bool prev_state_ = false;
};

class JoystickControllerNode : public rclcpp::Node {
public:
  JoystickControllerNode() : Node("joystick_controller_node") {
    comms_ = std::make_unique<LocalNucleoInterface>(1000);

    sub_left_x_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/left_x", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          left_x_ = apply_deadzone(msg->data);
        });

    sub_left_y_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/left_y", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          left_y_ = apply_deadzone(msg->data);
        });

    sub_third_axis_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/third_axis", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          third_axis_ = apply_deadzone(msg->data);
        });

    sub_lt_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/lt", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
          lt_ = apply_deadzone(msg->data);
        });

    sub_rt_ = this->create_subscription<std_msgs::msg::Float32>(
        "joystick/rt", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
          rt_ = apply_deadzone(msg->data);
        });

    sub_start_ = this->create_subscription<std_msgs::msg::Bool>(
        "joystick/start", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          start_.update(msg->data);
        });

    sub_select_ = this->create_subscription<std_msgs::msg::Bool>(
        "joystick/select", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          select_.update(msg->data);
        });

    // Start every servo at 90°
    servo_angles_.fill(90.0);

    // Timer to send wheel speeds at a regular interval
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     [this]() { this->update_wheel_speeds(); });
  }

private:
  void update_wheel_speeds() {
    // Allow speed selection via start and select.
    if (start_.just_pressed()) {
      if (speed_gain_idx_ != std::size(SPEED_GAINS) - 1)
        speed_gain_idx_ += 1;
    } else if (select_.just_pressed()) {
      if (speed_gain_idx_ != 0)
        speed_gain_idx_ -= 1;
    }
    auto speed_gain_ = SPEED_GAINS[speed_gain_idx_];

    // Send servo angles and wheel speeds.
    comms_->set_servo_angles(servo_angles_);
    comms_->set_body_velocity(-speed_gain_ * left_x_, speed_gain_ * left_y_,
                              THETA_DOT_GAIN * (-third_axis_));
  }

  std::unique_ptr<LocalNucleoInterface> comms_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_x_,
      sub_left_y_, sub_third_axis_, sub_lt_, sub_rt_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_, sub_select_;

  double left_x_{}, left_y_{}, third_axis_{}, lt_{}, rt_{};
  Button start_{}, select_{};

  std::array<double, 6> servo_angles_{};

  std::size_t speed_gain_idx_ = 1;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
