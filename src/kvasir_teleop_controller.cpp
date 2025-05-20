#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "kvasir_hw_interface/local_nucleo_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

// Scaling factors for velocities.
constexpr double LINEAR_VELOCITY_SCALE = 1.0;
constexpr double ANGULAR_VELOCITY_SCALE = 1.0;
constexpr double DEFAULT_SERVO_ANGLE = 90.0;
constexpr long SERVO_COMMAND_INTERVAL_MS = 100;

class KvasirTeleopControllerNode : public rclcpp::Node {
public:
  KvasirTeleopControllerNode() : Node("kvasir_teleop_controller") {
    comms_ = std::make_unique<LocalNucleoInterface>(1000);

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          this->twist_callback(msg);
        });

    servo_angles_.fill(DEFAULT_SERVO_ANGLE);
    servo_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(SERVO_COMMAND_INTERVAL_MS),
        [this]() { comms_->set_servo_angles(servo_angles_); });

    sub_elevator_steps_ = this->create_subscription<std_msgs::msg::UInt8>(
        "elevator_steps", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          comms_->elevator_step(msg->data, elevator_dir_);
        });
    sub_elevator_dir_ = this->create_subscription<std_msgs::msg::Bool>(
        "elevator_dir", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          elevator_dir_ = msg->data;
        });

    for (size_t i = 0; i < LocalNucleoInterface::SERVO_COUNT; ++i) {
      auto topic_name = "servo" + std::to_string(i);
      servo_subscribers_[i] = this->create_subscription<std_msgs::msg::Float64>(
          topic_name, 10,
          [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
            if (msg->data < 0.0 || msg->data > 180.0) {
              RCLCPP_WARN(this->get_logger(),
                          "Received invalid servo angle: %f. "
                          "Angle must be between 0 and 180 degrees.",
                          msg->data);
              return;
            } else {
              RCLCPP_DEBUG(this->get_logger(),
                           "Received servo angle for servo %zu: %f", i,
                           msg->data);
            }
            servo_angles_[i] = msg->data;
          });
    }

    RCLCPP_INFO(this->get_logger(), "Kvasir Teleop Controller started.");
    RCLCPP_INFO(this->get_logger(),
                "Listening for Twist messages on /cmd_vel.");
    RCLCPP_INFO(this->get_logger(),
                "Ensure teleop_twist_keyboard is running (ros2 run "
                "teleop_twist_keyboard teleop_twist_keyboard).");
    RCLCPP_INFO(this->get_logger(),
                "Listening for individual servo angle commands on /servoX.");
    RCLCPP_INFO(this->get_logger(),
                "e.g. 'ros2 topic pub --once /servo0 std_msgs/msg/Float64 "
                "\"{data: 60.0}\"' to test servos.");
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double x_dot = LINEAR_VELOCITY_SCALE * msg->linear.x;
    double y_dot = -LINEAR_VELOCITY_SCALE * msg->linear.y;
    double theta_dot = ANGULAR_VELOCITY_SCALE * msg->angular.z;

    RCLCPP_DEBUG(this->get_logger(),
                 "Received Twist: lin_x=%.2f, lin_y=%.2f, ang_z=%.2f",
                 msg->linear.x, msg->linear.y, msg->angular.z);
    RCLCPP_DEBUG(this->get_logger(),
                 "Sending to HW: vx_hw=%.2f, vy_hw=%.2f, vtheta_hw=%.2f", x_dot,
                 y_dot, theta_dot);

    comms_->set_body_velocity(x_dot, y_dot, theta_dot);
  }

  std::array<double, LocalNucleoInterface::SERVO_COUNT> servo_angles_{};
  std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr,
             LocalNucleoInterface::SERVO_COUNT>
      servo_subscribers_;
  rclcpp::TimerBase::SharedPtr servo_timer_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_elevator_steps_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_elevator_dir_;
  bool elevator_dir_{};

  std::unique_ptr<LocalNucleoInterface> comms_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KvasirTeleopControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
