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
#include "std_msgs/msg/int16.hpp"
#include "std_srvs/srv/empty.hpp"

// Scaling factors for velocities.
constexpr double LINEAR_VELOCITY_SCALE = 1.0;
constexpr double ANGULAR_VELOCITY_SCALE = 1.0;

class KvasirTeleopControllerNode : public rclcpp::Node {
public:
  KvasirTeleopControllerNode() : Node("kvasir_teleop_controller") {
    comms_ = std::make_unique<LocalNucleoInterface>(1000);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          this->twist_callback(msg);
        });

    elevator_pos_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "elevator_pos", 10,
        [this](const std_msgs::msg::Int16::SharedPtr msg) {
          comms_->move_elevator(msg->data);
        });

    extend_arm_service_ = this->create_service<std_srvs::srv::Empty>(
        "extend_arm",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               const std::shared_ptr<std_srvs::srv::Empty::Response>) {
          comms_->extend_arm();
        });
    retract_arm_service_ = this->create_service<std_srvs::srv::Empty>(
        "retract_arm",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               const std::shared_ptr<std_srvs::srv::Empty::Response>) {
          comms_->retract_arm();
        });
    extend_pusher_service_ = this->create_service<std_srvs::srv::Empty>(
        "extend_pusher",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               const std::shared_ptr<std_srvs::srv::Empty::Response>) {
          for (size_t i = 0; i < LocalNucleoInterface::TIM1_SERVOS; ++i) {
            pushers_[i] = !pushers_[i]; // Toggle pusher state
          }
          comms_->extend_pusher(pushers_);
        });
    retract_pusher_service_ = this->create_service<std_srvs::srv::Empty>(
        "retract_pusher",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               const std::shared_ptr<std_srvs::srv::Empty::Response>) {
          comms_->retract_pusher();
        });

    RCLCPP_INFO(this->get_logger(), "Kvasir Teleop Controller started.");
    RCLCPP_INFO(this->get_logger(),
                "Listening for Twist messages on /cmd_vel.");
    RCLCPP_INFO(this->get_logger(),
                "Ensure teleop_twist_keyboard is running (ros2 run "
                "teleop_twist_keyboard teleop_twist_keyboard).");
    RCLCPP_INFO(this->get_logger(),
                "Available services: /extend_arm, /retract_arm, "
                "/extend_pusher, /retract_pusher.");
    RCLCPP_INFO(this->get_logger(),
                "Use 'ros2 topic pub /elevator_pos std_msgs/Int16 1' "
                "to set elevator position.");
    RCLCPP_INFO(this->get_logger(),
                "Use 'ros2 service call /extend_arm std_srvs/srv/Empty' "
                "to extend the arm, and '/retract_arm' to retract it.");
    RCLCPP_INFO(this->get_logger(),
                "Use 'ros2 service call /extend_pusher std_srvs/srv/Empty' "
                "to extend the pusher, and '/retract_pusher' to retract it.");
    RCLCPP_INFO(this->get_logger(),
                "Use 'ros2 service call /extend_pusher std_srvs/srv/Empty' "
                "to extend the pusher, and '/retract_pusher' to retract it.");
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

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr elevator_pos_sub_;

  std::unique_ptr<LocalNucleoInterface> comms_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  bool pushers_[LocalNucleoInterface::TIM1_SERVOS] = {true, false, false, true};
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr extend_arm_service_,
      retract_arm_service_, extend_pusher_service_, retract_pusher_service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KvasirTeleopControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
