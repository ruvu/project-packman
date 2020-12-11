#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "./packman_interface.hpp"

using packman_hardware::PackmanInterface;
using packman_hardware::TxPDO1;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  PackmanInterface interface;
  interface.init("vcan0");

  const auto node = std::make_shared<rclcpp::Node>("test_packman_hardware");
  const auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [&interface, &node](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        TxPDO1 pdo{};
        pdo.enableLeftMotor(true);
        pdo.enableRightMotor(true);
        pdo.ok(true);
        pdo.target_left_motor_speed = msg->linear.x * 60 / 2 / M_PI * 1e3;
        pdo.target_right_motor_speed = msg->angular.z * 60 / 2 / M_PI * 1e3;
        interface.sendValues(pdo);
        RCLCPP_INFO(node->get_logger(), "Callback...");
      });

  RCLCPP_INFO(node->get_logger(), "%s started", node->get_name());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
