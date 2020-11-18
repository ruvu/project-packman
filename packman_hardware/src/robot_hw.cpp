// Copyright 2020 RUVU BV.

#include "./robot_hw.h"

#include <socketcan_interface/threading.h>

#include <string>

namespace packman_hardware
{
const auto name = "robot_hw";
const std::array<std::string, 2> joint_names = { "left_wheel", "right_wheel" };

void sendRecover(std::shared_ptr<can::ThreadedSocketCANInterface> interface, const can::Frame& frame)
{
  while (!interface->send(frame))
  {
    ROS_WARN("Could not send can frame, trying to recover within 1 second ...");
    ros::Duration(1.0).sleep();
    if (interface->recover())
    {
      ROS_INFO("Recovered can interface, resending CAN message ...");
    }
    else
    {
      ROS_ERROR("Failed to recover can interface ...");
    }
  }
}

RobotHW::RobotHW(const std::string& can_device) : interface_(can_device)
{
}

bool RobotHW::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& /*robot_hw_nh*/)
{
  interface_.init();

  auto num_joints = joint_names.size();
  joints_.resize(num_joints);
  commands_.resize(num_joints);

  for (auto i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joints_[i].position, &joints_[i].velocity,
                                                            &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &commands_[i].velocity);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  return true;
}

void RobotHW::read(const ros::Time& /*time*/, const ros::Duration& period)
{
  auto pdo = interface_.lastValues();

  joints_[0].velocity = pdo.actual_left_motor_speed / 60. * 2 * M_PI / 1e3;
  joints_[0].position += joints_[0].velocity * period.toSec();
  joints_[1].velocity = pdo.actual_right_motor_speed / 60. * 2 * M_PI / 1e3;
  joints_[1].position += joints_[1].velocity * period.toSec();
}

void RobotHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  TxPDO1 pdo{};
  pdo.enableLeftMotor(true);
  pdo.enableRightMotor(true);
  pdo.ok(true);
  pdo.target_left_motor_speed = commands_[0].velocity * 60 / 2 / M_PI;
  pdo.target_right_motor_speed = commands_[1].velocity * 60 / 2 / M_PI;
  // ROS_INFO_STREAM_NAMED(name, "left: " << commands_[0].velocity << ", cmd: " << pdo.target_left_motor_speed);
  // ROS_INFO_STREAM_NAMED(name, "right: " << commands_[0].velocity << ", cmd: " << pdo.target_left_motor_speed);
  interface_.sendValues(pdo);
}
}  // namespace packman_hardware
