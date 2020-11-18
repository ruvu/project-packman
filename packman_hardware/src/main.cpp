// Copyright 2020 RUVU BV.

#include <string>

#include "./control_loop.hpp"
#include "./robot_hw.hpp"

using packman_hardware::ControlLoop;
using packman_hardware::RobotHW;

int main(int argc, char* argv[])
{
  // This main function is inspired by ros_control_boilerplate

  ros::init(argc, argv, "packman_hw_interface");

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");
  std::string can_device = local_nh.param("can_device", std::string("can0"));

  // Create the hardware interface specific to your robot
  auto robot_hw = std::make_shared<RobotHW>(can_device);
  robot_hw->init(nh, local_nh);

  // Start the control loop
  ControlLoop control_loop(nh, local_nh, robot_hw);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}
