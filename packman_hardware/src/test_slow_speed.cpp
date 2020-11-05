// Copyright 2020 RUVU BV.

#include <ros/init.h>
#include <ros/node_handle.h>

#include "./pacman_interface.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_slow_speed");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle local_nh("~");
  {
    PacmanInterface interface;
    interface.init();
    ROS_INFO("%s started", local_nh.getNamespace().c_str());
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
  }
}
