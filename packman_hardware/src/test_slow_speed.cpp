// Copyright 2020 RUVU BV.

#include <geometry_msgs/Twist.h>
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

  ros::NodeHandle nh;
  PacmanInterface interface;
  interface.init();
  ROS_INFO("%s started", ros::NodeHandle("~").getNamespace().c_str());

  boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb =
      [&interface](const geometry_msgs::Twist::ConstPtr& msg) { interface.drive(msg->linear.x, msg->angular.z); };
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1, cb);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
