// Copyright 2020 RUVU BV.

#include <string>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "./robot_hw.h"

using packman::RobotHW;

//!
//! \brief controlThread Separate thread for running the controller
//!
void controlThread(ros::Rate rate, RobotHW* robot, controller_manager::ControllerManager* cm)
{
  ros::Time last_cycle_time = ros::Time::now();
  while (ros::ok())
  {
    robot->read(ros::Time::now(), ros::Time::now() - last_cycle_time);
    cm->update(ros::Time::now(), ros::Time::now() - last_cycle_time);
    robot->write(ros::Time::now(), ros::Time::now() - last_cycle_time);

    if (rate.cycleTime() > rate.expectedCycleTime())
    {
      ROS_WARN_STREAM_DELAYED_THROTTLE(10.0, "Cycle time too high: Cycle time: " << rate.cycleTime()
                                                                                 << ", Expected cycle time: "
                                                                                 << rate.expectedCycleTime());
    }

    last_cycle_time = ros::Time::now();

    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "packman_hardware");

  ROS_INFO("Packman Hardware interface initialized");

  ros::NodeHandle local_nh("~");
  ros::Rate rate(local_nh.param("frequency", 100));
  std::string can_device = local_nh.param("can_device", std::string("can0"));

  try
  {
    RobotHW robot(can_device);

    controller_manager::ControllerManager cm(&robot);

    boost::thread(boost::bind(controlThread, rate, &robot, &cm));

    ROS_INFO("Packman Controller Manager initialized, spinning ...");

    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Packman hardware error: " << e.what());
    return 1;
  }

  return 0;
}
