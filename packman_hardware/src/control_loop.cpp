/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example control loop for reading, updating, and writing commands to a hardware interface
   using MONOTOIC system time
*/

#include "./control_loop.h"

#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>

namespace packman_hardware
{
// Used to convert seconds elapsed to nanoseconds
const double BILLION = 1000000000.0;

// Name of this class
const auto name = "control_loop";

ControlLoop::ControlLoop(const ros::NodeHandle& nh, const ros::NodeHandle& local_nh,
                         std::shared_ptr<hardware_interface::RobotHW> hardware_interface)
  : hardware_interface_(hardware_interface)
{
  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh));

  if (!local_nh.getParam("loop_hz", loop_hz_))
    throw std::runtime_error("Missing parameter loop_hz");
  if (!local_nh.getParam("cycle_time_error_threshold", cycle_time_error_threshold_))
    throw std::runtime_error("Missing parameter cycle_time_error_threshold");

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &last_time_);

  desired_update_period_ = ros::Duration(1 / loop_hz_);
}

void ControlLoop::run()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    update();
    rate.sleep();
  }
}

void ControlLoop::update()
{
  // Get change in time
  clock_gettime(CLOCK_MONOTONIC, &current_time_);
  elapsed_time_ =
      ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
  last_time_ = current_time_;
  ros::Time now = ros::Time::now();
  // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, name, "Sampled update loop with elapsed time " << elapsed_time_.toSec());

  // Error check cycle time
  const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
  if (cycle_time_error > cycle_time_error_threshold_)
  {
    ROS_WARN_STREAM_NAMED(name, "Cycle time exceeded error threshold by: "
                                    << cycle_time_error << ", cycle time: " << elapsed_time_
                                    << ", threshold: " << cycle_time_error_threshold_);
  }

  // Input
  hardware_interface_->read(now, elapsed_time_);

  // Control
  controller_manager_->update(now, elapsed_time_);

  // Output
  hardware_interface_->write(now, elapsed_time_);
}

}  // namespace packman_hardware
