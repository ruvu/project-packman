// Copyright 2020 RUVU BV.

#include "./robot_hw.h"

#include <socketcan_interface/threading.h>
#include <string>

const auto name = "robot_hw";

namespace packman
{
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

RobotHW::RobotHW(const std::string& can_device)
{
  for (unsigned int i = 0; i < DRIVE_NAMES.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(DRIVE_NAMES[i], &state_.joints_[i].position,
                                                            &state_.joints_[i].velocity, &state_.joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &state_.joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  ROS_INFO_STREAM_NAMED(name, "Binding to socketcan interface " << can_device << " ...");
  if (!can_interface_.init(can_device, false, can::NoSettings::create()))
  {
    throw std::runtime_error("Packman: Could not set-up socketcan interface on device " + can_device);
  }
  ROS_INFO_STREAM_NAMED(name, "Initialized socketcan interface on device " << can_device);

  // Register CAN comm
  using std::placeholders::_1;
  can_listener_ =
      can_interface_.createMsgListener(can::MsgHeader(PLC_STATE_ID), std::bind(&RobotHW::plcStateCb, this, _1));
  state_listener_ = can_interface_.createStateListener(std::bind(&RobotHW::CANStateCb, this, _1));
}

RobotHW::~RobotHW()
{
  ROS_INFO_NAMED(name, "Shutting down the can device");
  can_interface_.shutdown();
}

bool RobotHW::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& /*robot_hw_nh*/)
{
  // TODO(paul):
  // Send NMT pre-operational
  // Set heartbeat cycle time to 0.3s using a SDO to 1017 (in ms)
  // Send NMT operational
  return true;
}

void RobotHW::read(const ros::Time& /*time*/, const ros::Duration& period)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  state_.joints_[packman::LEFT].velocity = state_.plc_.motor_speeds_[packman::LEFT] / 60 * 2 * M_PI;
  state_.joints_[packman::LEFT].position += state_.joints_[packman::LEFT].velocity * period.toSec();

  state_.joints_[packman::RIGHT].velocity = state_.plc_.motor_speeds_[packman::RIGHT] / 60 * 2 * M_PI;
  state_.joints_[packman::RIGHT].position += state_.joints_[packman::RIGHT].velocity * period.toSec();
}

void RobotHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  // TODO(paul): Cast command velocities to can frame
  // sendRecover(can_interface_, frame);
}

void RobotHW::plcStateCb(const can::Frame& f)
{
  ROS_INFO_STREAM("Received PlcState frame " << f);

  RxPDO1 pdo(f.data);
  ROS_INFO_STREAM_NAMED(name, pdo);

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.plc_.set(f);
}

void RobotHW::CANStateCb(const can::State& s)
{
  std::string err;
  can_interface_.translateError(s.internal_error, err);

  std::stringstream msg;
  msg << "CAN state=" << s.driver_state << " error=" << s.internal_error << "(" << err << ") asio: " << s.error_code;

  ROS_INFO_STREAM("CANState Callback: " << msg.str());
}
}  // namespace packman
