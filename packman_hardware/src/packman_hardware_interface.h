// Copyright 2020 RUVU BV.

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <socketcan_interface/socketcan.h>
#include <thread>  // NOLINT
#include <mutex>  // NOLINT
#include <string>
#include <vector>

#include "./packman_state.h"

class Packman : public hardware_interface::RobotHW
{
public:
  using FrameDelegate = can::CommInterface::FrameFunc;
  using StateDelegate = can::StateInterface::StateFunc;

  //!
  //! \brief Packman Robot hardware interface
  //!
  Packman(const std::string& can_device);
  ~Packman() override;

  //!
  //! \brief read Packman data to JointState interface
  //!
  void read(const ros::Time& /*time*/, const ros::Duration& period) override;

  //!
  //! \brief write VelocityJointInterface data to Packman
  //!
  void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

private:
  //! CAN Callbacks
  void plcStateCb(const can::Frame& f);
  void CANStateCb(const can::State& s);

  // Interface to ROS_CONTROL
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  //!
  //! \brief can_interface_ Socketcan interface (can connection)
  //!
  std::shared_ptr<can::ThreadedSocketCANInterface> can_interface_;

  //!
  //! \brief can_listeners_ Listeners for the various different frames coming from the plc
  //!
  std::vector<can::CommInterface::FrameListenerConstSharedPtr> can_listeners_;
  can::StateInterface::StateListenerConstSharedPtr state_listener_;


  //!
  //! \brief state_ State of the Packman
  //!
  packman::State state_;

  //!
  //! \brief state_mutex_ Mutex for access to state_
  //!
  std::mutex state_mutex_;
};
