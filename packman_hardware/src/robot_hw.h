// Copyright 2020 RUVU BV.

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <string>
#include <vector>

#include "./messages.h"
#include "./packman_interface.h"

namespace packman_hardware
{
struct Joint
{
  double position;
  double velocity;
  double effort;
};

struct Command
{
  double velocity;
};

class RobotHW : public hardware_interface::RobotHW
{
public:
  //!
  //! \brief Packman Robot hardware interface
  //!
  explicit RobotHW(const std::string& can_device);

  bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& /*robot_hw_nh*/) override;

  //!
  //! \brief read Packman data to JointState interface
  //!
  void read(const ros::Time& /*time*/, const ros::Duration& period) override;

  //!
  //! \brief write VelocityJointInterface data to Packman
  //!
  void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

private:
  // Interface to ROS_CONTROL
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  std::vector<Joint> joints_;
  std::vector<Command> commands_;

  //!
  //! \brief can_interface_ Socketcan interface (can connection)
  //!
  PackmanInterface interface_;
};
}  // namespace packman_hardware
