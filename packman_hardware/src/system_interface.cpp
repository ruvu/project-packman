#include "system_interface.hpp"

#include <type_traits>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using hardware_interface::return_type;

namespace packman_hardware
{
// TODO: make can0 configurable
SystemInterface::SystemInterface()
  : logger_(rclcpp::get_logger("SystemInterface")), interface_("can0"), clock_(RCL_STEADY_TIME)
{
}

hardware_interface::return_type SystemInterface::configure(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(logger_, "Configure...");
  auto err = configure_default(info);
  if (static_cast<bool>(err))
    return err;

  joints_.resize(info_.joints.size());
  commands_.resize(info_.joints.size());

  interface_.init();

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &commands_[i].velocity));
  }

  return command_interfaces;
}

hardware_interface::return_type SystemInterface::start()
{
  RCLCPP_INFO(logger_, "Start!");
  return return_type::OK;
}

hardware_interface::return_type SystemInterface::stop()
{
  RCLCPP_INFO(logger_, "Stop!");
  return return_type::OK;
}

hardware_interface::return_type SystemInterface::read()
{
  auto pdo = interface_.lastValues();

  joints_[0].velocity = pdo.actual_left_motor_speed / 60. * 2 * M_PI / 1e3;
  // joints_[0].position += joints_[0].velocity * period.toSec();
  joints_[1].velocity = pdo.actual_right_motor_speed / 60. * 2 * M_PI / 1e3;
  // joints_[1].position += joints_[1].velocity * period.toSec();
  return return_type::OK;
}

hardware_interface::return_type SystemInterface::write()
{
  TxPDO1 pdo{};
  pdo.enableLeftMotor(true);
  pdo.enableRightMotor(true);
  pdo.ok(true);
  pdo.target_left_motor_speed = commands_[0].velocity * 60 / 2 / M_PI * 1e3;
  pdo.target_right_motor_speed = commands_[1].velocity * 60 / 2 / M_PI * 1e3;
  interface_.sendValues(pdo);
  return return_type::OK;
}
}  // namespace packman_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(packman_hardware::SystemInterface, hardware_interface::SystemInterface)
