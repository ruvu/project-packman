#pragma once

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/clock.hpp>
#include "./packman_interface.hpp"

namespace packman_hardware
{
class SystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  SystemInterface();

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  struct Joint
  {
    double velocity;
  };
  struct Command
  {
    double velocity;
  };

  rclcpp::Logger logger_;
  std::vector<Joint> joints_{};
  std::vector<Command> commands_{};
  PackmanInterface interface_{};
  rclcpp::Clock clock_{ RCL_STEADY_TIME };
};
}  // namespace packman_hardware
