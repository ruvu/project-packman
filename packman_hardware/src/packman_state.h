// Copyright 2020 RUVU BV.

#pragma once

#include <vector>
#include <string>
#include <socketcan_interface/interface.h>
#include <socketcan_interface/string.h>

namespace packman
{
const std::vector<std::string> DRIVE_NAMES = { "left_wheel", "right_wheel" };
enum DriveIds
{
  LEFT,
  RIGHT,
  NUM_DRIVES
};

//!
//! \brief The PlcStatus struct holds the status information of the Packman PLC
//!
struct PlcStatus
{
  PlcStatus() = default;

  bool move_fork_failure_ = false;
  bool move_fork_success_ = false;
  bool activate_navcore_ = false;
  bool manual_mode_ = false;
  bool left_drive_ready_ = false;
  bool right_drive_ready_ = false;
  bool general_purpose_inputs_[8] = { false };
  int battery_state_of_charge_ = 0;  // in %
};

//!
//! \brief The PlcState struct holds the state information of the Packman PLC
//!
const unsigned int PLC_STATE_ID = 0x182;
struct PlcState
{
  void set(const can::Frame& f)
  {
    // TODO(paul): deserialize can frame and cast on PlcState
    received_ = true;
  }

  PlcStatus status_;
  double motor_speeds_[2] = { 0, 0 };
  bool received_ = false;
};

enum TopLevelStatus
{
  WARNING,
  ERROR,
  OK,
  STALE
};

//!
//! \brief The State class Holds the state information of the Packman PLC
//!
struct State
{
  PlcState plc_;

  TopLevelStatus getTopLevelStatus() const
  {
    if (!plc_.received_)
    {
      return TopLevelStatus::STALE;
    }

    if ((plc_.status_.left_drive_ready_) && (plc_.status_.right_drive_ready_))
    {
      return TopLevelStatus::OK;
    }

    return TopLevelStatus::ERROR;
  }

  // These are mutated on the controls thread only. SI units
  struct Joint
  {
    double position = 0;
    double velocity = 0;
    double effort = 0;
    double velocity_command = 0;

    Joint() = default;
  } joints_[packman::NUM_DRIVES];  // NOLINT
};
}  // namespace packman
