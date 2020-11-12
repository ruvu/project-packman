// Copyright 2020 RUVU BV.

#include "./nmt.hpp"

namespace packman_hardware
{
std::ostream& operator<<(std::ostream& os, const NMTstate& state)
{
  os << "NMTstate: ";
  switch (state.command)
  {
    case NMTstate::Initialising:
      return os << "Initialising";
    case NMTstate::Stopped:
      return os << "Stopped";
    case NMTstate::Operational:
      return os << "Operational";
    case NMTstate::PreOperational:
      return os << "PreOperational";
    default:
      std::stringstream ss;
      ss << "Unknown NMTstate: " << +state.command;
      throw std::runtime_error(ss.str());
  }
}

std::ostream& operator<<(std::ostream& os, const NMTstate::Frame& state)
{
  return os << state.data;
}
}  // namespace packman_hardware
