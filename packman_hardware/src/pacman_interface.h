#pragma once

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include "./nmt.hpp"

class PacmanInterface
{
public:
  PacmanInterface();
  ~PacmanInterface();

  void init();

private:
  static const uint8_t PLC_NODE_ID = 0x02;

  void plcStateCb(const can::Frame& f);
  void CANStateCb(const can::State& s);

  can::ThreadedSocketCANInterface can_interface_;
  can::CommInterface::FrameListenerConstSharedPtr can_listener_;
  can::StateInterface::StateListenerConstSharedPtr state_listener_;
  can::CommInterface::FrameListenerConstSharedPtr heartbeat_listener_;

  //  std::atomic<NMTstate::Frame> nmt_state_;
  std::atomic<NMTstate> nmt_state_;
};
