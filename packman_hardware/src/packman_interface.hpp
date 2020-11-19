// Copyright 2020 RUVU BV.

#pragma once

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include <string>

#include "./messages.hpp"
#include "./nmt.hpp"

namespace packman_hardware
{
class PackmanInterface
{
public:
  explicit PackmanInterface(const std::string& can_device);
  ~PackmanInterface();

  void init();
  RxPDO1 lastValues();
  void sendValues(TxPDO1 pdo);

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
  std::atomic<RxPDO1> state_;
};
}  // namespace packman_hardware
