// Copyright 2020 RUVU BV.

#include "./packman_interface.hpp"

#include <ros/console.h>
#include <ros/node_handle.h>

#include <string>

namespace packman_hardware
{
const auto name = "packman_interface";

PackmanInterface::PackmanInterface(const std::string& can_device)
{
  ROS_INFO_STREAM_NAMED(name, "Binding to socketcan interface " << can_device << " ...");
  if (!can_interface_.init(can_device, false, can::NoSettings::create()))
  {
    throw std::runtime_error("Packman: Could not set-up socketcan interface on device " + can_device);
  }
  ROS_INFO_STREAM_NAMED(name, "Initialized socketcan interface on device " << can_device);

  std::atomic_init(&nmt_state_, NMTstate{});

  // Register CAN comm
  using std::placeholders::_1;
  can_listener_ =
      can_interface_.createMsgListener(can::MsgHeader(RxPDO1::ID), std::bind(&PackmanInterface::plcStateCb, this, _1));
  state_listener_ = can_interface_.createStateListener(std::bind(&PackmanInterface::CANStateCb, this, _1));
  heartbeat_listener_ = can_interface_.createMsgListener(can::MsgHeader(NMTstate::ID), [this](const can::Frame& frame) {
    // ROS_DEBUG_STREAM_NAMED(name, "Heartbeat: " << frame);
    NMTstate::Frame state(frame);
    nmt_state_.store(state.data);
    ROS_DEBUG_STREAM_NAMED(name, "Heartbeat: " << state);
  });
}

PackmanInterface::~PackmanInterface()
{
  ROS_INFO_NAMED(name, "Sending Stop");
  can_interface_.send(NMTcommand::Frame(PLC_NODE_ID, NMTcommand::Stop));
  ros::Duration(0.2).sleep();
  puts("Shutting down the CAN device");
  can_interface_.shutdown();
  puts("Successfully shut down the CAN device");
}

void PackmanInterface::init()
{
  ros::Rate r(1);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    auto state = nmt_state_.load();
    switch (state.command)
    {
      case NMTstate::Initialising:
        ROS_INFO_NAMED(name, "Sending Prepare");
        can_interface_.send(NMTcommand::Frame(PLC_NODE_ID, NMTcommand::Prepare));
        break;
      case NMTstate::PreOperational:
        ROS_INFO_NAMED(name, "Sending Start");
        can_interface_.send(NMTcommand::Frame(PLC_NODE_ID, NMTcommand::Start));
        break;
      case NMTstate::Operational:
        ROS_INFO_NAMED(name, "PLC operational, init finished");
        return;
      case NMTstate::Stopped:
        ROS_INFO_NAMED(name, "Sending Start");
        can_interface_.send(NMTcommand::Frame(PLC_NODE_ID, NMTcommand::Start));
        break;
      default:
        std::stringstream ss;
        ss << "Unknown PLC state: " << state;
        throw std::runtime_error(ss.str());
    }
  }

  // TODO(ramon): Set heartbeat cycle time to 0.3s using a SDO to 1017 (in ms)
}

RxPDO1 PackmanInterface::lastValues()
{
  return state_.load();
}

void PackmanInterface::sendValues(TxPDO1 pdo)
{
  // ROS_INFO_STREAM_NAMED(name, "Sending " << pdo);
  auto data = static_cast<decltype(can::Frame::data)>(pdo);
  can::Frame frame(can::Header(TxPDO1::ID, false, false, false), 8);
  frame.data = data;

  can_interface_.send(frame);
}

void PackmanInterface::plcStateCb(const can::Frame& f)
{
  // ROS_INFO_STREAM("Received PlcState frame " << f);

  RxPDO1 pdo(f.data);
  state_.store(RxPDO1(f.data));
  // ROS_INFO_STREAM_NAMED(name, pdo);
}

void PackmanInterface::CANStateCb(const can::State& s)
{
  std::string err;
  can_interface_.translateError(s.internal_error, err);
  ROS_INFO_STREAM_NAMED(name, "CANState Callback: CAN state=" << s.driver_state << " error=" << s.internal_error << "("
                                                              << err << ") asio: " << s.error_code);
}
}  // namespace packman_hardware
