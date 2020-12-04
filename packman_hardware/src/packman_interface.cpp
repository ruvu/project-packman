// Copyright 2020 RUVU BV.

#include "./packman_interface.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/executor.hpp>

#include <string>

namespace packman_hardware
{
PackmanInterface::PackmanInterface(const std::string& can_device) : logger_(rclcpp::get_logger("packman_interface"))
{
  RCLCPP_INFO_STREAM(logger_, "Binding to socketcan interface " << can_device << " ...");
  if (!can_interface_.init(can_device, false))
  {
    throw std::runtime_error("Packman: Could not set-up socketcan interface on device " + can_device);
  }
  RCLCPP_INFO_STREAM(logger_, "Initialized socketcan interface on device " << can_device);

  std::atomic_init(&nmt_state_, NMTstate{});

  // Register CAN comm
  using std::placeholders::_1;
  can_listener_ =
      can_interface_.createMsgListener(can::MsgHeader(RxPDO1::ID), std::bind(&PackmanInterface::plcStateCb, this, _1));
  state_listener_ = can_interface_.createStateListener(std::bind(&PackmanInterface::CANStateCb, this, _1));
  heartbeat_listener_ = can_interface_.createMsgListener(can::MsgHeader(NMTstate::ID), [this](const can::Frame& frame) {
    // ROS_DEBUG_STREAM_NAMED(logger_, "Heartbeat: " << frame);
    NMTstate::Frame state(frame);
    nmt_state_.store(state.data);
    RCLCPP_DEBUG_STREAM(logger_, "Heartbeat: " << state);
  });
}

PackmanInterface::~PackmanInterface()
{
  RCLCPP_INFO(logger_, "Sending Stop");
  can_interface_.send(NMTcommand::Frame(PLC_NODE_ID, NMTcommand::Stop));
  // TODO: implmement sleep:
  // ros::Duration(0.2).sleep();
  puts("Shutting down the CAN device");
  can_interface_.shutdown();
  puts("Successfully shut down the CAN device");
}

void PackmanInterface::init()
{
  rclcpp::Rate r(1);
  while (rclcpp::ok())
  {
    // TODO: rclcpp::spin_once();
    r.sleep();
    auto state = nmt_state_.load();
    switch (state.command)
    {
      case NMTstate::Initialising:
        RCLCPP_INFO(logger_, "Sending Prepare");
        can_interface_.send(NMTcommand::Frame(PLC_NODE_ID, NMTcommand::Prepare));
        break;
      case NMTstate::PreOperational:
        RCLCPP_INFO(logger_, "Sending Start");
        can_interface_.send(NMTcommand::Frame(PLC_NODE_ID, NMTcommand::Start));
        break;
      case NMTstate::Operational:
        RCLCPP_INFO(logger_, "PLC operational, init finished");
        return;
      case NMTstate::Stopped:
        RCLCPP_INFO(logger_, "Sending Start");
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
  // RCLCPP_INFO_STREAM(logger_, "Sending " << pdo);
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
  // RCLCPP_INFO_STREAM(logger_, pdo);
}

void PackmanInterface::CANStateCb(const can::State& s)
{
  std::string err;
  can_interface_.translateError(s.internal_error, err);
  RCLCPP_INFO_STREAM(logger_, "CANState Callback: CAN state=" << s.driver_state << " error=" << s.internal_error << "("
                                                              << err << ") asio: " << s.error_code);
}
}  // namespace packman_hardware
