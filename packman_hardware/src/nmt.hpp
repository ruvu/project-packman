// Copyright 2020 RUVU BV.

#pragma once

#include <ros/console.h>
#include <socketcan_interface/interface.h>

namespace packman_hardware
{
template <typename T>
struct FrameOverlay : public can::Frame
{
  T& data;
  FrameOverlay(const Header& h) : can::Frame(h, sizeof(T)), data(*(T*)can::Frame::c_array())
  {
    can::Frame::data.fill(0);
  }
  FrameOverlay(const can::Frame& f) : can::Frame(f), data(*(T*)can::Frame::c_array())
  {
  }
};

struct NMTcommand
{
  enum Command
  {
    Start = 0x01,     // Go to 'operational'
    Stop = 0x02,      // Go to 'stopped'
    Prepare = 0x80,   // Go to 'pre-operational'
    Reset = 0x81,     // Go to 'reset node'
    Reset_Com = 0x82  // Go to 'reset communication'
  };
  uint8_t command;
  uint8_t node_id;

  struct Frame : public FrameOverlay<NMTcommand>
  {
    Frame(uint8_t node_id, const Command& c) : FrameOverlay(can::Header())
    {
      data.command = c;
      data.node_id = node_id;
    }
  };
};

struct NMTstate
{
  static const unsigned int ID = 0x702;

  enum Command
  {
    Initialising = 0x00,  // Boot up
    Stopped = 0x04,
    Operational = 0x05,
    PreOperational = 0x7f
  };
  uint8_t command;

  struct Frame : public FrameOverlay<NMTstate>
  {
    Frame() noexcept : FrameOverlay(can::Frame())
    {
    }
    Frame(const Command& c) : FrameOverlay(can::Header())
    {
      data.command = c;
    }
    Frame(const can::Frame& f) : FrameOverlay(f)
    {
    }
  };
};

std::ostream& operator<<(std::ostream& os, const NMTstate& state);

std::ostream& operator<<(std::ostream& os, const NMTstate::Frame& state);
}  // namespace packman_hardware
