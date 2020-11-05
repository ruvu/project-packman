// Copyright 2020 RUVU BV.

#pragma once

#include <array>
#include <boost/endian/arithmetic.hpp>
#include <iostream>

struct TxPDO1
{
  boost::endian::big_int16_t actual_left_motor_speed;
  boost::endian::big_int16_t actual_right_motor_speed;
  uint8_t flags;
  uint8_t digins;
  uint8_t battery;
  uint8_t reserved;
};
static_assert(sizeof(TxPDO1) == 8, "sizeof(TxPDO1) != 8");

struct RxPDO1
{
public:
  boost::endian::big_int16_t target_left_motor_speed;
  boost::endian::big_int16_t target_right_motor_speed;
  uint8_t flags;
  uint8_t digins;
  uint8_t battery;

  static const unsigned int ID = 0x182;

private:
  uint8_t reserved;

public:
  explicit RxPDO1(const std::array<unsigned char, 8> data);

  bool moveForkFailure() const
  {
    return flags & (1 << 7);
  }
  bool moveForkSuccess() const
  {
    return flags & (1 << 6);
  }
  bool activateANT() const
  {
    return flags & (1 << 3);
  }
  bool automatic() const
  {
    return flags & (1 << 2);
  }
  bool rightMotorReady() const
  {
    return flags & (1 << 1);
  }
  bool leftMotorReady() const
  {
    return flags & (1 << 0);
  }

  friend std::ostream& operator<<(std::ostream& os, const RxPDO1& pdo);
};
static_assert(sizeof(RxPDO1) == 8, "sizeof(RxPDO1) != 8");
