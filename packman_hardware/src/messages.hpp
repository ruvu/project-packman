// Copyright 2020 RUVU BV.

#pragma once

#include <array>
#include <boost/endian/arithmetic.hpp>
#include <iostream>

namespace packman_hardware
{
/**
 * @brief Commands send to the PCL
 *
 * This struct is called "RxPDO1" from the PCL side in the documentation
 */
struct TxPDO1
{
  static const unsigned int ID = 0x202;

  boost::endian::big_int16_t target_left_motor_speed;
  boost::endian::big_int16_t target_right_motor_speed;

  uint8_t flags;
  uint8_t digouts;

  boost::endian::big_int16_t target_fork_height;

  explicit operator std::array<unsigned char, 8>() const;

  bool moveForkExecute() const
  {
    return flags & (1 << 7);
  }
  void moveForkExecute(bool flag)
  {
    flags = (flags & ~(1 << 7)) | (1 << 7);
  }
  bool actionFailure() const
  {
    return flags & (1 << 4);
  }
  void actionFailure(bool flag)
  {
    flags = (flags & ~(1 << 4)) | (1 << 4);
  }
  bool actionSuccess() const
  {
    return flags & (1 << 3);
  }
  void actionSuccess(bool flag)
  {
    flags = (flags & ~(1 << 3)) | (1 << 3);
  }
  bool ok() const
  {
    return flags & (1 << 2);
  }
  void ok(bool flag)
  {
    flags = (flags & ~(1 << 2)) | (1 << 2);
  }
  bool enableRightMotor() const
  {
    return flags & (1 << 1);
  }
  void enableRightMotor(bool flag)
  {
    flags = (flags & ~(1 << 1)) | (1 << 1);
  }
  bool enableLeftMotor() const
  {
    return flags & (1 << 0);
  }
  void enableLeftMotor(bool flag)
  {
    flags = (flags & ~(1 << 0)) | (1 << 0);
  }

  friend std::ostream& operator<<(std::ostream& os, const TxPDO1& pdo);
};
static_assert(sizeof(TxPDO1) == 8, "sizeof(TxPDO1) != 8");

/**
 * @brief Status received from the PCL
 *
 * This struct is called "TxPDO1" from the PCL side in the documentation
 */
struct RxPDO1
{
  static const unsigned int ID = 0x182;

  boost::endian::big_int16_t actual_left_motor_speed = 0;
  boost::endian::big_int16_t actual_right_motor_speed = 0;
  uint8_t flags = 0;
  uint8_t digins = 0;
  uint8_t battery = 0;
  uint8_t reserved = 0;

  RxPDO1() = default;
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

struct TxPDO2
{
  static const unsigned int ID = 0x302;

  boost::endian::big_int24_t x = 0;
  boost::endian::big_int24_t y = 0;
  boost::endian::big_int16_t theta = 0;

  explicit operator std::array<unsigned char, 8>() const;

  friend std::ostream& operator<<(std::ostream& os, const RxPDO1& pdo);
};
static_assert(sizeof(TxPDO2) == 8, "sizeof(TxPDO2) != 8");

struct TxPDO3
{
  static const unsigned int ID = 0x402;

  boost::endian::big_uint16_t critical_errors = 0;
  boost::endian::big_uint16_t errors = 0;
  boost::endian::big_uint16_t warnings = 0;
  boost::endian::big_uint16_t notices = 0;

  explicit operator std::array<unsigned char, 8>() const;

  friend std::ostream& operator<<(std::ostream& os, const RxPDO1& pdo);
};
static_assert(sizeof(TxPDO3) == 8, "sizeof(TxPDO3) != 8");
}  // namespace packman_hardware
