// Copyright 2020 RUVU BV.

#include "./messages.h"

#include <bitset>

RxPDO1::RxPDO1(const std::array<unsigned char, 8> data)
{
  static_assert(sizeof(*this) == sizeof(data), "sizeof(RxPDO1) != sizeof(data)");
  *this = *reinterpret_cast<RxPDO1 const*>(data.data());
}

std::ostream& operator<<(std::ostream& os, const RxPDO1& pdo)
{
  os << "RxPDO1:\n";
  os << "\tmotor left: " << pdo.target_left_motor_speed << " rpm \n";
  os << "\tmotor right: " << pdo.target_right_motor_speed << " rpm \n";

  os << "\tmoveForkFailure: " << pdo.moveForkFailure() << '\n';
  os << "\tmoveForkSuccess: " << pdo.moveForkSuccess() << '\n';
  os << "\tactivateANT: " << pdo.activateANT() << '\n';
  os << "\tautomatic: " << pdo.automatic() << '\n';
  os << "\trightMotorReady: " << pdo.rightMotorReady() << '\n';
  os << "\tleftMotorReady: " << pdo.leftMotorReady() << '\n';

  os << "\tdigins: " << std::bitset<8>(pdo.digins) << '\n';
  os << "\tbattery: " << +pdo.battery << '%';

  return os;
}
