//    Copyright 2023 Christoph Hellmann Santos
//    Copyright 2014-2022 Authors of ros_canopen
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#ifndef MODE_TARGET_HELPER_HPP
#define MODE_TARGET_HELPER_HPP

#include <atomic>
#include <boost/numeric/conversion/cast.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "canopen_402_driver/mode.hpp"

namespace ros2_canopen
{

template <typename T>
class ModeTargetHelper : public Mode
{
  T target_;
  std::atomic<bool> has_target_;

public:
  ModeTargetHelper(uint16_t mode) : Mode(mode) {}
  bool hasTarget() { return has_target_; }
  T getTarget() { return target_; }
  virtual bool setTarget(const double & val)
  {
    if (std::isnan(val))
    {
      // std::cout << "canopen_402 target command is not a number" << std::endl;
      RCLCPP_DEBUG(rclcpp::get_logger("canopen_402_target"), "Target command is not a number");
      return false;
    }

    using boost::numeric_cast;
    using boost::numeric::negative_overflow;
    using boost::numeric::positive_overflow;

    try
    {
      target_ = numeric_cast<T>(val);
    }
    catch (negative_overflow &)
    {
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to min limit" << std::endl;
      target_ = std::numeric_limits<T>::min();
    }
    catch (positive_overflow &)
    {
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to max limit" << std::endl;
      target_ = std::numeric_limits<T>::max();
    }
    catch (...)
    {
      std::cout << "canopen_402 Was not able to cast command " << val << std::endl;
      return false;
    }

    has_target_ = true;
    return true;
  }
  virtual bool start()
  {
    // ===================== EXPERIMENT -- MUST NOT SHIP =====================
    // Crude test build. Revert or rewrite properly before any release.
    //
    // has_target_ = false already blocked writes of the stale target during a
    // fault (that part is correct and is why the zero-write experiment in
    // Motor402::setTarget() works). But target_ itself was never reset here,
    // only has_target_ -- so it froze at its last pre-fault value. The FIRST
    // setTarget() to succeed after recovery sets has_target_ = true again,
    // but until that happens, anything that reads getTarget() while
    // hasTarget() is momentarily true from an earlier partial state sees the
    // frozen stale value, not a fresh one. Confirmed on hardware: the CAN
    // capture's first accepted 0x60FF write after recovery carried
    // 80-90% of the pre-fault setpoint, not a small ramp-start value, even
    // though cmd_vel_out (ROS side, upstream of this) was verified zero at
    // that exact instant.
    //
    // Hardcoded to T{} (zero) with no mode awareness -- correct for velocity
    // mode (zero velocity is always a safe target), NOT necessarily correct
    // for position mode, where "reset to zero" means "target absolute
    // position zero", not "hold here". Acceptable only because this robot's
    // drives are fixed in Profile Velocity mode (bus.yml velocity_mode: 3);
    // upstream would need this mode-aware (e.g. only zero for velocity/
    // torque modes, not position).
    //
    // This also fires on every NORMAL (non-fault) mode switch via
    // switchMode()'s next_mode->start() call (motor.cpp:147), not just
    // handleRecover()'s (motor.cpp:497) -- start() has no fault-specific
    // variant, and both call sites use this same, single implementation.
    // Harmless for velocity mode: a freshly-selected mode starting at zero
    // target is already the expected/desired behavior on a normal switch.
    has_target_ = false;
    target_ = T{};
    // ======================= END EXPERIMENT =======================
    return true;
  }
};
}  // namespace ros2_canopen

#endif  // MODE_TARGET_HELPER_HPP
