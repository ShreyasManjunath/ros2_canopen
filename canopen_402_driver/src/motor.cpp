//    Copyright 2023 Christoph Hellmann Santos
//    Copyright 2023 Vishnuprasad Prachandabhanu
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

#include "canopen_402_driver/motor.hpp"
using namespace ros2_canopen;

bool Motor402::setTarget(double val)
{
  if (state_handler_.getState() == State402::Operation_Enable)
  {
    std::scoped_lock lock(mode_mutex_);
    return selected_mode_ && selected_mode_->setTarget(val);
  }
  // ===================== EXPERIMENT -- MUST NOT SHIP =====================
  // Crude test build. Revert or rewrite properly before any release.
  //
  // Question: does the CyberDrive act on the live value in 0x60FF when it
  // reaches Operation_Enabled, or latch the setpoint at the transition?
  // Undocumented -- the EDS gives [60FF] AccessType=rww and no semantics.
  //
  // Why it has to be written here: ModeForwardHelper::write() is the only code
  // that writes 0x60FF, and handleWrite() gates it on Operation_Enable, so it
  // never runs during a fault. The master RPDOs are transmission type 0x01
  // (SYNC-driven), so Lely re-serialises the stale OD entry every 20 ms on its
  // own. universal_set_value is the only way to change what is on the wire.
  //
  // 0x60FF is hardcoded with no mode dispatch. setTarget() is mode-agnostic and
  // also serves 0x607A (position) and 0x6071 (torque), so this is WRONG in
  // those modes. Acceptable only because this robot runs Profile Velocity
  // (0x6061 == 3 confirmed on both drives) and the test only needs velocity.
  //
  // Deliberately does not take mode_mutex_ and does not touch target_ or
  // has_target_. The helper's cached target and the master's OD diverge until
  // the first successful setTarget() reconciles them; only getTarget() can see
  // it, and that is acceptable for a test.
  this->driver->universal_set_value<int32_t>(0x60FF, 0x0, 0);
  // ======================= END EXPERIMENT =======================
  return false;
}
bool Motor402::isModeSupported(uint16_t mode)
{
  return mode != MotorBase::Homing && allocMode(mode);
}

bool Motor402::enterModeAndWait(uint16_t mode)
{
  bool okay = mode != MotorBase::Homing && switchMode(mode);
  return okay;
}

uint16_t Motor402::getMode()
{
  std::scoped_lock lock(mode_mutex_);
  return selected_mode_ ? selected_mode_->mode_id_ : (uint16_t)MotorBase::No_Mode;
}

bool Motor402::isModeSupportedByDevice(uint16_t mode)
{
// uint32_t supported_modes = 0;
//   try 
//   {
//     RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), 
//                 "Reading supported modes...");
//     supported_modes = driver->universal_get_value<uint32_t>(supported_drive_modes_index, 0x0);
//   }
//   catch (...) 
//   {
//     uint32_t supported_modes = 3; // hardcode velocity mode.
//     RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), 
//                 "Failed to read supported_drive_modes (0x6502). Hardcoding to mode %d.", supported_modes);
//   }
  uint32_t supported_modes = 4; // hardcode velocity mode. (0100)
    RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), 
                 "Hardcoding to mode %d.", supported_modes);
  bool supported = supported_modes & (1 << (mode - 1));
  bool below_max = mode <= 32;
  bool above_min = mode > 0;
  return below_max && above_min && supported;
}
void Motor402::registerMode(uint16_t id, const ModeSharedPtr & m)
{
    RCLCPP_INFO(rclcpp::get_logger("motor.cpp"), "registerMode: %d...", id);
  std::scoped_lock map_lock(map_mutex_);
  if (m && m->mode_id_ == id){
    modes_.insert(std::make_pair(id, m));
    RCLCPP_INFO(rclcpp::get_logger("motor.cpp"), "registerMode: %d inserted.", id);
  }
}

ModeSharedPtr Motor402::allocMode(uint16_t mode)
{
  ModeSharedPtr res;
  if (isModeSupportedByDevice(mode))
  {
    std::scoped_lock map_lock(map_mutex_);
    std::unordered_map<uint16_t, ModeSharedPtr>::iterator it = modes_.find(mode);
    if (it != modes_.end())
    {
      res = it->second;
    }
  }
  return res;
}

bool Motor402::switchMode(uint16_t mode)
{
  if (mode == MotorBase::No_Mode)
  {
    std::scoped_lock lock(mode_mutex_);
    selected_mode_.reset();
    try
    {  // try to set mode
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);
    }
    catch (...)
    {
    }
    if (enable_diagnostics_.load())
    {
      this->diag_collector_->addf("cia402_mode", "No mode selected: %d", mode);
    }
    return true;
  }

  ModeSharedPtr next_mode = allocMode(mode);
  if (!next_mode)
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode is not supported.");
    return false;
  }

  if (!next_mode->start())
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Could not  start mode.");
    return false;
  }

  {  // disable mode handler
    std::scoped_lock lock(mode_mutex_);

    if (mode_id_ == mode && selected_mode_ && selected_mode_->mode_id_ == mode)
    {
      // nothing to do
      return true;
    }

    selected_mode_.reset();
  }

  if (!switchState(switching_state_)) return false;

  driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);

  bool okay = false;

  {  // wait for switch
    std::unique_lock lock(mode_mutex_);

    std::chrono::steady_clock::time_point abstime =
      std::chrono::steady_clock::now() + std::chrono::seconds(60);
    if (monitor_mode_)
    {
      while (mode_id_ != mode && mode_cond_.wait_until(lock, abstime) == std::cv_status::no_timeout)
      {
      }
    }
    else
    {
      while (mode_id_ != mode && std::chrono::steady_clock::now() < abstime)
      {
        lock.unlock();                                                    // unlock inside loop
        driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);  // poll
        std::this_thread::sleep_for(std::chrono::milliseconds(20));       // wait some time
        lock.lock();
      }
    }

    if (mode_id_ == mode)
    {
      selected_mode_ = next_mode;
      okay = true;
      if (enable_diagnostics_.load())
      {
        this->diag_collector_->addf("cia402_mode", "Mode switched to: %d", mode);
      }
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode switch timed out.");
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode_id_);
      if (enable_diagnostics_.load())
      {
        this->diag_collector_->addf("cia402_mode", "Mode switch timed out: %d", mode);
      }
    }
  }

  if (!switchState(State402::Operation_Enable)) return false;

  return okay;
}

bool Motor402::switchState(const State402::InternalState & target)
{
  std::chrono::steady_clock::time_point abstime =
    std::chrono::steady_clock::now() + state_switch_timeout_;
  State402::InternalState state = state_handler_.getState();
  target_state_ = target;
  while (state != target_state_)
  {
    std::unique_lock lock(cw_mutex_);
    State402::InternalState next = State402::Unknown;
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), 
                "Transition step: Current State=%d, Target State=%d, Next State=%d, Calculated CW=0x%04X", 
                static_cast<int>(state), 
                static_cast<int>(target_state_.load()), 
                static_cast<int>(next), 
                static_cast<uint16_t>(control_word_));
    bool success = Command402::setTransition(control_word_, state, target_state_.load(), &next);
    if (!success)
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Could not set transition.");
      return false;
    }
    else if (enable_diagnostics_.load() && success)
    {
      this->diag_collector_->addf("cia402_state", "State switched to: %d", next);
    }
    lock.unlock();
    if (state != next && !state_handler_.waitForNewState(abstime, state))
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Transition timed out.");
      if (enable_diagnostics_.load())
      {
        this->diag_collector_->addf(
          "cia402_state", "State transition timed out: %d -> %d", state, next);
      }
      return false;
    }
  }
  return state == target;
}

bool Motor402::readState()
{
  uint16_t old_sw, sw = driver->universal_get_value<uint16_t>(
                     status_word_entry_index, 0x0);  // TODO: added error handling
  old_sw = status_word_.exchange(sw);

  // ===== EXPERIMENT -- MUST NOT SHIP =====
  // Root cause (confirmed via call-graph trace, not inference): the target_/
  // has_target_ reset placed in start() earlier only executes via
  // switchMode() or handleRecover() -- neither of which handleEnable() (the
  // function motion_supervisor actually calls, via /left_wheel/enable and
  // /right_wheel/enable) ever reaches. So that reset never ran on this
  // robot's real recovery path, and has_target_/target_ stayed frozen at
  // their last pre-fault value for the entire fault.
  //
  // Fix: reset at fault ENTRY instead of at recovery, right here where
  // readState() -- the only writer of state_handler_'s state, called once
  // per 20ms poll on this same thread -- observes the transition. old_state
  // is read before this cycle's read(sw); new_state is read(sw)'s return
  // value (the state it just computed and stored). Comparing the two is a
  // one-shot rising-edge check on "just left Operation_Enable", not a level
  // check: on the cycle state_ actually flips, old_state==Operation_Enable
  // and new_state!=Operation_Enable, so the branch fires exactly once. Every
  // later cycle while still faulted has old_state==new_state (whatever the
  // fault state is), so the branch stays closed until the next genuine
  // Operation_Enable -> non-Operation_Enable transition -- it cannot fight
  // setTarget()'s own logic on a legitimate re-enable, since re-enabling
  // moves toward Operation_Enable, not away from it.
  //
  // Race note (100Hz setTarget() zero-write experiment, different thread,
  // no mode_mutex_, vs this 20ms readState() poll, under mode_mutex_): at
  // the instant of the transition, controller_manager's setTarget() checks
  // state_handler_.getState() BEFORE taking mode_mutex_ -- a classic
  // check-then-act gap. If that unlocked check lands and reads the OLD
  // (still-Operation_Enable) state a moment before this reset runs, but its
  // subsequent selected_mode_->setTarget(val) call (under mode_mutex_) is
  // scheduled to run AFTER this reset releases the lock, that single
  // setTarget(val) call can "revive" has_target_=true with whatever val was
  // live at that instant (not the full pre-fault setpoint -- cmd_vel_out is
  // already mid-deceleration by then in every capture so far). Bounded to
  // at most one 100Hz controller_manager cycle (<=10ms) right at fault
  // entry, and self-limiting: the very next such cycle sees the now-visible
  // non-Operation_Enable state and returns to the zero-write branch, which
  // does not touch has_target_/target_. This edge has already fired once
  // for this fault episode and will not refire, so if the race is hit, the
  // revived (small, non-setpoint) target_ sits until the drive is actually
  // re-enabled. Flagging as a known, narrow residual window -- not closed
  // by this change -- rather than silently claiming it away.
  State402::InternalState const old_state = state_handler_.getState();
  State402::InternalState const new_state = state_handler_.read(sw);

  std::unique_lock lock(mode_mutex_);
  if (old_state == State402::Operation_Enable && new_state != State402::Operation_Enable)
  {
    if (selected_mode_)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("canopen_402_driver"),
        "readState: left Operation_Enable (state %d -> %d); resetting selected_mode_ target",
        static_cast<int>(old_state), static_cast<int>(new_state));
      selected_mode_->start();
    }
  }
  // ======================= END EXPERIMENT =======================
  uint16_t new_mode;
  new_mode = driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);
  // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode %hhi",new_mode);

  if (selected_mode_ && selected_mode_->mode_id_ == new_mode)
  {
    if (!selected_mode_->read(sw))
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode handler has error.");
    }
  }
  if (new_mode != mode_id_)
  {
    mode_id_ = new_mode;
    mode_cond_.notify_all();
  }
  if (selected_mode_ && selected_mode_->mode_id_ != new_mode)
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode does not match.");
  }
  if (sw & (1 << State402::SW_Internal_limit))
  {
    if (old_sw & (1 << State402::SW_Internal_limit))
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Internal limit active");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Internal limit active");
    }
  }

  return true;
}
void Motor402::handleRead() { readState(); }
void Motor402::handleWrite()
{
  std::scoped_lock lock(cw_mutex_);
  // Re-enabled under test; 9a3d9fa disabled it. 0x60FF keeps the stale setpoint
  // through a fault (setTarget() drops writes outside Operation_Enable), so bit 8
  // is the only thing telling the drive not to act on it. Do not re-disable blind.
  control_word_ |= (1 << Command402::CW_Halt);
  if (state_handler_.getState() == State402::Operation_Enable)
  {
    std::scoped_lock lock(mode_mutex_);
    Mode::OpModeAccesser cwa(control_word_);
    bool okay = false;
    if (selected_mode_ && selected_mode_->mode_id_ == mode_id_)
    {
      okay = selected_mode_->write(cwa);
    }
    else
    {
      cwa = 0;
    }
    if (okay)
    {
      // Cleared only on a successful mode write, i.e. has_target_ true.
      // Re-enabled under test; 9a3d9fa disabled it. See the set site above.
      control_word_ &= ~(1 << Command402::CW_Halt);
    }
  }
  if (start_fault_reset_.exchange(false))
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Fault reset");
    this->driver->universal_set_value<uint16_t>(
      control_word_entry_index, 0x0, control_word_ & ~(1 << Command402::CW_Fault_Reset));
  }
  else
  {
    // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Control Word %s",
    // std::bitset<16>{control_word_}.to_string());
    this->driver->universal_set_value<uint16_t>(control_word_entry_index, 0x0, control_word_);
  }
}
void Motor402::handleDiag()
{
  uint16_t sw = status_word_;
  State402::InternalState state = state_handler_.getState();

  switch (state)
  {
    case State402::Not_Ready_To_Switch_On:
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "Not ready to switch on");
      break;
    case State402::Switch_On_Disabled:
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "Switch on disabled");
      break;
    case State402::Ready_To_Switch_On:
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK, "Ready to switch on");
      break;
    case State402::Switched_On:
      // std::cout << "Motor operation is not enabled" << std::endl;
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Switched on");
      break;
    case State402::Operation_Enable:
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK, "Operation enabled");
      break;
    case State402::Quick_Stop_Active:
      // std::cout << "Quick stop is active" << std::endl;
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "Quick stop active");
      break;
    case State402::Fault:
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fault");
      break;
    case State402::Fault_Reaction_Active:
      // std::cout << "Motor has fault" << std::endl;
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fault reaction active");
      break;
    case State402::Unknown:
      // std::cout << "State is unknown" << std::endl;
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown state");
      break;
  }

  if (sw & (1 << State402::SW_Warning))
  {
    // std::cout << "Warning bit is set" << std::endl;
    this->diag_collector_->summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warning bit is set");
  }
  if (sw & (1 << State402::SW_Internal_limit))
  {
    // std::cout << "Internal limit active" << std::endl;
    this->diag_collector_->summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Internal limit active");
  }
}

bool Motor402::handleInit()
{
  for (std::unordered_map<uint16_t, AllocFuncType>::iterator it = mode_allocators_.begin();
       it != mode_allocators_.end(); ++it)
  {
    RCLCPP_INFO(rclcpp::get_logger("motor.cpp"), "handleInit, calling isModeSupportedByDevice");
    (it->second)();
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Read State");
  if (!readState())
  {
    std::cout << "Could not read motor state" << std::endl;
    return false;
  }
  {
    std::scoped_lock lock(cw_mutex_);
    control_word_ = 0;
    start_fault_reset_ = true;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Enable");
  if (!switchState(State402::Operation_Enable))
  {
    std::cout << "Could not enable motor" << std::endl;
    return false;
  }

  ModeSharedPtr m = allocMode(MotorBase::Homing);
  if (!m)
  {
    std::cout << "Homeing mode not supported" << std::endl;
    return true;  // homing not supported
  }

  HomingMode * homing = dynamic_cast<HomingMode *>(m.get());

  if (!homing)
  {
    std::cout << "Homing mode has incorrect handler" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Switch to homing");
  if (!switchMode(MotorBase::Homing))
  {
    std::cout << "Could not enter homing mode" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Execute homing");
  if (!homing->executeHoming())
  {
    std::cout << "Homing failed" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Switch no mode");
  if (!switchMode(MotorBase::No_Mode))
  {
    std::cout << "Could not enter no mode" << std::endl;
    return false;
  }
  return true;
}
bool Motor402::handleShutdown()
{
  switchMode(MotorBase::No_Mode);
  return switchState(State402::Switch_On_Disabled);
}
bool Motor402::handleHalt()
{
  State402::InternalState state = state_handler_.getState();
  std::scoped_lock lock(cw_mutex_);

  // do not demand quickstop in case of fault
  if (state == State402::Fault_Reaction_Active || state == State402::Fault) return false;

  if (state != State402::Operation_Enable)
  {
    target_state_ = state;
  }
  else
  {
    target_state_ = State402::Quick_Stop_Active;
    if (!Command402::setTransition(control_word_, state, State402::Quick_Stop_Active, 0))
    {
      std::cout << "Could not quick stop" << std::endl;
      return false;
    }
  }
  return true;
}
bool Motor402::handleRecover()
{
  start_fault_reset_ = true;
  {
    std::scoped_lock lock(mode_mutex_);
    // ===== DIAGNOSTIC -- MUST NOT SHIP =====
    // Settles whether selected_mode_->start() (where the target_/has_target_
    // fault-recovery reset lives) actually runs here, or is silently skipped
    // because selected_mode_ is null at this instant. Restructured to call
    // start() exactly once and reuse its result, so behavior is unchanged
    // from the original `if (selected_mode_ && !selected_mode_->start())`.
    bool const mode_selected = static_cast<bool>(selected_mode_);
    RCLCPP_INFO(
      rclcpp::get_logger("canopen_402_driver"), "handleRecover: selected_mode_ is %s",
      mode_selected ? "non-null" : "null");
    bool start_okay = true;
    if (mode_selected)
    {
      start_okay = selected_mode_->start();
      RCLCPP_INFO(
        rclcpp::get_logger("canopen_402_driver"), "handleRecover: selected_mode_->start() returned %s",
        start_okay ? "true" : "false");
    }
    // ===== END DIAGNOSTIC =====
    if (mode_selected && !start_okay)
    {
      std::cout << "Could not restart mode." << std::endl;
      return false;
    }
  }
  if (!switchState(State402::Operation_Enable))
  {
    std::cout << "Could not enable motor" << std::endl;
    return false;
  }
  return true;
}
bool Motor402::handleEnable()
{
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Enable: Read State");
  if (!readState())
  {
    std::cout << "Could not read motor state" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Enable");
  if (!switchState(State402::Operation_Enable))
  {
    std::cout << "Could not enable motor" << std::endl;
    return false;
  }
  return true;
}

bool Motor402::handleDisable()
{
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Disable: Read State");
  if (!readState())
  {
    std::cout << "Could not read motor state" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Disable");
  if (!switchState(State402::Switched_On))
  {
    std::cout << "Could not disable motor" << std::endl;
    return false;
  }
  return true;
}
