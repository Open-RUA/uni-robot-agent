#pragma once
#include "arx_x5_controller/joint_feedback.hpp"
#include <cstdint>
#include <stdexcept>

namespace arx::x5 {
// Operator-only SOFT mode, not motor power-off. All seven motors lose holding
// torque. Only the normal controller uses this gate; ROS callbacks are serial.
class TeachGate {
 public:
  bool active() const { return active_; }
  bool accepts(std::int64_t stamp) const {
    return !active_ && (barrier_ == 0 || stamp > barrier_);
  }
  template <typename Driver>
  void enter(Driver& driver) {
    if (active_) throw std::runtime_error("already in SOFT teaching mode");
    if (!driver.getErrorCode().empty()) throw std::runtime_error("controller fault");
    active_ = true;  // Fail closed before changing vendor state.
    driver.setArmStatus(0);
  }
  template <typename Driver>
  void resume(Driver& driver, int end_type, std::int64_t now) {
    if (!active_) throw std::runtime_error("not in SOFT teaching mode");
    const auto velocities = driver.getJointVelocities();
    if (velocities.size() != 7 || !std::all_of(velocities.begin(), velocities.end(),
        [](double v) { return std::isfinite(v) && std::abs(v) <= .03; }))
      throw std::runtime_error("arm/gripper must be stationary before hold");
    if (!HoldPostInitPosition(driver, end_type, 5))
      throw std::runtime_error("cannot hold current measured pose");
    barrier_ = now;  // Reject queued commands from before resume.
    active_ = false;
  }
 private:
  bool active_ = false;
  std::int64_t barrier_ = 0;
};
}
