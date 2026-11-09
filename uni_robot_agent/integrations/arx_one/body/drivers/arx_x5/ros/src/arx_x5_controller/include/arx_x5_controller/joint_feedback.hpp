#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace arx::x5 {

// The end_type=2 vendor Init stores a reference and then zeros the gripper
// motor. getJointPositons() still subtracts that old reference from element 6;
// getJointStatus()[6].position is in the post-zero setCatch command frame.
// For end_type=0, CatchPositionCtrl targets reference + setCatch - 0.1,
// while the legacy getter returns position - reference. Add the exact SDK
// bias so feedback and hold targets use the same command coordinate.
// Preserve the legacy getter for other end types and for the six arm joints.
// Never estimate a reference from endpoint calibration or silently fall back
// to the offset value when the post-zero feedback is unavailable.
template <typename Driver>
bool ReadCommandJointPositions(Driver& driver, int end_type,
                               std::vector<double>& output) {
  auto positions = driver.getJointPositons();
  if (positions.size() != 7) return false;
  if (end_type == 2) {
    const auto status = driver.getJointStatus();
    if (status.size() != 7) return false;
    positions[6] = status[6].position;
  } else if (end_type == 0) {
    positions[6] += 0.1;
  }
  if (!std::all_of(positions.begin(), positions.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }
  output = std::move(positions);
  return true;
}

// Vendor Init has already established the gripper reference. Leaving its
// GO_HOME mode active overwrites setCatch with zero every update. Hold the
// measured post-init pose in joint mode; an explicit Body joint_zero operation
// then supplies the open-gripper target. This does not skip vendor Init or
// clear a latched motor fault.
template <typename Driver>
bool HoldPostInitPosition(Driver& driver, int end_type, int joint_mode) {
  if (!driver.getErrorCode().empty()) return false;
  std::vector<double> positions;
  if (!ReadCommandJointPositions(driver, end_type, positions)) return false;
  driver.setJointPositions(std::vector<double>(positions.begin(), positions.begin() + 6));
  driver.setCatch(positions[6]);
  driver.setArmStatus(joint_mode);
  return true;
}

}  // namespace arx::x5
