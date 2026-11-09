#pragma once

#include <algorithm>
#include <vector>
#include <unordered_set>

namespace arx::x5 {

struct VendorErrorSummary {
  std::size_t history_count = 0;
  std::vector<int> codes;
  bool truncated = false;
};

// The vendor accumulates errors for the life of its driver. Keep fault evidence
// and its count, without serializing millions of repeated integers over DDS.
inline VendorErrorSummary SummarizeVendorErrors(const std::vector<int>& history) {
  VendorErrorSummary result;
  result.history_count = history.size();
  std::unordered_set<int> unique;
  for (int code : history) {
    if (unique.count(code)) continue;
    if (unique.size() == 64) {
      result.truncated = true;
      break;
    }
    unique.insert(code);
  }
  result.codes.assign(unique.begin(), unique.end());
  std::sort(result.codes.begin(), result.codes.end());
  return result;
}

// Leave the message untouched unless every feedback vector can fill its field.
template <typename Status>
bool SerializeRobotStatus(const std::vector<double>& xyzrpy,
                          const std::vector<double>& positions,
                          const std::vector<double>& velocities,
                          const std::vector<double>& currents,
                          Status& message) {
  if (xyzrpy.size() < message.end_pos.size() ||
      positions.size() < message.joint_pos.size() ||
      velocities.size() < message.joint_vel.size() ||
      currents.size() < message.joint_cur.size()) {
    return false;
  }
  std::copy_n(xyzrpy.begin(), message.end_pos.size(), message.end_pos.begin());
  std::copy_n(positions.begin(), message.joint_pos.size(), message.joint_pos.begin());
  std::copy_n(velocities.begin(), message.joint_vel.size(), message.joint_vel.begin());
  std::copy_n(currents.begin(), message.joint_cur.size(), message.joint_cur.begin());
  return true;
}

}  // namespace arx::x5
