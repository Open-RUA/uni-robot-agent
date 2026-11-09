#include "arx_x5_controller/robot_status.hpp"

#include <array>
#include <iostream>
#include <stdexcept>

struct Status {
  int stamp = 42;
  std::array<double, 6> end_pos{};
  std::array<double, 7> joint_pos{};
  std::array<double, 7> joint_vel{};
  std::array<double, 7> joint_cur{};
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

bool equal(const Status& a, const Status& b) {
  return a.stamp == b.stamp && a.end_pos == b.end_pos && a.joint_pos == b.joint_pos &&
      a.joint_vel == b.joint_vel && a.joint_cur == b.joint_cur;
}

int main() {
  const auto clear = arx::x5::SummarizeVendorErrors({});
  require(clear.history_count == 0 && clear.codes.empty() && !clear.truncated,
          "an empty history remains clear");
  std::vector<int> repeated(1000000, 31);
  repeated[1] = 47;
  repeated.back() = 63;
  const auto errors = arx::x5::SummarizeVendorErrors(repeated);
  require(errors.history_count == 1000000 && errors.codes == std::vector<int>({31, 47, 63}) && !errors.truncated,
          "retain complete error kinds and original count without a giant response");
  for (int i = 0; i != 100; ++i) repeated[i] = i;
  const auto bounded = arx::x5::SummarizeVendorErrors(repeated);
  require(bounded.history_count == 1000000 && bounded.codes.size() == 64 && bounded.truncated,
          "unknown fault diversity must be bounded and explicitly marked truncated");
  const std::vector<double> pose{.1, .2, .3, .4, .5, .6};
  const std::vector<double> positions{1, 2, 3, 4, 5, 6, -3.27};
  const std::vector<double> velocities{11, 12, 13, 14, 15, 16, 17};
  const std::vector<double> currents{21, 22, 23, 24, 25, 26, 27};
  Status expected;
  expected.end_pos = {.1, .2, .3, .4, .5, .6};
  expected.joint_pos = {1, 2, 3, 4, 5, 6, -3.27};
  expected.joint_vel = {11, 12, 13, 14, 15, 16, 17};
  expected.joint_cur = {21, 22, 23, 24, 25, 26, 27};
  Status message;
  require(arx::x5::SerializeRobotStatus(pose, positions, velocities, currents, message),
          "complete feedback must serialize");
  require(equal(message, expected), "preserve pose, all seven channels and caller timestamp");

  // Check every short length independently, including empty vendor vectors.
  for (int field = 0; field != 4; ++field) {
    const std::size_t required = field == 0 ? 6 : 7;
    for (std::size_t size = 0; size < required; ++size) {
      std::array<std::vector<double>, 4> values{pose, positions, velocities, currents};
      values[field].resize(size);
      Status unchanged = expected;
      unchanged.end_pos.fill(-100);
      unchanged.joint_pos.fill(-200);
      unchanged.joint_vel.fill(-300);
      unchanged.joint_cur.fill(-400);
      message = unchanged;
      require(!arx::x5::SerializeRobotStatus(values[0], values[1], values[2], values[3], message),
              "short feedback must be rejected before indexing");
      require(equal(message, unchanged), "invalid feedback cannot partially overwrite status");
    }
  }

  auto extended_velocities = velocities;
  auto extended_currents = currents;
  extended_velocities.push_back(999);
  extended_currents.push_back(999);
  require(arx::x5::SerializeRobotStatus(pose, positions, extended_velocities, extended_currents, message),
          "trailing vendor values remain compatible");
  require(equal(message, expected), "publish the original seven channels from extended vectors");
  std::cout << "RobotStatus serialization regression passed\n";
}
