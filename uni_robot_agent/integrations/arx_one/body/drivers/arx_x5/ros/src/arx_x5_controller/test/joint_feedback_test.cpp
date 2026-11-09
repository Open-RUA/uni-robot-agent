#include "arx_x5_controller/joint_feedback.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

struct Status { double position; };
struct Driver {
  std::vector<double> positions{.1, .2, .3, .4, .5, .6, -6.0616455078125};
  std::vector<Status> status = std::vector<Status>(7, Status{42});
  int status_reads = 0;
  bool fail = false;
  std::vector<int> errors;
  std::vector<double> target;
  double catch_target = 123;
  std::vector<int> writes;
  std::vector<int> getErrorCode() { return errors; }
  void setJointPositions(std::vector<double> values) { target = values; writes.push_back(1); }
  void setCatch(double value) { catch_target = value; writes.push_back(2); }
  void setArmStatus(int mode) { writes.push_back(mode); }
  std::vector<double> getJointPositons() { return positions; }
  std::vector<Status> getJointStatus() {
    ++status_reads;
    if (fail) throw std::runtime_error("unavailable getter");
    return status;
  }
};

void require(bool condition) {
  if (!condition) throw std::runtime_error("joint feedback regression");
}

int main() {
  // Observed left/right closed/open samples: only the gripper is re-referenced.
  for (const auto& sample : std::vector<std::pair<double, double>>{
         {-6.0616455078125, -.02880096435546875},
         {-9.306095123291016, -3.2732505798339844},
         {-6.153963088989258, -.02079010009765625},
         {-9.431220054626465, -3.2980470657348633}}) {
    Driver driver;
    driver.positions[6] = sample.first;
    driver.status[6].position = sample.second;
    std::vector<double> result;
    require(arx::x5::ReadCommandJointPositions(driver, 2, result));
    require(result[6] == sample.second && driver.status_reads == 1);
    for (int i = 0; i != 6; ++i) require(result[i] == driver.positions[i]);
  }

  // Other end types do not inherit the end2 motor-zero convention.
  for (int end_type : {1, 3}) {
    Driver driver;
    driver.fail = true;
    std::vector<double> result;
    require(arx::x5::ReadCommandJointPositions(driver, end_type, result));
    require(result == driver.positions && driver.status_reads == 0);
  }

  // The type0 SDK uses motor_target = reference + command - 0.1.
  // Holding an arbitrary observed position must reproduce that position.
  for (double motor_position : {-3.4, -1.0, 0.0}) {
    Driver legacy;
    const double reference = 1.25;
    legacy.positions[6] = motor_position - reference;
    legacy.fail = true;  // Type0 does not need the end2 post-zero getter.
    require(arx::x5::HoldPostInitPosition(legacy, 0, 5));
    require(std::abs(reference + legacy.catch_target - 0.1 - motor_position) < 1e-12);
    require(legacy.status_reads == 0);
  }

  const std::vector<double> previous{123};
  for (int fault = 0; fault != 5; ++fault) {
    Driver driver;
    if (fault == 0) driver.positions.resize(6);
    if (fault == 1) driver.status.resize(6);
    if (fault == 2) driver.status[6].position = std::numeric_limits<double>::quiet_NaN();
    if (fault == 3) driver.status[6].position = std::numeric_limits<double>::infinity();
    if (fault == 4) driver.positions[2] = std::numeric_limits<double>::quiet_NaN();
    auto result = previous;
    require(!arx::x5::ReadCommandJointPositions(driver, 2, result));
    require(result == previous);  // No replacement by zero or legacy gripper.
  }
  Driver driver;
  driver.fail = true;
  auto result = previous;
  bool failed = false;
  try { arx::x5::ReadCommandJointPositions(driver, 2, result); }
  catch (const std::runtime_error&) { failed = true; }
  require(failed && result == previous);

  Driver startup;
  startup.status[6].position = -.021;
  require(arx::x5::HoldPostInitPosition(startup, 2, 5));
  require(startup.target == std::vector<double>({.1, .2, .3, .4, .5, .6}));
  require(startup.catch_target == -.021);  // Post-zero feedback, never stale offset.
  require(startup.writes == std::vector<int>({1, 2, 5}));  // Targets before mode switch.
  for (int fault = 0; fault != 3; ++fault) {
    Driver blocked;
    if (fault == 0) blocked.errors = {47};
    if (fault == 1) blocked.positions.resize(6);
    if (fault == 2) blocked.status[6].position = std::numeric_limits<double>::quiet_NaN();
    require(!arx::x5::HoldPostInitPosition(blocked, 2, 5));
    require(blocked.writes.empty());
  }
  std::cout << "joint feedback regression passed\n";
}
