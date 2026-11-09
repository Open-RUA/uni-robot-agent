#include "arx_x5_controller/X5Controller.hpp"
#include "arx_x5_controller/robot_status.hpp"

#include <algorithm>
#include <stdexcept>

namespace arx::x5 {
void X5Controller::DriverCommandCallback(
    const std::shared_ptr<arx5_arm_msg::srv::DriverCommand::Request> request,
    std::shared_ptr<arx5_arm_msg::srv::DriverCommand::Response> response) {
  try {
    if (!interfaces_ptr_) {
      throw std::runtime_error("Driver is shutting down");
    }
    const auto& operation = request->operation;
    const auto& values = request->values;
    auto require_size = [&](std::size_t count) {
      if (values.size() != count) {
        throw std::invalid_argument(operation + " requires " +
                                    std::to_string(count) + " values");
      }
      if (!std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); })) {
        throw std::invalid_argument("All values must be finite");
      }
    };

    const bool teach_operation = operation == "get_teach_state" || operation == "enter_soft_teach" || operation == "resume_teach_hold";
    if (teach_operation && this->get_parameter("arm_control_type").as_string() != "normal")
      throw std::runtime_error("teaching is only supported by the normal controller");
    if (teach_gate_.active() && !teach_operation && operation.rfind("get_", 0) != 0 && operation != "forward_kinematics")
      throw std::runtime_error("SOFT teaching mode blocks ordinary commands");
    if (operation == "get_teach_state") {
      require_size(0);
      response->values = {1., teach_gate_.active() ? 1. : 0.};
    } else if (operation == "enter_soft_teach") {
      require_size(0);
      teach_gate_.enter(*interfaces_ptr_);
      response->values = {1., 1.};
    } else if (operation == "resume_teach_hold") {
      require_size(0);
      teach_gate_.resume(*interfaces_ptr_, arm_end_type_, this->get_clock()->now().nanoseconds());
      response->values = {1., 0.};
    } else if (operation == "get_error_summary") {
      require_size(0);
      const auto summary = SummarizeVendorErrors(interfaces_ptr_->getErrorCode());
      response->error_codes.assign(summary.codes.begin(), summary.codes.end());
      response->values = {static_cast<double>(summary.history_count), summary.truncated ? 1.0 : 0.0};
    } else if (operation == "get_errors") {
      require_size(0);
      const auto errors = interfaces_ptr_->getErrorCode();
      response->error_codes.assign(errors.begin(), errors.end());
    } else if (operation == "get_joint_status") {
      require_size(0);
      for (const auto& joint : interfaces_ptr_->getJointStatus()) {
        response->values.insert(response->values.end(),
                                {joint.position, joint.velocity,
                                 joint.torque, joint.current});
      }
    } else if (operation == "get_joint_positions") {
      require_size(0);
      response->values = interfaces_ptr_->getJointPositons();
    } else if (operation == "get_joint_velocities") {
      require_size(0);
      response->values = interfaces_ptr_->getJointVelocities();
    } else if (operation == "get_joint_current") {
      require_size(0);
      response->values = interfaces_ptr_->getJointCurrent();
    } else if (operation == "get_end_pose") {
      require_size(0);
      response->values = solve::Isometry2Xyzrpy(interfaces_ptr_->getEndPose());
    } else if (operation == "set_home") {
      require_size(6);
      interfaces_ptr_->setHomePositions(values);
      go_home_positions_ = values;
      this->set_parameter(rclcpp::Parameter("go_home_position", values));
    } else if (operation == "set_joint_positions") {
      require_size(6);
      interfaces_ptr_->setJointPositions(values);
    } else if (operation == "set_end_pose") {
      require_size(6);
      double pose[6];
      std::copy(values.begin(), values.end(), pose);
      interfaces_ptr_->setEndPose(solve::Xyzrpy2Isometry(pose));
    } else if (operation == "set_catch") {
      require_size(1);
      interfaces_ptr_->setCatch(values[0]);
    } else if (operation == "set_catch_torque") {
      require_size(1);
      interfaces_ptr_->setCatchTorque(values[0]);
    } else if (operation == "set_mode") {
      require_size(1);
      if (values[0] != std::floor(values[0]) || values[0] < 0 || values[0] > 5) {
        throw std::invalid_argument("Mode must be an integer from 0 to 5");
      }
      interfaces_ptr_->setArmStatus(static_cast<int>(values[0]));
    } else if (operation == "arx_x") {
      require_size(3);
      interfaces_ptr_->arx_x(values[0], values[1], values[2]);
    } else if (operation == "forward_kinematics") {
      require_size(6);
      response->values = tools_ptr_->ForwardKinematicsRpy(values);
    } else {
      throw std::invalid_argument("Unknown driver operation: " + operation);
    }
    response->success = true;
    response->message = "ok";
  } catch (const std::exception& error) {
    response->success = false;
    response->message = error.what();
    response->values.clear();
    response->error_codes.clear();
  }
}
}
