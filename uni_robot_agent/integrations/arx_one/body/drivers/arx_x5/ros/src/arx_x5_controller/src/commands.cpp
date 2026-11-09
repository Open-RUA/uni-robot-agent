#include "arx_x5_controller/X5Controller.hpp"
#include "arx_x5_controller/robot_status.hpp"

namespace arx::x5 {
void X5Controller::CmdCallback(const arx5_arm_msg::msg::RobotCmd::SharedPtr msg) {
  if (!teach_gate_.accepts(rclcpp::Time(msg->header.stamp).nanoseconds())) return;
  double end_pos[6] = {
      msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};

  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

  interfaces_ptr_->setEndPose(transform);

  std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++) {
    joint_positions[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(joint_positions);

  interfaces_ptr_->setArmStatus(msg->mode);

  interfaces_ptr_->setCatch(msg->gripper);
}

void X5Controller::arxJoyCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  // Untimestamped joystick input cannot bypass the operator teaching gate.
  if (!teach_gate_.accepts(0)) return;
  if (msg->data.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Ignoring arx_joy message with fewer than two entries");
    return;
  }
  if (msg->data[0] == 1)
    interfaces_ptr_->setArmStatus(InterfacesThread::state::G_COMPENSATION);
  if (msg->data[1] == 1)
    interfaces_ptr_->setArmStatus(InterfacesThread::state::GO_HOME);
}

void X5Controller::PubState() {
  std::vector<double> joint_pos_vector;
  if (!ReadJointPositions(joint_pos_vector)) return;
  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();
  std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);
  PublishRobotStatus(joint_pos_vector, xyzrpy);
}

void X5Controller::PublishRobotStatus(const std::vector<double>& positions,
                                     const std::vector<double>& xyzrpy) {
  auto message = arx5_arm_msg::msg::RobotStatus();
  message.header.stamp = this->get_clock()->now();
  std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();
  if (!SerializeRobotStatus(xyzrpy, positions, joint_velocities_vector, joint_current_vector, message)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Invalid RobotStatus feedback sizes: pose=%zu positions=%zu velocities=%zu currents=%zu",
                         xyzrpy.size(), positions.size(), joint_velocities_vector.size(), joint_current_vector.size());
    return;
  }
  joint_state_publisher_->publish(message);
}

}
