#include "arx_x5_controller/X5Controller.hpp"

namespace arx::x5 {
void X5Controller::VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg) {
  double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

  interfaces_ptr_->setEndPose(transform);

  interfaces_ptr_->setArmStatus(InterfacesThread::state::END_CONTROL);

  double gripper = msg->gripper;
  if(new_version_)
    gripper *= (-3.4 / 5);
  interfaces_ptr_->setCatch(gripper);
}

void X5Controller::VrPubState() {
  std::vector<double> joint_pos_vector;
  if (!ReadJointPositions(joint_pos_vector)) return;
  auto message = arm_control::msg::PosCmd();

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  Eigen::Quaterniond quat(transform.rotation());

  std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

  message.x = xyzrpy[0];
  message.y = xyzrpy[1];
  message.z = xyzrpy[2];
  message.roll = xyzrpy[3];
  message.pitch = xyzrpy[4];
  message.yaw = xyzrpy[5];
  message.quater_x = quat.x();
  message.quater_y = quat.y();
  message.quater_z = quat.z();
  message.quater_w = quat.w();

  message.gripper = joint_pos_vector[6];

  vr_joint_state_publisher_->publish(message);

  PublishRobotStatus(joint_pos_vector, xyzrpy);
}

void X5Controller::FollowCmdCallback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg) {
  std::vector<double> target_joint_position(6, 0.0);

  for (int i = 0; i < 6; i++) {
    target_joint_position[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(target_joint_position);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);

  if (catch_control_mode_ == CatchControlMode::kPosition) {
    double pos;
    if(new_version_)
      pos = msg->joint_pos[6];
    else
      pos = msg->joint_pos[6] * 5;
    interfaces_ptr_->setCatch(pos);
  }
  else
    interfaces_ptr_->setCatchTorque(msg->joint_cur[6]);
}

}
