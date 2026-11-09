#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <std_msgs/msg/empty.hpp>

#include <mutex>
#include <string>
#include <cmath>


#include "arx_x5_src/interfaces/InterfacesThread.hpp"
#include "arx_x5_src/interfaces/InterfaceTools.hpp"


#include "arx5_arm_msg/msg/robot_cmd.hpp"
#include "arx5_arm_msg/msg/robot_status.hpp"
#include "arx5_arm_msg/srv/driver_command.hpp"
#include "arm_control/msg/pos_cmd.hpp"
#include "arm_control/msg/joint_control.hpp"
#include "arx_x5_controller/vr_input_gate.hpp"
#include "arx_x5_controller/teach_gate.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

namespace arx::x5 {
class X5Controller : public rclcpp::Node {
 public:
  X5Controller();
  void cleanup() {
    interfaces_ptr_.reset();
  }

  void CmdCallback(const arx5_arm_msg::msg::RobotCmd::SharedPtr msg);

  void PubState();

  void VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg);

  void VrPubState();

  void FollowCmdCallback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg);
  void arxJoyCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
 private:
  TeachGate teach_gate_;
  bool ReadJointPositions(std::vector<double>& positions);
  void PublishRobotStatus(const std::vector<double>& positions,
                          const std::vector<double>& xyzrpy);
  int arm_end_type_ = 0;
  void DriverCommandCallback(
      const std::shared_ptr<arx5_arm_msg::srv::DriverCommand::Request> request,
      std::shared_ptr<arx5_arm_msg::srv::DriverCommand::Response> response);
  rclcpp::Service<arx5_arm_msg::srv::DriverCommand>::SharedPtr driver_service_;
  std::shared_ptr<InterfacesThread> interfaces_ptr_;

  enum class CatchControlMode {
    kPosition,
    kTorque
  } catch_control_mode_;

  // Normal and remote follower modes
  rclcpp::Publisher<arx5_arm_msg::msg::RobotStatus>::SharedPtr joint_state_publisher_;
  // VR mode
  rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_publisher_;

  // Normal mode
  rclcpp::Subscription<arx5_arm_msg::msg::RobotCmd>::SharedPtr joint_state_subscriber_;
  // VR mode
  rclcpp::Subscription<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_subscriber_;
  // Remote follower mode
  rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr follow_joint_state_subscriber_;
  // joint control
  rclcpp::Subscription<arm_control::msg::JointControl>::SharedPtr joint_control_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr arx_joy_sub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vr_takeover_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vr_release_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool new_version_ = false;
  std::vector<double> go_home_positions_ = {0, 0, 0, 0, 0, 0};
    // ===== Arbitration =====
  std::mutex cmd_mtx_;

  // Cache: model joint command
  arm_control::msg::JointControl last_model_cmd_;
  bool have_model_cmd_ = false;

  // Cache: VR pose command (current and previous frame)
  arm_control::msg::PosCmd last_vr_cmd_;
  arm_control::msg::PosCmd prev_vr_cmd_;
  bool have_vr_cmd_ = false;
  bool have_prev_vr_ = false;
  VrInputGate::Time last_vr_received_at_{};
  std::uint64_t vr_cmd_sequence_ = 0;
  VrInputGate vr_input_gate_;

  // Parameters: VR topic and thresholds
  std::string vr_sub_topic_name_ = "ARX_VR_L";
  double vr_move_threshold_ = 0.0001;


  // Subscription: VR (enabled only in joint_control_v1)
  rclcpp::Subscription<arm_control::msg::PosCmd>::SharedPtr vr_cmd_subscriber_v1_;

  // Timer: dispatch the arbitrated command
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<InterfacesTools> tools_ptr_;
  bool was_using_vr_ = false;   // Detects whether VR has just taken over

  bool vr_align_valid_ = false;

  Eigen::Isometry3d T_vr_initial_ = Eigen::Isometry3d::Identity();   // Pose when VR took over
  Eigen::Isometry3d T_curr_initial_ = Eigen::Isometry3d::Identity(); // Arm pose when VR took over
  Eigen::Isometry3d T_last_target = Eigen::Isometry3d::Identity();

  double last_gripper = -2.448;

  // ack: sent back after the Python reset completes
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr vr_reset_ack_sub_;

  // gate: after release, wait for the ack before model control resumes
  std::atomic_bool model_allowed_{true};


  // Arbitrated command dispatch
  void ControlTick();

};
}
