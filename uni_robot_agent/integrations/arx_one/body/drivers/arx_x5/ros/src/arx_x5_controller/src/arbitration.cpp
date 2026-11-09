#include "arx_x5_controller/X5Controller.hpp"

namespace arx::x5 {
static inline Eigen::Isometry3d FKJointToIso(
    InterfacesTools& tools, const std::vector<double>& q7) {

  std::vector<double> q6(6, 0.0);
  for (int i = 0; i < 6 && i < (int)q7.size(); ++i) q6[i] = q7[i];

  std::vector<double> xyzrpy = tools.ForwardKinematicsRpy(q6);
  double in[6] = {xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]};
  return arx::solve::Xyzrpy2Isometry(in);
}
static inline Eigen::Isometry3d PosCmdToIso(const arm_control::msg::PosCmd& v) {
  double in[6] = {v.x, v.y, v.z, v.roll, v.pitch, v.yaw};
  return arx::solve::Xyzrpy2Isometry(in);
}
void X5Controller::ControlTick() {
  arm_control::msg::PosCmd vr, vr_prev;
  arm_control::msg::JointControl model;
  bool have_vr = false, have_prev = false, have_model = false;
  VrInputGate::Time vr_received_at;
  std::uint64_t vr_sequence = 0;

  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    have_vr = have_vr_cmd_;
    have_prev = have_prev_vr_;
    have_model = have_model_cmd_;
    if (have_vr) {
      vr = last_vr_cmd_;
      vr_received_at = last_vr_received_at_;
      vr_sequence = vr_cmd_sequence_;
    }
    if (have_prev) vr_prev = prev_vr_cmd_;
    if (have_model) model = last_model_cmd_;
  }

  if (!have_model&&!have_vr&&!was_using_vr_) {
    return;
  }
  bool vr_moving = false;
  if (have_vr && have_prev) {
    double dx = vr.x - vr_prev.x;
    double dy = vr.y - vr_prev.y;
    double dz = vr.z - vr_prev.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    vr_moving = (dist > vr_move_threshold_);
  }
  const auto now = VrInputGate::Clock::now();
  const auto decision = vr_input_gate_.Update(vr_sequence, vr_received_at, vr_moving,
                                              was_using_vr_, now);
  const bool vr_active = decision.active;

  if (decision.use_vr) {
    // Hold ownership between receipts, but never replay a cached VR command.
    if (!decision.new_frame) return;
    // Validate the command-frame feedback before any setter or takeover pulse.
    std::vector<double> q_now;
    if (!ReadJointPositions(q_now)) return;

    Eigen::Isometry3d T_vr = PosCmdToIso(vr);

    if (!was_using_vr_) {

      T_curr_initial_ = FKJointToIso(*tools_ptr_, q_now);
      T_vr_initial_ = T_vr;
      T_last_target = T_curr_initial_;  // Initialize the previous target to the current pose
      vr_align_valid_ = true;
      was_using_vr_ = true;

      RCLCPP_WARN(this->get_logger(),
        "[VR TAKEOVER] align VR deltas to the current pose (avoid jump from model->VR)");

      std_msgs::msg::Empty msg;
      vr_takeover_pub_->publish(msg);

      RCLCPP_WARN(this->get_logger(),
        "[VR TAKEOVER BEGIN] publish takeover pulse to policy client");

    }

    if(!vr_align_valid_) {
      RCLCPP_ERROR(this->get_logger(),
        "[VR CONTROL] align not valid! skip this vr command.");
      return;
    }

    Eigen::Isometry3d T_target = Eigen::Isometry3d::Identity();
    Eigen::Vector3d pos_delta = T_vr.translation() - T_vr_initial_.translation();
    T_target.translation() = T_curr_initial_.translation() + pos_delta;

    Eigen::Quaterniond q_vr_initial(T_vr_initial_.rotation());
    Eigen::Quaterniond q_vr_now(T_vr.rotation());
    Eigen::Quaterniond q_curr_initial(T_curr_initial_.rotation());

    if (q_vr_initial.w() < 0) {
      q_vr_initial.coeffs() = -q_vr_initial.coeffs();
    }
    if (q_curr_initial.w() < 0) {
      q_curr_initial.coeffs() = -q_curr_initial.coeffs();
    }
    if (q_vr_initial.dot(q_vr_now) < 0) {
      q_vr_now.coeffs() = -q_vr_now.coeffs();
    }

    Eigen::Quaterniond q_delta = q_vr_initial.inverse() * q_vr_now;

    Eigen::Quaterniond q_target = q_curr_initial * q_delta;
    q_target.normalize();

    Eigen::Quaterniond q_last(T_last_target.rotation());
    if (q_last.dot(q_target) < 0) {
      q_target.coeffs() = -q_target.coeffs();
    }

    const double smooth_alpha = 0.8;  // 0-1; smaller is smoother
    q_target = q_last.slerp(smooth_alpha, q_target);
    q_target.normalize();

    T_target.linear() = q_target.toRotationMatrix();

    Eigen::Matrix3d R_check = T_target.rotation();
    double sin_pitch = -R_check(2, 0);
    sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));  // clamp
    double pitch_deg = std::asin(sin_pitch) * 180.0 / M_PI;

    const double pitch_limit = 60.0;  // Beyond ±60° is treated as near a singularity
    bool near_singularity = std::abs(pitch_deg) > pitch_limit;

    if (near_singularity && vr_active) {
      T_target.linear() = T_last_target.rotation();
      static auto last_warn_time = std::chrono::steady_clock::now();
      auto now_warn = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now_warn - last_warn_time).count() >= 1.0) {
        last_warn_time = now_warn;
        RCLCPP_WARN(this->get_logger(), "[SINGULARITY] Pitch=%.1f° > %.1f°, orientation locked", pitch_deg, pitch_limit);
      }
    }

    if (vr_active) {
      const double max_rot_per_frame_deg = 8.5;  // Maximum rotation allowed per frame (degrees)
      const double max_rot_per_frame_rad = max_rot_per_frame_deg * M_PI / 180.0;

      Eigen::Quaterniond q_curr(T_target.rotation());
      Eigen::Quaterniond q_prev(T_last_target.rotation());

      if (q_curr.dot(q_prev) < 0) {
        q_curr.coeffs() = -q_curr.coeffs();
      }

      Eigen::Quaterniond q_diff = q_prev.inverse() * q_curr;
      Eigen::AngleAxisd aa_diff(q_diff);
      double frame_rot_angle = aa_diff.angle();  // radians

      if (frame_rot_angle > max_rot_per_frame_rad) {
        double ratio = max_rot_per_frame_rad / frame_rot_angle;
        Eigen::Quaterniond q_limited = q_prev.slerp(ratio, q_curr);
        q_limited.normalize();
        T_target.linear() = q_limited.toRotationMatrix();
      }
    }

    if (!vr_active) {
      T_target = T_last_target;
    }

    interfaces_ptr_->setEndPose(T_target);
    interfaces_ptr_->setArmStatus(InterfacesThread::state::END_CONTROL);

    T_last_target = T_target;
    double gripper_target = vr.gripper;
    gripper_target = -gripper_target + 5.0;
    gripper_target = gripper_target * (3.6 / 5);
    if (new_version_) gripper_target *= (-3.4 / 5);
    double gripper_current = q_now[6];
    double gripper_cmd = gripper_target;
    double max_gripper_delta1 = 0.4;  // Maximum change per cycle; tunable
    double max_gripper_delta2 = 0.005;  // Maximum change per cycle; tunable

    double delta = gripper_target - gripper_current;
    if (delta > 0) {
      if (delta > max_gripper_delta1) {
        delta = max_gripper_delta1;
      }
    } else {
      if (-delta > max_gripper_delta2) {
        delta = -max_gripper_delta2;
      }
    }

    gripper_cmd = gripper_current + delta;
    if (!vr_active) {
      gripper_cmd = last_gripper;
    }
    interfaces_ptr_->setCatch(gripper_cmd);
    last_gripper = gripper_cmd;

    return;
  }

  if (was_using_vr_) {
    was_using_vr_ = false;
    vr_align_valid_ = false;
    vr_input_gate_.Release(now);
    model_allowed_.store(false, std::memory_order_release);

    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      have_prev_vr_ = false;
      have_vr_cmd_ = false;
      have_model_cmd_ = false;
    }
    have_model = false;
    std_msgs::msg::Empty msg;
    vr_release_pub_->publish(msg);

    RCLCPP_WARN(this->get_logger(), "[VR RELEASE] publish reset pulse to policy client");
  }

  if (have_model) {
    if (!model_allowed_.load(std::memory_order_acquire)) {
      return;
    }
    std::vector<double> target_joint_position(6, 0.0);
    for (int i = 0; i < 6; i++)
      target_joint_position[i] = model.joint_pos[i];
    interfaces_ptr_->setJointPositions(target_joint_position);
    interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);
    interfaces_ptr_->setCatch(model.joint_pos[6]);
  }
}


}
