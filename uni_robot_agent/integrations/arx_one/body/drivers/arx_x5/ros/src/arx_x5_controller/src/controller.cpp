#include "arx_x5_controller/X5Controller.hpp"
#include "arx_x5_controller/joint_feedback.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>

namespace arx::x5 {
X5Controller::X5Controller() : Node("x5_controller_node") {
  // Report the ABI/model used to build this controller. Read-only values ignore
  // launch overrides so a new Body cannot silently drive an older controller.
  rcl_interfaces::msg::ParameterDescriptor identity;
  identity.read_only = true;
  this->declare_parameter<std::string>("vendor_sdk_sha256", RUA_VENDOR_SDK_SHA256, identity, true);
  auto ack_topic = this->declare_parameter("vr_ack_topic", "/vr_release_ack");
  vr_takeover_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      this->declare_parameter("vr_takeover_topic", "/vr_takeover_begin"),
      10);
  vr_reset_ack_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      ack_topic, 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        model_allowed_.store(true, std::memory_order_release);
        RCLCPP_WARN(this->get_logger(), "[VR ACK] policy client reset done, model allowed");
      });
  vr_release_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      this->declare_parameter("vr_release_topic", "/vr_release_reset"),
      10);
  RCLCPP_INFO(this->get_logger(), "Arm initialization started...");
  rclcpp::on_shutdown(std::bind(&X5Controller::cleanup, this));
  std::string arm_control_type = this->declare_parameter("arm_control_type", "normal");
  std::string package_name = "arx_x5_controller";
  std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
  int end_type = this->declare_parameter("arm_end_type", 0);
  arm_end_type_ = end_type;
  this->declare_parameter<std::string>("kinematic_model_sha256",
      end_type == 0 ? RUA_TYPE0_MODEL_SHA256 :
      end_type == 1 ? RUA_MASTER_MODEL_SHA256 : RUA_KINEMATIC_MODEL_SHA256,
      identity, true);
  catch_control_mode_ = static_cast<CatchControlMode>(this->declare_parameter("catch_control_mode", 0));
  go_home_positions_ = this->declare_parameter<std::vector<double>>("go_home_position",std::vector<double>{0,0,0,0,0,0});
  std::string urdf_path;
  if (end_type == 0)
    urdf_path = package_share_dir + "/" + "x5.urdf";
  else if (end_type == 1)
    urdf_path = package_share_dir + "/" + "x5_master.urdf";
  else {
    urdf_path = package_share_dir + "/x5_2025.urdf";
    new_version_ = true;
  }
  interfaces_ptr_ =
      std::make_shared<InterfacesThread>(urdf_path, this->declare_parameter("arm_can_id", "can0"), end_type);

  tools_ptr_ = std::make_shared<InterfacesTools>(0);
  interfaces_ptr_->arx_x(150, 600, 10);
  interfaces_ptr_->setHomePositions(go_home_positions_);
  auto pub_name = this->declare_parameter("arm_pub_topic_name", "arm_status");

  if (arm_control_type == "normal") {
    if (!HoldPostInitPosition(*interfaces_ptr_, arm_end_type_,
                             InterfacesThread::state::POSITION_CONTROL)) {
      throw std::runtime_error("Post-init hold requires valid feedback and no vendor errors");
    }
    this->declare_parameter("startup_policy", "post_vendor_init_snapshot_hold_v1");
    RCLCPP_INFO(this->get_logger(),
                "Vendor reference initialization complete; holding measured pose. "
                "Explicit joint_zero opens the gripper.");
    RCLCPP_INFO(this->get_logger(), "Normal mode started");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name, 1);
    joint_state_subscriber_ = this->create_subscription<arx5_arm_msg::msg::RobotCmd>(
        this->declare_parameter("arm_sub_topic_name", "arm_cmd"),
        10,
        std::bind(&X5Controller::CmdCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&X5Controller::PubState, this));
  } else if (arm_control_type == "vr_slave") {
    RCLCPP_INFO(this->get_logger(), "VR teleoperation mode started");
    vr_joint_state_publisher_ = this->create_publisher<arm_control::msg::PosCmd>(pub_name, 1);

    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name + "_full", 1);
    vr_joint_state_subscriber_ = this->create_subscription<arm_control::msg::PosCmd>(
        this->declare_parameter("arm_sub_topic_name", "ARX_VR_L"),
        10,
        std::bind(&X5Controller::VrCmdCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::VrPubState, this));
  } else if (arm_control_type == "remote_master") {
    RCLCPP_INFO(this->get_logger(), "Remote leader mode started");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name, 1);
    interfaces_ptr_->setArmStatus(InterfacesThread::state::G_COMPENSATION);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::PubState, this));
  } else if (arm_control_type == "remote_slave") {
    RCLCPP_INFO(this->get_logger(), "Remote follower mode started");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name, 1);
    follow_joint_state_subscriber_ = this->create_subscription<arx5_arm_msg::msg::RobotStatus>(
        this->declare_parameter("arm_sub_topic_name", "followed_arm_topic"),
        1,
        std::bind(&X5Controller::FollowCmdCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::PubState, this));
  }
  else if (arm_control_type == "joint_control_v1")
  {
    RCLCPP_INFO(this->get_logger(),"Joint control mode started (with VR motion arbitration)");

    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name, 1);

    joint_control_subscriber_ = this->create_subscription<arm_control::msg::JointControl>(
        this->declare_parameter("arm_sub_topic_name", "joint_control"),
        10,
        [this](const arm_control::msg::JointControl::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          last_model_cmd_ = *msg;
          have_model_cmd_ = true;
        });

    vr_sub_topic_name_ = this->declare_parameter("vr_sub_topic_name", "ARX_VR_L");
    vr_move_threshold_ = this->declare_parameter("vr_move_threshold", 0.0001);
    vr_input_gate_.hold_sec = this->declare_parameter("vr_hold_sec", 0.3);
    vr_input_gate_.takeover_sec = this->declare_parameter("vr_takeover_sec", 0.1);
    vr_input_gate_.input_timeout_sec = this->declare_parameter("vr_input_timeout_sec", 0.3);
    if (!std::isfinite(vr_input_gate_.input_timeout_sec) || vr_input_gate_.input_timeout_sec <= 0.0) {
      throw std::invalid_argument("vr_input_timeout_sec must be finite and positive");
    }

    vr_cmd_subscriber_v1_ = this->create_subscription<arm_control::msg::PosCmd>(
        vr_sub_topic_name_,
        10,
        [this](const arm_control::msg::PosCmd::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(cmd_mtx_);
          if(!(*msg).roll && !(*msg).pitch && !(*msg).yaw) {
            return;
          }
          const auto received_at = VrInputGate::Clock::now();
          have_prev_vr_ = have_vr_cmd_ && vr_input_gate_.Fresh(last_vr_received_at_, received_at);
          if (have_prev_vr_) {
            prev_vr_cmd_ = last_vr_cmd_;
          }
          last_vr_cmd_ = *msg;
          have_vr_cmd_ = true;
          last_vr_received_at_ = received_at;
          ++vr_cmd_sequence_;
        });

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&X5Controller::ControlTick, this));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::PubState, this));
  }
  arx_joy_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("/arx_joy",1,std::bind(&X5Controller::arxJoyCB,this,std::placeholders::_1));
  driver_service_ = this->create_service<arx5_arm_msg::srv::DriverCommand>(
      this->declare_parameter("driver_service", "~/driver_command"),
      std::bind(&X5Controller::DriverCommandCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

bool X5Controller::ReadJointPositions(std::vector<double>& positions) {
  try {
    if (ReadCommandJointPositions(*interfaces_ptr_, arm_end_type_, positions)) {
      return true;
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Joint position feedback unavailable: %s", error.what());
    return false;
  }
  RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Invalid joint position feedback; skipping this callback");
  return false;
}

}
