cmake_minimum_required(VERSION 3.8)
project(arx5_arm_msg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCmd.msg"
  "msg/RobotStatus.msg"
  "srv/DriverCommand.srv"
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
