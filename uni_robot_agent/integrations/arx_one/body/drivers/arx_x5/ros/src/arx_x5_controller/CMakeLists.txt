cmake_minimum_required(VERSION 3.8)
project(arx_x5_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(arm_control REQUIRED)
find_package(arx5_arm_msg REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Prepared from native/dependencies.lock.json; no bundled header snapshot.
set(RUA_DEPENDENCIES_ROOT "" CACHE PATH "Verified native dependency staging directory")
if(NOT EXISTS "${RUA_DEPENDENCIES_ROOT}/dependencies.resolved.json")
    message(FATAL_ERROR "Prepare locked dependencies with native/dependencies.py first")
endif()
set(RUA_VENDOR_ROOT "${RUA_DEPENDENCIES_ROOT}/arx")
include_directories(include)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    message(STATUS "Target architecture is ARM")
    set(DEPEND_LIB ${RUA_VENDOR_ROOT}/arx_x5_src/libarx_x5_src-arm64.so)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
    set(DEPEND_LIB ${RUA_VENDOR_ROOT}/arx_x5_src/libarx_x5_src.so)
else ()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif ()

add_library(arx_x5_vendor SHARED IMPORTED)
set_target_properties(arx_x5_vendor PROPERTIES
    IMPORTED_LOCATION "${DEPEND_LIB}"
    IMPORTED_NO_SONAME TRUE)

add_executable(X5Controller
    src/main.cpp
    src/controller.cpp
    src/commands.cpp
    src/teleoperation.cpp
    src/arbitration.cpp
    src/driver_service.cpp)
target_include_directories(X5Controller PRIVATE
    "${RUA_DEPENDENCIES_ROOT}/eigen"
    "${RUA_VENDOR_ROOT}/arx_hardware_interface/include"
    "${RUA_VENDOR_ROOT}/arx_x5_src/include")
target_link_libraries(X5Controller arx_x5_vendor)
file(SHA256 "${DEPEND_LIB}" RUA_VENDOR_SDK_SHA256)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/x5_2025.urdf" RUA_KINEMATIC_MODEL_SHA256)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/x5.urdf" RUA_TYPE0_MODEL_SHA256)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/x5_master.urdf" RUA_MASTER_MODEL_SHA256)
target_compile_definitions(X5Controller PRIVATE
    RUA_VENDOR_SDK_SHA256="${RUA_VENDOR_SDK_SHA256}"
    RUA_KINEMATIC_MODEL_SHA256="${RUA_KINEMATIC_MODEL_SHA256}"
    RUA_TYPE0_MODEL_SHA256="${RUA_TYPE0_MODEL_SHA256}"
    RUA_MASTER_MODEL_SHA256="${RUA_MASTER_MODEL_SHA256}")
ament_target_dependencies(X5Controller rclcpp ament_index_cpp std_msgs kdl_parser arx5_arm_msg arm_control)
set_target_properties(X5Controller PROPERTIES INSTALL_RPATH "$ORIGIN/..")

add_library(rua_arx_kinematics SHARED src/kinematics.cpp)
ament_target_dependencies(rua_arx_kinematics kdl_parser)
target_include_directories(rua_arx_kinematics PRIVATE "${RUA_DEPENDENCIES_ROOT}/eigen")
install(TARGETS rua_arx_kinematics LIBRARY DESTINATION lib)

install(TARGETS
        X5Controller
        DESTINATION lib/${PROJECT_NAME})

install(FILES ${DEPEND_LIB}
        DESTINATION lib/
)

install(FILES x5.urdf
        DESTINATION share/${PROJECT_NAME}
)

install(FILES x5_master.urdf
        DESTINATION share/${PROJECT_NAME}
)

install(FILES x5_2025.urdf
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    enable_testing()
    add_executable(teach_gate_test test/teach_gate_test.cpp)
    add_test(NAME teach_gate_test COMMAND teach_gate_test)
    add_executable(joint_feedback_test test/joint_feedback_test.cpp)
    add_test(NAME joint_feedback_test COMMAND joint_feedback_test)
    add_executable(vr_input_gate_test test/vr_input_gate_test.cpp)
    add_test(NAME vr_input_gate_test COMMAND vr_input_gate_test)
    add_executable(robot_status_test test/robot_status_test.cpp)
    add_test(NAME robot_status_test COMMAND robot_status_test)
endif()

ament_package()
