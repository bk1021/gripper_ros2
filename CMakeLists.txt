cmake_minimum_required(VERSION 3.8)
project(gripper_ros2)

# C++17 required for std::optional, std::clamp, etc.
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Generate custom interfaces ─────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperStatus.msg"
  "msg/GripperMotorState.msg"
  "msg/GripperConfig.msg"
  "srv/SetParam.srv"
  "srv/GetParam.srv"
  "srv/SetState.srv"
  "srv/SetFloat32.srv"
  "action/Grasp.action"
  "action/Open.action"
  "action/CalibrateForce.action"
  DEPENDENCIES std_msgs
)

# ── Node executable ────────────────────────────────────────────────────────
add_executable(gripper_ros_bridge
  src/gripper_ros_bridge.cpp
  src/serial_transport.cpp
)

target_include_directories(gripper_ros_bridge PRIVATE include)

ament_target_dependencies(gripper_ros_bridge
  rclcpp rclcpp_action std_msgs std_srvs diagnostic_msgs
)

# Link generated interfaces to the node
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_ros_bridge "${cpp_typesupport_target}")

install(TARGETS gripper_ros_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
