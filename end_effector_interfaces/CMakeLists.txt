cmake_minimum_required(VERSION 3.8)
project(end_effector_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Genrerate service and action interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EEJointControlMode.msg"
  "msg/EEJointControl.msg"
  "msg/EEStatus.msg"
  "msg/EEJointStatus.msg"
  "msg/EEJointState.msg"
  "msg/EETactileState.msg"
  "msg/EEFeedback.msg"
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

ament_package()

