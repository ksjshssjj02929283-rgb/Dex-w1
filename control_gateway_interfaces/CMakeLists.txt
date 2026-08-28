cmake_minimum_required(VERSION 3.8)
project(control_gateway_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(end_effector_interfaces REQUIRED)

set(MSG_FILES
  "msg/JointCommand.msg"
  "msg/ForwardedJointCommand.msg"
  "msg/EndEffectorCommand.msg"
)

set(SRV_FILES
  "srv/StartJointCommandSession.srv"
  "srv/SwitchJointCommandSource.srv"
  "srv/StopJointCommandSession.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs end_effector_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(end_effector_interfaces)

ament_package()
