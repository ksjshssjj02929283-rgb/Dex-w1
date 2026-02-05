cmake_minimum_required(VERSION 3.8)
project(chassis_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Msg files
set(MSG_FILES
  "msg/ChassisWholeControl.msg"
  "msg/ChassisMotorControl.msg"
)

set(SRV_FILES
  "srv/SaveMap.srv"
)

set(ACTION_FILES                 # <<< ADD >>>
  "action/AutoCharging.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES 
    std_msgs
    action_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(action_msgs) 

ament_package()

