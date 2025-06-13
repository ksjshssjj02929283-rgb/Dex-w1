cmake_minimum_required(VERSION 3.8)
project(template_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Msg files
set(MSG_FILES
  "msg/Image.msg"
)

# Srv files
set(SRV_FILES
  "srv/Echo.srv"
)

# Action files
set(ACTION_FILES
  "action/Simple.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES sensor_msgs std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)

ament_package()
