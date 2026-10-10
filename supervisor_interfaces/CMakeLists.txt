cmake_minimum_required(VERSION 3.8)
project(supervisor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ErrorInfo.msg"
  "msg/SystemCommand.msg"
  "msg/SystemMode.msg"
  "msg/SystemStatus.msg"
  "srv/DiagnoseError.srv"
  "srv/RecoverError.srv"
  "action/ChangeSystemMode.action"
  DEPENDENCIES builtin_interfaces
)

# Shared status-code catalog: each robot module appends its own entry under `modules:`.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
