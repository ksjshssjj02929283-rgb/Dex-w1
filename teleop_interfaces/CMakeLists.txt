cmake_minimum_required(VERSION 3.8)
project(teleop_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Srv files
set(SRV_FILES
  "srv/ImageForwardingGet.srv"
  "srv/ImageForwardingSet.srv"
  "srv/TeleopConfigGet.srv"
  "srv/TeleopConfigSet.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
