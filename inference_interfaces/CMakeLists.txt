cmake_minimum_required(VERSION 3.8)
project(inference_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Msg files
set(MSG_FILES
  "msg/InferenceStatus.msg"
  "msg/TeleopStatus.msg"
  "msg/JointGroup.msg"
  "msg/ShadowSegment.msg"
)

# Srv files
set(SRV_FILES
  "srv/Deploy.srv"
  "srv/StartInference.srv"
  "srv/InferenceCommand.srv"
  "srv/GetModelInfo.srv"
  "srv/ExportShadowSegments.srv"
)


# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
