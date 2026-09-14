cmake_minimum_required(VERSION 3.8)
project(dexe_application_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTask.action"
  "srv/RobotAppStatus.srv"
  "srv/RobotAppDiagnostic.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
