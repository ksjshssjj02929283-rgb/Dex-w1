cmake_minimum_required(VERSION 3.8)
project(robot_client_interfaces)

# 默认编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成 srv 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ClientService.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# ament package 必须调用
ament_package()