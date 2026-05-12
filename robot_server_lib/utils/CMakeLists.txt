cmake_minimum_required(VERSION 3.15)

project(robot_server_utils)

aux_source_directory(. UTILS_SRCS)
add_library(${PROJECT_NAME} STATIC ${UTILS_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../lib/magic_enum>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dynamics>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  nlohmann_json
  robot_server_interfaces
  pinocchio
)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wpedantic -Wno-psabi -Wno-reorder -Wno-dev
    -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable
  )
endif()

ament_export_targets(export_robot_server_utils HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_robot_server_utils
  DESTINATION lib
)

install(
  DIRECTORY . DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
