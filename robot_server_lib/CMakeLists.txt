cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

project(robot_server_lib)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi -Wno-reorder -Wno-dev -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable)
endif()

option(ENABLE_TORQUE_CONTROL_SIMULATION "Enable torque control" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PythonInterp 3.6 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ruckig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(robot_server_interfaces REQUIRED)

aux_source_directory(src LIB_SRCS)
aux_source_directory(kinematics LIB_SRCS)
aux_source_directory(dynamics LIB_SRCS)
aux_source_directory(motion_planner LIB_SRCS)
aux_source_directory(torque_controller LIB_SRCS)
aux_source_directory(utils LIB_SRCS)
add_library(${PROJECT_NAME} ${LIB_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/kinematics>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/dynamics>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/motion_planner>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/torque_controller>
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  pinocchio::pinocchio
  ruckig::ruckig
)

if(ENABLE_TORQUE_CONTROL_SIMULATION)
  target_compile_definitions(${PROJECT_NAME} PRIVATE TORQUE_CONTROL=1)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE TORQUE_CONTROL=0)
endif()

ament_target_dependencies(robot_server_lib
  Eigen3 

  #ros2 dependencies
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  robot_server_interfaces
)


install(
  TARGETS
    robot_server_lib
  EXPORT export_robot_server_lib
  DESTINATION lib
)

install(
  DIRECTORY
    include/
    utils/
    kinematics/
    dynamics/
    motion_planner/
    torque_controller/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY robot_models
  DESTINATION share/${PROJECT_NAME}
)

# 添加头文件导出
ament_export_targets(export_robot_server_lib HAS_LIBRARY_TARGET)
ament_export_libraries(robot_server_lib)
ament_export_include_directories(include)

option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  enable_testing()
  file(GLOB TEST_SRCS test/*.cpp)
  foreach(test_src ${TEST_SRCS})
    get_filename_component(test_name ${test_src} NAME_WE)
    ament_add_gtest(${test_name} ${test_src})
    target_link_libraries(${test_name} robot_server_lib)
  endforeach()
endif()

ament_package()

