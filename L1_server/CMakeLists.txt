cmake_minimum_required(VERSION 3.15)

project(L1_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(W1_SERVER_VERSION "0.3.1")

# 如果系统安装了 ccache 则启用，否则保持不启用
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

option(ENABLE_TORQUE_CONTROL_SIMULATION "Enable torque control" OFF)

# Python interpreter setup
set(PYTHON_EXECUTABLE "/usr/bin/python3")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-psabi
    -Wno-reorder
    -Wno-dev
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-but-set-variable
  )
endif()

find_package(PythonInterp 3.6 REQUIRED)
set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}" CACHE FILEPATH "Python interpreter" FORCE)

add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Other ROS 2 packages
find_package(joint_interfaces REQUIRED)
find_package(robot_server_interfaces REQUIRED)

aux_source_directory(src LIB_SRCS)
aux_source_directory(utils LIB_SRCS)

add_executable(${PROJECT_NAME} ${LIB_SRCS})

if(ENABLE_TORQUE_CONTROL_SIMULATION)
  target_compile_definitions(${PROJECT_NAME} PRIVATE TORQUE_CONTROL=1)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE TORQUE_CONTROL=0)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  Eigen3
  eigen3_cmake_module
  joint_interfaces
  robot_server_interfaces
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# feedback 可执行文件
add_executable(feedback tools/feedback.cpp utils/NanoLog.cpp)
target_include_directories(feedback PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
)
target_link_libraries(feedback PUBLIC
    Threads::Threads
    rt
    Eigen3::Eigen
)
install(TARGETS feedback DESTINATION lib/${PROJECT_NAME})

# 测试
find_package(ament_cmake_gtest REQUIRED)
enable_testing()
file(GLOB TEST_SRCS test/*.cpp)
foreach(test_src ${TEST_SRCS})
  get_filename_component(test_name ${test_src} NAME_WE)
  ament_add_gtest(${test_name} ${test_src})
  target_include_directories(${test_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${EIGEN3_INCLUDE_DIR}
  )
  target_sources(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/utils/parallel_ankle_calc.cpp)
endforeach()

ament_package()
