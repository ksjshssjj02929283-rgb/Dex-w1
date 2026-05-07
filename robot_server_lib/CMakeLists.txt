cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# ccache 支持（默认关闭，如需启用请设置 ENABLE_CCACHE=ON）
option(ENABLE_CCACHE "Enable ccache" OFF)
if(ENABLE_CCACHE)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  endif()
endif()

project(robot_server_lib)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi -Wno-reorder -Wno-dev -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable)
endif()

option(ENABLE_TORQUE_CONTROL_SIMULATION "Enable torque control" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PythonInterp 3.6 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ruckig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(robot_server_interfaces REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

if(NOT TARGET DexeRP)
  # choose per-arch folder
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|armv7|armv7l|armv8|arm)$")
    set(DEXERP_WS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../lib/dexe_rp/arm")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|x64|i[3-6]86)$")
    set(DEXERP_WS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../lib/dexe_rp/x86")
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}. Please provide DexeRP under lib/.")
  endif()

  set(DEXERP_LOCAL_LIB "${DEXERP_WS_ROOT}/libDexeRP.so")

  if(EXISTS "${DEXERP_LOCAL_LIB}")
    add_library(DexeRP SHARED IMPORTED GLOBAL)
    set_target_properties(DexeRP PROPERTIES IMPORTED_LOCATION "${DEXERP_LOCAL_LIB}")

    # so that #include "dexe_rp/xxx.h" works
    target_include_directories(DexeRP INTERFACE "${DEXERP_WS_ROOT}/..")

    # install headers + .so into colcon install/ (deployment just ships install/)
    install(FILES "${DEXERP_LOCAL_LIB}" DESTINATION lib)
    install(DIRECTORY "${DEXERP_WS_ROOT}/../dexe_rp" DESTINATION include)
  else()
    message(FATAL_ERROR "DexeRP not found: ${DEXERP_LOCAL_LIB} (and no system target DexeRP)")
  endif()
endif()

aux_source_directory(src LIB_SRCS)
aux_source_directory(kinematics LIB_SRCS)
aux_source_directory(dynamics LIB_SRCS)
aux_source_directory(motion_planner LIB_SRCS)
aux_source_directory(torque_controller LIB_SRCS)

add_subdirectory(utils)

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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../lib/magic_enum>
)

target_link_libraries(${PROJECT_NAME}
  robot_server_utils
  DexeRP
  fmt::fmt
  spdlog::spdlog
  yaml-cpp
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
    robot_server_utils
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
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../lib/magic_enum/
  DESTINATION include
)

install(
  DIRECTORY robot_models
  DESTINATION share/${PROJECT_NAME}
)

# 添加头文件导出
ament_export_targets(export_robot_server_lib HAS_LIBRARY_TARGET)
ament_export_libraries(robot_server_lib robot_server_utils)
ament_export_include_directories(include)

option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  enable_testing()
  file(GLOB TEST_SRCS test/*.cpp)
  foreach(test_src ${TEST_SRCS})
    get_filename_component(test_name ${test_src} NAME_WE)
    ament_add_gtest(${test_name} ${test_src})
    target_link_libraries(${test_name} robot_server_lib robot_server_utils)
  endforeach()
endif()

ament_package()

