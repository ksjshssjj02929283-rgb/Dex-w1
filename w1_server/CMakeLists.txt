cmake_minimum_required(VERSION 3.15)

project(w1_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(W1_SERVER_VERSION "0.3.0")

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
find_package(ruckig REQUIRED)

# ROS 2 dependencies
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pinocchio REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Other ROS 2 packages
find_package(chassis_interfaces REQUIRED)
find_package(joint_interfaces REQUIRED)
find_package(robot_server_interfaces REQUIRED)
find_package(robot_server_lib REQUIRED)

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
aux_source_directory(utils LIB_SRCS)

add_executable(${PROJECT_NAME} ${LIB_SRCS})

target_compile_definitions(${PROJECT_NAME} PRIVATE "W1_SERVER_VERSION=\"${W1_SERVER_VERSION}\"")

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
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  Eigen3
  eigen3_cmake_module
  chassis_interfaces
  joint_interfaces
  robot_server_interfaces
  robot_server_lib
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  ruckig::ruckig
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# robot.cfg：如果目标已存在则跳过安装
set(ROBOT_CFG_INSTALL_PATH
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/robot.cfg"
)

if(EXISTS "${ROBOT_CFG_INSTALL_PATH}")
  message(STATUS
    "[${PROJECT_NAME}] robot.cfg already exists at install destination, skipping install."
  )
else()
  message(STATUS
    "[${PROJECT_NAME}] robot.cfg not found at install destination, will be installed."
  )
  install(
    FILES robot.cfg
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# chassis.cfg：临时增加底盘类型配置
set(ROBOT_CFG_INSTALL_PATH
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/chassis.cfg"
)

if(EXISTS "${ROBOT_CFG_INSTALL_PATH}")
  message(STATUS
    "[${PROJECT_NAME}] chassis.cfg already exists at install destination, skipping install."
  )
else()
  message(STATUS
    "[${PROJECT_NAME}] chassis.cfg not found at install destination, will be installed."
  )
  install(
    FILES chassis.cfg
    DESTINATION share/${PROJECT_NAME}
  )
endif()


ament_package()
