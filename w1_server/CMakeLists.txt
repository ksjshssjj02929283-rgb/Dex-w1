cmake_minimum_required(VERSION 3.15)

project(w1_server)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(W1_SERVER_VERSION "0.5.1")

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
    -g
    -fno-omit-frame-pointer
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
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Other ROS 2 packages
find_package(chassis_interfaces REQUIRED)
find_package(joint_interfaces REQUIRED)
find_package(robot_server_interfaces REQUIRED)
# 🔧 STEERING_ACTION_DIRECT: SteeringAngleControl Action Server now lives in ChassisNode.
find_package(nav_interfaces REQUIRED)
find_package(robot_server_lib REQUIRED)

# tcmalloc
find_library(TCMALLOC_LIBRARY
  NAMES tcmalloc
  PATHS
    /usr/lib/aarch64-linux-gnu
    /lib/aarch64-linux-gnu
    /usr/local/lib
)
if(NOT TCMALLOC_LIBRARY)
  message(FATAL_ERROR "tcmalloc library not found")
endif()
message(STATUS "Found tcmalloc: ${TCMALLOC_LIBRARY}")

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
  diagnostic_updater
  diagnostic_msgs
  ament_index_cpp
  Eigen3
  eigen3_cmake_module
  chassis_interfaces
  joint_interfaces
  robot_server_interfaces
  nav_interfaces
  robot_server_lib
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  ruckig::ruckig
  ${TCMALLOC_LIBRARY}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    BOOST_MPL_LIMIT_LIST_SIZE=30
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES robot_config.yaml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# feedback 可执行文件
add_executable(feedback tools/feedback.cpp)
target_include_directories(feedback PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../robot_server_lib/utils>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../robot_server_lib>
)
ament_target_dependencies(feedback PUBLIC robot_server_lib)
target_link_libraries(feedback PUBLIC
    Threads::Threads
    rt
    Eigen3::Eigen
)
install(TARGETS feedback DESTINATION lib/${PROJECT_NAME})

# ==================== 单元测试 ====================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # chassis_ekf 是纯计算模块，只依赖 Eigen，不牵扯 ROS 运行时与共享内存，
  # 因此可以脱离节点单独测。测试直接编译 src/chassis_ekf.cpp，不链接主目标。
  ament_add_gtest(test_chassis_ekf
    test/test_chassis_ekf.cpp
    src/chassis_ekf.cpp
  )
  if(TARGET test_chassis_ekf)
    target_include_directories(test_chassis_ekf PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_chassis_ekf Eigen3::Eigen)
  endif()

  # 底盘运动学与命令生成：纯计算 header-only，含与切分前实现的等价性比对
  ament_add_gtest(test_chassis_command
    test/test_chassis_command.cpp
  )
  if(TARGET test_chassis_command)
    target_include_directories(test_chassis_command PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/utils
      ${robot_server_lib_INCLUDE_DIRS}   # 只为取零依赖的 chassis_type.h，不链接
    )
  endif()

  # 全向 6 维 EKF：与差速共用 Params，测试同时编译两个滤波器以做退化性对比
  ament_add_gtest(test_omni_chassis_ekf
    test/test_omni_chassis_ekf.cpp
    src/omni_chassis_ekf.cpp
    src/chassis_ekf.cpp
  )
  if(TARGET test_omni_chassis_ekf)
    target_include_directories(test_omni_chassis_ekf PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_omni_chassis_ekf Eigen3::Eigen)
  endif()

  # imu_health 是无依赖的 header-only 状态机，同样脱离节点单测
  ament_add_gtest(test_imu_health
    test/test_imu_health.cpp
  )
  if(TARGET test_imu_health)
    target_include_directories(test_imu_health PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
  endif()
endif()

ament_package()
