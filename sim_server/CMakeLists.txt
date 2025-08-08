cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
project(sim_server)

option(ENABLE_SIM_SERVER "Enable sim_server package compilation" ON)
option(ENABLE_TORQUE_CONTROL_SIMULATION "Enable torque control" OFF)

# Python interpreter setup
set(PYTHON_EXECUTABLE "/usr/bin/python3")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi -Wno-reorder -Wno-dev)
endif()
find_package(PythonInterp 3.6 REQUIRED)
set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}" CACHE FILEPATH "Python interpreter" FORCE)

# Find dependencies.
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(ruckig REQUIRED) 

# Ros 2 dependencies.
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pinocchio REQUIRED)

# Other Ros2 packages
find_package(robot_server_interfaces REQUIRED)
find_package(robot_server_lib REQUIRED)

if(ENABLE_SIM_SERVER)

  aux_source_directory(src LIB_SRCS)

  add_executable(${PROJECT_NAME} ${LIB_SRCS})

  if(ENABLE_TORQUE_CONTROL_SIMULATION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TORQUE_CONTROL=1)
  else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE TORQUE_CONTROL=0)
  endif()

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_action
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    ament_index_cpp 
    Eigen3 
    eigen3_cmake_module
    robot_server_interfaces
    robot_server_lib
  )

  target_link_libraries(${PROJECT_NAME}
    pinocchio::pinocchio
    ruckig::ruckig
  )

  install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    FILES robot.cfg
    DESTINATION share/${PROJECT_NAME}
  )
endif()
ament_package()
