cmake_minimum_required(VERSION 3.8)
project(nav_interfaces)

# 编译配置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) 基础依赖（ROS 2）
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2) 依赖的消息/动作包
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

# 3) 声明要生成的消息
set(MSG_FILES
  "msg/Path.msg"
  "msg/PathPose.msg"
  "msg/AreaInfo.msg"
  "msg/DTLane.msg"
  "msg/Lane.msg"
  "msg/LaneArray.msg"
  "msg/Waypoint.msg"
  "msg/WaypointState.msg"
  "msg/NavPoint.msg"
  "msg/DockingInfo.msg"
  "msg/Task.msg"
  "msg/TaskGroup.msg"
  "msg/BatteryInfo.msg"
  "msg/Line.msg"
  "msg/LineArray.msg"
  "msg/LineMapData.msg"
  "msg/LineMapDataInfo.msg"
  "msg/TargetInfo.msg"
  "msg/TargetMapInfo.msg"
  "msg/LocalizationState.msg"
  "msg/RobotStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/ChangeTopology.msg"
  "msg/TopologyLine.msg"
)

# 4) 声明要生成的服务
set(SRV_FILES
  "srv/AddTask.srv"
  "srv/AddTaskGroup.srv"
  "srv/AddNavPoint.srv"
  "srv/SampleNavPoint.srv"
  "srv/RemoveNavPoint.srv"
  "srv/GetNavPoint.srv"
  "srv/GetAllNavPoints.srv"
  "srv/NavToPoint.srv"
  "srv/GetCurrentTaskInfo.srv"
  "srv/SetPose.srv"
  "srv/GetStatus.srv"
  "srv/SetCommandBoard.srv"
  "srv/SetMapName.srv"
  "srv/GetMapList.srv"
  "srv/LoadMap.srv"
  "srv/DeleteMap.srv"
  "srv/GetMapInfo.srv"
  "srv/GetString.srv"
  "srv/AddAreaInfo.srv"
  "srv/GetAreaInfoList.srv"
)

# 5) 生成接口（取代 ROS 1 的 add_message_files / add_service_files / generate_messages）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    action_msgs
    actionlib_msgs
)

# 6) 导出运行期依赖
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  nav_msgs
  action_msgs
  actionlib_msgs
)

ament_package()
