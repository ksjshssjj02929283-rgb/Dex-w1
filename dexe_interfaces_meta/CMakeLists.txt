cmake_minimum_required(VERSION 3.8)
project(dexe_interfaces NONE)

# 纯 metapackage:本身不产出 lib / 头文件,
# 只用 <exec_depend> 在 package.xml 里聚合 12 个接口子包,
# 让消费者一行 <exec_depend>dexe_interfaces</exec_depend> 即可拉齐。
find_package(ament_cmake REQUIRED)
ament_package()
