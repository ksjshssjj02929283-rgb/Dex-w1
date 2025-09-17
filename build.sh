# 切换到包含所有包的根目录
cd "$(dirname "$0")/.."

# rm -rf build install log
# 查找指定文件夹中的包名
PACKAGES=$(find dexe_interfaces dexe_robot_server -name "package.xml" -exec sh -c 'dirname {} | xargs basename' \; | sort -u | tr '\n' ' ')

# 只编译指定包
colcon build --cmake-args -DENABLE_TORQUE_CONTROL_SIMULATION=ON --packages-select $PACKAGES 