# header
std_msgs/Header header

# 关节控制模式 JointControlMode
uint8 mode

# 关节名称
string[] name

# 关节位置 [0,100]
# 关节速度 [-100,100]
# 关节电流 [-100,100]
# 关节力/扭矩
float64[] value