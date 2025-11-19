# header
std_msgs/Header header

# 关节名称
string[] name

# 关节位置 [0,100]
float64[] position

# 关节速度 [0,100]
float64[] velocity

# 关节电流 [0,100]
float64[] current

# 关节力/扭矩
float64[] force

# 关节状态 JointState
uint8[] state