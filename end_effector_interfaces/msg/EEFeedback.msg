# 末端执行器状态反馈

std_msgs/Header header

# 末端执行器型号（品牌_型号_版本）
string ee_name

# 整个末端执行器的状态
uint8 status

# 关节状态 
EEJointState[] joint_states

# 触觉传感器状态
EETactileState[] tactile_states