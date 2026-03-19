# 电源板状态反馈
std_msgs/Header header
string software_version
string hardware_version
string sn_code
uint8 peripheral_connectivity # 联通性
ChannelStatus[] power_status  # 电源通道状态
ChannelStatus[] key_status      # 按键通道状态