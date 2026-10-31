# command_board 传感器监测：每路被监测话题的实时状态（随 NavSystemStatus 一起发布）。
string  name            # 配置中的名称，标准传感器包括：
                        # - imu         : /imu/chassis
                        # - odom        : /feedback/odom
                        # - lidar_front : /laser_scan/front
                        # - lidar_rear  : /laser_scan/rear
                        # - rgbd_front  : /orbbec/front/laser_scan
                        # - rgbd_rear   : /orbbec/rear/laser_scan
string  topic           # 被监测话题
uint8   state           # STATE_*，见下方常量
bool    required        # 是否导航必需（FAULT 时是否计入导航故障）
float32 rate_hz         # 统计窗口内的实际频率；样本不足为 0
float32 age_sec         # 距上一帧的时间；从未收到为 -1
float32 max_gap_sec     # 统计窗口内的最大到达间隔；样本不足为 0

uint8 STATE_UNKNOWN = 0     # 启动宽限期内尚未收到首帧
uint8 STATE_OK = 1          # 距上一帧 <= 超时
uint8 STATE_FAULT = 2       # 停发超时 / 宽限期后从未收到
uint8 STATE_SUPPRESSED = 3  # 急停中且该路配置急停抑制
