# Unified header-carrying int64 array message.
# 统一的带 Header 的 int64 数组消息，供各模块复用。
# header.stamp 标记数据产生时间，header.frame_id 标记数据来源坐标系/设备。

std_msgs/Header header
int64[] data
