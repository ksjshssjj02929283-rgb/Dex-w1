# 底图编辑的一笔（MapManagerCommand OP_EDIT_GRID 使用）。
# 坐标是 map 坐标系下的米，不是像素；与地图显示旋转（display_yaw）无关。
# 服务端按笔刷直径把折线 / 矩形栅格化到 map.png，像素规则见 tool。
uint8 TOOL_WALL    = 1   # 绘制墙体：覆盖到的像素一律置为占用（0）
uint8 TOOL_ERASE   = 2   # 擦除噪点：只把占用像素置为自由（255）；未知保持未知，不会把未建图区域变成可通行
uint8 TOOL_UNKNOWN = 3   # 未知区域：用画笔把覆盖到的区域改成未知（非未知像素写回本图阈值下的未知灰度）

uint8 SHAPE_POLYLINE = 1 # 折线：自由绘制与直线都按折线提交，单点 = 一个圆点
uint8 SHAPE_RECT     = 2 # 矩形：points 给两个对角点，按填充矩形处理（三种工具都接受，界面上主要给未知区域用）

uint8 tool
uint8 shape
geometry_msgs/Point[] points   # map 坐标系，单位米；z 忽略
float32 width_m                # 笔刷直径（米），> 0；SHAPE_RECT 忽略
