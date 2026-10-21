# 存量资产失效项（计算出来的只读清单，不写进资产文件）。
# 底图编辑、禁行区变化、地图加载后重算；资产本身保留，只是被标记为不可用。
# 出现在 GetAssets / SetAssets 响应与 AssetSnapshot 话题中。
string kind     # point / edge / route
string key      # 点位名、边 id 或路线名
string code     # 原因码：out_of_map / in_unknown / in_obstacle / in_restricted /
                #         too_close_boundary / too_close_unknown / too_close_obstacle / too_close_restricted /
                #         endpoint_unavailable / crosses_unknown / crosses_obstacle / crosses_restricted /
                #         point_unavailable（路线引用的点位不可用）/ missing_control_point / width_too_small
string detail   # 人可读说明（英文），如 "point inside obstacle after map edit"
