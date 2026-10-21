# 资产变更事件（map_asset_manager ~/asset_changes，reliable、不 latched）：
# 任意一张地图（含未加载的）的资产被写入 / 回滚 / 显式保存 / 坐标变换后发一条，只是「变了」的通知，不带资产内容。
# 前端编辑某张地图时：收到 map_name 相同、revision 与本地持有的不同的事件 → 调 get_assets{map_name} 重新读取。
# 自己写入的那次，set_assets 响应已带新 revision，事件里的 revision 与之相同，无需重读。
# 当前已加载地图另有 latched 的 ~/assets 快照话题（带全部内容），导航等运行时消费方仍订阅它。
builtin_interfaces/Time stamp
string map_name      # 变更的地图
bool   is_current    # 是否当前已加载地图
string revision      # 变更后的资产内容指纹（与 get_assets / set_assets 的 revision 同一算法）
string op            # set（set_assets）/ rollback（rollback_assets）/ save（save_assets 存版本）
                     # 对齐 / 重设原点时的资产坐标变换由离线工具 transform_assets_tool 完成，不发本事件
string version_ts    # op = save / rollback 时相关的版本号；其它为空
