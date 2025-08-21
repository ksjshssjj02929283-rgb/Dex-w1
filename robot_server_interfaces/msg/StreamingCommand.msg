# streaming request对应的id
int64 request_id

# 本指令是否是本次streaming request的最后一帧
bool streaming_finished

# 指令目标位置(单位: jointstreaming时为弧度, eeposestreaming时为米)
float32[] position