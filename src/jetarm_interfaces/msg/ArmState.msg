# JetArm 机械臂状态
#
# tick 顺序固定为舵机 ID 1,2,3,4,5,10；
# NaN 表示本次读取无效（读超时），deg 对应位置也写 0。
std_msgs/Header header

float32[6] tick
float32[6] deg

# 底座坐标系 TCP，单位 mm；读不全时全部为 0
geometry_msgs/Point tcp_mm

# 6 个舵机是否全部处于锁力状态
bool loaded

# 是否有运动指令正在执行
bool moving

# 最近一次错误（空字符串表示正常）
string error