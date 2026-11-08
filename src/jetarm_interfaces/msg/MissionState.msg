# 取物任务状态
# state: idle / armed / to_target / grasping / to_home / done / error / canceled
std_msgs/Header header
string state
string detail
geometry_msgs/PoseStamped target
geometry_msgs/PoseStamped home