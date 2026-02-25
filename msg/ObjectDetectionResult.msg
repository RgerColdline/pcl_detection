# 检测结果的总体消息
std_msgs/Header header

# 所有检测到的物体
DetectedObject[] objects

# 各类物体数量
int32 wall_count
int32 cylinder_count
int32 circle_count

# 处理时间统计
float32 total_time
float32 downsample_time
float32 normals_time
float32 wall_time
float32 cylinder_time
float32 circle_time

# 状态
bool success
string status_message
