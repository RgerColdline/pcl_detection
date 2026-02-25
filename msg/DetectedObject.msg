# 检测到的物体信息
std_msgs/Header header

# 物体类型：0=Wall, 1=Cylinder, 2=Circle
int32 type

# 物体名称
string name

# 物体位置（中心点）
geometry_msgs/Point position

# 物体尺寸
float32 width
float32 height
float32 depth

# 圆柱/圆环特有参数
float32 radius

# 平面方程参数 (Ax + By + Cz + D = 0)
float32[] plane_coeffs

# 点云内点数量
int32 point_count

# 提取耗时 (ms)
float32 extraction_time
