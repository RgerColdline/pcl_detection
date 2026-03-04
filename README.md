# PCL 障碍物检测节点

基于 Livox 雷达的障碍物检测 ROS 节点，采用**新 Pipeline 架构**，支持墙体、方环、一般障碍物检测。

## 功能特性

### 检测流程

```
Livox 点云 → 下采样 → 法向量估计 → 地面分割 → 墙体检测 → 剩余点聚类 → 方环/障碍物识别
```

### 支持物体类型

| 类型 | 检测算法 | 输出信息 |
|------|----------|----------|
| **墙体** | RANSAC 平面拟合 + 法向量验证 | 平面方程、法向量、尺寸 |
| **方环** | 聚类 + PCA 投影 + 角点拟合 + 中空验证 | 长宽、角度、中心位置 |
| **障碍物** | 欧式聚类 + OBB 包围盒 + 圆柱拟合 | 半径、高度、中心位置 |

### 技术特点

- ✅ **地面分割**：RANSAC 平面拟合，自动过滤地面点
- ✅ **墙体检测**：仅确认满足条件的墙体，避免误用点云
- ✅ **方环检测**：支持"日"字形方环（上口 + 下腿），自动识别中空结构
- ✅ **智能聚类**：动态调整聚类距离，适应不同场景
- ✅ **日志节流**：可配置日志输出频率，避免刷屏

## 依赖

- **ROS**: Noetic
- **PCL**: 1.10+
- **雷达驱动**: livox_ros_driver2
- **数学库**: Eigen3

## 快速开始

### 1. 编译

```bash
cd ~/catkin_ws
catkin build pcl_detection
source devel/setup.bash
```

### 2. 启动雷达驱动

```bash
roslaunch livox_ros_driver2 livox_lidar_msg.launch
```

### 3. 启动障碍物检测

```bash
roslaunch pcl_detection obstacle_detection.launch
```

### 4. 查看结果

```bash
# 查看检测统计
rostopic echo /pcl_detection/obstacles

# 查看物体列表
rostopic echo /pcl_detection/objects
```

## 使用示例

### C++ 订阅器完整示例

```cpp
// src/obstacle_subscriber.cpp
#include <ros/ros.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>
#include <string>

class ObstacleSubscriber
{
public:
    ObstacleSubscriber(ros::NodeHandle& nh)
    {
        sub_ = nh.subscribe("/pcl_detection/obstacles", 10, 
                            &ObstacleSubscriber::callback, this);
        ROS_INFO("障碍物订阅节点已启动");
    }

private:
    void callback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        // 1. 检查检测状态
        if (!msg->success)
        {
            ROS_WARN("检测失败：%s", msg->status_message.c_str());
            return;
        }

        // 2. 获取统计信息
        ROS_INFO("========== 检测结果 ==========");
        ROS_INFO("墙体：%d, 方环：%d, 障碍物：%d", 
                 msg->wall_count, msg->rectangle_count, msg->obstacle_count);
        ROS_INFO("总耗时：%.2f ms", msg->total_time);

        // 3. 遍历所有物体
        for (const auto& obj : msg->objects)
        {
            processObject(obj);
        }
    }

    void processObject(const pcl_detection::DetectedObject& obj)
    {
        // --- 基础信息接口 ---
        std::string name = obj.name;                    // 物体名称
        int type = obj.type;                            // 物体类型 (0=墙，3=方环，4=障碍物)
        int point_count = obj.point_count;              // 点云数量
        double extraction_time = obj.extraction_time;   // 提取耗时 (ms)
        
        // 位置接口
        double x = obj.position.x;
        double y = obj.position.y;
        double z = obj.position.z;
        
        // 尺寸接口
        double width = obj.width;     // 长度/宽度
        double height = obj.height;   // 高度/宽度
        double depth = obj.depth;     // 深度
        
        ROS_INFO("[%s] %s", getTypeName(type).c_str(), name.c_str());
        ROS_INFO("  位置：(%.2f, %.2f, %.2f)", x, y, z);
        ROS_INFO("  尺寸：%.2f x %.2f x %.2f", width, height, depth);
        ROS_INFO("  点数：%d, 耗时：%.2f ms", point_count, extraction_time);

        // --- 类型特定接口 ---
        switch (type)
        {
            case 0:  // 墙体
                processWall(obj);
                break;
            case 3:  // 方环
                processRectangle(obj);
                break;
            case 4:  // 障碍物
                processObstacle(obj);
                break;
        }
    }

    void processWall(const pcl_detection::DetectedObject& obj)
    {
        // 墙体特有接口
        if (obj.plane_coeffs.size() >= 4)
        {
            double A = obj.plane_coeffs[0];
            double B = obj.plane_coeffs[1];
            double C = obj.plane_coeffs[2];
            double D = obj.plane_coeffs[3];
            
            ROS_INFO("  [墙体] 平面方程：%.3fx + %.3fy + %.3fz + %.3f = 0", A, B, C, D);
            ROS_INFO("  [墙体] 法向量：(%.3f, %.3f, %.3f)", A, B, C);
        }
    }

    void processRectangle(const pcl_detection::DetectedObject& obj)
    {
        // 方环特有接口
        double length = obj.width;      // 长度 (m)
        double rect_width = obj.height; // 宽度 (m)
        double angle = obj.radius;      // 角度 (度)
        
        ROS_INFO("  [方环] 长度：%.2f m, 宽度：%.2f m, 角度：%.1f°", 
                 length, rect_width, angle);
    }

    void processObstacle(const pcl_detection::DetectedObject& obj)
    {
        // 障碍物特有接口（OBB 立方体）
        double length = obj.width;    // 长度 (m，含膨胀)
        double width = obj.height;    // 宽度 (m，含膨胀)
        double height = obj.depth;    // 高度 (m，含膨胀)

        ROS_INFO("  [障碍物] 尺寸：%.2f x %.2f x %.2f m (长 x 宽 x 高)", 
                 length, width, height);
        
        // 可选：读取主轴方向（从 obb_coeffs）
        if (obj.obb_coeffs.size() >= 6) {
            double ax = obj.obb_coeffs[3];
            double ay = obj.obb_coeffs[4];
            double az = obj.obb_coeffs[5];
            ROS_INFO("  [障碍物] 主轴方向：(%.2f, %.2f, %.2f)", ax, ay, az);
        }
    }

    std::string getTypeName(int type)
    {
        switch (type)
        {
            case 0: return "墙体";
            case 1: return "圆柱";
            case 2: return "圆环";
            case 3: return "方环";
            case 4: return "障碍物";
            default: return "未知";
        }
    }

private:
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_subscriber");
    ros::NodeHandle nh;
    
    ObstacleSubscriber subscriber(nh);
    ros::spin();
    
    return 0;
}
```

### CMakeLists.txt 配置

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_obstacle_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_detection
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 编译订阅器节点
add_executable(obstacle_subscriber src/obstacle_subscriber.cpp)
target_link_libraries(obstacle_subscriber ${catkin_LIBRARIES})
add_dependencies(obstacle_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS})
```

### 可用接口汇总

#### ObjectDetectionResult 消息接口

```cpp
// 检测状态
msg->success           // bool - 检测是否成功
msg->status_message    // string - 状态信息

// 物体统计
msg->wall_count        // int32 - 墙体数量
msg->cylinder_count    // int32 - 圆柱数量
msg->circle_count      // int32 - 圆环数量
msg->rectangle_count   // int32 - 方环数量
msg->obstacle_count    // int32 - 障碍物数量

// 时间统计
msg->total_time        // float32 - 总耗时 (ms)
msg->downsample_time   // float32 - 下采样耗时 (ms)
msg->normals_time      // float32 - 法向量估计耗时 (ms)
msg->wall_time         // float32 - 墙体检测耗时 (ms)
msg->cylinder_time     // float32 - 圆柱检测耗时 (ms)
msg->circle_time       // float32 - 圆环检测耗时 (ms)
msg->rectangle_time    // float32 - 方环检测耗时 (ms)

// 物体列表
msg->objects           // DetectedObject[] - 所有物体
```

#### DetectedObject 消息接口

```cpp
// 基础信息
obj->header            // Header - 消息头
obj->name              // string - 物体名称 (如 "wall_1", "rectangle_1")
obj->type              // int32 - 物体类型 (0=墙，1=圆柱，2=圆环，3=方环，4=障碍物)
obj->point_count       // int32 - 点云数量
obj->extraction_time   // float32 - 提取耗时 (ms)

// 位置信息
obj->position.x        // float64 - X 坐标 (m)
obj->position.y        // float64 - Y 坐标 (m)
obj->position.z        // float64 - Z 坐标 (m)

// 尺寸信息
obj->width             // float64 - 长度/宽度 (m)
obj->height            // float64 - 高度 (m)
obj->depth             // float64 - 深度 (m)

// 类型特定信息
obj->radius            // float64 - 半径 (圆柱/圆环) 或 角度 (方环)
obj->plane_coeffs      // float64[] - 平面系数 [A,B,C,D] (仅墙体)
```

## 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 10Hz | 输入雷达点云 |
| `/pcl_detection/obstacles` | `ObjectDetectionResult` | 10Hz | 检测结果汇总 |
| `/pcl_detection/objects` | `DetectedObject[]` | 10Hz | 物体列表 |
| `/pcl_detection/pointcloud` | `PointCloud2` | 10Hz | 下采样后点云（可选） |
| `/pcl_detection/cloud/wall` | `PointCloud2` | 10Hz | 墙体点云（红色） |
| `/pcl_detection/cloud/rectangle` | `PointCloud2` | 10Hz | 方环点云（黄色） |
| `/pcl_detection/cloud/obstacle` | `PointCloud2` | 10Hz | 障碍物点云（绿色） |

## 配置参数

编辑 `config/sample_config.yaml`：

### 基础配置

```yaml
# 输入输出话题
input_topic: "/livox/lidar"
result_topic: "/pcl_detection/result"
objects_topic: "/pcl_detection/objects"

# 跳帧设置（0=每帧处理，1=隔 1 帧处理）
skip_frames: 0

# 日志控制
log_skip_frames: 19        # 每 20 帧输出一次日志
log_interval_sec: 2.0      # 每 2 秒输出一次
ros_log_level: "INFO"      # DEBUG/INFO/WARN/ERROR
```

### 新 Pipeline 配置

```yaml
use_obstacle_pipeline: true  # 启用新 Pipeline

obstacle_pipeline:
  # 下采样（影响精度和速度）
  downsample_config:
    approx: true           # true=快速下采样，false=标准下采样
    leaf_size: 0.08        # 网格大小（米），建议 0.05-0.12

  # 地面分割
  ground_config:
    enable: false          # 新 Pipeline 中地面点直接过滤
    distance_threshold: 0.05
    max_iterations: 100

  # 墙体检测
  wall_config:
    enable: true
    distance_threshold: 0.04
    min_inliers: 500       # 最小点数（下采样后）
    angle_threshold: 15.0  # 法向量与垂直方向夹角（度）

  # 方环检测
  rectangle_config:
    enable: true
    min_cluster_size: 100   # 最小聚类点数
    length_min: 0.5         # 最小长度（米）
    length_max: 2.5         # 最大长度（米）
    width_min: 0.4          # 最小宽度（米）
    width_max: 1.5          # 最大宽度（米）
    hollow_ratio_threshold: 0.4  # 中空比例（0-1，越大要求越中空）

  # 聚类配置
  cluster_config:
    cluster_tolerance: 0.5  # 聚类距离阈值（米）
    min_cluster_size: 30    # 最小簇点数
    max_cluster_size: 2000  # 最大簇点数

  # OBB 包围盒
  obb_config:
    inflation_radius: 0.1   # 膨胀半径（米）
    min_obstacle_height: 0.2 # 最小障碍物高度（米）
```

## 输出示例

### 终端日志

```
[INFO] [ObstaclePipeline] 地面分割完成：地面 3245 点，非地面 2156 点
[INFO] [ObstaclePipeline] 墙体检测完成：检测到 2 个墙体
[INFO] [RectFromCluster] 开始处理 8 个聚类
[INFO] [RectFromCluster] 方环 #1 验证通过：1.12x0.78, angle=0.0° (中空)
[INFO] [Objects] 检测物体统计：墙体=2, 方环=1, 障碍物=5, 总计=8
```

### 话题消息

```bash
$ rostopic echo /pcl_detection/obstacles
wall_count: 2
cylinder_count: 0
circle_count: 0
rectangle_count: 1
obstacle_count: 5
total_time: 45.23
objects:
  - name: "wall_1"
    type: 0
    position: {x: 5.23, y: 1.45, z: 0.12}
    width: 2.50
    height: 1.80
    depth: 0.05
  - name: "rectangle_1"
    type: 3
    position: {x: 2.10, y: 0.47, z: 0.15}
    width: 1.12
    height: 0.78
  - name: "obstacle_1"
    type: 4
    position: {x: 6.48, y: 1.29, z: 0.01}
    radius: 0.91
    height: 3.17
```

## 调试技巧

### 1. 调整检测灵敏度

**方环检测不稳定**：
```yaml
rectangle_config:
  min_cluster_size: 50     # 降低到 50，检测更小的方环
  hollow_ratio_threshold: 0.3  # 降低到 0.3，放宽中空要求
```

**墙体检测过多**：
```yaml
wall_config:
  min_inliers: 800         # 增大到 800，提高点数要求
  distance_threshold: 0.02  # 降低到 0.02，更严格
```

**障碍物聚类过碎**：
```yaml
cluster_config:
  cluster_tolerance: 0.8   # 增大到 0.8，合并更多点
  min_cluster_size: 100    # 增大到 100，过滤小物体
```

### 2. 查看详细日志

```yaml
ros_log_level: "DEBUG"  # 输出所有调试信息
```

日志将显示：
- 每个聚类的点数、中心、尺寸
- 边界点数量、角点数量
- 中空检测详情

### 3. 性能优化

**提高帧率**：
```yaml
downsample_config:
  leaf_size: 0.12        # 增大下采样网格
obstacle_pipeline:
  wall_config:
    min_inliers: 300     # 降低点数要求
```

**提高精度**：
```yaml
downsample_config:
  leaf_size: 0.05        # 减小下采样网格
obstacle_pipeline:
  wall_config:
    min_inliers: 800     # 提高点数要求
```

## 常见问题

### Q1: 方环检测不稳定

**原因**：点云密度不足或聚类距离太小

**解决**：
```yaml
cluster_config:
  cluster_tolerance: 0.5  # 增大聚类距离
  min_cluster_size: 30    # 降低最小点数
```

### Q2: 墙体用掉太多点

**原因**：墙体检测条件过松

**解决**：
```yaml
wall_config:
  min_inliers: 800        # 提高点数要求
  # 只有满足条件的墙体才标记点为已使用
```

### Q3: 日志输出太频繁

**解决**：
```yaml
log_skip_frames: 39       # 每 40 帧输出一次
ros_log_level: "WARN"     # 只输出警告及以上
```

### Q4: 检测延迟高

**原因**：点云数量过多或法向量计算耗时

**解决**：
```yaml
downsample_config:
  leaf_size: 0.12         # 增大下采样
obstacle_pipeline:
  normal_config:
    num_threads: 4        # 增加线程数
```

## 性能参考

| 配置 | 点云数 | 处理时间 | FPS |
|------|--------|----------|-----|
| 快速 | 3000 点 | 20ms | 50Hz |
| 平衡 | 6000 点 | 45ms | 22Hz |
| 高精度 | 12000 点 | 90ms | 11Hz |

*测试环境：Intel i7-10700, 8 核*

## 版本历史

- **v2.0** (当前版本)
  - 新 Pipeline：地面分割 + 墙体 + 方环 + 聚类
  - 方环中空检测，避免实心矩形误检
  - 日志节流，避免刷屏

- **v1.0** (旧版本)
  - 原 Pipeline：墙体 + 圆柱 + 圆环 + 矩形框
  - 使用 RANSAC 直接拟合

## License

MIT License
