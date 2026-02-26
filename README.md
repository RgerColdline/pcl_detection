# PCL 对象检测包

基于 PCL 和 ROS 的实时物体检测系统，支持墙面、圆柱、圆环检测，适用于无人机避障和导航。

## 快速开始

### 启动节点

一键启动脚本位于本包的 `shell/obs.sh`。

请根据自身 ROS 环境修改脚本中的工作空间路径后运行。

或手动启动：
```bash
cd 你的工作空间
source devel/setup.bash  # 或 setup.zsh

# 启动障碍物检测节点
roslaunch pcl_detection object_detector.launch

# （可选）启动障碍物监听测试节点
roslaunch pcl_detection obstacle_listener.launch
```

## 功能特性

- ✅ **实时检测**：墙面、圆柱、圆环
- ✅ **高性能**：~70ms 处理时间（Jetson Xavier NX）
- ✅ **ROS 集成**：标准话题接口
- ✅ **可配置**：支持 YAML 配置文件

## 话题说明

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox 激光雷达点云 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/pcl_detection/result` | `pcl_detection/ObjectDetectionResult` | 检测结果汇总 |
| `/pcl_detection/objects` | `pcl_detection/DetectedObject` | 单个检测结果 |
| `/pcl_detection/visualization/marker` | `visualization_msgs/MarkerArray` | 可视化标记 |

## C++ 使用示例

### 基础订阅

```cpp
#include <ros/ros.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>

class ObstacleAvoidanceNode
{
public:
    ObstacleAvoidanceNode(ros::NodeHandle& nh)
    {
        // 订阅检测结果汇总
        result_sub_ = nh.subscribe("/pcl_detection/result", 10, 
                                   &ObstacleAvoidanceNode::resultCallback, this);
        
        // 或订阅单个物体（高频）
        object_sub_ = nh.subscribe("/pcl_detection/objects", 100,
                                   &ObstacleAvoidanceNode::objectCallback, this);
    }

private:
    // 方式 1：接收检测结果汇总（推荐）
    void resultCallback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        ROS_INFO("=== 检测结果 ===");
        ROS_INFO("墙体：%d 个，圆柱：%d 个，圆环：%d 个",
                 msg->wall_count, msg->cylinder_count, msg->circle_count);
        ROS_INFO("总耗时：%.2f ms", msg->total_time);
        
        // 遍历所有物体
        for (const auto& obj : msg->objects)
        {
            processObject(obj);
        }
    }
    
    // 方式 2：接收单个物体
    void objectCallback(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        processObject(obj);
    }
    
    // 处理单个物体
    void processObject(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        ROS_INFO("物体：%s", obj->name.c_str());
        ROS_INFO("  位置：(%.3f, %.3f, %.3f)", 
                 obj->position.x, obj->position.y, obj->position.z);
        ROS_INFO("  尺寸：%.3f x %.3f x %.3f m", 
                 obj->width, obj->height, obj->depth);
        
        // 根据物体类型处理
        switch (obj->type)
        {
            case 0: processWall(obj); break;    // 墙体
            case 1: processCylinder(obj); break; // 圆柱
            case 2: processCircle(obj); break;   // 圆环
            default: break;
        }
    }

private:
    ros::Subscriber result_sub_;
    ros::Subscriber object_sub_;
};
```

### 获取墙体信息

```cpp
void processWall(const pcl_detection::DetectedObject::ConstPtr& obj)
{
    ROS_INFO("[墙体] %s", obj->name.c_str());
    
    // 1. 获取位置
    double x = obj->position.x;
    double y = obj->position.y;
    double z = obj->position.z;
    ROS_INFO("  中心位置：(%.3f, %.3f, %.3f)", x, y, z);
    
    // 2. 获取尺寸
    double width = obj->width;    // 宽度（米）
    double height = obj->height;  // 高度（米）
    ROS_INFO("  尺寸：宽=%.2f m, 高=%.2f m", width, height);
    
    // 3. 获取平面方程参数
    // plane_coeffs: [A, B, C, D] 对应 Ax + By + Cz + D = 0
    if (!obj->plane_coeffs.empty())
    {
        double A = obj->plane_coeffs[0];
        double B = obj->plane_coeffs[1];
        double C = obj->plane_coeffs[2];
        double D = obj->plane_coeffs[3];
        
        ROS_INFO("  平面方程：%.3fx + %.3fy + %.3fz + %.3f = 0", A, B, C, D);
        
        // 4. 计算法向量
        double normal_length = std::sqrt(A*A + B*B + C*C);
        double nx = A / normal_length;
        double ny = B / normal_length;
        double nz = C / normal_length;
        ROS_INFO("  法向量：(%.3f, %.3f, %.3f)", nx, ny, nz);
        
        // 5. 计算原点到墙面的距离
        double distance = std::abs(D) / normal_length;
        ROS_INFO("  距原点距离：%.3f m", distance);
    }
    
    // 6. 获取点云信息
    ROS_INFO("  内点数：%d", obj->point_count);
    ROS_INFO("  提取耗时：%.3f ms", obj->extraction_time);
    
    // TODO: 在这里添加避障逻辑
    // 例如：判断墙体是否在无人机前方，计算避障方向
}
```

### 获取圆柱信息

```cpp
void processCylinder(const pcl_detection::DetectedObject::ConstPtr& obj)
{
    ROS_INFO("[圆柱] %s", obj->name.c_str());
    
    // 1. 获取位置（圆柱中心）
    double x = obj->position.x;
    double y = obj->position.y;
    double z = obj->position.z;
    ROS_INFO("  中心位置：(%.3f, %.3f, %.3f)", x, y, z);
    
    // 2. 获取半径
    double radius = obj->radius;
    ROS_INFO("  半径：%.3f m", radius);
    
    // 3. 获取高度
    double height = obj->height;
    ROS_INFO("  高度：%.3f m", height);
    
    // 4. 计算与无人机的距离
    double distance = std::sqrt(x*x + y*y + z*z);
    ROS_INFO("  距原点距离：%.3f m", distance);
    
    // 5. 判断是否在碰撞路径上
    double horizontal_dist = std::sqrt(x*x + y*y);
    if (horizontal_dist < radius + 0.5)  // 0.5m 安全距离
    {
        ROS_WARN("  警告：圆柱在碰撞路径上！");
    }
    
    // TODO: 在这里添加避障逻辑
    // 例如：计算绕行方向
}
```

### 获取圆环信息

```cpp
void processCircle(const pcl_detection::DetectedObject::ConstPtr& obj)
{
    ROS_INFO("[圆环] %s", obj->name.c_str());
    
    // 1. 获取位置（圆心）
    double x = obj->position.x;
    double y = obj->position.y;
    double z = obj->position.z;
    ROS_INFO("  圆心位置：(%.3f, %.3f, %.3f)", x, y, z);
    
    // 2. 获取半径
    double radius = obj->radius;
    ROS_INFO("  半径：%.3f m", radius);
    
    // 3. 获取圆环所在平面（如果有平面方程）
    if (!obj->plane_coeffs.empty())
    {
        double A = obj->plane_coeffs[0];
        double B = obj->plane_coeffs[1];
        double C = obj->plane_coeffs[2];
        
        // 法向量
        double normal_length = std::sqrt(A*A + B*B + C*C);
        double nx = A / normal_length;
        double ny = B / normal_length;
        double nz = C / normal_length;
        ROS_INFO("  法向量：(%.3f, %.3f, %.3f)", nx, ny, nz);
    }
    
    // 4. 计算与无人机的距离
    double distance = std::sqrt(x*x + y*y + z*z);
    ROS_INFO("  距原点距离：%.3f m", distance);
    
    // TODO: 在这里添加避障逻辑
}
```

### 完整示例代码

```cpp
#include <ros/ros.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>
#include <cmath>

class ObstacleAvoidanceNode
{
public:
    ObstacleAvoidanceNode(ros::NodeHandle& nh)
    {
        result_sub_ = nh.subscribe("/pcl_detection/result", 10, 
                                   &ObstacleAvoidanceNode::resultCallback, this);
        ROS_INFO("避障节点已启动，等待检测结果...");
    }
    
    void run()
    {
        ros::spin();
    }

private:
    void resultCallback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        // 检查检测是否成功
        if (!msg->success)
        {
            ROS_WARN("检测失败：%s", msg->status_message.c_str());
            return;
        }
        
        // 遍历所有物体
        for (const auto& obj : msg->objects)
        {
            switch (obj->type)
            {
                case 0: processWall(obj); break;
                case 1: processCylinder(obj); break;
                case 2: processCircle(obj); break;
            }
        }
    }
    
    void processWall(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        // 获取平面方程
        if (obj->plane_coeffs.size() < 4) return;
        
        double A = obj->plane_coeffs[0];
        double B = obj->plane_coeffs[1];
        double C = obj->plane_coeffs[2];
        double D = obj->plane_coeffs[3];
        
        // 计算法向量
        double norm = std::sqrt(A*A + B*B + C*C);
        double nx = A / norm;
        double ny = B / norm;
        double nz = C / norm;
        
        // 判断墙体方向（简单示例）
        if (std::abs(nz) < 0.3)  // 竖直墙面
        {
            ROS_INFO("检测到竖直墙面：%s", obj->name.c_str());
            // TODO: 添加避障逻辑
        }
    }
    
    void processCylinder(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        double dist = std::sqrt(obj->position.x * obj->position.x + 
                               obj->position.y * obj->position.y);
        
        if (dist < obj->radius + 0.5)
        {
            ROS_WARN("圆柱障碍物过近：%s", obj->name.c_str());
            // TODO: 添加避障逻辑
        }
    }
    
    void processCircle(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        // 处理圆环障碍物
        ROS_INFO("检测到圆环：%s", obj->name.c_str());
        // TODO: 添加避障逻辑
    }

private:
    ros::Subscriber result_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");
    ros::NodeHandle nh;
    
    ObstacleAvoidanceNode node(nh);
    node.run();
    
    return 0;
}
```

## 配置参数

### 日志控制

编辑 `launch/object_detector.launch`：

```xml
<!-- 每 20 帧输出一次摘要 -->
<param name="log_skip_frames" value="19" />

<!-- 每 2 秒输出一次 -->
<param name="log_interval_sec" value="2.0" />

<!-- Livox 日志级别：0=无，1=摘要，2=详细 -->
<param name="livox_debug_level" value="0" />
```

### 检测参数

编辑 `config/sample_config.yaml`：

```yaml
# 墙面检测
wall_extraction:
  min_inliers: 500        # 最小内点数
  distance_threshold: 0.08 # 距离阈值（米）

# 圆柱检测
cylinder_extraction:
  min_inliers: 100
  radius_min: 0.1         # 最小半径（米）
  radius_max: 0.4         # 最大半径（米）

# 圆环检测
circle_extraction:
  min_inliers: 80
  radius_min: 0.2
  radius_max: 0.7
  min_coverage_angle: 240.0  # 最小覆盖角度（度）
```

## 性能

**Jetson Xavier NX 测试结果**：

| 模块 | 耗时 |
|------|------|
| 下采样 | ~1ms |
| 法向量估计 | ~25ms |
| 墙面提取 | ~25ms |
| 圆柱提取 | ~5ms |
| 圆环提取 | ~5ms |
| **总计** | **~70ms** |

## 目录结构

```
pcl_detection/
├── shell/              # 启动脚本
│   └── obs.sh         # 一键启动脚本
├── launch/             # Launch 文件
│   ├── object_detector.launch
│   └── points_sample.launch
├── config/             # 配置文件
│   └── sample_config.yaml
├── msg/                # 消息定义
│   ├── DetectedObject.msg
│   └── ObjectDetectionResult.msg
├── src/
│   ├── nodes/         # ROS 节点
│   │   ├── object_detector_node.cpp
│   │   ├── detection_visualizer.cpp
│   │   └── points_samples.cpp
│   └── core/          # 核心算法
├── include/           # 头文件
└── README.md          # 本文档
```

## 依赖

- ROS Noetic
- PCL 1.8+
- livox_ros_driver2
- Eigen3
- yaml-cpp

## 许可证

TODO

## 联系方式

TODO
