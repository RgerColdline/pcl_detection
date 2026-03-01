# PCL 对象检测包

基于 PCL 和 ROS 的实时物体检测系统，支持墙面、圆柱、圆环检测，适用于无人机避障和导航。

## 目录

1. [快速开始](#快速开始)
2. [功能特性](#功能特性)
3. [重要参数：全局处理开关](#重要参数全局处理开关) ⭐
4. [话题说明](#话题说明)
5. [C++ 使用示例](#c-使用示例)
6. [配置参数详解](#配置参数详解)
7. [性能](#性能)
8. [目录结构](#目录结构)
9. [依赖](#依赖)

---

## 快速开始

### 启动节点

```bash
cd 你的工作空间
source devel/setup.bash  # 或 setup.zsh

# 启动障碍物检测节点
roslaunch pcl_detection object_detector.launch
```

### 验证运行状态

```bash
# 查看节点是否运行
rosnode list | grep pcl_detection

# 查看话题
rostopic list | grep pcl_detection

# 查看检测结果
rostopic echo /pcl_detection/result
```

---

## 功能特性

- ✅ **实时检测**：墙面、圆柱、圆环
- ✅ **高性能**：~70ms 处理时间（Jetson Xavier NX）
- ✅ **ROS 集成**：标准话题接口
- ✅ **可配置**：支持 YAML 配置文件
- ✅ **动态控制**：运行时可启停处理

---

## 重要参数：全局处理开关 ⭐

> **`enable_pcl_processing`** （布尔值，默认 `true`）
>
> 控制节点是否处理点云。设置为 `false` 时，节点跳过处理并返回空结果。
> **这是一个全局参数，可被其他节点动态修改。**

### 使用方式

| 方式 | 说明 |
|------|------|
| **YAML 配置** | 设置初始值（`config/sample_config.yaml`） |
| **launch 文件** | 启动时设置 |
| **命令行** | 运行时动态修改 |
| **C++ 代码** | 其他节点编程控制 |

### 1. YAML 配置（初始值）

编辑 `config/sample_config.yaml`：
```yaml
enable_pcl_processing: true  # 初始值
```

### 2. launch 文件设置

```xml
<param name="enable_pcl_processing" value="true" />
```

### 3. 命令行动态控制

```bash
# 关闭 PCL 处理（节点将跳过处理）
rosparam set enable_pcl_processing false

# 开启 PCL 处理（节点恢复正常工作）
rosparam set enable_pcl_processing true

# 查看当前状态
rosparam get enable_pcl_processing
```

### 4. C++ 代码控制（其他节点）

```cpp
#include <ros/ros.h>

class MyControlNode
{
public:
    MyControlNode(ros::NodeHandle& nh) : nh_(nh) {}

    // 关闭 PCL 处理
    void disablePCLProcessing()
    {
        nh_.setParam("enable_pcl_processing", false);
        ROS_INFO("已关闭 PCL 处理");
    }

    // 开启 PCL 处理
    void enablePCLProcessing()
    {
        nh_.setParam("enable_pcl_processing", true);
        ROS_INFO("已开启 PCL 处理");
    }

    // 检查当前状态
    bool isProcessingEnabled()
    {
        bool enabled = true;
        nh_.getParam("enable_pcl_processing", enabled);
        return enabled;
    }

private:
    ros::NodeHandle nh_;  // 使用全局句柄访问全局参数
};
```

### 典型应用场景

- **紧急停止**：检测到异常时快速关闭处理
- **节能模式**：无人机悬停时暂停处理
- **调试模式**：配合其他传感器选择性启用
- **多机协同**：由主控节点统一调度

---

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

---

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
        // 订阅检测结果汇总（推荐）
        result_sub_ = nh.subscribe("/pcl_detection/result", 10,
                                   &ObstacleAvoidanceNode::resultCallback, this);

        // 或订阅单个物体（高频）
        object_sub_ = nh.subscribe("/pcl_detection/objects", 100,
                                   &ObstacleAvoidanceNode::objectCallback, this);
    }

private:
    void resultCallback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        ROS_INFO("=== 检测结果 ===");
        ROS_INFO("墙体：%d 个，圆柱：%d 个，圆环：%d 个",
                 msg->wall_count, msg->cylinder_count, msg->circle_count);
        ROS_INFO("总耗时：%.2f ms", msg->total_time);

        for (const auto& obj : msg->objects)
        {
            processObject(obj);
        }
    }

    void objectCallback(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        processObject(obj);
    }

    void processObject(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        ROS_INFO("物体：%s", obj->name.c_str());
        ROS_INFO("  位置：(%.3f, %.3f, %.3f)",
                 obj->position.x, obj->position.y, obj->position.z);

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

    // 1. 位置
    ROS_INFO("  中心：(%.3f, %.3f, %.3f)",
             obj->position.x, obj->position.y, obj->position.z);

    // 2. 尺寸
    ROS_INFO("  尺寸：%.2f x %.2f x %.2f m",
             obj->width, obj->height, obj->depth);

    // 3. 平面方程：Ax + By + Cz + D = 0
    if (!obj->plane_coeffs.empty())
    {
        double A = obj->plane_coeffs[0];
        double B = obj->plane_coeffs[1];
        double C = obj->plane_coeffs[2];
        double D = obj->plane_coeffs[3];

        ROS_INFO("  平面方程：%.3fx + %.3fy + %.3fz + %.3f = 0", A, B, C, D);

        // 法向量
        double norm = std::sqrt(A*A + B*B + C*C);
        ROS_INFO("  法向量：(%.3f, %.3f, %.3f)", A/norm, B/norm, C/norm);

        // 距原点距离
        ROS_INFO("  距原点：%.3f m", std::abs(D) / norm);
    }

    ROS_INFO("  内点数：%d", obj->point_count);
}
```

### 获取圆柱/圆环信息

```cpp
void processCylinder(const pcl_detection::DetectedObject::ConstPtr& obj)
{
    ROS_INFO("[圆柱] %s", obj->name.c_str());
    ROS_INFO("  中心：(%.3f, %.3f, %.3f)",
             obj->position.x, obj->position.y, obj->position.z);
    ROS_INFO("  半径：%.3f m", obj->radius);
    ROS_INFO("  高度：%.2f m", obj->height);
}

void processCircle(const pcl_detection::DetectedObject::ConstPtr& obj)
{
    ROS_INFO("[圆环] %s", obj->name.c_str());
    ROS_INFO("  圆心：(%.3f, %.3f, %.3f)",
             obj->position.x, obj->position.y, obj->position.z);
    ROS_INFO("  半径：%.3f m", obj->radius);
}
```

### 完整示例

```cpp
#include <ros/ros.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <cmath>

class ObstacleAvoidanceNode
{
public:
    ObstacleAvoidanceNode(ros::NodeHandle& nh)
    {
        result_sub_ = nh.subscribe("/pcl_detection/result", 10,
                                   &ObstacleAvoidanceNode::resultCallback, this);
        ROS_INFO("避障节点已启动");
    }

    void run() { ros::spin(); }

private:
    void resultCallback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        if (!msg->success)
        {
            ROS_WARN("检测失败：%s", msg->status_message.c_str());
            return;
        }

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
        if (obj->plane_coeffs.size() < 4) return;

        double A = obj->plane_coeffs[0];
        double B = obj->plane_coeffs[1];
        double C = obj->plane_coeffs[2];
        double norm = std::sqrt(A*A + B*B + C*C);
        double nz = C / norm;

        if (std::abs(nz) < 0.3)  // 竖直墙面
        {
            ROS_INFO("检测到竖直墙面：%s", obj->name.c_str());
            // TODO: 避障逻辑
        }
    }

    void processCylinder(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        double dist = std::sqrt(obj->position.x * obj->position.x +
                               obj->position.y * obj->position.y);
        if (dist < obj->radius + 0.5)
        {
            ROS_WARN("圆柱过近：%s", obj->name.c_str());
            // TODO: 避障逻辑
        }
    }

    void processCircle(const pcl_detection::DetectedObject::ConstPtr& obj)
    {
        ROS_INFO("检测到圆环：%s", obj->name.c_str());
        // TODO: 避障逻辑
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

---

## 配置参数详解

### 检测参数（config/sample_config.yaml）

```yaml
# 墙面检测
wall_extraction:
  enable: true              # 是否启用
  using_normal: true        # 是否使用法线
  distance_threshold: 0.04  # 距离阈值（米）
  min_inliers: 1500         # 最小内点数
  angle_threshold: 15.0     # 角度阈值（度）
  axis: [0, 0, 1]           # 参考轴向

# 圆柱检测
cylinder_extraction:
  enable: true
  using_normal: false
  distance_threshold: 0.06
  min_inliers: 300
  radius_min: 0.1           # 最小半径（米）
  radius_max: 1.0           # 最大半径（米）
  axis: [0, 0, 1]
  eps_angle: 25.0           # 角度容差（度）

# 圆环检测
circle_extraction:
  enable: true
  using_normal: false
  distance_threshold: 0.03
  min_inliers: 80
  radius_min: 0.2
  radius_max: 0.7
  plane_distance_threshold: 0.06
  plane_angle_threshold: 45.0
  min_coverage_angle: 240.0  # 最小覆盖角度（度）
  ransac_iterations: 200
  max_circles: 5
```

### 日志控制（launch 文件）

```xml
<!-- 每 20 帧输出一次日志 -->
<param name="log_skip_frames" value="19" />

<!-- 或每 2 秒输出一次 -->
<param name="log_interval_sec" value="2.0" />

<!-- Livox 日志级别：0=无，1=摘要，2=详细 -->
<param name="livox_debug_level" value="0" />
```

### 时间测量

```yaml
timing:
  enable: true
  downsample: true
  walls: true
  cylinders: true
  circles: true
  total: true
```

---

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

---

## 目录结构

```
pcl_detection/
├── shell/                  # 启动脚本
│   └── obs.sh             # 一键启动脚本
├── launch/                 # Launch 文件
│   ├── object_detector.launch
│   ├── obstacle_listener.launch
│   └── points_sample.launch
├── config/                 # 配置文件
│   └── sample_config.yaml
├── msg/                    # 消息定义
│   ├── DetectedObject.msg
│   └── ObjectDetectionResult.msg
├── src/
│   ├── nodes/             # ROS 节点
│   │   ├── object_detector_node.cpp
│   │   ├── detection_visualizer.cpp
│   │   └── points_samples.cpp
│   └── core/              # 核心算法
├── include/               # 头文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 依赖

- ROS Noetic
- PCL 1.8+
- livox_ros_driver2
- Eigen3
- yaml-cpp

---

## 许可证

TODO

## 联系方式

TODO
