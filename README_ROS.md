# PCL Detection ROS 节点使用指南

## 功能说明

该 ROS 节点实时接收 Livox 激光雷达点云数据，检测场景中的**墙面、圆柱和圆环**物体，并发布检测结果到 ROS 话题，供其他模块（如避障、路径规划）使用。

## 快速开始

### 方式 1：一键启动脚本（推荐）

```bash
# 直接运行启动脚本
/home/jetson/group2_ws/src/pcl_detection/shell/obs.sh
```

脚本会自动：
1. 加载 ROS 环境
2. 启动 roscore（如果未运行）
3. 启动检测节点

### 方式 2：手动启动

```bash
# 1. 加载环境
cd /home/jetson/group2_ws
source devel/setup.bash  # 或 setup.zsh

# 2. 启动节点
roslaunch pcl_detection object_detector.launch
```

## 话题说明

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox 激光雷达点云数据 |

### 发布话题

| 话题 | 类型 | 说明 | 频率 |
|------|------|------|------|
| `/pcl_detection/result` | `pcl_detection/ObjectDetectionResult` | 检测结果汇总 | ~10Hz |
| `/pcl_detection/objects` | `pcl_detection/DetectedObject` | 单个检测结果 | ~10Hz |
| `/pcl_detection/visualization/marker` | `visualization_msgs/MarkerArray` | RViz 可视化标记 | ~10Hz |

## 在代码中订阅检测结果

### C++ 示例

```cpp
#include <ros/ros.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>

class ObstacleAvoidanceNode
{
public:
    ObstacleAvoidanceNode(ros::NodeHandle& nh)
    {
        // 订阅检测结果
        result_sub_ = nh.subscribe("/pcl_detection/result", 10, 
                                   &ObstacleAvoidanceNode::detectionCallback, this);
        
        // 或者订阅单个物体
        objects_sub_ = nh.subscribe("/pcl_detection/objects", 100,
                                    &ObstacleAvoidanceNode::objectCallback, this);
    }

private:
    // 方式 1：接收检测结果汇总
    void detectionCallback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        ROS_INFO("检测到 %lu 个物体", msg->objects.size());
        ROS_INFO("墙体：%d 个，圆柱：%d 个，圆环：%d 个",
                 msg->wall_count, msg->cylinder_count, msg->circle_count);
        
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
        std::string type_name;
        switch (obj->type)
        {
            case 0: type_name = "墙体 (Wall)"; break;
            case 1: type_name = "圆柱 (Cylinder)"; break;
            case 2: type_name = "圆环 (Circle)"; break;
            default: type_name = "未知";
        }
        
        ROS_INFO("物体：%s", obj->name.c_str());
        ROS_INFO("  类型：%s", type_name.c_str());
        ROS_INFO("  位置：(%.2f, %.2f, %.2f)", 
                 obj->position.x, obj->position.y, obj->position.z);
        ROS_INFO("  尺寸：%.2f x %.2f x %.2f", 
                 obj->width, obj->height, obj->depth);
        ROS_INFO("  点数：%d", obj->point_count);
        
        // 根据物体类型处理
        if (obj->type == 0)  // 墙体
        {
            // 获取平面方程参数
            // plane_coeffs: [A, B, C, D] 对应 Ax + By + Cz + D = 0
            double A = obj->plane_coeffs[0];
            double B = obj->plane_coeffs[1];
            double C = obj->plane_coeffs[2];
            double D = obj->plane_coeffs[3];
            
            ROS_INFO("  平面方程：%.3fx + %.3fy + %.3fz + %.3f = 0", A, B, C, D);
        }
        else if (obj->type == 1 || obj->type == 2)  // 圆柱或圆环
        {
            ROS_INFO("  半径：%.3f m", obj->radius);
        }
        
        // TODO: 在这里添加你的避障逻辑
        // 例如：计算与无人机的距离，判断是否需要避障
    }

private:
    ros::Subscriber result_sub_;
    ros::Subscriber objects_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");
    ros::NodeHandle nh;
    
    ObstacleAvoidanceNode node(nh);
    
    ros::spin();
    
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from pcl_detection.msg import ObjectDetectionResult, DetectedObject

class ObstacleAvoidanceNode:
    def __init__(self):
        # 订阅检测结果
        rospy.Subscriber("/pcl_detection/result", ObjectDetectionResult, 
                        self.detection_callback)
        
        # 或者订阅单个物体
        rospy.Subscriber("/pcl_detection/objects", DetectedObject,
                        self.object_callback)
        
        self.type_names = {
            0: "墙体 (Wall)",
            1: "圆柱 (Cylinder)",
            2: "圆环 (Circle)"
        }
    
    def detection_callback(self, msg):
        """接收检测结果汇总"""
        rospy.loginfo("检测到 %d 个物体", len(msg.objects))
        rospy.loginfo("墙体：%d 个，圆柱：%d 个，圆环：%d 个",
                     msg.wall_count, msg.cylinder_count, msg.circle_count)
        
        for obj in msg.objects:
            self.process_object(obj)
    
    def object_callback(self, obj):
        """接收单个物体"""
        self.process_object(obj)
    
    def process_object(self, obj):
        """处理单个物体"""
        type_name = self.type_names.get(obj.type, "未知")
        
        rospy.loginfo("物体：%s", obj.name)
        rospy.loginfo("  类型：%s", type_name)
        rospy.loginfo("  位置：(%.2f, %.2f, %.2f)",
                     obj.position.x, obj.position.y, obj.position.z)
        rospy.loginfo("  尺寸：%.2f x %.2f x %.2f",
                     obj.width, obj.height, obj.depth)
        rospy.loginfo("  点数：%d", obj.point_count)
        
        # 根据物体类型处理
        if obj.type == 0:  # 墙体
            # 获取平面方程参数
            A, B, C, D = obj.plane_coeffs
            rospy.loginfo("  平面方程：%.3fx + %.3fy + %.3fz + %.3f = 0",
                         A, B, C, D)
        elif obj.type in [1, 2]:  # 圆柱或圆环
            rospy.loginfo("  半径：%.3f m", obj.radius)
        
        # TODO: 在这里添加你的避障逻辑

if __name__ == "__main__":
    try:
        rospy.init_node("obstacle_avoidance_node")
        node = ObstacleAvoidanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

## 消息类型详解

### DetectedObject.msg

```plaintext
std_msgs/Header header      # 消息头
int32 type                  # 物体类型：0=墙体，1=圆柱，2=圆环
string name                 # 物体名称（如 wall_1, cylinder_2）
geometry_msgs/Point position # 物体中心位置
float32 width               # 宽度（米）
float32 height              # 高度（米）
float32 depth               # 深度（米）
float32 radius              # 半径（圆柱/圆环特有）
float32[] plane_coeffs      # 平面方程参数 [A,B,C,D]
int32 point_count           # 点云内点数量
float32 extraction_time     # 提取耗时（毫秒）
```

### ObjectDetectionResult.msg

```plaintext
std_msgs/Header header      # 消息头
DetectedObject[] objects    # 所有检测到的物体
int32 wall_count            # 墙体数量
int32 cylinder_count        # 圆柱数量
int32 circle_count          # 圆环数量
float32 total_time          # 总处理时间（毫秒）
float32 downsample_time     # 下采样耗时
float32 normals_time        # 法向量估计耗时
float32 wall_time           # 墙面提取耗时
float32 cylinder_time       # 圆柱提取耗时
float32 circle_time         # 圆环提取耗时
bool success                # 检测是否成功
string status_message       # 状态信息
```

## 配置参数

### 通过 launch 文件配置

编辑 `launch/object_detector.launch`：

```xml
<launch>
    <node pkg="pcl_detection" type="pcl_detection_detector_node" name="object_detector">
        <!-- 配置文件 -->
        <param name="config_file" value="$(find pcl_detection)/config/sample_config.yaml" />
        
        <!-- 日志控制 -->
        <param name="log_skip_frames" value="19" />      <!-- 每 20 帧输出一次 -->
        <param name="log_interval_sec" value="2.0" />    <!-- 每 2 秒输出一次 -->
        <param name="livox_debug_level" value="0" />     <!-- 0=无，1=摘要，2=详细 -->
    </node>
</launch>
```

### 通过命令行覆盖

```bash
# 关闭所有调试日志
roslaunch pcl_detection object_detector.launch livox_debug_level:=0

# 显示详细 Livox 点云信息
roslaunch pcl_detection object_detector.launch livox_debug_level:=2
```

### 通过 YAML 配置文件

编辑 `config/sample_config.yaml` 可调整检测算法参数：

```yaml
# 墙面检测参数
wall_extraction:
  min_inliers: 500        # 最小内点数
  distance_threshold: 0.08 # 距离阈值（米）
  angle_threshold: 15.0   # 角度阈值（度）

# 圆柱检测参数
cylinder_extraction:
  min_inliers: 100
  radius_min: 0.1         # 最小半径（米）
  radius_max: 0.4         # 最大半径（米）

# 圆环检测参数
circle_extraction:
  min_inliers: 80
  radius_min: 0.2
  radius_max: 0.7
  min_coverage_angle: 240.0  # 最小覆盖角度（度）
```

## RViz 可视化

### 启动 RViz

```bash
# 方式 1：手动启动
rviz

# 方式 2：在 launch 文件中添加（取消注释）
# <node pkg="rviz" type="rviz" name="rviz" args="-d $(find pcl_detection)/config/detection.rviz" />
```

### 添加显示项

1. **添加 MarkerArray**
   - Display Type: `By topic`
   - Topic: `/pcl_detection/visualization/marker`
   - 颜色：墙体=红色，圆柱=绿色，圆环=蓝色

2. **添加 PointCloud2**（可选）
   - Topic: `/livox/lidar`
   - 用于查看原始点云

## 性能优化

### 默认性能（Jetson Xavier NX）

- 总处理时间：~70ms
- 下采样：~1ms
- 法向量估计：~25ms
- 墙面提取：~25ms
- 圆柱提取：~5ms
- 圆环提取：~5ms

### 优化建议

1. **降低点云分辨率**
   ```yaml
   downsample_config:
     leaf_size: 0.05  # 增大此值可提高速度
   ```

2. **减少检测对象数量**
   ```yaml
   circle_extraction:
     max_circles: 3  # 限制最大检测数量
   ```

3. **跳帧处理**
   ```xml
   <param name="skip_frames" value="1" />  <!-- 每 2 帧处理 1 次 -->
   ```

## 常见问题

### Q1: 检测不到物体？

**A:** 检查以下几点：
1. Livox 雷达是否正常工作：`rostopic hz /livox/lidar`
2. 点云质量是否足够：`rostopic echo /livox/lidar/point_num`
3. 调整检测参数（减小 `min_inliers`）

### Q2: 误检太多？

**A:** 提高检测阈值：
```yaml
wall_extraction:
  min_inliers: 800  # 增大此值
circle_extraction:
  min_coverage_angle: 270.0  # 提高覆盖度要求
```

### Q3: 处理速度太慢？

**A:** 参考上方"性能优化"章节

## 依赖项

- ROS Noetic
- PCL 1.8+
- livox_ros_driver2
- Eigen3
- yaml-cpp

## 许可证

TODO

## 联系方式

TODO
