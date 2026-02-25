# PCL Detection ROS 节点

## 功能说明

该 ROS 节点实时接收 Livox 激光雷达点云数据，检测场景中的墙面、圆柱和圆环物体，并发布检测结果。

## 节点说明

### object_detector_node

**订阅话题：**
- `/livox/lidar` ([livox_ros_driver2/CustomMsg](file:///home/jetson/group2_ws/src/pcl_detection/include/pcl_object_detection/core/config.hpp#L5-L9)) - Livox 激光雷达点云数据

**发布话题：**
- `/pcl_detection/result` ([pcl_detection/ObjectDetectionResult](file:///home/jetson/group2_ws/devel/.private/pcl_detection/include/pcl_detection/ObjectDetectionResult.h)) - 检测结果汇总
- `/pcl_detection/objects` ([pcl_detection/DetectedObject](file:///home/jetson/group2_ws/devel/.private/pcl_detection/include/pcl_detection/DetectedObject.h)) - 单个检测结果

**参数：**
- `config_file` (string, 默认：`config/sample_config.yaml`) - 配置文件路径
- `input_topic` (string, 默认：`/livox/lidar`) - 输入点云话题
- `result_topic` (string, 默认：`/pcl_detection/result`) - 结果发布话题
- `objects_topic` (string, 默认：`/pcl_detection/objects`) - 单个物体发布话题
- `skip_frames` (int, 默认：`0`) - 跳帧处理（0=不跳帧，1=每 2 帧处理 1 次）

## 启动方法

```bash
# 启动检测节点
roslaunch pcl_detection object_detector.launch

# 查看检测结果
rostopic echo /pcl_detection/result

# 查看单个物体
rostopic echo /pcl_detection/objects
```

## 消息类型

### DetectedObject

```plaintext
Header header
int32 type              # 0=Wall, 1=Cylinder, 2=Circle
string name
geometry_msgs/Point position
float32 width
float32 height
float32 depth
float32 radius
float32[] plane_coeffs
int32 point_count
float32 extraction_time
```

### ObjectDetectionResult

```plaintext
Header header
DetectedObject[] objects
int32 wall_count
int32 cylinder_count
int32 circle_count
float32 total_time
float32 downsample_time
float32 normals_time
float32 wall_time
float32 cylinder_time
float32 circle_time
bool success
string status_message
```

## 性能优化

默认配置下，处理时间约 100ms：
- 下采样：~1ms
- 法向量估计：~20ms
- 墙面提取：~40ms
- 圆柱提取：~25ms
- 圆环提取：~10ms

如需提高处理速度，可以：
1. 设置 `skip_frames` 参数跳帧处理
2. 降低配置中的 `min_inliers` 和 `ransac_iterations`
3. 增大 `leaf_size` 降低点云分辨率

## 配置文件

编辑 `config/sample_config.yaml` 调整检测参数。
