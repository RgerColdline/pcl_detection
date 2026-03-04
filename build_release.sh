#!/bin/bash
# 发布精简版本脚本

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_DIR="${REPO_DIR}/release_build"

echo "==================================="
echo "  PCL 障碍物检测 - 发布版本打包"
echo "==================================="

# 清理旧版本
rm -rf "${RELEASE_DIR}"
mkdir -p "${RELEASE_DIR}/pcl_detection"

# 复制必需文件
echo "复制核心文件..."

# 包配置
cp "${REPO_DIR}/CMakeLists.txt" "${RELEASE_DIR}/pcl_detection/"
cp "${REPO_DIR}/package.xml" "${RELEASE_DIR}/pcl_detection/"

# 消息定义
mkdir -p "${RELEASE_DIR}/pcl_detection/msg"
cp "${REPO_DIR}/msg/"*.msg "${RELEASE_DIR}/pcl_detection/msg/"

# 配置文件
mkdir -p "${RELEASE_DIR}/pcl_detection/config"
cp "${REPO_DIR}/config/sample_config.yaml" "${RELEASE_DIR}/pcl_detection/config/"

# 启动文件
mkdir -p "${RELEASE_DIR}/pcl_detection/launch"
cp "${REPO_DIR}/launch/obstacle_detection.launch" "${RELEASE_DIR}/pcl_detection/launch/"

# 头文件
mkdir -p "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core"
mkdir -p "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/ros"

cp "${REPO_DIR}/include/pcl_object_detection/core/"*.hpp \
   "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/"

cp "${REPO_DIR}/include/pcl_object_detection/ros/"*.hpp \
   "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/ros/"

# 源代码
mkdir -p "${RELEASE_DIR}/pcl_detection/src/nodes"
cp "${REPO_DIR}/src/nodes/object_detector_node.cpp" \
   "${RELEASE_DIR}/pcl_detection/src/nodes/"

# 文档
cp "${REPO_DIR}/README_RELEASE.md" "${RELEASE_DIR}/pcl_detection/README.md"

# 删除不需要的文件
echo "清理不需要的文件..."

# 删除旧 Pipeline 相关头文件
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/pipeline.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/cylinder_extraction.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/circle_extraction.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/cylinder.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/cylinder_object.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/circle.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/circle_object.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/rectangle_extraction.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/rectangle.hpp"
rm -f "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/core/rectangle_object.hpp"

# 删除可视化相关
rm -rf "${RELEASE_DIR}/pcl_detection/include/pcl_object_detection/visualization"

# 删除不需要的源文件
rm -f "${RELEASE_DIR}/pcl_detection/src/nodes/detection_visualizer.cpp"
rm -f "${RELEASE_DIR}/pcl_detection/src/nodes/obstacle_listener.cpp"
rm -f "${RELEASE_DIR}/pcl_detection/src/nodes/points_samples.cpp"
rm -f "${RELEASE_DIR}/pcl_detection/src/main.cpp"

# 创建打包
echo "创建压缩包..."
cd "${RELEASE_DIR}"
tar -czf pcl_detection_release.tar.gz pcl_detection/

echo "==================================="
echo "  发布版本打包完成！"
echo "  位置：${RELEASE_DIR}/pcl_detection_release.tar.gz"
echo "==================================="
echo ""
echo "使用方法："
echo "  1. 复制 pcl_detection_release.tar.gz 到目标工作空间"
echo "  2. tar -xzf pcl_detection_release.tar.gz -C ~/catkin_ws/src/"
echo "  3. cd ~/catkin_ws && catkin build pcl_detection"
echo "  4. source devel/setup.bash"
echo "  5. roslaunch pcl_detection obstacle_detection.launch"
