cmake_minimum_required(VERSION 3.0.2)
project(pcl_detection)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  message_generation
  geometry_msgs
  livox_ros_driver2
)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 添加消息文件
add_message_files(
  FILES
  DetectedObject.msg
  ObjectDetectionResult.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 包配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pcl_conversions pcl_ros message_runtime geometry_msgs
  DEPENDS PCL Eigen3 yaml-cpp
)

# 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# === 核心修改：收集所有源文件 ===
# 方法 1: 手动列出（推荐，清晰可控）
set(CORE_SOURCES
  src/core/object_base.cpp
)

set(IO_SOURCES
  src/io/pointcloud_io.cpp
)

set(MAIN_SOURCES
  src/main.cpp
)

set(DETECTOR_SOURCES
  src/nodes/object_detector_node.cpp
)

set(VIZ_SOURCES
  src/nodes/detection_visualizer.cpp
)

set(FLIGHT_SOURCES
  src/nodes/points_samples.cpp
)

# 方法 2: 自动收集（省事，但可能误收）
# file(GLOB_RECURSE CORE_SOURCES "src/core/*.cpp")
# file(GLOB_RECURSE UTIL_SOURCES "src/utils/*.cpp")

# 添加可执行文件
add_executable(${PROJECT_NAME}_node
  ${MAIN_SOURCES}
  ${IO_SOURCES}
  ${CORE_SOURCES}
)

# 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 检测器节点
add_executable(${PROJECT_NAME}_detector_node
  ${DETECTOR_SOURCES}
  ${CORE_SOURCES}
  ${IO_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 可视化节点
add_executable(${PROJECT_NAME}_visualizer_node
  ${VIZ_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_visualizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_visualizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 飞行控制节点
add_executable(${PROJECT_NAME}_samples_node
  ${FLIGHT_SOURCES}
  ${CORE_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_samples_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 障碍物监听测试节点
add_executable(${PROJECT_NAME}_obstacle_listener
  src/nodes/obstacle_listener.cpp
)

target_link_libraries(${PROJECT_NAME}_obstacle_listener
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_obstacle_listener
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 安装规则
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_detector_node ${PROJECT_NAME}_visualizer_node ${PROJECT_NAME}_samples_node ${PROJECT_NAME}_obstacle_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config/ launch/ msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)