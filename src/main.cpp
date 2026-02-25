#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <pcl_object_detection/pipeline.hpp>

#include <chrono>
#include <cmath>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh("~");

    // 从参数服务器获取参数
    std::string config_file, input_pcd;
    if (!nh.getParam("config", config_file) || !nh.getParam("input_pcd", input_pcd)) {
        std::cerr << "Error: Missing required parameters 'config' or 'input_pcd'" << std::endl;
        return -1;
    }

    // 创建应用流水线并运行
    pcl_object_detection::ApplicationPipeline<pcl::PointXYZI> pipeline;
    if (!pipeline.run(config_file, input_pcd)) {
        return -1;
    }

    return 0;
}