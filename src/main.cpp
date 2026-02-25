#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <pcl_object_detection/pipeline.hpp>

#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh("~");

    std::string config_file, input_pcd;
    
    // 从 ROS 参数服务器获取配置
    if (!nh.getParam("config", config_file) || config_file.empty()) {
        ROS_ERROR("Missing 'config' parameter. Usage: rosrun pcl_detection pcl_detection_node _config:=<config.yaml> _input_pcd:=<input.pcd>");
        return -1;
    }
    
    if (!nh.getParam("input_pcd", input_pcd) || input_pcd.empty()) {
        ROS_ERROR("Missing 'input_pcd' parameter. Usage: rosrun pcl_detection pcl_detection_node _config:=<config.yaml> _input_pcd:=<input.pcd>");
        return -1;
    }

    ROS_INFO("Config file: %s", config_file.c_str());
    ROS_INFO("Input PCD file: %s", input_pcd.c_str());

    // 创建应用流水线
    pcl_object_detection::ApplicationPipeline<pcl::PointXYZI> pipeline;

    // 运行应用
    if (!pipeline.run(config_file, input_pcd)) {
        ROS_ERROR("Pipeline execution failed");
        return -1;
    }

    return 0;
}