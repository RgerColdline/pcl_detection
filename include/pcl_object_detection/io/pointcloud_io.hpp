#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace pcl_object_detection
{
namespace core
{
struct ObjectDetectionConfig;  // 前向声明
}  // namespace core

namespace io
{

// 从YAML文件加载配置
bool load_config(const std::string &config_file, core::ObjectDetectionConfig &config);

// 加载点云
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPointCloud(const std::string &file_path) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1) {
        std::cerr << "Error loading PCD file: " << file_path << "\n";
        return nullptr;
    }
    return cloud;
}

// 保存对象点云
template <typename PointT>
bool saveObject(typename pcl::PointCloud<PointT>::Ptr cloud,
                typename pcl::PointIndices::Ptr inliers, const std::string &output_dir,
                const std::string &object_name) {
    // 创建目录
    std::filesystem::create_directories(output_dir);

    // 提取对象点云
    typename pcl::PointCloud<PointT>::Ptr object_cloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*object_cloud);

    // 保存
    std::string filename = output_dir + "/" + object_name + ".pcd";
    int result           = pcl::io::savePCDFile(filename, *object_cloud);

    // 添加日志
    if (result >= 0) {
        std::cout << "  保存至: " << filename << " (" << object_cloud->size() << " points)"
                  << std::endl;
        return true;
    }
    else {
        std::cerr << "  保存失败: " << filename << " (错误码: " << result << ")" << std::endl;
        return false;
    }
}

}  // namespace io
}  // namespace pcl_object_detection