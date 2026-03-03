#pragma once

#include "obstacle_pipeline_config.hpp"
#include "timer.hpp"
#include "object_base.hpp"
#include "object_factory.hpp"
#include "logger_limiter.hpp"
#include "centroid_utils.hpp"

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <vector>
#include <set>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 欧式聚类障碍物
 * 
 * @param cloud 输入点云（非地面点）
 * @param config 聚类配置
 * @param excluded_indices 需要排除的点索引（墙体、方环已用点）
 * @param clusters 输出的聚类结果（每个聚类的点索引）
 * @return 成功聚类的数量
 */
template <typename PointT>
int clusterObstacles(typename pcl::PointCloud<PointT>::Ptr cloud,
                     const ClusterConfig &config,
                     const std::set<int> &excluded_indices,
                     std::vector<typename pcl::PointIndices::Ptr> &clusters) {
    
    clusters.clear();
    
    if (cloud->empty()) {
        ROS_WARN("[ClusterExtraction] 输入点云为空");
        return 0;
    }

    // 创建可用索引（排除已用点）
    typename pcl::PointIndices::Ptr available_indices(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (excluded_indices.find(i) == excluded_indices.end()) {
            available_indices->indices.push_back(i);
        }
    }

    if (available_indices->indices.empty()) {
        ROS_DEBUG("[ClusterExtraction] 没有可用点进行聚类");
        return 0;
    }

    ROS_DEBUG("[ClusterExtraction] 可用点数：%lu", available_indices->indices.size());

    // 构建 KD 树
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // 欧式聚类
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(config.cluster_tolerance);  // 聚类距离阈值
    ec.setMinClusterSize(config.min_cluster_size);
    ec.setMaxClusterSize(config.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.setIndices(available_indices);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 转换输出格式
    for (auto &cluster : cluster_indices) {
        typename pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        indices->indices = cluster.indices;
        clusters.push_back(indices);
    }

    // 输出聚类结果（每 N 帧输出一次）
    if (LOG_SHOULD()) {
        ROS_INFO("[ClusterExtraction] 找到 %lu 个聚类", clusters.size());
    }

    return clusters.size();
}

/**
 * @brief 创建障碍物对象（从聚类结果）
 * 
 * @param cloud 原始点云
 * @param cluster_indices 聚类索引
 * @param cluster_id 聚类 ID
 * @param extraction_time 提取时间
 * @return 障碍物对象
 */
template <typename PointT>
typename Object::Ptr createObstacleFromCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    typename pcl::PointIndices::Ptr cluster_indices,
    int cluster_id,
    double extraction_time) {

    // 创建模型系数（存储质心位置）
    typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(3);

    // 计算质心：使用体素化质心（避免点云分布不均导致的偏移）
    Eigen::Vector4f centroid = core::computeRobustCentroid(*cloud, *cluster_indices, "Obstacle", 0.1f);

    coefficients->values[0] = centroid[0];
    coefficients->values[1] = centroid[1];
    coefficients->values[2] = centroid[2];

    // 创建障碍物对象（使用 ObjectFactory）
    auto obstacle = ObjectFactory::createObstacle(
        "obstacle_" + std::to_string(cluster_id),
        cluster_indices,
        coefficients,
        extraction_time,
        0.0f, 0.0f, 0.0f  // width, height, depth（后续会更新）
    );

    // 计算尺寸（使用 getMinMax3D）
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, *cluster_indices, min_pt, max_pt);

    obstacle->width = max_pt[0] - min_pt[0];   // X 方向尺寸
    obstacle->height = max_pt[2] - min_pt[2];  // Z 方向尺寸（高度）
    obstacle->depth = max_pt[1] - min_pt[1];   // Y 方向尺寸

    return obstacle;
}

}  // namespace core
}  // namespace pcl_object_detection
