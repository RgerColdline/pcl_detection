#pragma once

#include "obstacle_pipeline_config.hpp"
#include "timer.hpp"

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

#include <vector>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 多线程法向量估计
 * 
 * 使用 PCL 的 OpenMP 加速版本（NormalEstimationOMP）
 * 如果不可用，则回退到单线程版本
 */
template <typename PointT>
typename pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    const NormalEstimationConfig& config)
{
    typename pcl::PointCloud<pcl::Normal>::Ptr normals(
        new pcl::PointCloud<pcl::Normal>);
    
    if (cloud->empty() || !config.enable) {
        return normals;
    }
    
    Timer timer;
    
    // 构建 KD 树
    typename pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    
    // 尝试使用 OMP 加速版本
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setNumberOfThreads(config.num_threads);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(config.k_search);
    
    // 设置视角（用于法向量定向）
    if (config.view_angle != 0.0f) {
        Eigen::Vector4f viewpoint;
        viewpoint << 0, 0, 0, 0;  // 原点作为视角
        ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
    }
    
    ne.compute(*normals);
    
    ROS_DEBUG("[NormalEstimation] 法向量估计完成：%lu 个点，耗时：%.2f ms (threads=%d)",
              normals->size(), timer.elapsed(), config.num_threads);
    
    return normals;
}

}  // namespace core
}  // namespace pcl_object_detection
