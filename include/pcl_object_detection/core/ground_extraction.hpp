#pragma once

#include "obstacle_pipeline_config.hpp"
#include "timer.hpp"
#include "object_base.hpp"

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <vector>
#include <set>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 地面分割（使用 RANSAC 平面分割）
 * 
 * @param cloud 输入点云
 * @param config 地面分割配置
 * @param ground_indices 输出地面点索引
 * @param non_ground_indices 输出非地面点索引
 * @param ground_coefficients 地面平面系数
 * @return 是否成功分割
 */
template <typename PointT>
bool extractGround(typename pcl::PointCloud<PointT>::Ptr cloud,
                   const GroundConfig &config,
                   typename pcl::PointIndices::Ptr ground_indices,
                   typename pcl::PointIndices::Ptr non_ground_indices,
                   typename pcl::ModelCoefficients::Ptr ground_coefficients) {
    
    if (cloud->empty()) {
        ROS_ERROR("[GroundExtraction] 点云为空");
        return false;
    }

    // RANSAC 平面分割
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config.distance_threshold);
    seg.setMaxIterations(config.max_iterations);

    typename pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("[GroundExtraction] RANSAC 未找到地面平面");
        return false;
    }

    // 检查地面法向量
    // 注意：Livox 点云坐标系可能是 X 向前，所以地面法向量可能指向 X 轴而非 Z 轴
    // 这里我们检查法向量的最大分量，而不是强制要求 Z 分量
    Eigen::Vector3f normal(coefficients->values[0], 
                           coefficients->values[1], 
                           coefficients->values[2]);
    
    // 计算法向量与各轴的夹角余弦
    float normal_x = std::abs(normal[0]);
    float normal_y = std::abs(normal[1]);
    float normal_z = std::abs(normal[2]);
    
    // 找到最大分量
    float max_component = std::max({normal_x, normal_y, normal_z});
    
    // 判断地面方向
    std::string ground_axis = "Z";
    if (normal_x == max_component) {
        ground_axis = "X";  // 地面法向量指向 X 轴（点云 X 向前）
    } else if (normal_y == max_component) {
        ground_axis = "Y";  // 地面法向量指向 Y 轴
    }
    
    // ========== 逻辑过滤 1: 检查点数 ==========
    if (config.min_ground_points > 0 && 
        inliers->indices.size() < static_cast<size_t>(config.min_ground_points)) {
        ROS_WARN("[GroundExtraction] 地面点数不足：%lu < %d，过滤", 
                 inliers->indices.size(), config.min_ground_points);
        return false;
    }

    // ========== 逻辑过滤 2: 检查法向量 ==========
    // 检查最大分量是否接近 1（表示平面接近水平）
    if (max_component < config.normal_z_min) {
        ROS_WARN("[GroundExtraction] 地面法向量不够水平：max=%.3f (期望：>%.3f), 轴向：%s，过滤",
                 max_component, config.normal_z_min, ground_axis.c_str());
        return false;
    }
    
    ROS_DEBUG("[GroundExtraction] 地面法向量：(%.3f, %.3f, %.3f), 主轴：%s",
              normal[0], normal[1], normal[2], ground_axis.c_str());

    // 分离地面和非地面点
    ground_indices->indices = inliers->indices;
    
    std::set<int> ground_set(inliers->indices.begin(), inliers->indices.end());
    non_ground_indices->indices.clear();
    non_ground_indices->indices.reserve(cloud->size() - inliers->indices.size());
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (ground_set.find(i) == ground_set.end()) {
            non_ground_indices->indices.push_back(i);
        }
    }

    ground_coefficients = coefficients;

    ROS_DEBUG("[GroundExtraction] 地面点：%lu, 非地面点：%lu",
              ground_indices->indices.size(), non_ground_indices->indices.size());

    return true;
}

}  // namespace core
}  // namespace pcl_object_detection
