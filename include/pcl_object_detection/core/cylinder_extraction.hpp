#pragma once

#include "config.hpp"
#include "object_base.hpp"
#include "extractors_base.hpp"
#include "timer.hpp"

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <Eigen/Geometry>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

// 提取圆柱（支持使用或不使用法向量）
template <typename PointT>
std::vector<Object::Ptr> extractCylinders(typename pcl::PointCloud<PointT>::Ptr cloud,
                                          typename pcl::PointCloud<pcl::Normal>::Ptr normals,
                                          const CylinderConfig &config,
                                          const std::set<int>& excluded_indices = {}) {

    std::vector<Object::Ptr> cylinders;

    if (cloud->empty()) {
        return cylinders;
    }

    // 圆柱提取必须使用法向量
    bool use_normals = true;
    if (!normals) {
        ROS_WARN("圆柱提取：法向量为空，无法提取圆柱");
        return cylinders;
    }
    
    int max_iterations = 300;  // 使用法向量时的迭代次数

    // 创建初始索引（排除已用点）
    typename pcl::PointIndices::Ptr all_indices(new pcl::PointIndices);
    if (excluded_indices.empty()) {
        all_indices->indices.resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) all_indices->indices[i] = i;
    } else {
        all_indices->indices.reserve(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            if (excluded_indices.find(i) == excluded_indices.end()) {
                all_indices->indices.push_back(i);
            }
        }
    }

    typename pcl::PointIndices::Ptr current_indices = all_indices;
    int cylinder_num = 1;
    int max_cylinders = 5;

    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers) && cylinder_num <= max_cylinders) {
        Timer timer;  // 开始计时当前圆柱的提取

        typename pcl::PointIndices::Ptr inliers_local(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        if (use_normals) {
            // 使用法向量的圆柱拟合
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(config.distance_threshold);
            seg.setMaxIterations(max_iterations);
            seg.setRadiusLimits(config.radius_min, config.radius_max);
            seg.setAxis(Eigen::Vector3f(config.axis[0], config.axis[1], config.axis[2]));
            seg.setEpsAngle(config.eps_angle * M_PI / 180.0f);
            seg.setIndices(current_indices);
            seg.setInputCloud(cloud);
            seg.setInputNormals(normals);
            seg.segment(*inliers_local, *coefficients);
        } else {
            // 不使用法向量的圆柱拟合
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(config.distance_threshold);
            seg.setMaxIterations(max_iterations);
            seg.setRadiusLimits(config.radius_min, config.radius_max);
            seg.setIndices(current_indices);
            seg.setInputCloud(cloud);
            seg.segment(*inliers_local, *coefficients);
        }

        if (inliers_local->indices.size() < static_cast<size_t>(config.min_inliers)) {
            break;
        }

        // 从当前索引中移除这些点
        std::set<int> inlier_set(inliers_local->indices.begin(), inliers_local->indices.end());
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (int idx : current_indices->indices) {
            if (inlier_set.find(idx) == inlier_set.end()) {
                new_indices->indices.push_back(idx);
            }
        }
        current_indices = new_indices;

        // 二次检查：确保内点数量足够（防止 RANSAC 误判）
        if (inliers_local->indices.size() < static_cast<size_t>(config.min_inliers * 1.5f)) {
            // 内点数量刚过阈值，质量可能不好，跳过
            continue;
        }

        // 计算尺寸
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, *inliers_local, min_pt, max_pt);

        float height = max_pt[2] - min_pt[2];
        float width = 2 * coefficients->values[6];
        float depth = width;

        // 创建圆柱对象（记录提取时间）
        auto cylinder = Object::createCylinder("cylinder_" + std::to_string(cylinder_num),
                                               inliers_local, coefficients, timer.elapsed(), width, height,
                                               depth, use_normals);
        cylinders.push_back(cylinder);

        cylinder_num++;
    }

    return cylinders;
}

// ============================================================================
// 圆柱提取器（封装 extractCylinders 函数）
// ============================================================================
template <typename PointT>
class CylinderExtractor : public IObjectExtractor<PointT> {
    CylinderConfig config_;
public:
    using PointCloudPtrT = typename pcl::PointCloud<PointT>::Ptr;
    using NormalCloudPtrT = typename pcl::PointCloud<pcl::Normal>::Ptr;
    
    explicit CylinderExtractor(const CylinderConfig& config) : config_(config) {}
    
    std::vector<Object::Ptr> extract(
        PointCloudPtrT cloud,
        NormalCloudPtrT normals,
        const std::set<int>& excluded,
        PointCloudPtrT /*temp_cloud1*/ = nullptr,
        PointCloudPtrT /*temp_cloud2*/ = nullptr) override 
    {
        return extractCylinders<PointT>(cloud, normals, config_, excluded);
    }
};

}  // namespace core
}  // namespace pcl_object_detection
