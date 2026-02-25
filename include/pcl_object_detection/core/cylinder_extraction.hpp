#pragma once

#include "config.hpp"
#include "object_base.hpp"

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

// 提取圆柱
template <typename PointT>
std::vector<Object::Ptr> extractCylinders(typename pcl::PointCloud<PointT>::Ptr cloud,
                                          typename pcl::PointCloud<pcl::Normal>::Ptr normals,
                                          const CylinderConfig &config,
                                          const std::set<int>& excluded_indices = {}) {

    std::vector<Object::Ptr> cylinders;

    // 如果使用法向量但没有提供，则返回空
    if (config.using_normal && !normals) {
        return cylinders;
    }

    // 创建分割对象
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config.distance_threshold);
    seg.setMaxIterations(1000);

    // 设置圆柱参数
    seg.setRadiusLimits(config.radius_min, config.radius_max);
    seg.setAxis(Eigen::Vector3f(config.axis[0], config.axis[1], config.axis[2]));
    seg.setEpsAngle(config.eps_angle * M_PI / 180.0f);

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
    int cylinder_num                                = 1;

    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers)) {
        // 设置当前索引
        seg.setIndices(current_indices);
        seg.setInputCloud(cloud);
        if (config.using_normal) {
            seg.setInputNormals(normals);
        }

        // 执行分割
        typename pcl::PointIndices::Ptr inliers_local(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        auto start = std::chrono::high_resolution_clock::now();
        seg.segment(*inliers_local, *coefficients);
        auto end     = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(end - start).count();

        // 检查内点数量
        if (inliers_local->indices.size() < static_cast<size_t>(config.min_inliers)) {
            break;
        }

        // 检查是否与已提取的圆柱重复
        bool is_duplicate = false;
        for (const auto &cylinder : cylinders) {
            // 比较圆柱参数（中心、轴、半径）
            float dx   = coefficients->values[0] - cylinder->coefficients->values[0];
            float dy   = coefficients->values[1] - cylinder->coefficients->values[1];
            float dz   = coefficients->values[2] - cylinder->coefficients->values[2];
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            // 检查中心点距离和半径
            if (dist < 0.01 &&
                std::abs(coefficients->values[6] - cylinder->coefficients->values[6]) < 0.01)
            {
                is_duplicate = true;
                break;
            }
        }

        if (is_duplicate) {
            // 从当前索引中移除这些点
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (size_t i = 0; i < current_indices->indices.size(); ++i) {
                if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(),
                              current_indices->indices[i]) == inliers_local->indices.end())
                {
                    new_indices->indices.push_back(current_indices->indices[i]);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 计算尺寸
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, *inliers_local, min_pt, max_pt);

        float height  = max_pt[2] - min_pt[2];
        float width   = 2 * coefficients->values[6];  // 直径
        float depth   = width;                        // 暂时用直径作为深度

        // 保存圆柱信息
        auto cylinder = Object::createCylinder("cylinder_" + std::to_string(cylinder_num),
                                               inliers_local, coefficients, elapsed, width, height,
                                               depth, config.using_normal);

        cylinders.push_back(cylinder);

        // 从当前索引中移除这些点
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (size_t i = 0; i < current_indices->indices.size(); ++i) {
            if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(),
                          current_indices->indices[i]) == inliers_local->indices.end())
            {
                new_indices->indices.push_back(current_indices->indices[i]);
            }
        }
        current_indices = new_indices;

        cylinder_num++;

        // 防止无限循环
        if (cylinder_num > 20) {
            break;
        }
    }

    return cylinders;
}

}  // namespace core
}  // namespace pcl_object_detection