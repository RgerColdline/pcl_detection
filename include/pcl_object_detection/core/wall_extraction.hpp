#pragma once

#include "object_base.hpp"

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <Eigen/Geometry>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

// 提取多个垂直墙面
template <typename PointT>
std::vector<WallInfo<PointT>>
extractWalls(typename pcl::PointCloud<PointT>::Ptr cloud,
             typename pcl::PointCloud<pcl::Normal>::Ptr normals,  // 可能为nullptr
             const WallConfig &config) {
    std::vector<WallInfo<PointT>> walls;

    // 如果使用法向量但没有提供，则返回空
    if (config.using_normal && !normals) {
        return walls;
    }

    // 创建分割对象
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    // 提取法向量与指定轴垂直的平面（即墙面法向量在XY平面内）
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config.distance_threshold);
    seg.setMaxIterations(1000);
    // 设置轴
    seg.setAxis(Eigen::Vector3f(config.axis[0], config.axis[1], config.axis[2]));
    // 设置角度阈值（允许法向量与指定轴有±angle_threshold度的偏差）
    seg.setEpsAngle(config.angle_threshold * M_PI / 180.0f);

    // 创建初始索引
    typename pcl::PointIndices::Ptr all_indices(new pcl::PointIndices);
    all_indices->indices.resize(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) all_indices->indices[i] = i;

    // 当前要处理的索引
    typename pcl::PointIndices::Ptr current_indices = all_indices;

    int wall_num                                    = 1;

    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers)) {
        // 设置当前要处理的索引
        seg.setIndices(current_indices);
        seg.setInputCloud(cloud);
        if (config.using_normal) {
            seg.setInputNormals(normals);
        }

        // 执行分割
        typename pcl::PointIndices::Ptr inliers_local(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // 计时
        auto start = std::chrono::high_resolution_clock::now();
        seg.segment(*inliers_local, *coefficients);
        auto end     = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(end - start).count();

        // 检查是否找到足够的内点
        if (inliers_local->indices.size() < static_cast<size_t>(config.min_inliers)) {
            break;
        }

        // 检查是否与已提取的墙面重复
        bool is_duplicate = false;
        for (const auto &wall : walls) {
            // 计算法向量夹角
            Eigen::Vector3f normal1(coefficients->values[0], coefficients->values[1],
                                    coefficients->values[2]);
            Eigen::Vector3f normal2(wall.coefficients->values[0], wall.coefficients->values[1],
                                    wall.coefficients->values[2]);

            float dot    = normal1.dot(normal2);
            dot          = std::max(-1.0f, std::min(1.0f, dot));
            double angle = std::acos(dot) * 180.0 / M_PI;

            if (angle < config.angle_threshold || angle > (180.0 - config.angle_threshold)) {
                // 进一步检查距离
                double d1 = coefficients->values[3];
                double d2 = wall.coefficients->values[3];

                if (std::abs(d1 - d2) < 0.1) {
                    is_duplicate = true;
                    break;
                }
            }
        }

        if (is_duplicate) {
            // 从当前索引中移除这些点
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (size_t i = 0; i < current_indices->indices.size(); ++i) {
                // 比较原始点云索引，而不是位置索引
                if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(),
                              current_indices->indices[i]) == inliers_local->indices.end())
                {
                    new_indices->indices.push_back(current_indices->indices[i]);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 保存墙面信息
        WallInfo<PointT> wall;
        wall.inliers         = inliers_local;  // 保存的是原始点云的索引
        wall.coefficients    = coefficients;
        wall.name            = "wall_" + std::to_string(wall_num);
        wall.extraction_time = elapsed;
        wall.using_normal    = config.using_normal;
        walls.push_back(wall);

        // 从当前索引中移除这些点
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (size_t i = 0; i < current_indices->indices.size(); ++i) {
            // 比较原始点云索引，而不是位置索引
            if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(),
                          current_indices->indices[i]) == inliers_local->indices.end())
            {
                new_indices->indices.push_back(current_indices->indices[i]);
            }
        }
        current_indices = new_indices;

        wall_num++;

        // 防止无限循环
        if (wall_num > 20) {
            break;
        }
    }

    return walls;
}

}  // namespace core
}  // namespace pcl_object_detection