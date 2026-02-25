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

// 提取多个垂直墙面
template <typename PointT>
std::vector<Object::Ptr> extractWalls(typename pcl::PointCloud<PointT>::Ptr cloud,
                                      typename pcl::PointCloud<pcl::Normal>::Ptr normals,
                                      const WallConfig &config,
                                      const std::set<int>& excluded_indices = {}) {

    std::vector<Object::Ptr> walls;

    if (cloud->empty()) {
        return walls;
    }

    // 创建分割对象 - 使用普通平面模型
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config.distance_threshold);
    seg.setMaxIterations(1000);

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

    // 当前要处理的索引
    typename pcl::PointIndices::Ptr current_indices = all_indices;

    int wall_num = 1;

    // 角度阈值转换为弧度（用于判断法向量是否接近水平）
    // 竖直墙面的法向量应该接近水平（与 Z 轴夹角接近 90 度）
    // angle_threshold 表示允许偏离水平面的最大角度
    float max_z_component = std::sin(config.angle_threshold * M_PI / 180.0f);

    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers)) {
        // 设置当前要处理的索引
        seg.setIndices(current_indices);
        seg.setInputCloud(cloud);
        if (config.using_normal && normals) {
            seg.setInputNormals(normals);
        }

        // 执行分割
        typename pcl::PointIndices::Ptr inliers_local(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // 计时
        auto start_time = std::chrono::high_resolution_clock::now();
        seg.segment(*inliers_local, *coefficients);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // 检查是否找到足够的内点
        if (inliers_local->indices.empty() || 
            inliers_local->indices.size() < static_cast<size_t>(config.min_inliers)) {
            break;
        }

        // 检查平面法向量是否接近水平（竖直墙面）
        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        float z_component = std::abs(normal[2]);
        
        if (z_component > max_z_component) {
            // 法向量太接近 Z 轴，这是水平面（地面/天花板），不是墙面，跳过
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (size_t i = 0; i < current_indices->indices.size(); ++i) {
                if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(),
                              current_indices->indices[i]) == inliers_local->indices.end()) {
                    new_indices->indices.push_back(current_indices->indices[i]);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 检查是否与已提取的墙面重复
        bool is_duplicate = false;
        for (const auto &wall : walls) {
            // 计算法向量夹角
            Eigen::Vector3f normal2(wall->coefficients->values[0], wall->coefficients->values[1],
                                    wall->coefficients->values[2]);

            float dot    = normal.dot(normal2);
            dot          = std::max(-1.0f, std::min(1.0f, dot));
            double angle = std::acos(dot) * 180.0 / M_PI;

            // 如果法向量几乎平行（夹角小或接近 180 度）且距离相近，则是重复
            if (angle < config.angle_threshold || angle > (180.0 - config.angle_threshold)) {
                double d1 = coefficients->values[3];
                double d2 = wall->coefficients->values[3];

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
                if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(),
                              current_indices->indices[i]) == inliers_local->indices.end()) {
                    new_indices->indices.push_back(current_indices->indices[i]);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 计算尺寸
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, *inliers_local, min_pt, max_pt);

        float width =
            std::sqrt(std::pow(max_pt[0] - min_pt[0], 2) + std::pow(max_pt[1] - min_pt[1], 2));
        float height = max_pt[2] - min_pt[2];
        float depth  = 0.0f;  // 墙面厚度通常很小

        // 保存墙面信息
        auto wall =
            Object::createWall("wall_" + std::to_string(wall_num), inliers_local, coefficients,
                               elapsed, width, height, depth, config.using_normal);

        walls.push_back(wall);

        // 从当前索引中移除这些点
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (size_t i = 0; i < current_indices->indices.size(); ++i) {
            if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(),
                          current_indices->indices[i]) == inliers_local->indices.end()) {
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
