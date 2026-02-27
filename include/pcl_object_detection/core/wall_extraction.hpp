#pragma once

#include "config.hpp"
#include "object_base.hpp"
#include "extractors_base.hpp"
#include "timer.hpp"

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

// 提取多个垂直墙面（支持使用或不使用法向量）
template <typename PointT>
std::vector<Object::Ptr> extractWalls(typename pcl::PointCloud<PointT>::Ptr cloud,
                                      typename pcl::PointCloud<pcl::Normal>::Ptr normals,
                                      const WallConfig &config,
                                      const std::set<int>& excluded_indices = {}) {

    std::vector<Object::Ptr> walls;

    if (cloud->empty()) {
        return walls;
    }

    bool use_normals = config.using_normal && normals;
    int max_iterations = use_normals ? 200 : 500;  // 不使用法向量时增加迭代次数

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
    int max_walls = 4;

    // 角度阈值转换为弧度（用于判断法向量是否接近水平）
    float max_z_component = std::sin(config.angle_threshold * M_PI / 180.0f);

    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers) && wall_num <= max_walls) {
        Timer timer;  // 开始计时当前墙面的提取

        typename pcl::PointIndices::Ptr inliers_local(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        if (use_normals) {
            // 使用法向量的平面拟合
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(config.distance_threshold);
            seg.setMaxIterations(max_iterations);
            seg.setIndices(current_indices);
            seg.setInputCloud(cloud);
            seg.setInputNormals(normals);
            seg.segment(*inliers_local, *coefficients);
        } else {
            // 不使用法向量的平面拟合
            pcl::SACSegmentation<PointT> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(config.distance_threshold);
            seg.setMaxIterations(max_iterations);
            seg.setIndices(current_indices);
            seg.setInputCloud(cloud);
            seg.segment(*inliers_local, *coefficients);
        }

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
            std::set<int> inlier_set(inliers_local->indices.begin(), inliers_local->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (inlier_set.find(idx) == inlier_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 二次检查：确保内点数量足够（防止 RANSAC 误判）
        if (inliers_local->indices.size() < static_cast<size_t>(config.min_inliers * 1.5f)) {
            // 内点数量刚过阈值，质量可能不好，跳过
            std::set<int> inlier_set(inliers_local->indices.begin(), inliers_local->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (inlier_set.find(idx) == inlier_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 检查是否与已提取的墙面重复
        bool is_duplicate = false;
        for (const auto &wall : walls) {
            Eigen::Vector3f normal2(wall->coefficients->values[0], wall->coefficients->values[1],
                                    wall->coefficients->values[2]);
            float dot = normal.dot(normal2);
            dot = std::max(-1.0f, std::min(1.0f, dot));
            double angle = std::acos(dot) * 180.0 / M_PI;

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
            std::set<int> inlier_set(inliers_local->indices.begin(), inliers_local->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (inlier_set.find(idx) == inlier_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 计算尺寸
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, *inliers_local, min_pt, max_pt);

        float width = std::sqrt(std::pow(max_pt[0] - min_pt[0], 2) + std::pow(max_pt[1] - min_pt[1], 2));
        float height = max_pt[2] - min_pt[2];
        float depth = 0.0f;

        // 保存墙面信息（记录提取时间）
        auto wall = Object::createWall("wall_" + std::to_string(wall_num), inliers_local, coefficients,
                                       timer.elapsed(), width, height, depth, use_normals);

        walls.push_back(wall);

        // 从当前索引中移除这些点
        std::set<int> inlier_set(inliers_local->indices.begin(), inliers_local->indices.end());
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (int idx : current_indices->indices) {
            if (inlier_set.find(idx) == inlier_set.end()) {
                new_indices->indices.push_back(idx);
            }
        }
        current_indices = new_indices;
        wall_num++;
    }

    return walls;
}

// ============================================================================
// 墙面提取器（封装 extractWalls 函数）
// ============================================================================
template <typename PointT>
class WallExtractor : public IObjectExtractor<PointT> {
    WallConfig config_;
public:
    using PointCloudPtrT = typename pcl::PointCloud<PointT>::Ptr;
    using NormalCloudPtrT = typename pcl::PointCloud<pcl::Normal>::Ptr;
    
    explicit WallExtractor(const WallConfig& config) : config_(config) {}
    
    std::vector<Object::Ptr> extract(
        PointCloudPtrT cloud,
        NormalCloudPtrT normals,
        const std::set<int>& excluded,
        PointCloudPtrT /*temp_cloud1*/ = nullptr,
        PointCloudPtrT /*temp_cloud2*/ = nullptr) override 
    {
        return extractWalls<PointT>(cloud, normals, config_, excluded);
    }
};

}  // namespace core
}  // namespace pcl_object_detection
