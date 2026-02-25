#pragma once

#include "circle_extraction.hpp"
#include "config.hpp"
#include "cylinder_extraction.hpp"
#include "object_base.hpp"
#include "timer.hpp"
#include "wall_extraction.hpp"

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <memory>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

// 对象检测流水线
template <typename PointT> struct ObjectDetectionPipeline
{
    ObjectDetectionConfig config;

    // 原始点云
    typename pcl::PointCloud<PointT>::Ptr input_cloud;
    // 下采样后的点云
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud;
    // 法向量（如果需要）
    typename pcl::PointCloud<pcl::Normal>::Ptr normals;

    // 检测到的所有对象（统一容器）
    std::vector<Object::Ptr> objects;

    // 时间测量结果
    double downsample_time_ = 0.0;
    double total_time_      = 0.0;

    // 下采样点云
    typename pcl::PointCloud<PointT>::Ptr
    downsamplePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                         const std::string &downsample_method, float leaf_size) {

        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

        if (downsample_method == "approx") {
            pcl::ApproximateVoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel.filter(*cloud_filtered);
        }
        else {
            pcl::VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel.filter(*cloud_filtered);
        }

        return cloud_filtered;
    }

    // 估计法向量
    typename pcl::PointCloud<pcl::Normal>::Ptr
    estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, int k_search = 50) {

        typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(tree);
        ne.setKSearch(k_search);
        ne.compute(*normals);

        return normals;
    }

    // 运行整个流水线
    bool run(typename pcl::PointCloud<PointT>::Ptr input) {
        Timer total_timer;

        input_cloud = input;

        // 1. 下采样
        Timer downsample_timer;
        filtered_cloud =
            downsamplePointCloud(input_cloud, config.downsample_config.downsample_method,
                                 config.downsample_config.leaf_size);
        downsample_time_  = downsample_timer.elapsed();

        // 2. 估计法向量（如果需要）
        bool need_normals = config.wall_config.using_normal ||
                            config.cylinder_config.using_normal ||
                            config.circle_config.using_normal;
        if (need_normals) {
            normals = estimateNormals(filtered_cloud);
        }

        // 3. 提取所有对象（使用点云共享排除机制）
        objects.clear();
        std::set<int> used_indices;  // 已使用的点索引
        
        // 各类对象提取时间
        double wall_time = 0.0;
        double cylinder_time = 0.0;
        double circle_time = 0.0;

        // 提取墙面
        if (config.wall_config.enable) {
            std::cout << "\n[Pipeline] ========== 开始提取墙面 ==========" << std::endl;
            auto wall_start = std::chrono::high_resolution_clock::now();
            auto walls = extractWalls<PointT>(filtered_cloud, normals, config.wall_config);
            auto wall_end = std::chrono::high_resolution_clock::now();
            wall_time = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();
            objects.insert(objects.end(), walls.begin(), walls.end());
            // 收集墙面使用的点
            for (const auto& wall : walls) {
                for (int idx : wall->inliers->indices) {
                    used_indices.insert(idx);
                }
            }
            std::cout << "[Pipeline] 墙面提取完成：检测到 " << walls.size()
                      << " 个墙面，使用 " << used_indices.size() << " 点"
                      << ", 耗时：" << wall_time << " ms" << std::endl;
        }

        // 提取圆柱（排除墙面已用点）
        if (config.cylinder_config.enable) {
            std::cout << "\n[Pipeline] ========== 开始提取圆柱 ==========" << std::endl;
            std::cout << "[Pipeline] 排除已用点：" << used_indices.size() << std::endl;
            auto cylinder_start = std::chrono::high_resolution_clock::now();
            auto cylinders =
                extractCylinders<PointT>(filtered_cloud, normals, config.cylinder_config, used_indices);
            auto cylinder_end = std::chrono::high_resolution_clock::now();
            cylinder_time = std::chrono::duration<double, std::milli>(cylinder_end - cylinder_start).count();
            objects.insert(objects.end(), cylinders.begin(), cylinders.end());
            // 收集圆柱使用的点
            for (const auto& cylinder : cylinders) {
                for (int idx : cylinder->inliers->indices) {
                    used_indices.insert(idx);
                }
            }
            std::cout << "[Pipeline] 圆柱提取完成：检测到 " << cylinders.size() << " 个圆柱"
                      << ", 耗时：" << cylinder_time << " ms" << std::endl;
        }

        // 提取圆环（排除墙面和圆柱已用点）
        if (config.circle_config.enable) {
            std::cout << "\n[Pipeline] ========== 开始提取圆环 ==========" << std::endl;
            std::cout << "[Pipeline] 排除已用点：" << used_indices.size() << std::endl;
            auto circle_start = std::chrono::high_resolution_clock::now();
            auto circles = extractCircles<PointT>(filtered_cloud, normals, config.circle_config, used_indices);
            auto circle_end = std::chrono::high_resolution_clock::now();
            circle_time = std::chrono::duration<double, std::milli>(circle_end - circle_start).count();
            objects.insert(objects.end(), circles.begin(), circles.end());
            std::cout << "[Pipeline] 圆环提取完成：检测到 " << circles.size() << " 个圆环"
                      << ", 耗时：" << circle_time << " ms" << std::endl;
        }

        std::cout << "\n[Pipeline] ========== 时间汇总 ==========" << std::endl;
        std::cout << "下采样：" << downsample_time_ << " ms" << std::endl;
        if (config.wall_config.enable) {
            std::cout << "墙面提取：" << wall_time << " ms" << std::endl;
        }
        if (config.cylinder_config.enable) {
            std::cout << "圆柱提取：" << cylinder_time << " ms" << std::endl;
        }
        if (config.circle_config.enable) {
            std::cout << "圆环提取：" << circle_time << " ms" << std::endl;
        }
        total_time_ = total_timer.elapsed();
        std::cout << "总执行时间：" << total_time_ << " ms" << std::endl;
        std::cout << "[Pipeline] ===============================\n" << std::endl;

        return true;
    }

    // 辅助方法：获取特定类型对象
    template <typename T> std::vector<std::shared_ptr<T>> getObjectsByType() const {
        std::vector<std::shared_ptr<T>> result;
        for (const auto &obj : objects) {
            if (auto ptr = std::dynamic_pointer_cast<T>(obj)) {
                result.push_back(ptr);
            }
        }
        return result;
    }
};

}  // namespace core
}  // namespace pcl_object_detection