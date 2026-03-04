#pragma once

#include "obstacle_pipeline_config.hpp"
#include "ground_extraction.hpp"
#include "wall_extraction.hpp"
#include "cluster_extraction.hpp"
#include "rectangle_from_cluster.hpp"
#include "normal_estimation.hpp"
#include "obb_calculator.hpp"
#include "object_factory.hpp"
#include "logger_limiter.hpp"
#include "timer.hpp"

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <memory>
#include <vector>
#include <set>
#include <algorithm>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 障碍物检测流水线（重构版）
 *
 * 流程：
 * 1. 下采样
 * 2. 法向量估计（多线程，可选）
 * 3. 地面分割（RANSAC + 逻辑过滤）
 * 4. 墙壁分割（RANSAC + 逻辑过滤）
 * 5. 剩余点云聚类
 * 6. 对每个聚类进行方环提取（PCA 投影 + 边界 + 角点）
 * 7. 剩余聚类作为障碍物
 * 8. 计算 OBB/膨胀
 */
template <typename PointT>
class ObstaclePipeline
{
public:
    using PointCloudPtrT = typename pcl::PointCloud<PointT>::Ptr;
    using NormalCloudPtrT = typename pcl::PointCloud<pcl::Normal>::Ptr;

    ObstaclePipeline() : config_() {}

    explicit ObstaclePipeline(const ObstaclePipelineConfig &config) : config_(config) {}

    // 检测到的所有对象
    std::vector<Object::Ptr> objects;
    
    // 聚类结果（用于可视化）
    std::vector<typename pcl::PointIndices::Ptr> clusters;

    // 时间测量
    double downsample_time_ = 0.0;
    double normal_time_ = 0.0;
    double ground_time_ = 0.0;
    double wall_time_ = 0.0;
    double cluster_time_ = 0.0;
    double rectangle_time_ = 0.0;
    double obb_time_ = 0.0;
    double total_time_ = 0.0;

    // 原始点云和下采样点云
    PointCloudPtrT input_cloud;
    PointCloudPtrT filtered_cloud;
    NormalCloudPtrT normals;

    /**
     * @brief 运行整个流水线
     * @param input 输入点云
     * @return 是否成功
     */
    bool run(PointCloudPtrT input) {
        Timer total_timer;

        input_cloud = input;
        objects.clear();

        // ========================================================================
        // 步骤 1: 下采样
        // ========================================================================
        Timer downsample_timer;
        filtered_cloud = downsamplePointCloud(input_cloud);
        downsample_time_ = downsample_timer.elapsed();

        ROS_DEBUG("[ObstaclePipeline] 下采样完成：%d -> %d 点",
                  static_cast<int>(input_cloud->size()),
                  static_cast<int>(filtered_cloud->size()));

        // ========================================================================
        // 步骤 2: 法向量估计（多线程，可选）
        // ========================================================================
        Timer normal_timer;
        normals = estimateNormals<PointT>(filtered_cloud, config_.normal_config);
        normal_time_ = normal_timer.elapsed();

        // ========================================================================
        // 步骤 3: 地面分割（RANSAC + 逻辑过滤）
        // ========================================================================
        Timer ground_timer;
        typename pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
        typename pcl::PointIndices::Ptr non_ground_indices(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr ground_coefficients(new pcl::ModelCoefficients);

        bool ground_success = false;
        if (config_.ground_config.enable) {
            ground_success = extractGround<PointT>(filtered_cloud, config_.ground_config,
                                           ground_indices, non_ground_indices,
                                           ground_coefficients);
        } else {
            // 不进行地面分割，所有点都作为非地面点
            non_ground_indices->indices.resize(filtered_cloud->size());
            for (size_t i = 0; i < filtered_cloud->size(); ++i) {
                non_ground_indices->indices[i] = i;
            }
        }
        ground_time_ = ground_timer.elapsed();

        // 输出地面分割日志（每 N 帧输出一次）
        if (LOG_SHOULD()) {
            if (ground_success) {
                ROS_INFO("[ObstaclePipeline] 地面分割完成：地面 %lu 点，非地面 %lu 点",
                         ground_indices->indices.size(),
                         non_ground_indices->indices.size());
            } else {
                ROS_INFO("[ObstaclePipeline] 地面分割失败，使用全部 %lu 点作为非地面点",
                         filtered_cloud->size());
            }
        }

        // ========================================================================
        // 步骤 4: 墙壁分割（RANSAC + 逻辑过滤，不使用法向量）
        // ========================================================================
        Timer wall_timer;
        std::set<int> used_indices;  // 已使用的点索引（相对于 filtered_cloud）

        // 首先将地面点标记为已使用
        for (int idx : ground_indices->indices) {
            used_indices.insert(idx);
        }

        if (config_.wall_config.enable) {
            // 调用 extractWalls 重载版本（使用 WallExtractionConfig）
            auto wall_objects = extractWalls<PointT>(
                filtered_cloud, nullptr, config_.wall_config, used_indices);

            for (auto &obj : wall_objects) {
                objects.push_back(obj);
                // 标记已用点
                for (int idx : obj->inliers->indices) {
                    used_indices.insert(idx);
                }
            }

            // 输出墙体检测日志（每 N 帧输出一次）
            if (LOG_SHOULD()) {
                ROS_INFO("[ObstaclePipeline] 墙体检测完成：检测到 %lu 个墙体，used_indices=%lu",
                         wall_objects.size(), used_indices.size());
            }
        } else {
            ROS_DEBUG("[ObstaclePipeline] 墙体检测被跳过");
        }
        wall_time_ = wall_timer.elapsed();

        // ========================================================================
        // 步骤 5: 剩余点云聚类
        // ========================================================================
        Timer cluster_timer;
        clusters.clear();  // 清空上一帧的聚类结果
        std::vector<typename pcl::PointIndices::Ptr> clusters;

        if (config_.cluster_config.enable) {
            clusterObstacles<PointT>(filtered_cloud, config_.cluster_config,
                                     used_indices, clusters);
        }
        cluster_time_ = cluster_timer.elapsed();

        ROS_DEBUG("[ObstaclePipeline] 聚类完成：%lu 个聚类", clusters.size());
        
        // 保存聚类结果（用于可视化）
        this->clusters = clusters;

        // ========================================================================
        // 步骤 6: 方环提取（基于聚类，PCA 投影 + 边界 + 角点）
        // ========================================================================
        Timer rectangle_timer;
        std::set<int> rectangle_used_indices;  // 方环已使用的聚类索引

        if (config_.rectangle_config.enable && !clusters.empty()) {
            RectangleFromClusterExtractor<PointT> rect_extractor(config_.rectangle_config);
            
            // 注意：这里传入的是原始 clusters，extractor 内部会处理
            auto rect_objects = rect_extractor.extract(
                clusters, filtered_cloud, rectangle_used_indices);

            for (auto &obj : rect_objects) {
                objects.push_back(obj);
            }

            ROS_DEBUG("[ObstaclePipeline] 方环检测完成：%lu 个方环", rect_objects.size());
        }
        rectangle_time_ = rectangle_timer.elapsed();

        // ========================================================================
        // 步骤 7: 剩余聚类作为障碍物，计算 OBB 并膨胀成圆柱
        // ========================================================================
        Timer obb_timer;
        int obstacle_id = 1;

        for (size_t i = 0; i < clusters.size(); ++i) {
            // 跳过已用于方环的聚类
            bool is_rectangle = false;
            for (int idx : clusters[i]->indices) {
                if (rectangle_used_indices.find(idx) != rectangle_used_indices.end()) {
                    is_rectangle = true;
                    break;
                }
            }
            if (is_rectangle) continue;

            // 创建障碍物对象
            auto obstacle = createObstacleFromCluster<PointT>(
                filtered_cloud, clusters[i], obstacle_id, 0.0);

            // 计算 OBB 并转换为圆柱表示
            if (updateObstacleAsCylinder<PointT>(obstacle, filtered_cloud, config_.obb_config)) {
                obstacle->name = "obstacle_" + std::to_string(obstacle_id++);
                objects.push_back(obstacle);
            }
        }
        obb_time_ = obb_timer.elapsed();

        // ========================================================================
        // 完成
        // ========================================================================
        total_time_ = total_timer.elapsed();

        // 输出检测结果（每 N 帧输出一次）
        if (LOG_SHOULD()) {
            ROS_INFO("[ObstaclePipeline] 检测完成：墙体=%d, 方环=%d, 障碍物=%d, 总计=%lu",
                     countObjectsByType("Wall"),
                     countObjectsByType("Rectangle"),
                     countObjectsByType("obstacle"),
                     objects.size());
        }

        return true;
    }

    /**
     * @brief 输出时间日志（每 N 帧输出一次）
     */
    void printTimingInfo() {
        if (!config_.timing_config.enable) return;
        if (!LOG_SHOULD()) return;

        ROS_INFO("=== 障碍物 Pipeline 时间统计 ===");
        if (config_.timing_config.downsample) {
            ROS_INFO("  下采样：%.2f ms", downsample_time_);
        }
        if (config_.timing_config.normal && config_.normal_config.enable) {
            ROS_INFO("  法向量估计：%.2f ms (threads=%d)", normal_time_, config_.normal_config.num_threads);
        }
        if (config_.timing_config.ground && config_.ground_config.enable) {
            ROS_INFO("  地面分割：%.2f ms", ground_time_);
        }
        if (config_.timing_config.wall && config_.wall_config.enable) {
            ROS_INFO("  墙体检测：%.2f ms", wall_time_);
        }
        if (config_.timing_config.cluster && config_.cluster_config.enable) {
            ROS_INFO("  聚类：%.2f ms", cluster_time_);
        }
        if (config_.timing_config.rectangle && config_.rectangle_config.enable) {
            ROS_INFO("  方环检测：%.2f ms", rectangle_time_);
        }
        if (config_.timing_config.obb) {
            ROS_INFO("  OBB 计算：%.2f ms", obb_time_);
        }
        if (config_.timing_config.total) {
            ROS_INFO("  总耗时：%.2f ms", total_time_);
        }
    }

    /**
     * @brief 获取配置
     */
    const ObstaclePipelineConfig& getConfig() const { return config_; }

    /**
     * @brief 设置配置
     */
    void setConfig(const ObstaclePipelineConfig &config) { config_ = config; }

private:
    ObstaclePipelineConfig config_;

    /**
     * @brief 下采样点云
     */
    PointCloudPtrT downsamplePointCloud(PointCloudPtrT cloud) {
        PointCloudPtrT cloud_filtered(new pcl::PointCloud<PointT>());

        if (config_.downsample_config.approx) {
            pcl::ApproximateVoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(config_.downsample_config.leaf_size,
                              config_.downsample_config.leaf_size,
                              config_.downsample_config.leaf_size);
            voxel.filter(*cloud_filtered);
            // 注意：ApproximateVoxelGrid 有内部缓存，但局部对象销毁时会自动清理
        } else {
            pcl::VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(config_.downsample_config.leaf_size,
                              config_.downsample_config.leaf_size,
                              config_.downsample_config.leaf_size);
            voxel.filter(*cloud_filtered);
            // VoxelGrid 没有持久化缓存，每次都是独立的
        }

        return cloud_filtered;
    }

    /**
     * @brief 按类型统计对象数量
     */
    int countObjectsByType(const std::string &type) {
        int count = 0;
        for (const auto &obj : objects) {
            if (obj->getType() == type ||
                (type == "obstacle" && obj->name.find("obstacle_") != std::string::npos)) {
                count++;
            }
        }
        return count;
    }
};

}  // namespace core
}  // namespace pcl_object_detection
