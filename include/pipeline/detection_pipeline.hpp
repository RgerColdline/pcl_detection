// include/pcl_detection/pipeline/detection_pipeline.hpp
#pragma once

#include "core/clustering.hpp"
#include "core/feature_extractor.hpp"
#include "core/filtering.hpp"

#include <pcl/memory.h>
#include <pcl/point_cloud.h>

#include <vector>

namespace pcl_detection
{
    namespace pipeline
    {
        constexpr float k_default_cluster_tolerance = 0.10F;
        constexpr int k_default_min_size            = 100;
        constexpr int k_default_max_size            = 25000;

        template <typename PointT> class DetectionPipeline
        {
          public:
            using PointCloud         = pcl::PointCloud<PointT>;
            using PointCloudPtr      = typename PointCloud::Ptr;
            using PointCloudConstPtr = typename PointCloud::ConstPtr;
            using Obstacle           = typename core::ShapeClassifier<PointT>::ShapeType;

            explicit DetectionPipeline(
                float voxel_leaf_size                    = core::k_def_leaf_size,
                core::ClusterTolerance cluster_tolerance = {k_default_cluster_tolerance},
                core::MinClusterSize min_size            = {k_default_min_size},
                core::MaxClusterSize max_size            = {k_default_max_size})
                : filter_(voxel_leaf_size), clustering_(cluster_tolerance, min_size, max_size),
                  classifier_() {}

            [[nodiscard]] std::vector<Obstacle> process(const PointCloudConstPtr &input) {
                // 1. 滤波（重用输出缓冲区）
                filter_.filter(input, filtered_cloud_);

                // 2. 聚类
                clusters_ = clustering_.cluster(filtered_cloud_);

                // 3. 特征提取
                std::vector<Obstacle> obstacles;
                obstacles.reserve(clusters_.size());
                for (const auto &cluster : clusters_) {
                    obstacles.push_back(classifier_.classify(cluster));
                }
                return obstacles;
            }

            // ✅ 新增：获取最新聚类结果（用于可视化）
            [[nodiscard]] const std::vector<PointCloudPtr> &get_clusters() const noexcept {
                return clusters_;
            }

          private:
            // 关键：所有组件作为成员变量持有（避免每帧重建）
            core::VoxelGridFilter<PointT> filter_{};
            core::EuclideanClustering<PointT> clustering_{};
            core::ShapeClassifier<PointT> classifier_{};

            // 重用缓冲区（性能关键）
            PointCloudPtr filtered_cloud_ = pcl::make_shared<PointCloud>();
            std::vector<PointCloudPtr> clusters_{};  // 存储聚类结果
        };

    }  // namespace pipeline
}  // namespace pcl_detection