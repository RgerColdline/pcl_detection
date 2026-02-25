// include/pcl_detection/adapters/marker_converter.hpp
#pragma once
#include "core/feature_extractor.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <pcl/common/centroid.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstddef>
#include <string>

namespace pcl_detection
{
    namespace adapters
    {

        struct Config
        {
            std::string frame_id_  = "map";        ///< 坐标系 (默认 "map")
            std::string marker_ns_ = "obstacles";  ///< 标记命名空间 (默认 "obstacles")
        };
        class MarkerConverter
        {
          public:
            /// ====================== 标记配置结构体（命名参数） ======================
            /// @brief 可视化标记配置参数
            /// @note 使用聚合初始化避免参数顺序混淆
            /// @example
            ///   auto markers = MarkerConverter::to_markers(
            ///       obstacles, clusters,
            ///       {.frame_id = "odom", .marker_ns = "cones"}
            ///   );

            /// ====================== 标记尺寸常量 ======================
            static constexpr float k_ring_radius     = 0.15F;
            static constexpr float k_ring_height     = 1.2F;
            static constexpr float k_cylinder_radius = 0.3F;
            static constexpr float k_cylinder_height = 2.0F;
            static constexpr float k_unknown_radius  = 0.2F;
            static constexpr float k_lift_height     = 0.5F;
            static constexpr float k_ring_alpha      = 0.8F;
            static constexpr float k_cylinder_alpha  = 0.7F;
            static constexpr float k_unknown_alpha   = 0.9F;

            /**
             * @brief 生成障碍物可视化标记数组
             *
             * @param obstacles 障碍物形状类型列表
             * @param clusters 聚类点云列表
             * @param config 标记配置（命名参数，避免顺序混淆）
             * @return visualization_msgs::MarkerArray 可视化标记
             *
             * @note 使用命名参数模式消除参数交换风险：
             *       to_markers(obstacles, clusters, {.frame_id="odom", .marker_ns="cones"})
             */
            [[nodiscard]] static visualization_msgs::MarkerArray
            to_markers(const std::vector<core::ShapeType> &obstacles,
                       const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
                       const Config &config = Config())  // ← 关键：单一配置参数
            {
                visualization_msgs::MarkerArray markers;
                markers.markers.reserve(obstacles.size() + 1);

                // 1. 添加清除标记
                {
                    visualization_msgs::Marker clear_marker;
                    clear_marker.action = visualization_msgs::Marker::DELETEALL;
                    clear_marker.ns     = config.marker_ns_;
                    markers.markers.push_back(std::move(clear_marker));
                }

                // 2. 为每个障碍物生成标记
                for (std::size_t i = 0; i < obstacles.size() && i < clusters.size(); ++i) {
                    const auto &cluster = clusters[i];
                    if (cluster->empty()) {
                        continue;
                    }

                    visualization_msgs::Marker marker;
                    marker.header.frame_id   = config.frame_id_;
                    marker.header.stamp      = ros::Time::now();
                    marker.ns                = config.marker_ns_;
                    marker.id                = static_cast<int>(i);
                    marker.type              = visualization_msgs::Marker::CYLINDER;
                    marker.action            = visualization_msgs::Marker::ADD;

                    // 计算质心
                    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
                    pcl::compute3DCentroid(*cluster, centroid);

                    marker.pose.position.x    = centroid.x();
                    marker.pose.position.y    = centroid.y();
                    marker.pose.position.z    = centroid.z() + k_lift_height;
                    marker.pose.orientation.w = 1.0;

                    // 根据形状设置尺寸/颜色
                    switch (obstacles[i]) {
                    case core::ShapeType::RING:
                        marker.scale.x = k_ring_radius;
                        marker.scale.y = k_ring_radius;
                        marker.scale.z = k_ring_height;
                        marker.color.r = 0.0F;
                        marker.color.g = 1.0F;
                        marker.color.b = 0.0F;
                        marker.color.a = k_ring_alpha;
                        break;

                    case core::ShapeType::CYLINDER:
                        marker.scale.x = k_cylinder_radius;
                        marker.scale.y = k_cylinder_radius;
                        marker.scale.z = k_cylinder_height;
                        marker.color.r = 1.0F;
                        marker.color.g = 0.0F;
                        marker.color.b = 0.0F;
                        marker.color.a = k_cylinder_alpha;
                        break;

                    case core::ShapeType::UNKNOWN:
                    default:
                        marker.type    = visualization_msgs::Marker::SPHERE;
                        marker.scale.x = k_unknown_radius;
                        marker.scale.y = k_unknown_radius;
                        marker.scale.z = k_unknown_radius;
                        marker.color.r = 1.0F;
                        marker.color.g = 1.0F;
                        marker.color.b = 0.0F;
                        marker.color.a = k_unknown_alpha;
                        break;
                    }

                    markers.markers.push_back(std::move(marker));
                }

                return markers;
            }
        };

    }  // namespace adapters
}  // namespace pcl_detection