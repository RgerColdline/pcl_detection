// src/nodes/detector_node.cpp
#include "adapters/livox_converter.hpp"
#include "adapters/marker_converter.hpp"
#include "core/feature_extractor.hpp"
#include "pipeline/detection_pipeline.hpp"
#include "utils/logger.h"

#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/impl/point_types.hpp>

#include <cstddef>
#include <exception>
#include <vector>

namespace pcl_detection
{
    namespace nodes
    {

        class DetectorNode
        {
          public:
            explicit DetectorNode(ros::NodeHandle nh)  // NOLINT(readability-function-size)
                : nh_(std::move(nh)),
                  pipeline_(nh_.param("voxel_leaf_size", 0.05f),
                            core::ClusterTolerance{nh_.param("cluster_tolerance", 0.10f)},
                            core::MinClusterSize{nh_.param("min_cluster_size", 100)},
                            core::MaxClusterSize{nh_.param("max_cluster_size", 25000)}),
                  livox_sub_(nh_.subscribe("livox/lidar", 5, &DetectorNode::livox_callback, this)),
                  pose_sub_(nh_.subscribe("mavros/local_position/odom", 5,
                                          &DetectorNode::pose_callback, this)),
                  marker_pub_(
                      nh_.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 5)),
                  debug_enabled_(nh_.param("debug_enabled", false)) {
                log_initialization();
            }

          private:
            // ========== ROS 接口 ==========
            ros::NodeHandle nh_;
            ros::Subscriber livox_sub_;
            ros::Subscriber pose_sub_;
            ros::Publisher marker_pub_;

            // ========== 核心组件（成员持有，避免每帧重建）==========
            pipeline::DetectionPipeline<pcl::PointXYZ> pipeline_{};  // NOLINT(hicpp-member-init)

            // ========== 状态管理（应用层唯一职责）==========
            nav_msgs::Odometry latest_pose_{};  // NOLINT(hicpp-member-init)
            bool pose_received_ = false;        // NOLINT(hicpp-member-init)
            bool debug_enabled_ = false;        // NOLINT(hicpp-member-init)


            // ========== 日志输出 ==========
            void log_initialization() const {  // NOLINT(readability-function-size)
                if (!debug_enabled_) {
                    return;
                }

                // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                ROS_INFO("DetectorNode initialized with parameters:");
                ROS_INFO("  voxel_leaf_size: %.3f", nh_.param("voxel_leaf_size", 0.05f));
                ROS_INFO("  cluster_tolerance: %.3f", nh_.param("cluster_tolerance", 0.10f));
                ROS_INFO("  min_cluster_size: %d", nh_.param("min_cluster_size", 100));
                ROS_INFO("  max_cluster_size: %d", nh_.param("max_cluster_size", 25000));
                // NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
            }

            // ========== 回调函数 ==========
            void pose_callback(const nav_msgs::Odometry::ConstPtr &msg) {
                // 1. 更新位姿缓存（应用层核心职责）
                latest_pose_   = *msg;
                pose_received_ = true;

                if (debug_enabled_) {
                    // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                    ROS_DEBUG("Received pose: (%.2f, %.2f, %.2f)",
                              latest_pose_.pose.pose.position.x, latest_pose_.pose.pose.position.y,
                              latest_pose_.pose.pose.position.z);
                    // NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                }
            }

            void livox_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
                // 2. 安全检查：位姿未到达则跳过（避免使用无效位姿）
                if (!validate_pose()) {
                    return;
                }

                // 3. 适配层：Livox → PCL 点云（世界坐标系）
                auto pcl_cloud = convert_to_world(msg);
                if (pcl_cloud->empty()) {
                    // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                    ROS_WARN_THROTTLE(1.0, "Converted point cloud is empty, skipping");
                    // NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                    return;
                }

                if (debug_enabled_) {
                    // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                    ROS_INFO("Processing %zu points from Livox", pcl_cloud->size());
                    // NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                }

                // 4. 核心层：障碍物检测（算法逻辑）
                auto obstacles = pipeline_.process(pcl_cloud);

                if (debug_enabled_) {
                    log_detected_obstacles(obstacles);
                }

                // 5. 适配层：障碍物 → RViz 标记
                publish_markers(obstacles);
            }

            // ========== 辅助函数（降低认知复杂度）==========
            [[nodiscard]] bool validate_pose() const {
                if (!pose_received_) {
                    // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                    ROS_WARN_THROTTLE(1.0, "No pose received yet, skipping Livox message");
                    // NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                    return false;
                }
                return true;
            }

            [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr
            convert_to_world(const livox_ros_driver::CustomMsg::ConstPtr &msg) const {
                return adapters::LivoxConverter::convert_to_world(
                    msg,
                    adapters::DronePose{
                        x_=latest_pose_.pose.pose.position.x, y_=latest_pose_.pose.pose.position.y,
                        yaw_=tf2::getYaw(latest_pose_.pose.pose.orientation)  // 从四元数提取偏航角
                    },
                    nh_.param("height_threshold", 1.5f));
            }

            // NOLINTNEXTLINE(readability-function-size)
            static void log_detected_obstacles(const std::vector<core::ShapeType> &obstacles) {
                // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
                ROS_INFO("Detected %zu obstacles", obstacles.size());
                for (size_t i = 0; i < obstacles.size(); ++i) {
                    ROS_INFO("  Obstacle %zu: %s", i, core::to_string(obstacles[i]));
                }
                // NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
            }

            void publish_markers(const std::vector<core::ShapeType> &obstacles) {
                auto markers = adapters::MarkerConverter::to_markers(
                    obstacles, pipeline_.get_clusters(), {"map", "obstacles"});

                // 6. 发布结果
                marker_pub_.publish(markers);
            }
        };

    }  // namespace nodes
}  // namespace pcl_detection

// ========== 节点入口 ==========
int main(int argc, char **argv) {
    ros::init(argc, argv, "detector_node");
    ros::NodeHandle nh("~");
    // 私有命名空间（参数从 ~/param 读取）

    try {
        pcl_detection::nodes::DetectorNode node(nh);  // NOLINT(misc-const-correctness)
        ros::spin();
        return 0;
    }
    catch (const std::exception &e) {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
        ROS_FATAL("DetectorNode terminated with exception: %s", e.what());
        // NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-do-while)
        return 1;
    }
}