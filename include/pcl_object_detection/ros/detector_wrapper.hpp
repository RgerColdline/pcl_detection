#pragma once

#include "pcl_object_detection/core/pipeline.hpp"
#include "pcl_object_detection/core/obstacle_pipeline.hpp"
#include "pcl_object_detection/core/obstacle_pipeline_config.hpp"
#include "pcl_object_detection/core/timer.hpp"
#include "pcl_object_detection/core/logger_limiter.hpp"
#include "pcl_object_detection/core/object_factory.hpp"
#include "pcl_object_detection/core/centroid_utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_detection/DetectedObject.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <ros/ros.h>

#include <vector>
#include <sstream>

namespace pcl_object_detection
{
namespace ros
{

/**
 * @brief 对象检测器的 ROS 包装器（支持原 Pipeline 和新障碍物 Pipeline）
 *
 * 从 ROS 参数服务器加载配置（参数应在 launch 文件中使用 rosparam command="load" 加载）
 *
 * 使用方式：
 *   - use_obstacle_pipeline=false: 使用原 Pipeline（墙体 + 圆柱 + 圆环 + 矩形）
 *   - use_obstacle_pipeline=true: 使用新 Pipeline（地面分割 + 墙体 + 方环 + 聚类障碍物）
 */
template <typename PointT = pcl::PointXYZI> class ObjectDetectorWrapper
{
  public:
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    // 初始化检测器（从 ROS 参数服务器加载配置）
    ObjectDetectorWrapper(::ros::NodeHandle &nh, ::ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh), use_obstacle_pipeline_(false), has_pose_(false) {
        loadConfigFromROS();

        // 初始化日志限流器（每 N 帧输出一次）
        LOG_INIT(log_skip_frames_ + 1);  // +1 因为配置是"每 20 帧输出 1 次"，即 skip=19

        // 初始化点云发布器
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection/pointcloud", 1);
        wall_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection/cloud/wall", 1);
        rectangle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection/cloud/rectangle", 1);
        obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection/cloud/obstacle", 1);
        cluster_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_detection/cloud/cluster", 10);

        // 订阅无人机位姿（如果启用坐标转换）
        if (use_obstacle_pipeline_ && obstacle_config_.transform_config.enable) {
            pose_sub_ = nh_.subscribe(obstacle_config_.transform_config.pose_topic, 10,
                                      &ObjectDetectorWrapper::poseCallback, this);
            ROS_INFO("订阅无人机位姿话题：%s", obstacle_config_.transform_config.pose_topic.c_str());
        }

        if (use_obstacle_pipeline_) {
            // 使用新 Pipeline
            obstacle_pipeline_.setConfig(obstacle_config_);
            ROS_INFO("使用新 Pipeline：地面分割 + 墙体 + 方环 + 聚类障碍物");
        } else {
            // 使用原 Pipeline
            pipeline_.config = config_;
            ROS_INFO("使用原 Pipeline：墙体 + 圆柱 + 圆环 + 矩形");
        }

        ROS_INFO("ObjectDetectorWrapper 初始化完成（从 ROS 参数服务器加载配置）");
    }

    /**
     * @brief 无人机位姿回调
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 提取位置
        Eigen::Vector3f position;
        position.x() = msg->pose.position.x;
        position.y() = msg->pose.position.y;
        position.z() = msg->pose.position.z;

        // 提取四元数
        Eigen::Quaternionf orientation;
        orientation.x() = msg->pose.orientation.x;
        orientation.y() = msg->pose.orientation.y;
        orientation.z() = msg->pose.orientation.z;
        orientation.w() = msg->pose.orientation.w;

        // 设置到 Pipeline
        obstacle_pipeline_.setDronePose(position, orientation);
        has_pose_ = true;
    }

    // 检测步骤：输入点云，输出检测结果
    bool process(const PointCloudPtrT &cloud, pcl_detection::ObjectDetectionResult &result) {
        // 增加帧计数（用于日志限流）
        LOG_FRAME();
        
        // 检查全局处理开关（可被其他节点通过 rosparam set 动态修改）
        bool enable_processing = true;
        nh_.getParam("enable_pcl_processing", enable_processing);

        if (!enable_processing) {
            // 跳过处理，返回空结果
            result.success        = false;
            result.status_message = "PCL processing is disabled (enable_pcl_processing=false)";
            result.header.stamp   = ::ros::Time::now();
            result.header.frame_id = "laser_livox";
            result.objects.clear();
            result.wall_count     = 0;
            result.cylinder_count = 0;
            result.circle_count   = 0;
            result.total_time     = 0.0;
            return false;
        }

        core::Timer timer;  // 开始计时处理时间

        cloud_ = cloud;

        // 处理点云（根据配置选择 Pipeline）
        if (use_obstacle_pipeline_) {
            if (!obstacle_pipeline_.run(cloud)) {
                ROS_ERROR("障碍物 Pipeline 处理失败");
                result.success        = false;
                result.status_message = "Obstacle pipeline execution failed";
                return false;
            }
        } else {
            if (!pipeline_.run(cloud)) {
                ROS_ERROR("原 Pipeline 处理失败");
                result.success        = false;
                result.status_message = "Pipeline execution failed";
                return false;
            }
        }

        // 填充结果消息
        fillResultMessage(cloud, result);

        // 发布下采样后的点云（用于 RViz 可视化）
        if (publish_cloud_) {
            publishCloud(cloud);
        }
        
        // 发布按物体分类的点云
        if (publish_cloud_) {
            publishObjectClouds(cloud);
        }
        
        // 发布聚类后的簇点云（新 Pipeline 专用）
        if (publish_cloud_ && use_obstacle_pipeline_) {
            publishClusterClouds(cloud);
        }

        // 统计处理耗时
        result.total_time = timer.elapsed();

        // 输出时间日志（每 N 帧输出一次）
        printTimingInfo();

        // 输出物体详细信息（每 N 帧输出一次）
        printObjectDetails();

        return true;
    }

  private:
    // 帧计数器
    int frame_count_ = 0;

    // 日志频率参数
    int log_skip_frames_ = 19;
    double log_interval_sec_ = 2.0;

    // Pipeline 选择标志
    bool use_obstacle_pipeline_;

  private:
    // 打印时间日志
    void printTimingInfo() {
        if (use_obstacle_pipeline_) {
            obstacle_pipeline_.printTimingInfo();
        } else {
            pipeline_.printTimingInfo();
        }
    }

    // 打印物体详细信息（每 N 帧输出一次）
    void printObjectDetails() {
        // 每 N 帧输出一次
        if (!LOG_SHOULD()) return;

        const auto& objs = use_obstacle_pipeline_ ? obstacle_pipeline_.objects : pipeline_.objects;
        const auto& filtered_cloud = use_obstacle_pipeline_ ? obstacle_pipeline_.filtered_cloud : pipeline_.filtered_cloud;

        // 输出物体统计信息
        int wall_count = 0, cylinder_count = 0, circle_count = 0, rect_count = 0, obstacle_count = 0;
        
        for (const auto& obj : objs) {
            if (obj->getType() == "Wall") wall_count++;
            else if (obj->getType() == "Cylinder") cylinder_count++;
            else if (obj->getType() == "Circle") circle_count++;
            else if (obj->getType() == "Rectangle") rect_count++;
            else if (obj->getType() == "Obstacle") obstacle_count++;
        }
        
        ROS_INFO("[Objects] 检测物体统计：墙体=%d, 圆柱=%d, 圆环=%d, 矩形=%d, 障碍物=%d, 总计=%lu",
                 wall_count, cylinder_count, circle_count, rect_count, obstacle_count, objs.size());

        // 输出所有墙体的详细信息
        if (wall_count > 0) {
            int printed = 0;
            for (const auto& obj : objs) {
                if (obj->getType() == "Wall") {
                    std::ostringstream oss;
                    oss << "\n=== " << obj->name << " (" << obj->getType() << ") ===\n";
                    oss << "  点数：" << obj->getPointCount() << "\n";
                    oss << "  尺寸：宽=" << obj->width << "m, 高=" << obj->height << "m, 深=" << obj->depth << "m\n";
                    
                    if (obj->coefficients && obj->coefficients->values.size() >= 4) {
                        oss << "  法向量：(" << obj->coefficients->values[0] << ", "
                            << obj->coefficients->values[1] << ", "
                            << obj->coefficients->values[2] << ")\n";
                        oss << "  平面距离：" << obj->coefficients->values[3] << "\n";
                    }
                    
                    // 计算并输出墙体中心位置
                    if (obj->inliers && !obj->inliers->indices.empty()) {
                        Eigen::Vector4f centroid;
                        pcl::compute3DCentroid(*filtered_cloud, *obj->inliers, centroid);
                        oss << "  中心：(" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")\n";
                    }
                    
                    ROS_INFO_STREAM(oss.str());
                    printed++;
                    
                    if (printed >= 10) {
                        ROS_INFO("[Objects] ... 还有 %d 个墙体未显示", wall_count - 10);
                        break;
                    }
                }
            }
        }

        // 输出所有障碍物的详细信息
        if (obstacle_count > 0) {
            int printed = 0;
            for (const auto& obj : objs) {
                if (obj->getType() == "Obstacle") {
                    std::ostringstream oss;
                    oss << "\n=== " << obj->name << " (" << obj->getType() << ") ===\n";
                    oss << "  点数：" << obj->getPointCount() << "\n";
                    oss << "  尺寸：宽=" << obj->width << "m, 高=" << obj->height << "m, 深=" << obj->depth << "m\n";

                    if (obj->coefficients && obj->coefficients->values.size() >= 3) {
                        oss << "  中心：(" << obj->coefficients->values[0] << ", "
                            << obj->coefficients->values[1] << ", "
                            << obj->coefficients->values[2] << ")\n";
                    }

                    if (obj->coefficients && obj->coefficients->values.size() >= 7) {
                        oss << "  半径：" << obj->coefficients->values[6] << "m\n";
                        oss << "  高度：" << obj->height << "m\n";
                    }

                    ROS_INFO_STREAM(oss.str());
                    printed++;
                    
                    if (printed >= 20) {
                        ROS_INFO("[Objects] ... 还有 %d 个障碍物未显示", obstacle_count - 20);
                        break;
                    }
                }
            }
        }
    }

  private:
    // 从 ROS 参数服务器加载配置
    void loadConfigFromROS() {
        // 检查是否使用新 Pipeline
        pnh_.param("use_obstacle_pipeline", use_obstacle_pipeline_, false);

        // 日志频率配置
        pnh_.param("log_skip_frames", log_skip_frames_, 19);
        pnh_.param("log_interval_sec", log_interval_sec_, 2.0);

        // ========================================================================
        // 原 Pipeline 配置
        // ========================================================================
        // 下采样配置
        pnh_.param("downsample_config/approx", config_.downsample_config.approx, false);
        pnh_.param("downsample_config/leaf_size", config_.downsample_config.leaf_size, 0.05f);

        // 墙面配置
        pnh_.param("wall_extraction/enable", config_.wall_config.enable, true);
        pnh_.param("wall_extraction/using_normal", config_.wall_config.using_normal, true);
        pnh_.param("wall_extraction/distance_threshold", config_.wall_config.distance_threshold, 0.02f);
        pnh_.param("wall_extraction/min_inliers", config_.wall_config.min_inliers, 200);
        pnh_.param("wall_extraction/angle_threshold", config_.wall_config.angle_threshold, 15.0f);

        std::vector<double> wall_axis;
        if (pnh_.getParam("wall_extraction/axis", wall_axis) && wall_axis.size() == 3) {
            config_.wall_config.axis = {static_cast<float>(wall_axis[0]), static_cast<float>(wall_axis[1]), static_cast<float>(wall_axis[2])};
        } else {
            config_.wall_config.axis = {0.0f, 0.0f, 1.0f};
        }

        // 圆柱配置
        pnh_.param("cylinder_extraction/enable", config_.cylinder_config.enable, true);
        pnh_.param("cylinder_extraction/using_normal", config_.cylinder_config.using_normal, true);
        pnh_.param("cylinder_extraction/distance_threshold", config_.cylinder_config.distance_threshold, 0.02f);
        pnh_.param("cylinder_extraction/min_inliers", config_.cylinder_config.min_inliers, 200);
        pnh_.param("cylinder_extraction/radius_min", config_.cylinder_config.radius_min, 0.05f);
        pnh_.param("cylinder_extraction/radius_max", config_.cylinder_config.radius_max, 0.2f);

        std::vector<double> cylinder_axis;
        if (pnh_.getParam("cylinder_extraction/axis", cylinder_axis) && cylinder_axis.size() == 3) {
            config_.cylinder_config.axis = {static_cast<float>(cylinder_axis[0]), static_cast<float>(cylinder_axis[1]), static_cast<float>(cylinder_axis[2])};
        } else {
            config_.cylinder_config.axis = {0.0f, 0.0f, 1.0f};
        }
        pnh_.param("cylinder_extraction/eps_angle", config_.cylinder_config.eps_angle, 15.0f);

        // 圆环配置
        pnh_.param("circle_extraction/enable", config_.circle_config.enable, true);
        pnh_.param("circle_extraction/using_normal", config_.circle_config.using_normal, true);
        pnh_.param("circle_extraction/distance_threshold", config_.circle_config.distance_threshold, 0.02f);
        pnh_.param("circle_extraction/min_inliers", config_.circle_config.min_inliers, 100);
        pnh_.param("circle_extraction/radius_min", config_.circle_config.radius_min, 0.1f);
        pnh_.param("circle_extraction/radius_max", config_.circle_config.radius_max, 0.7f);

        pnh_.param("circle_extraction/plane_distance_threshold", config_.circle_config.plane_distance_threshold, 0.05f);
        pnh_.param("circle_extraction/plane_angle_threshold", config_.circle_config.plane_angle_threshold, 45.0f);
        pnh_.param("circle_extraction/plane_normal_z_max", config_.circle_config.plane_normal_z_max, 0.7f);
        pnh_.param("circle_extraction/min_coverage_angle", config_.circle_config.min_coverage_angle, 270.0f);
        pnh_.param("circle_extraction/ransac_iterations", config_.circle_config.ransac_iterations, 500);
        pnh_.param("circle_extraction/max_circles", config_.circle_config.max_circles, 10);
        pnh_.param("circle_extraction/normal_tolerance", config_.circle_config.normal_tolerance, 0.5f);

        // 矩形框配置
        pnh_.param("rectangle_extraction/enable", config_.rectangle_config.enable, true);
        pnh_.param("rectangle_extraction/using_normal", config_.rectangle_config.using_normal, true);
        pnh_.param("rectangle_extraction/distance_threshold", config_.rectangle_config.distance_threshold, 0.03f);
        pnh_.param("rectangle_extraction/min_inliers", config_.rectangle_config.min_inliers, 100);
        pnh_.param("rectangle_extraction/length_min", config_.rectangle_config.length_min, 0.5f);
        pnh_.param("rectangle_extraction/length_max", config_.rectangle_config.length_max, 3.0f);
        pnh_.param("rectangle_extraction/width_min", config_.rectangle_config.width_min, 0.3f);
        pnh_.param("rectangle_extraction/width_max", config_.rectangle_config.width_max, 2.0f);
        pnh_.param("rectangle_extraction/plane_distance_threshold", config_.rectangle_config.plane_distance_threshold, 0.05f);
        pnh_.param("rectangle_extraction/plane_angle_threshold", config_.rectangle_config.plane_angle_threshold, 45.0f);
        pnh_.param("rectangle_extraction/plane_normal_z_max", config_.rectangle_config.plane_normal_z_max, 0.7f);
        pnh_.param("rectangle_extraction/coverage_threshold", config_.rectangle_config.coverage_threshold, 0.3f);
        pnh_.param("rectangle_extraction/ransac_iterations", config_.rectangle_config.ransac_iterations, 300);
        pnh_.param("rectangle_extraction/max_rectangles", config_.rectangle_config.max_rectangles, 10);

        // 时间测量配置
        pnh_.param("timing/enable", config_.timing_config.enable, false);
        pnh_.param("timing/downsample", config_.timing_config.downsample, true);
        pnh_.param("timing/walls", config_.timing_config.walls, true);
        pnh_.param("timing/cylinders", config_.timing_config.cylinders, true);
        pnh_.param("timing/circles", config_.timing_config.circles, true);
        pnh_.param("timing/total", config_.timing_config.total, true);

        // 其他配置
        pnh_.param("save_objects", config_.save_objects, false);
        pnh_.param("output_dir", config_.output_dir, std::string("./objects/"));

        // 点云发布配置
        pnh_.param("publish_cloud", publish_cloud_, true);  // 默认发布点云到 RViz
        pnh_.param("min_cloud_points", min_cloud_points_, 10);  // 最小点云数量阈值

        // ========================================================================
        // 新 Pipeline（障碍物 Pipeline）配置
        // ========================================================================
        // 下采样配置
        pnh_.param("obstacle_pipeline/downsample_config/approx", 
                   obstacle_config_.downsample_config.approx, true);
        pnh_.param("obstacle_pipeline/downsample_config/leaf_size", 
                   obstacle_config_.downsample_config.leaf_size, 0.08f);

        // 地面分割配置
        pnh_.param("obstacle_pipeline/ground_config/enable",
                   obstacle_config_.ground_config.enable, true);
        pnh_.param("obstacle_pipeline/ground_config/distance_threshold",
                   obstacle_config_.ground_config.distance_threshold, 0.05f);
        pnh_.param("obstacle_pipeline/ground_config/max_iterations",
                   obstacle_config_.ground_config.max_iterations, 100);
        pnh_.param("obstacle_pipeline/ground_config/normal_z_min",
                   obstacle_config_.ground_config.normal_z_min, 0.8f);
        pnh_.param("obstacle_pipeline/ground_config/normal_z_max",
                   obstacle_config_.ground_config.normal_z_max, 1.0f);
        pnh_.param("obstacle_pipeline/ground_config/min_ground_points",
                   obstacle_config_.ground_config.min_ground_points, 500);

        // 法向量估计配置（新 Pipeline）
        pnh_.param("obstacle_pipeline/normal_config/enable",
                   obstacle_config_.normal_config.enable, true);
        pnh_.param("obstacle_pipeline/normal_config/k_search",
                   obstacle_config_.normal_config.k_search, 50);
        pnh_.param("obstacle_pipeline/normal_config/num_threads",
                   obstacle_config_.normal_config.num_threads, 4);

        // 墙体配置（新 Pipeline，复用原配置结构）
        pnh_.param("obstacle_pipeline/wall_config/enable", 
                   obstacle_config_.wall_config.enable, true);
        pnh_.param("obstacle_pipeline/wall_config/using_normal", 
                   obstacle_config_.wall_config.using_normal, true);
        pnh_.param("obstacle_pipeline/wall_config/distance_threshold", 
                   obstacle_config_.wall_config.distance_threshold, 0.04f);
        pnh_.param("obstacle_pipeline/wall_config/min_inliers", 
                   obstacle_config_.wall_config.min_inliers, 1500);
        pnh_.param("obstacle_pipeline/wall_config/angle_threshold", 
                   obstacle_config_.wall_config.angle_threshold, 15.0f);

        std::vector<double> obs_wall_axis;
        if (pnh_.getParam("obstacle_pipeline/wall_config/axis", obs_wall_axis) && obs_wall_axis.size() == 3) {
            obstacle_config_.wall_config.axis = {static_cast<float>(obs_wall_axis[0]), 
                                                  static_cast<float>(obs_wall_axis[1]), 
                                                  static_cast<float>(obs_wall_axis[2])};
        } else {
            obstacle_config_.wall_config.axis = {0.0f, 0.0f, 1.0f};
        }

        // 矩形框配置（新 Pipeline，复用原配置结构）
        pnh_.param("obstacle_pipeline/rectangle_config/enable", 
                   obstacle_config_.rectangle_config.enable, true);
        pnh_.param("obstacle_pipeline/rectangle_config/using_normal", 
                   obstacle_config_.rectangle_config.using_normal, false);
        pnh_.param("obstacle_pipeline/rectangle_config/distance_threshold", 
                   obstacle_config_.rectangle_config.distance_threshold, 0.05f);
        pnh_.param("obstacle_pipeline/rectangle_config/min_inliers", 
                   obstacle_config_.rectangle_config.min_inliers, 300);
        pnh_.param("obstacle_pipeline/rectangle_config/length_min", 
                   obstacle_config_.rectangle_config.length_min, 0.8f);
        pnh_.param("obstacle_pipeline/rectangle_config/length_max", 
                   obstacle_config_.rectangle_config.length_max, 2.5f);
        pnh_.param("obstacle_pipeline/rectangle_config/width_min", 
                   obstacle_config_.rectangle_config.width_min, 0.5f);
        pnh_.param("obstacle_pipeline/rectangle_config/width_max", 
                   obstacle_config_.rectangle_config.width_max, 1.5f);
        pnh_.param("obstacle_pipeline/rectangle_config/plane_distance_threshold", 
                   obstacle_config_.rectangle_config.plane_distance_threshold, 0.08f);
        pnh_.param("obstacle_pipeline/rectangle_config/plane_angle_threshold", 
                   obstacle_config_.rectangle_config.plane_angle_threshold, 30.0f);
        pnh_.param("obstacle_pipeline/rectangle_config/plane_normal_z_max", 
                   obstacle_config_.rectangle_config.plane_normal_z_max, 0.5f);
        pnh_.param("obstacle_pipeline/rectangle_config/coverage_threshold", 
                   obstacle_config_.rectangle_config.coverage_threshold, 0.5f);
        pnh_.param("obstacle_pipeline/rectangle_config/ransac_iterations",
                   obstacle_config_.rectangle_config.ransac_iterations, 200);
        pnh_.param("obstacle_pipeline/rectangle_config/max_rectangles",
                   obstacle_config_.rectangle_config.max_rectangles, 3);
        pnh_.param("obstacle_pipeline/rectangle_config/hollow_ratio_threshold",
                   obstacle_config_.rectangle_config.hollow_ratio_threshold, 0.4f);

        // 坐标转换配置
        pnh_.param("obstacle_pipeline/transform_config/enable",
                   obstacle_config_.transform_config.enable, false);
        pnh_.param("obstacle_pipeline/transform_config/pose_topic",
                   obstacle_config_.transform_config.pose_topic, std::string("/mavros/local_position/pose"));
        pnh_.param("obstacle_pipeline/transform_config/frame_id",
                   obstacle_config_.transform_config.frame_id, std::string("world"));

        // 聚类配置
        pnh_.param("obstacle_pipeline/cluster_config/enable", 
                   obstacle_config_.cluster_config.enable, true);
        pnh_.param("obstacle_pipeline/cluster_config/cluster_tolerance", 
                   obstacle_config_.cluster_config.cluster_tolerance, 0.1f);
        pnh_.param("obstacle_pipeline/cluster_config/min_cluster_size", 
                   obstacle_config_.cluster_config.min_cluster_size, 50);
        pnh_.param("obstacle_pipeline/cluster_config/max_cluster_size", 
                   obstacle_config_.cluster_config.max_cluster_size, 1000);

        // OBB 配置
        pnh_.param("obstacle_pipeline/obb_config/inflation_radius", 
                   obstacle_config_.obb_config.inflation_radius, 0.1f);
        pnh_.param("obstacle_pipeline/obb_config/min_obstacle_height", 
                   obstacle_config_.obb_config.min_obstacle_height, 0.2f);

        // 时间测量配置（新 Pipeline）
        pnh_.param("obstacle_pipeline/timing_config/enable", 
                   obstacle_config_.timing_config.enable, true);
        pnh_.param("obstacle_pipeline/timing_config/downsample", 
                   obstacle_config_.timing_config.downsample, true);
        pnh_.param("obstacle_pipeline/timing_config/normal", 
                   obstacle_config_.timing_config.normal, true);
        pnh_.param("obstacle_pipeline/timing_config/ground", 
                   obstacle_config_.timing_config.ground, true);
        pnh_.param("obstacle_pipeline/timing_config/wall", 
                   obstacle_config_.timing_config.wall, true);
        pnh_.param("obstacle_pipeline/timing_config/cluster", 
                   obstacle_config_.timing_config.cluster, true);
        pnh_.param("obstacle_pipeline/timing_config/rectangle", 
                   obstacle_config_.timing_config.rectangle, true);
        pnh_.param("obstacle_pipeline/timing_config/obb", 
                   obstacle_config_.timing_config.obb, true);
        pnh_.param("obstacle_pipeline/timing_config/total", 
                   obstacle_config_.timing_config.total, true);
    }

    // 填充结果消息（使用多态）
    void fillResultMessage(const PointCloudPtrT &cloud,
                           pcl_detection::ObjectDetectionResult &result) {
        // 信息头与状态
        result.header.stamp    = ::ros::Time::now();
        result.header.frame_id = "base_link";
        result.success         = true;
        result.status_message  = "Success";

        // 各物体个数
        result.wall_count      = 0;
        result.cylinder_count  = 0;
        result.circle_count    = 0;
        result.rectangle_count = 0;
        result.obstacle_count  = 0;
        result.rectangle_time  = 0.0f;

        const auto& objs = use_obstacle_pipeline_ ? obstacle_pipeline_.objects : pipeline_.objects;
        const auto& filtered_cloud = use_obstacle_pipeline_ ? obstacle_pipeline_.filtered_cloud : pipeline_.filtered_cloud;

        // 各物体具体信息（使用多态）
        for (const auto &obj : objs) {
            pcl_detection::DetectedObject obj_msg;

            // 基础信息
            obj_msg.header          = result.header;
            obj_msg.name            = obj->name;
            obj_msg.point_count     = obj->inliers->indices.size();
            obj_msg.extraction_time = obj->extraction_time;
            obj_msg.width           = obj->width;
            obj_msg.height          = obj->height;
            obj_msg.depth           = obj->depth;

            // 使用多态获取类型和参数
            obj_msg.type = obj->getMessageType();
            
            // 填充类型特定信息
            switch (obj_msg.type) {
                case 0:  // Wall
                    result.wall_count++;
                    obj_msg.plane_coeffs.resize(4);
                    obj->fillMessageParams(obj_msg.plane_coeffs);

                    {
                        // 墙体：使用边界框中心（避免点云分布不均导致的偏移）
                        Eigen::Vector4f centroid = core::computeRobustCentroid(
                            *filtered_cloud, *obj->inliers, "Wall", 0.1f);
                        obj_msg.position.x = centroid[0];
                        obj_msg.position.y = centroid[1];
                        obj_msg.position.z = centroid[2];
                    }
                    break;

                case 1:  // Cylinder
                    result.cylinder_count++;
                    obj_msg.radius = obj->coefficients->values[6];
                    obj_msg.position.x = obj->coefficients->values[0];
                    obj_msg.position.y = obj->coefficients->values[1];
                    obj_msg.position.z = obj->coefficients->values[2];
                    break;

                case 2:  // Circle
                    result.circle_count++;
                    obj_msg.radius = obj->coefficients->values[6];
                    obj_msg.position.x = obj->coefficients->values[0];
                    obj_msg.position.y = obj->coefficients->values[1];
                    obj_msg.position.z = obj->coefficients->values[2];
                    break;

                case 3:  // Rectangle
                    result.rectangle_count++;
                    obj_msg.width = obj->coefficients->values[6];
                    obj_msg.height = obj->coefficients->values[7];
                    obj_msg.radius = obj->coefficients->values[8];  // 角度
                    obj_msg.position.x = obj->coefficients->values[0];
                    obj_msg.position.y = obj->coefficients->values[1];
                    obj_msg.position.z = obj->coefficients->values[2];
                    break;

                case 4:  // Obstacle
                    result.obstacle_count++;
                    // 障碍物：使用体素化质心（避免点云分布不均）
                    {
                        Eigen::Vector4f centroid = core::computeRobustCentroid(
                            *filtered_cloud, *obj->inliers, "Obstacle", 0.1f);
                        obj_msg.position.x = centroid[0];
                        obj_msg.position.y = centroid[1];
                        obj_msg.position.z = centroid[2];
                    }
                    // 障碍物 OBB 系数
                    if (obj->coefficients->values.size() >= 9) {
                        obj_msg.obb_coeffs.resize(9);
                        for (int i = 0; i < 9; ++i) {
                            obj_msg.obb_coeffs[i] = obj->coefficients->values[i];
                        }
                    }
                    break;

                default:
                    ROS_WARN("Unknown object type: %d", obj_msg.type);
                    break;
            }

            result.objects.push_back(obj_msg);
        }

        // 下采样时间
        result.downsample_time = use_obstacle_pipeline_ ?
                                  obstacle_pipeline_.downsample_time_ :
                                  pipeline_.downsample_time_;
    }

    /**
     * @brief 发布点云到 RViz
     */
    void publishCloud(const PointCloudPtrT &cloud) {
        if (!cloud || cloud->empty()) return;

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "laser_livox";  // 与 marker 坐标系一致
        cloud_msg.header.stamp = ::ros::Time::now();
        cloud_pub_.publish(cloud_msg);
    }
    
    /**
     * @brief 按物体类型发布点云（墙体、方环、障碍物）
     */
    void publishObjectClouds(const PointCloudPtrT &cloud) {
        const auto& objs = use_obstacle_pipeline_ ? obstacle_pipeline_.objects : pipeline_.objects;
        const auto& filtered_cloud = use_obstacle_pipeline_ ? obstacle_pipeline_.filtered_cloud : pipeline_.filtered_cloud;
        
        if (!cloud || !filtered_cloud) return;
        
        // 创建带颜色的点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectangle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for (const auto& obj : objs) {
            if (!obj->inliers) continue;
            
            // 根据物体类型选择目标点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud;
            uint8_t r, g, b;
            
            if (obj->getType() == "Wall") {
                target_cloud = wall_cloud;
                r = 255; g = 0; b = 0;  // 红色：墙体
            } else if (obj->getType() == "Rectangle") {
                target_cloud = rectangle_cloud;
                r = 255; g = 255; b = 0;  // 黄色：方环
            } else if (obj->getType() == "Obstacle") {
                target_cloud = obstacle_cloud;
                r = 0; g = 255; b = 0;  // 绿色：障碍物
            } else {
                continue;
            }
            
            // 提取物体点云并着色
            for (int idx : obj->inliers->indices) {
                if (idx >= 0 && idx < static_cast<int>(filtered_cloud->size())) {
                    pcl::PointXYZRGB pt;
                    pt.x = filtered_cloud->points[idx].x;
                    pt.y = filtered_cloud->points[idx].y;
                    pt.z = filtered_cloud->points[idx].z;
                    pt.r = r;
                    pt.g = g;
                    pt.b = b;
                    target_cloud->push_back(pt);
                }
            }
        }
        
        // 发布点云（检查点数阈值）
        if (wall_cloud->size() >= min_cloud_points_) {
            publishColoredCloud(wall_cloud, wall_cloud_pub_);
        }
        if (rectangle_cloud->size() >= min_cloud_points_) {
            publishColoredCloud(rectangle_cloud, rectangle_cloud_pub_);
        }
        if (obstacle_cloud->size() >= min_cloud_points_) {
            publishColoredCloud(obstacle_cloud, obstacle_cloud_pub_);
        }
    }
    
    /**
     * @brief 发布带颜色的点云
     */
    void publishColoredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                             ::ros::Publisher& pub) {
        if (!cloud || cloud->empty()) return;

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "laser_livox";
        cloud_msg.header.stamp = ::ros::Time::now();
        pub.publish(cloud_msg);
    }
    
    /**
     * @brief 发布聚类后的簇点云（每个簇一种颜色）
     */
    void publishClusterClouds(const PointCloudPtrT &cloud) {
        const auto& filtered_cloud = obstacle_pipeline_.filtered_cloud;
        const auto& clusters = obstacle_pipeline_.clusters;
        
        if (!filtered_cloud || clusters.empty()) return;
        
        // 创建带颜色的簇点云（每个簇不同颜色）
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // 颜色循环（最多 10 种颜色）
        const uint8_t colors[][3] = {
            {255, 0, 0},      // 红
            {0, 255, 0},      // 绿
            {0, 0, 255},      // 蓝
            {255, 255, 0},    // 黄
            {255, 0, 255},    // 品红
            {0, 255, 255},    // 青
            {255, 128, 0},    // 橙
            {128, 0, 255},    // 紫
            {128, 255, 0},    // 黄绿
            {255, 0, 128},    // 粉红
        };
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto& cluster = clusters[i];
            uint8_t r = colors[i % 10][0];
            uint8_t g = colors[i % 10][1];
            uint8_t b = colors[i % 10][2];
            
            // 提取簇点云并着色
            for (int idx : cluster->indices) {
                if (idx >= 0 && idx < static_cast<int>(filtered_cloud->size())) {
                    pcl::PointXYZRGB pt;
                    pt.x = filtered_cloud->points[idx].x;
                    pt.y = filtered_cloud->points[idx].y;
                    pt.z = filtered_cloud->points[idx].z;
                    pt.r = r;
                    pt.g = g;
                    pt.b = b;
                    cluster_cloud->push_back(pt);
                }
            }
        }
        
        // 发布簇点云（检查点数阈值）
        if (cluster_cloud->size() >= static_cast<size_t>(min_cloud_points_)) {
            publishColoredCloud(cluster_cloud, cluster_cloud_pub_);
        }
    }

  private:
    ::ros::NodeHandle nh_;   // 全局节点句柄
    ::ros::NodeHandle pnh_;  // 私有节点句柄

    // 原 Pipeline 配置和实例
    core::ObjectDetectionConfig config_;
    core::ObjectDetectionPipeline<PointT> pipeline_;

    // 新 Pipeline（障碍物 Pipeline）配置和实例
    core::ObstaclePipelineConfig obstacle_config_;
    core::ObstaclePipeline<PointT> obstacle_pipeline_;

    PointCloudPtrT cloud_;

    // 点云发布器
    ::ros::Publisher cloud_pub_;
    ::ros::Publisher wall_cloud_pub_;
    ::ros::Publisher rectangle_cloud_pub_;
    ::ros::Publisher obstacle_cloud_pub_;
    ::ros::Publisher cluster_cloud_pub_;  // 聚类簇点云发布器
    
    // 无人机位姿订阅器
    ::ros::Subscriber pose_sub_;
    bool has_pose_;  // 是否收到过位姿

    // 点云发布配置
    bool publish_cloud_ = true;      // 默认发布点云
    int min_cloud_points_ = 10;      // 最小点云数量阈值
};

}  // namespace ros
}  // namespace pcl_object_detection
