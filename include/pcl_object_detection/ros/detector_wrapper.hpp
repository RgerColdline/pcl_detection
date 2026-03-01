#pragma once

#include "pcl_object_detection/core/pipeline.hpp"
#include "pcl_object_detection/core/timer.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
 * @brief 对象检测器的 ROS 包装器
 * 
 * 从 ROS 参数服务器加载配置（参数应在 launch 文件中通过 rosparam command="load" 加载）
 */
template <typename PointT = pcl::PointXYZI> class ObjectDetectorWrapper
{
  public:
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    // 初始化检测器（从 ROS 参数服务器加载配置）
    ObjectDetectorWrapper(::ros::NodeHandle &nh, ::ros::NodeHandle &pnh) 
        : nh_(nh), pnh_(pnh) {
        loadConfigFromROS();

        pipeline_.config = config_;

        ROS_INFO("ObjectDetectorWrapper 初始化完成（从 ROS 参数服务器加载配置）");
    }

    // 检测步骤：输入点云，输出检测结果
    bool process(const PointCloudPtrT &cloud, pcl_detection::ObjectDetectionResult &result) {
        core::Timer timer;  // 开始计时处理时间
        frame_count_++;  // 帧计数递增

        cloud_ = cloud;

        // 处理点云
        if (!pipeline_.run(cloud)) {
            ROS_ERROR("Pipeline 处理失败");
            result.success        = false;
            result.status_message = "Pipeline execution failed";
            return false;
        }

        // 填充结果消息
        fillResultMessage(cloud, result);

        // 统计处理耗时
        result.total_time = timer.elapsed();

        // 输出时间日志（受频率控制）
        pipeline_.printTimingInfo(frame_count_, log_interval_sec_, log_skip_frames_);

        // 输出物体详细信息（受频率控制）
        printObjectDetails(pipeline_.objects);

        return true;
    }

  private:
    // 帧计数器
    int frame_count_ = 0;
    
    // 日志频率参数
    int log_skip_frames_ = 19;
    double log_interval_sec_ = 2.0;

  private:
    // 打印物体详细信息（受频率控制）
    void printObjectDetails(const std::vector<pcl_object_detection::core::Object::Ptr>& objs) {
        // 从参数服务器读取日志频率配置
        int log_skip_frames;
        double log_interval_sec;
        nh_.param("log_skip_frames", log_skip_frames, 19);
        nh_.param("log_interval_sec", log_interval_sec, 2.0);
        
        // 检查是否应该输出日志
        static ::ros::Time last_log_time(0);
        static int frame_count = 0;
        frame_count++;
        ::ros::Time now = ::ros::Time::now();
        
        bool should_log = false;
        if (log_interval_sec > 0 && (now - last_log_time).toSec() >= log_interval_sec) {
            should_log = true;
        } else if (log_skip_frames >= 0 && (frame_count % (log_skip_frames + 1)) == 0) {
            should_log = true;
        }
        
        if (!should_log) return;
        last_log_time = now;
        
        // 输出物体详细信息
        for (const auto& obj : objs) {
            std::ostringstream oss;
            oss << "\n=== " << obj->name << " (" << obj->getType() << ") ===\n";
            oss << "  点数：" << obj->getPointCount() << "\n";
            oss << "  尺寸：宽=" << obj->width << "m, 高=" << obj->height << "m, 深=" << obj->depth << "m\n";
            oss << "  提取时间：" << obj->extraction_time << " ms\n";

            // 输出位置（质心）
            // 注意：必须使用 pipeline_.filtered_cloud 而不是 cloud_，因为 inliers 索引是相对于 filtered_cloud 的
            if (obj->inliers && !obj->inliers->indices.empty()) {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*pipeline_.filtered_cloud, *obj->inliers, centroid);
                oss << "  位置 (centroid 完整值): [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ", " << centroid[3] << "]\n";
                oss << "  有效点数：" << pcl::compute3DCentroid(*pipeline_.filtered_cloud, *obj->inliers, centroid) << "\n";
            }

            // 输出法向量/轴向（墙面是法向量，圆柱/圆环是轴向）
            if (obj->coefficients && obj->coefficients->values.size() >= 3) {
                if (obj->getType() == "Wall") {
                    // 墙面：coefficients[0-2] 是法向量
                    oss << "  法向量：(" << obj->coefficients->values[0] << ", "
                        << obj->coefficients->values[1] << ", " << obj->coefficients->values[2] << ")\n";
                } else if (obj->getType() == "Cylinder") {
                    // 圆柱：coefficients[0-2] 是中心位置，[3-5] 是轴向
                    oss << "  中心位置：(" << obj->coefficients->values[0] << ", "
                        << obj->coefficients->values[1] << ", " << obj->coefficients->values[2] << ")\n";
                    oss << "  轴向：(" << obj->coefficients->values[3] << ", "
                        << obj->coefficients->values[4] << ", " << obj->coefficients->values[5] << ")\n";
                } else if (obj->getType() == "Circle") {
                    // 圆环：coefficients[0-2] 是中心位置，[3-5] 是法向量
                    oss << "  中心位置：(" << obj->coefficients->values[0] << ", "
                        << obj->coefficients->values[1] << ", " << obj->coefficients->values[2] << ")\n";
                    oss << "  法向量：(" << obj->coefficients->values[3] << ", "
                        << obj->coefficients->values[4] << ", " << obj->coefficients->values[5] << ")\n";
                }
            }
            
            // 输出模型参数
            std::vector<float> params;
            obj->getParameters(params);
            if (!params.empty()) {
                oss << "  参数：";
                for (size_t i = 0; i < params.size(); ++i) {
                    if (i > 0) oss << ", ";
                    oss << params[i];
                }
                oss << "\n";
            }

            ROS_INFO_STREAM(oss.str());
        }
    }

  private:
    // 从 ROS 参数服务器加载配置（使用私有节点句柄读取）
    void loadConfigFromROS() {
        // 日志频率配置
        pnh_.param("log_skip_frames", log_skip_frames_, 19);
        pnh_.param("log_interval_sec", log_interval_sec_, 2.0);
        
        // 调试：先检查参数是否存在
        if (!pnh_.hasParam("wall_extraction/min_inliers")) {
            ROS_ERROR("参数服务器中没有 wall_extraction/min_inliers 参数！");
            ROS_INFO("请确认 launch 文件中使用 <rosparam command=\"load\" ... /> 加载配置");
        }

        int yaml_min_inliers = -1;
        pnh_.getParam("wall_extraction/min_inliers", yaml_min_inliers);
        ROS_INFO("从参数服务器读取：wall_extraction/min_inliers = %d", yaml_min_inliers);
        
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
    }

    // 填充结果消息
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

        // 各物体具体信息
        for (const auto &obj : pipeline_.objects) {
            pcl_detection::DetectedObject obj_msg;

            // 基础信息
            obj_msg.header          = result.header;
            obj_msg.name            = obj->name;
            obj_msg.point_count     = obj->inliers->indices.size();
            obj_msg.extraction_time = obj->extraction_time;
            obj_msg.width           = obj->width;
            obj_msg.height          = obj->height;
            obj_msg.depth           = obj->depth;

            // 类型特定信息
            if (obj->getType() == "Wall") {
                obj_msg.type = 0;
                result.wall_count++;

                obj_msg.plane_coeffs.resize(4);
                for (int i = 0; i < 4; ++i) {
                    obj_msg.plane_coeffs[i] = obj->coefficients->values[i];
                }

                // 注意：必须使用 pipeline_.filtered_cloud，因为 inliers 索引是相对于 filtered_cloud 的
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*pipeline_.filtered_cloud, *obj->inliers, centroid);
                obj_msg.position.x = centroid[0];
                obj_msg.position.y = centroid[1];
                obj_msg.position.z = centroid[2];
            }
            else if (obj->getType() == "Cylinder") {
                obj_msg.type = 1;
                result.cylinder_count++;

                obj_msg.radius     = obj->coefficients->values[6];

                obj_msg.position.x = obj->coefficients->values[0];
                obj_msg.position.y = obj->coefficients->values[1];
                obj_msg.position.z = obj->coefficients->values[2];
            }
            else if (obj->getType() == "Circle") {
                obj_msg.type = 2;
                result.circle_count++;

                obj_msg.radius     = obj->coefficients->values[6];

                obj_msg.position.x = obj->coefficients->values[0];
                obj_msg.position.y = obj->coefficients->values[1];
                obj_msg.position.z = obj->coefficients->values[2];
            }

            result.objects.push_back(obj_msg);
        }

        // 下采样时间
        result.downsample_time = pipeline_.downsample_time_;
    }

  private:
    ::ros::NodeHandle nh_;   // 全局节点句柄（用于发布/订阅）
    ::ros::NodeHandle pnh_;  // 私有节点句柄（用于读取参数）
    core::ObjectDetectionConfig config_;
    core::ObjectDetectionPipeline<PointT> pipeline_;
    PointCloudPtrT cloud_;
};

}  // namespace ros
}  // namespace pcl_object_detection
