/**
 * @file object_detector_node.cpp
 * @brief ROS 对象检测节点
 *
 * 订阅：/livox/lidar (livox_ros_driver2/CustomMsg)
 * 发布：/pcl_detection/result (ObjectDetectionResult)
 *       /pcl_detection/objects (DetectedObject[])
 * 
 * 配置说明：
 *   在 launch 文件中使用 <rosparam command="load" file="config.yaml" /> 加载配置
 *   所有检测参数通过 ROS 参数服务器传递
 */

#include "pcl_object_detection/ros/detector_wrapper.hpp"
#include "pcl_object_detection/ros/livox_converter.hpp"
#include "pcl_object_detection/core/timer.hpp"

#include <livox_ros_driver2/CustomMsg.h>
#include <pcl_detection/DetectedObject.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <ros/ros.h>

namespace pcl_object_detection
{
namespace ros
{

class ObjectDetectorNode
{
  public:
    ObjectDetectorNode(::ros::NodeHandle &nh, ::ros::NodeHandle &pnh)
        : nh_(nh)
        , pnh_(pnh)
        , detector_(nullptr)
        , process_count_(0)
        , skip_count_(0)
        , last_log_time_(::ros::Time(0))
    {
        // 从 ROS 参数服务器加载所有参数（YAML 已加载到全局命名空间）
        std::string input_topic;
        std::string result_topic;
        std::string objects_topic;

        nh_.param("input_topic", input_topic, std::string("/livox/lidar"));
        nh_.param("result_topic", result_topic, std::string("/pcl_detection/result"));
        nh_.param("objects_topic", objects_topic, std::string("/pcl_detection/objects"));
        nh_.param("skip_frames", skip_frames_, 0);
        nh_.param("log_skip_frames", log_skip_frames_, 19);
        nh_.param("log_interval_sec", log_interval_sec_, 2.0);
        nh_.param("livox_debug_level", livox_debug_level_, 1);

        ROS_INFO("输入话题：%s", input_topic.c_str());
        ROS_INFO("输出结果话题：%s", result_topic.c_str());
        ROS_INFO("日志控制：log_skip_frames=%d, log_interval_sec=%.1f, livox_debug_level=%d",
                 log_skip_frames_, log_interval_sec_, livox_debug_level_);

        // 检测器初始化（配置从 ROS 参数服务器加载）
        try {
            detector_ = std::make_unique<ObjectDetectorWrapper<>>(nh_, pnh_);
            ROS_INFO("Detector initialized successfully");
        }
        catch (const std::exception &e) {
            ROS_FATAL("Failed to initialize detector: %s", e.what());
            throw;
        }

        // 发布器和订阅器
        result_pub_  = nh_.advertise<pcl_detection::ObjectDetectionResult>(result_topic, 10);
        objects_pub_ = nh_.advertise<pcl_detection::DetectedObject>(objects_topic, 100);

        livox_sub_   = nh_.subscribe(input_topic, 10, &ObjectDetectorNode::livoxCb, this);

        ROS_INFO("ObjectDetectorNode 初始化成功");
    }

  private:
    void livoxCb(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg) {
        // 降频采样
        if (skip_count_ > 0) {
            skip_count_--;
            return;
        }
        skip_count_ = skip_frames_;

        // 统计处理帧数
        process_count_++;

        // 转换计时
        pcl_object_detection::core::Timer convert_timer;

        // 转换（根据日志频率控制 debug 级别）
        // 先计算是否应该输出日志
        ::ros::Time now = ::ros::Time::now();
        bool should_log = false;
        if (log_interval_sec_ > 0 && (now - last_log_time_).toSec() >= log_interval_sec_) {
            should_log = true;
        } else if (log_skip_frames_ >= 0 && (process_count_ % (log_skip_frames_ + 1)) == 0) {
            should_log = true;
        }

        // 根据 should_log 决定 debug 级别
        int effective_debug_level = livox_debug_level_;
        if (effective_debug_level >= 1 && !should_log) {
            effective_debug_level = 0;  // 抑制日志
        }

        LivoxConverter::PointCloudPtrT cloud;
        if (!LivoxConverter::convert(livox_msg, cloud, effective_debug_level)) {
            ROS_WARN("[%d] 未能转换 livox 点云", process_count_);
            return;
        }

        // 点云数量过少->跳过
        if (cloud->size() < 100) {
            ROS_WARN("[%d] 点云数量过少：%lu 个点，已跳过", process_count_, cloud->size());
            return;
        }

        // 统计转换时间
        double convert_time = convert_timer.elapsed();

        // 处理
        pcl_detection::ObjectDetectionResult result;
        if (!detector_->process(cloud, result)) {
            ROS_WARN("[%d] 处理点云失败", process_count_);
            return;
        }

        // 发布结果
        result.header.frame_id = livox_msg->header.frame_id;

        result_pub_.publish(result);

        for (const auto &obj : result.objects) {
            objects_pub_.publish(obj);
        }

        // 日志输出控制（基于帧数或时间）
        if (should_log) {
            ROS_INFO("[%d] 已发布：%lu 个物体，处理耗时：%.2f ms, 转换耗时：%.2f ms",
                     process_count_, result.objects.size(), result.total_time, convert_time);
            last_log_time_ = now;
        }
    }

  private:
    ::ros::NodeHandle nh_;                               // 全局节点句柄（从全局命名空间读取参数）
    ::ros::NodeHandle pnh_;                              // 私有节点句柄（保留但暂不使用）

    std::unique_ptr<ObjectDetectorWrapper<>> detector_;  // 检测器

    ::ros::Publisher result_pub_;                        // 发布器
    ::ros::Publisher objects_pub_;                       // 发布器
    ::ros::Subscriber livox_sub_;                        // 订阅器

    int process_count_;                                  // 已处理帧数
    int skip_count_;                                     // 当前跳帧计数
    int skip_frames_;                                    // 跳帧
    int log_skip_frames_;                                // 日志跳帧数
    double log_interval_sec_;                            // 日志时间间隔（秒）
    int livox_debug_level_;                              // Livox 日志级别：0=无，1=摘要，2=详细
    ::ros::Time last_log_time_;                          // 上次日志时间
};

}  // namespace ros
}  // namespace pcl_object_detection

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");                          // 支持中文日志输出
    ros::init(argc, argv, "object_detector_node");  // ROS 节点初始化
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 初始化识别物体节点
    try {
        pcl_object_detection::ros::ObjectDetectorNode node(nh, pnh);
        ros::spin();
    }
    catch (const std::exception &e) {
        ROS_FATAL("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
