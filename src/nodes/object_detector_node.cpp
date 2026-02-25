/**
 * @file object_detector_node.cpp
 * @brief ROS 对象检测节点
 * 
 * 订阅：/livox/lidar (livox_ros_driver2/CustomMsg)
 * 发布：/pcl_detection/result (ObjectDetectionResult)
 *       /pcl_detection/objects (DetectedObject[])
 */

#include <ros/ros.h>

#include <livox_ros_driver2/CustomMsg.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>

#include "pcl_object_detection/ros/livox_converter.hpp"
#include "pcl_object_detection/ros/detector_wrapper.hpp"

namespace pcl_object_detection
{
namespace ros
{

class ObjectDetectorNode
{
public:
    ObjectDetectorNode(::ros::NodeHandle& nh, ::ros::NodeHandle& pnh)
        : nh_(nh)
        , pnh_(pnh)
        , detector_(nullptr)
        , process_count_(0)
        , skip_count_(0)
        , last_log_time_(::ros::Time(0))
        , log_skip_frames_(19)
        , log_interval_sec_(2.0)
        , livox_debug_level_(1)  // 默认只输出摘要
    {
        std::string config_file;
        std::string input_topic;
        std::string result_topic;
        std::string objects_topic;

        pnh_.param("config_file", config_file, std::string("config/sample_config.yaml"));
        pnh_.param("input_topic", input_topic, std::string("/livox/lidar"));
        pnh_.param("result_topic", result_topic, std::string("/pcl_detection/result"));
        pnh_.param("objects_topic", objects_topic, std::string("/pcl_detection/objects"));
        pnh_.param("skip_frames", skip_frames_, 0);
        
        // 加载日志频率配置
        pnh_.param("log_skip_frames", log_skip_frames_, 19);
        pnh_.param("log_interval_sec", log_interval_sec_, 2.0);
        pnh_.param("livox_debug_level", livox_debug_level_, 1);  // 0=无，1=摘要，2=详细

        ROS_INFO("Config file: %s", config_file.c_str());
        ROS_INFO("Input topic: %s", input_topic.c_str());
        ROS_INFO("Result topic: %s", result_topic.c_str());
        ROS_INFO("Log control: skip_frames=%d (every %d frames), interval=%.1f sec, livox_debug=%d", 
                 log_skip_frames_, log_skip_frames_ + 1, log_interval_sec_, livox_debug_level_);

        try {
            detector_ = std::make_unique<ObjectDetectorWrapper<>>(nh_, config_file);
            ROS_INFO("Detector initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to initialize detector: %s", e.what());
            throw;
        }

        result_pub_ = nh_.advertise<pcl_detection::ObjectDetectionResult>(result_topic, 10);
        objects_pub_ = nh_.advertise<pcl_detection::DetectedObject>(objects_topic, 100);

        livox_sub_ = nh_.subscribe(input_topic, 10, &ObjectDetectorNode::livoxCallback, this);

        ROS_INFO("ObjectDetectorNode initialized successfully");
    }

private:
    void livoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg)
    {
        if (skip_count_ > 0) {
            skip_count_--;
            return;
        }
        skip_count_ = skip_frames_;

        process_count_++;

        auto start_convert = std::chrono::high_resolution_clock::now();

        LivoxConverter::PointCloudPtrT cloud;
        if (!LivoxConverter::convert(livox_msg, cloud, livox_debug_level_)) {
            ROS_WARN("[%d] Failed to convert Livox point cloud", process_count_);
            return;
        }

        if (cloud->size() < 100) {
            ROS_WARN("[%d] Point cloud too small: %lu points, skipping", process_count_, cloud->size());
            return;
        }

        auto end_convert = std::chrono::high_resolution_clock::now();
        double convert_time = std::chrono::duration<double, std::milli>(end_convert - start_convert).count();

        pcl_detection::ObjectDetectionResult result;
        if (!detector_->process(cloud, result)) {
            ROS_WARN("[%d] Detection failed", process_count_);
            return;
        }

        result.header.frame_id = livox_msg->header.frame_id;

        result_pub_.publish(result);

        for (const auto& obj : result.objects) {
            objects_pub_.publish(obj);
        }

        // 日志输出控制（基于帧数或时间）
        ::ros::Time now = ::ros::Time::now();
        bool should_log = false;
        
        // 方式 1：基于帧数
        if (log_skip_frames_ >= 0 && (process_count_ % (log_skip_frames_ + 1)) == 0) {
            should_log = true;
        }
        
        // 方式 2：基于时间（优先级更高）
        if (log_interval_sec_ > 0 && (now - last_log_time_).toSec() >= log_interval_sec_) {
            should_log = true;
        }
        
        if (should_log) {
            ROS_INFO("[%d] Published: %lu objects, total: %.2f ms, convert: %.2f ms",
                     process_count_, result.objects.size(), result.total_time, convert_time);
            last_log_time_ = now;
        }
    }

private:
    ::ros::NodeHandle& nh_;
    ::ros::NodeHandle& pnh_;

    std::unique_ptr<ObjectDetectorWrapper<>> detector_;

    ::ros::Publisher result_pub_;
    ::ros::Publisher objects_pub_;
    ::ros::Subscriber livox_sub_;

    int process_count_;
    int skip_count_;
    int skip_frames_;
    int log_skip_frames_;       // 日志跳帧数
    double log_interval_sec_;   // 日志时间间隔（秒）
    int livox_debug_level_;     // Livox 日志级别：0=无，1=摘要，2=详细
    ::ros::Time last_log_time_;
};

}  // namespace ros
}  // namespace pcl_object_detection

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        pcl_object_detection::ros::ObjectDetectorNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
