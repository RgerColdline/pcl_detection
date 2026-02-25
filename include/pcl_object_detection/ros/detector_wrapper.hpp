#pragma once

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_object_detection/core/pipeline.hpp"
#include "pcl_object_detection/io/pointcloud_io.hpp"
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>

namespace pcl_object_detection
{
namespace ros
{

/**
 * @brief 对象检测器的 ROS 包装器
 */
template <typename PointT = pcl::PointXYZI>
class ObjectDetectorWrapper
{
public:
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    ObjectDetectorWrapper(::ros::NodeHandle& nh, const std::string& config_file)
        : nh_(nh)
    {
        if (!io::load_config(config_file, config_)) {
            ROS_ERROR("Failed to load config file: %s", config_file.c_str());
            throw std::runtime_error("Failed to load config");
        }

        pipeline_.config = config_;

        ROS_INFO("ObjectDetectorWrapper initialized with config: %s", config_file.c_str());
    }

    bool process(const PointCloudPtrT& cloud,
                 pcl_detection::ObjectDetectionResult& result)
    {
        auto start = std::chrono::high_resolution_clock::now();

        cloud_ = cloud;

        if (!pipeline_.run(cloud)) {
            ROS_ERROR("Pipeline execution failed");
            result.success = false;
            result.status_message = "Pipeline execution failed";
            return false;
        }

        fillResultMessage(cloud, result);

        auto end = std::chrono::high_resolution_clock::now();
        result.total_time = std::chrono::duration<double, std::milli>(end - start).count();

        return true;
    }

private:
    void fillResultMessage(const PointCloudPtrT& cloud,
                           pcl_detection::ObjectDetectionResult& result)
    {
        result.header.stamp = ::ros::Time::now();
        result.header.frame_id = "base_link";
        result.success = true;
        result.status_message = "Success";

        result.wall_count = 0;
        result.cylinder_count = 0;
        result.circle_count = 0;

        for (const auto& obj : pipeline_.objects) {
            pcl_detection::DetectedObject obj_msg;
            
            obj_msg.header = result.header;
            obj_msg.name = obj->name;
            obj_msg.point_count = obj->inliers->indices.size();
            obj_msg.extraction_time = obj->extraction_time;
            obj_msg.width = obj->width;
            obj_msg.height = obj->height;
            obj_msg.depth = obj->depth;

            if (obj->getType() == "Wall") {
                obj_msg.type = 0;
                result.wall_count++;
                
                obj_msg.plane_coeffs.resize(4);
                for (int i = 0; i < 4; ++i) {
                    obj_msg.plane_coeffs[i] = obj->coefficients->values[i];
                }
                
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_, *obj->inliers, centroid);
                obj_msg.position.x = centroid[0];
                obj_msg.position.y = centroid[1];
                obj_msg.position.z = centroid[2];
            }
            else if (obj->getType() == "Cylinder") {
                obj_msg.type = 1;
                result.cylinder_count++;
                
                obj_msg.radius = obj->coefficients->values[6];
                
                obj_msg.position.x = obj->coefficients->values[0];
                obj_msg.position.y = obj->coefficients->values[1];
                obj_msg.position.z = obj->coefficients->values[2];
            }
            else if (obj->getType() == "Circle") {
                obj_msg.type = 2;
                result.circle_count++;
                
                obj_msg.radius = obj->coefficients->values[6];
                
                obj_msg.position.x = obj->coefficients->values[0];
                obj_msg.position.y = obj->coefficients->values[1];
                obj_msg.position.z = obj->coefficients->values[2];
            }

            result.objects.push_back(obj_msg);
        }

        result.downsample_time = pipeline_.downsample_time_;
    }

private:
    ::ros::NodeHandle& nh_;
    core::ObjectDetectionConfig config_;
    core::ObjectDetectionPipeline<PointT> pipeline_;
    PointCloudPtrT cloud_;
};

}  // namespace ros
}  // namespace pcl_object_detection
