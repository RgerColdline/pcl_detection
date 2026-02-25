#pragma once

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <livox_ros_driver2/CustomMsg.h>

namespace pcl_object_detection
{
namespace ros
{

/**
 * @brief Livox 点云转换工具
 */
class LivoxConverter
{
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtrT = PointCloudT::Ptr;

    /**
     * @brief 将 Livox CustomMsg 转换为 PCL 点云
     */
    static bool convert(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg,
                        PointCloudPtrT& output_cloud)
    {
        if (!livox_msg) {
            ROS_ERROR("Livox msg is null!");
            return false;
        }

        output_cloud = PointCloudPtrT(new PointCloudT());
        output_cloud->header.stamp = livox_msg->header.stamp.toSec() * 1e6;
        output_cloud->header.frame_id = livox_msg->header.frame_id;

        output_cloud->reserve(livox_msg->point_num);

        int count = 0;
        for (const auto& point : livox_msg->points) {
            PointT p;
            p.x = point.x;  // 不除 1000，直接使用原始值
            p.y = point.y;
            p.z = point.z;
            p.intensity = point.reflectivity;

            if (count < 5) {
                ROS_INFO("Point %d: raw=(%d,%d,%d) -> (%.3f,%.3f,%.3f)",
                         count, point.x, point.y, point.z, p.x, p.y, p.z);
            }
            count++;

            output_cloud->push_back(p);
        }

        ROS_INFO("Livox conversion: input=%d, output=%lu",
                 livox_msg->point_num, output_cloud->size());

        output_cloud->width = output_cloud->size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;

        return !output_cloud->empty();
    }
};

}  // namespace ros
}  // namespace pcl_object_detection
