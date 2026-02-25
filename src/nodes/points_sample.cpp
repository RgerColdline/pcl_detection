#include "points_sample.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <string>

void livox_cb(const livox_ros_driver2::CustomMsg::ConstPtr& msg, ros::NodeHandle& nh)
{
    bool save_sample = false;
    nh.getParam(SAVE_SAMPLE_PARAM, save_sample);
    
    if (!save_sample) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->header.stamp = msg->header.stamp.toSec() * 1e6;
    cloud->header.frame_id = msg->header.frame_id;

    cloud->reserve(msg->point_num);

    for (const auto& point : msg->points) {
        pcl::PointXYZI p;
        p.x = point.x;  // 不除 1000
        p.y = point.y;
        p.z = point.z;
        p.intensity = point.reflectivity;
        
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            cloud->push_back(p);
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (!cloud->empty()) {
        std::string output_dir = "/home/jetson/livox_samples";
        std::filesystem::create_directories(output_dir);
        
        static int sample_count = 0;
        std::string filename = output_dir + "/sample_" + std::to_string(sample_count++) + ".pcd";
        
        if (pcl::io::savePCDFile(filename, *cloud) >= 0) {
            ROS_INFO("Saved %lu points to %s", cloud->size(), filename.c_str());
        }
        
        nh.setParam(SAVE_SAMPLE_PARAM, false);
    }
}
