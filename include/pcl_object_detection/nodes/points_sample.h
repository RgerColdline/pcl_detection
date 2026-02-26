#pragma once

#include <livox_ros_driver2/CustomMsg.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sys/stat.h>

#include <ctime>
#include <string>

/**
 * @file points_sample.h
 * @brief 极简Livox点云样本采集工具（修复PCD保存错误）
 *
 * 修复了关键错误：
 * - 确保点云width和height正确设置
 * - 避免"Number of points different than width * height"错误
 *
 * 用法：
 *   #include "points_sample.h"
 *   ros::Subscriber sub = nh.subscribe("/livox/lidar", 10, livox_cb);
 *
 * 触发保存：
 *   rosparam set /livox/save_sample true
 */

// 样本保存目录
#define SAMPLE_DIR        "~/livox_samples"

// ROS参数名称
#define SAVE_SAMPLE_PARAM "/livox/save_sample"

/**
 * @brief 确保样本目录存在
 */
inline void ensure_sample_dir() {
    static bool dir_checked = false;
    if (dir_checked) return;

    // 展开~为家目录
    std::string dir = SAMPLE_DIR;
    if (dir.find("~/") == 0) {
        char *home = getenv("HOME");
        if (home) {
            dir = std::string(home) + dir.substr(1);
        }
    }

    // 创建目录
    mkdir(dir.c_str(), 0755);
    dir_checked = true;
}

/**
 * @brief 生成带时间戳的PCD文件名
 * @return std::string 文件路径
 */
inline std::string generate_pcd_filename() {
    ensure_sample_dir();

    // 获取当前时间
    auto now = std::time(nullptr);
    auto *tm = std::localtime(&now);

    // 格式化文件名
    char filename[100];
    snprintf(filename, sizeof(filename), "%s/livox_%04d%02d%02d_%02d%02d%02d.pcd", getenv("HOME"),
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

    return std::string(filename);
}

/**
 * @brief 保存当前帧为PCD文件
 *
 * @param msg Livox消息
 * @param nh ROS节点句柄
 */
inline void livox_cb(const livox_ros_driver2::CustomMsg::ConstPtr &msg, ros::NodeHandle &nh) {
    // 检查是否需要保存样本
    bool save_requested = false;
    nh.getParam(SAVE_SAMPLE_PARAM, save_requested);

    if (save_requested) {
        // 创建PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 填充点云数据
        for (uint32_t i = 0; i < msg->point_num; ++i) {
            const auto &src = msg->points[i];
            if (std::isfinite(src.x) && std::isfinite(src.y) && std::isfinite(src.z)) {
                pcl::PointXYZI pt;
                pt.x         = src.x;
                pt.y         = src.y;
                pt.z         = src.z;
                pt.intensity = static_cast<float>(src.reflectivity);
                cloud->points.push_back(pt);
            }
        }

        // ====== 关键修复：正确设置点云维度 ======
        cloud->width    = cloud->points.size();
        cloud->height   = 1;  // 表示这是一个非结构化点云
        cloud->is_dense = (cloud->points.size() > 0);

        if (!cloud->points.empty()) {
            // 保存PCD文件
            std::string filename = generate_pcd_filename();

            // 使用正确的保存函数（指定二进制或ASCII）
            if (pcl::io::savePCDFile(filename, *cloud) == 0) {
                ROS_INFO("[PCD] 已保存: %s (%zu points)", filename.c_str(), cloud->size());
            }
            else {
                ROS_ERROR("[PCD] 保存失败: %s", filename.c_str());
            }
        }
        else {
            ROS_WARN("[PCD] 无法保存: 点云为空");
        }

        // 重置参数
        nh.setParam(SAVE_SAMPLE_PARAM, false);
    }
}