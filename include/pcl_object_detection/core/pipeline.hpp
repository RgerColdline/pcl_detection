#pragma once

#include "circle_extraction.hpp"
#include "config.hpp"
#include "cylinder_extraction.hpp"
#include "extractors_base.hpp"
#include "object_base.hpp"
#include "timer.hpp"
#include "wall_extraction.hpp"

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <memory>
#include <vector>
#include <sstream>
#include <streambuf>
#include <unistd.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace pcl_object_detection
{
namespace core
{

// 过滤特定的 PCL RANSAC 警告
class PclWarningFilter {
public:
    PclWarningFilter() {
        // 保存原始 stderr
        original_fd_ = dup(STDERR_FILENO);
        // 创建管道用于捕获 stderr
        if (pipe(pipefd_) == 0) {
            // 重定向 stderr 到管道写入端
            dup2(pipefd_[1], STDERR_FILENO);
            close(pipefd_[1]);
            pipefd_[1] = -1;  // 标记已关闭
        }
    }
    
    // 读取管道内容，过滤后输出到原始 stderr，并恢复 stderr
    void restoreAndFlush() {
        if (pipefd_[0] < 0 || original_fd_ < 0) return;
        
        // 恢复原始 stderr
        dup2(original_fd_, STDERR_FILENO);
        close(original_fd_);
        original_fd_ = -1;
        
        // 读取管道内容并过滤
        char buffer[4096];
        ssize_t n;
        while ((n = read(pipefd_[0], buffer, sizeof(buffer) - 1)) > 0) {
            buffer[n] = '\0';
            std::string line(buffer);
            
            // 过滤 RANSAC 警告
            if (line.find("[pcl::RandomSampleConsensus::computeModel] RANSAC found no model") == std::string::npos &&
                line.find("[pcl::SACSegmentationFromNormals::segment] Error segmenting the model") == std::string::npos) {
                // 不是 RANSAC 警告，输出到原始 stderr
                ::write(STDERR_FILENO, buffer, n);
            }
        }
        close(pipefd_[0]);
        pipefd_[0] = -1;
    }
    
    ~PclWarningFilter() {
        if (original_fd_ >= 0) {
            dup2(original_fd_, STDERR_FILENO);
            close(original_fd_);
        }
        if (pipefd_[0] >= 0) close(pipefd_[0]);
        if (pipefd_[1] >= 0) close(pipefd_[1]);
    }

private:
    int original_fd_ = -1;
    int pipefd_[2] = {-1, -1};
};

// ============================================================================
// 对象检测流水线
// ============================================================================
template <typename PointT> struct ObjectDetectionPipeline
{
    ObjectDetectionConfig config;

    // 原始点云
    typename pcl::PointCloud<PointT>::Ptr input_cloud;
    // 下采样后的点云
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud;
    // 法向量（如果需要）
    typename pcl::PointCloud<pcl::Normal>::Ptr normals;

    // 检测到的所有对象（统一容器）
    std::vector<Object::Ptr> objects;

    // 时间测量结果
    double downsample_time_ = 0.0;
    double total_time_      = 0.0;

    // 各类对象提取时间
    double wall_time     = 0.0;
    double cylinder_time = 0.0;
    double circle_time   = 0.0;

    // 临时点云缓冲区（复用内存，减少分配）
    typename pcl::PointCloud<PointT>::Ptr temp_cloud1_;
    typename pcl::PointCloud<PointT>::Ptr temp_cloud2_;

    // 下采样点云
    typename pcl::PointCloud<PointT>::Ptr
    downsamplePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                         bool approx, float leaf_size) {

        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

        if (approx) {
            pcl::ApproximateVoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel.filter(*cloud_filtered);
        }
        else {
            pcl::VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel.filter(*cloud_filtered);
        }

        return cloud_filtered;
    }

    // 估计法向量
    typename pcl::PointCloud<pcl::Normal>::Ptr
    estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, int k_search = 50) {

        typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(tree);
        ne.setKSearch(k_search);
        ne.compute(*normals);

        return normals;
    }

    // 运行整个流水线
    bool run(typename pcl::PointCloud<PointT>::Ptr input) {
        Timer total_timer;

        input_cloud = input;

        // 1. 下采样
        Timer downsample_timer;
        filtered_cloud =
            downsamplePointCloud(input_cloud, config.downsample_config.approx,
                                 config.downsample_config.leaf_size);
        downsample_time_  = downsample_timer.elapsed();

        // 2. 估计法向量（如果需要）
        bool need_normals = config.wall_config.using_normal ||
                            config.cylinder_config.using_normal ||
                            config.circle_config.using_normal;
        if (need_normals) {
            normals = estimateNormals(filtered_cloud);
        }

        // 3. 提取所有对象（使用提取器多态）
        objects.clear();
        std::set<int> used_indices;  // 已使用的点索引

        // 创建提取器队列
        std::vector<std::unique_ptr<IObjectExtractor<PointT>>> extractors;
        
        if (config.wall_config.enable) {
            extractors.push_back(std::make_unique<WallExtractor<PointT>>(config.wall_config));
        }
        if (config.cylinder_config.enable) {
            extractors.push_back(std::make_unique<CylinderExtractor<PointT>>(config.cylinder_config));
        }
        if (config.circle_config.enable) {
            extractors.push_back(std::make_unique<CircleExtractor<PointT>>(config.circle_config));
        }

        // 统一执行提取（过滤 PCL RANSAC 警告，保留其他错误）
        for (auto& extractor : extractors) {
            Timer timer;
            
            // 创建过滤器，临时捕获 stderr
            PclWarningFilter filter;
            
            auto objs = extractor->extract(filtered_cloud, normals, used_indices, temp_cloud1_, temp_cloud2_);
            double elapsed = timer.elapsed();
            
            // 恢复 stderr 并过滤输出
            filter.restoreAndFlush();

            // 记录时间（根据类型存储到对应变量）
            if (dynamic_cast<WallExtractor<PointT>*>(extractor.get())) {
                wall_time = elapsed;
            } else if (dynamic_cast<CylinderExtractor<PointT>*>(extractor.get())) {
                cylinder_time = elapsed;
            } else if (dynamic_cast<CircleExtractor<PointT>*>(extractor.get())) {
                circle_time = elapsed;
            }

            // 添加到对象列表
            objects.insert(objects.end(), objs.begin(), objs.end());

            // 收集已用点
            for (const auto& obj : objs) {
                for (int idx : obj->inliers->indices) {
                    used_indices.insert(idx);
                }
            }
        }

        // 合并距离过近的同类物体
        mergeNearbyCylindersAndCircles(0.3f);  // 合并质心距离小于 0.3 米的圆柱和圆环
        mergeNearbyWalls(0.1f, 10.0f);  // 合并法向量夹角<10 度、平面距离<0.1 米的墙面

        total_time_ = total_timer.elapsed();

        return true;
    }

    // 输出时间日志（受频率控制）
    void printTimingInfo(int frame_count, double log_interval_sec, int log_skip_frames) {
        if (!config.timing_config.enable) return;
        
        // 检查是否应该输出日志
        static ::ros::Time last_log_time(0);
        ::ros::Time now = ::ros::Time::now();
        
        bool should_log = false;
        if (log_interval_sec > 0 && (now - last_log_time).toSec() >= log_interval_sec) {
            should_log = true;
        } else if (log_skip_frames >= 0 && (frame_count % (log_skip_frames + 1)) == 0) {
            should_log = true;
        }
        
        if (!should_log) return;
        last_log_time = now;
        
        ROS_INFO("=== 处理时间统计 ===");
        if (config.timing_config.downsample) {
            ROS_INFO("  下采样：%.2f ms", downsample_time_);
        }
        if (config.timing_config.walls && config.wall_config.enable) {
            ROS_INFO("  墙面提取：%.2f ms", wall_time);
        }
        if (config.timing_config.cylinders && config.cylinder_config.enable) {
            ROS_INFO("  圆柱提取：%.2f ms", cylinder_time);
        }
        if (config.timing_config.circles && config.circle_config.enable) {
            ROS_INFO("  圆环提取：%.2f ms", circle_time);
        }
        if (config.timing_config.total) {
            ROS_INFO("  总耗时：%.2f ms", total_time_);
        }
    }

    // 合并距离过近的圆柱和圆环（基于质心距离）
    void mergeNearbyCylindersAndCircles(float distance_threshold) {
        if (objects.empty()) return;
        
        std::vector<bool> merged(objects.size(), false);
        
        for (size_t i = 0; i < objects.size(); ++i) {
            if (merged[i]) continue;
            
            // 只处理圆柱和圆环
            if (objects[i]->getType() != "Cylinder" && objects[i]->getType() != "Circle") continue;
            
            Eigen::Vector4f centroid_i;
            pcl::compute3DCentroid(*filtered_cloud, *objects[i]->inliers, centroid_i);
            
            for (size_t j = i + 1; j < objects.size(); ++j) {
                if (merged[j]) continue;
                
                // 只合并同类物体
                if (objects[i]->getType() != objects[j]->getType()) continue;
                
                Eigen::Vector4f centroid_j;
                pcl::compute3DCentroid(*filtered_cloud, *objects[j]->inliers, centroid_j);
                
                float dist = (centroid_i.head<3>() - centroid_j.head<3>()).norm();
                
                if (dist < distance_threshold) {
                    // 合并物体 j 到物体 i
                    mergeObjects(i, j, merged);
                }
            }
        }
        
        // 移除已合并的物体并重新编号
        finalizeMerge(merged);
    }

    // 合并墙面（基于法向量和平面几何特征）
    void mergeNearbyWalls(float plane_distance_threshold, float normal_angle_threshold_deg) {
        if (objects.empty()) return;
        
        std::vector<bool> merged(objects.size(), false);
        
        for (size_t i = 0; i < objects.size(); ++i) {
            if (merged[i]) continue;
            if (objects[i]->getType() != "Wall") continue;
            
            // 获取墙面 i 的法向量
            Eigen::Vector3f normal_i(
                objects[i]->coefficients->values[0],
                objects[i]->coefficients->values[1],
                objects[i]->coefficients->values[2]
            );
            float d_i = objects[i]->coefficients->values[3];  // 平面方程 ax+by+cz+d=0 中的 d
            
            for (size_t j = i + 1; j < objects.size(); ++j) {
                if (merged[j]) continue;
                if (objects[j]->getType() != "Wall") continue;
                
                // 获取墙面 j 的法向量
                Eigen::Vector3f normal_j(
                    objects[j]->coefficients->values[0],
                    objects[j]->coefficients->values[1],
                    objects[j]->coefficients->values[2]
                );
                float d_j = objects[j]->coefficients->values[3];
                
                // 1. 检查法向量是否平行（夹角小于阈值）
                normal_i.normalize();
                normal_j.normalize();
                float dot = std::abs(normal_i.dot(normal_j));  // cos(θ)
                float angle_deg = std::acos(std::min(1.0f, dot)) * 180.0f / M_PI;
                
                if (angle_deg > normal_angle_threshold_deg) {
                    continue;  // 法向量夹角太大，不是同一平面
                }
                
                // 2. 检查平面距离（两个平行平面的距离）
                float plane_dist = std::abs(d_i - d_j) / normal_i.norm();
                if (plane_dist > plane_distance_threshold) {
                    continue;  // 平面距离太远
                }
                
                // 3. 检查边界框是否有重叠（在平面上的投影）
                if (!checkWallOverlap(i, j)) {
                    continue;  // 墙面没有重叠
                }
                
                // 满足所有条件，合并墙面
                mergeObjects(i, j, merged);
            }
        }
        
        // 移除已合并的物体并重新编号
        finalizeMerge(merged);
    }
    
    // 检查两个墙面是否有重叠
    bool checkWallOverlap(size_t idx_i, size_t idx_j) {
        // 计算两个墙面在各自平面上的 2D 边界框
        auto getWallBounds = [this](size_t idx, Eigen::Vector3f& center, float& width, float& height) {
            const auto& obj = objects[idx];
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*filtered_cloud, *obj->inliers, centroid);
            center = centroid.head<3>();
            width = obj->width;
            height = obj->height;
        };
        
        Eigen::Vector3f center_i, center_j;
        float width_i, height_i, width_j, height_j;
        getWallBounds(idx_i, center_i, width_i, height_i);
        getWallBounds(idx_j, center_j, width_j, height_j);
        
        // 计算中心距离
        float dist = (center_i - center_j).norm();
        
        // 如果中心距离小于两个墙面宽度之和的一半，认为有重叠
        float combined_width = (width_i + width_j) / 2.0f;
        float combined_height = (height_i + height_j) / 2.0f;
        
        return (dist < combined_width * 1.5f);  // 宽松的重叠判断
    }
    
    // 合并物体 j 到物体 i
    void mergeObjects(size_t i, size_t j, std::vector<bool>& merged) {
        // 合并内点索引
        for (int idx : objects[j]->inliers->indices) {
            objects[i]->inliers->indices.push_back(idx);
        }
        std::sort(objects[i]->inliers->indices.begin(), objects[i]->inliers->indices.end());
        objects[i]->inliers->indices.erase(
            std::unique(objects[i]->inliers->indices.begin(), objects[i]->inliers->indices.end()),
            objects[i]->inliers->indices.end()
        );
        
        // 更新尺寸（取最大值）
        objects[i]->width  = std::max(objects[i]->width, objects[j]->width);
        objects[i]->height = std::max(objects[i]->height, objects[j]->height);
        objects[i]->depth  = std::max(objects[i]->depth, objects[j]->depth);
        
        // 累加提取时间
        objects[i]->extraction_time += objects[j]->extraction_time;
        
        // 标记为已合并
        merged[j] = true;
        
        ROS_DEBUG("合并物体：%s -> %s", objects[j]->name.c_str(), objects[i]->name.c_str());
    }
    
    // 完成合并：移除已合并的物体并重新编号
    void finalizeMerge(std::vector<bool>& merged) {
        std::vector<Object::Ptr> new_objects;
        for (size_t i = 0; i < objects.size(); ++i) {
            if (!merged[i]) {
                new_objects.push_back(objects[i]);
            }
        }
        objects = new_objects;
        
        // 重命名物体
        int wall_num = 1, cylinder_num = 1, circle_num = 1;
        for (auto& obj : objects) {
            if (obj->getType() == "Wall") {
                obj->name = "wall_" + std::to_string(wall_num++);
            } else if (obj->getType() == "Cylinder") {
                obj->name = "cylinder_" + std::to_string(cylinder_num++);
            } else if (obj->getType() == "Circle") {
                obj->name = "circle_" + std::to_string(circle_num++);
            }
        }
    }

    // 辅助方法：获取特定类型对象
    template <typename T> std::vector<std::shared_ptr<T>> getObjectsByType() const {
        std::vector<std::shared_ptr<T>> result;
        for (const auto &obj : objects) {
            if (auto ptr = std::dynamic_pointer_cast<T>(obj)) {
                result.push_back(ptr);
            }
        }
        return result;
    }
};

}  // namespace core
}  // namespace pcl_object_detection
