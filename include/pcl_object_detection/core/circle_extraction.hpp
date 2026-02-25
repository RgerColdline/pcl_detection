#pragma once

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <vector>
#include <algorithm>
#include <Eigen/Geometry>

namespace pcl_object_detection {
namespace core {

// 圆环信息结构 (这里使用2D圆表示圆环)
template <typename PointT>
struct CircleInfo
{
    typename pcl::PointIndices::Ptr inliers; // 这是原始点云的索引
    typename pcl::ModelCoefficients::Ptr coefficients;
    std::string name;
    double extraction_time;
    Eigen::Vector3f color; // 颜色
    bool using_normal;     // 是否使用了法向量 (对于2D圆通常为false)
};

// 配置参数结构
struct CircleConfig
{
    bool enable = true;
    bool using_normal = false;
    float distance_threshold = 0.02f;
    int min_inliers = 200;
    float radius_min = 0.1f;
    float radius_max = 0.5f;
};

// 提取圆（2D圆，作为圆环的近似）
template <typename PointT>
std::vector<CircleInfo<PointT>> extractCircles(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    const CircleConfig& config)
{
    std::vector<CircleInfo<PointT>> circles;
    
    // 创建分割对象
    pcl::SACSegmentation<PointT> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE2D); // 使用2D圆模型
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config.distance_threshold);
    seg.setMaxIterations(1000);
    
    // 创建初始索引
    typename pcl::PointIndices::Ptr all_indices(new pcl::PointIndices);
    all_indices->indices.resize(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i)
        all_indices->indices[i] = i;
    
    typename pcl::PointIndices::Ptr current_indices = all_indices;
    int circle_num = 1;
    
    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers))
    {
        // 设置当前索引
        seg.setIndices(current_indices);
        seg.setInputCloud(cloud);
        
        // 执行分割
        typename pcl::PointIndices::Ptr inliers_local(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        auto start = std::chrono::high_resolution_clock::now();
        seg.segment(*inliers_local, *coefficients);
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(end - start).count();
        
        // 检查内点数量
        if (inliers_local->indices.size() < static_cast<size_t>(config.min_inliers)) {
            break;
        }
        
        // 对于SACMODEL_CIRCLE2D，系数格式为[x, y, radius]
        float radius = coefficients->values[2];
        
        // 检查半径是否在范围内
        if (radius < config.radius_min || radius > config.radius_max) {
            // 从当前索引中移除这些点
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (size_t i = 0; i < current_indices->indices.size(); ++i) {
                if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(), 
                              current_indices->indices[i]) == inliers_local->indices.end()) {
                    new_indices->indices.push_back(current_indices->indices[i]);
                }
            }
            current_indices = new_indices;
            continue;
        }
        
        // 检查是否与已提取的圆重复
        bool is_duplicate = false;
        for (const auto &circle : circles) {
            // 比较圆参数（中心、半径）
            float dx = coefficients->values[0] - circle.coefficients->values[0];
            float dy = coefficients->values[1] - circle.coefficients->values[1];
            float dr = std::sqrt(dx*dx + dy*dy);
            
            if (dr < 0.01 && std::abs(radius - circle.coefficients->values[2]) < 0.01) {
                is_duplicate = true;
                break;
            }
        }
        
        if (is_duplicate) {
            // 从当前索引中移除这些点
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (size_t i = 0; i < current_indices->indices.size(); ++i) {
                if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(), 
                              current_indices->indices[i]) == inliers_local->indices.end()) {
                    new_indices->indices.push_back(current_indices->indices[i]);
                }
            }
            current_indices = new_indices;
            continue;
        }
        
        // 保存圆信息
        CircleInfo<PointT> circle;
        circle.inliers = inliers_local;
        circle.coefficients = coefficients;
        circle.name = "circle_" + std::to_string(circle_num);
        circle.extraction_time = elapsed;
        circle.using_normal = false; // 2D圆不使用法向量
        circles.push_back(circle);
        
        // 从当前索引中移除这些点
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (size_t i = 0; i < current_indices->indices.size(); ++i) {
            if (std::find(inliers_local->indices.begin(), inliers_local->indices.end(), 
                          current_indices->indices[i]) == inliers_local->indices.end()) {
                new_indices->indices.push_back(current_indices->indices[i]);
            }
        }
        current_indices = new_indices;
        
        circle_num++;
        
        // 防止无限循环
        if (circle_num > 20) {
            break;
        }
    }
    
    return circles;
}

} // namespace core
} // namespace pcl_object_detection