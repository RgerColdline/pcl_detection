#pragma once

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 计算鲁棒质心（解决点云分布不均匀导致的偏移问题）
 * 
 * 问题：pcl::compute3DCentroid 计算的是算术平均质心，当点云分布不均匀时
 *       （如墙体一侧密集一侧稀疏），质心会偏向密集侧。
 * 
 * 解决方案：
 * 1. 对于平面物体（墙体、方环）：使用边界框中心
 * 2. 对于 3D 物体（圆柱、障碍物）：先体素化再计算质心
 * 3. 对于圆环：使用拟合的圆心
 */

/**
 * @brief 方法 1：边界框中心（适合平面物体：墙体、方环）
 */
template <typename PointT>
Eigen::Vector4f computeBBoxCentroid(
    const pcl::PointCloud<PointT>& cloud,
    const pcl::PointIndices& indices)
{
    if (indices.indices.empty()) {
        return Eigen::Vector4f::Zero();
    }
    
    // 遍历索引找最小最大值
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    float min_z = FLT_MAX, max_z = -FLT_MAX;
    
    for (int idx : indices.indices) {
        if (idx >= 0 && idx < static_cast<int>(cloud.size())) {
            const PointT& pt = cloud.points[idx];
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
            min_z = std::min(min_z, pt.z);
            max_z = std::max(max_z, pt.z);
        }
    }
    
    Eigen::Vector4f center;
    center[0] = (min_x + max_x) / 2.0f;
    center[1] = (min_y + max_y) / 2.0f;
    center[2] = (min_z + max_z) / 2.0f;
    center[3] = 1.0f;
    
    return center;
}

/**
 * @brief 方法 2：体素化质心（适合 3D 物体：圆柱、障碍物）
 * 
 * 先体素下采样均匀化点云，再计算质心
 */
template <typename PointT>
Eigen::Vector4f computeVoxelizedCentroid(
    const pcl::PointCloud<PointT>& cloud,
    const pcl::PointIndices& indices,
    float leaf_size = 0.1f)
{
    // 提取子点云
    typename pcl::PointCloud<PointT>::Ptr sub_cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(cloud, indices, *sub_cloud);
    
    // 体素下采样（均匀化）
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(sub_cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    voxel.filter(*filtered);
    
    if (filtered->empty()) {
        // 如果体素化后为空，回退到原始质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*sub_cloud, centroid);
        return centroid;
    }
    
    // 计算均匀化后的质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*filtered, centroid);
    return centroid;
}

/**
 * @brief 方法 3：中位数质心（抗离群点）
 * 
 * 使用坐标中位数而非平均值，对离群点不敏感
 */
template <typename PointT>
Eigen::Vector4f computeMedianCentroid(
    const pcl::PointCloud<PointT>& cloud,
    const pcl::PointIndices& indices)
{
    if (indices.indices.empty()) {
        return Eigen::Vector4f::Zero();
    }
    
    std::vector<float> xs, ys, zs;
    xs.reserve(indices.indices.size());
    ys.reserve(indices.indices.size());
    zs.reserve(indices.indices.size());
    
    for (int idx : indices.indices) {
        if (idx >= 0 && idx < static_cast<int>(cloud.size())) {
            xs.push_back(cloud.points[idx].x);
            ys.push_back(cloud.points[idx].y);
            zs.push_back(cloud.points[idx].z);
        }
    }
    
    if (xs.empty()) {
        return Eigen::Vector4f::Zero();
    }
    
    // 排序取中位数
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());
    
    Eigen::Vector4f median;
    size_t mid = xs.size() / 2;
    median[0] = xs[mid];
    median[1] = ys[mid];
    median[2] = zs[mid];
    median[3] = 1.0f;
    
    return median;
}

/**
 * @brief 统一接口：根据物体类型选择质心计算方法
 * 
 * @param cloud 输入点云
 * @param indices 点索引
 * @param object_type 物体类型 ("Wall", "Rectangle", "Cylinder", "Circle", "Obstacle")
 * @param leaf_size 体素大小（仅对 3D 物体有效）
 * @return 质心坐标 (x, y, z, 1)
 */
template <typename PointT>
Eigen::Vector4f computeRobustCentroid(
    const pcl::PointCloud<PointT>& cloud,
    const pcl::PointIndices& indices,
    const std::string& object_type = "Obstacle",
    float leaf_size = 0.1f)
{
    // 平面物体：使用边界框中心
    if (object_type == "Wall" || object_type == "Rectangle") {
        return computeBBoxCentroid(cloud, indices);
    }
    // 圆环：使用原始质心（通常已经过 RANSAC 拟合）
    else if (object_type == "Circle") {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, indices, centroid);
        return centroid;
    }
    // 3D 物体（圆柱、障碍物）：使用体素化质心
    else {
        return computeVoxelizedCentroid(cloud, indices, leaf_size);
    }
}

}  // namespace core
}  // namespace pcl_object_detection
