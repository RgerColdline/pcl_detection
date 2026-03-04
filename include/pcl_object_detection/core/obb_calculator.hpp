#pragma once

#include "obstacle_pipeline_config.hpp"
#include "timer.hpp"
#include "object_base.hpp"

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

#include <Eigen/Geometry>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 计算点云的 OBB（有向包围盒）
 * 
 * @param cloud 输入点云
 * @param indices 点云索引
 * @param obb_center OBB 中心点
 * @param obb_size OBB 尺寸 [length, width, height]
 * @param obb_axes OBB 轴向 [3 个正交单位向量]
 * @return 是否成功计算
 */
template <typename PointT>
bool computeOBB(typename pcl::PointCloud<PointT>::Ptr cloud,
                const pcl::PointIndices &indices,
                Eigen::Vector3f &obb_center,
                Eigen::Vector3f &obb_size,
                std::vector<Eigen::Vector3f> &obb_axes) {
    
    if (indices.indices.empty() || cloud->empty()) {
        return false;
    }

    // 提取子点云
    typename pcl::PointCloud<PointT>::Ptr sub_cloud(new pcl::PointCloud<PointT>);
    for (int idx : indices.indices) {
        if (idx >= 0 && idx < static_cast<int>(cloud->size())) {
            sub_cloud->push_back((*cloud)[idx]);
        }
    }

    if (sub_cloud->size() < 3) {
        return false;
    }

    // 使用 PCA 计算主方向
    pcl::PCA<PointT> pca;
    pca.setInputCloud(sub_cloud);

    // 获取特征值和特征向量
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f eigen_values = pca.getEigenValues();

    // OBB 中心（质心）
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*sub_cloud, centroid);
    obb_center = centroid.head<3>();

    // 将点投影到主方向坐标系
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    float min_z = FLT_MAX, max_z = -FLT_MAX;

    for (const auto &pt : *sub_cloud) {
        Eigen::Vector3f p(pt.x - obb_center[0], pt.y - obb_center[1], pt.z - obb_center[2]);
        
        float x = eigen_vectors(0, 0) * p[0] + eigen_vectors(1, 0) * p[1] + eigen_vectors(2, 0) * p[2];
        float y = eigen_vectors(0, 1) * p[0] + eigen_vectors(1, 1) * p[1] + eigen_vectors(2, 1) * p[2];
        float z = eigen_vectors(0, 2) * p[0] + eigen_vectors(1, 2) * p[1] + eigen_vectors(2, 2) * p[2];

        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
    }

    // OBB 尺寸
    obb_size[0] = max_x - min_x;  // 长
    obb_size[1] = max_y - min_y;  // 宽
    obb_size[2] = max_z - min_z;  // 高

    // OBB 轴向（特征向量）
    obb_axes.clear();
    obb_axes.push_back(eigen_vectors.col(0));
    obb_axes.push_back(eigen_vectors.col(1));
    obb_axes.push_back(eigen_vectors.col(2));

    return true;
}

/**
 * @brief 将 OBB 膨胀成圆柱模型（用于避障）
 * 
 * @param obb_center OBB 中心
 * @param obb_size OBB 尺寸
 * @param obb_axes OBB 轴向
 * @param inflation_radius 膨胀半径
 * @param cylinder_center 输出圆柱中心
 * @param cylinder_radius 输出圆柱半径
 * @param cylinder_height 输出圆柱高度
 * @param cylinder_axis 输出圆柱轴向
 */
void obbToCylinder(const Eigen::Vector3f &obb_center,
                   const Eigen::Vector3f &obb_size,
                   const std::vector<Eigen::Vector3f> &obb_axes,
                   float inflation_radius,
                   Eigen::Vector3f &cylinder_center,
                   float &cylinder_radius,
                   float &cylinder_height,
                   Eigen::Vector3f &cylinder_axis) {
    
    // 圆柱中心 = OBB 中心
    cylinder_center = obb_center;

    // 找到最长的两个维度作为圆柱的径向
    std::vector<std::pair<float, int>> dims;
    dims.push_back({obb_size[0], 0});
    dims.push_back({obb_size[1], 1});
    dims.push_back({obb_size[2], 2});
    
    // 按尺寸排序
    std::sort(dims.begin(), dims.end(), [](const auto &a, const auto &b) {
        return a.first > b.first;
    });

    // 最长维度作为圆柱高度方向
    int height_axis = dims[0].second;
    cylinder_height = dims[0].first;
    cylinder_axis = obb_axes[height_axis];

    // 次长维度 + 第三维度 + 膨胀半径计算圆柱半径
    float max_radial = std::max(dims[1].first, dims[2].first);
    cylinder_radius = max_radial / 2.0f + inflation_radius;

    ROS_DEBUG("[OBB->Cylinder] 半径：%.3f, 高度：%.3f", cylinder_radius, cylinder_height);
}

/**
 * @brief 更新障碍物对象为 OBB 立方体表示（带膨胀）
 *
 * @param obstacle 障碍物对象（输入/输出）
 * @param cloud 原始点云
 * @param config OBB 配置
 * @return 是否成功
 */
template <typename PointT>
bool updateObstacleAsBox(typename Object::Ptr obstacle,
                         typename pcl::PointCloud<PointT>::Ptr cloud,
                         const ObbConfig &config) {

    if (!obstacle || !cloud) {
        return false;
    }

    // 计算 OBB
    Eigen::Vector3f obb_center, obb_size;
    std::vector<Eigen::Vector3f> obb_axes;

    if (!computeOBB<PointT>(cloud, *obstacle->inliers, obb_center, obb_size, obb_axes)) {
        ROS_WARN("[OBB] 无法计算障碍物 %s 的 OBB", obstacle->name.c_str());
        return false;
    }

    // 检查最小高度
    if (obb_size[2] < config.min_obstacle_height) {
        ROS_DEBUG("[OBB] 障碍物 %s 高度过小：%.3f < %.3f，跳过",
                  obstacle->name.c_str(), obb_size[2], config.min_obstacle_height);
        // 仍然保留，但标记
    }

    // 膨胀 OBB 尺寸（各方向均匀膨胀）
    float inflation = config.inflation_radius;
    float inflated_length = obb_size[0] + 2 * inflation;
    float inflated_width  = obb_size[1] + 2 * inflation;
    float inflated_height = obb_size[2] + 2 * inflation;

    // 更新障碍物系数：存储 OBB 信息
    // [0-2]: 中心位置
    // [3-5]: 主轴方向（最长维度方向）
    // [6]: 长度（最长维度）
    // [7]: 宽度（次长维度）
    // [8]: 高度（Z 方向）
    obstacle->coefficients->values.resize(9);
    obstacle->coefficients->values[0] = obb_center[0];           // 中心 X
    obstacle->coefficients->values[1] = obb_center[1];           // 中心 Y
    obstacle->coefficients->values[2] = obb_center[2];           // 中心 Z
    obstacle->coefficients->values[3] = obb_axes[2][0];          // 主轴 X（最长维度方向）
    obstacle->coefficients->values[4] = obb_axes[2][1];          // 主轴 Y
    obstacle->coefficients->values[5] = obb_axes[2][2];          // 主轴 Z
    obstacle->coefficients->values[6] = inflated_length;         // 长度
    obstacle->coefficients->values[7] = inflated_width;          // 宽度
    obstacle->coefficients->values[8] = inflated_height;         // 高度

    // 更新尺寸（使用膨胀后的尺寸）
    obstacle->width = inflated_length;
    obstacle->depth = inflated_width;
    obstacle->height = inflated_height;

    ROS_DEBUG("[OBB] 障碍物 %s: 中心 (%.2f, %.2f, %.2f), 尺寸：%.2f x %.2f x %.2f m (膨胀后)",
              obstacle->name.c_str(),
              obb_center[0], obb_center[1], obb_center[2],
              inflated_length, inflated_width, inflated_height);

    return true;
}

}  // namespace core
}  // namespace pcl_object_detection
