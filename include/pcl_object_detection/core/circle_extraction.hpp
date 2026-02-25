#pragma once

#include "config.hpp"
#include "object_base.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <random>
#include <set>
#include <Eigen/Geometry>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

// 2D 圆拟合（Taubin 方法）
bool fitCircleTaubin(const std::vector<Eigen::Vector2f>& points, Eigen::Vector3f& result) {
    if (points.size() < 5) {
        return false;
    }

    Eigen::Vector2d centroid(0, 0);
    for (const auto& pt : points) {
        centroid += pt.cast<double>();
    }
    centroid /= points.size();

    Eigen::MatrixXd X(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        double x = points[i][0] - centroid[0];
        double y = points[i][1] - centroid[1];
        X(i, 0) = x;
        X(i, 1) = y;
        X(i, 2) = x * x + y * y;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd v = svd.matrixV().col(2);

    double A = v[0];
    double B = v[1];
    double C = v[2];

    if (std::abs(C) < 1e-10) {
        return false;
    }

    double center_x = -A / (2 * C);
    double center_y = -B / (2 * C);
    double radius = std::sqrt(center_x * center_x + center_y * center_y + 1.0 / C);

    result[0] = static_cast<float>(center_x + centroid[0]);
    result[1] = static_cast<float>(center_y + centroid[1]);
    result[2] = static_cast<float>(radius);

    return true;
}

// 检查点是否覆盖圆周
float checkCircleCoverage(const std::vector<Eigen::Vector2f>& points, const Eigen::Vector2f& center, float radius) {
    if (points.empty() || radius <= 0) return 0;

    std::vector<float> angles;
    angles.reserve(points.size());
    for (const auto& pt : points) {
        float angle = std::atan2(pt[1] - center[1], pt[0] - center[0]) * 180.0 / M_PI;
        angles.push_back(angle);
    }

    std::sort(angles.begin(), angles.end());
    float max_gap = 0;
    for (size_t i = 1; i < angles.size(); ++i) {
        float gap = angles[i] - angles[i-1];
        if (gap > max_gap) max_gap = gap;
    }
    float wrap_gap = 360.0 - (angles.back() - angles.front());
    if (wrap_gap > max_gap) max_gap = wrap_gap;

    return 360.0 - max_gap;
}

// 在 2D 点云中使用 RANSAC 拟合圆
bool fitCircleRANSAC(const std::vector<Eigen::Vector2f>& points, 
                     Eigen::Vector3f& best_circle,
                     std::vector<int>& best_inliers,
                     float distance_threshold,
                     int min_inliers,
                     float radius_min,
                     float radius_max,
                     int max_iterations = 500) {
    
    if (points.size() < 3) return false;

    best_inliers.clear();
    best_circle.setZero();
    float best_score = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选 3 个点
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        int idx3 = dis(gen);
        
        if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;

        // 计算三角形外接圆
        Eigen::Vector2f p1 = points[idx1];
        Eigen::Vector2f p2 = points[idx2];
        Eigen::Vector2f p3 = points[idx3];

        float D = 2 * (p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]));
        if (std::abs(D) < 1e-10) continue;

        float Ux = ((p1[0]*p1[0] + p1[1]*p1[1]) * (p2[1] - p3[1]) +
                    (p2[0]*p2[0] + p2[1]*p2[1]) * (p3[1] - p1[1]) +
                    (p3[0]*p3[0] + p3[1]*p3[1]) * (p1[1] - p2[1])) / D;
        float Uy = ((p1[0]*p1[0] + p1[1]*p1[1]) * (p3[0] - p2[0]) +
                    (p2[0]*p2[0] + p2[1]*p2[1]) * (p1[0] - p3[0]) +
                    (p3[0]*p3[0] + p3[1]*p3[1]) * (p2[0] - p1[0])) / D;

        float radius = std::sqrt((Ux - p1[0]) * (Ux - p1[0]) + (Uy - p1[1]) * (Uy - p1[1]));

        // 检查半径
        if (radius < radius_min || radius > radius_max) continue;

        // 统计内点
        std::vector<int> inliers;
        inliers.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            float dist = std::sqrt((points[i][0] - Ux) * (points[i][0] - Ux) +
                                   (points[i][1] - Uy) * (points[i][1] - Uy));
            if (std::abs(dist - radius) < distance_threshold) {
                inliers.push_back(i);
            }
        }

        // 更新最佳结果
        if (inliers.size() >= static_cast<size_t>(min_inliers) && inliers.size() > best_inliers.size()) {
            best_inliers = inliers;
            best_circle[0] = Ux;
            best_circle[1] = Uy;
            best_circle[2] = radius;
        }
    }

    return !best_inliers.empty();
}

// 提取竖直圆环（使用平面投影 + RANSAC 2D 圆拟合）
template <typename PointT>
std::vector<Object::Ptr> extractCircles(typename pcl::PointCloud<PointT>::Ptr cloud,
                                        typename pcl::PointCloud<pcl::Normal>::Ptr normals,
                                        const CircleConfig &config,
                                        const std::set<int>& excluded_indices = {}) {

    std::vector<Object::Ptr> circles;

    if (cloud->empty()) {
        return circles;
    }

    // 检查法向量
    typename pcl::PointCloud<pcl::Normal>::Ptr normals_to_use = normals;
    if (!normals_to_use && config.using_normal) {
        return circles;
    }

    // 构建可用索引
    std::vector<int> available_indices;
    available_indices.reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (excluded_indices.find(i) == excluded_indices.end()) {
            available_indices.push_back(i);
        }
    }

    if (available_indices.size() < static_cast<size_t>(config.min_inliers)) {
        return circles;
    }

    auto start_total = std::chrono::high_resolution_clock::now();

    // 步骤 1: 提取竖直平面
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> plane_seg;
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(config.plane_distance_threshold);
    plane_seg.setMaxIterations(config.ransac_iterations);

    if (config.using_normal && normals_to_use) {
        plane_seg.setInputNormals(normals_to_use);
        plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
        plane_seg.setEpsAngle(config.plane_angle_threshold * M_PI / 180.0f);
    }

    typename pcl::PointIndices::Ptr all_indices(new pcl::PointIndices);
    all_indices->indices = available_indices;

    typename pcl::PointIndices::Ptr current_indices = all_indices;
    int circle_num = 1;

    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers) && circle_num <= config.max_circles) {
        auto start_iter = std::chrono::high_resolution_clock::now();
        
        plane_seg.setIndices(current_indices);
        plane_seg.setInputCloud(cloud);

        typename pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients);

        plane_seg.segment(*plane_inliers, *plane_coeffs);


        if (plane_inliers->indices.size() < static_cast<size_t>(config.min_inliers)) {
            break;
        }

        // 验证平面法向量是否水平
        Eigen::Vector3f plane_normal(plane_coeffs->values[0], plane_coeffs->values[1], plane_coeffs->values[2]);
        float plane_normal_z = std::abs(plane_normal[2]);


        if (plane_normal_z > config.plane_normal_z_max) {
            // 移除平面点
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (plane_set.find(idx) == plane_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 提取平面点云
        typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(plane_inliers);
        extract.filter(*plane_cloud);

        if (plane_cloud->size() < static_cast<size_t>(config.min_inliers)) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (plane_set.find(idx) == plane_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            continue;
        }

        // 构建局部坐标系
        Eigen::Vector3f z_axis = plane_normal.normalized();
        Eigen::Vector3f x_axis = z_axis.cross(Eigen::Vector3f(0, 0, 1)).normalized();
        if (x_axis.norm() < 0.01f) {
            x_axis = z_axis.cross(Eigen::Vector3f(1, 0, 0)).normalized();
        }
        Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix3f rotation;
        rotation.col(0) = x_axis;
        rotation.col(1) = y_axis;
        rotation.col(2) = z_axis;

        // 计算平面中心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_cloud, centroid);

        // 变换到平面坐标系
        typename pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation.transpose();
        transform.block<3, 1>(0, 3) = -rotation.transpose() * centroid.head<3>();

        pcl::transformPointCloud(*plane_cloud, *transformed_cloud, transform);

        // 创建 2D 点集
        std::vector<Eigen::Vector2f> points_2d;
        points_2d.reserve(transformed_cloud->size());
        for (const auto& pt : *transformed_cloud) {
            points_2d.push_back(Eigen::Vector2f(pt.x, pt.y));
        }

        // RANSAC 圆拟合
        Eigen::Vector3f circle_params;
        std::vector<int> circle_inliers_2d;
        
        
        if (!fitCircleRANSAC(points_2d, circle_params, circle_inliers_2d,
                             config.distance_threshold, config.min_inliers,
                             config.radius_min, config.radius_max, config.ransac_iterations)) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (plane_set.find(idx) == plane_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            circle_num++;
            continue;
        }

        float radius = circle_params[2];

        // 检查圆周覆盖度
        float coverage = checkCircleCoverage(points_2d, circle_params.head<2>(), radius);

        if (coverage < config.min_coverage_angle) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (plane_set.find(idx) == plane_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            circle_num++;
            continue;
        }

        // 将圆心转换回世界坐标系
        Eigen::Vector3f center_2d(circle_params[0], circle_params[1], 0);
        Eigen::Vector3f center_world = rotation * center_2d + centroid.head<3>();

        // 创建 3D 内点索引
        typename pcl::PointIndices::Ptr circle_inliers(new pcl::PointIndices);
        for (int idx_2d : circle_inliers_2d) {
            if (idx_2d >= 0 && idx_2d < static_cast<int>(plane_inliers->indices.size())) {
                circle_inliers->indices.push_back(plane_inliers->indices[idx_2d]);
            }
        }

        if (circle_inliers->indices.size() < static_cast<size_t>(config.min_inliers)) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (plane_set.find(idx) == plane_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;
            circle_num++;
            continue;
        }

        // 创建系数
        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        coefficients->values.resize(7);
        coefficients->values[0] = center_world[0];
        coefficients->values[1] = center_world[1];
        coefficients->values[2] = center_world[2];
        coefficients->values[3] = plane_normal[0];
        coefficients->values[4] = plane_normal[1];
        coefficients->values[5] = plane_normal[2];
        coefficients->values[6] = radius;

        // 计算当前迭代耗时
        auto end_iter = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(end_iter - start_iter).count();

        // 计算尺寸
        float width = 2 * radius;
        float height = width;
        float depth = 0.01f;

        auto circle = Object::createCircle("circle_" + std::to_string(circle_num), circle_inliers,
                                           coefficients, elapsed, width, height, depth);
        circles.push_back(circle);


        // 从当前索引中移除这些点
        std::set<int> circle_set(circle_inliers->indices.begin(), circle_inliers->indices.end());
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (int idx : current_indices->indices) {
            if (circle_set.find(idx) == circle_set.end()) {
                new_indices->indices.push_back(idx);
            }
        }
        current_indices = new_indices;
        circle_num++;
    }

    auto end_total = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration<double, std::milli>(end_total - start_total).count();
    return circles;
}

}  // namespace core
}  // namespace pcl_object_detection
