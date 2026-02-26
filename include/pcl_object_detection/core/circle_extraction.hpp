#pragma once

#include "object_base.hpp"
#include "config.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
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

// Taubin 圆拟合
bool fitCircleTaubin(const std::vector<Eigen::Vector2f>& points, Eigen::Vector3f& result) {
    if (points.size() < 5) return false;
    Eigen::Vector2d centroid(0, 0);
    for (const auto& pt : points) centroid += pt.cast<double>();
    centroid /= points.size();
    Eigen::MatrixXd X(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        double x = points[i][0] - centroid[0];
        double y = points[i][1] - centroid[1];
        X(i, 0) = x; X(i, 1) = y; X(i, 2) = x * x + y * y;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd v = svd.matrixV().col(2);
    double A = v[0], B = v[1], C = v[2];
    if (std::abs(C) < 1e-10) return false;
    double center_x = -A / (2 * C);
    double center_y = -B / (2 * C);
    double radius = std::sqrt(center_x * center_x + center_y * center_y + 1.0 / C);
    result[0] = static_cast<float>(center_x + centroid[0]);
    result[1] = static_cast<float>(center_y + centroid[1]);
    result[2] = static_cast<float>(radius);
    return true;
}

// 检查圆周覆盖度
float checkCircleCoverage(const std::vector<Eigen::Vector2f>& points, const Eigen::Vector2f& center, float radius) {
    if (points.empty() || radius <= 0) return 0;
    std::vector<float> angles;
    for (const auto& pt : points)
        angles.push_back(std::atan2(pt[1] - center[1], pt[0] - center[0]) * 180.0 / M_PI);
    std::sort(angles.begin(), angles.end());
    float max_gap = 0;
    for (size_t i = 1; i < angles.size(); ++i)
        max_gap = std::max(max_gap, angles[i] - angles[i-1]);
    max_gap = std::max(max_gap, 360.0f - (angles.back() - angles.front()));
    return 360.0f - max_gap;
}

// RANSAC 圆拟合
bool fitCircleRANSAC(const std::vector<Eigen::Vector2f>& points, 
                     Eigen::Vector3f& best_circle, std::vector<int>& best_inliers,
                     float distance_threshold, int min_inliers,
                     float radius_min, float radius_max, int max_iterations = 200) {
    if (points.size() < 3) return false;
    best_inliers.clear();
    best_circle.setZero();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);
    for (int iter = 0; iter < max_iterations; ++iter) {
        int idx1 = dis(gen), idx2 = dis(gen), idx3 = dis(gen);
        if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;
        Eigen::Vector2f p1 = points[idx1], p2 = points[idx2], p3 = points[idx3];
        float D = 2 * (p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]));
        if (std::abs(D) < 1e-10) continue;
        float Ux = ((p1[0]*p1[0] + p1[1]*p1[1]) * (p2[1] - p3[1]) +
                    (p2[0]*p2[0] + p2[1]*p2[1]) * (p3[1] - p1[1]) +
                    (p3[0]*p3[0] + p3[1]*p3[1]) * (p1[1] - p2[1])) / D;
        float Uy = ((p1[0]*p1[0] + p1[1]*p1[1]) * (p3[0] - p2[0]) +
                    (p2[0]*p2[0] + p2[1]*p2[1]) * (p1[0] - p3[0]) +
                    (p3[0]*p3[0] + p3[1]*p3[1]) * (p2[0] - p1[0])) / D;
        float radius = std::sqrt((Ux - p1[0]) * (Ux - p1[0]) + (Uy - p1[1]) * (Uy - p1[1]));
        if (radius < radius_min || radius > radius_max) continue;
        std::vector<int> inliers;
        for (size_t i = 0; i < points.size(); ++i) {
            float dist = std::sqrt((points[i][0] - Ux) * (points[i][0] - Ux) +
                                   (points[i][1] - Uy) * (points[i][1] - Uy));
            if (std::abs(dist - radius) < distance_threshold)
                inliers.push_back(i);
        }
        if (inliers.size() >= static_cast<size_t>(min_inliers) && inliers.size() > best_inliers.size()) {
            best_inliers = inliers;
            best_circle[0] = Ux; best_circle[1] = Uy; best_circle[2] = radius;
        }
    }
    return !best_inliers.empty();
}

// 提取竖直圆环（支持使用或不使用法向量）
template <typename PointT>
std::vector<Object::Ptr> extractCircles(typename pcl::PointCloud<PointT>::Ptr cloud,
                                        typename pcl::PointCloud<pcl::Normal>::Ptr normals,
                                        const CircleConfig &config,
                                        const std::set<int>& excluded_indices = {}) {
    std::vector<Object::Ptr> circles;
    if (cloud->empty()) return circles;

    typename pcl::PointCloud<pcl::Normal>::Ptr normals_to_use = normals;
    if (!normals_to_use && config.using_normal) {
        return circles;
    }

    bool use_normals = config.using_normal && normals_to_use;

    // 构建可用索引
    std::vector<int> available_indices;
    for (size_t i = 0; i < cloud->size(); ++i)
        if (excluded_indices.find(i) == excluded_indices.end())
            available_indices.push_back(i);

    if (available_indices.size() < static_cast<size_t>(config.min_inliers))
        return circles;

    auto start_total = std::chrono::high_resolution_clock::now();

    // 步骤 1: 提取竖直平面（根据是否使用法向量选择不同方法）
    typename pcl::PointIndices::Ptr all_indices(new pcl::PointIndices);
    all_indices->indices = available_indices;
    typename pcl::PointIndices::Ptr current_indices = all_indices;
    int circle_num = 1;

    while (current_indices->indices.size() >= static_cast<size_t>(config.min_inliers) && circle_num <= config.max_circles) {
        typename pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients);

        if (use_normals) {
            // 使用法向量的平面拟合
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> plane_seg;
            plane_seg.setOptimizeCoefficients(true);
            plane_seg.setModelType(pcl::SACMODEL_PLANE);
            plane_seg.setMethodType(pcl::SAC_RANSAC);
            plane_seg.setDistanceThreshold(config.plane_distance_threshold);
            plane_seg.setMaxIterations(config.ransac_iterations / 2);
            plane_seg.setIndices(current_indices);
            plane_seg.setInputCloud(cloud);
            plane_seg.setInputNormals(normals_to_use);
            plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
            plane_seg.setEpsAngle(config.plane_angle_threshold * M_PI / 180.0f);
            plane_seg.segment(*plane_inliers, *plane_coeffs);
        } else {
            // 不使用法向量的平面拟合
            pcl::SACSegmentation<PointT> plane_seg;
            plane_seg.setOptimizeCoefficients(true);
            plane_seg.setModelType(pcl::SACMODEL_PLANE);
            plane_seg.setMethodType(pcl::SAC_RANSAC);
            plane_seg.setDistanceThreshold(config.plane_distance_threshold);
            plane_seg.setMaxIterations(config.ransac_iterations);
            plane_seg.setIndices(current_indices);
            plane_seg.setInputCloud(cloud);
            plane_seg.segment(*plane_inliers, *plane_coeffs);
        }

        if (plane_inliers->indices.size() < static_cast<size_t>(config.min_inliers))
            break;

        Eigen::Vector3f plane_normal(plane_coeffs->values[0], plane_coeffs->values[1], plane_coeffs->values[2]);
        float plane_normal_z = std::abs(plane_normal[2]);

        if (plane_normal_z > config.plane_normal_z_max) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices)
                if (plane_set.find(idx) == plane_set.end())
                    new_indices->indices.push_back(idx);
            current_indices = new_indices;
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(plane_inliers);
        extract.filter(*plane_cloud);

        if (plane_cloud->size() < static_cast<size_t>(config.min_inliers)) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices)
                if (plane_set.find(idx) == plane_set.end())
                    new_indices->indices.push_back(idx);
            current_indices = new_indices;
            continue;
        }

        Eigen::Vector3f z_axis = plane_normal.normalized();
        Eigen::Vector3f x_axis = z_axis.cross(Eigen::Vector3f(0, 0, 1)).normalized();
        if (x_axis.norm() < 0.01f)
            x_axis = z_axis.cross(Eigen::Vector3f(1, 0, 0)).normalized();
        Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix3f rotation;
        rotation.col(0) = x_axis; rotation.col(1) = y_axis; rotation.col(2) = z_axis;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_cloud, centroid);

        typename pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation.transpose();
        transform.block<3, 1>(0, 3) = -rotation.transpose() * centroid.head<3>();
        pcl::transformPointCloud(*plane_cloud, *transformed_cloud, transform);

        std::vector<Eigen::Vector2f> points_2d;
        for (const auto& pt : *transformed_cloud)
            points_2d.push_back(Eigen::Vector2f(pt.x, pt.y));

        Eigen::Vector3f circle_params;
        std::vector<int> circle_inliers_2d;
        if (!fitCircleRANSAC(points_2d, circle_params, circle_inliers_2d,
                             config.distance_threshold, config.min_inliers,
                             config.radius_min, config.radius_max, config.ransac_iterations)) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices)
                if (plane_set.find(idx) == plane_set.end())
                    new_indices->indices.push_back(idx);
            current_indices = new_indices;
            circle_num++;
            continue;
        }

        float radius = circle_params[2];

        float coverage = checkCircleCoverage(points_2d, circle_params.head<2>(), radius);
        if (coverage < config.min_coverage_angle) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices)
                if (plane_set.find(idx) == plane_set.end())
                    new_indices->indices.push_back(idx);
            current_indices = new_indices;
            circle_num++;
            continue;
        }

        Eigen::Vector3f center_2d(circle_params[0], circle_params[1], 0);
        Eigen::Vector3f center_world = rotation * center_2d + centroid.head<3>();

        typename pcl::PointIndices::Ptr circle_inliers(new pcl::PointIndices);
        for (int idx_2d : circle_inliers_2d)
            if (idx_2d >= 0 && idx_2d < static_cast<int>(plane_inliers->indices.size()))
                circle_inliers->indices.push_back(plane_inliers->indices[idx_2d]);

        if (circle_inliers->indices.size() < static_cast<size_t>(config.min_inliers)) {
            std::set<int> plane_set(plane_inliers->indices.begin(), plane_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices)
                if (plane_set.find(idx) == plane_set.end())
                    new_indices->indices.push_back(idx);
            current_indices = new_indices;
            circle_num++;
            continue;
        }

        typename pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        coefficients->values.resize(7);
        coefficients->values[0] = center_world[0];
        coefficients->values[1] = center_world[1];
        coefficients->values[2] = center_world[2];
        coefficients->values[3] = plane_normal[0];
        coefficients->values[4] = plane_normal[1];
        coefficients->values[5] = plane_normal[2];
        coefficients->values[6] = radius;

        auto start_time = std::chrono::high_resolution_clock::now();
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        float width = 2 * radius;
        auto circle = Object::createCircle("circle_" + std::to_string(circle_num), circle_inliers,
                                           coefficients, elapsed, width, width, 0.01f);
        circles.push_back(circle);

        std::set<int> circle_set(circle_inliers->indices.begin(), circle_inliers->indices.end());
        typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
        for (int idx : current_indices->indices)
            if (circle_set.find(idx) == circle_set.end())
                new_indices->indices.push_back(idx);
        current_indices = new_indices;
        circle_num++;
    }

    return circles;
}

}  // namespace core
}  // namespace pcl_object_detection
