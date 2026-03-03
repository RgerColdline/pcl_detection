#pragma once

#include "object_base.hpp"
#include "object_factory.hpp"
#include "config.hpp"
#include "extractors_base.hpp"
#include "timer.hpp"
#include "logger_limiter.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
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

/**
 * @brief 2D 点集的最小外接矩形拟合
 * @param points 2D 点集
 * @param center 矩形中心
 * @param length 长边长度
 * @param width 短边长度
 * @param angle 旋转角度（度）
 * @return 是否成功
 */
bool fitMinAreaRect(const std::vector<Eigen::Vector2f>& points,
                    Eigen::Vector2f& center, float& length, float& width, float& angle) {
    if (points.size() < 4) return false;

    // 计算质心
    Eigen::Vector2f centroid(0, 0);
    for (const auto& pt : points) centroid += pt;
    centroid /= points.size();

    // 计算协方差矩阵
    float xx = 0, xy = 0, yy = 0;
    for (const auto& pt : points) {
        float dx = pt[0] - centroid[0];
        float dy = pt[1] - centroid[1];
        xx += dx * dx;
        xy += dx * dy;
        yy += dy * dy;
    }
    xx /= points.size();
    xy /= points.size();
    yy /= points.size();

    // 特征值分解求主方向
    float trace = xx + yy;
    float det = xx * yy - xy * xy;
    float disc = std::sqrt(std::max(0.0f, trace * trace - 4 * det));
    float lambda1 = (trace + disc) / 2;
    float lambda2 = (trace - disc) / 2;

    // 主方向角度
    float theta = std::atan2(xy, xx - lambda1) * 180.0f / M_PI;
    if (theta < 0) theta += 180;
    if (theta > 90) theta -= 180;

    // 旋转到主方向
    float cos_t = std::cos(theta * M_PI / 180.0f);
    float sin_t = std::sin(theta * M_PI / 180.0f);

    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;

    for (const auto& pt : points) {
        float dx = pt[0] - centroid[0];
        float dy = pt[1] - centroid[1];
        float rx = dx * cos_t + dy * sin_t;
        float ry = -dx * sin_t + dy * cos_t;
        min_x = std::min(min_x, rx);
        max_x = std::max(max_x, rx);
        min_y = std::min(min_y, ry);
        max_y = std::max(max_y, ry);
    }

    center = centroid;
    length = max_x - min_x;
    width = max_y - min_y;
    angle = theta;

    // 确保 length >= width
    if (length < width) {
        std::swap(length, width);
        angle += 90;
        if (angle > 90) angle -= 180;
    }

    return true;
}

/**
 * @brief 检查矩形覆盖度（点是否分布在四条边附近）
 * 优化版：要求点必须分布在至少 3 条边上
 */
float checkRectangleCoverage(const std::vector<Eigen::Vector2f>& points,
                             const Eigen::Vector2f& center,
                             float length, float width, float angle,
                             float tolerance = 0.1f) {
    if (points.empty() || length <= 0 || width <= 0) return 0;

    float cos_t = std::cos(-angle * M_PI / 180.0f);
    float sin_t = std::sin(-angle * M_PI / 180.0f);

    float half_l = length / 2.0f;
    float half_w = width / 2.0f;

    int edge_points = 0;
    float edge_tolerance = tolerance * std::min(length, width);
    
    // 统计每条边上的点数
    int top_count = 0, bottom_count = 0, left_count = 0, right_count = 0;

    for (const auto& pt : points) {
        float dx = pt[0] - center[0];
        float dy = pt[1] - center[1];
        float rx = dx * cos_t + dy * sin_t;
        float ry = -dx * sin_t + dy * cos_t;

        bool on_top = (std::abs(ry - half_w) < edge_tolerance) && (std::abs(rx) < half_l);
        bool on_bottom = (std::abs(ry + half_w) < edge_tolerance) && (std::abs(rx) < half_l);
        bool on_left = (std::abs(rx + half_l) < edge_tolerance) && (std::abs(ry) < half_w);
        bool on_right = (std::abs(rx - half_l) < edge_tolerance) && (std::abs(ry) < half_w);

        if (on_top) { edge_points++; top_count++; }
        else if (on_bottom) { edge_points++; bottom_count++; }
        else if (on_left) { edge_points++; left_count++; }
        else if (on_right) { edge_points++; right_count++; }
    }

    // 检查点是否分布在至少 3 条边上（减少误检）
    int edges_with_points = 0;
    if (top_count > 3) edges_with_points++;
    if (bottom_count > 3) edges_with_points++;
    if (left_count > 3) edges_with_points++;
    if (right_count > 3) edges_with_points++;
    
    if (edges_with_points < 3) return 0;  // 点只分布在 1-2 条边上，不是有效矩形

    return static_cast<float>(edge_points) / points.size();
}

/**
 * @brief RANSAC 矩形拟合
 */
bool fitRectangleRANSAC(const std::vector<Eigen::Vector2f>& points,
                        Eigen::Vector3f& best_rect,  // [center_x, center_y, angle]
                        float& best_length, float& best_width,
                        std::vector<int>& best_inliers,
                        float distance_threshold, int min_inliers,
                        float length_min, float length_max,
                        float width_min, float width_max,
                        float coverage_threshold, int max_iterations = 300) {
    if (points.size() < 10) return false;

    best_inliers.clear();
    best_rect.setZero();
    best_length = 0;
    best_width = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    int best_inlier_count = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机采样 6 个点来拟合矩形
        std::set<int> sampled;
        while (sampled.size() < 6) {
            sampled.insert(dis(gen));
        }

        std::vector<Eigen::Vector2f> sample_points;
        for (int idx : sampled) {
            sample_points.push_back(points[idx]);
        }

        // 拟合最小外接矩形
        Eigen::Vector2f center;
        float length, width, angle;
        if (!fitMinAreaRect(sample_points, center, length, width, angle)) continue;

        // 检查尺寸约束
        if (length < length_min || length > length_max) continue;
        if (width < width_min || width > width_max) continue;
        
        // 检查长宽比（过滤太细长的矩形，减少误检）
        float aspect_ratio = length / (width + 0.01f);
        if (aspect_ratio > 4.0f) continue;  // 长宽比不超过 4:1

        // 计算内点
        std::vector<int> inliers;
        float cos_t = std::cos(-angle * M_PI / 180.0f);
        float sin_t = std::sin(-angle * M_PI / 180.0f);
        float half_l = length / 2.0f;
        float half_w = width / 2.0f;
        float edge_tol = distance_threshold;

        for (size_t i = 0; i < points.size(); ++i) {
            float dx = points[i][0] - center[0];
            float dy = points[i][1] - center[1];
            float rx = dx * cos_t + dy * sin_t;
            float ry = -dx * sin_t + dy * cos_t;

            // 检查是否在四条边附近
            float dist_to_edge = FLT_MAX;
            dist_to_edge = std::min(dist_to_edge, std::abs(ry - half_w));  // 上边
            dist_to_edge = std::min(dist_to_edge, std::abs(ry + half_w));  // 下边
            dist_to_edge = std::min(dist_to_edge, std::abs(rx + half_l));  // 左边
            dist_to_edge = std::min(dist_to_edge, std::abs(rx - half_l));  // 右边

            // 同时检查是否在边的延长线范围内
            bool within_length = std::abs(rx) <= half_l + edge_tol;
            bool within_width = std::abs(ry) <= half_w + edge_tol;

            if (dist_to_edge < edge_tol && (within_length || within_width)) {
                inliers.push_back(i);
            }
        }

        // 检查内点数量
        if (static_cast<int>(inliers.size()) < min_inliers) continue;
        if (static_cast<int>(inliers.size()) <= best_inlier_count) continue;

        // 检查覆盖度
        float coverage = checkRectangleCoverage(points, center, length, width, angle);
        if (coverage < coverage_threshold) continue;

        // 更新最佳结果
        best_inlier_count = inliers.size();
        best_inliers = inliers;
        best_rect[0] = center[0];
        best_rect[1] = center[1];
        best_rect[2] = angle;
        best_length = length;
        best_width = width;
    }

    return !best_inliers.empty();
}

/**
 * @brief 矩形框提取器
 */
template <typename PointT>
class RectangleExtractor : public IObjectExtractor<PointT>
{
  public:
    using PointCloudPtrT = typename pcl::PointCloud<PointT>::Ptr;
    using NormalCloudPtrT = typename pcl::PointCloud<pcl::Normal>::Ptr;

    RectangleExtractor(const RectangleConfig &config) : config_(config) {}

    std::vector<typename Object::Ptr> extract(
        PointCloudPtrT cloud,
        NormalCloudPtrT normals,
        const std::set<int>& excluded,
        PointCloudPtrT temp_cloud1 = nullptr,
        PointCloudPtrT temp_cloud2 = nullptr) override
    {
        std::vector<typename Object::Ptr> rectangles;

        if (!config_.enable || cloud->empty()) {
            return rectangles;
        }

        Timer timer;
        int rect_num = 0;

        // 创建可用索引（排除已使用的点）
        auto available_indices = std::make_shared<std::vector<int>>();
        for (size_t i = 0; i < cloud->size(); ++i) {
            if (excluded.find(i) == excluded.end()) {
                available_indices->push_back(i);
            }
        }

        ROS_DEBUG("[Rectangle] 可用点数：%lu", available_indices->size());

        if (available_indices->empty()) return rectangles;

        // 使用 pcl::PointIndices 类型
        typename pcl::PointIndices::Ptr all_indices(new pcl::PointIndices);
        all_indices->indices = *available_indices;
        typename pcl::PointIndices::Ptr current_indices = all_indices;

        // 初始化临时点云
        if (!temp_cloud1) temp_cloud1 = PointCloudPtrT(new pcl::PointCloud<PointT>);
        if (!temp_cloud2) temp_cloud2 = PointCloudPtrT(new pcl::PointCloud<PointT>);

        int plane_attempts = 0;
        int projection_attempts = 0;
        int ransac_attempts = 0;

        while (current_indices->indices.size() >= static_cast<size_t>(config_.min_inliers)) {
            // 使用平面分割提取平面（直接对原始点云操作，使用 setIndices 指定索引）
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> plane_seg;
            plane_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
            plane_seg.setMethodType(pcl::SAC_RANSAC);
            plane_seg.setDistanceThreshold(config_.distance_threshold);
            plane_seg.setNormalDistanceWeight(0.1);
            plane_seg.setMaxIterations(1000);
            plane_seg.setOptimizeCoefficients(true);

            typename pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
            typename pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients);
            plane_seg.setInputCloud(cloud);
            plane_seg.setInputNormals(normals);
            plane_seg.setIndices(current_indices);
            plane_seg.segment(*plane_inliers, *plane_coeffs);

            plane_attempts++;

            if (plane_inliers->indices.size() < static_cast<size_t>(config_.min_inliers)) {
                ROS_DEBUG("[Rectangle] 平面内点不足：%lu < %d", plane_inliers->indices.size(), config_.min_inliers);
                break;
            }

            // 检查平面法向量（应该是竖直平面）
            Eigen::Vector3f plane_normal(plane_coeffs->values[0],
                                         plane_coeffs->values[1],
                                         plane_coeffs->values[2]);
            if (std::abs(plane_normal[2]) > config_.plane_normal_z_max) {
                // 平面太水平，跳过
                ROS_DEBUG("[Rectangle] 平面法向量 Z 分量过大：%.3f > %.3f", std::abs(plane_normal[2]), config_.plane_normal_z_max);
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

            // 提取平面点云（用于 2D 投影）
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(plane_inliers);
            extract.filter(*temp_cloud2);

            projection_attempts++;

            if (temp_cloud2->size() < static_cast<size_t>(config_.min_inliers)) {
                ROS_DEBUG("[Rectangle] 投影后点数不足：%lu < %d", temp_cloud2->size(), config_.min_inliers);
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

            // 将平面点云投影到 2D
            std::vector<Eigen::Vector2f> points_2d;
            projectTo2D(*temp_cloud2, plane_normal, points_2d);

            if (points_2d.size() < 10) {
                ROS_DEBUG("[Rectangle] 2D 投影点数太少：%lu", points_2d.size());
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

            // RANSAC 拟合矩形
            Eigen::Vector3f rect_params;  // [center_x, center_y, angle]
            float rect_length, rect_width;
            std::vector<int> rect_inliers_2d;

            bool found = fitRectangleRANSAC(
                points_2d, rect_params, rect_length, rect_width, rect_inliers_2d,
                config_.distance_threshold, config_.min_inliers,
                config_.length_min, config_.length_max,
                config_.width_min, config_.width_max,
                config_.coverage_threshold, config_.ransac_iterations
            );

            if (!found) {
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

            // 将 2D 内点映射回 3D 索引
            typename pcl::PointIndices::Ptr rect_inliers(new pcl::PointIndices);
            for (int idx_2d : rect_inliers_2d) {
                if (idx_2d >= 0 && idx_2d < static_cast<int>(plane_inliers->indices.size())) {
                    rect_inliers->indices.push_back(plane_inliers->indices[idx_2d]);
                }
            }

            if (rect_inliers->indices.size() < static_cast<size_t>(config_.min_inliers)) {
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

            // 计算 3D 中心
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*temp_cloud2, centroid);

            // 创建矩形对象
            typename pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
            coeffs->values.resize(9);
            coeffs->values[0] = centroid[0];  // 中心 x
            coeffs->values[1] = centroid[1];  // 中心 y
            coeffs->values[2] = centroid[2];  // 中心 z
            coeffs->values[3] = plane_coeffs->values[0];  // 法向量 x
            coeffs->values[4] = plane_coeffs->values[1];  // 法向量 y
            coeffs->values[5] = plane_coeffs->values[2];  // 法向量 z
            coeffs->values[6] = rect_length;  // 长边
            coeffs->values[7] = rect_width;   // 短边
            coeffs->values[8] = rect_params[2];  // 旋转角度

            auto rect = ObjectFactory::createRectangle(
                "rectangle_" + std::to_string(++rect_num),
                rect_inliers, coeffs, timer.elapsed(),
                rect_length, rect_width, 0.05f,  // depth 设为估计的边框厚度
                config_.using_normal
            );

            rectangles.push_back(rect);

            // 从候选索引中移除已检测的点
            std::set<int> rect_set(rect_inliers->indices.begin(), rect_inliers->indices.end());
            typename pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
            for (int idx : current_indices->indices) {
                if (rect_set.find(idx) == rect_set.end()) {
                    new_indices->indices.push_back(idx);
                }
            }
            current_indices = new_indices;

            if (rect_num >= config_.max_rectangles) break;
        }

        // 输出检测结果（每 N 帧输出一次）
        if (LOG_SHOULD()) {
            ROS_INFO("[Rectangle] 检测完成：找到 %d 个矩形 (平面尝试=%d, 投影尝试=%d, RANSAC 尝试=%d)",
                     rect_num, plane_attempts, projection_attempts, ransac_attempts);
        }

        return rectangles;
    }

  private:
    /**
     * @brief 将 3D 点云投影到 2D 平面
     */
    void projectTo2D(const pcl::PointCloud<PointT> &cloud, const Eigen::Vector3f &normal,
                     std::vector<Eigen::Vector2f> &points_2d) {
        points_2d.clear();

        // 构建旋转矩阵，将法向量旋转到 Z 轴
        Eigen::Vector3f z_axis(0, 0, 1);
        Eigen::Vector3f axis = normal.cross(z_axis);
        float sin_a = axis.norm();
        float cos_a = normal.dot(z_axis);

        if (sin_a < 1e-6) {
            // 法向量已经接近 Z 轴，直接投影
            for (const auto &pt : cloud) {
                points_2d.emplace_back(pt.x, pt.y);
            }
            return;
        }

        axis.normalize();
        Eigen::Matrix3f R = Eigen::AngleAxisf(std::asin(sin_a), axis).toRotationMatrix();

        for (const auto &pt : cloud) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f rotated = R * p;
            points_2d.emplace_back(rotated[0], rotated[1]);
        }
    }

    RectangleConfig config_;
};

}  // namespace core
}  // namespace pcl_object_detection
