#pragma once

#include "obstacle_pipeline_config.hpp"
#include "object_base.hpp"
#include "object_factory.hpp"
#include "timer.hpp"
#include "logger_limiter.hpp"
#include "centroid_utils.hpp"

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <set>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 提取点云的边界点（基于角度分布）
 */
template <typename PointT>
std::vector<int> extractBoundaryPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float angle_threshold = 0.6f * M_PI)  // 约 108 度
{
    std::vector<int> boundary_indices;
    
    if (!cloud || cloud->size() < 5) return boundary_indices;
    
    // 计算质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    
    // 构建 KD 树
    pcl::search::KdTree<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    int k = std::min(15, static_cast<int>(cloud->size()) - 1);
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        kdtree.nearestKSearch(i, k, k_indices, k_distances);
        
        // 计算邻域点相对于质心的角度分布
        std::vector<float> angles;
        angles.reserve(k_indices.size());
        
        for (int idx : k_indices) {
            if (idx == static_cast<int>(i)) continue;
            
            const auto& pt = cloud->points[idx];
            float dx = pt.x - centroid[0];
            float dy = pt.y - centroid[1];
            float angle = std::atan2(dy, dx);
            angles.push_back(angle);
        }
        
        if (angles.size() < 3) continue;
        
        // 排序角度
        std::sort(angles.begin(), angles.end());
        
        // 检查最大角度间隔
        float max_gap = 0;
        for (size_t j = 1; j < angles.size(); ++j) {
            float gap = angles[j] - angles[j-1];
            max_gap = std::max(max_gap, gap);
        }
        // 检查首尾间隔
        float wrap_gap = 2*M_PI + angles[0] - angles.back();
        max_gap = std::max(max_gap, wrap_gap);
        
        // 如果最大间隔超过阈值，认为是边界点
        if (max_gap > angle_threshold) {
            boundary_indices.push_back(i);
        }
    }
    
    return boundary_indices;
}

/**
 * @brief 对边界点进行排序（按角度）
 */
std::vector<Eigen::Vector2f> sortBoundaryPoints(
    const std::vector<Eigen::Vector2f>& points,
    const Eigen::Vector2f& center)
{
    std::vector<std::pair<float, Eigen::Vector2f>> sorted;
    sorted.reserve(points.size());
    
    for (const auto& pt : points) {
        float angle = std::atan2(pt[1] - center[1], pt[0] - center[0]);
        sorted.emplace_back(angle, pt);
    }
    
    std::sort(sorted.begin(), sorted.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    std::vector<Eigen::Vector2f> result;
    result.reserve(points.size());
    for (const auto& p : sorted) {
        result.push_back(p.second);
    }
    
    return result;
}

/**
 * @brief 从排序的边界点检测角点
 */
std::vector<int> detectCornersFromBoundary(
    const std::vector<Eigen::Vector2f>& sorted_points,
    float angle_threshold = 0.5f * M_PI)  // 90 度
{
    std::vector<int> corner_indices;
    
    if (sorted_points.size() < 5) return corner_indices;
    
    int window = 3;
    
    for (size_t i = 0; i < sorted_points.size(); ++i) {
        int prev_idx = (i + sorted_points.size() - window) % sorted_points.size();
        int next_idx = (i + window) % sorted_points.size();
        
        const auto& prev = sorted_points[prev_idx];
        const auto& curr = sorted_points[i];
        const auto& next = sorted_points[next_idx];
        
        // 计算向量
        Eigen::Vector2f v1 = prev - curr;
        Eigen::Vector2f v2 = next - curr;
        
        // 计算夹角
        float dot = v1.dot(v2);
        float norm1 = v1.norm();
        float norm2 = v2.norm();
        
        if (norm1 < 0.01f || norm2 < 0.01f) continue;
        
        float cos_angle = dot / (norm1 * norm2);
        float angle = std::acos(std::max(-1.0f, std::min(1.0f, cos_angle)));
        
        // 角度小于阈值（锐角或直角）认为是角点
        if (angle < angle_threshold) {
            corner_indices.push_back(i);
        }
    }
    
    return corner_indices;
}

/**
 * @brief 从角点拟合矩形（4 个角点确定矩形）
 */
bool fitRectangleFromCorners(
    const std::vector<Eigen::Vector2f>& sorted_boundary,
    const std::vector<int>& corner_indices,
    Eigen::Vector2f& center,
    float& length,
    float& width,
    float& angle)
{
    if (corner_indices.size() < 3) return false;
    
    // 取前 4 个角点
    std::vector<Eigen::Vector2f> corners;
    for (size_t i = 0; i < std::min(size_t(4), corner_indices.size()); ++i) {
        corners.push_back(sorted_boundary[corner_indices[i]]);
    }
    
    if (corners.size() < 3) return false;
    
    // 计算角点的质心
    center.setZero();
    for (const auto& c : corners) {
        center += c;
    }
    center /= corners.size();
    
    // 使用 PCA 计算主方向和尺寸
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& c : corners) {
        Eigen::Vector2f diff = c - center;
        cov += diff * diff.transpose();
    }
    cov /= corners.size();
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    Eigen::Vector2f eigenvalues = solver.eigenvalues();
    Eigen::Matrix2f eigenvectors = solver.eigenvectors();
    
    // 计算每个角点在主方向上的投影
    float min_proj1 = FLT_MAX, max_proj1 = -FLT_MAX;
    float min_proj2 = FLT_MAX, max_proj2 = -FLT_MAX;
    
    Eigen::Vector2f dir1 = eigenvectors.col(1).normalized();
    Eigen::Vector2f dir2 = eigenvectors.col(0).normalized();
    
    for (const auto& c : corners) {
        Eigen::Vector2f diff = c - center;
        float proj1 = diff.dot(dir1);
        float proj2 = diff.dot(dir2);
        min_proj1 = std::min(min_proj1, proj1);
        max_proj1 = std::max(max_proj1, proj1);
        min_proj2 = std::min(min_proj2, proj2);
        max_proj2 = std::max(max_proj2, proj2);
    }
    
    length = max_proj1 - min_proj1;
    width = max_proj2 - min_proj2;
    
    // 确保 length >= width
    if (length < width) {
        std::swap(length, width);
        std::swap(dir1, dir2);
    }
    
    // 计算角度
    angle = std::atan2(dir1[1], dir1[0]) * 180.0f / M_PI;
    if (angle < 0) angle += 180;
    if (angle > 90) angle -= 180;
    
    return true;
}

/**
 * @brief 验证矩形是否有效
 */
bool validateRectangle(
    float length,
    float width,
    const RectangleExtractionConfig& config)
{
    // 检查尺寸约束
    if (length < config.length_min || length > config.length_max) return false;
    if (width < config.width_min || width > config.width_max) return false;
    
    // 检查长宽比
    float aspect_ratio = length / (width + 0.01f);
    if (aspect_ratio > config.aspect_ratio_max) return false;
    
    return true;
}

/**
 * @brief 基于聚类的方环提取器（PCA 投影 + 边界提取 + 角点检测）
 */
template <typename PointT>
class RectangleFromClusterExtractor
{
public:
    using PointCloudPtrT = typename pcl::PointCloud<PointT>::Ptr;

    explicit RectangleFromClusterExtractor(const RectangleExtractionConfig& config)
        : config_(config) {}

    /**
     * @brief 从聚类中提取方环
     * @param clusters 输入聚类
     * @param cloud 原始点云
     * @param used_indices 输出已使用的点索引
     * @return 检测到的方环对象
     */
    std::vector<Object::Ptr> extract(
        const std::vector<typename pcl::PointIndices::Ptr>& clusters,
        PointCloudPtrT cloud,
        std::set<int>& used_indices)
    {
        std::vector<Object::Ptr> rectangles;

        if (!config_.enable || clusters.empty()) {
            ROS_INFO("[RectFromCluster] 方环检测被跳过：enable=%d, clusters=%lu",
                     config_.enable ? 1 : 0, clusters.size());
            return rectangles;
        }

        Timer timer;
        int rect_num = 0;
        int total_clusters = clusters.size();
        int valid_clusters = 0;
        int boundary_fail = 0;      // 边界点提取失败
        int corner_fail = 0;        // 角点检测失败
        int fit_fail = 0;           // 矩形拟合失败
        int validate_fail = 0;      // 矩形验证失败
        
        // 存储所有聚类的尺寸分布（用于统计）
        std::vector<std::pair<float, float>> rectangle_sizes_;

        ROS_INFO("[RectFromCluster] 开始处理 %d 个聚类", total_clusters);

        for (const auto& cluster : clusters) {
            if (rect_num >= config_.max_rectangles) {
                ROS_DEBUG("[RectFromCluster] 已达到最大方环数量 %d", config_.max_rectangles);
                break;
            }

            // 检查聚类大小
            if (cluster->indices.size() < static_cast<size_t>(config_.min_cluster_size)) {
                ROS_DEBUG("[RectFromCluster] 聚类 #%d 点数=%lu < min_cluster_size=%d, 跳过",
                          valid_clusters, static_cast<unsigned long>(cluster->indices.size()), config_.min_cluster_size);
                continue;
            }
            valid_clusters++;

            ROS_DEBUG("[RectFromCluster] 处理聚类 #%d/%d (点数=%lu)",
                      valid_clusters, total_clusters, cluster->indices.size());

            // 提取聚类点云
            PointCloudPtrT cluster_cloud(new pcl::PointCloud<PointT>);
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(cluster);
            extract.filter(*cluster_cloud);

            // ========== 步骤 1: PCA 分析，确定投影平面 ==========
            // 使用边界框中心代替算术平均质心（避免点云分布不均导致的偏移）
            Eigen::Vector4f centroid4 = core::computeBBoxCentroid(*cluster_cloud, *cluster);
            Eigen::Vector3f centroid = centroid4.head<3>();

            // 计算协方差矩阵
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (const auto& pt : cluster_cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f diff = p - centroid;
                cov += diff * diff.transpose();
            }
            cov /= cluster_cloud->size();

            // 特征值分解
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
            Eigen::Vector3f eigenvalues = solver.eigenvalues();
            Eigen::Matrix3f eigenvectors = solver.eigenvectors();

            // 确保右手坐标系
            if (eigenvectors.determinant() < 0) {
                eigenvectors.col(2) = -eigenvectors.col(2);
            }

            // 打印特征值，帮助分析点云分布
            ROS_DEBUG("[RectFromCluster] PCA 特征值：[%.4f, %.4f, %.4f]",
                      eigenvalues[0], eigenvalues[1], eigenvalues[2]);

            // 投影矩阵（将点云投影到最大两个特征向量张成的平面）
            Eigen::Matrix3f R = eigenvectors.transpose();

            // ========== 步骤 2: 投影到 2D 平面 ==========
            std::vector<Eigen::Vector2f> points_2d;
            points_2d.reserve(cluster_cloud->size());

            for (const auto& pt : cluster_cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f centered = p - centroid;
                Eigen::Vector3f rotated = R * centered;
                points_2d.emplace_back(rotated[0], rotated[1]);
            }

            // ========== 步骤 3: 提取边界点 ==========
            std::vector<int> boundary_indices =
                extractBoundaryPoints<PointT>(cluster_cloud, 0.5f * M_PI);

            ROS_DEBUG("[RectFromCluster] 边界点数量=%lu", boundary_indices.size());

            if (boundary_indices.size() < 8) {
                ROS_DEBUG("[RectFromCluster] 边界点太少：%lu < 8, 跳过", boundary_indices.size());
                boundary_fail++;
                continue;
            }

            // 获取边界点的 2D 坐标
            std::vector<Eigen::Vector2f> boundary_2d;
            boundary_2d.reserve(boundary_indices.size());
            for (int idx : boundary_indices) {
                if (idx >= 0 && idx < static_cast<int>(points_2d.size())) {
                    boundary_2d.push_back(points_2d[idx]);
                }
            }

            // 计算边界点中心
            Eigen::Vector2f boundary_center(0, 0);
            for (const auto& pt : boundary_2d) {
                boundary_center += pt;
            }
            boundary_center /= boundary_2d.size();

            // 排序边界点
            std::vector<Eigen::Vector2f> sorted_boundary =
                sortBoundaryPoints(boundary_2d, boundary_center);

            ROS_DEBUG("[RectFromCluster] 排序后边界点=%lu", sorted_boundary.size());

            // ========== 步骤 4: 角点检测 ==========
            std::vector<int> corner_indices =
                detectCornersFromBoundary(sorted_boundary, 0.6f * M_PI);

            ROS_DEBUG("[RectFromCluster] 边界点=%lu, 角点=%lu",
                      sorted_boundary.size(), corner_indices.size());

            if (corner_indices.size() < 3) {
                ROS_DEBUG("[RectFromCluster] 角点太少：%lu < 3, 尝试最小外接矩形",
                          corner_indices.size());
                corner_fail++;
            }

            // ========== 步骤 5: 拟合矩形 ==========
            Eigen::Vector2f center_2d;
            float length, width, angle;

            bool fitted = false;
            if (corner_indices.size() >= 3) {
                // 尝试从角点拟合
                fitted = fitRectangleFromCorners(sorted_boundary, corner_indices,
                                                  center_2d, length, width, angle);
                if (fitted) {
                    ROS_DEBUG("[RectFromCluster] 角点拟合成功：%.2fx%.2f, angle=%.1f°",
                              length, width, angle);
                }
            }

            if (!fitted) {
                // 角点拟合失败，使用最小外接矩形
                fitted = fitMinAreaRect(sorted_boundary, center_2d, length, width, angle);
                if (fitted) {
                    ROS_DEBUG("[RectFromCluster] 最小外接矩形拟合成功：%.2fx%.2f, angle=%.1f°",
                              length, width, angle);
                } else {
                    ROS_DEBUG("[RectFromCluster] 矩形拟合失败");
                    fit_fail++;
                    continue;
                }
            }

            // ========== 步骤 6: 验证矩形 ==========
            if (!validateRectangle(length, width, config_)) {
                ROS_DEBUG("[RectFromCluster] 矩形验证失败：%.2fx%.2f (min=%.2fx%.2f, max=%.2fx%.2f)",
                          length, width, config_.length_min, config_.width_min,
                          config_.length_max, config_.width_max);
                validate_fail++;
                // 记录尺寸用于统计
                rectangle_sizes_.emplace_back(length, width);
                continue;
            }

            ROS_INFO("[RectFromCluster] 方环 #%d 验证通过：%.2fx%.2f, angle=%.1f°",
                     rect_num + 1, length, width, angle);

            // ========== 步骤 7: 计算 3D 中心 ==========
            Eigen::Vector3f center_3d = centroid +
                eigenvectors * Eigen::Vector3f(center_2d[0], center_2d[1], 0);

            // ========== 步骤 8: 创建矩形对象 ==========
            typename pcl::PointIndices::Ptr rect_indices(new pcl::PointIndices);
            rect_indices->indices = cluster->indices;

            typename pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
            coeffs->values.resize(9);
            coeffs->values[0] = center_3d[0];
            coeffs->values[1] = center_3d[1];
            coeffs->values[2] = center_3d[2];
            coeffs->values[3] = eigenvectors(0, 2);  // 法向量（最小特征值方向）
            coeffs->values[4] = eigenvectors(1, 2);
            coeffs->values[5] = eigenvectors(2, 2);
            coeffs->values[6] = length;
            coeffs->values[7] = width;
            coeffs->values[8] = angle;

            auto rect = ObjectFactory::createRectangle(
                "rectangle_" + std::to_string(++rect_num),
                rect_indices, coeffs, timer.elapsed(),
                length, width, 0.05f,
                false
            );

            rectangles.push_back(rect);

            // 标记已使用的点
            for (int idx : cluster->indices) {
                used_indices.insert(idx);
            }

            ROS_DEBUG("[RectFromCluster] 方环 #%d: center=(%.2f, %.2f, %.2f), size=%.2fx%.2f, angle=%.1f°",
                      rect_num, center_3d[0], center_3d[1], center_3d[2], length, width, angle);
        }

        if (LOG_SHOULD()) {
            ROS_INFO("[RectFromCluster] 检测完成：%d/%d 聚类，找到 %d 个方环",
                     valid_clusters, total_clusters, rect_num);
            ROS_INFO("[RectFromCluster] 失败统计：边界点=%d, 角点=%d, 拟合=%d, 验证=%d",
                     boundary_fail, corner_fail, fit_fail, validate_fail);
            
            // 添加详细失败原因分析
            if (validate_fail > 0 && rect_num == 0) {
                ROS_WARN("[RectFromCluster] 所有聚类都因尺寸过小被过滤！");
                ROS_WARN("[RectFromCluster] 当前配置：length_min=%.2f, width_min=%.2f",
                         config_.length_min, config_.width_min);
                ROS_WARN("[RectFromCluster] 建议：降低 min_inliers 或 length_min/width_min，或检查墙体是否用掉了太多点");
            }
            
            // 输出尺寸分布统计（帮助调试）
            if (!rectangle_sizes_.empty()) {
                ROS_INFO("[RectFromCluster] 尺寸分布统计：");
                float max_size = 0;
                for (const auto& s : rectangle_sizes_) {
                    max_size = std::max(max_size, std::max(s.first, s.second));
                }
                int bin_count = 10;
                float bin_size = max_size / bin_count + 0.01;
                std::vector<int> histogram(bin_count, 0);
                for (const auto& s : rectangle_sizes_) {
                    float avg = (s.first + s.second) / 2;
                    int bin = std::min(bin_count - 1, static_cast<int>(avg / bin_size));
                    histogram[bin]++;
                }
                for (int i = 0; i < bin_count; i++) {
                    if (histogram[i] > 0) {
                        ROS_INFO("  [%.2f-%.2f]: %d 个", i * bin_size, (i + 1) * bin_size, histogram[i]);
                    }
                }
            }
        }

        return rectangles;
    }
    
private:
    /**
     * @brief 最小外接矩形拟合（备用）
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

        // 特征值分解
        float trace = xx + yy;
        float det = xx * yy - xy * xy;
        float disc = std::sqrt(std::max(0.0f, trace * trace - 4 * det));
        float lambda1 = (trace + disc) / 2;
        float lambda2 = (trace - disc) / 2;

        float theta = std::atan2(xy, xx - lambda1) * 180.0f / M_PI;
        if (theta < 0) theta += 180;
        if (theta > 90) theta -= 180;

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

        if (length < width) {
            std::swap(length, width);
            angle += 90;
            if (angle > 90) angle -= 180;
        }

        return true;
    }
    
    RectangleExtractionConfig config_;
};

}  // namespace core
}  // namespace pcl_object_detection
