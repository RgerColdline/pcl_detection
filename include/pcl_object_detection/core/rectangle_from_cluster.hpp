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
 * @brief 提取点云的边界点（使用凸包算法）
 * @param points_2d 投影后的 2D 点
 * @return 边界点索引（凸包顶点）
 */
std::vector<int> extractBoundaryPoints2D(
    const std::vector<Eigen::Vector2f>& points_2d)
{
    std::vector<int> boundary_indices;

    if (points_2d.size() < 3) return boundary_indices;

    // 找到最左下角的点作为起点
    int start_idx = 0;
    for (size_t i = 1; i < points_2d.size(); ++i) {
        if (points_2d[i][1] < points_2d[start_idx][1] ||
            (points_2d[i][1] == points_2d[start_idx][1] && points_2d[i][0] < points_2d[start_idx][0])) {
            start_idx = i;
        }
    }

    // Graham 扫描算法
    std::vector<int> hull;
    hull.push_back(start_idx);

    int current = start_idx;
    while (true) {
        int next = 0;
        for (size_t i = 0; i < points_2d.size(); ++i) {
            if (i == static_cast<size_t>(current)) continue;
            
            if (next == current) {
                next = i;
                continue;
            }

            // 计算叉积
            Eigen::Vector2f v1 = points_2d[next] - points_2d[current];
            Eigen::Vector2f v2 = points_2d[i] - points_2d[current];
            float cross = v1[0] * v2[1] - v1[1] * v2[0];

            // 选择最右边的点（顺时针）
            if (cross < 0) {
                next = i;
            } else if (cross == 0) {
                // 共线时选择更远的点
                float dist1 = v1.norm();
                float dist2 = v2.norm();
                if (dist2 > dist1) {
                    next = i;
                }
            }
        }

        if (next == start_idx) break;
        hull.push_back(next);
        current = next;

        // 防止死循环
        if (hull.size() > points_2d.size()) break;
    }

    return hull;
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
 * @brief 从排序的边界点检测角点（增强版：支持检测角度变化最大的 N 个角点）
 * 
 * 对于"日"字形方环（上面是口，下面是两条腿）：
 * - 口的 4 个角点角度变化最明显（接近 90 度）
 * - 腿部末端的角度变化较小
 * - 通过选择角度变化最大的 4 个点，可以优先检测口的角点
 */
std::vector<int> detectCornersFromBoundary(
    const std::vector<Eigen::Vector2f>& sorted_points,
    float angle_threshold = 0.6f * M_PI,  // 108 度（比之前的 90 度更宽松）
    int max_corners = 4)                   // 最多返回的角点数量
{
    std::vector<int> corner_indices;

    if (sorted_points.size() < 5) return corner_indices;

    int window = 5;  // 增大窗口，使角度计算更稳定

    // 存储所有候选角点及其角度变化值
    std::vector<std::pair<int, float>> corner_candidates;

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

        // 角度越小，角点越明显（锐角/直角）
        // 存储角度值（用于排序）
        corner_candidates.emplace_back(i, angle);
    }

    // 按角度从小到大排序（角度最小的最可能是角点）
    std::sort(corner_candidates.begin(), corner_candidates.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    // 选择角度变化最大的 max_corners 个点
    // 但只保留角度小于阈值的点
    for (size_t i = 0; i < corner_candidates.size() && corner_indices.size() < static_cast<size_t>(max_corners); ++i) {
        if (corner_candidates[i].second < angle_threshold) {
            corner_indices.push_back(corner_candidates[i].first);
        }
    }

    // 按索引排序，使角点按顺时针/逆时针顺序排列
    std::sort(corner_indices.begin(), corner_indices.end());

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
 * @brief 检查矩形是否中空（方环的特征）
 * 
 * @param points_2d 所有投影后的 2D 点
 * @param center 矩形中心
 * @param length 矩形长度
 * @param width 矩形宽度
 * @param angle 矩形角度（度）
 * @param hollow_ratio_threshold 中空比例阈值（默认 0.4，即中间 40% 区域应该没有点）
 * @return true=中空，false=实心
 */
bool checkHollowRectangle(
    const std::vector<Eigen::Vector2f>& points_2d,
    const Eigen::Vector2f& center,
    float length,
    float width,
    float angle,
    float hollow_ratio_threshold = 0.3f)
{
    if (points_2d.size() < 10) return false;

    // 计算矩形边框的估计宽度（假设边框厚度约为短边的 10-20%）
    float border_thickness = std::min(length, width) * 0.15f;
    border_thickness = std::max(border_thickness, 0.05f);  // 最小 5cm

    // 内部矩形的尺寸
    float inner_length = length - 2 * border_thickness;
    float inner_width = width - 2 * border_thickness;

    if (inner_length <= 0 || inner_width <= 0) return false;

    // 将角度转换为弧度
    float theta = -angle * M_PI / 180.0f;
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    // 统计内部区域的点数
    int inner_points = 0;
    int total_points = 0;

    for (const auto& pt : points_2d) {
        // 将点旋转回矩形坐标系
        Eigen::Vector2f centered = pt - center;
        float rx = centered[0] * cos_t - centered[1] * sin_t;
        float ry = centered[0] * sin_t + centered[1] * cos_t;

        // 检查点是否在矩形内部
        if (std::abs(rx) < length / 2.0f && std::abs(ry) < width / 2.0f) {
            total_points++;

            // 检查点是否在内部矩形（中空区域）
            if (std::abs(rx) < inner_length / 2.0f && std::abs(ry) < inner_width / 2.0f) {
                inner_points++;
            }
        }
    }

    // 计算中空比例
    float hollow_ratio = 1.0f - static_cast<float>(inner_points) / std::max(1, total_points);

    ROS_DEBUG("[RectFromCluster] 中空检测：总点=%d, 内部点=%d, 中空比例=%.2f (阈值=%.2f)",
              total_points, inner_points, hollow_ratio, hollow_ratio_threshold);

    // 中空比例大于阈值，认为是方环
    return hollow_ratio > hollow_ratio_threshold;
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

        ROS_DEBUG("[RectFromCluster] 开始处理 %d 个聚类", total_clusters);

        int cluster_idx = 0;  // 实际处理的聚类索引（包括被跳过的）
        for (const auto& cluster : clusters) {
            cluster_idx++;
            // 检查聚类大小
            if (cluster->indices.size() < static_cast<size_t>(config_.min_cluster_size)) {
                ROS_DEBUG("[RectFromCluster] 聚类 #%d/%d 跳过：点数=%lu < min_cluster_size=%d",
                          cluster_idx, total_clusters, cluster->indices.size(), config_.min_cluster_size);
                continue;
            }
            valid_clusters++;

            ROS_DEBUG("[RectFromCluster] 处理聚类 #%d/%d (点数=%lu)",
                      cluster_idx, total_clusters, cluster->indices.size());

            // 提取聚类点云
            PointCloudPtrT cluster_cloud(new pcl::PointCloud<PointT>);
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(cluster);
            extract.filter(*cluster_cloud);

            // ========== 步骤 1: PCA 分析，确定投影平面 ==========
            // 直接使用 cluster_cloud 的所有点计算质心
            // 注意：不能用 cluster 索引，因为那是相对于原始 cloud 的索引
            Eigen::Vector4f centroid4;
            pcl::compute3DCentroid(*cluster_cloud, centroid4);
            Eigen::Vector3f centroid = centroid4.head<3>();

            ROS_DEBUG("[RectFromCluster] 聚类中心=(%.2f, %.2f, %.2f), 点数=%lu",
                      centroid[0], centroid[1], centroid[2], cluster_cloud->size());

            // 计算协方差矩阵
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (const auto& pt : cluster_cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f diff = p - centroid;
                cov += diff * diff.transpose();
            }
            cov /= cluster_cloud->size();

            // 特征值分解（特征值从小到大：[0]=最小，[2]=最大）
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
            Eigen::Vector3f eigenvalues = solver.eigenvalues();
            Eigen::Matrix3f eigenvectors = solver.eigenvectors();

            // 确保右手坐标系
            if (eigenvectors.determinant() < 0) {
                eigenvectors.col(2) = -eigenvectors.col(2);
            }

            // 打印特征值，帮助分析点云分布
            // eigenvalues[0] = 法向量方向（点云最薄）
            // eigenvalues[1] = 次长方向
            // eigenvalues[2] = 最长方向
            ROS_DEBUG("[RectFromCluster] PCA 特征值：[%.4f(法向), %.4f(次长), %.4f(最长)]",
                      eigenvalues[0], eigenvalues[1], eigenvalues[2]);

            // ========== 步骤 1.5: 计算法向量（最小特征值方向）==========
            Eigen::Vector3f normal = eigenvectors.col(0);  // 最小特征值方向是法向量
            
            // 检查法向量方向
            float normal_z = std::abs(normal[2]);
            ROS_DEBUG("[RectFromCluster] 法向量=(%.3f, %.3f, %.3f), Z 分量=%.3f",
                      normal[0], normal[1], normal[2], normal_z);

            // ========== 步骤 2: 投影到 2D 平面 ==========
            // 对于竖直方环，需要正确区分水平方向（长度）和竖直方向（高度）
            // 投影到垂直于法向量的平面，然后区分水平和竖直方向
            std::vector<Eigen::Vector2f> points_2d;
            points_2d.reserve(cluster_cloud->size());

            // 投影矩阵：将点云投影到特征向量空间
            Eigen::Matrix3f R = eigenvectors.transpose();

            // 确定哪个特征向量方向最接近竖直方向（Z 轴）
            // eigenvectors 的列是特征向量，按特征值从小到大排列
            // col(0)=法向量，col(1)=次长方向，col(2)=最长方向
            float dot1 = std::abs(eigenvectors(2, 1));  // col(1) 的 Z 分量
            float dot2 = std::abs(eigenvectors(2, 2));  // col(2) 的 Z 分量

            // 选择更接近 Z 轴的方向作为竖直方向（Y 轴）
            // 另一个方向作为水平方向（X 轴）
            int horizontal_idx = (dot1 > dot2) ? 2 : 1;  // 水平方向索引
            int vertical_idx = (dot1 > dot2) ? 1 : 2;    // 竖直方向索引

            ROS_DEBUG("[RectFromCluster] 水平方向索引=%d (dot=%.3f), 竖直方向索引=%d (dot=%.3f)",
                      horizontal_idx, dot1 > dot2 ? dot2 : dot1,
                      vertical_idx, dot1 > dot2 ? dot1 : dot2);

            for (const auto& pt : cluster_cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f centered = p - centroid;
                Eigen::Vector3f rotated = R * centered;
                
                // X=水平方向，Y=竖直方向
                points_2d.emplace_back(rotated(horizontal_idx), rotated(vertical_idx));
            }

            ROS_DEBUG("[RectFromCluster] 投影完成：2D 点数=%lu", points_2d.size());
            
            // 保存索引供后续 3D 中心重建使用
            int proj_horizontal_idx = horizontal_idx;
            int proj_vertical_idx = vertical_idx;

            // ========== 步骤 3: 直接使用所有投影点进行最小外接矩形拟合 ==========
            // 不提取边界点和角点，直接对所有点拟合矩形
            // 这样对于"日"字形方环也能正确拟合外轮廓
            std::vector<Eigen::Vector2f>& points_2d_ref = points_2d;
            
            ROS_DEBUG("[RectFromCluster] 投影点数=%lu", points_2d_ref.size());

            // ========== 步骤 4: 最小外接矩形拟合 ==========
            Eigen::Vector2f center_2d;
            float length, width, angle;

            bool fitted = fitMinAreaRect(points_2d_ref, center_2d, length, width, angle);
            
            if (!fitted) {
                ROS_DEBUG("[RectFromCluster] 最小外接矩形拟合失败");
                fit_fail++;
                continue;
            }

            ROS_DEBUG("[RectFromCluster] 最小外接矩形拟合成功：%.2fx%.2f, angle=%.1f°",
                      length, width, angle);

            // ========== 步骤 6: 计算 3D 中心 ==========
            // 重建 3D 中心：使用投影时的索引
            // center_2d[0]=水平方向，center_2d[1]=竖直方向
            Eigen::Vector3f center_offset(0, 0, 0);  // 初始化为 0（法向量方向无偏移）
            center_offset(proj_horizontal_idx) = center_2d[0];
            center_offset(proj_vertical_idx) = center_2d[1];
            
            Eigen::Vector3f center_3d = centroid + eigenvectors * center_offset;

            // 输出每个聚类的详细信息（方便调试）- 改为 DEBUG 级别避免刷屏
            ROS_DEBUG("[RectFromCluster] 聚类 #%d/%d: 中心=(%.2f, %.2f, %.2f), 尺寸=%.2f(水平)x%.2f(竖直), angle=%.1f°, 点数=%lu",
                     valid_clusters, total_clusters, center_3d[0], center_3d[1], center_3d[2],
                     length, width, angle, cluster->indices.size());

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

            // ========== 步骤 6.5: 中空检测（方环的特征）==========
            // 检查矩形是否中空，排除实心矩形
            bool is_hollow = checkHollowRectangle(points_2d, center_2d, length, width, angle, config_.hollow_ratio_threshold);
            
            if (!is_hollow) {
                ROS_DEBUG("[RectFromCluster] 矩形不是中空（实心），跳过：%.2fx%.2f, 中空比例<%.2f",
                          length, width, config_.hollow_ratio_threshold);
                validate_fail++;
                continue;
            }

            // 方环检测成功，节流输出（每 N 帧输出一次）
            if (LOG_SHOULD()) {
                ROS_INFO("[RectFromCluster] 方环 #%d 验证通过：%.2fx%.2f, angle=%.1f° (中空), 中心=(%.2f, %.2f, %.2f), 点数=%lu",
                         rect_num + 1, length, width, angle,
                         center_3d[0], center_3d[1], center_3d[2], cluster->indices.size());
            }

            // 检查是否超过最大方环数量
            if (rect_num >= config_.max_rectangles) {
                ROS_DEBUG("[RectFromCluster] 已达到最大方环数量 %d，跳过保存", config_.max_rectangles);
                continue;
            }

            // ========== 步骤 7: 创建矩形对象 ==========
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
