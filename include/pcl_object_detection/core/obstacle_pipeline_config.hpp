#pragma once

#include "config.hpp"  // 使用原有的配置结构

#include <string>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

// 时间测量配置（新 Pipeline 专用）
struct ObstacleTimingConfig
{
    bool enable     = true;
    bool downsample = true;
    bool normal     = true;
    bool ground     = true;
    bool wall       = true;
    bool cluster    = true;
    bool rectangle  = true;
    bool obb        = true;
    bool total      = true;
};

// 下采样配置（新 Pipeline 专用）
struct ObstacleDownsampleConfig
{
    bool approx     = true;   // true=ApproximateVoxelGrid, false=VoxelGrid
    float leaf_size = 0.08f;  // 下采样网格大小（米）
};

// 法向量估计配置（支持多线程）
struct NormalEstimationConfig
{
    bool enable        = true;
    int k_search       = 50;     // K 近邻搜索点数
    int num_threads    = 4;      // 并行线程数
    float view_angle   = 0.0f;   // 视角角度（用于法向量定向）
};

// 地面分割配置（不使用法向量，纯几何 + 逻辑过滤）
struct GroundConfig
{
    bool enable              = true;
    float distance_threshold = 0.05f;  // RANSAC 距离阈值
    int max_iterations       = 100;    // RANSAC 最大迭代次数
    float normal_z_min       = 0.8f;   // 地面法向量 Z 分量最小值（逻辑过滤用）
    float normal_z_max       = 1.0f;   // 地面法向量 Z 分量最大值
    int min_ground_points    = 500;    // 最小地面点数（逻辑过滤）
};

// 墙壁分割配置（不使用法向量，纯几何 + 逻辑过滤）
struct WallExtractionConfig
{
    bool enable              = true;
    bool using_normal        = false;  // 保留字段，兼容旧代码（但实际不使用）
    float distance_threshold = 0.02f;  // RANSAC 距离阈值
    int max_iterations       = 500;    // RANSAC 最大迭代次数
    int min_inliers          = 200;    // 最小内点数
    float angle_threshold    = 15.0f;  // 平面角度阈值（度）
    float normal_z_max       = 0.3f;   // 墙面法向量 Z 分量最大值（逻辑过滤，排除水平面）
    int max_walls            = 4;      // 最大检测墙面数量
    std::vector<float> axis  = {0.0f, 0.0f, 1.0f};  // 保留字段，兼容旧代码
};

// 聚类配置
struct ClusterConfig
{
    bool enable              = true;
    float cluster_tolerance  = 0.1f;   // 聚类距离阈值（米）
    int min_cluster_size     = 50;     // 最小簇点数
    int max_cluster_size     = 1000;   // 最大簇点数
};

// 方环提取配置（基于 PCA 投影 + 边界提取）
struct RectangleExtractionConfig
{
    bool enable              = true;
    bool using_normal        = false;  // 保留字段，兼容旧代码（但实际不使用）
    float distance_threshold = 0.03f;  // 保留字段，兼容旧代码
    int min_inliers          = 100;    // 保留字段，兼容旧代码
    int min_cluster_size     = 100;    // 最小聚类点数
    float plane_threshold    = 0.03f;  // 平面拟合距离阈值
    float boundary_threshold = 0.05f;  // 边界提取距离阈值
    float corner_threshold   = 0.1f;   // 角点检测距离阈值
    float length_min         = 0.5f;   // 最小长度（米）
    float length_max         = 3.0f;   // 最大长度（米）
    float width_min          = 0.3f;   // 最小宽度（米）
    float width_max          = 2.0f;   // 最大宽度（米）
    float aspect_ratio_max   = 4.0f;   // 最大长宽比
    int max_rectangles       = 10;     // 最大检测方环数量
    // 中空检测参数（方环特征）
    float hollow_ratio_threshold = 0.4f;  // 中空比例阈值（0-1，越大要求越中空）
    // 以下字段保留，兼容旧代码
    float plane_distance_threshold = 0.05f;
    float plane_angle_threshold    = 30.0f;
    float plane_normal_z_max       = 0.5f;
    float coverage_threshold       = 0.5f;
    int ransac_iterations          = 200;
};

// OBB 包围盒配置
struct ObbConfig
{
    float inflation_radius   = 0.1f;   // 膨胀半径（米）
    float min_obstacle_height = 0.2f;  // 最小障碍物高度（米）
};

// 新 Pipeline 统一配置
struct ObstaclePipelineConfig
{
    // 坐标系转换配置
    struct TransformConfig {
        bool enable = false;           // 是否启用坐标转换
        std::string pose_topic = "/mavros/local_position/pose";  // 位姿话题
        std::string frame_id = "world";  // 世界坐标系名称
    };
    
    TransformConfig transform_config;  // 坐标转换配置
    
    ObstacleDownsampleConfig downsample_config;
    NormalEstimationConfig normal_config;      // 法向量估计配置
    GroundConfig ground_config;
    WallExtractionConfig wall_config;          // 墙壁提取配置（无 normal 版本）
    ClusterConfig cluster_config;
    RectangleExtractionConfig rectangle_config;  // 方环提取配置（基于聚类）
    ObbConfig obb_config;
    ObstacleTimingConfig timing_config;

    bool save_objects      = false;
    std::string output_dir = "./obstacle_objects/";
};

}  // namespace core
}  // namespace pcl_object_detection
