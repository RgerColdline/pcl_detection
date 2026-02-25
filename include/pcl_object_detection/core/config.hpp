#pragma once

#include <string>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

// 时间测量配置
struct TimingConfig
{
    bool enable     = false;
    bool downsample = true;
    bool walls      = true;
    bool cylinders  = true;
    bool circles    = true;
    bool total      = true;
};

// 墙面配置
struct WallConfig
{
    bool enable              = true;
    bool using_normal        = true;
    float distance_threshold = 0.02f;
    int min_inliers          = 200;
    float angle_threshold    = 15.0f;
    std::vector<float> axis  = {0.0f, 0.0f, 1.0f};  // 默认Z轴
};

// 圆柱配置
struct CylinderConfig
{
    bool enable              = true;
    bool using_normal        = true;
    float distance_threshold = 0.02f;
    int min_inliers          = 200;
    float radius_min         = 0.05f;
    float radius_max         = 0.2f;
    std::vector<float> axis  = {0.0f, 0.0f, 1.0f};  // 默认Z轴
    float eps_angle          = 15.0f;
};

// 圆环配置
struct CircleConfig
{
    bool enable              = true;
    bool using_normal        = true;
    float distance_threshold = 0.02f;
    int min_inliers          = 100;
    float radius_min         = 0.1f;
    float radius_max         = 0.7f;
    
    // 平面提取参数
    float plane_distance_threshold = 0.05f;  // 平面提取距离阈值
    float plane_angle_threshold    = 45.0f;  // 平面法向量与 Z 轴的最大夹角（度）
    float plane_normal_z_max       = 0.7f;   // 平面法向量 Z 分量最大值
    
    // 圆环验证参数
    float min_coverage_angle     = 270.0f;   // 最小圆周覆盖角度（度）
    int ransac_iterations        = 500;      // RANSAC 迭代次数
    int max_circles              = 10;       // 最大检测圆环数量
    float normal_tolerance       = 0.5f;     // 法向量 Z 分量容差（竖直圆环法向量应接近水平）
};

// 统一的配置结构
struct ObjectDetectionConfig
{
    struct
    {
        std::string downsample_method = "standard";  // "standard" or "approx"
        float leaf_size               = 0.05f;
    } downsample_config;

    WallConfig wall_config;
    CylinderConfig cylinder_config;
    CircleConfig circle_config;

    TimingConfig timing_config;

    bool save_objects      = false;
    std::string output_dir = "./objects/";
};

}  // namespace core
}  // namespace pcl_object_detection