#pragma once

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <memory>
#include <ostream>

namespace pcl_object_detection
{
namespace core
{

// 前向声明
class Object;
using ObjectPtr = std::shared_ptr<Object>;
using ObjectConstPtr = std::shared_ptr<const Object>;

/**
 * @brief 所有检测到的对象的基类（纯接口）
 */
class Object
{
public:
    using Ptr = std::shared_ptr<Object>;
    using ConstPtr = std::shared_ptr<const Object>;

    virtual ~Object() = default;

    // ========================================================================
    // 基础属性（所有对象共有）
    // ========================================================================
    typename pcl::PointIndices::Ptr inliers;            // 点云索引
    typename pcl::ModelCoefficients::Ptr coefficients;  // 模型系数
    std::string name;                                   // 对象名称
    double extraction_time;                             // 提取时间 (ms)
    Eigen::Vector3f color;                              // 显示颜色
    bool using_normal;                                  // 是否使用法向量

    // 尺寸信息
    float width  = 0.0f;   // X 方向尺寸或直径
    float height = 0.0f;   // Z 方向尺寸（高度）
    float depth  = 0.0f;   // Y 方向尺寸

    // ========================================================================
    // 纯虚函数（必须由子类实现）
    // ========================================================================
    
    /// @brief 获取类型名称
    virtual std::string getType() const = 0;
    
    /// @brief 打印详细信息
    virtual void printDetails(std::ostream &os) const = 0;
    
    /// @brief 获取模型参数
    virtual void getParameters(std::vector<float> &params) const = 0;
    
    /// @brief 填充 ROS 消息（在 detector_wrapper 中使用）
    virtual int getMessageType() const = 0;
    virtual void fillMessageParams(std::vector<float>& params) const = 0;

    // ========================================================================
    // 辅助方法
    // ========================================================================
    int getPointCount() const { 
        return inliers ? static_cast<int>(inliers->indices.size()) : 0; 
    }
    
    /// @brief 计算质心
    template <typename PointT>
    Eigen::Vector3f getCentroid(const pcl::PointCloud<PointT>& cloud) const {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, *inliers, centroid);
        return centroid.head<3>();
    }
};

}  // namespace core
}  // namespace pcl_object_detection
