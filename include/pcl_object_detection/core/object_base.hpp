#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace pcl_object_detection
{
namespace core
{

// 所有检测到的对象的基类
class Object
{
  public:
    using Ptr         = std::shared_ptr<Object>;
    using ConstPtr    = std::shared_ptr<const Object>;

    virtual ~Object() = default;

    // 基础属性（所有对象共有）
    typename pcl::PointIndices::Ptr inliers;            // 点云索引
    typename pcl::ModelCoefficients::Ptr coefficients;  // 模型系数
    std::string name;                                   // 对象名称
    double extraction_time;                             // 提取时间
    Eigen::Vector3f color;                              // 显示颜色
    bool using_normal;                                  // 是否使用法向量

    // 虚函数 - 多态核心
    virtual std::string getType() const                                         = 0;
    virtual void printDetails(std::ostream &os) const                           = 0;
    virtual void getDimensions(float &width, float &height, float &depth) const = 0;
    virtual void getParameters(std::vector<float> &params) const                = 0;

    // 辅助方法
    int getPointCount() const { return inliers ? inliers->indices.size() : 0; }

    // 静态创建方法（工厂方法）
    static Ptr createWall(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                          typename pcl::ModelCoefficients::Ptr coefficients, double extraction_time,
                          bool using_normal = true);

    static Ptr createCylinder(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                              typename pcl::ModelCoefficients::Ptr coefficients,
                              double extraction_time, bool using_normal = true);

    static Ptr createCircle(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                            typename pcl::ModelCoefficients::Ptr coefficients,
                            double extraction_time, bool using_normal = false);
};

}  // namespace core
}  // namespace pcl_object_detection