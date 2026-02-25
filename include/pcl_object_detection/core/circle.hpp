#pragma once

#include "object_base.hpp"

#include <Eigen/Geometry>

namespace pcl_object_detection
{
namespace core
{

class Circle : public Object
{
  public:
    Circle(const std::string &name, typename pcl::PointIndices::Ptr inliers,
           typename pcl::ModelCoefficients::Ptr coefficients, double extraction_time,
           bool using_normal = false) {
        this->name            = name;
        this->inliers         = inliers;
        this->coefficients    = coefficients;
        this->extraction_time = extraction_time;
        this->using_normal    = using_normal;
    }

    std::string getType() const override { return "Circle"; }

    void printDetails(std::ostream &os) const override {
        // SACMODEL_CIRCLE3D 系数格式：[center_x, center_y, center_z, normal_x, normal_y, normal_z, radius]
        os << "  圆环参数：中心=(" << coefficients->values[0] << ", " << coefficients->values[1]
           << ", " << coefficients->values[2] << "), "
           << "法向量=(" << coefficients->values[3] << ", " << coefficients->values[4]
           << ", " << coefficients->values[5] << "), "
           << "半径=" << coefficients->values[6] << "\n";
    }

    void getParameters(std::vector<float> &params) const override { params = coefficients->values; }

    // 圆环特有方法
    double getRadius() const { return coefficients->values[6]; }
    
    // 获取圆心
    Eigen::Vector3f getCenter() const { 
        return Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]); 
    }
    
    // 获取法向量
    Eigen::Vector3f getNormal() const { 
        return Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]); 
    }
};

}  // namespace core
}  // namespace pcl_object_detection
