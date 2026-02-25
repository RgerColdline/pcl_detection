#pragma once

#include "object_base.hpp"

#include <pcl/common/common.h>

namespace pcl_object_detection
{
namespace core
{

class Wall : public Object
{
  public:
    Wall(const std::string &name, typename pcl::PointIndices::Ptr inliers,
         typename pcl::ModelCoefficients::Ptr coefficients, double extraction_time,
         bool using_normal = true) {
        this->name            = name;
        this->inliers         = inliers;
        this->coefficients    = coefficients;
        this->extraction_time = extraction_time;
        this->using_normal    = using_normal;
    }

    std::string getType() const override { return "Wall"; }

    void printDetails(std::ostream &os) const override {
        os << "  平面方程: " << coefficients->values[0] << "x + " << coefficients->values[1]
           << "y + " << coefficients->values[2] << "z + " << coefficients->values[3] << " = 0\n";
    }

    void getDimensions(float &width, float &height, float &depth) const override {
        // 这里需要访问点云数据，可以考虑在提取时传入或后续设置
        // 临时实现
        width  = 0.0f;
        height = 0.0f;
        depth  = 0.0f;
    }

    void getParameters(std::vector<float> &params) const override { params = coefficients->values; }

    // 墙面特有方法
    Eigen::Vector3f getNormal() const {
        return Eigen::Vector3f(coefficients->values[0], coefficients->values[1],
                               coefficients->values[2]);
    }

    double getDistance() const { return coefficients->values[3]; }
};

// 在object_base.cpp中实现工厂方法
}  // namespace core
}  // namespace pcl_object_detection