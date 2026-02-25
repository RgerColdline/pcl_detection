#pragma once

#include "object_base.hpp"

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
        os << "  圆环参数: 中心=(" << coefficients->values[0] << ", " << coefficients->values[1]
           << ", 0), "
           << "半径=" << coefficients->values[2] << "\n";
    }

    void getDimensions(float &width, float &height, float &depth) const override {
        width  = 2 * coefficients->values[2];  // 直径
        height = 0.0f;
        depth  = width;                        // 暂时用直径作为深度
    }

    void getParameters(std::vector<float> &params) const override {
        params = coefficients->values;
    }

    // 圆环特有方法
    double getRadius() const { return coefficients->values[2]; }
};

}  // namespace core
}  // namespace pcl_object_detection