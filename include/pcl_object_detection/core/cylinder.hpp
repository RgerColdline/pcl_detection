#pragma once

#include "object_base.hpp"

namespace pcl_object_detection
{
namespace core
{

class Cylinder : public Object
{
  public:
    Cylinder(const std::string &name, typename pcl::PointIndices::Ptr inliers,
             typename pcl::ModelCoefficients::Ptr coefficients, double extraction_time,
             bool using_normal = true) {
        this->name            = name;
        this->inliers         = inliers;
        this->coefficients    = coefficients;
        this->extraction_time = extraction_time;
        this->using_normal    = using_normal;
    }

    std::string getType() const override { return "Cylinder"; }

    void printDetails(std::ostream &os) const override {
        os << "  圆柱参数: 中心=(" << coefficients->values[0] << ", " << coefficients->values[1]
           << ", " << coefficients->values[2] << "), "
           << "轴向=(" << coefficients->values[3] << ", " << coefficients->values[4] << ", "
           << coefficients->values[5] << "), "
           << "半径=" << coefficients->values[6] << "\n";
    }

    void getParameters(std::vector<float> &params) const override { params = coefficients->values; }

    // 圆柱特有方法
    double getRadius() const { return coefficients->values[6]; }

    Eigen::Vector3f getAxisDirection() const {
        return Eigen::Vector3f(coefficients->values[3], coefficients->values[4],
                               coefficients->values[5]);
    }
};

}  // namespace core
}  // namespace pcl_object_detection