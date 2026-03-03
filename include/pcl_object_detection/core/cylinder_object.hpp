#pragma once

#include "object_base.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <string>
#include <vector>
#include <ostream>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 圆柱对象
 */
class Cylinder : public Object
{
public:
    Cylinder() {
        color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // 红色
        using_normal = true;
    }

    Cylinder(const std::string &name_,
             typename pcl::PointIndices::Ptr inliers_,
             typename pcl::ModelCoefficients::Ptr coefficients_,
             double extraction_time_,
             float width_, float height_, float depth_,
             bool using_normal_ = true) {
        name = name_;
        inliers = inliers_;
        coefficients = coefficients_;
        extraction_time = extraction_time_;
        width = width_;
        height = height_;
        depth = depth_;
        using_normal = using_normal_;
        color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    }

    std::string getType() const override { return "Cylinder"; }
    int getMessageType() const override { return 1; }

    void printDetails(std::ostream &os) const override {
        os << "  类型：圆柱\n";
        os << "  中心：(" << coefficients->values[0] << ", "
           << coefficients->values[1] << ", " << coefficients->values[2] << ")\n";
        os << "  轴向：(" << coefficients->values[3] << ", "
           << coefficients->values[4] << ", " << coefficients->values[5] << ")\n";
        os << "  半径：" << coefficients->values[6] << "\n";
    }

    void getParameters(std::vector<float> &params) const override {
        params.resize(7);
        for (int i = 0; i < 7; ++i) {
            params[i] = coefficients->values[i];
        }
    }

    void fillMessageParams(std::vector<float>& params) const override {
        getParameters(params);
    }
};

}  // namespace core
}  // namespace pcl_object_detection
