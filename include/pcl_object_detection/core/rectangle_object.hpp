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
 * @brief 矩形框对象
 */
class Rectangle : public Object
{
public:
    Rectangle() {
        color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);  // 黄色
        using_normal = false;
    }

    Rectangle(const std::string &name_,
              typename pcl::PointIndices::Ptr inliers_,
              typename pcl::ModelCoefficients::Ptr coefficients_,
              double extraction_time_,
              float width_, float height_, float depth_,
              bool using_normal_ = false) {
        name = name_;
        inliers = inliers_;
        coefficients = coefficients_;
        extraction_time = extraction_time_;
        width = width_;
        height = height_;
        depth = depth_;
        using_normal = using_normal_;
        color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
    }

    std::string getType() const override { return "Rectangle"; }
    int getMessageType() const override { return 3; }

    void printDetails(std::ostream &os) const override {
        os << "  类型：矩形框\n";
        os << "  中心：(" << coefficients->values[0] << ", "
           << coefficients->values[1] << ", " << coefficients->values[2] << ")\n";
        os << "  法向量：(" << coefficients->values[3] << ", "
           << coefficients->values[4] << ", " << coefficients->values[5] << ")\n";
        os << "  尺寸：长=" << coefficients->values[6] << "m, 宽=" << coefficients->values[7] << "m\n";
        os << "  旋转角：" << coefficients->values[8] << "°\n";
    }

    void getParameters(std::vector<float> &params) const override {
        params.resize(9);
        for (int i = 0; i < 9; ++i) {
            params[i] = coefficients->values[i];
        }
    }

    void fillMessageParams(std::vector<float>& params) const override {
        getParameters(params);
    }
};

}  // namespace core
}  // namespace pcl_object_detection
