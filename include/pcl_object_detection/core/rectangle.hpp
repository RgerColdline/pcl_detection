#pragma once

#include "object_base.hpp"

#include <Eigen/Geometry>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 矩形框（方环）类
 * 
 * 用于检测矩形框架结构（如窗户、门框等）
 * 
 * coefficients 系数格式：
 *   [0-2]: 中心位置 (center_x, center_y, center_z)
 *   [3-5]: 法向量 (normal_x, normal_y, normal_z)
 *   [6]:   长边长度 (length)
 *   [7]:   短边长度 (width)
 *   [8]:   旋转角度 (rotation_angle，单位：度)
 */
class Rectangle : public Object
{
  public:
    Rectangle(const std::string &name, typename pcl::PointIndices::Ptr inliers,
              typename pcl::ModelCoefficients::Ptr coefficients, double extraction_time,
              bool using_normal = false) {
        this->name            = name;
        this->inliers         = inliers;
        this->coefficients    = coefficients;
        this->extraction_time = extraction_time;
        this->using_normal    = using_normal;
    }

    std::string getType() const override { return "Rectangle"; }

    void printDetails(std::ostream &os) const override {
        os << "  矩形参数：中心=(" << coefficients->values[0] << ", " << coefficients->values[1]
           << ", " << coefficients->values[2] << "), "
           << "法向量=(" << coefficients->values[3] << ", " << coefficients->values[4]
           << ", " << coefficients->values[5] << "), "
           << "长=" << coefficients->values[6] << "m, "
           << "宽=" << coefficients->values[7] << "m, "
           << "旋转角=" << coefficients->values[8] << "°\n";
    }

    void getParameters(std::vector<float> &params) const override { params = coefficients->values; }

    // 矩形特有方法
    double getLength() const { return coefficients->values[6]; }  // 长边
    double getWidth() const { return coefficients->values[7]; }   // 短边
    double getRotationAngle() const { return coefficients->values[8]; }

    // 获取中心
    Eigen::Vector3f getCenter() const {
        return Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    }

    // 获取法向量
    Eigen::Vector3f getNormal() const {
        return Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    }

    // 获取内框尺寸（考虑边框厚度后的净空尺寸）
    double getInnerLength() const {
        // 假设边框厚度约为 0.05m，可根据实际情况调整
        return std::max(0.0, coefficients->values[6] - 0.1);
    }

    double getInnerWidth() const {
        return std::max(0.0, coefficients->values[7] - 0.1);
    }
};

}  // namespace core
}  // namespace pcl_object_detection
