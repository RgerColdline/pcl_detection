#pragma once

#include "object_base.hpp"
#include "wall_object.hpp"
#include "cylinder_object.hpp"
#include "circle_object.hpp"
#include "rectangle_object.hpp"
#include "obstacle_object.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <ros/ros.h>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 物体工厂（工厂模式）
 * 
 * 解耦 Object 基类与具体子类的依赖
 * 
 * 使用方式：
 *   auto wall = ObjectFactory::create("wall", name, inliers, coeffs, ...);
 *   auto cylinder = ObjectFactory::create("cylinder", name, inliers, coeffs, ...);
 */
class ObjectFactory {
public:
    /**
     * @brief 创建墙体对象
     */
    static ObjectPtr createWall(
        const std::string &name,
        typename pcl::PointIndices::Ptr inliers,
        typename pcl::ModelCoefficients::Ptr coefficients,
        double extraction_time,
        float width, float height, float depth,
        bool using_normal = true) {
        
        auto wall = std::make_shared<Wall>(
            name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        return std::static_pointer_cast<Object>(wall);
    }

    /**
     * @brief 创建圆柱对象
     */
    static ObjectPtr createCylinder(
        const std::string &name,
        typename pcl::PointIndices::Ptr inliers,
        typename pcl::ModelCoefficients::Ptr coefficients,
        double extraction_time,
        float width, float height, float depth,
        bool using_normal = true) {
        
        auto cylinder = std::make_shared<Cylinder>(
            name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        return std::static_pointer_cast<Object>(cylinder);
    }

    /**
     * @brief 创建圆环对象
     */
    static ObjectPtr createCircle(
        const std::string &name,
        typename pcl::PointIndices::Ptr inliers,
        typename pcl::ModelCoefficients::Ptr coefficients,
        double extraction_time,
        float width, float height, float depth,
        bool using_normal = false) {
        
        auto circle = std::make_shared<Circle>(
            name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        return std::static_pointer_cast<Object>(circle);
    }

    /**
     * @brief 创建矩形对象
     */
    static ObjectPtr createRectangle(
        const std::string &name,
        typename pcl::PointIndices::Ptr inliers,
        typename pcl::ModelCoefficients::Ptr coefficients,
        double extraction_time,
        float width, float height, float depth,
        bool using_normal = false) {
        
        auto rectangle = std::make_shared<Rectangle>(
            name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        return std::static_pointer_cast<Object>(rectangle);
    }

    /**
     * @brief 创建障碍物对象
     */
    static ObjectPtr createObstacle(
        const std::string &name,
        typename pcl::PointIndices::Ptr inliers,
        typename pcl::ModelCoefficients::Ptr coefficients,
        double extraction_time,
        float width, float height, float depth) {
        
        auto obstacle = std::make_shared<Obstacle>(
            name, inliers, coefficients, extraction_time, width, height, depth);
        return std::static_pointer_cast<Object>(obstacle);
    }

    /**
     * @brief 通过类型名称创建对象（通用工厂方法）
     * @param type 类型名称 ("wall", "cylinder", "circle", "rectangle", "obstacle")
     * @param name 对象名称
     * @param inliers 点云索引
     * @param coefficients 模型系数
     * @param extraction_time 提取时间
     * @param width, height, depth 尺寸
     * @param using_normal 是否使用法向量
     * @return 对象实例
     */
    static ObjectPtr create(
        const std::string& type,
        const std::string& name,
        typename pcl::PointIndices::Ptr inliers,
        typename pcl::ModelCoefficients::Ptr coefficients,
        double extraction_time,
        float width, float height, float depth,
        bool using_normal = false) {
        
        if (type == "wall") {
            return createWall(name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        } else if (type == "cylinder") {
            return createCylinder(name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        } else if (type == "circle") {
            return createCircle(name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        } else if (type == "rectangle") {
            return createRectangle(name, inliers, coefficients, extraction_time, width, height, depth, using_normal);
        } else if (type == "obstacle") {
            return createObstacle(name, inliers, coefficients, extraction_time, width, height, depth);
        } else {
            ROS_ERROR("Unknown object type: %s", type.c_str());
            return nullptr;
        }
    }
};

}  // namespace core
}  // namespace pcl_object_detection
