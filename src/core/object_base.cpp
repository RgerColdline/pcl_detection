#include "pcl_object_detection/core/object_base.hpp"

#include "pcl_object_detection/core/circle.hpp"
#include "pcl_object_detection/core/cylinder.hpp"
#include "pcl_object_detection/core/wall.hpp"

namespace pcl_object_detection
{
namespace core
{

Object::Ptr Object::createWall(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                               typename pcl::ModelCoefficients::Ptr coefficients,
                               double extraction_time, float width, float height, float depth,
                               bool using_normal) {
    auto wall = std::make_shared<Wall>(name, inliers, coefficients, extraction_time, using_normal);
    wall->width  = width;
    wall->height = height;
    wall->depth  = depth;
    return wall;
}

Object::Ptr Object::createCylinder(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                                   typename pcl::ModelCoefficients::Ptr coefficients,
                                   double extraction_time, float width, float height, float depth,
                                   bool using_normal) {
    auto cylinder =
        std::make_shared<Cylinder>(name, inliers, coefficients, extraction_time, using_normal);
    cylinder->width  = width;
    cylinder->height = height;
    cylinder->depth  = depth;
    return cylinder;
}

Object::Ptr Object::createCircle(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                                 typename pcl::ModelCoefficients::Ptr coefficients,
                                 double extraction_time, float width, float height, float depth,
                                 bool using_normal) {
    auto circle =
        std::make_shared<Circle>(name, inliers, coefficients, extraction_time, using_normal);
    circle->width  = width;
    circle->height = height;
    circle->depth  = depth;
    return circle;
}

}  // namespace core
}  // namespace pcl_object_detection