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
                               double extraction_time, bool using_normal) {
    return std::make_shared<Wall>(name, inliers, coefficients, extraction_time, using_normal);
}

Object::Ptr Object::createCylinder(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                                   typename pcl::ModelCoefficients::Ptr coefficients,
                                   double extraction_time, bool using_normal) {
    return std::make_shared<Cylinder>(name, inliers, coefficients, extraction_time, using_normal);
}

Object::Ptr Object::createCircle(const std::string &name, typename pcl::PointIndices::Ptr inliers,
                                 typename pcl::ModelCoefficients::Ptr coefficients,
                                 double extraction_time, bool using_normal) {
    return std::make_shared<Circle>(name, inliers, coefficients, extraction_time, using_normal);
}

}  // namespace core
}  // namespace pcl_object_detection