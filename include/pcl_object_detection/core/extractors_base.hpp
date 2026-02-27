#pragma once

#include "object_base.hpp"

#include <memory>
#include <set>
#include <vector>

namespace pcl_object_detection
{
namespace core
{

// ============================================================================
// 提取器基类（策略模式）
// ============================================================================
template <typename PointT>
class IObjectExtractor {
public:
    using PointCloudPtrT = typename pcl::PointCloud<PointT>::Ptr;
    using NormalCloudPtrT = typename pcl::PointCloud<pcl::Normal>::Ptr;
    
    virtual ~IObjectExtractor() = default;
    
    // 纯虚函数：提取物体（支持传入临时缓冲区）
    virtual std::vector<Object::Ptr> extract(
        PointCloudPtrT cloud,
        NormalCloudPtrT normals,
        const std::set<int>& excluded,
        PointCloudPtrT temp_cloud1 = nullptr,
        PointCloudPtrT temp_cloud2 = nullptr) = 0;
};

}  // namespace core
}  // namespace pcl_object_detection
