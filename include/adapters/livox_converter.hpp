// include/pcl_detection/adapters/livox_converter.hpp
#pragma once
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace pcl_detection
{
    namespace adapters
    {

        /**
         * @brief 无人机位姿结构体（安全封装）
         */
        struct DronePose
        {
            // 1. 提供只读访问器（符合无状态适配层原则）
            [[nodiscard]] constexpr float x() const noexcept { return x_; }
            [[nodiscard]] constexpr float y() const noexcept { return y_; }
            [[nodiscard]] constexpr float yaw() const noexcept { return yaw_; }
            [[nodiscard]] constexpr bool is_valid() const noexcept {
                return std::isfinite(x_) && std::isfinite(y_) && std::isfinite(yaw_);
            }

            // 2. 禁止外部修改（适配层应是无状态转换器）
            DronePose() = default;

          private:
            // 4. 成员变量必须私有（工业级强制要求）
            float x_   = NAN;
            float y_   = NAN;
            float yaw_ = NAN;
        };
        class LivoxConverter
        {
          public:
            /// 默认高度过滤阈值 (米)
            static constexpr float k_default_height_threshold = 1.5F;

            /**
             * @brief 转换 Livox 消息为 PCL 点云（世界坐标系）
             *
             * @param livox_msg Livox 原始消息
             * @param drone_pose 无人机位姿（世界坐标系）
             * @param height_threshold 高度过滤阈值（米），仅保留 |z| <= threshold 的点
             * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 转换后的点云（世界坐标系）
             *
             * @throw std::invalid_argument 如果无人机位姿无效
             */
            [[nodiscard]] static pcl::PointCloud<pcl::PointXYZ>::Ptr
            convert_to_world(const livox_ros_driver::CustomMsg::ConstPtr &livox_msg,
                             const DronePose &drone_pose,
                             float height_threshold = k_default_height_threshold) {
                // 1. 检查位姿有效性
                if (!drone_pose.is_valid()) {
                    throw std::invalid_argument("Invalid drone pose: all values must be finite");
                }

                // 2. 创建点云容器
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
                    new pcl::PointCloud<pcl::PointXYZ>());  // ← 关键：使用PCL标准方式
                pcl_cloud->reserve(livox_msg->point_num);

                // 3. 预计算旋转参数
                const float cos_yaw = std::cos(drone_pose.yaw());
                const float sin_yaw = std::sin(drone_pose.yaw());

                // 4. 转换所有点（使用范围for循环避免指针算术）
                for (const auto &livox_point : livox_msg->points) {
                    // 4.1 过滤无效点
                    if (!std::isfinite(livox_point.x) || !std::isfinite(livox_point.y) ||
                        !std::isfinite(livox_point.z))
                    {
                        continue;
                    }

                    // 4.2 高度过滤
                    if (std::abs(livox_point.z) > height_threshold) {
                        continue;
                    }

                    // 4.3 机体坐标系 → 世界坐标系
                    const float x_world =
                        livox_point.x * cos_yaw - livox_point.y * sin_yaw + drone_pose.x();
                    const float y_world =
                        livox_point.x * sin_yaw + livox_point.y * cos_yaw + drone_pose.y();

                    // 4.4 添加到点云
                    pcl_cloud->emplace_back(x_world, y_world, livox_point.z);
                }

                // 5. 设置点云元数据
                pcl_cloud->width    = static_cast<std::uint32_t>(pcl_cloud->size());
                pcl_cloud->height   = 1;
                pcl_cloud->is_dense = true;

                return pcl_cloud;
            }
        };

    }  // namespace adapters
}  // namespace pcl_detection