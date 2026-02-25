#pragma once

#include "core/pipeline.hpp"
#include "io/pointcloud_io.hpp"
#include "visualization/pointcloud_visualization.hpp"

#include <iomanip>
#include <iostream>

namespace pcl_object_detection
{

// 应用程序流水线
template <typename PointT> class ApplicationPipeline
{
  public:
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    // 运行应用程序
    bool run(const std::string &config_file, const std::string &input_pcd) {
        // 1. 加载配置
        if (!io::load_config(config_file, config_)) {
            std::cerr << "Failed to load config file\n";
            return false;
        }

        // 2. 加载点云
        PointCloudPtrT cloud = io::loadPointCloud<PointT>(input_pcd);
        if (!cloud) {
            return false;
        }

        std::cout << "Loaded " << cloud->size() << " points from " << input_pcd << "\n";

        // 3. 运行核心流水线
        core_pipeline_.config = config_;
        if (!core_pipeline_.run(cloud)) {
            std::cerr << "Failed to run object detection pipeline\n";
            return false;
        }

        // 4. 为检测到的对象分配颜色 - 这是设置颜色的最佳位置
        assignColorsToObject();

        // 5. 输出结果
        printResults();

        // 6. 保存对象（如果配置了）
        if (config_.save_objects) {
            saveObjects();
        }

        // 7. 可视化
        visualize();

        return true;
    }

  private:
    void assignColorsToObject() {
        // 为墙面分配颜色
        const std::vector<Eigen::Vector3f> wall_colors = {
            {1.0, 0.0, 0.0},  // 红
            {0.0, 0.0, 1.0},  // 蓝
            {1.0, 1.0, 0.0},  // 黄
            {1.0, 0.0, 1.0},  // 紫
            {0.0, 1.0, 1.0},  // 青
            {1.0, 0.5, 0.0},  // 橙
            {0.5, 0.0, 1.0},  // 紫罗兰
            {0.0, 0.5, 0.0},  // 深绿
        };

        for (size_t i = 0; i < core_pipeline_.walls.size(); ++i) {
            // 直接修改core_pipeline_中的对象
            core_pipeline_.walls[i].color = wall_colors[i % wall_colors.size()];
        }

        // 为圆柱分配颜色
        const std::vector<Eigen::Vector3f> cylinder_colors = {
            {1.0, 0.5, 0.0},  // 橙色
            {0.5, 0.0, 1.0},  // 紫罗兰
            {0.0, 0.5, 0.0},  // 深绿
            {1.0, 0.0, 0.5},  // 玫瑰红
            {0.5, 0.5, 0.0},  // 橄榄色
            {1.0, 0.0, 0.0},  // 红
            {0.0, 0.0, 1.0},  // 蓝
            {1.0, 1.0, 0.0},  // 黄
        };

        for (size_t i = 0; i < core_pipeline_.cylinders.size(); ++i) {
            core_pipeline_.cylinders[i].color = cylinder_colors[i % cylinder_colors.size()];
        }

        // 为圆分配颜色
        const std::vector<Eigen::Vector3f> circle_colors = {
            {0.5, 0.5, 1.0},   // 浅蓝
            {1.0, 0.5, 0.5},   // 浅红
            {0.5, 1.0, 0.5},   // 浅绿
            {1.0, 0.5, 1.0},   // 浅紫
            {0.5, 1.0, 1.0},   // 浅青
            {1.0, 1.0, 0.5},   // 浅黄
            {1.0, 0.75, 0.5},  // 浅橙
        };

        for (size_t i = 0; i < core_pipeline_.circles.size(); ++i) {
            core_pipeline_.circles[i].color = circle_colors[i % circle_colors.size()];
        }
    }

    void printResults() {
        std::cout << "\n=== 对象提取结果 ===\n";

        // 墙面
        if (!core_pipeline_.walls.empty()) {
            std::cout << "总墙面数: " << core_pipeline_.walls.size() << "\n";

            for (size_t i = 0; i < core_pipeline_.walls.size(); ++i) {
                const auto &wall = core_pipeline_.walls[i];
                std::cout << "\n" << wall.name << ":\n";
                std::cout << "  点数: " << wall.inliers->indices.size() << "\n";
                std::cout << "  提取时间: " << wall.extraction_time << " ms\n";
                std::cout << "  平面方程: " << wall.coefficients->values[0] << "x + "
                          << wall.coefficients->values[1] << "y + " << wall.coefficients->values[2]
                          << "z + " << wall.coefficients->values[3] << " = 0\n";

                // 将颜色从0.0-1.0转换为0-255格式显示
                int r                             = static_cast<int>(wall.color[0] * 255);
                int g                             = static_cast<int>(wall.color[1] * 255);
                int b                             = static_cast<int>(wall.color[2] * 255);

                // 保存原始格式状态
                std::ios::fmtflags original_flags = std::cout.flags();

                // 使用setw和setfill格式化输出
                std::cout << "  颜色: #" << std::hex << std::setw(2) << std::setfill('0')
                          << std::uppercase << r << std::setw(2) << std::setfill('0') << g
                          << std::setw(2) << std::setfill('0') << b << " (RGB: " << r << ", " << g
                          << ", " << b << ")\n";
                std::cout << "  使用法向量: " << (wall.using_normal ? "是" : "否") << "\n";

                // 恢复原始格式状态
                std::cout.flags(original_flags);

                // 获取墙面尺寸
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D(*core_pipeline_.filtered_cloud, wall.inliers->indices, min_pt,
                                 max_pt);

                float width  = std::sqrt(std::pow(max_pt[0] - min_pt[0], 2) +
                                         std::pow(max_pt[1] - min_pt[1], 2));
                float height = max_pt[2] - min_pt[2];

                std::cout << "  近似尺寸: 宽度=" << width << "m, 高度=" << height << "m\n";
            }
        }

        // 圆柱
        if (!core_pipeline_.cylinders.empty()) {
            std::cout << "\n总圆柱数: " << core_pipeline_.cylinders.size() << "\n";

            for (size_t i = 0; i < core_pipeline_.cylinders.size(); ++i) {
                const auto &cylinder = core_pipeline_.cylinders[i];
                std::cout << "\n" << cylinder.name << ":\n";
                std::cout << "  点数: " << cylinder.inliers->indices.size() << "\n";
                std::cout << "  提取时间: " << cylinder.extraction_time << " ms\n";

                // 系数格式：[point_on_axis.x, point_on_axis.y, point_on_axis.z,
                // axis_direction.x, axis_direction.y, axis_direction.z, radius]
                std::cout << "  圆柱参数: 中心=(" << cylinder.coefficients->values[0] << ", "
                          << cylinder.coefficients->values[1] << ", "
                          << cylinder.coefficients->values[2] << "), "
                          << "轴向=(" << cylinder.coefficients->values[3] << ", "
                          << cylinder.coefficients->values[4] << ", "
                          << cylinder.coefficients->values[5] << "), "
                          << "半径=" << cylinder.coefficients->values[6] << "\n";

                // 将颜色从0.0-1.0转换为0-255格式显示
                int r                             = static_cast<int>(cylinder.color[0] * 255);
                int g                             = static_cast<int>(cylinder.color[1] * 255);
                int b                             = static_cast<int>(cylinder.color[2] * 255);

                // 保存原始格式状态
                std::ios::fmtflags original_flags = std::cout.flags();

                // 使用setw和setfill格式化输出
                std::cout << "  颜色: #" << std::hex << std::setw(2) << std::setfill('0')
                          << std::uppercase << r << std::setw(2) << std::setfill('0') << g
                          << std::setw(2) << std::setfill('0') << b << " (RGB: " << r << ", " << g
                          << ", " << b << ")\n";
                std::cout << "  使用法向量: " << (cylinder.using_normal ? "是" : "否") << "\n";

                // 恢复原始格式状态
                std::cout.flags(original_flags);

                // 获取圆柱尺寸
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D(*core_pipeline_.filtered_cloud, cylinder.inliers->indices, min_pt,
                                 max_pt);

                float height = max_pt[2] - min_pt[2];
                float radius = cylinder.coefficients->values[6];

                std::cout << "  近似尺寸: 半径=" << radius << "m, 高度=" << height << "m\n";
                std::cout << "  表面积: " << (2 * M_PI * radius * height) << " m²\n";
                std::cout << "  体积: " << (M_PI * radius * radius * height) << " m³\n";
            }
        }

        // 圆
        if (!core_pipeline_.circles.empty()) {
            std::cout << "\n总圆环数: " << core_pipeline_.circles.size() << "\n";

            for (size_t i = 0; i < core_pipeline_.circles.size(); ++i) {
                const auto &circle = core_pipeline_.circles[i];
                std::cout << "\n" << circle.name << ":\n";
                std::cout << "  点数: " << circle.inliers->indices.size() << "\n";
                std::cout << "  提取时间: " << circle.extraction_time << " ms\n";

                // 系数格式：[x, y, radius]
                std::cout << "  圆环参数: 中心=(" << circle.coefficients->values[0] << ", "
                          << circle.coefficients->values[1] << ", 0), "
                          << "半径=" << circle.coefficients->values[2] << "\n";

                // 将颜色从0.0-1.0转换为0-255格式显示
                int r                             = static_cast<int>(circle.color[0] * 255);
                int g                             = static_cast<int>(circle.color[1] * 255);
                int b                             = static_cast<int>(circle.color[2] * 255);

                // 保存原始格式状态
                std::ios::fmtflags original_flags = std::cout.flags();

                // 使用setw和setfill格式化输出
                std::cout << "  颜色: #" << std::hex << std::setw(2) << std::setfill('0')
                          << std::uppercase << r << std::setw(2) << std::setfill('0') << g
                          << std::setw(2) << std::setfill('0') << b << " (RGB: " << r << ", " << g
                          << ", " << b << ")\n";
                std::cout << "  使用法向量: " << (circle.using_normal ? "是" : "否") << "\n";

                // 恢复原始格式状态
                std::cout.flags(original_flags);

                // 获取圆环尺寸
                float radius = circle.coefficients->values[2];

                std::cout << "  近似尺寸: 半径=" << radius << "m\n";
                std::cout << "  周长: " << (2 * M_PI * radius) << " m\n";
                std::cout << "  面积: " << (M_PI * radius * radius) << " m²\n";
            }
        }
        // 显示下采样时间（如果启用）
        if (config_.timing_config.enable && config_.timing_config.downsample) {
            std::cout << "\n下采样: " << core_pipeline_.downsample_time_ << " ms\n";
        }

        // 显示墙面提取时间（如果启用）
        if (config_.timing_config.enable && config_.timing_config.walls) {
            std::cout << "\n墙面提取: " << core_pipeline_.wall_extraction_time_ << " ms\n";
        }

        // 显示圆柱提取时间（如果启用）
        if (config_.timing_config.enable && config_.timing_config.cylinders) {
            std::cout << "\n圆柱提取: " << core_pipeline_.cylinder_extraction_time_ << " ms\n";
        }

        // 显示圆环提取时间（如果启用）
        if (config_.timing_config.enable && config_.timing_config.circles) {
            std::cout << "\n圆环提取: " << core_pipeline_.circle_extraction_time_ << " ms\n";
        }

        // 显示总时间（如果启用）
        if (config_.timing_config.enable && config_.timing_config.total) {
            std::cout << "\n总处理时间: " << core_pipeline_.total_time_ << " ms\n";
        }

        if (core_pipeline_.walls.empty() && core_pipeline_.cylinders.empty() &&
            core_pipeline_.circles.empty())
        {
            std::cerr << "\n警告: 未检测到任何对象！可能需要调整参数。\n";
            std::cerr << "建议: 检查点云数据是否包含墙面、圆柱或圆环\n";
        }
    }

    void saveObjects() {
        for (size_t i = 0; i < core_pipeline_.walls.size(); ++i) {
            const auto &wall = core_pipeline_.walls[i];
            io::saveObject<PointT>(core_pipeline_.filtered_cloud, wall.inliers, config_.output_dir,
                                   wall.name);
        }

        for (size_t i = 0; i < core_pipeline_.cylinders.size(); ++i) {
            const auto &cylinder = core_pipeline_.cylinders[i];
            io::saveObject<PointT>(core_pipeline_.filtered_cloud, cylinder.inliers,
                                   config_.output_dir, cylinder.name);
        }

        for (size_t i = 0; i < core_pipeline_.circles.size(); ++i) {
            const auto &circle = core_pipeline_.circles[i];
            io::saveObject<PointT>(core_pipeline_.filtered_cloud, circle.inliers,
                                   config_.output_dir, circle.name);
        }
    }

    void visualize() {
        try {
            visualization::PointCloudVisualizer<PointT, core::WallInfo<PointT>> visualizer;

            // 添加对象
            visualizer.addWalls(core_pipeline_.filtered_cloud, core_pipeline_.walls);
            visualizer.addCylinders(core_pipeline_.filtered_cloud, core_pipeline_.cylinders);
            visualizer.addCircles(core_pipeline_.filtered_cloud, core_pipeline_.circles);

            // 调整相机
            visualizer.adjustCamera(core_pipeline_.filtered_cloud);

            // 添加信息文本
            std::vector<std::string> wall_names;
            for (const auto &wall : core_pipeline_.walls) wall_names.push_back(wall.name);

            std::vector<std::string> cylinder_names;
            for (const auto &cylinder : core_pipeline_.cylinders)
                cylinder_names.push_back(cylinder.name);

            std::vector<std::string> circle_names;
            for (const auto &circle : core_pipeline_.circles) circle_names.push_back(circle.name);

            visualizer.addInfoText(wall_names, cylinder_names, circle_names);

            std::cout << "\n可视化已启动。交互说明:\n";
            std::cout << "  左键拖动: 旋转视角\n";
            std::cout << "  右键拖动: 平移视角\n";
            std::cout << "  滇轮: 缩放\n";
            std::cout << "  Q: 退出程序\n";

            visualizer.spin();
        }
        catch (const std::exception &e) {
            std::cerr << "\n可视化错误: " << e.what() << std::endl;
            std::cerr << "请确保您的图形驱动程序已正确安装。\n";
        }
    }

  private:
    core::ObjectDetectionConfig config_;
    core::ObjectDetectionPipeline<PointT> core_pipeline_;
};

}  // namespace pcl_object_detection