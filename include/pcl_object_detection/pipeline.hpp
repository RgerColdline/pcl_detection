#pragma once

#include "core/object_base.hpp"
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

        // 打印加载的配置（调试用）
        std::cout << "\n=== 加载的配置 ===\n";
        std::cout << "Wall extraction:\n";
        std::cout << "  enable: " << (config_.wall_config.enable ? "true" : "false") << "\n";
        std::cout << "  using_normal: " << (config_.wall_config.using_normal ? "true" : "false") << "\n";
        std::cout << "  distance_threshold: " << config_.wall_config.distance_threshold << "\n";
        std::cout << "  min_inliers: " << config_.wall_config.min_inliers << "\n";
        std::cout << "  angle_threshold: " << config_.wall_config.angle_threshold << "\n";
        std::cout << "Cylinder extraction:\n";
        std::cout << "  enable: " << (config_.cylinder_config.enable ? "true" : "false") << "\n";
        std::cout << "  using_normal: " << (config_.cylinder_config.using_normal ? "true" : "false") << "\n";
        std::cout << "  distance_threshold: " << config_.cylinder_config.distance_threshold << "\n";
        std::cout << "  min_inliers: " << config_.cylinder_config.min_inliers << "\n";
        std::cout << "  radius_min: " << config_.cylinder_config.radius_min << ", radius_max: " << config_.cylinder_config.radius_max << "\n";
        std::cout << "Circle extraction:\n";
        std::cout << "  enable: " << (config_.circle_config.enable ? "true" : "false") << "\n";
        std::cout << "  using_normal: " << (config_.circle_config.using_normal ? "true" : "false") << "\n";
        std::cout << "  distance_threshold: " << config_.circle_config.distance_threshold << "\n";
        std::cout << "  min_inliers: " << config_.circle_config.min_inliers << "\n";
        std::cout << "  radius_min: " << config_.circle_config.radius_min << ", radius_max: " << config_.circle_config.radius_max << "\n";
        std::cout << "===================\n\n";

        // 2. 加载点云
        PointCloudPtrT cloud = io::loadPointCloud<PointT>(input_pcd);
        if (!cloud) {
            return false;
        }

        std::cout << "Loaded " << cloud->size() << " points from " << input_pcd << "\n";

        // 3. 运行核心流水线
        core_pipeline_.config = config_;
        core::Timer total_timer;
        if (!core_pipeline_.run(cloud)) {
            std::cerr << "Failed to run object detection pipeline\n";
            return false;
        }
        double total_time = total_timer.elapsed();

        // 4. 为检测到的对象分配颜色
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
        // 获取所有对象
        auto &objects                             = core_pipeline_.objects;

        // 预定义颜色
        const std::vector<Eigen::Vector3f> colors = {
            {1.0, 0.0, 0.0},  // 红
            {0.0, 0.0, 1.0},  // 蓝
            {1.0, 1.0, 0.0},  // 黄
            {1.0, 0.0, 1.0},  // 紫
            {0.0, 1.0, 1.0},  // 青
            {1.0, 0.5, 0.0},  // 橙
            {0.5, 0.0, 1.0},  // 紫罗兰
            {0.0, 0.5, 0.0},  // 深绿
        };

        // 为每个对象分配颜色
        for (size_t i = 0; i < objects.size(); ++i) {
            objects[i]->color = colors[i % colors.size()];
        }
    }

    void printResults() {
        std::cout << "\n=== 对象提取结果 ===\n";

        // 显示总时间
        if (config_.timing_config.enable && config_.timing_config.total) {
            std::cout << "总执行时间: " << core_pipeline_.total_time_ << " ms\n";
        }

        // 显示下采样时间
        if (config_.timing_config.enable && config_.timing_config.downsample) {
            std::cout << "下采样: " << core_pipeline_.downsample_time_ << " ms\n";
        }

        // 打印所有对象
        for (const auto &obj : core_pipeline_.objects) {
            std::cout << "\n" << obj->name << " (" << obj->getType() << "):\n";
            std::cout << "  点数: " << obj->getPointCount() << "\n";

            // 显示提取时间
            if (config_.timing_config.enable) {
                std::cout << "  提取时间: " << obj->extraction_time << " ms\n";
            }

            // 打印对象特有信息
            obj->printDetails(std::cout);

            // 打印颜色
            int r                             = static_cast<int>(obj->color[0] * 255);
            int g                             = static_cast<int>(obj->color[1] * 255);
            int b                             = static_cast<int>(obj->color[2] * 255);
            std::ios::fmtflags original_flags = std::cout.flags();
            std::cout << "  颜色: #" << std::hex << std::setw(2) << std::setfill('0')
                      << std::uppercase << r << std::setw(2) << std::setfill('0') << g
                      << std::setw(2) << std::setfill('0') << b << " (RGB: " << r << ", " << g
                      << ", " << b << ")\n";
            std::cout << "  使用法向量: " << (obj->using_normal ? "是" : "否") << "\n";
            std::cout.flags(original_flags);

            // 打印尺寸
            std::cout << "  近似尺寸: ";
            if (obj->width > 0) std::cout << "宽度=" << obj->width << "m, ";
            if (obj->height > 0) std::cout << "高度=" << obj->height << "m, ";
            if (obj->depth > 0) std::cout << "深度=" << obj->depth << "m";
            std::cout << "\n";

            // 打印参数
            std::vector<float> params;
            obj->getParameters(params);
            std::cout << "  参数: ";
            for (size_t i = 0; i < params.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << params[i];
            }
            std::cout << "\n";
        }

        if (core_pipeline_.objects.empty()) {
            std::cerr << "\n警告: 未检测到任何对象！可能需要调整参数。\n";
            std::cerr << "建议: 检查点云数据是否包含墙面、圆柱或圆环\n";
        }
    }

    void saveObjects() {
        for (const auto &obj : core_pipeline_.objects) {
            io::saveObject<PointT>(core_pipeline_.filtered_cloud, obj->inliers, config_.output_dir,
                                   obj->name);
        }
    }

    void visualize() {
        try {
            visualization::PointCloudVisualizer<PointT> visualizer;

            // 添加所有对象
            for (const auto &obj : core_pipeline_.objects) {
                // 提取对象点云
                PointCloudPtrT obj_cloud(new PointCloudT);
                pcl::ExtractIndices<PointT> extract;
                extract.setInputCloud(core_pipeline_.filtered_cloud);
                extract.setIndices(obj->inliers);
                extract.filter(*obj_cloud);

                // 添加到可视化
                visualizer.addObject(obj_cloud, obj->name, obj->color);
            }

            // 调整相机
            visualizer.adjustCamera(core_pipeline_.filtered_cloud);

            // 添加信息文本
            std::stringstream ss;
            if (!core_pipeline_.objects.empty()) {
                // 统计各类型对象数量
                int walls = 0, cylinders = 0, circles = 0;
                for (const auto &obj : core_pipeline_.objects) {
                    if (obj->getType() == "Wall")
                        walls++;
                    else if (obj->getType() == "Cylinder")
                        cylinders++;
                    else if (obj->getType() == "Circle")
                        circles++;
                }

                ss << "Detected:\n";
                if (walls > 0) ss << "Walls: " << walls << "\n";
                if (cylinders > 0) ss << "Cylinders: " << cylinders << "\n";
                if (circles > 0) ss << "Circles: " << circles << "\n";

                // 添加时间信息
                if (config_.timing_config.enable) {
                    ss << "\nTiming (ms):\n";
                    ss << "Total: " << core_pipeline_.total_time_ << "\n";
                    if (config_.timing_config.downsample) {
                        ss << "Downsample: " << core_pipeline_.downsample_time_ << "\n";
                    }
                    if (config_.timing_config.walls) {
                        // 计算墙面提取总时间
                        double wall_time = 0.0;
                        for (const auto &obj : core_pipeline_.objects) {
                            if (obj->getType() == "Wall") {
                                wall_time += obj->extraction_time;
                            }
                        }
                        ss << "Walls: " << wall_time << "\n";
                    }
                    if (config_.timing_config.cylinders) {
                        // 计算圆柱提取总时间
                        double cylinder_time = 0.0;
                        for (const auto &obj : core_pipeline_.objects) {
                            if (obj->getType() == "Cylinder") {
                                cylinder_time += obj->extraction_time;
                            }
                        }
                        ss << "Cylinders: " << cylinder_time << "\n";
                    }
                    if (config_.timing_config.circles) {
                        // 计算圆环提取总时间
                        double circle_time = 0.0;
                        for (const auto &obj : core_pipeline_.objects) {
                            if (obj->getType() == "Circle") {
                                circle_time += obj->extraction_time;
                            }
                        }
                        ss << "Circles: " << circle_time << "\n";
                    }
                }
            }
            else {
                ss << "No objects detected";
            }
            visualizer.addText(ss.str(), 10, 10, 12, 1.0, 1.0, 1.0, "info_text");

            std::cout << "\n可视化已启动。交互说明:\n";
            std::cout << "  左键拖动: 旋转视角\n";
            std::cout << "  右键拖动: 平移视角\n";
            std::cout << "  滚轮: 缩放\n";
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