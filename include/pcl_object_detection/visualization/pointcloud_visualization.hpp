#pragma once

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>
#include <iomanip>
#include <string>
#include <thread>
#include <vector>

namespace pcl_object_detection
{
    namespace visualization
    {

        // 可视化配置
        struct VisualizationConfig
        {
            bool show_walls     = true;
            bool show_cylinders = true;
            bool show_circles   = true;
            bool auto_camera    = true;
        };

        // 可视化点云
        template <typename PointT, typename ObjectInfo> class PointCloudVisualizer
        {
          public:
            using PointCloudT    = pcl::PointCloud<PointT>;
            using PointCloudPtrT = typename PointCloudT::Ptr;

            PointCloudVisualizer(const std::string &window_name = "Object Detection") {
                viewer_.reset(new pcl::visualization::PCLVisualizer(window_name));
                viewer_->setBackgroundColor(0, 0, 0);
                viewer_->addCoordinateSystem(0.5);
                viewer_->initCameraParameters();
            }

            // 添加墙面
            // template <typename WallInfoT>
            // void addWalls(PointCloudPtrT cloud, const std::vector<WallInfoT> &walls,
            //               const VisualizationConfig &config = VisualizationConfig()) {
            //     if (!config.show_walls || walls.empty()) return;

            //     // 使用更明亮、对比度更高的颜色，避免黑色
            //     const std::vector<Eigen::Vector3f> wall_colors = {
            //         {1.0, 0.0, 0.0},  // 红
            //         {0.0, 0.0, 1.0},  // 蓝
            //         {1.0, 1.0, 0.0},  // 黄
            //         {1.0, 0.0, 1.0},  // 紫
            //         {0.0, 1.0, 1.0},  // 青
            //         {1.0, 0.5, 0.0},  // 橙
            //         {0.5, 0.0, 1.0},  // 紫罗兰
            //         {0.0, 0.5, 0.0},  // 深绿
            //     };

            //     for (size_t i = 0; i < walls.size(); ++i) {
            //         const auto &wall = walls[i];

            //         // 确保使用对象自身的颜色，而不是固定数组
            //         Eigen::Vector3f color;
            //         if (i < wall_colors.size()) {
            //             // 仅当对象没有预设颜色时使用wall_colors
            //             if (wall.color == Eigen::Vector3f(0, 0, 0)) {
            //                 color = wall_colors[i % wall_colors.size()];
            //             }
            //             else {
            //                 color = wall.color;
            //             }
            //         }
            //         else {
            //             // 如果对象数量超过预定义颜色，使用循环且避免黑色
            //             size_t idx = i % wall_colors.size();
            //             color      = wall_colors[idx];
            //         }

            //         // 确保颜色不为黑色（RGB值至少有一个大于0.1）
            //         if (color[0] < 0.1 && color[1] < 0.1 && color[2] < 0.1) {
            //             // 如果接近黑色，使用默认红色
            //             color = Eigen::Vector3f(1.0, 0.0, 0.0);
            //         }

            //         PointCloudPtrT wall_cloud(new PointCloudT);
            //         pcl::ExtractIndices<PointT> extract;
            //         extract.setInputCloud(cloud);
            //         extract.setIndices(wall.inliers);
            //         extract.filter(*wall_cloud);

            //         std::string wall_id = "wall_" + std::to_string(i);
            //         viewer_->addPointCloud<PointT>(wall_cloud, wall_id);
            //         viewer_->setPointCloudRenderingProperties(
            //             pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, wall_id);

            //         viewer_->setPointCloudRenderingProperties(
            //             pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2],
            //             wall_id);
            //     }
            // }

            // 添加墙面
            template <typename WallInfoT>
            void addWalls(PointCloudPtrT cloud, const std::vector<WallInfoT> &walls,
                          const VisualizationConfig &config = VisualizationConfig()) {
                if (!config.show_walls || walls.empty()) return;

                for (size_t i = 0; i < walls.size(); ++i) {
                    const auto &wall      = walls[i];

                    // 确保颜色在[0,1]范围内
                    Eigen::Vector3f color = wall.color;
                    color[0]              = std::max(0.0f, std::min(1.0f, color[0]));
                    color[1]              = std::max(0.0f, std::min(1.0f, color[1]));
                    color[2]              = std::max(0.0f, std::min(1.0f, color[2]));

                    // 调试输出
                    std::cout << "添加墙面 " << i << " - 点数: " << wall.inliers->indices.size()
                              << ", 颜色: (" << color[0] << ", " << color[1] << ", " << color[2]
                              << ")\n";

                    PointCloudPtrT wall_cloud(new PointCloudT);
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud);
                    extract.setIndices(wall.inliers);
                    extract.filter(*wall_cloud);

                    // 调试输出
                    std::cout << "wall_cloud size: " << wall_cloud->size() << std::endl;
                    std::cout << "First point: (" << wall_cloud->points[0].x << ", "
                              << wall_cloud->points[0].y << ", " << wall_cloud->points[0].z << ")"
                              << std::endl;

                    // 关键修复：明确指定点云类型
                    std::string wall_id = "wall_" + std::to_string(i);
                    viewer_->addPointCloud<PointT>(wall_cloud, wall_id);
                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, wall_id);

                    // 确保颜色值在[0,1]范围内
                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2],
                        wall_id);
                }
            }

            // 添加圆柱
            template <typename CylinderInfoT>
            void addCylinders(PointCloudPtrT cloud, const std::vector<CylinderInfoT> &cylinders,
                              const VisualizationConfig &config = VisualizationConfig()) {
                if (!config.show_cylinders || cylinders.empty()) return;

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

                for (size_t i = 0; i < cylinders.size(); ++i) {
                    const auto &cylinder = cylinders[i];

                    Eigen::Vector3f color;
                    if (i < cylinder_colors.size()) {
                        if (cylinder.color == Eigen::Vector3f(0, 0, 0)) {
                            color = cylinder_colors[i % cylinder_colors.size()];
                        }
                        else {
                            color = cylinder.color;
                        }
                    }
                    else {
                        size_t idx = i % cylinder_colors.size();
                        color      = cylinder_colors[idx];
                    }

                    if (color[0] < 0.1 && color[1] < 0.1 && color[2] < 0.1) {
                        color = Eigen::Vector3f(1.0, 0.5, 0.0);  // 默认橙色
                    }

                    PointCloudPtrT cylinder_cloud(new PointCloudT);
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud);
                    extract.setIndices(cylinder.inliers);
                    extract.filter(*cylinder_cloud);

                    std::string cylinder_id = "cylinder_" + std::to_string(i);
                    viewer_->addPointCloud<PointT>(cylinder_cloud, cylinder_id);
                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cylinder_id);

                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2],
                        cylinder_id);
                }
            }

            // 添加圆
            template <typename CircleInfoT>
            void addCircles(PointCloudPtrT cloud, const std::vector<CircleInfoT> &circles,
                            const VisualizationConfig &config = VisualizationConfig()) {
                if (!config.show_circles || circles.empty()) return;

                // 完全移除黑色，使用更明亮的颜色
                const std::vector<Eigen::Vector3f> circle_colors = {
                    {0.5, 0.5, 1.0},   // 浅蓝
                    {1.0, 0.5, 0.5},   // 浅红
                    {0.5, 1.0, 0.5},   // 浅绿
                    {1.0, 0.5, 1.0},   // 浅紫
                    {0.5, 1.0, 1.0},   // 浅青
                    {1.0, 1.0, 0.5},   // 浅黄
                    {1.0, 0.75, 0.5},  // 浅橙
                };

                for (size_t i = 0; i < circles.size(); ++i) {
                    const auto &circle = circles[i];

                    Eigen::Vector3f color;
                    if (i < circle_colors.size()) {
                        if (circle.color == Eigen::Vector3f(0, 0, 0)) {
                            color = circle_colors[i % circle_colors.size()];
                        }
                        else {
                            color = circle.color;
                        }
                    }
                    else {
                        size_t idx = i % circle_colors.size();
                        color      = circle_colors[idx];
                    }

                    if (color[0] < 0.1 && color[1] < 0.1 && color[2] < 0.1) {
                        color = Eigen::Vector3f(0.5, 0.5, 1.0);  // 默认浅蓝
                    }

                    PointCloudPtrT circle_cloud(new PointCloudT);
                    pcl::ExtractIndices<PointT> extract;
                    extract.setInputCloud(cloud);
                    extract.setIndices(circle.inliers);
                    extract.filter(*circle_cloud);

                    std::string circle_id = "circle_" + std::to_string(i);
                    viewer_->addPointCloud<PointT>(circle_cloud, circle_id);
                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, circle_id);

                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2],
                        circle_id);
                }
            }

            // 调整相机视角
            void adjustCamera(PointCloudPtrT cloud, bool auto_camera = true) {
                if (!auto_camera) return;

                // 计算所有对象的中心点
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D(*cloud, min_pt, max_pt);

                // 调试输出点云范围
                std::cout << "点云范围: min=(" << min_pt[0] << ", " << min_pt[1] << ", "
                          << min_pt[2] << "), max=(" << max_pt[0] << ", " << max_pt[1] << ", "
                          << max_pt[2] << ")" << std::endl;

                Eigen::Vector3f center((min_pt[0] + max_pt[0]) / 2, (min_pt[1] + max_pt[1]) / 2,
                                       (min_pt[2] + max_pt[2]) / 2);

                // 计算点云范围
                float x_range         = max_pt[0] - min_pt[0];
                float y_range         = max_pt[1] - min_pt[1];
                float z_range         = max_pt[2] - min_pt[2];
                float max_range       = std::max({x_range, y_range, z_range});

                // 动态计算相机距离，确保能看到整个场景
                float camera_distance = max_range * 1.5;

                // 关键修复：调整相机角度，不要垂直向下
                // 从XZ平面45度角观察，这样能看到墙面的宽度和高度
                float camera_x        = center[0] + camera_distance * cos(M_PI / 4.0);
                float camera_y        = center[1] + camera_distance * sin(M_PI / 4.0);
                float camera_z        = center[2] + camera_distance * 0.5;

                // 调整相机视角
                viewer_->setCameraPosition(camera_x, camera_y, camera_z,     // 相机位置
                                           center[0], center[1], center[2],  // 看向的点
                                           0, 0, 1                           // 上方向
                );

                // 调试输出
                std::cout << "相机位置: (" << camera_x << ", " << camera_y << ", " << camera_z
                          << ")" << std::endl;
                std::cout << "相机距离: " << camera_distance << std::endl;
            }

            // 添加信息文本
            void addInfoText(const std::vector<std::string> &wall_names,
                             const std::vector<std::string> &cylinder_names,
                             const std::vector<std::string> &circle_names) {
                std::stringstream ss;
                if (!wall_names.empty() || !cylinder_names.empty() || !circle_names.empty()) {
                    ss << "Detected:\n";
                    if (!wall_names.empty()) ss << "Walls: " << wall_names.size() << "\n";
                    if (!cylinder_names.empty())
                        ss << "Cylinders: " << cylinder_names.size() << "\n";
                    if (!circle_names.empty()) ss << "Circles: " << circle_names.size() << "\n";
                }
                else {
                    ss << "No objects detected";
                }
                viewer_->addText(ss.str(), 10, 10, 12, 1.0, 1.0, 1.0, "info_text");
            }

            // 显示并等待
            void spin() {
                while (!viewer_->wasStopped()) {
                    viewer_->spinOnce(100);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            // 获取底层viewer
            pcl::visualization::PCLVisualizer &getViewer() { return *viewer_; }

          private:
            std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
        };

    }  // namespace visualization
}  // namespace pcl_object_detection