#pragma once

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>
#include <string>
#include <thread>
#include <vector>

namespace pcl_object_detection
{
namespace visualization
{

// 可视化点云
template <typename PointT> class PointCloudVisualizer
{
  public:
    using PointCloudT    = pcl::PointCloud<PointT>;
    using PointCloudPtrT = typename PointCloudT::Ptr;

    PointCloudVisualizer(const std::string &window_name = "Object Detection") {
        viewer_.reset(new pcl::visualization::PCLVisualizer(window_name));
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(1.0);  // 增大坐标轴大小
        viewer_->initCameraParameters();
    }

    // 添加任意对象
    void addObject(PointCloudPtrT cloud, const std::string &name, const Eigen::Vector3f &color) {
        // 确保颜色在[0,1]范围内
        Eigen::Vector3f safe_color = color;
        safe_color[0]              = std::max(0.0f, std::min(1.0f, safe_color[0]));
        safe_color[1]              = std::max(0.0f, std::min(1.0f, safe_color[1]));
        safe_color[2]              = std::max(0.0f, std::min(1.0f, safe_color[2]));

        // 确保不是黑色
        if (safe_color[0] < 0.05f && safe_color[1] < 0.05f && safe_color[2] < 0.05f) {
            safe_color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // 默认红色
        }

        // 添加点云
        viewer_->addPointCloud<PointT>(cloud, name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                  name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  safe_color[0], safe_color[1], safe_color[2],
                                                  name);
    }

    // 调整相机视角
    void adjustCamera(PointCloudPtrT cloud, bool auto_camera = true) {
        if (!auto_camera) return;

        // 计算所有对象的中心点
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);

        // 调试输出
        std::cout << "点云范围: min=(" << min_pt[0] << ", " << min_pt[1] << ", " << min_pt[2]
                  << "), max=(" << max_pt[0] << ", " << max_pt[1] << ", " << max_pt[2] << ")"
                  << std::endl;

        Eigen::Vector3f center((min_pt[0] + max_pt[0]) / 2, (min_pt[1] + max_pt[1]) / 2,
                               (min_pt[2] + max_pt[2]) / 2);

        // 计算点云范围
        float x_range         = max_pt[0] - min_pt[0];
        float y_range         = max_pt[1] - min_pt[1];
        float z_range         = max_pt[2] - min_pt[2];
        float max_range       = std::max({x_range, y_range, z_range});

        // 动态计算相机距离
        float camera_distance = max_range * 1.5;

        // 调整相机角度，避免垂直视角
        float camera_x        = center[0] + camera_distance * cos(M_PI / 4.0);
        float camera_y        = center[1] + camera_distance * sin(M_PI / 4.0);
        float camera_z        = center[2] + camera_distance * 0.5;

        // 调整相机视角
        viewer_->setCameraPosition(camera_x, camera_y, camera_z, center[0], center[1], center[2], 0,
                                   0, 1);

        std::cout << "相机位置: (" << camera_x << ", " << camera_y << ", " << camera_z << ")"
                  << std::endl;
    }

    // 添加文本
    void addText(const std::string &text, int x, int y, int font_size, double r, double g, double b,
                 const std::string &id) {
        viewer_->addText(text, x, y, font_size, r, g, b, id);
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