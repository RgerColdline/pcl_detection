#pragma once

#include "circle_extraction.hpp"
#include "cylinder_extraction.hpp"
#include "wall_extraction.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <chrono>
#include <memory>
#include <vector>

namespace pcl_object_detection
{
    namespace core
    {
        // 时间测量工具类
        class Timer
        {
          public:
            Timer() : start_(std::chrono::high_resolution_clock::now()) {}

            double elapsed() const {
                auto end = std::chrono::high_resolution_clock::now();
                return std::chrono::duration<double, std::milli>(end - start_).count();
            }

          private:
            std::chrono::high_resolution_clock::time_point start_;
        };

        // 统一的配置结构
        struct TimingConfig
        {
            bool enable     = false;
            bool downsample = true;
            bool walls      = true;
            bool cylinders  = true;
            bool circles    = true;
            bool total      = true;
        };

        struct ObjectDetectionConfig
        {
            struct
            {
                std::string downsample_method = "standard";  // "standard" or "approx"
                float leaf_size               = 0.05f;
            } downsample_config;

            WallConfig wall_config;
            CylinderConfig cylinder_config;
            CircleConfig circle_config;

            TimingConfig timing_config;

            bool save_objects      = false;
            std::string output_dir = "./objects/";
        };

        // 下采样点云
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr
        downsamplePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                             const std::string &downsample_method, float leaf_size) {
            typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

            if (downsample_method == "approx") {
                pcl::ApproximateVoxelGrid<PointT> voxel;
                voxel.setInputCloud(cloud);
                voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
                voxel.filter(*cloud_filtered);
            }
            else {
                pcl::VoxelGrid<PointT> voxel;
                voxel.setInputCloud(cloud);
                voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
                voxel.filter(*cloud_filtered);
            }

            return cloud_filtered;
        }

        // 估计法向量
        template <typename PointT>
        typename pcl::PointCloud<pcl::Normal>::Ptr
        estimateNormals(typename pcl::PointCloud<PointT>::Ptr cloud, int k_search = 50) {
            typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            ne.setInputCloud(cloud);
            typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
            ne.setSearchMethod(tree);
            ne.setKSearch(k_search);
            ne.compute(*normals);

            return normals;
        }

        // 对象检测流水线
        template <typename PointT> struct ObjectDetectionPipeline
        {
            using WallInfoT     = WallInfo<PointT>;
            using CylinderInfoT = CylinderInfo<PointT>;
            using CircleInfoT   = CircleInfo<PointT>;

            ObjectDetectionConfig config;

            // 原始点云
            typename pcl::PointCloud<PointT>::Ptr input_cloud;
            // 下采样后的点云
            typename pcl::PointCloud<PointT>::Ptr filtered_cloud;
            // 法向量（如果需要）
            typename pcl::PointCloud<pcl::Normal>::Ptr normals;

            // 检测到的对象
            std::vector<WallInfoT> walls;
            std::vector<CylinderInfoT> cylinders;
            std::vector<CircleInfoT> circles;

            // 时间测量结果
            double downsample_time_          = 0.0;
            double wall_extraction_time_     = 0.0;
            double cylinder_extraction_time_ = 0.0;
            double circle_extraction_time_   = 0.0;
            double total_time_               = 0.0;

            // 运行整个流水线
            bool run(typename pcl::PointCloud<PointT>::Ptr input) {
                Timer total_timer;
                input_cloud = input;

                // 1. 下采样
                Timer downsample_timer;
                filtered_cloud = downsamplePointCloud<PointT>(
                    input_cloud, config.downsample_config.downsample_method,
                    config.downsample_config.leaf_size);
                downsample_time_ = downsample_timer.elapsed();

                // 2. 估计法向量（如果需要）
                bool need_normals =
                    config.wall_config.using_normal || config.cylinder_config.using_normal;
                if (need_normals) {
                    normals = estimateNormals<PointT>(filtered_cloud);
                }

                // 3. 提取墙面
                if (config.wall_config.enable) {
                    Timer wall_timer;
                    walls = extractWalls<PointT>(filtered_cloud, normals, config.wall_config);
                    wall_extraction_time_ = wall_timer.elapsed();
                }

                // 4. 提取圆柱
                if (config.cylinder_config.enable) {
                    Timer cylinder_timer;
                    cylinders =
                        extractCylinders<PointT>(filtered_cloud, normals, config.cylinder_config);
                    cylinder_extraction_time_ = cylinder_timer.elapsed();
                }

                // 5. 提取圆
                if (config.circle_config.enable) {
                    Timer circle_timer;
                    circles = extractCircles<PointT>(filtered_cloud, config.circle_config);
                    circle_extraction_time_ = circle_timer.elapsed();
                }
                total_time_ = total_timer.elapsed();


                return true;
            }
        };

    }  // namespace core
}  // namespace pcl_object_detection