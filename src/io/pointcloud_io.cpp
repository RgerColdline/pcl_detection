#include "pcl_object_detection/io/pointcloud_io.hpp"

#include "pcl_object_detection/core/config.hpp"

namespace pcl_object_detection
{
namespace io
{

// 从YAML文件加载配置
bool load_config(const std::string &config_file, core::ObjectDetectionConfig &config) {
    try {
        YAML::Node node = YAML::LoadFile(config_file);

        // 时间测量配置
        if (node["timing"]) {
            config.timing_config.enable     = node["timing"]["enable"].as<bool>(false);
            config.timing_config.downsample = node["timing"]["downsample"].as<bool>(true);
            config.timing_config.walls      = node["timing"]["walls"].as<bool>(true);
            config.timing_config.cylinders  = node["timing"]["cylinders"].as<bool>(true);
            config.timing_config.circles    = node["timing"]["circles"].as<bool>(true);
            config.timing_config.total      = node["timing"]["total"].as<bool>(true);
        }

        // 下采样配置
        if (node["downsample_config"]) {
            config.downsample_config.approx =
                node["downsample_config"]["approx"].as<bool>(false);
            config.downsample_config.leaf_size =
                node["downsample_config"]["leaf_size"].as<float>(0.05f);
        }

        // 加载墙面配置
        if (node["wall_extraction"]) {
            config.wall_config.enable = node["wall_extraction"]["enable"].as<bool>(true);
            config.wall_config.using_normal =
                node["wall_extraction"]["using_normal"].as<bool>(true);
            config.wall_config.distance_threshold =
                node["wall_extraction"]["distance_threshold"].as<float>(0.02f);
            config.wall_config.min_inliers = node["wall_extraction"]["min_inliers"].as<int>(200);
            config.wall_config.angle_threshold =
                node["wall_extraction"]["angle_threshold"].as<float>(15.0f);

            // 加载轴向
            if (node["wall_extraction"]["axis"] && node["wall_extraction"]["axis"].IsSequence() &&
                node["wall_extraction"]["axis"].size() == 3)
            {
                config.wall_config.axis = node["wall_extraction"]["axis"].as<std::vector<float>>();
            }
            else {
                config.wall_config.axis = {0.0f, 0.0f, 1.0f};  // 默认Z轴
            }
        }

        // 加载圆柱配置
        if (node["cylinder_extraction"]) {
            config.cylinder_config.enable = node["cylinder_extraction"]["enable"].as<bool>(true);
            config.cylinder_config.using_normal =
                node["cylinder_extraction"]["using_normal"].as<bool>(true);
            config.cylinder_config.distance_threshold =
                node["cylinder_extraction"]["distance_threshold"].as<float>(0.02f);
            config.cylinder_config.min_inliers =
                node["cylinder_extraction"]["min_inliers"].as<int>(200);
            config.cylinder_config.radius_min =
                node["cylinder_extraction"]["radius_min"].as<float>(0.05f);
            config.cylinder_config.radius_max =
                node["cylinder_extraction"]["radius_max"].as<float>(0.2f);

            // 加载轴向
            if (node["cylinder_extraction"]["axis"] &&
                node["cylinder_extraction"]["axis"].IsSequence() &&
                node["cylinder_extraction"]["axis"].size() == 3)
            {
                config.cylinder_config.axis =
                    node["cylinder_extraction"]["axis"].as<std::vector<float>>();
            }
            else {
                config.cylinder_config.axis = {0.0f, 0.0f, 1.0f};  // 默认Z轴
            }

            config.cylinder_config.eps_angle =
                node["cylinder_extraction"]["eps_angle"].as<float>(15.0f);
        }

        // 加载圆环配置
        if (node["circle_extraction"]) {
            config.circle_config.enable = node["circle_extraction"]["enable"].as<bool>(true);
            config.circle_config.using_normal =
                node["circle_extraction"]["using_normal"].as<bool>(true);
            config.circle_config.distance_threshold =
                node["circle_extraction"]["distance_threshold"].as<float>(0.02f);
            config.circle_config.min_inliers =
                node["circle_extraction"]["min_inliers"].as<int>(100);
            config.circle_config.radius_min =
                node["circle_extraction"]["radius_min"].as<float>(0.2f);
            config.circle_config.radius_max =
                node["circle_extraction"]["radius_max"].as<float>(0.7f);
            // 平面提取参数
            config.circle_config.plane_distance_threshold =
                node["circle_extraction"]["plane_distance_threshold"].as<float>(0.05f);
            config.circle_config.plane_angle_threshold =
                node["circle_extraction"]["plane_angle_threshold"].as<float>(45.0f);
            config.circle_config.plane_normal_z_max =
                node["circle_extraction"]["plane_normal_z_max"].as<float>(0.7f);
            // 圆环验证参数
            config.circle_config.min_coverage_angle =
                node["circle_extraction"]["min_coverage_angle"].as<float>(270.0f);
            config.circle_config.ransac_iterations =
                node["circle_extraction"]["ransac_iterations"].as<int>(500);
            config.circle_config.max_circles = node["circle_extraction"]["max_circles"].as<int>(10);
            config.circle_config.normal_tolerance =
                node["circle_extraction"]["normal_tolerance"].as<float>(0.5f);
        }

        // 其他配置
        config.save_objects = node["save_objects"].as<bool>(false);
        config.output_dir   = node["output_dir"].as<std::string>("./objects/");

        return true;
    }
    catch (const std::exception &e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

}  // namespace io
}  // namespace pcl_object_detection