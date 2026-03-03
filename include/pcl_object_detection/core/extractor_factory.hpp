#pragma once

#include "extractors_base.hpp"
#include "config.hpp"
#include "wall_extraction.hpp"
#include "cylinder_extraction.hpp"
#include "circle_extraction.hpp"
#include "rectangle_extraction.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <vector>
#include <ros/ros.h>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 提取器工厂（工厂模式）
 * 
 * 解耦 Pipeline 与具体提取器类的依赖
 * 
 * 使用方式：
 *   ExtractorFactory::create<PointT>("wall", config.wall_config);
 *   ExtractorFactory::create<PointT>("cylinder", config.cylinder_config);
 */
class ExtractorFactory {
public:
    // 配置变体（支持多种配置类型）
    struct ConfigVariant {
        enum Type { NONE, WALL, CYLINDER, CIRCLE, RECTANGLE };
        Type type = NONE;
        
        // 各种配置类型
        WallConfig wall;
        CylinderConfig cylinder;
        CircleConfig circle;
        RectangleConfig rectangle;
        
        static ConfigVariant fromWall(const WallConfig& c) {
            ConfigVariant v; v.type = WALL; v.wall = c; return v;
        }
        static ConfigVariant fromCylinder(const CylinderConfig& c) {
            ConfigVariant v; v.type = CYLINDER; v.cylinder = c; return v;
        }
        static ConfigVariant fromCircle(const CircleConfig& c) {
            ConfigVariant v; v.type = CIRCLE; v.circle = c; return v;
        }
        static ConfigVariant fromRectangle(const RectangleConfig& c) {
            ConfigVariant v; v.type = RECTANGLE; v.rectangle = c; return v;
        }
    };

    /**
     * @brief 创建提取器实例
     * @param name 提取器名称 ("wall", "cylinder", "circle", "rectangle")
     * @param config 配置
     * @return 提取器实例，失败返回 nullptr
     */
    template <typename PointT>
    static std::unique_ptr<IObjectExtractor<PointT>> create(
        const std::string& name, const ConfigVariant& config) {
        
        auto it = getCreators<PointT>().find(name);
        if (it == getCreators<PointT>().end()) {
            ROS_ERROR("Extractor '%s' not registered", name.c_str());
            return nullptr;
        }
        
        try {
            return it->second(config);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to create extractor '%s': %s", name.c_str(), e.what());
            return nullptr;
        }
    }
    
    /**
     * @brief 获取所有已注册的提取器名称
     */
    template <typename PointT>
    static std::vector<std::string> getRegisteredNames() {
        std::vector<std::string> names;
        for (const auto& pair : getCreators<PointT>()) {
            names.push_back(pair.first);
        }
        return names;
    }
    
    /**
     * @brief 初始化所有内置提取器（在程序启动时调用一次）
     */
    template <typename PointT>
    static void registerBuiltins() {
        static bool registered = false;
        if (registered) return;
        
        // 墙体提取器
        getCreators<PointT>()["wall"] = [](const ConfigVariant& cfg) {
            if (cfg.type != ConfigVariant::WALL) {
                throw std::runtime_error("Wrong config type for wall extractor");
            }
            return std::make_unique<WallExtractor<PointT>>(cfg.wall);
        };
        
        // 圆柱提取器
        getCreators<PointT>()["cylinder"] = [](const ConfigVariant& cfg) {
            if (cfg.type != ConfigVariant::CYLINDER) {
                throw std::runtime_error("Wrong config type for cylinder extractor");
            }
            return std::make_unique<CylinderExtractor<PointT>>(cfg.cylinder);
        };
        
        // 圆环提取器
        getCreators<PointT>()["circle"] = [](const ConfigVariant& cfg) {
            if (cfg.type != ConfigVariant::CIRCLE) {
                throw std::runtime_error("Wrong config type for circle extractor");
            }
            return std::make_unique<CircleExtractor<PointT>>(cfg.circle);
        };
        
        // 矩形提取器
        getCreators<PointT>()["rectangle"] = [](const ConfigVariant& cfg) {
            if (cfg.type != ConfigVariant::RECTANGLE) {
                throw std::runtime_error("Wrong config type for rectangle extractor");
            }
            return std::make_unique<RectangleExtractor<PointT>>(cfg.rectangle);
        };
        
        registered = true;
        ROS_DEBUG("Registered all builtin extractors");
    }

private:
    template <typename PointT>
    using CreatorFunc = std::function<std::unique_ptr<IObjectExtractor<PointT>>(const ConfigVariant&)>;
    
    template <typename PointT>
    using CreatorMap = std::unordered_map<std::string, CreatorFunc<PointT>>;
    
    template <typename PointT>
    static CreatorMap<PointT>& getCreators() {
        static CreatorMap<PointT> creators;
        return creators;
    }
};

}  // namespace core
}  // namespace pcl_object_detection
