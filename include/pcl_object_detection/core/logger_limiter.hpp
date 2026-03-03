#pragma once

#include <ros/ros.h>
#include <string>

namespace pcl_object_detection
{
namespace core
{

/**
 * @brief 简单的日志限流器（按帧数取模）
 * 
 * 使用方式：
 *   LOG_INIT(20);  // 每 20 帧输出一次
 *   LOG_FRAME();   // 每帧调用
 *   if (LOG_SHOULD()) { ROS_INFO("..."); }  // 满足条件时输出
 */
class LoggerLimiter
{
public:
    /**
     * @brief 初始化日志限流器
     * @param skip_frames 每 N 帧输出一次（0=每次都输出）
     */
    static void init(int skip_frames = 20) {
        instance().skip_frames_ = skip_frames;
        instance().frame_count_ = 0;
    }

    /**
     * @brief 增加帧计数（每帧调用一次）
     */
    static void incrementFrame() {
        instance().frame_count_++;
    }

    /**
     * @brief 检查是否应该输出日志
     * @return true=应该输出，false=跳过
     */
    static bool shouldLog() {
        auto& inst = instance();
        if (inst.skip_frames_ <= 0) return true;
        return (inst.frame_count_ % inst.skip_frames_) == 0;
    }

    /**
     * @brief 获取当前帧计数
     */
    static int getFrameCount() {
        return instance().frame_count_;
    }

private:
    static LoggerLimiter& instance() {
        static LoggerLimiter inst;
        return inst;
    }

    LoggerLimiter() : skip_frames_(20), frame_count_(0) {}

    int skip_frames_;
    int frame_count_;
};

}  // namespace core
}  // namespace pcl_object_detection

// ============================================================================
// 便捷宏
// ============================================================================
#define LOG_INIT(skip_frames) \
    pcl_object_detection::core::LoggerLimiter::init(skip_frames)

#define LOG_FRAME() \
    pcl_object_detection::core::LoggerLimiter::incrementFrame()

#define LOG_SHOULD() \
    pcl_object_detection::core::LoggerLimiter::shouldLog()
