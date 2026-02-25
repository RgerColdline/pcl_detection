// utils/logger.h
#pragma once
#include <ros/console.h>

#define LOG_INFO(...) ROS_INFO(__VA_ARGS__)  // NOLINT(misc-const-correctness, readability-identifier-length)
#define LOG_WARN(...) ROS_WARN(__VA_ARGS__)  // NOLINT(...)
#define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__) // NOLINT(...)
#define LOG_FATAL(...) ROS_FATAL(__VA_ARGS__) // NOLINT(...)