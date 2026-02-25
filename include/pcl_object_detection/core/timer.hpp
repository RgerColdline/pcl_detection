#pragma once

#include <chrono>

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

}  // namespace core
}  // namespace pcl_object_detection