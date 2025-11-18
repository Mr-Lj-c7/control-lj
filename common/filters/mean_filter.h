/**
 * @file
 * @brief Defines the MeanFilter class.
 */

#ifndef MEAN_FILTER_H
#define MEAN_FILTER_H

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>
#include <stdexcept>
#include <cassert>

namespace control {
namespace common {

/**
 * @class 均值滤波器 MeanFilter
 * @brief 
    MeanFilter类用于平滑一系列有噪声的数字，例如传感器数据或我们希望更平滑的函数输出。
    这是通过跟踪最后k个测量值（其中k是窗口大小），
    并返回除最小和最大测量值之外的所有测量值的平均值来实现的，这些测量值可能是异常值。
 */
class MeanFilter {
 public:
  /**
   * @brief 初始化滤波器，旧的测量值将被丢弃
   * @param window_size 均值滤波器窗口大小
   */
  explicit MeanFilter(const std::uint_fast8_t window_size);

  MeanFilter() = default;

  ~MeanFilter() = default;
  /**
   * @brief 更新测量值并计算滤波后的值
   * @param measurement 测量值
   */
  double Update(const double measurement);

 private:
  void RemoveEarliest();
  void Insert(const double value);
  double GetMin() const;
  double GetMax() const;
  bool ShouldPopOldestCandidate(const std::uint_fast8_t old_time) const;
  std::uint_fast8_t window_size_ = 0;
  double sum_ = 0.0;
  std::uint_fast8_t time_ = 0;
  // front = earliest
  std::deque<double> values_;
  // front = min
  std::deque<std::pair<std::uint_fast8_t, double>> min_candidates_;
  // front = max
  std::deque<std::pair<std::uint_fast8_t, double>> max_candidates_;
  bool initialized_ = false;
};
}  // namespace common
}  // namespace control
#endif
