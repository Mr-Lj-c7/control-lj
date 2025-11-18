#include "mean_filter.h"
#include <limits>

namespace control {
namespace common { 

using MF = MeanFilter;
using uint8 = std::uint_fast8_t;
using TimedValue = std::pair<uint8, double>;

const uint8 kMaxWindowSize = std::numeric_limits<uint8>::max() / 2;

MF::MeanFilter(const uint8 window_size) : window_size_(window_size) {
  if (!(window_size_ > 0)) {
    throw std::runtime_error("[MF]:windows size must be greater than zero");
  }
  if (!(window_size_ <= kMaxWindowSize)) {
    throw std::runtime_error("[MF]:Window size exceeds maximum allowed size ");
  }
  initialized_ = true;
}

double MF::GetMin() const {
  if (min_candidates_.empty()) {
    return std::numeric_limits<double>::infinity();
  } else {
    return min_candidates_.front().second;
  }
}

double MF::GetMax() const {
  if (max_candidates_.empty()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return max_candidates_.front().second;
  }
}

double MF::Update(const double measurement) {
  // assert 只能在release模式下使用
  // assert(initialized_ && "MeanFilter not initialized");
  // assert(values_.size() <= window_size_ && "Values size exceeds window size");
  // assert(min_candidates_.size() <= window_size_ && "Min candidates size exceeds window size");
  // assert(max_candidates_.size() <= window_size_ && "Max candidates size exceeds window size");
  if (!initialized_) {
    throw std::runtime_error("[MF]:MeanFilter not initialized");
  }
  if (values_.size() > window_size_) {
    throw std::runtime_error("[MF]:Values size exceeds window size");
  }
  if (min_candidates_.size() > window_size_) {
    throw std::runtime_error("[MF]:Min candidates size exceeds window size");
  }
  if (max_candidates_.size() > window_size_) {
    throw std::runtime_error("[MF]:Max candidates size exceeds window size");
  }

  ++time_;
  time_ %= static_cast<std::uint_fast8_t>(2 * window_size_);
  if (values_.size() == window_size_) {
    RemoveEarliest();
  }
  Insert(measurement);
  if (values_.size() > 2) {
    return (sum_ - GetMin() - GetMax()) /
           static_cast<double>(values_.size() - 2);
  } else {
    return sum_ / static_cast<double>(values_.size());
  }
}

bool MF::ShouldPopOldestCandidate(const uint8 old_time) const {
    if (old_time < window_size_) {
      if (time_ <= old_time + window_size_) {
        throw std::runtime_error("[MF]:Time constraint violation");
      }
    // assert(time_ <= old_time + window_size_ && "Time constraint violation");
    return old_time + window_size_ == time_;
  } else if (time_ < window_size_) {
    if (old_time >= time_ + window_size_) {
      throw std::runtime_error("[MF]:Time constraint violation");
    }
    // assert(old_time >= time_ + window_size_ && "Time constraint violation");
    return old_time == time_ + window_size_;
  } else {
    return false;
  }
}

void MF::RemoveEarliest() {
  if (!(values_.size() == window_size_)) {
    throw std::runtime_error("[MF]:Values size does not match window size");
  }
  // assert(values_.size() == window_size_ && "Values size does not match window size");
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (ShouldPopOldestCandidate(min_candidates_.front().first)) {
    min_candidates_.pop_front();
  }
  if (ShouldPopOldestCandidate(max_candidates_.front().first)) {
    max_candidates_.pop_front();
  }
}

void MF::Insert(const double value) {
  values_.push_back(value);
  sum_ += value;
  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));
  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}
}  // namespace common
}  // namespace control
