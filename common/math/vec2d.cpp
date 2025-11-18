#include "vec2d.h"

namespace control {
namespace common {
Vec2d Vec2d::CreateUnitVec2d(const double angle) {
  return Vec2d(std::cos(angle), std::sin(angle));
}

double Vec2d::Length() const { return std::hypot(x_, y_); }

double Vec2d::LengthSquare() const { return x_ * x_ + y_ * y_; }

double Vec2d::Angle() const { return std::atan2(y_, x_); }

void Vec2d::Normalize() {
  const double l = Length();
  if (l > kMathEpsilon) {
    x_ /= l;
    y_ /= l;
  }
}

double Vec2d::DistanceTo(const Vec2d &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec2d::DistanceSquareTo(const Vec2d &other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

double Vec2d::CrossProd(const Vec2d &other) const {
  return x_ * other.y() - y_ * other.x();
}

double Vec2d::InnerProd(const Vec2d &other) const {
  return x_ * other.x() + y_ * other.y();
}

Vec2d Vec2d::rotate(const double angle) const {
  return Vec2d(x_ * cos(angle) - y_ * sin(angle),
               x_ * sin(angle) + y_ * cos(angle));
}

void Vec2d::SelfRotate(const double angle) {
  double tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2d Vec2d::operator+(const Vec2d &other) const {
  return Vec2d(x_ + other.x(), y_ + other.y());
}

Vec2d Vec2d::operator-(const Vec2d &other) const {
  return Vec2d(x_ - other.x(), y_ - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const {
  return Vec2d(x_ * ratio, y_ * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const {
  // debug 模式
  assert(std::abs(ratio) > kMathEpsilon && "[vec2d]: error ratio is zero.");
//  // release模式
//   if (!(std::abs(ratio) > kMathEpsilon)) {
//     throw std::invalid_argument("[vec2d]: error ratio is zero.");
//   }
  return Vec2d(x_ / ratio, y_ / ratio);
}

Vec2d &Vec2d::operator+=(const Vec2d &other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2d &Vec2d::operator-=(const Vec2d &other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2d &Vec2d::operator*=(const double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2d &Vec2d::operator/=(const double ratio) {
  // debug 模式
  assert(std::abs(ratio) > kMathEpsilon && "[vec2d]: error ratio is zero.");
//  // release模式
//   if (!(std::abs(ratio) > kMathEpsilon)) {
//     throw std::invalid_argument("[vec2d]: error ratio is zero.");
//   }
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon);
}

Vec2d operator*(const double ratio, const Vec2d &vec) { return vec * ratio; }

std::string Vec2d::DebugString() const {
    std::stringstream ss;
    ss << "vec2d ( x = " <<  x_ << "  y = " << y_ << " )";
    const std::string str_vec2d = ss.str();
    return str_vec2d;
}

} // control
} // common