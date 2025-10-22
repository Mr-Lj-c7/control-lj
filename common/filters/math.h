#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <iostream>
#include <functional>
const double kMathEpsilon = 1e-10;;

namespace control {
namespace common {

// // 注意：在头文件中直接定义函数时，使用 inline 关键字，以避免多重定义错误

// // 将角度标准化到 [-π, π] 范围内)
// inline double NormalizeAngle(const double angle) {
//     double normalized_angle = std::fmod(angle + M_PI, 2.0 * M_PI);
//     if (normalized_angle < 0.0) {
//         normalized_angle += 2.0 * M_PI;
//     }
//     return normalized_angle - M_PI;
// }

// // 角度按时间比例插值
// inline double slerp(const double a0, const double t0, const double a1, const double t1,
//              const double t) {
//   if (std::abs(t1 - t0) <= kMathEpsilon) {
//     std::cerr << "input time difference is too small" << std::endl;
//     return NormalizeAngle(a0);
//   }
//   const double a0_n = NormalizeAngle(a0);
//   const double a1_n = NormalizeAngle(a1);
//   double d = a1_n - a0_n;
//   if (d > M_PI) {
//     d = d - 2 * M_PI;
//   } else if (d < -M_PI) {
//     d = d + 2 * M_PI;
//   }

//   const double r = (t - t0) / (t1 - t0);
//   const double a = a0_n + d * r;
//   return NormalizeAngle(a);
// }

// // 黄金分割搜索算法
// inline double GoldenSectionSearch(const std::function<double(double)> &func,
//                            const double lower_bound, const double upper_bound,
//                            const double tol = 1e-6) {
//   static constexpr double gr = 1.618033989;  // (sqrt(5) + 1) / 2

//   double a = lower_bound;
//   double b = upper_bound;

//   double t = (b - a) / gr;
//   double c = b - t;
//   double d = a + t;

//   while (std::abs(c - d) > tol) {
//     if (func(c) < func(d)) {
//       b = d;
//     } else {
//       a = c;
//     }
//     t = (b - a) / gr;
//     c = b - t;
//     d = a + t;
//   }
//   return (a + b) * 0.5;
// }

double NormalizeAngle(const double angle);

double slerp(const double a0, const double t0, 
            const double a1, const double t1,
            const double t);

double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol = 1e-6);

// 一维线性插值，用于数据平滑处理
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    std::cerr << "input time difference is too small" << std::endl;
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

}  // namespace control
}  // namespace common



#endif