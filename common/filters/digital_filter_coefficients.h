#ifndef DIGITAL_FILTER_COEFFICIENTS_H
#define DIGITAL_FILTER_COEFFICIENTS_H

#include <string>
#include <cmath>
#include <vector>
#include <iostream>

namespace control {
namespace common {

/**
 * @brief 计算低通系数,使用差分方程实现滤波IIR滤波器
 * @param[in] ts 采样间隔
 * @param[in] cutoff_freq 截止频率
 * @param[out] denominators 与y值相关系数
 * @param[out] numerators   与x值相关系数
 */
void LpCoefficients(const double ts, 
    const double cutoff_freq, 
    std::vector<double> *denominators, 
    std::vector<double> *numerators);


/**
 * @brief  计算低通滤波器的系数,ZOH滤波器
 * @param[in] ts 采样间隔
 * @param[in] settling_time 调节时间常数，表示系统响应的快慢
 * @param[in] dead_time 时延
 * @param[out] denominators 分母系数
 * @param[out] numerators 分子系数
 */
void LpFirstOrderCoefficients(const double ts, 
    const double settling_time,
    const double dead_time,
    std::vector<double> *denominators,
    std::vector<double> *numerators
);

} // common
} // control

#endif
