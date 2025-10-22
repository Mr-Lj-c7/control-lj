#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

#include <deque>
#include <assert.h>
#include <vector>
#include <string>
#include <iostream>

namespace control {
namespace common {

class DigitalFilter
{
public:
    DigitalFilter(/* args */) = default;
    DigitalFilter(std::vector<double> &denomintors,
                std::vector<double> &numerators);
    ~DigitalFilter() = default;

    // IIR滤波器,低通二阶,return y[0]
    double Filter(const double x_insert);

    // 设置输出（分母）系数
    void SetDenominators(std::vector<double> &denominators);
    // 设置输入（分子）系数
    void SetNumerators(std::vector<double> &numerators);

    void SetCoefficients(std::vector<double> &denominators,
                        std::vector<double> &numerators);

    // 设置死区
    void SetDeadZone(const double dead_zone);

    // 重置滤波器
    void ResetValues();

    // API接口
    const std::vector<double> &GetDenomrators() const;
    const std::vector<double> &GetNumerators() const;
    double GetDeadZone() const;
    const std::deque<double> &GetInputs() const;
    const std::deque<double> &GetOutputs() const;

private:
    // 更新滤波后的输出值
    double UpdateLast(const double input);

    /**
     * 差分计算: y[n] = (num[0]*x[n] + num[1]*x[n-1] + num[2]*x[n-2] - 
                        den[1]*y[n-1] - den[2]*y[n-2]) / den[0]
    */
    double Compute(const std::deque<double> &values, 
        const std::vector<double> &cofficients, 
        const std::size_t coeff_start, 
        const std::size_t coeff_end);
    
    /**
     * 前端为最新值，后端为旧值，保持总量为系数数量,
       使用deque为块内存分配，适用于连续增减队列值场景
     */
    std::deque<double> x_values_;
    std::deque<double> y_values_;

    // 滤波系统输入与输出值系数
    std::vector<double> denominators_;
    std::vector<double> numerators_;

    double dead_zone_ = 0.0;  // 更新最后输出值的阈值

    double last_ = 0.0;       // 上一次输出值
};

}  // common
}  // control

#endif
