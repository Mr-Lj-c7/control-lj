#include "digital_filter.h"

namespace {
    const double KDoubleEpsilon = 1e-6;
}

namespace control {
namespace common {
DigitalFilter::DigitalFilter(std::vector<double> &denomintors,
    std::vector<double> &numerators) {
    SetCoefficients(denomintors, numerators);
};

// 设置输出（分母）系数
void DigitalFilter::SetDenominators(std::vector<double> &denominators) {
    denominators_ = denominators;
    return;
};

// 设置输入（分子）系数
void DigitalFilter::SetNumerators(std::vector<double> &numerators) {
    numerators_ = numerators;
    return;
};

void DigitalFilter::SetCoefficients(std::vector<double> &denominators,
    std::vector<double> &numerators) {
    SetDenominators(denominators);
    SetNumerators(numerators);
    return;
};

// 设置死区
void DigitalFilter::SetDeadZone(const double dead_zone) {
    dead_zone_ = std::abs(dead_zone);
    std::cerr << "[dead_zone_]: " << dead_zone_ << std::endl;
    return;
};

// 重置滤波器
void DigitalFilter::ResetValues() {
    std::fill(x_values_.begin(), x_values_.end(), 0.0);
    std::fill(y_values_.begin(), y_values_.end(), 0.0);
    return;
};

// API接口
const std::vector<double> &DigitalFilter::GetDenomrators() 
    const {
        return denominators_;
    };

const std::vector<double> &DigitalFilter::GetNumerators() 
    const {
        return numerators_;
    };

double DigitalFilter::GetDeadZone() 
    const {
        return dead_zone_;
    };

const std::deque<double> &DigitalFilter::GetInputs() 
    const {
        return x_values_;
    };

const std::deque<double> &DigitalFilter::GetOutputs() 
    const {
        return y_values_;
    };

// IIR滤波器,低通二阶,return y[0]
double DigitalFilter::Filter(const double x_insert) {
    // 检查系数
    if (denominators_.empty() || numerators_.empty()) {
        std::cerr << "denominators or numerators is empty" << std::endl;
        return 0.0;
    }
    // 更新x_values_
    x_values_.pop_back();
    x_values_.push_front(x_insert);
    const double xside = 
        Compute(x_values_, numerators_, 0, numerators_.size() - 1);
    // 更新y_values_
    y_values_.pop_back();
    const double yside = 
        Compute(y_values_, denominators_, 1, denominators_.size() - 1);
    double y_insert = 0.0;
    if (denominators_.front() > KDoubleEpsilon) {
        y_insert = (xside - yside) / denominators_.front();  // 差分方程
    }
    y_values_.push_front(y_insert);

    return UpdateLast(y_insert);
};


// 更新滤波后的输出值
double DigitalFilter::UpdateLast(const double input) {
    const double diff = std::abs(input - last_);
    if (diff < dead_zone_) {
        return last_;
    }
    last_ = input;
    return input;
};

/**
 * 差分计算: y[n] = (num[0]*x[n] + num[1]*x[n-1] + num[2]*x[n-2] - 
                    den[1]*y[n-1] - den[2]*y[n-2]) / den[0]
*/
double DigitalFilter::Compute(const std::deque<double> &values, 
    const std::vector<double> &cofficients, 
    const std::size_t coeff_start, 
    const std::size_t coeff_end) {
    // 检查系数合理性 
    assert((coeff_start <= coeff_end && coeff_end < cofficients.size()) 
        && "cofficients index out of range");
    assert((coeff_end - coeff_start + 1 == values.size()) 
        && "cofficients size != values size");
    double sum = 0.0;
    auto i = coeff_start;
    for (const auto &value : values) {
        sum += value * cofficients[i];
        ++i;
    }
    return sum;
};
    

}  // common
}  // control

