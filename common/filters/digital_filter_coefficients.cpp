#include "digital_filter_coefficients.h"

namespace control {
namespace common {

/**
 * @brief 巴特沃斯二阶低通滤波器系数,使用差分方程实现
 * @param[in] ts 采样间隔
 * @param[in] cutoff_freq 截止频率
 * @param[out] denominators 与y值相关系数
 * @param[out] numerators   与x值相关系数
 */
void LpCoefficients(const double ts, 
    const double cutoff_freq, 
    std::vector<double> *denominators, 
    std::vector<double> *numerators) {
    denominators->clear();
    numerators->clear(); 
    denominators->reserve(3); // 预分配内存
    numerators->reserve(3);
    double wa = 2.0 * M_PI * cutoff_freq;  // 模拟信号角频率，频率->角频率：ω = 2πf
    double alpha = wa * ts / 2.0;          // 双线性变换：预畸变因子
    double alpha_sqr = alpha * alpha;
    double tmp_term = std::sqrt(2.0) * alpha + alpha_sqr;
    double gain = alpha_sqr / (1.0 + tmp_term);  // 滤波器增益系数

    // 系数输出b[n], a[n]
    denominators->push_back(1.0);
    denominators->push_back(2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term));
    denominators->push_back((1.0 - std::sqrt(2.0) * alpha + alpha_sqr) / 
                            (1.0 + tmp_term));
    
    numerators->push_back(gain);
    numerators->push_back(2.0 * gain);
    numerators->push_back(gain);
}


/**
 * @brief  一阶低通滤波器的系数,G(s) = K * e^(-dead_time*s) / (tau*s + 1)
 * @param[in] ts 采样间隔
 * @param[in] settling_time 调节时间常数，表示系统响应的快慢
 * @param[in] dead_time 系统响应时延
 * @param[out] denominators 分母系数
 * @param[out] numerators 分子系数
 */
void LpFirstOrderCoefficients(const double ts, 
    const double settling_time,
    const double dead_time,
    std::vector<double> *denominators,
    std::vector<double> *numerators) {
    // 参数检查
    if (ts <=0.0 || settling_time < 0.0 || dead_time < 0.0) {
        std::cerr << "[LpFirstOrderCoefficients]:time cannot be negative" 
          << std::endl;
        return;
    }
    // 延迟采样点数
    const size_t k_d = static_cast<size_t>(dead_time / ts);
    double a_term;

    denominators->clear();
    numerators->clear();
    denominators->reserve(2);
    numerators->reserve(k_d + 1);

    // 计算滤波系数
    if (settling_time == 0.0) {
        a_term = 0.0;
    } else {
        a_term = exp(-1 * ts / settling_time);  // 指数衰减
    }

    // 系数输出b[n], a[n]
    denominators->push_back(1.0);
    denominators->push_back(-a_term);
    numerators->insert(numerators->end(), k_d, 0.0);
    numerators->push_back(1 - a_term);
}

} // common
} // control
