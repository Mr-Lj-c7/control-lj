#ifndef MRAC_CONF_H
#define MRAC_CONF_H 

#include <string>
#include <cmath>
#include <vector>

namespace control {
namespace common_msg {

typedef struct MRAC_CONF {
    // mrac基础配置
    int32_t mrac_model_order = 1;        // 模型阶数；
    double reference_time_constant;      // 参考模型时域常数
    double reference_natural_frequency;  // 参考模型自然频率
    double reference_damping_ratio;      // 参考模型阻尼比
    // 自适应增益配置
    double adaption_state_gain;
    double adaption_desired_gain;
    double adaption_nonlinear_gain;
    // 自适应矩阵配置
    double adaption_matrix_p;
    double mrac_saturation_level = 1.0;
    // 饱和和防积分饱和配置
    double anti_windup_compensation_gain;
    double clamping_time_constant;
} MracConf;

}  // namespace common_msg
}  // namespace control

#endif