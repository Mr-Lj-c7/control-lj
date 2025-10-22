#ifndef LEADLAG_CONTROLLER_H
#define LEADLAG_CONTROLLER_H 

#include "control_component/config/leadlag_conf.h"
#include <iostream>

namespace control {
namespace common {

/**
 * 超前/滞后-lead/lag控制器，减小控制系统的响应延迟，
   提高控制系统的响应速度与稳定性能，
   这里的LeadlagController是独立的控制器，输入error,T,输出控制量
   H(s) = beta*((tau*s+1)/(tau*alpha*s+1))
   双线性变换：H(z) = (k_n0+k_n1*z)/(k_d0+k_d1*z)
   a1 = alpha*tau, a0 = 1.0, b1 = bata*tau, b0 = beta
   k_n1 = 2*beta*tau+T*beta, k_n0 = T*beta-2*beta*tau
   k_d1 = 2*alpha*tau+T, k_d0 = T-2*alpha*tau
 */ 
class LeadLagController { 
public:
    LeadLagController() = default;
    ~LeadLagController() = default;
    /**
     * @brief 加载leadlag控制器参数并初始化
     * @param[in] leadlag_conf leadlag控制器参数
     * @param[in] dt 车辆控制系统控制周期
     */
    void Init(const common_msg::LeadlagConf &leadlag_conf, const double dt);

    /**
     * @brief 设置leadlag控制器参数
     * @param[in] leadlag_conf leadlag控制器参数
     */
    void SetLeadlag(const common_msg::LeadlagConf &leadlag_conf);

    /**
     * @brief 双线性变换离散化系统
     * @param[in] dt 车辆控制系统控制周期
     */
    void TransformC2d(const double dt);

    /**
     * @brief 重置leadlag控制器
     */
    void Reset();

    /**
     * @brief 补偿控制计算,声明为虚函数便于后续重写控制逻辑
     * @param[in] error  控制误差（期望值与实际测量值差值）
     * @param[in] dt  车辆控制系统控制周期
     * @param[out] control_value  控制补偿值
     * y(k) = k_n0*x(k) + k_n1*x(k+1)
     */
    virtual double Control(const double error, const double dt);

    /**
     * @brief 幅值判断：获取内部状态饱和状态
     * x(k+1) = (u(k)-k_d0*x(k)) / k_d1
     * @param[out] innerstate 内部状态饱和状态
     * innerstate取值范围：
     * > saturate_up_limit == 1 
     * < saturate_down_limit == -1
     */
    int InnerstateSaturationStatus() const;

protected:
    // 连续时域中的控制系数
    double alpha_ = 0.0;
    double beta_ = 0.0;
    double tau_ = 0.0;
    double Ts_ = 0.01;  // 默认控制周期为10ms
    // 离散时域中的控制系数
    double kn0_ = 0.0;
    double kn1_ = 0.0; 
    double kd0_ = 0.0; 
    double kd1_ = 0.0;
    // 离散时间域中的内部（中间）状态
    double previous_output_ = 0.0;
    double previous_innerstate_ = 0.0;
    double innerstate_ = 0.0;
    double innerstate_saturation_high_ = 0.0;
    double innerstate_saturation_low_ = 0.0;
    int innerstate_saturation_status_ = 0;  // 1, -1
    bool transformc2d_enable_ = false;
private:
};

} // control
} // common
#endif