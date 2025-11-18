#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H 

#include "control_component/config/pid_conf.h"
#include <string>
#include <iostream>

namespace control {
namespace common {

class PIDController {
public:
    PIDController() = default;
    virtual ~PIDController() = default;

    // 初始化控制器
    void Init(const common_msg::PidConf &pid_conf);

    // 重置控制器参数
    void Reset();

    // 重置积分项
    void Reset_intergal();

    void SetPID(const common_msg::PidConf &pid_conf);

    /**
     * @brief 计算控制量
     * @param[in] error 误差
     * @param[in] dt 控制周期
     * @return 控制量u 
     */
    virtual double Control(const double error, const double dt);

    /**
     * @brief 积分器饱和状态
     * @return 0 - 正常,1 - 过饱和,-1 - 不足积分 
     * */ 
    int IntegratorSaturationStatus() const;

    /**
     * @brief 积分器是否保持
     * @return true - 积分保持,false - 积分不保持
     */
    bool IntegratorHold() const;

    /**
     * @brief 设置积分器保持
     * @param[in] hole 积分保持
     */
    void SetIntegratorHold(bool hole);

protected:
    bool first_hit_ = false;  // PID控制器启用标记

    // pid参数
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double kaw_ = 0.0;

    double previous_error_ = 0.0;   // 上一次误差值
    double previous_output_ = 0.0;  // 上一次输出
    
    double integral_ = 0.0;         // 积分量
    double integrator_saturation_high_ = 0.0;  // 积分上限
    double integrator_saturation_low_ = 0.0;   // 积分下限
    bool integrator_hold_ = false;             // 积分器保持
    bool integrator_enabled_ = false;          // 积分计算
    int integrator_saturation_status_ = 0;     // 积分器状态 0, 1, -1

    // 积分量限幅
    double ouput_saturation_high_ = 0.0;  // 输出积分上限
    double ouput_saturation_low_ = 0.0;   // 输出积分下限
    int ouput_saturation_status_ = 0;     // 输出积分状态

private:

};

} // control
} // common

#endif