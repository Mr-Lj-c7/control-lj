#include "pid_controller.h"

namespace control {
namespace common {

// 初始化控制器
void PIDController::Init(const common_msg::PidConf &pid_conf){
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_enabled_ = pid_conf.integrator_enable;
    integrator_saturation_high_ = 
        std::fabs(pid_conf.integrator_saturation_level);
    integrator_saturation_low_ = 
        -std::fabs(pid_conf.integrator_saturation_level);
    integrator_saturation_status_ = 0;
    integrator_hold_ = false;
    ouput_saturation_high_ = std::fabs(pid_conf.output_saturation_level);
    ouput_saturation_low_ = -std::fabs(pid_conf.output_saturation_level);
    ouput_saturation_status_ = 0;
    SetPID(pid_conf);   
}

// 重置控制器参数
void PIDController::Reset(){
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_saturation_status_ = 0;
    ouput_saturation_status_ = 0;
}

// 重置积分项
void PIDController::Reset_intergal(){
    integral_ = 0.0;
    integrator_saturation_status_ = 0;
}

void PIDController::SetPID(const common_msg::PidConf &pid_conf){
    kp_ = pid_conf.kp;
    ki_ = pid_conf.ki;
    kd_ = pid_conf.kd;
    kaw_ = pid_conf.kaw;  // 已弃用
}

/**
 * @brief 计算控制量
 * @param[in] error 误差
 * @param[in] dt 控制周期
 * @return 控制量u 
 */
double PIDController::Control(const double error, const double dt){
    if (dt <=0.0) {
        std::cerr << 
          "[PIDController]:dt <= 0.0, use the last output, dt: " << dt << std::endl;
    }
    double diff = 0.0;
    double output = 0.0;
    if (first_hit_) { 
        first_hit_ = false;
    } else { 
        diff = (error - previous_error_) / dt;
    }
    // 积分器使用
    if (!integrator_enabled_) {
        integral_ = 0.0;
    } else if (!integrator_hold_){
        integral_ += error * dt * ki_;  // 积分器
        // 积分器监控
        if (integral_ > integrator_saturation_high_) {
            integral_ = integrator_saturation_high_;
            integrator_saturation_status_ = 1;  // 积分器过饱和
        } else if (integral_ < integrator_saturation_low_) {
            integral_ = integrator_saturation_low_;
            integrator_saturation_status_ = -1;  // 积分器不足
        } else {
            integrator_saturation_status_ = 0;   // 积分器正常
        }
    }
    previous_error_ = error;
    output = kp_ * error + integral_ + kd_ * diff;
    previous_output_ = output;
    return output;
}

// API 接口
/**
 * @brief 积分器饱和状态
 * @return 0 - 正常,1 - 过饱和,-1 - 不足积分 
 * */ 
int PIDController::IntegratorSaturationStatus() const{
    return integrator_saturation_status_;
}

/**
 * @brief 积分器是否保持
 * @return true - 积分保持,false - 积分不保持
 */
bool PIDController::IntegratorHold() const{
    return integrator_hold_;
}

/**
 * @brief 设置积分器保持
 * @param[in] hole 积分保持
 */
void PIDController::SetIntegratorHold(bool hole){
    integrator_hold_ = hole;
}



} // control
} // common
