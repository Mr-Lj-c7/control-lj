
#include "leadlag_controller.h"

namespace control {
namespace common {

/**
 * 超前/滞后-lead/lag控制器，减小控制系统的响应延迟，
   提高控制系统的响应速度与稳定性能
   H(s) = beta*((tau*s+1)/(tau*alpha*s+1))
   双线性变换：H(z) = (k_n0+k_n1*z)/(k_d0+k_d1*z)
   a1 = alpha*tau, a0 = 1.0, b1 = bata*tau, b0 = beta
   k_n1 = 2*beta*tau+T*beta, k_n0 = T*beta-2*beta*tau
   k_d1 = 2*alpha*tau+T, k_d0 = T-2*alpha*tau
 */ 

    /**
     * @brief 加载leadlag控制器参数并初始化
     * @param[in] leadlag_conf leadlag控制器参数
     * @param[in] dt 车辆控制系统控制周期
     */
    void LeadLagController::Init(const common_msg::LeadlagConf &leadlag_conf, 
      const double dt) {
        previous_output_ = 0.0;
        previous_innerstate_ = 0.0;
        innerstate_ = 0.0;
        innerstate_saturation_high_ = 
            std::fabs(leadlag_conf.innerstate_saturation_level);
        innerstate_saturation_low_ = 
            -std::fabs(leadlag_conf.innerstate_saturation_level);
        innerstate_saturation_status_ = 0;
        SetLeadlag(leadlag_conf);
        TransformC2d(dt);
        return;
      }

    /**
     * @brief 设置leadlag控制器参数
     * @param[in] leadlag_conf leadlag控制器参数
     */
    void LeadLagController::SetLeadlag(const common_msg::LeadlagConf 
      &leadlag_conf) {
        alpha_ = leadlag_conf.alpha;
        beta_ = leadlag_conf.beta;
        tau_ = leadlag_conf.tau;
        return;
      }

    /**
     * @brief 双线性变换离散化系统
     * @param[in] dt 车辆控制系统控制周期
     */
    void LeadLagController::TransformC2d(const double dt) {
      if (dt <= 0.0 ) {
        std::cerr << "dt <= 0, continuous-discrete transformation failed, dt: "
                  << dt << std::endl;
        transformc2d_enable_ = false;
      } else {
        double a1 = alpha_ * tau_;
        double a0 = 1.0;
        double b1 = beta_ * tau_;
        double b0 = beta_;  
        Ts_ = dt;
        kn0_ = 2 * b1 - Ts_ * b0;
        kn1_ = 2 * b1 + b0 * Ts_;
        kd0_ = Ts_ * a0 - 2 * a1;
        kd1_ = 2 * a1 + Ts_ * a0;
        // 系统稳定性判定，极点|kd0/kd1| < 1
        if (kd1_ <= 0.0) {
          std::cerr << "kd1 <= 0, continuous-discrete transformation failed, "
                    << "kd1: " << kd1_ << std::endl;
          transformc2d_enable_ = false;
          return;
        } else {
          transformc2d_enable_ = true;
          return;
        }
      }
      
    }

    /**
     * @brief 重置leadlag控制器
     */
    void LeadLagController::Reset() {
      previous_innerstate_ = 0.0;
      previous_output_ = 0.0;
      innerstate_ = 0.0;
      innerstate_saturation_status_ = 0;
      return;
    }

    /**
     * @brief 补偿控制计算,声明为虚函数便于后续重写控制逻辑
     * @param[in] error  控制误差（期望值与实际测量值差值）
     * @param[in] dt  车辆控制系统控制周期
     * @param[out] control_value  控制补偿值
     * y(k) = k_n0*x(k) + k_n1*x(k+1)
     */
    double LeadLagController::Control(const double error, 
      const double dt) {
        if (!transformc2d_enable_) { // 视为统一比例调节器
          TransformC2d(dt); 
          std::cerr << "C2d transform failed; will work as a unity compensator, dt:" 
                    << dt <<std::endl;
          return error;
        }
        if (dt <= 0.0) {
          std::cerr << "dt <= 0, will use the last uotput, dt: "
                  << dt << std::endl;
          return previous_output_;
        }
        double output = 0.0;
        // (幅值)饱和计算，确保控制器内部数值计算稳定性
        innerstate_ = (error - previous_innerstate_ * kd0_) / kd1_;
        // 副值限制
        if (innerstate_ > innerstate_saturation_high_) {
          innerstate_ = innerstate_saturation_high_;
          innerstate_saturation_status_ = 1;
        } else if (innerstate_ < innerstate_saturation_low_) {
          innerstate_ = innerstate_saturation_low_;
          innerstate_saturation_status_ = -1;
        } else {
          innerstate_saturation_status_ = 0;
        }
        // 输出
        output = kn0_ * previous_innerstate_ + kn1_ * innerstate_;
        previous_innerstate_ = innerstate_;
        previous_output_ = output;
        return output;  
      }

    /**
     * @brief 幅值判断：获取leadlag控制器内部状态饱和状态
     * x(k+1) = (u(k)-k_d0*x(k)) / k_d1
     * @param[out] innerstate 内部状态饱和状态
     * innerstate取值范围：0
     * > saturate_up_limit == 1 
     * < saturate_down_limit == -1
     */
    int LeadLagController::InnerstateSaturationStatus() const {
      return innerstate_saturation_status_;
    }

} // control
} // common
