#ifndef LAT_BASED_LQR_CONTROLLER_CONF_H
#define LAT_BASED_LQR_CONTROLLER_CONF_H 

#include <vector>
#include <string>
#include "control_component/config/gain_scheduler_conf.h"
#include "control_component/config/leadlag_conf.h"
#include "control_component/config/mrac_conf.h"

namespace control{
typedef struct LAT_BASE_LQR_CONTROLLER_CONF {
    double ts = 0.01;               // 采样周期，0.01s
    int32_t preview_window = 0;  // 预览窗口，预览时间 = 预览窗口*采样时间
    // 前(2)、后(2)轮侧偏刚度
    double cf = 155494.663;               // N/rad
    double cr = 155494.663;               // N/rad
    // 四轮车体分布质量
    int32_t mass_fl = 520;
    int32_t mass_fr = 520;
    int32_t mass_rl = 520;
    int32_t mass_rr = 520;
    // LQR求解器参数
    double eps = 0.01;               // LQR求解器收敛阈值
    // LQR控制器Q矩阵对角线元素
    std::vector<double> matrix_q{};  // 1*4       
    double matrix_q_0 = 0.05;          
    double matrix_q_1 = 0.0;          
    double matrix_q_2 = 1.0;          
    double matrix_q_3 = 0.0;       
    // 倒车LQR控制器Q矩阵对角线元素  
    std::vector<double> reverse_matrix_q{};  
    double reverse_matrix_q_0 = 0.05;  
    double reverse_matrix_q_1 = 0.0;  
    double reverse_matrix_q_2 = 1.0;  
    double reverse_matrix_q_3 = 0.0;  
    int32_t max_iteration = 150;    // LQR求解最大迭代次数
    // 均值滤波器参数
    int32_t cutoff_freq = 10;              // 均值滤波器截止频率
    int32_t mean_filter_window_size = 10;  // 均值滤波窗口大小
    double max_lateral_acceleration = 5.0;  // 横向加速度最大值限制
    
    bool enable_reverse_leadlag_compensation = false;

    // 预瞄控制配置
    bool enable_look_ahead_back_control = false;
    double lookahead_station = 1.4224;
    double lookback_station = 2.8448;  
    double lookahead_station_high_speed = 1.4224;  
    double lookback_station_high_speed = 2.8448;  
    bool enable_steer_mrac_control = false;
     
    // 导航模式配置
    double lock_steer_speed = -0.081;
    bool enable_navigation_mode_error_filter = false;
    bool enable_navigation_mode_position_update = true;
    double query_relative_time = 0.8;

    // 轨迹变换配置
    bool trajectory_transform_to_com_reverse = false;  // 高速启用反馈增强
    bool trajectory_transform_to_com_drive = false;  // 高速启用反馈增强
    bool enable_maximum_steer_rate_limit = false;      // 最大转向速率限制
    bool query_time_nearest_point_only = false;        // 最近点查询策略

    bool enable_feedback_augment_on_high_speed = false;        
    bool enable_maximun_steer_rate_limit = false;   

    // 速度切换配置
    double switch_speed = 3.0;               // 低速与高速切换速度
    double switch_speed_window = 0.0;
    double reverse_feedforward_ratio = 1.0;  // 倒车前馈比例
    bool reverse_use_dynamic_model = false;  // 倒车动态模型
    double reverse_feedback_ratio = 1.4;

    double minimum_speed_protection;         // 最小速度保护

    // 增益调度器（用于高速时的Q参数调节）
    common_msg::GainScheduler lat_error_gain_scheduler;
    common_msg::GainScheduler heading_error_gain_scheduler;
    // 倒车控制补偿
    common_msg::LeadlagConf reverse_leadlag_conf;
    // mrac 主要用于适配不同车型的转向系统
    common_msg::MracConf steer_mrac_conf; 

} LatBaseLqrControllerConf;
}


#endif