#ifndef CONTROL_CMD_MSG_H
#define CONTROL_CMD_MSG_H 

#include "common_msg/basic_msgs/pnc_point_msg.h" 
#include "common_msg/basic_msgs/drive_state_msg.h" 
#include "common_msg/basic_msgs/header_msg.h" 
#include "common_msg/basic_msgs/vehicle_signal_msg.h" 
#include "common_msg/chassis_msgs/chassis_msg.h"
#include "pad_msg.h"
#include <vector> 
#include <string>
#include <cmath>

namespace control {
namespace common_msg {

typedef struct SIMPLE_LATERAL_DEBUG { 
    // 控制指令、状态量误差
    double lateral_error;
    double ref_heading;
    double heading;
    double heading_error;
    double heading_error_rate;
    double lateral_error_rate;
    double curvature;
    double steer_angle;
    double steer_angle_feedforward;
    double steer_angle_lateral_contribution;
    double steer_angle_lateral_rate_contribution;
    double steer_angle_heading_contribution;
    double steer_angle_heading_rate_contribution;
    double steer_angle_feedback;
    double steering_position;
    double ref_speed;
    double steer_angle_limited;
    // 横向加速度
    double lateral_acceleration;
    // 横向加速度变化率 Jerk
    double lateral_jerk;

    double ref_heading_rate;
    double heading_rate;

    // 参考点航向加速度，当前航向加速度，航向加速度误差
    double ref_heading_acceleration;
    double heading_acceleration;
    double heading_error_acceleration;

    // 参考点航向加速度变化率，当前航向加速度变化率，航行加速度变化率误差
    double ref_heading_jerk;
    double heading_jerk;
    double heading_error_jerk;
    
    // 横向误差，航行误差（考虑预瞄距离）
    double lateral_error_feedback;
    double heading_error_feedback;

    // 路径规划点
    TrajectoryPoint current_target_point;

    // 方向盘反馈增强
    double steer_angle_feedback_augment;

    // 侧向向心加速度
    double lateral_centripetal_acceleration;
} SimpleLateralDebug;

typedef struct SIMPLE_LONGITUDINAL_DEBUG {
   double station_reference ;
   double station_error ;
   double station_error_limited ;
   double preview_station_error ;
   double speed_reference ;
   double speed_error ;
   double speed_controller_input_limited ;
   double preview_speed_reference ;
   double preview_speed_error ;
   double preview_acceleration_reference ;
   double acceleration_cmd_closeloop ;
   double acceleration_cmd ;
   double acceleration_lookup ;
   double speed_lookup ;
   double calibration_value ;
   double throttle_cmd ;
   double brake_cmd ;
   bool is_full_stop ;
   double slope_offset_compensation ;
   double current_station ;
   double path_remain ;
   int32_t pid_saturation_status ;
   int32_t leadlag_saturation_status;
   double speed_offset ;
   double current_speed ;
   double acceleration_reference ;
   double current_acceleration ;
   double acceleration_error ;
   double jerk_reference ;
   double current_jerk;
   double jerk_error ;
   TrajectoryPoint current_matched_point ;
   TrajectoryPoint current_reference_point ;
   TrajectoryPoint preview_reference_point ;
   double acceleration_lookup_limit ;
   double vehicle_pitch ;
   bool is_epb_brake ;
   double current_steer_interval ;
   bool is_wait_steer ;
   bool is_stop_reason_by_destination ;
   bool is_stop_reason_by_prdestrian ;
   bool is_full_stop_soft ;
} SimpleLongitudinalDebug;

typedef struct DEBUG {
    SimpleLateralDebug simple_lat_debug;
    SimpleLongitudinalDebug simple_lon_debug;
} Debug;

typedef struct LATENCYSTATS {
   double total_time_ms;
   double controller_time_ms;
   bool total_time_exceeded;
} LatencyStats;

typedef struct CONTROL_COMMAND { 
    double throttle;  // percentage [0, 100]
    
    double brake;  // percentage [0, 100]

    // 目标非定向转向率，以每秒满刻度的百分比表示[0，100]
    double steering_rate;

    // 目标转向角度，百分比[-100, 100]
    double steering_target;

    // 泊车制动 epd
    bool parking_brake;

    // 目标速度
    double speed;

    // 目标加速度
    double acceleration;    

    // 重置模型
    bool reset_model;  // 弃用

    // 车辆启动
    bool engine_on_off;

    // 上一周期规划轨迹的完成百分比
    double trajectory_fraction;

    Chassis::DrivingMode driving_mode;
    Chassis::GearPosition gear_location;
    Debug debug;
    VehicleSignal vehicle_signal;
    LatencyStats latency_stats;
    PadMessage pad_message;
    EngageAdvice engage_advice;

    bool is_in_safe_mode = false;

} ControlCommand;

}  // common_msg
}  // control
#endif