#ifndef CHASSIS_MSG_H
#define CHASSIS_MSG_H 

#include <vector>
#include <string>
#include <cmath>
#include "common_msg/basic_msgs/header_msg.h"
#include "common_msg/basic_msgs/vehicle_id_msg.h"
#include "common_msg/basic_msgs/vehicle_signal_msg.h"
#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/basic_msgs/drive_state_msg.h"

namespace control {
namespace common_msg { 

    typedef struct LICENSE {
        std::string vin;        // 车辆识别码
    } License;

    // GPS状态
    enum GpsQuality {
        FIX_NO = 0,
        FIX_2D = 1,
        FIX_3D = 2,
        FIX_INVALID = 3
    };

    // GPS信息
    typedef struct CHASSIS_GPS {
       double latitude;
       double longitude;
       bool gps_valid;

       int32_t year;
       int32_t month;
       int32_t day;
       int32_t hours;
       int32_t minutes;
       int32_t seconds;
       double compass_direction;
       double pdop;
       bool is_gps_fault;
       bool is_inferred;

       double altitude;
       double heading;
       double hdop;
       double vdop;
       GpsQuality quality;
       int32_t num_satellites;
       double gps_speed;
    } ChassisGPS;

    // 车轮速度
    typedef struct WHEEL_SPEED {
        enum WheelSpeedType {
            FORWARD = 0,
            BACKWARD = 1,
            STANDSTILL = 2,
            INVALID = 3
        };
        bool is_wheel_spd_rr_valid = false;
        WheelSpeedType wheel_direction_rr = INVALID;
        double wheel_spd_rr = 0.0;
        bool is_wheel_spd_rl_valid = false;
        WheelSpeedType wheel_direction_rl = INVALID;
        double wheel_spd_rl = 0.0;
        bool is_wheel_spd_fr_valid = false;
        WheelSpeedType wheel_direction_fr = INVALID;
        double wheel_spd_fr = 0.0;
        bool is_wheel_spd_fl_valid = false;
        WheelSpeedType wheel_direction_fl = INVALID;
        double wheel_spd_fl = 0.0;
    } WheelSpeed;

    // 超声波信息
    typedef struct SONAR {
     double range;         // Meter
     Point3D translation;  // Meter
     Quaternion rotation;  // Quaternion
    } Sonar;

    // 车辆周围环境信息
    typedef struct SURROUND {
     bool cross_traffic_alert_left;
     bool cross_traffic_alert_left_enabled;
     bool blind_spot_left_alert;
     bool blind_spot_left_alert_enabled;
     bool cross_traffic_alert_right;
     bool cross_traffic_alert_right_enabled;
     bool blind_spot_right_alert;
     bool blind_spot_right_alert_enabled;
     double sonar00;
     double sonar01;
     double sonar02;
     double sonar03;
     double sonar04;
     double sonar05;
     double sonar06;
     double sonar07;
     double sonar08;
     double sonar09;
     double sonar10;
     double sonar11;
     bool sonar_enabled;
     bool sonar_fault;
     double sonar_range;
     Sonar sonar;
    } Surround;

    // 底盘自检响应
    typedef struct CHECK_RESPONSE {
        bool is_eps_online = false;
        bool is_epb_online = false;
        bool is_esp_online = false;
        bool is_vtog_online = false;
        bool is_scu_online = false;
        bool is_switch_online = false;
        bool is_vcu_online = false;
    } CheckResponse;

    typedef struct CHASSIS {
        // 驾驶模式
        enum DrivingMode {
            COMPLETE_MANUAL = 0,      // 手动驾驶
            COMPLETE_AUTO_DRIVE = 1,  // 自动驾驶
            AUTO_STEER_ONLY = 2,      // 横向辅助
            AUTO_SPEED_ONLY = 3,      // 纵向辅助
            EMERGENCY_MODE = 4        // 紧急模式（手动干预响应）
        };
        // 故障码
        enum ErrorCode {
            NO_ERROR = 0,             
            
            CMD_NOT_IN_PERIOD = 1,         // 指令未响应

            CHASSIS_ERROR = 2,             // 底盘故障信号
            CHASSIS_ERROR_ON_STEER = 3,    // 方向盘故障
            CHASSIS_ERROR_ON_BRAKE = 4,    // 刹车故障
            CHASSIS_ERROR_ON_THROTTLE = 5, // 油门故障
            CHASSIS_ERROR_ON_GEAR = 6,     // 变速箱故障
            CHASSIS_CAN_LOST = 7,          // CAN总线故障

            MANUAL_INTERVENTION = 8,       // 人工干预

            CHASSIS_CAN_NOT_IN_PERIOD = 9, // 底盘CAN通信延迟

            UNKNOWN_ERROR = 10             // 未知故障
        };
        // 档位状态
        enum GearPosition {
            GEAR_NEUTRAL = 0,       // 空档
            GEAR_DRIVE = 1,         // 前进档
            GEAR_REVERSE = 2,       // 倒车档
            GEAR_PARKING = 3,       // 驻车档
            GEAR_LOW = 4,           // 低速档
            GEAR_INVALID = 5,       // 无效档位
            GEAR_NONE = 6           // 无档位状态
        };
        // 碰撞事件检测（保险杠）
        enum BumperEvent {
            BUMPER_INVALID = 0,     // 无效
            BUMPER_NORMAL = 1,      // 保险杠正常
            BUMPER_PRESSED = 2      // 保险杠碰撞
        };

        // 车辆状态
        bool engine_started;  // 发动机状态
        float engine_rpm;     // 转速
        float speed_mps;      // 车速
        float odometer_m;     // 车辆里程表数
        float fule_range_m;   // 续航里程

        // 控制器状态
        float throttle_percentage;   // 油门踏板开度（0-100%）
        float brake_percentage;      // 刹车踏板开度（0-100%）
        float steering_percentage;   // 方向盘位置（-100-100%），左正右负
        float steering_torque_nm;    // 方向盘扭矩
        bool parking_brake;          // 驻车制动状态

        // 灯光信号
        bool high_beam_signal;  // 远光灯
        bool low_beam_signal;   // 近光灯
        bool left_turn_signal;  // 左转灯
        bool right_turn_signal; // 右转灯
        bool horn;              // 喇叭
        bool wiper;             // 雨刷
        bool disengage_status;  // 脱离状态
        
        DrivingMode driving_mode;
        ErrorCode error_code;
        GearPosition gear_location;

        double steering_timestamp;
        Header header;
        
        int32_t chassis_error_mask;

        VehicleSignal signal;

        ChassisGPS chassis_gps;

        EngageAdvice engage_advice;

        WheelSpeed wheel_speed;

        Surround surround;

        VehicleID vehicle_id;

        int32_t battery_soc_percentage = -1;  // 电池剩余电量百分比
        float throttle_percentage_cmd;  // 油门踏板开度命令（0-100%）
        float brake_percentage_cmd;     // 刹车踏板开度命令（0-100%）
        float steering_percentage_cmd;  // 方向盘位置命令（-100-100%），左正右负

        BumperEvent front_bumper_event;
        BumperEvent back_bumper_event;

        CheckResponse check_response;

    } Chassis;

}  // namespace common_msg
}  // namespace control
#endif