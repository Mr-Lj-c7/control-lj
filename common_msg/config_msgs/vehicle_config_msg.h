#ifndef VEHICLE_CONFIG_MSG_H
#define VEHICLE_CONFIG_MSG_H 

#include "common_msg/basic_msgs/header_msg.h"
#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/basic_msgs/vehicle_id_msg.h"

namespace control {
namespace common_msg {

  typedef struct TRANSFORM {
    int32_t source_frame;   // Also known as "frame_id."
    int32_t target_frame;   // Also known as "child_frame_id."
    Point3D position;
    Quaternion rotation;
} Transform;

typedef struct EXTRINSICS {
  Transform transforms;
} Extrinsics;


// 车辆品牌
enum VehicleBrand {
  LINCOLN_MKZ = 0,
  GEM = 1,
  LEXUS = 2,
  TRANSIT = 3,
  GE3 = 4,
  WEY = 5,
  ZHONGYUN = 6,
  CH = 7,
  DKIT = 8,
  NEOLIX = 9,
  OTHER = 10
};

typedef struct LATENCY_PARAM {
    double dead_time;    // 延时
    double rise_time;    // 延时上升时间
    double peak_time;    // 延时峰值
    double setting_time; // 稳定时间
} LatencyParam;

typedef struct VEHICLE_PARAM {
    VehicleBrand brand;  // 车辆品牌
    VehicleID vehicle_id; // 车辆ID信息
    // 车辆几何边界
    double front_edge_to_center;
    double back_edge_to_center;
    double left_edge_to_center;
    double right_edge_to_center;
    // 车辆尺寸
    double length;   // 长度
    double width;    // 宽度
    double height;   // 高度
    // 车辆运动性能参数
    double min_turn_radius;    // 最小转弯半径
    double max_acceleration;   // 最大加速度
    double max_deceleration;   // 最大减速度
    // 转向系统参数
    double max_steer_angle;        // 最大转向角
    double max_steer_angle_rate;   // 最大转向角速度
    double min_steer_angle_rate;   // 最小转向角速度
    double steer_ratio;            // 转向传动比
    double wheel_base;             // 轴距
    double wheel_rolling_radius;   // 车轮滚动半径
    //
    float max_abs_speed_when_stopped;    // 停车时的最小速度
    double brake_deadzone;               // 最小有效制动百分数  
    double throttle_deadzone;            // 最小有效油门百分数
    //   
    LatencyParam steering_latency_param;  // 转向系统延时参数
    LatencyParam throttle_latency_param;  // 油门系统延时参数
    LatencyParam brake_latency_param;     // 制动系统延时参数
} VehicleParam;

typedef struct VEHICLE_CONFIG {
  Header header;  
  VehicleParam vehicle_param;  
  Extrinsics extrinsics;
} VehicleConfig;

}  // common_msg
}  // control

#endif