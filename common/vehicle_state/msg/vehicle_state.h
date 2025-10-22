#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H 

# include "common_msg/chassis_msgs/chassis_msg.h"
# include "common_msg/localization_msgs/pose_msg.h"

namespace control {
namespace common_msg {

typedef struct VEHICLE_STATE {
    double x;               // 车辆位置x，世界坐标系
    double y;               // 车辆位置y，世界坐标系
    double z;               // 车辆位置z，世界坐标系
    double timestamp;       // 时间戳
    double roll;            // 绕X轴翻滚角，车体坐标系
    double pitch;           // 绕Y轴俯仰角，车体坐标系
    double yaw;             // 绕Z轴偏航角，车体坐标系
    double heading;         // 车辆航向角，世界坐标系
    double kappa;           // 车辆当前路径曲率，世界坐标系
    double linear_velocity;       // 车辆总速度，车体坐标系
    double angular_velocity;      // 车辆航向角速度，车体坐标系
    double linear_acceleration;   // 车辆纵向加速度，车体坐标系
    Chassis::GearPosition gear;   // 车辆档位
    Chassis::DrivingMode driving_mode;
    Pose pose;                    // 车辆定位位姿
    double steering_percentage;   // 方向盘转角
} VehicleState;

} // common_msg
} // control
#endif