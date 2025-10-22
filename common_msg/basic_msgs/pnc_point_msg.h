#ifndef PNC_POINT_MSG_H
#define PNC_POINT_MSG_H

#include <string>
#include <cmath>
#include <vector>

namespace control {
namespace common_msg {


typedef struct SpeedPoint {
   double s ;
   double t ;
  // speed (m/s)
   double v ;
  // acceleration (m/s^2)
   double a ;
  // jerk (m/s^3)
   double da ;
}SpeedPoint;

typedef struct SLPOINT {
   double s;
   double l;
} SLPoint;

typedef struct GAUSSIAN_INFO
{
    // 高斯分布参数
    double sigma_x;
    double sigma_y;
    double correlation;
    // 不确定区域参数
    double area_probability; 
    double ellipse_a; 
    double ellipse_b; 
    double theta_a; 
} GaussianInfo;

typedef struct PATH_POINT
{
    // 全局坐标
    double x;
    double y;
    double z;
    
    double theta;  // 航向角
    double kappa;  // 曲率
    double s;      // 路径长度
    
    double dkappa;
    double ddkappa;
    double lane_id;  // 路径ID
    
    double x_derivative;  // 纵向导数v_x
    double y_derivative;  // 横向导数v_y
} PathPoint;

typedef struct TRAJECTORY_POINT
{
    PathPoint path_point;  // 路径点
    double v;              // 纵向速度 
    double a;              // 纵向加速度
    double relative_time;  // 轨迹起点相对时间
    double da;             // 纵向 Jerk
    double steer;          // 前轮转角
    GaussianInfo gaussian_info;  // 高斯概率信息
} TrajectoryPoint;

typedef struct PATH {
   std::string name;
   PathPoint path_point;
} Path;

}  // namespace common_msg
}  // namespace control

#endif