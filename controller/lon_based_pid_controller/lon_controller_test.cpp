#include "lon_controller_test.h"

namespace control {

LonControllerTest::LonControllerTest(){}

LonControllerTest::~LonControllerTest() {}

double LonControllerTest::GetSystemTimeSeconds() const {
        return std::chrono::duration<double>
               (std::chrono::steady_clock::now().time_since_epoch()).count();
    }

void LonControllerTest::ComputeLongitudinalErrors(
    TrajectoryAnalyzer *trajectory_analyzer,
    const double preview_time,
    const double ts,
    common_msg::SimpleLongitudinalDebug *debug){
    double s_matched = 0.0;      // 匹配点纵向距离
    double s_dot_matched = 0.0;  // 匹配点纵向速度
    double d_matched = 0.0;      // 匹配点横向距离
    double d_dot_matched = 0.0;  // 匹配点横向速度

    auto  vehicle_state = injector_->vehicle_state();
    // 计算匹配点
    auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
        vehicle_state->x(), vehicle_state->y());
    // 坐标系转换 VRF->FreNet
    trajectory_analyzer->ToTrajectoryFrame(
        vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
        vehicle_state->linear_velocity(), matched_point,
        &s_matched, &s_dot_matched,
        &d_matched, &d_dot_matched);
    std::cout << "[s_matched]: " << s_matched << " "
              << "[s_dot_matched]: " << s_dot_matched << std::endl;
    // 这里的时间使用系统时间匹配
    double current_control_time = GetSystemTimeSeconds();
    double preview_control_time = current_control_time + preview_time;

   /**
    * 参考点ref_point:(x_ref, y_ref, ∅_ref, k_ref, v_ref, a_ref, s_ref, t_ref),    \
     其中t_ref为相对参考路径时间戳，k_ref为曲率，v_ref为速度，a_ref为加速度，s_ref为累计距离
   */
   common_msg::TrajectoryPoint reference_point = 
    trajectory_analyzer->QueryNearestPointByAbsoluteTime(current_control_time);  // 时间采用系统时间
    // trajectory_analyzer->QueryNearestPointByPosition(matched_point.x, matched_point.y); // 位置采用匹配点位置
   // 预瞄点preview_point:(x_pre, y_pre, ∅_pre, k_pre, v_pre, a_pre, s_pre, t_pre)
   common_msg::TrajectoryPoint preview_point = 
    trajectory_analyzer->QueryNearestPointByAbsoluteTime(preview_control_time);
   // 将匹配点、参考点、预瞄点的坐标（x, y）存放到debug中
   debug->current_matched_point.path_point.x = matched_point.x;
   debug->current_matched_point.path_point.y = matched_point.y;
   debug->current_reference_point.path_point.x = reference_point.path_point.x;
   debug->current_reference_point.path_point.y = reference_point.path_point.y;
   debug->preview_reference_point.path_point.x = preview_point.path_point.x;
   debug->preview_reference_point.path_point.y = preview_point.path_point.y;
   std::cerr << "[matched_point.x]: " << matched_point.x << " " 
             << "[matched_point.y]: " << matched_point.y << "\n" 
             << "[reference_point.path_point.x]: " << reference_point.path_point.x << " " 
             << "[reference_point.path_point.y]: " << reference_point.path_point.y << "\n" 
             << "[preview_point.path_point.x]: " << preview_point.path_point.x << " " 
             << "[preview_point.path_point.y]: " << preview_point.path_point.y 
             << std::endl;
    // 航向误差
    double heading_error = common::NormalizeAngle(
        vehicle_state->heading() - matched_point.theta);
    // 纵向速度
    double lon_speed = vehicle_state->linear_velocity() * std::cos(heading_error);
    // 纵向加速度
    double lon_acceleration = 
        vehicle_state->linear_acceleration() * std::cos(heading_error);
    // 横向最小曲率偏差（kappa->曲率）由全局坐标转换的Ft坐标引入，这里应该是：1-kd,但是计算中是1-k(∂d)，考虑动态横向曲率的变化
    double one_minus_kappa_lat_error = 1 - reference_point.path_point.kappa *
                                             vehicle_state->linear_velocity() *
                                             std::sin(heading_error);
    // Ft坐标系下
    // 参考点纵向位置
    debug->station_reference = (reference_point.path_point.s);
    // 车辆当前纵向位置
    debug->current_station = (s_matched);
    // 车辆纵向位置误差
    debug->station_error = (reference_point.path_point.s - s_matched);
    // 参考点纵向速度
    debug->speed_reference = (reference_point.v);
    // 车辆当前纵向速度
    debug->current_speed = (lon_speed);
    // 车辆纵向速度误差
    debug->speed_error = (reference_point.v - s_dot_matched);
    //   debug->speed_error = (matched_point.v - s_dot_matched);
    // 参考点纵向加速度
    debug->acceleration_reference = (reference_point.a);
    // 车辆当前纵向加速度
    debug->current_acceleration = (lon_acceleration);
    // 车辆纵向加速度误差
    debug->acceleration_error = (reference_point.a -
                                    lon_acceleration / one_minus_kappa_lat_error);
    // 参考点纵向加加速度
    double jerk_reference =
        (debug->acceleration_reference - previous_acceleration_reference_) / ts;
    // 车辆纵向加加速度 
    double lon_jerk =
        (debug->current_acceleration - previous_acceleration_) / ts;  // 加速度变化率
    debug->jerk_reference = (jerk_reference);
    debug->current_jerk = (lon_jerk);
    // 车辆纵向加加速度误差
    debug->jerk_error = (jerk_reference - lon_jerk / one_minus_kappa_lat_error);
    // 参考点加速度赋给上一次参考点加速度进行迭代
    previous_acceleration_reference_ = debug->acceleration_reference;
    // 当前加速度赋给上一次加速度进行迭代
    previous_acceleration_ = debug->current_acceleration;

    // 车辆与预瞄点纵向位置误差
    debug->preview_station_error = (preview_point.path_point.s - s_matched);
    // 车辆与预瞄点纵向速度误差
    debug->preview_speed_error = (preview_point.v - s_dot_matched);
    // 预瞄点纵向速度
    debug->preview_speed_reference = (preview_point.v);
    // 预瞄点纵向加速度
    debug->preview_acceleration_reference = (preview_point.a);
    if (lon_based_pidcontroller_conf_.use_speed_itfc) {
        reference_spd_ = reference_point.v;  // 参考点车速
    }
}

}