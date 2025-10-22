#ifndef LON_BASED_PID_CONTROLLER_CONF_H
#define LON_BASED_PID_CONTROLLER_CONF_H 

#include <string>
#include <vector>
#include "control_component/config/leadlag_conf.h"
#include "control_component/config/pid_conf.h"


namespace control {
typedef struct FilterConf {
   int32_t cutoff_freq;
} FilterConf;

typedef struct LON_BASED_PID_CONTROLLER_CONF
{
   double ts = 0.01;  // longitudinal controller sampling time 采样时间
   double brake_minimum_action = 0.0;
   double throttle_minimum_action = 0.0;
   double speed_controller_input_limit = 1.5;
   double station_error_limit = 2.0;
   double preview_window = 20.0;
   double standstill_acceleration = -0.3;

   common_msg::PidConf station_pid_conf;
   common_msg::PidConf low_speed_pid_conf;
   common_msg::PidConf high_speed_pid_conf;
   double switch_speed = 3.0;  // low/high speed controller switch speed
   common_msg::PidConf reverse_station_pid_conf;
   common_msg::PidConf reverse_speed_pid_conf;
   FilterConf pitch_angle_filter_conf;
   common_msg::LeadlagConf reverse_station_leadlag_conf;
   common_msg::LeadlagConf reverse_speed_leadlag_conf;

   bool enable_reverse_leadlag_compensation = false;

  // low/high speed switch transition-window
   double switch_speed_window = 0.0;

  // from gflags and control_conf.proto
   bool enable_speed_station_preview = false;
   bool enable_slope_offset = false;
   double max_path_remain_when_stopped = 0.3;
   bool use_acceleration_lookup_limit = false;
   bool use_preview_reference_check = false;
   double steer_cmd_interval = 0.0;
   bool use_steering_check = false;
   double pedestrian_stop_time = 10.0;
   double standstill_narmal_acceleration = 0.0;
   double full_stop_long_time = 0.0;

  // pid parameter in pit
   common_msg::PidConf pit_station_pid_conf;
   common_msg::PidConf pit_speed_pid_conf;
   double pit_replan_check_time = 14.0;
   int32_t pit_replan_check_count = 3;

   int32_t epb_change_count = 2;

   double stop_gain_acceleration = -1.0;

   bool use_vehicle_epb = false;

   double full_stop_path_remain_gain = 0.3;

   int32_t use_opposite_slope_compensation = 1;

   double speed_itfc_full_stop_speed = 0.09;

   double speed_itfc_path_remain_min = 0.10;

   double speed_itfc_dcc_emergency = -1.5;

   double speed_itfc_speed_cmd = 0.10;

   double speed_itfc_path_remain_max = 0.60;

   double speed_itfc_acc_thres = 0.0;

   bool use_speed_itfc = false;
} LonBasedPidControllerConf;


} // control  

#endif