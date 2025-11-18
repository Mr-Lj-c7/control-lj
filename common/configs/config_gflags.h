#ifndef CONFIG_GFLAGS_H
#define CONFIG_GFLAGS_H

#include <string>

using namespace std;

// 全局配置声明
// 地图
extern string FLAGS_map_dir;             // 地图目录
extern int32_t FLAGS_local_utm_zone_id;  // UTM坐标系ID

extern string FLAGS_test_base_map_filename;  // 测试地图文件

extern string FLAGS_base_map_filename;  // 地图文件
extern string FLAGS_sim_map_filename;   // simulation仿真地图文件
extern string FLAGS_routing_map_filename;      // 路线地图文件
extern string FLAGS_end_way_point_filename;    // 地图终点文件
extern string FLAGS_default_routing_filename;  // 默认环线地图文件

extern string FLAGS_current_start_point_filename;  // 车辆起始点文件
extern string FLAGS_park_go_routing_filename;      // 地图文件
extern string FLAGS_speed_control_filename;        // 速度控制区域文件

// 车辆
extern string FLAGS_vehicle_config_path;          // 车辆参数文件
extern string FLAGS_vehicle_model_filename;       // 车辆模型配置文件
extern double FLAGS_half_vehicle_width;           // 车辆宽度/2
extern double FLAGS_look_forward_time_sec;        // 预瞄时间
extern bool FLAGS_reverse_heading_vehicle_state;  // 倒车模式
extern bool FLAGS_state_transform_to_com_reverse; // 倒车模式车辆坐标系从后轴中心转换到车辆质心
extern bool FLAGS_state_transform_to_com_drive;   // 前进模式车辆坐标系从后轴中心转换到车辆质心

extern bool FLAGS_use_cyber_time;    // cyber时间

// 定位
extern string FLAGS_localization_tf2_frame_id;        // 定位基坐标TF2 frame_id
extern string FLAGS_localization_tf2_child_frame_id;  // 定位子坐标TF2 frame_id
extern bool FLAGS_enable_map_reference_unify;         // IMU数据转换为地图参考

// 导航
extern bool FLAGS_use_navigation_mode;  // 导航模式
extern string FLAGS_navigation_mode_end_way_point_file;  // 终点路径文件

extern bool FLAGS_use_sim_time;       // simulation仿真时间
extern bool FLAGS_multithread_run;    // 仿真-多线程

// 控制
extern bool FLAGS_enable_csv_debug;   // csv调试
extern string FLAGS_calibration_table_file;  // 车辆标定表
extern int16_t FLAGS_control_test_duration;  // 控制测试时长
extern bool FLAGS_is_control_test_mode;      // 控制测试模式
extern bool FLAGS_use_preview_speed_for_table;    // 使用前一次速度
extern double FLAGS_steer_angle_rate;       // 方向盘转角频率
extern bool FLAGS_enable_gain_scheduler;    // 补偿控制
extern bool FLAGS_set_steer_limit;          // 方向盘转角限制
extern bool FLAGS_use_vehicle_epb;          // epb制动

extern uint32_t FLAGS_chassis_pending_queue_size;   // 底盘消息队列大小
extern uint32_t FLAGS_planning_pending_queue_size;  // 规划消息队列大小
extern uint32_t FLAGS_localization_pending_queue_size;  // 定位消息队列大小
extern uint32_t FLAGS_pad_msg_pending_queue_size;   // pad消息队列大小

extern bool FLAGS_reverse_heading_control;          // 倒车控制
extern bool FLAGS_query_forward_time_point_only;      // 查询时间点
extern bool FLAGS_query_forward_station_point_only;   // 查询位置点
extern bool FLAGS_enable_gear_drive_negative_speed_protection;
extern bool FLAGS_use_control_submodules;        // 控制子模块
extern bool FLAGS_enable_input_timestamp_check;  // 输入时间戳检查
extern int32_t FLAGS_max_localization_miss_num;  // 定位消息最大丢失数
extern int32_t FLAGS_max_chassis_miss_num;    // 底盘消息最大丢失数
extern int32_t FLAGS_max_planning_miss_num;   // 规划消息最大丢失数
extern double FLAGS_max_acceleration_when_stopped;   // 停车最大加速度
extern bool FLAGS_enable_persistent_estop;   // 长时间停车

extern double FLAGS_control_period;      // 控制周期
extern double FLAGS_trajectory_period;   // 规划周期
extern double FLAGS_chassis_period;      // 底盘消息周期
extern double FLAGS_localization_period; // 定位消息周期
extern double FLAGS_soft_estop_brake;    // 缓慢制动
extern double FLAGS_minimum_speed_resolution;      // 最小速度分辨率
extern double FLAGS_minimum_speed_protection;      // 最小保护速度
extern double FLAGS_max_path_remain_when_stopped;  // 距离终点最大路径
extern int8_t FLAGS_action;  
extern bool FLAGS_use_calibration_dimension_equal_check;  // 使用车辆标定表

extern double FLAGS_throttle_smoothing_factor; // 油门平滑因子
extern double FLAGS_speed_smoothing_factor;    // 速度平滑因子
extern double FLAGS_pitch_offset_deg;          // 传感器偏移角
extern bool FLAGS_use_throttle_filter;         // 使用油门滤波
extern bool FLAGS_use_speed_filter;            // 使用速度滤波
extern bool FLAGS_publish_control_debug_info;  // 发布控制调试信息
extern bool FLAGS_is_control_ut_test_mode;     // 控制单元测试模式

extern int32_t FLAGS_planning_status_msg_pending_queue_size;   // 规划消息队列  

#endif 