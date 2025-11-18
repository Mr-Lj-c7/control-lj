#include "config_gflags.h"

// 定义并初始化全局变量

// 地图
 string FLAGS_map_dir = "";                 // 地图目录
 int32_t FLAGS_local_utm_zone_id = 10;      // UTM坐标系ID

 string FLAGS_test_base_map_filename = "";  // 测试地图文件

 string FLAGS_base_map_filename = "";          // 地图文件
 string FLAGS_sim_map_filename = "";          // simulation仿真地图文件
 string FLAGS_routing_map_filename = "";      // 路线地图文件
 string FLAGS_end_way_point_filename = "";    // 地图终点文件
 string FLAGS_default_routing_filename = "";  // 默认环线地图文件

 string FLAGS_current_start_point_filename = "";  // 车辆起始点文件
 string FLAGS_park_go_routing_filename = "";      // 地图文件
 string FLAGS_speed_control_filename = "";        // 速度控制区域文件

// 车辆
 string FLAGS_vehicle_config_path = "../../common/data/vehicle_param.json";   
 string FLAGS_vehicle_model_filename = "";           // 车辆模型配置文件
 double FLAGS_half_vehicle_width = 1.05;             // 车辆宽度/2
 double FLAGS_look_forward_time_sec = 8.0;           // 预瞄时间s
 bool FLAGS_reverse_heading_vehicle_state = false;   // 倒车模式
 bool FLAGS_state_transform_to_com_reverse = false;  // 倒车模式车辆坐标系从后轴中心转换到车辆质心
 bool FLAGS_state_transform_to_com_drive = true;     // 前进模式车辆坐标系从后轴中心转换到车辆质心


 bool FLAGS_use_cyber_time = false;    // cyber时间

// 定位
 string FLAGS_localization_tf2_frame_id = "world";        // 定位基坐标TF2 frame_id
 string FLAGS_localization_tf2_child_frame_id = "localization";  // 定位子坐标TF2 frame_id
 bool FLAGS_enable_map_reference_unify = false;           // IMU数据转换为地图参考

// 导航
 bool FLAGS_use_navigation_mode = false;  // 导航模式
 string FLAGS_navigation_mode_end_way_point_file = "";  // 终点路径文件

 bool FLAGS_use_sim_time = false;       // simulation仿真时间
 bool FLAGS_multithread_run = false;    // 仿真-多线程

// 控制
 bool FLAGS_enable_csv_debug = true;   // csv调试
 string FLAGS_calibration_table_file = 
     "control_component/config/conf/calibration_table.pb.txt";  // 车辆标定表
 int16_t FLAGS_control_test_duration = 1;  // 控制测试时长
 bool FLAGS_is_control_test_mode = false;    // 控制测试模式
 bool FLAGS_use_preview_speed_for_table = false;    // 使用前一次速度
 double FLAGS_steer_angle_rate = 100;       // 方向盘转角频率
 bool FLAGS_enable_gain_scheduler = true;    // 补偿控制
 bool FLAGS_set_steer_limit = true;          // 方向盘转角限制
 bool FLAGS_use_vehicle_epb = false;          // epb制动

 uint32_t FLAGS_chassis_pending_queue_size = 10;   // 底盘消息队列大小
 uint32_t FLAGS_planning_pending_queue_size = 10;  // 规划消息队列大小
 uint32_t FLAGS_localization_pending_queue_size = 10;  // 定位消息队列大小
 uint32_t FLAGS_pad_msg_pending_queue_size = 10;   // pad消息队列大小

 bool FLAGS_reverse_heading_control = false;          // 倒车控制
 bool FLAGS_query_forward_time_point_only = false;      // 查询时间点
 bool FLAGS_query_forward_station_point_only = false;   // 查询位置点
 bool FLAGS_enable_gear_drive_negative_speed_protection = false;
 bool FLAGS_use_control_submodules = false;        // 控制子模块
 bool FLAGS_enable_input_timestamp_check = false;  // 输入时间戳检查
 int32_t FLAGS_max_localization_miss_num = 20;  // 定位消息最大丢失数
 int32_t FLAGS_max_chassis_miss_num = 20;    // 底盘消息最大丢失数
 int32_t FLAGS_max_planning_miss_num = 5;   // 规划消息最大丢失数
 double FLAGS_max_acceleration_when_stopped = 0.01;   // 停车最大加速度
 bool FLAGS_enable_persistent_estop = false;   // 长时间停车

 double FLAGS_control_period =  0.01;      // 控制周期
 double FLAGS_trajectory_period = 0.1;   // 规划周期
 double FLAGS_chassis_period = 0.01;      // 底盘消息周期
 double FLAGS_localization_period = 0.01; // 定位消息周期
 double FLAGS_soft_estop_brake = 15.0;    // 缓慢制动
 double FLAGS_minimum_speed_resolution = 0.2;      // 最小速度分辨率
 double FLAGS_minimum_speed_protection = 0.1;      // 最小保护速度
 double FLAGS_max_path_remain_when_stopped = 0.2;  // 距离终点最大路径
 int8_t FLAGS_action = 1;  
 bool FLAGS_use_calibration_dimension_equal_check = false;  // 使用车辆标定表

double FLAGS_throttle_smoothing_factor = 0.05; // 油门平滑因子
double FLAGS_speed_smoothing_factor = 0.05;    // 速度平滑因子
double FLAGS_pitch_offset_deg = 0.0;          // 传感器偏移角
bool FLAGS_use_throttle_filter = false;         // 使用油门滤波
bool FLAGS_use_speed_filter = false;            // 使用速度滤波
bool FLAGS_publish_control_debug_info = false;  // 发布控制调试信息
bool FLAGS_is_control_ut_test_mode = false;     // 控制单元测试模式

int32_t FLAGS_planning_status_msg_pending_queue_size = 10;   // 最大缓冲消息队列 




 inline void InitGflags(){
    // 地图
     FLAGS_map_dir = "";                 // 地图目录
     FLAGS_local_utm_zone_id = 10;      // UTM坐标系ID

     FLAGS_test_base_map_filename = "";  // 测试地图文件

     FLAGS_base_map_filename = "";          // 地图文件
     FLAGS_sim_map_filename = "";          // simulation仿真地图文件
     FLAGS_routing_map_filename = "";      // 路线地图文件
     FLAGS_end_way_point_filename = "";    // 地图终点文件
     FLAGS_default_routing_filename = "";  // 默认环线地图文件

     FLAGS_current_start_point_filename = "";  // 车辆起始点文件
     FLAGS_park_go_routing_filename = "";      // 地图文件
     FLAGS_speed_control_filename = "";        // 速度控制区域文件

    // 车辆
     FLAGS_vehicle_config_path = 
        "common/data/vehicle_param.json";   
     FLAGS_vehicle_model_filename = "";           // 车辆模型配置文件
     FLAGS_half_vehicle_width = 1.05;             // 车辆宽度/2
     FLAGS_look_forward_time_sec = 8.0;           // 预瞄时间s
     FLAGS_reverse_heading_vehicle_state = false;   // 倒车模式
     FLAGS_state_transform_to_com_reverse = false;  // 倒车模式车辆坐标系从后轴中心转换到车辆质心
     FLAGS_state_transform_to_com_drive = true;     // 前进模式车辆坐标系从后轴中心转换到车辆质心


     FLAGS_use_cyber_time = false;    // cyber时间

    // 定位
     FLAGS_localization_tf2_frame_id = "world";        // 定位基坐标TF2 frame_id
     FLAGS_localization_tf2_child_frame_id = "localization";  // 定位子坐标TF2 frame_id
     FLAGS_enable_map_reference_unify = true;           // IMU数据转换为地图参考

    // 导航
     FLAGS_use_navigation_mode = false;  // 导航模式
     FLAGS_navigation_mode_end_way_point_file = "";  // 终点路径文件

     FLAGS_use_sim_time = false;       // simulation仿真时间
     FLAGS_multithread_run = false;    // 仿真-多线程

     // 控制
     FLAGS_enable_csv_debug = false;   // csv调试
     FLAGS_calibration_table_file = 
        "control_component/config/conf/calibration_table.pb.txt";  // 车辆标定表
     FLAGS_control_test_duration = 1;  // 控制测试时长
     FLAGS_is_control_test_mode = false;    // 控制测试模式
     FLAGS_use_preview_speed_for_table = false;    // 使用前一次速度
     FLAGS_steer_angle_rate = 100;       // 方向盘转角频率
     FLAGS_enable_gain_scheduler = true;    // 补偿控制
     FLAGS_set_steer_limit = true;          // 方向盘转角限制
     FLAGS_use_vehicle_epb = false;          // epb制动

     FLAGS_chassis_pending_queue_size = 10;   // 底盘消息队列大小
     FLAGS_planning_pending_queue_size = 10;  // 规划消息队列大小
     FLAGS_localization_pending_queue_size = 10;  // 定位消息队列大小
     FLAGS_pad_msg_pending_queue_size = 10;   // pad消息队列大小

     FLAGS_reverse_heading_control = false;          // 倒车控制
     FLAGS_query_forward_time_point_only = false;      // 查询时间点
     FLAGS_query_forward_station_point_only = false;   // 查询位置点
     FLAGS_enable_gear_drive_negative_speed_protection = false;
     FLAGS_use_control_submodules = false;        // 控制子模块
     FLAGS_enable_input_timestamp_check = false;  // 输入时间戳检查
     FLAGS_max_localization_miss_num = 20;  // 定位消息最大丢失数
     FLAGS_max_chassis_miss_num = 20;    // 底盘消息最大丢失数
     FLAGS_max_planning_miss_num = 5;   // 规划消息最大丢失数
     FLAGS_max_acceleration_when_stopped = 0.01;   // 停车最大加速度
     FLAGS_enable_persistent_estop = false;   // 长时间停车

     FLAGS_control_period =  0.01;      // 控制周期
     FLAGS_trajectory_period = 0.1;   // 规划周期
     FLAGS_chassis_period = 0.01;      // 底盘消息周期
     FLAGS_localization_period = 0.01; // 定位消息周期
     FLAGS_soft_estop_brake = 15.0;    // 缓慢制动
     FLAGS_minimum_speed_resolution = 0.2;      // 最小速度分辨率
     FLAGS_minimum_speed_protection = 0.1;      // 最小保护速度
     FLAGS_max_path_remain_when_stopped = 0.2;  // 距离终点最大路径
     FLAGS_action = 1;  
     FLAGS_use_calibration_dimension_equal_check = false;  // 使用车辆标定表

     
    FLAGS_throttle_smoothing_factor = 0.05; // 油门平滑因子
    FLAGS_speed_smoothing_factor = 0.05;    // 速度平滑因子
    FLAGS_pitch_offset_deg = 0.0;          // 传感器偏移角
    FLAGS_use_throttle_filter = false;         // 使用油门滤波
    FLAGS_use_speed_filter = false;            // 使用速度滤波
    FLAGS_publish_control_debug_info = false;  // 发布控制调试信息
    FLAGS_is_control_ut_test_mode = false;     // 控制单元测试模式

    FLAGS_planning_status_msg_pending_queue_size = 10;   // 最大缓冲消息队列  
 }