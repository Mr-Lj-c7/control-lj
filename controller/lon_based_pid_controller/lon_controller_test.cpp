#include "lon_controller_test.h"

namespace control {
namespace {
constexpr double GRA_ACC = 9.8;  // 重力加速度9.8m/s^2
// 日志表头
void WriteHeader(std::ofstream& file_stream) {
    file_stream << "station_reference, "
        << "station_error, "
        << "station_error_limited,"
        << "preview_station_error,"
        << "speed_reference,"
        << "speed_error,"
        << "speed_error_limited,"
        << "preview_speed_reference,"
        << "preview_speed_error,"
        << "preview_acceleration_reference,"
        << "acceleration_cmd_closeloop,"
        << "acceleration_cmd,"
        << "acceleration_lookup,"
        << "acceleration_lookup_limit,"
        << "speed_lookup,"
        << "calibration_value,"
        << "throttle_cmd,"
        << "brake_cmd,"
        << "is_full_stop,"
        << "\r\n";
}
}

LonControllerTest::LonControllerTest() : name_("PID-BASED_PID_CONTROLLER"){
  // 初始化日志记录器    
  if (FLAGS_enable_csv_debug) {
    time_t rawtime;
    char name_buffer[100];
    std::time(&rawtime);  // 系统时间
    std::tm time_tm;      // 时间结构体
    #ifdef _WIN32
        localtime_s(&rawtime, &time_tm);
    #else  
        localtime_r(&rawtime, &time_tm);
    #endif
    // 标准格式输出
    strftime(name_buffer, 100, 
        "../../controller/lon_based_pid_controller/csv/speed_log_simple_optimmal_%F_%H%M%S.csv", 
        &time_tm);
    speed_log_file_.open(std::string(name_buffer));
    speed_log_file_ << std::fixed;
    speed_log_file_ << std::setprecision(6);
    WriteHeader(speed_log_file_);
  }
    std::cout << "Using: " << name_ << std::endl;
}

LonControllerTest::~LonControllerTest() { CloseLogFile(); }

std::string LonControllerTest::Name() const { return name_; }

void LonControllerTest::CloseLogFile() {
  if (FLAGS_enable_csv_debug && speed_log_file_.is_open()) {
    speed_log_file_.close();
  }
}

// 系统时间
double LonControllerTest::GetSystemTimeSeconds() const {
    return std::chrono::duration<double>
            (std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 加载油门-刹车-加速度标定表
bool LonControllerTest::LoadCalibrationTable(const std::string& calibration_table_path_){
    if (calibration_table_.calibration.size() > 0) {
        calibration_table_.calibration.clear();
    }
    std::ifstream file(calibration_table_path_);
    if (!file.is_open()) { 
        std::cerr << "[LonControllerTest]: Failed to open calibration table file: "
                << calibration_table_path_ << std::endl;
        return false;
    }
    Json::Value root_;
    Json::Reader reader_;
    if (!reader_.parse(file, root_)) {
        std::cerr << "[LonControllerTest]: Failed to parse calibration table file: "
                << calibration_table_path_ << std::endl;
    }
    if (root_.isMember("calibration")) {
      Json::Value calibration_array = root_["calibration"];
      uint32_t data_size = calibration_array.size();
      std::cerr << "[LonControllerTest]: Calibration table size: " 
                << data_size << std::endl;
      for (Json::ArrayIndex i = 0; i < data_size; ++i) {
        common_msg::ControlCalibrationInfo calibration_info;
        calibration_info.acceleration = calibration_array[i]["acceleration"].asDouble();
        calibration_info.command = calibration_array[i]["command"].asDouble();
        calibration_info.speed = calibration_array[i]["speed"].asDouble();
      // if (calibration_info.speed == 2.2) {
      //   std::cerr << "[LonControllerTest]: Speed 2.0 data - acceleration: " 
      //             << calibration_info.acceleration 
      //             << ", command: " << calibration_info.command << std::endl;
      // }
        calibration_table_.calibration.push_back(calibration_info);
      }
    }
    return true;
}

// 初始化车辆标定表
void LonControllerTest::InitControlCalibrationTable() {
  std::cout << "Control Calibration table size is " 
            << calibration_table_.calibration.size() << std::endl;
  // 二维插值器存储标定关系数据
  common::Interpolation2D::DataType xyz;
  for (const auto& calibration : calibration_table_.calibration) {
    // 三维元组存储
    xyz.push_back(std::make_tuple(calibration.speed, 
      calibration.acceleration, calibration.command));
  }
  std::cout << "xyz table size is " 
            << xyz.size() << std::endl;
  // 二维插值器初始化(以speed,acceleration为查找键，利用插值算法计算control_command值)
  control_interpolation_.reset(new common::Interpolation2D);
  if (!control_interpolation_->Init(xyz)) {
    std::cerr << "[LonControllerTest]:Failed to load control calibration table" 
      << std::endl;
  }
}

// 加载纵向控制器参数
bool LonControllerTest::LoadLonBasedPIDControllerConf(
    const std::string &lon_based_pidcontroller_config_file) {
    std::ifstream file(lon_based_pidcontroller_config_file);
    if (!file.is_open()) {
        std::cerr << "[LonControllerTest]: Failed to load lon control parameter file: "
                << lon_based_pidcontroller_config_file << std::endl;
        return false;
    }
    Json::Value root_;
    Json::Reader reader_;
    if (!reader_.parse(file, root_)) {
        std::cerr << "[LonControllerTest]: Failed to parse lon control parameter file: "
                << lon_based_pidcontroller_config_file << std::endl;
        return false;
    }
    lon_based_pidcontroller_conf_.ts = root_["ts"].asDouble();
    lon_based_pidcontroller_conf_.brake_minimum_action = 
      root_["brake_minimum_action"].asDouble();
    lon_based_pidcontroller_conf_.throttle_minimum_action = 
      root_["throttle_minimum_action"].asDouble();
    lon_based_pidcontroller_conf_.speed_controller_input_limit = 
      root_["speed_controller_input_limit"].asDouble();
    lon_based_pidcontroller_conf_.station_error_limit = 
      root_["station_error_limit"].asDouble();
    lon_based_pidcontroller_conf_.preview_window = 
      root_["preview_window"].asDouble();
    lon_based_pidcontroller_conf_.standstill_acceleration = 
      root_["standstill_acceleration"].asDouble();
    lon_based_pidcontroller_conf_.steer_cmd_interval = 
      root_["steer_cmd_interval"].asDouble();
    lon_based_pidcontroller_conf_.enable_reverse_leadlag_compensation = 
      root_["enable_reverse_leadlag_compensation"].asBool();
    lon_based_pidcontroller_conf_.use_preview_reference_check = 
      root_["use_preview_reference_check"].asBool();
    lon_based_pidcontroller_conf_.enable_speed_station_preview = 
      root_["enable_speed_station_preview"].asBool();
    lon_based_pidcontroller_conf_.enable_slope_offset = 
      root_["enable_slope_offset"].asBool();
    lon_based_pidcontroller_conf_.use_acceleration_lookup_limit = 
      root_["use_acceleration_lookup_limit"].asBool();
    lon_based_pidcontroller_conf_.use_steering_check = 
      root_["use_steering_check"].asBool();
    lon_based_pidcontroller_conf_.pedestrian_stop_time = 
      root_["pedestrian_stop_time"].asDouble();
    lon_based_pidcontroller_conf_.standstill_narmal_acceleration = 
      root_["standstill_narmal_acceleration"].asDouble();
    lon_based_pidcontroller_conf_.full_stop_long_time = 
      root_["full_stop_long_time"].asDouble();
    lon_based_pidcontroller_conf_.epb_change_count = 
      root_["epb_change_count"].asInt();
    lon_based_pidcontroller_conf_.stop_gain_acceleration = 
      root_["stop_gain_acceleration"].asDouble();
    lon_based_pidcontroller_conf_.full_stop_path_remain_gain = 
      root_["full_stop_path_remain_gain"].asDouble();
    lon_based_pidcontroller_conf_.use_opposite_slope_compensation = 
      root_["use_opposite_slope_compensation"].asInt();
    lon_based_pidcontroller_conf_.speed_itfc_full_stop_speed = 
      root_["speed_itfc_full_stop_speed"].asDouble();
    lon_based_pidcontroller_conf_.speed_itfc_path_remain_min = 
      root_["speed_itfc_path_remain_min"].asDouble();
    lon_based_pidcontroller_conf_.speed_itfc_dcc_emergency = 
      root_["speed_itfc_dcc_emergency"].asDouble();
    lon_based_pidcontroller_conf_.speed_itfc_speed_cmd = 
      root_["speed_itfc_speed_cmd"].asDouble();
    lon_based_pidcontroller_conf_.speed_itfc_path_remain_max = 
      root_["speed_itfc_path_remain_max"].asDouble();
    lon_based_pidcontroller_conf_.speed_itfc_acc_thres = 
      root_["speed_itfc_acc_thres"].asDouble();
    lon_based_pidcontroller_conf_.switch_speed = 
      root_["switch_speed"].asDouble();
    lon_based_pidcontroller_conf_.switch_speed_window = 
      root_["switch_speed_window"].asDouble();
    if (root_.isMember("station_pid_conf")) {
        lon_based_pidcontroller_conf_.station_pid_conf.integrator_enable
           = root_["integrator_enable"].asBool();
        lon_based_pidcontroller_conf_.station_pid_conf.integrator_saturation_level
           = root_["integrator_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.station_pid_conf.kp
           = root_["kp"].asDouble();
        lon_based_pidcontroller_conf_.station_pid_conf.ki
           = root_["ki"].asDouble();
        lon_based_pidcontroller_conf_.station_pid_conf.kd
           = root_["kd"].asDouble();
    }
    if (root_.isMember("low_speed_pid_conf")) {
        lon_based_pidcontroller_conf_.low_speed_pid_conf.integrator_enable
           = root_["integrator_enable"].asBool();
        lon_based_pidcontroller_conf_.low_speed_pid_conf.integrator_saturation_level
           = root_["integrator_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.low_speed_pid_conf.kp
           = root_["kp"].asDouble();
        lon_based_pidcontroller_conf_.low_speed_pid_conf.ki
           = root_["ki"].asDouble();
        lon_based_pidcontroller_conf_.low_speed_pid_conf.kd
           = root_["kd"].asDouble();
    }
    if (root_.isMember("high_speed_pid_conf")) {
        lon_based_pidcontroller_conf_.high_speed_pid_conf.integrator_enable
           = root_["integrator_enable"].asBool();
        lon_based_pidcontroller_conf_.high_speed_pid_conf.integrator_saturation_level
           = root_["integrator_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.high_speed_pid_conf.kp
           = root_["kp"].asDouble();
        lon_based_pidcontroller_conf_.high_speed_pid_conf.ki
           = root_["ki"].asDouble();
        lon_based_pidcontroller_conf_.high_speed_pid_conf.kd
           = root_["kd"].asDouble();
    }
    if (root_.isMember("reverse_station_pid_conf")) {
        lon_based_pidcontroller_conf_.reverse_station_pid_conf.integrator_enable
           = root_["integrator_enable"].asBool();
        lon_based_pidcontroller_conf_.reverse_station_pid_conf.integrator_saturation_level
           = root_["integrator_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.reverse_station_pid_conf.kp
           = root_["kp"].asDouble();
        lon_based_pidcontroller_conf_.reverse_station_pid_conf.ki
           = root_["ki"].asDouble();
        lon_based_pidcontroller_conf_.reverse_station_pid_conf.kd
           = root_["kd"].asDouble();
    }
    if (root_.isMember("reverse_speed_pid_conf")) {
        lon_based_pidcontroller_conf_.reverse_speed_pid_conf.integrator_enable
           = root_["integrator_enable"].asBool();
        lon_based_pidcontroller_conf_.reverse_speed_pid_conf.integrator_saturation_level
           = root_["integrator_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.reverse_speed_pid_conf.kp
           = root_["kp"].asDouble();
        lon_based_pidcontroller_conf_.reverse_speed_pid_conf.ki
           = root_["ki"].asDouble();
        lon_based_pidcontroller_conf_.reverse_speed_pid_conf.kd
           = root_["kd"].asDouble();
    }
    if (root_.isMember("reverse_station_leadlag_conf")) {
        lon_based_pidcontroller_conf_.reverse_station_leadlag_conf.innerstate_saturation_level
           = root_["innerstate_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.reverse_station_leadlag_conf.alpha
           = root_["alpha"].asDouble();
        lon_based_pidcontroller_conf_.reverse_station_leadlag_conf.beta
           = root_["beta"].asDouble();
        lon_based_pidcontroller_conf_.reverse_station_leadlag_conf.tau
           = root_["tau"].asDouble();
    }
    if (root_.isMember("reverse_speed_leadlag_conf")) {
        lon_based_pidcontroller_conf_.reverse_speed_leadlag_conf.innerstate_saturation_level
           = root_["innerstate_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.reverse_speed_leadlag_conf.alpha
           = root_["alpha"].asDouble();
        lon_based_pidcontroller_conf_.reverse_speed_leadlag_conf.beta
           = root_["beta"].asDouble();
        lon_based_pidcontroller_conf_.reverse_speed_leadlag_conf.tau
           = root_["tau"].asDouble();
    }
    if (root_.isMember("pitch_angle_filter_conf")) {
        lon_based_pidcontroller_conf_.pitch_angle_filter_conf.cutoff_freq
           = root_["cutoff_freq"].asInt();
    }
    if (root_.isMember("pit_station_pid_conf")) {
        lon_based_pidcontroller_conf_.pit_station_pid_conf.integrator_enable
            = root_["integrator_enable"].asBool();
        lon_based_pidcontroller_conf_.pit_station_pid_conf.integrator_saturation_level
            = root_["integrator_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.pit_station_pid_conf.kp
            = root_["kp"].asDouble();
        lon_based_pidcontroller_conf_.pit_station_pid_conf.ki
            = root_["ki"].asDouble();
        lon_based_pidcontroller_conf_.pit_station_pid_conf.kd
            = root_["kd"].asDouble();
    }
    if (root_.isMember("pit_speed_pid_conf")) {
        lon_based_pidcontroller_conf_.pit_speed_pid_conf.integrator_enable
            = root_["integrator_enable"].asBool();
        lon_based_pidcontroller_conf_.pit_speed_pid_conf.integrator_saturation_level
            = root_["integrator_saturation_level"].asDouble();
        lon_based_pidcontroller_conf_.pit_speed_pid_conf.kp
            = root_["kp"].asDouble();
        lon_based_pidcontroller_conf_.pit_speed_pid_conf.ki
            = root_["ki"].asDouble();
        lon_based_pidcontroller_conf_.pit_speed_pid_conf.kd
            = root_["kd"].asDouble();
    }
    return true;
}

// 初始化纵向控制系统
bool LonControllerTest::Init(std::shared_ptr<DependencyInjector> injector) {
  if (!LoadLonBasedPIDControllerConf(lon_based_pidcontroller_conf_path_)) {
    std::cerr << "[LonControllerTest]:Failde to load lon pidcontroller conf.\n";
    return false;
  }
  if (!LoadCalibrationTable(calibration_table_path_)) {
    std::cerr << "[LonControllerTest]: Failed to load calibration table.\n";
    return false;
  }
  injector_ = injector;
  // 停车加速度
  standstill_narmal_acceleration_ = 
    -fabs(lon_based_pidcontroller_conf_.standstill_narmal_acceleration);
  // 急刹加速度
  stop_gain_acceleration_ = 
    -fabs(lon_based_pidcontroller_conf_.stop_gain_acceleration);
  // pid控制周期
  double ts = lon_based_pidcontroller_conf_.ts;
  // 前馈补偿控制(倒车)
  bool enable_leadlag = 
    lon_based_pidcontroller_conf_.enable_reverse_leadlag_compensation;
  // 初始化位置PID控制器
  station_pid_controller_.Init(
    lon_based_pidcontroller_conf_.station_pid_conf);
  // 初始化速度PID控制器
  speed_pid_controller_.Init(
    lon_based_pidcontroller_conf_.low_speed_pid_conf);
  // 前馈补偿控制器初始化（倒车）
  if (enable_leadlag) { 
    station_leadlag_controller_.Init(
      lon_based_pidcontroller_conf_.reverse_station_leadlag_conf, ts);
    speed_leadlag_controller_.Init(
      lon_based_pidcontroller_conf_.reverse_speed_leadlag_conf, ts);
  }
  // 车辆配置参数
  vehicle_param_ = common::VehicleConfigHelper::GetConfig();

  // 坡度滤波
  SetDigitalFilterPitchAngle();

  // 初始化车辆标定标
  InitControlCalibrationTable();

  controller_initialized_ = true;
  return true;
}

/**
 * @brief 计算停车点索引与停车距离(用于路径跟踪与停车判断) 
 * @param[in] debug
 * @return null
 */                                
void LonControllerTest::GetPathRemain(common_msg::SimpleLongitudinalDebug *debug) {
  int stop_index = 0;  // 停车点
  // 编译初始化变量
  static constexpr double KSpeedThreshold = 1e-3;        // 停车速度阈值
  static constexpr double KForwardAccThreshold = -1e-2;  // 前向加速度阈值（前进档）
  static constexpr double KBackwardAccThreshold = 1e-1;  // 后向加速度阈值（倒档）
  static constexpr double KParkingSpeed = 0.1;           // 停车最小速度
  // 前进档逻辑
  if (trajectory_message_->gear == common_msg::Chassis::GEAR_DRIVE) {
    while (stop_index < trajectory_message_->trajectory_point.size()) {
      auto &current_trajectory_point = 
        trajectory_message_->trajectory_point[stop_index];
      if (fabs(current_trajectory_point.v) < KSpeedThreshold && 
          current_trajectory_point.a > KForwardAccThreshold &&
          current_trajectory_point.a < 0.0) {
            break;
          }
          ++stop_index;
    }
  } else {  // 倒车逻辑
    while (stop_index < trajectory_message_->trajectory_point.size()) {
      auto &current_trajectory_point = 
        trajectory_message_->trajectory_point[stop_index];
      if (fabs(current_trajectory_point.a) > -KSpeedThreshold &&
          current_trajectory_point.v < KBackwardAccThreshold &&
          current_trajectory_point.v > 0.0) {
        break;
      }
      ++stop_index;
    }
  }
  std::cout << "stop_index: " << stop_index << std::endl;
  if (stop_index == trajectory_message_->trajectory_point.size()) {
    --stop_index;
    if (fabs(trajectory_message_->trajectory_point[stop_index].v) < KParkingSpeed) {
      std::cout << "the last point is select as patking point\n" ;
    } else {
      std::cerr << "the last point found in path and speed > speed_deadzone\n" ;
    }
  }
  // 停车距离
  debug->path_remain = 
    trajectory_message_->trajectory_point[stop_index].path_point.s - 
    debug->current_matched_point.path_point.s;
}

/**
 * @brief 设置滤波器
 * @param[in] ts 采样时间
 * @param[in] cutoff_freq 截止频率
 * @param[in] digital_filter 低通滤波器
*/
void LonControllerTest::SetDigitalFilter(double ts, double cutoff_freq, 
                      common::DigitalFilter *digital_filter) {
  std::vector<double> denominators;  // 分母系数
  std::vector<double> numerators;    // 分子系数
  // 计算低通滤波器系数
  common::LpCoefficients(ts, cutoff_freq, &denominators, &numerators);
  digital_filter->SetCoefficients(denominators, numerators);
}

// 坡度角滤波
void LonControllerTest::SetDigitalFilterPitchAngle() {
  double cutoff_freq = lon_based_pidcontroller_conf_.
                         pitch_angle_filter_conf.cutoff_freq;
  double ts = lon_based_pidcontroller_conf_.ts;
  SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}

/**
 * @brief 计算纵向误差
 * @param[in] trajectory    规划轨迹
 * @param[in] preview_time  预瞄时间
 * @param[in] ts            采样时间
 * @param[in] debug         调试变量
 * @return    void
*/
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

/**
 * @brief 终点停车
 * @param[in] debug  
 * @return ture:destination stop ,
 *         false:not destination stop
 */   
bool LonControllerTest::IsStopByDestination(common_msg::SimpleLongitudinalDebug *debug) {
  common_msg::StopReasonCode stop_reason_code = 
    trajectory_message_->decision.main_decision.task.stop.reason_code;
  // common_msg::StopReasonCode stop_reason_code = stop_reason.reason_code;
  if (stop_reason_code == common_msg::StopReasonCode::STOP_REASON_SIGNAL ||
      stop_reason_code == common_msg::StopReasonCode::STOP_REASON_REFERENCE_END ||
      stop_reason_code == common_msg::StopReasonCode::STOP_REASON_PRE_OPEN_SPACE_STOP) {
        debug->is_stop_reason_by_destination = true;
        return true;
      }
      debug->is_stop_reason_by_destination = false;
      return false;
}

/**
 * @brief 行人相关因素长时间停车
 * @param[in] debug 
*/
bool LonControllerTest::IsPedestrianStopLongTerm(common_msg::SimpleLongitudinalDebug *debug) {
  common_msg::StopReasonCode stop_reason_code = 
    trajectory_message_->decision.main_decision.task.stop.reason_code;
  // 行人、障碍物停车
  if (stop_reason_code == common_msg::StopReasonCode::STOP_REASON_PEDESTRIAN ||
      stop_reason_code == common_msg::StopReasonCode::STOP_REASON_OBSTACLE) {
        is_stop_by_pedestrian_ = true;
  } else {
    is_stop_by_pedestrian_ = false;
  }
  std::cout << "Current is_stop_by_pedestrian_: " << is_stop_by_pedestrian_ 
            << ", is_stop_by_pedestrian_previous: " 
            << is_stop_by_pedestrian_previous_ << std::endl;
  // 时间记录逻辑
  if (is_stop_by_pedestrian_) {
    if (!(is_stop_by_pedestrian_ && is_stop_by_pedestrian_previous_)) {  // 初次行人停车判断
      start_time_ = GetSystemTimeSeconds();
      std::cerr << "Stop reason for pedestrian, start time(s) is: " << start_time_ << std::endl;
    } else {
      std::cerr << "Last time stop is already pedestrian, skip start_time init.\n";
    }
    double end_time = GetSystemTimeSeconds();
    std::cerr << "Stop reason for pedestrian, current time(s) is " << end_time << "\n";
    wait_time_diff_ = end_time - start_time_;
  } else {
    start_time_ = 0.0;
    wait_time_diff_ = 0.0;
  }
  is_stop_by_pedestrian_previous_ = is_stop_by_pedestrian_;
  // 长期行人等待判断
  if (wait_time_diff_ > lon_based_pidcontroller_conf_.pedestrian_stop_time) {
    std::cout << "Current pedestrian stop lasting time(s) is: " << wait_time_diff_ 
      << ", larger than threshold: " 
      << lon_based_pidcontroller_conf_.pedestrian_stop_time << std::endl;
    debug->is_stop_reason_by_prdestrian = true;
    return true;
  } else {
    std::cout << "Current pedestrian stop lasting time(s) is: " << wait_time_diff_ 
      << ", smaller than threshold: " 
      << lon_based_pidcontroller_conf_.pedestrian_stop_time << std::endl;
    debug->is_stop_reason_by_prdestrian = false;
    return false;
  }
}

// 长期完全停车，用于EPB控制
bool LonControllerTest::IsFullStopLongTerm(common_msg::SimpleLongitudinalDebug *debug){
  if (debug->is_full_stop) {
    // 初次完全停车判断
    if (debug->is_full_stop && !is_full_stop_previous_) {
      is_full_stop_start_time_ = GetSystemTimeSeconds();
      std::cout << "Full stop long term start time(s) is " << is_full_stop_start_time_ 
        << std::endl;
    } else {
      std::cout << "Last time stop is already full stop, skip start_time init.\n";
    }
    double is_full_stop_end_time = GetSystemTimeSeconds();
    is_full_stop_wait_time_diff_ = is_full_stop_end_time - is_full_stop_start_time_;
  } else {
    is_full_stop_start_time_ = 0.0;
    is_full_stop_wait_time_diff_ = 0.0;
  }
  is_full_stop_previous_ = debug->is_full_stop;  // 更新停车状态
  if (is_full_stop_wait_time_diff_ > 
      lon_based_pidcontroller_conf_.full_stop_long_time) {
        std::cout << "Current full stop lasting time(s) is: " 
          << is_full_stop_wait_time_diff_ 
          << ", larger than threshold: " 
          << lon_based_pidcontroller_conf_.full_stop_long_time << std::endl;
      return true;
      } else {
        std::cout << "Current full stop lasting time(s) is: " 
          << is_full_stop_wait_time_diff_ 
          << ", smaller than threshold: " 
          << lon_based_pidcontroller_conf_.full_stop_long_time << std::endl;
      return false;
    }
}

/**
 * @brief EPB（电子驻车制动）控制，状态翻转策略
 * @param[in]  conf 纵向PID控制器参数
 * @param[out] control_command 控车命令
*/
void LonControllerTest::SetParkingBrake(const LonBasedPidControllerConf *conf,
                    common_msg::ControlCommand *control_command) {
  // epb开启切换逻辑，一阶段关闭，二阶段开启
  if (control_command->parking_brake) {
    // epb on, parking brake: 0 -> 1
    if (epb_on_change_switch_) {
      std::cout << "Epb on, first set parking brake to 0\n";
      control_command->parking_brake = false;
      ++epb_change_count_;
     // 持续发送关闭信息
     if (epb_change_count_ >= conf->epb_change_count) {
      epb_on_change_switch_ = false;
      epb_change_count_ = 0;
      std::cout << "Epb on, first stage has been done.\n";
     }
    } else{
      std::cout << "Epb on, second set parking brake to 1\n";
      control_command->parking_brake = true;
      ++epb_change_count_;
      // 切换
      if (epb_change_count_ >= conf->epb_change_count) {
        epb_on_change_switch_ = true;
        epb_change_count_ = 0;
        std::cout << "Epb on, second stage has been done.\n";
      } 
    }
  } else {
    // epb off, parking brake: 1 -> 0
    if (epb_off_change_switch_) {
      std::cout << "Epb off, first set parking brake to 1\n";
      control_command->parking_brake = true;
      ++epb_change_count_;
      // 持续发送开启信息
      if (epb_change_count_ >= conf->epb_change_count) {
        epb_off_change_switch_ = false;
        epb_change_count_ = 0;
        std::cout << "Epb off, first stage has been done.\n";
      }
    } else {
       std::cout << "Epb off, second set parking brake to 0\n";
       control_command->parking_brake = false;
       ++epb_change_count_;
       // 持续发送关闭信息
       if (epb_change_count_ >= conf->epb_change_count) {
        epb_off_change_switch_ = true;
        epb_change_count_ = 0;
        std::cout << "Epb off, second stage has been done.\n";
       }
    }
  }
}

void LonControllerTest::ProcessLogs(
  const common_msg::SimpleLongitudinalDebug *debug) {
    const std::string log_line = std::to_string(debug->station_reference) +
      "," + std::to_string(debug->station_error) +
      "," + std::to_string(debug->station_error_limited) +
      "," + std::to_string(debug->preview_station_error) +
      "," + std::to_string(debug->speed_reference) +
      "," + std::to_string(debug->speed_error) + 
      "," + std::to_string(debug->speed_controller_input_limited) + 
      "," + std::to_string(debug->preview_speed_reference) + 
      "," + std::to_string(debug->preview_speed_error) + 
      "," + std::to_string(debug->preview_acceleration_reference) + 
      "," + std::to_string(debug->acceleration_cmd_closeloop) + 
      "," + std::to_string(debug->acceleration_cmd) + 
      "," + std::to_string(debug->acceleration_lookup) + 
      "," + std::to_string(debug->acceleration_lookup_limit) + 
      "," + std::to_string(debug->speed_lookup) + 
      "," + std::to_string(debug->calibration_value) + 
      "," + std::to_string(debug->throttle_cmd) + 
      "," + std::to_string(debug->brake_cmd) + 
      "," + std::to_string(debug->is_full_stop);
  if (FLAGS_enable_csv_debug) {
    speed_log_file_ << log_line << std::endl;
  }
  std::cout << "Speed Control Detial: " << log_line << std::endl;
}

/**
 * @brief 计算纵向控制指令
 * @param[in] localization 定位消息
 * @param[in] chassis 线控消息
 * @param[in] planning_published_trajectory  规划消息
 * @param[out] cmd 控制指令
 * @return    void
 */
bool LonControllerTest::ComputeControlCommand(const common_msg::LocalizationEstimate *localization,
                            const common_msg::Chassis *chassis,
                            const common_msg::ADCTrajectory *planning_published_trajectory,
                            common_msg::ControlCommand *cmd) {
  // 定位、底盘数据此处未直接调用，通过injector
  localization_ = localization;
  chassis_ = chassis;
  trajectory_message_ = planning_published_trajectory;
  // 标定结果检查
  if (!control_interpolation_) {
    std::cerr << "Failed to initialize calibration table.\n";
    return false;
  }
  // 轨迹分析器初始化
  if (trajectory_analyzer_ == nullptr || 
    trajectory_analyzer_->seq_num() != 
    trajectory_message_->header.sequence_num) {
      // 初始化规划消息
      trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }
  // 初始化调试器
  common_msg::SimpleLongitudinalDebug *debug = &cmd->debug.simple_lon_debug;
  // debug->clear();
  double brake_cmd = 0.0;    // 刹车信号[0 ~ 1]，百分比
  double throttle_cmd = 0.0; // 油门信号[0 ~ 1]，百分比
  double ts = lon_based_pidcontroller_conf_.ts;  // 控制周期
  double preview_time =   // 预瞄时间
    lon_based_pidcontroller_conf_.preview_window * ts;
  bool enable_leadlag =   // 启用前馈 leadlag 补偿（倒车）
    lon_based_pidcontroller_conf_.enable_reverse_leadlag_compensation;
  if (preview_time < 0.0) {
    std::cerr << "Preview time set as: " << preview_time << " less than 0.\n";
    return false;
  }
  // 纵向误差（位置、速度误差）计算
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts, debug);
  // 位置误差限制
  double station_error_limite = 
    lon_based_pidcontroller_conf_.station_error_limit;
  double station_error_limited = 0.0;
  if (lon_based_pidcontroller_conf_.enable_speed_station_preview) {
      station_error_limited = 
    common::Clamp(debug->preview_station_error,
      -station_error_limite, station_error_limite);
  } else {
    station_error_limited = 
    common::Clamp(debug->station_error,
      -station_error_limite, station_error_limite);
  }
  // 根据车辆当前的档位、速度、状态实现动态调整PID控制器参数
  if (trajectory_message_->gear == common_msg::Chassis::GEAR_REVERSE) {
    if (CheckPit::CheckInPit(debug, &lon_based_pidcontroller_conf_, 
      injector_->vehicle_state()->linear_velocity(), 
      trajectory_message_->is_replan)) {
        std::cerr << "in pit\n";  // 洼地：专用PID参数组合
        station_pid_controller_.SetPID(
          lon_based_pidcontroller_conf_.pit_station_pid_conf);
        speed_pid_controller_.SetPID(
          lon_based_pidcontroller_conf_.pit_speed_pid_conf);
      } else {  // 非洼地：倒车专用PID参数组合
        station_pid_controller_.SetPID(
          lon_based_pidcontroller_conf_.reverse_station_pid_conf);  
        speed_pid_controller_.SetPID(
          lon_based_pidcontroller_conf_.reverse_speed_pid_conf);
      }
      // 补偿控制器
      if (enable_leadlag) {
        station_leadlag_controller_.SetLeadlag(
          lon_based_pidcontroller_conf_.reverse_station_leadlag_conf);
        speed_leadlag_controller_.SetLeadlag(
          lon_based_pidcontroller_conf_.reverse_speed_leadlag_conf);
      }
      // 低速前进,switch_speed-低速阈值
  } else if (injector_->vehicle_state()->linear_velocity() <=
    lon_based_pidcontroller_conf_.switch_speed) {
      if (CheckPit::CheckInPit(debug, &lon_based_pidcontroller_conf_, 
        injector_->vehicle_state()->linear_velocity(),
        trajectory_message_->is_replan)) {
          std::cerr << "in pit\n";  // 洼地：专用PID参数组合
          station_pid_controller_.SetPID(
            lon_based_pidcontroller_conf_.pit_station_pid_conf);
          speed_pid_controller_.SetPID(
            lon_based_pidcontroller_conf_.pit_speed_pid_conf);
        } else { // 非洼地：正常位置PID+低速PID参数组合 
          station_pid_controller_.SetPID(
            lon_based_pidcontroller_conf_.station_pid_conf);
          speed_pid_controller_.SetPID(
            lon_based_pidcontroller_conf_.low_speed_pid_conf);
        }
  } else {  // 前进档高速行驶模式，正常位置PID+高速PID参数组合
    station_pid_controller_.SetPID(
      lon_based_pidcontroller_conf_.station_pid_conf);
    speed_pid_controller_.SetPID(
      lon_based_pidcontroller_conf_.high_speed_pid_conf);
  }
  // 外环控制：位置PID输出速度调节量，s = v*t
  double speed_offset =
    station_pid_controller_.Control(station_error_limited, ts);
  // 补偿控制输出
  if(enable_leadlag) {
    speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
  }
  // 内环控制：速度PID控制输出加速度调节量,v = (1*a*t^2)/2
  double speed_controller_input = 0.0;
  double speed_controller_input_limite = 
    lon_based_pidcontroller_conf_.speed_controller_input_limit;
  double speed_controller_input_limited = 0.0;
  // 预瞄控制
  if (lon_based_pidcontroller_conf_.enable_speed_station_preview) {
    speed_controller_input = speed_offset + debug->preview_speed_error;
  } else { 
    speed_controller_input = speed_offset + debug->speed_error;
  }
  speed_controller_input_limited = 
    common::Clamp(speed_controller_input, 
      -speed_controller_input_limite,
      speed_controller_input_limite);
  double acceleration_cmd_closeloop = 0.0;  // 速度调节量
  acceleration_cmd_closeloop = speed_pid_controller_.Control(
    speed_controller_input_limited, ts);
  // 监控积分器状态
  debug->pid_saturation_status = 
    speed_pid_controller_.IntegratorSaturationStatus();
  // 补偿控制
  if (enable_leadlag) {
    acceleration_cmd_closeloop = 
      speed_leadlag_controller_.Control(acceleration_cmd_closeloop, ts);
    debug->leadlag_saturation_status = // 积分器状态
      speed_leadlag_controller_.InnerstateSaturationStatus();
  }
  /**
   * 车辆位于空档时，重置位置、速度PID控制器积分量，                 \
     避免空档无意义积分导致积分过饱和，降低系统响应，可能导致控制混乱  \
   */
  if (chassis_->gear_location == common_msg::Chassis::GEAR_DRIVE) {
    speed_pid_controller_.Reset_intergal();
    station_pid_controller_.Reset_intergal();
  }
  // 坡度加速度补偿
  // 车辆俯仰角输入低通滤波器进行滤波处理，消除车辆震动噪声
  double vehicle_pitch_rad = 
    digital_filter_pitch_angle_.Filter(injector_->vehicle_state()->pitch());
  // 弧度转角度值，添加全局偏移量（补偿传感器安装角度误差）
  double vehicle_pitch = 
    vehicle_pitch_rad  
    * 180 / M_PI 
    + FLAGS_pitch_offset_deg;
  std::cout << "[LonControllerTest]:" << vehicle_pitch << std::endl;
  debug->vehicle_pitch = vehicle_pitch;
  // 坡度补偿量 = 重力加速度 * sin(道路坡度角+传感器误差角度)，传感器可能未完全水平安装
  double slope_offset_compensation = 
    lon_based_pidcontroller_conf_.use_opposite_slope_compensation
    * GRA_ACC 
    * sin(FLAGS_pitch_offset_deg * M_PI / 180 + vehicle_pitch_rad);
  if (std::isnan(slope_offset_compensation)) {
    slope_offset_compensation = 0.0;
  }
  debug->slope_offset_compensation = slope_offset_compensation;
  /**
   * 加速度调节量 = 速度PID控制器输出量（/前馈补偿控制器输出量）+          \
                   预瞄点参考加速度（基础加速度，提高系统响应与控制精度）+ \
                   坡度补偿                                            \
  */
  double acceleration_cmd = 
    acceleration_cmd_closeloop 
    + debug->preview_acceleration_reference 
    + lon_based_pidcontroller_conf_.enable_slope_offset
    * debug->slope_offset_compensation;
  /**
   * 转向控制检查机制，当车辆方向盘转角较大时暂停纵向控制等待  \
     方向盘转角差值（变化量）                               \
   */
  double current_steer_interval = 
    cmd->steering_target - chassis_->steering_percentage;
  // 是否启用转向检查功能 + 轨迹类型未知 + 方向盘变化量大于最大阈值
  if (lon_based_pidcontroller_conf_.use_steering_check &&
      (trajectory_message_->trajectory_type 
        == common_msg::ADCTrajectory::UNKNOWN) && 
      std::abs(current_steer_interval) > 
        lon_based_pidcontroller_conf_.steer_cmd_interval) {
      std::cout << "Steer target is " << cmd->steering_target 
        << ", steering_percentage is " << chassis_->steering_percentage
        << ", Steering cmd interval is larger than " 
        << lon_based_pidcontroller_conf_.steer_cmd_interval
        << std::endl;
      // 重置位置、速度PID控制器积分量，避免积分饱和、加速度输出为0，等待转向
      speed_pid_controller_.Reset_intergal();
      station_pid_controller_.Reset_intergal();
      acceleration_cmd = 0.0;
      debug->is_wait_steer = true;
  } else { 
    debug->is_wait_steer = false;
  }
  // 方向盘转角
  debug->current_steer_interval = current_steer_interval;
  /**
   * 多重停车判断机制，根据预瞄点状态检查、路径剩余距离、停车原因等，切换停车状态：
     完全停车（AEB控制） ，软停车（行人相关因素长时间停车） 
  */
  debug->is_full_stop = false;
  debug->is_full_stop_soft = false;
  auto previous_full_stop = 
    injector_->Get_pervious_lon_debug_info()->is_full_stop;
  GetPathRemain(debug);             // 停车点计算
  IsStopByDestination(debug);       // 终点停车判断
  IsPedestrianStopLongTerm(debug);  // 软停车判断
  /**
   * 预瞄点参考检查 +                             \
     预瞄点参考加速度abs小于停车时最大加速度阈值 +  \
     预瞄点参考速度abs小于停车时最大速度阈值 +      \
     轨迹类型不是开放空间（eg:泊车场景）           \
  */
 if (lon_based_pidcontroller_conf_.use_preview_reference_check &&
     (std::fabs(debug->preview_acceleration_reference <= 
      FLAGS_max_acceleration_when_stopped)) && 
      std::fabs(debug->preview_speed_reference) <= 
      vehicle_param_.vehicle_param.max_abs_speed_when_stopped && 
      trajectory_message_->trajectory_type != 
      common_msg::ADCTrajectory::OPEN_SPACE) {
    // 终点、行人相关停车
    if (debug->is_stop_reason_by_destination || 
      debug->is_stop_reason_by_prdestrian) {
      debug->is_full_stop = true;
      std::cout << "Into full stop within preview acc and reference speed," <<
        " is_full_stop is " << debug->is_full_stop << std::endl;
    } else {  // 软停车
      debug->is_full_stop_soft = true;
      std::cout << "Into soft stop within preview acc and reference speed," <<
        " is_full_stop_soft is " << debug->is_full_stop_soft << std::endl;
    }
    // 上一控制周期不处于停车状态，路径剩余距离阈值使用初始默认值
    if (!previous_full_stop) {
      max_path_remain_when_stopped_ = 
        FLAGS_max_path_remain_when_stopped;
    } else {
      max_path_remain_when_stopped_ = 
        FLAGS_max_path_remain_when_stopped + 
        lon_based_pidcontroller_conf_.full_stop_path_remain_gain;
    }
  }
  /*
   * path remain停止判断：
     前进档 + （车辆剩余距离 < 路径剩余距离阈值）
     倒车档 + （车辆剩余距离 > 路径剩余距离阈值），倒车时基于距离为负值，s为矢量
   */
  if ((trajectory_message_->gear == common_msg::Chassis::GEAR_DRIVE &&
      debug->path_remain < max_path_remain_when_stopped_) || 
      (trajectory_message_->gear == common_msg::Chassis::GEAR_REVERSE &&
      debug->path_remain > -max_path_remain_when_stopped_)) {
    std::cout << "Into full stop decision by path remain.\n";
    // 终点或行人停车，设置完全停车标记
    if (debug->is_stop_reason_by_destination ||
        debug->is_stop_reason_by_prdestrian) {
      debug->is_full_stop = true;   
      std::cout << "Current path remain distance: " << debug->path_remain 
        << ", is within max_path_remain threshold: " << max_path_remain_when_stopped_
        << ", into full stop because vehicle is in destination: " 
        << debug->is_stop_reason_by_destination
        << ", or pedestrian is in long time stop: " 
        << debug->is_stop_reason_by_prdestrian 
        << "is_full_stop flag:" << debug->is_full_stop << "\n";
    } else {  // 软停车
      debug->is_full_stop_soft = true;
      std::cout << "Current path remain distance: " << debug->path_remain 
        << ", is within max_path_remain threshold: " 
        << max_path_remain_when_stopped_
        << ", but not into full stop because stop not in destination: " 
        << debug->is_stop_reason_by_destination
        << ", or pedestrian is not long time stop: " 
        << debug->is_stop_reason_by_prdestrian
        << ", is_full_stop_soft flag: "
        << debug->is_full_stop_soft; 
    }
    // 特殊处理：车辆速度过低 + 停车原因为行人
    if (injector_->vehicle_state()->linear_velocity() < 
        vehicle_param_.vehicle_param.max_abs_speed_when_stopped &&
        debug->is_stop_reason_by_prdestrian) {
      std::cout << "Current stop is for long time pedestrian stop, "
        << debug->is_stop_reason_by_destination;
      debug->is_full_stop = true; 
    }
  } else {
    std::cerr << "Not into full stop decision by path remain.\n";
  }
  /**
   * 完全停车：加速度调节量
   * 倒车档停车（一般为泊车）：加速度为负，取控制器计算出的加速度值与完全停车配置的加速度值中的最大值  \
     正常停车：加速度为正，取控制器计算出的加速度值与完全停车配置的加速度值中的最小值                \
   */
  if (debug->is_full_stop) {
    acceleration_cmd = 
      (chassis_->gear_location == common_msg::Chassis::GEAR_REVERSE)
      ? std::max(acceleration_cmd, 
        -lon_based_pidcontroller_conf_.standstill_acceleration)
      : std::min(acceleration_cmd,
        lon_based_pidcontroller_conf_.standstill_acceleration);
    // 重置积分器，低速稳态控制
    speed_pid_controller_.Reset_intergal();
    station_pid_controller_.Reset_intergal();
  }
  // 软停车：对应不同的加速度执行逻辑
  if (debug->is_full_stop_soft) {
    /**
     * 前进档处理逻辑： 
       1. acceleration_cmd >= 0 车辆处于加速状态则强制执行正常停车加速度调节量；                \
       2. acceleration_cmd < 0 && 剩余路径>=0 车辆处于减速状态且剩余路径非负则保持加速度调节量； \
       3. 剩余路径为负（已过停车点）且轨迹类型不是normal，增加急刹车加速度补偿；                  \
       4. 剩余路径为负（已过停车点）且轨迹类型是normal，则执行正常停车加速度调节量。              \ 
    */ 
    if (chassis_->gear_location != common_msg::Chassis::GEAR_REVERSE) {
      acceleration_cmd = 
        (acceleration_cmd > 0.0) ? standstill_narmal_acceleration_
        : (debug->path_remain < 0.0) ? acceleration_cmd
        : (trajectory_message_->trajectory_type != common_msg::ADCTrajectory::NORMAL)
        ? (acceleration_cmd + stop_gain_acceleration_)  // 加速度增益（急刹车）
        : (acceleration_cmd + standstill_narmal_acceleration_); // 正常停车加速度补偿
    } else {
      /**
       * 倒车档处理逻辑：
         1. acceleration_cmd <= 0 车辆倒车加速度为负则处于倒车加速状态，使用负的正常停车加速度（制动）；        \
         2. acceleration_cmd > 0 && 剩余路径<=0 倒车车辆处于减速状态且剩余路径为负则保持加速度调节量；         \
         3. acceleration_cmd > 0 && 剩余路径>0 && 轨迹类型不是normal，车辆倒车越过终点，增加急刹车加速度补偿； \
         4. acceleration_cmd > 0 && 剩余路径>0 && 轨迹类型是normal，车辆倒车越过终点，增加正常停车加速度补偿。 \ 
       */
      acceleration_cmd = 
        (acceleration_cmd <= 0.0) ? -standstill_narmal_acceleration_
        : (debug->path_remain <= 0.0) ? acceleration_cmd
        : (trajectory_message_->trajectory_type != common_msg::ADCTrajectory::NORMAL)
        ? (acceleration_cmd - stop_gain_acceleration_)  // 加速度增益（急刹车）
        : (acceleration_cmd - standstill_narmal_acceleration_); // 正常停车加速度补偿  
    }
    // 重置积分器，低速稳态控制
    speed_pid_controller_.Reset_intergal();
    station_pid_controller_.Reset_intergal();
  }
  // ------ 计算纵向控制命令command -------
  // 油门下限值
  double throttle_lowerbound = 
    std::max(vehicle_param_.vehicle_param.throttle_deadzone,  // 无效值（车辆性质决定）
             lon_based_pidcontroller_conf_.throttle_minimum_action);  // 控制配置的最小值
  // 刹车下限值
  double brake_lowerbound = 
    std::max(vehicle_param_.vehicle_param.brake_deadzone,  // 无效值（车辆性质决定）
             lon_based_pidcontroller_conf_.brake_minimum_action);  // 配置的最小值
  double calibration_value = 0.0;  // 标定值
  double acceleration_lookup = 
    (chassis_->gear_location == common_msg::Chassis::GEAR_REVERSE)
    ? -acceleration_cmd : acceleration_cmd;
  // 最大加速度限制(最大加速度+坡度补偿)
  double acceleration_lookup_limited = 
    vehicle_param_.vehicle_param.max_acceleration 
    + lon_based_pidcontroller_conf_.enable_slope_offset 
    * debug->slope_offset_compensation;
  double acceleration_lookup_limit = 0.0;
  // 使用加速度查找限制，当加速度查找值超过限制时，使用最大限制值，否则使用查找值
  if (lon_based_pidcontroller_conf_.use_acceleration_lookup_limit) {
    acceleration_lookup_limit = 
      (acceleration_lookup > acceleration_lookup_limited)
      ? acceleration_lookup_limited : acceleration_lookup;  
  }
  // 预瞄速度查找标定表并进行插值计算控制结果
  // 二维插值查找标定表中对应的控制指令值（油门/刹车值），
  // 标定表的输入：（速度、加速度）， 输出：油门/刹车值（百分比）
  if (FLAGS_use_preview_speed_for_table) {
    // 加速度限制
    if (lon_based_pidcontroller_conf_.use_acceleration_lookup_limit) {
      calibration_value = 
        control_interpolation_->Interpolate(std::make_pair(
          std::fabs(debug->preview_speed_reference), 
          acceleration_lookup_limit));
    } else {
      calibration_value = 
        control_interpolation_->Interpolate(std::make_pair(
          std::fabs(debug->preview_speed_reference), 
          acceleration_lookup));
    }
  } else {  // 当前车速查找
      if (lon_based_pidcontroller_conf_.use_acceleration_lookup_limit) {
        calibration_value = 
          control_interpolation_->Interpolate(std::make_pair(
            std::fabs(injector_->vehicle_state()->linear_velocity()), 
            acceleration_lookup_limit));
      } else {
        calibration_value = 
          control_interpolation_->Interpolate(std::make_pair(
            std::fabs(injector_->vehicle_state()->linear_velocity()), 
            acceleration_lookup));
      }
  }
  // 加速控制
  if (acceleration_lookup >= 0) {
    if (calibration_value >= 0) {
      throttle_cmd = std::max(calibration_value, throttle_lowerbound);
    } else {
      throttle_cmd = throttle_lowerbound;
    }
    brake_cmd = 0;  // 刹车值为0
  } else { // 制动减速
    throttle_cmd = 0;  // 油门值为0
    /**
     * 插值大于0（该工作点无合适刹车值，直接使用刹车下限值）,    \
       使用刹车下限值（标定表中command值，正值为油门值，负值为刹车值（但使用时为绝对值调用））
     */
    if (calibration_value >= 0) {
      brake_cmd = brake_lowerbound;
    } else {
      brake_cmd = std::max(-calibration_value, brake_lowerbound);
    }
  }
  // EPB控制
  if(FLAGS_use_vehicle_epb) {
    std::cout << "Into use vehicle epb.\n";
    // 车辆加速，epb - off
    if (acceleration_lookup >= 0) {
      if (debug->slope_offset_compensation > 0) {  // 坡度补偿
        if (acceleration_lookup > debug->slope_offset_compensation) {
          parking_release_ = true;
        } 
      } else {  // 正常加速
          parking_release_ = true;
      }
      if (chassis_->parking_brake && parking_release_) {
        std::cout << "Into park brake release.\n";
        cmd->parking_brake = false;
        SetParkingBrake(&lon_based_pidcontroller_conf_, cmd);
      }
    } else {  // 停车 epb - on
      cmd->parking_brake = false;
      if (debug->is_full_stop && IsFullStopLongTerm(debug)) {
        std::cout << "Into park brake trigger.\n";
        cmd->parking_brake = true;
        if (chassis_->parking_brake) {
          brake_cmd = 0.0;
        }
      }
    }
  }
  // debug record
  debug->station_error_limited = 
    station_error_limited;                    // 位置误差限值
  debug->speed_offset = speed_offset;         // 位置调节量
  debug->speed_controller_input_limited = 
    speed_controller_input_limited;           // 速度PID输入限值
  debug->acceleration_cmd = acceleration_cmd; // 加速度
  debug->throttle_cmd = throttle_cmd;         // 油门值
  debug->brake_cmd = brake_cmd;               // 刹车值
  debug->acceleration_lookup =                // 加速度标定表值
    acceleration_lookup;
  debug->acceleration_lookup_limit =          // 加速度标定表限值
    acceleration_lookup_limit;
  debug->speed_lookup =                        // 当前车速
    injector_->vehicle_state()->linear_velocity();
  debug->calibration_value = calibration_value; // 油门/刹车标定表值
  debug->acceleration_cmd_closeloop = 
    acceleration_cmd_closeloop;                // 速度PID控制
  // 日志写入
  ProcessLogs(debug);
  /**
   * 支持多种控制接口，下发油门、刹车、目标加速度、目标速度、档位信号，
     适配不同车辆的控制接口需求，提高控制系统冗余
   */
  cmd->throttle = throttle_cmd;  // 油门指令[0~1]百分比
  cmd->brake = brake_cmd;        // 刹车指令[0~1]百分比
  // 加速度指令
  if (lon_based_pidcontroller_conf_.use_acceleration_lookup_limit) {
    cmd->acceleration = acceleration_lookup_limit;
  } else {
    cmd->acceleration = acceleration_cmd;
  }
  // 档位控制
  // 车辆速度小于停车速度阈值（接近停车状态）
  // 或当前车辆档位与规划档位一致,或当前车辆档位为空档，下发规划档位
  if ((std::fabs(injector_->vehicle_state()->linear_velocity()) <= 
  vehicle_param_.vehicle_param.max_abs_speed_when_stopped) || 
  chassis_->gear_location == trajectory_message_->gear ||
  chassis_->gear_location == common_msg::Chassis::GEAR_NEUTRAL) {
    cmd->gear_location = trajectory_message_->gear;
  } else { // 底盘档位
    cmd->gear_location = chassis_->gear_location;
  }
  // 速度控制指令（ACC场景等）
  if (lon_based_pidcontroller_conf_.use_speed_itfc) {
    // 速度 = 参考点速度 + PID速度
    reference_spd_cmd = reference_spd_ + debug->speed_offset;
    // 参考速度小于完全停车速度阈值（车辆将停车）且车辆档位为前进档
    if ((reference_spd_ <= 
      lon_based_pidcontroller_conf_.speed_itfc_full_stop_speed) && 
      (chassis_->gear_location == common_msg::Chassis::GEAR_DRIVE)) {
        // 剩余路径距离在指定范围内且预瞄点加速度大于紧急减速加速度阈值（急刹车）
        if ((debug->path_remain >= 
          lon_based_pidcontroller_conf_.speed_itfc_path_remain_min) &&
         (debug->path_remain <= 
          lon_based_pidcontroller_conf_.speed_itfc_path_remain_max) &&
         (debug->preview_acceleration_reference >= 
          lon_based_pidcontroller_conf_.speed_itfc_dcc_emergency)) {
            // 预瞄加速度小于ACC模式加速度阈值
            if (debug->preview_acceleration_reference <= 
            lon_based_pidcontroller_conf_.speed_itfc_acc_thres) {
              // 下发速度使用配置的特殊速度（eg:泊车场景）
              reference_spd_cmd = 
                lon_based_pidcontroller_conf_.speed_itfc_speed_cmd;
            }
          }
      }
      // 正常情况控制速度
      cmd->speed = reference_spd_cmd;
  }
  return true;
}

}  // namespace control