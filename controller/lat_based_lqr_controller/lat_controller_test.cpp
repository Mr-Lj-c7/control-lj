#include "lat_controller_test.h"


namespace control{
using Matrix = Eigen::MatrixXd;
constexpr double kDoubleEpsilon = 1e-6;
namespace {
    std::string GetLogFileName() {
        time_t raw_time;
        char name_buffer[100];
        std::time(&raw_time);  // 系统时间
        std::tm time_tm;       // 时间结构体
        #ifdef _WIN32
            localtime_s(&time_tm, &raw_time);
        #else
            localtime_r(&raw_time, &time_tm);
        #endif
        // 格式化时间
        strftime(name_buffer, 100, 
          "../../controller/lat_based_lqr_controller/csv/steer_log_simple_optimmal_%F_%H%M%S.csv", 
          &time_tm);
        return std::string(name_buffer);
    }
    
    void WriteHeaders(std::ofstream &file_stream) {
        // 表头
        file_stream << "current_lateral_error," << "current_ref_heading,"
                    << "current_heading," << "current_heading_error,"
                    << "heading_error_rate," << "lateral_error_rate,"
                    << "current_curvature," << "steer_angle,"
                    << "steer_angle_feedforward,"
                    << "steer_angle_lateral_contribution,"
                    << "steer_angle_lateral_rate_contribution,"
                    << "steer_angle_heading_contribution,"
                    << "steer_angle_heading_rate_contribution,"
                    << "steer_angle_feedback," 
                    << "steering_position," << "v"
                    << std::endl;
    }
} // namespace

    LatControllerTest::LatControllerTest() : name_("LQR-based lateral Controller test") {
        if (FLAGS_enable_csv_debug) {
            steer_log_file_.open(GetLogFileName());
            // 6位小数精度
            steer_log_file_ << std::fixed;
            steer_log_file_ << std::setprecision(6);
            WriteHeaders(steer_log_file_);
        }
        std::cerr << "Using: " << name_ << std::endl;
    }
    LatControllerTest::~LatControllerTest(){ CloseLogFile(); }

    double LatControllerTest::GetSystemTimeSeconds() const
    {
        return std::chrono::duration<double>
               (std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void LatControllerTest::CloseLogFile( ) {
        if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
            steer_log_file_.close();
        }
    }
    
    void LatControllerTest::LogInitParameters( ) {
        std::cout << name_ << "begin." << std::endl;
        std::cout << "[LatController parameters]" 
                    << " mass_: " << mass_<< "," 
                    << " iz_: " << iz_ << "," 
                    << " lf_: " << lf_<< "," 
                    << " lr_: " << lr_
                    << std::endl;
    }

    std::string LatControllerTest::Name() const { return name_; }

    void LatControllerTest::Stop() { CloseLogFile();}

    bool LatControllerTest::Init(std::shared_ptr<DependencyInjector> injector) {
        if (!LoadLatBasedLqrControllerConf(lat_based_lqr_controller_conf_path)) {
            std::cerr << "[LatControllerTest]: Failed to load lat controller conf." 
                    << std::endl; 
            return false;   
        }
        injector_ = injector;
        if (!LoadControlConf()) {
            std::cerr << "[LatControllerTest]: Failed to load control conf." << std::endl; 
            return false;   
        }
        // 初始化矩阵-系统状态矩阵A
        const int matrix_size = basic_state_size_ + preview_window_;  // 预瞄控制（横向控制一般不采用预瞄控制）
        matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
        matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
        matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
        // 系统状态矩阵A中的常系数
        matrix_a_(0, 1) = 1.0;
        matrix_a_(1, 2) = (cf_ + cr_) / mass_;
        matrix_a_(2, 3) = 1.0;
        matrix_a_(3, 2) = (cf_ * lf_ - cr_ * lr_) / iz_;
        // 系统状态矩阵A中的非常系数存储到matrix_a_coeff矩阵
        matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
        matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
        matrix_a_coeff_(1, 3) = (cr_ * lr_ - cf_ * lf_) / mass_;
        matrix_a_coeff_(3, 1) = (cr_ * lr_ - cf_ * lf_) / iz_;
        matrix_a_coeff_(3, 3) = -1.0 * (cf_ * lf_ * lf_ + cr_ * lr_ * lr_) / iz_;
        // 初始化-系统控制矩阵B
        matrix_b_ = Matrix::Zero(basic_state_size_, 1);
        matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
        matrix_bdc_ = Matrix::Zero(matrix_size, 1);
        // 控制矩阵B中的常系数
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = cf_ * lf_ / iz_;
        // B_d = B * Ts
        matrix_bd_ = matrix_b_ * ts_;
        // 状态矩阵 X(t) = [e1, e1', e2, e2']
        matrix_state_ = Matrix::Zero(matrix_size, 1);
        /**
        * 状态反馈矩阵K=[k1 k2 k3 k4]分别对应[e1 e1' e2 e2']^T的各项误差的状态反馈系数
        每时刻都会通过LQR求解到一个最优的K矩阵，然后用K矩阵来计算控制量，初始化为1x4零矩阵
        */
        matrix_k_ = Matrix::Zero(1, matrix_size);
        /**
        * 初始化R矩阵为1*1的单位阵，R矩阵就是LQR中目标函数中控制量平方和的权重系数
        横向控制只有一个控制量就是前轮转角
        */
        matrix_r_ = Matrix::Identity(1, 1);
        /**
        * 初始化Q矩阵为4*4的0矩阵，Q矩阵是LQR中目标函数中各个状态量(X=[e1 e1' e2 e2'])平方和的权重系数
        Q是一个对角阵，对角线上4个元素分别存放e1 e1' e2 e2'平方和在LQR目标函数中的权重系数，这里只是初始化一下
        */
        matrix_q_ = Matrix::Zero(matrix_size, matrix_size);
        int q_param_size = 
            static_cast<int>(lat_based_lqr_controller_conf_.matrix_q.size());
        int reverse_q_param_size = 
            static_cast<int>(lat_based_lqr_controller_conf_.reverse_matrix_q.size());
        // Q,R矩阵检查
        if (matrix_size != q_param_size || matrix_size != reverse_q_param_size) {
            std::stringstream ss;
            ss << "[LatControllerTest]: " << "lateral controller error: matrix_q size: " 
               << q_param_size << ", lateral controller error: reverse_matrix_q size: " 
               << reverse_q_param_size << ", in parameter file not equal to matrix_size: "  
               << matrix_size;
            const std::string error_msg = ss.str();
            std::cerr << error_msg << std::endl;
            return false;
        }
    /**
    * 加载控制配置中matrix_q(0),matrix_q(1),matrix_q(2),matrix_q(3)。默认分别为0.05，0.0，1.0，0.0
      可以看出实际上只考虑了横向误差和航向误差且航向误差的比重要比横向误差大很多，误差变化率Q阵中系数为0
      加载到LatController类数据成员matrix_q_即LQR的Q矩阵中
    */
    for (size_t i = 0; i < q_param_size; ++i) {
        matrix_q_(i, i) = lat_based_lqr_controller_conf_.matrix_q[i];
    }
    matrix_q_updated_ = matrix_q_;
    /**
    * 初始化3个滤波器，1个低通滤波是对计算出方向盘转角控制指令进行滤波
        另外两个滤波器是横向误差，航向误差的均值滤波器
    */ 
    InitialzeFilters();
    /**
    * LoadLatGainScheduler加载增益调度表，就是横向误差和航向误差在车速不同时乘上个不同的比例
        这个比例决定了实际时的控制效果，根据实际经验低速和高速应该采取不同的比例，低速比例较大，若高速
        采用同样比例极有可能导致画龙现象
    */ 
    LoadLatGainScheduler();
    // 显示一些车辆参数
    LogInitParameters();
    enable_leadlag_ = 
        lat_based_lqr_controller_conf_.enable_reverse_leadlag_compensation;
        if (enable_leadlag_) {
            leadlag_controller_.Init(lat_based_lqr_controller_conf_.reverse_leadlag_conf, ts_);
        }
        // Mrac
        enable_mrac_ = lat_based_lqr_controller_conf_.enable_steer_mrac_control;
        if (enable_mrac_) {
            std::cerr << "[LatControllerTest]: Mrac controller failed." << std::endl;
        }
        // 考虑前后距离反馈控制
        enable_look_ahead_back_control_ = 
            lat_based_lqr_controller_conf_.enable_look_ahead_back_control;  
        return true;
    }

    void LatControllerTest::InitialzeFilters() { 
        std::vector<double> den(3, 0.0);
        std::vector<double> num(3, 0.0);
        // IIR二阶低通滤波器系数
        common::LpCoefficients(ts_, 
            lat_based_lqr_controller_conf_.cutoff_freq, &den, &num);
        // 方向盘控制指令滤波平滑
        digital_filter_.SetCoefficients(den, num);
        // 横向误差平滑
        lateral_error_filter_ = common::MeanFilter(
            static_cast<std::uint_fast8_t>(
                lat_based_lqr_controller_conf_.mean_filter_window_size));
        // 航向误差平滑
        heading_error_filter_ = common::MeanFilter(
            static_cast<std::uint_fast8_t>(
                lat_based_lqr_controller_conf_.mean_filter_window_size));
    }
    
    void LatControllerTest::LoadLatGainScheduler() {
        // 加载增益调度表
        const auto &lat_err_gain_scheduler = 
            lat_based_lqr_controller_conf_.lat_error_gain_scheduler;
        const auto &heading_err_gain_scheduler = 
            lat_based_lqr_controller_conf_.heading_error_gain_scheduler;
        std::cerr << "[LatControllerTest]: Lateral control  gain scheduler load." 
                  << std::endl;
        common::Interpolation1D::DataType xy1, xy2;
        size_t i = 0;
        for (const auto& scheduler : lat_err_gain_scheduler.scheduler) {
            xy1.push_back(std::pair<double, double>(scheduler.speed, scheduler.ratio));
            // std::cerr << "[LatControllerTest]: Lateral control gain scheduler: " 
            //           << "speed: " << scheduler.speed << ", ratio: " << scheduler.ratio 
            //           << std::endl;
        }
        for (const auto& scheduler : heading_err_gain_scheduler.scheduler) {
            xy2.push_back(std::pair<double, double>(scheduler.speed, scheduler.ratio));
        }
        // 清空并初始化插值器
        lat_err_interpolation_.reset(new common::Interpolation1D);
        if (!lat_err_interpolation_->Init(xy1)) {
            std::cerr << "[LatControllerTest]: Failed to initialize lateral error gain scheduler." 
                      << std::endl;
        }
        // assert()只有在debug模式下有效
        // assert(lat_err_interpolation_->Init(xy1) && 
        //     "Failed to initialize lateral error gain scheduler.");
        heading_err_interpolation_.reset(new common::Interpolation1D);
        if (!heading_err_interpolation_->Init(xy2)) {
            std::cerr << "[LatControllerTest]: Failed to initialize heading error gain scheduler." 
                      << std::endl;
        }
        // assert(heading_err_interpolation_->Init(xy2) && 
        //     "Failed to initialize heading error gain scheduler.");
    }

    bool LatControllerTest::LoadLatBasedLqrControllerConf(
        const std::string &lat_based_lqr_controller_config_file) {
        // LatBaseLqrControllerConf lat_based_lqr_controller_conf_;
        std::ifstream file(lat_based_lqr_controller_config_file);
        if (!file.is_open()) {
            std::cerr << "[LatControllerTest]: Failed to load lat control parameter file: "
                      << lat_based_lqr_controller_config_file << std::endl;
            return false;
        }
        Json::Value root_;
        Json::Reader reader_;
        if (!reader_.parse(file, root_)) {
            std::cerr << "[LatControllerTest]: Failed to parse lat control parameter file: "
                      << lat_based_lqr_controller_config_file << std::endl;
            return false;
        }
        lat_based_lqr_controller_conf_.ts = root_["ts"].asDouble();
        lat_based_lqr_controller_conf_.preview_window = root_["preview_windows"].asInt();
        lat_based_lqr_controller_conf_.cf = root_["cf"].asDouble();
        lat_based_lqr_controller_conf_.cr = root_["cr"].asDouble();
        lat_based_lqr_controller_conf_.mass_fl = root_["mass_fl"].asInt();
        lat_based_lqr_controller_conf_.mass_fr = root_["mass_fr"].asInt();
        lat_based_lqr_controller_conf_.mass_rl = root_["mass_rl"].asInt();
        lat_based_lqr_controller_conf_.mass_rr = root_["mass_rr"].asInt();
        lat_based_lqr_controller_conf_.eps = root_["esp"].asDouble();
        if (root_.isMember("matrix_q")) {
            uint16_t matrix_size = root_["matrix_q"].size();
            for (size_t i = 0; i < matrix_size; ++i) {
                lat_based_lqr_controller_conf_.matrix_q.push_back(
                    root_["matrix_q"][static_cast<uint8_t>(i)].asDouble());
            }
        }
        // lat_based_lqr_controller_conf_.matrix_q_0 = root_["mtrix_q_0"].asDouble();
        // lat_based_lqr_controller_conf_.matrix_q_1 = root_["matrix_q_1"].asDouble();
        // lat_based_lqr_controller_conf_.matrix_q_2 = root_["matrix_q_2"].asDouble();
        // lat_based_lqr_controller_conf_.matrix_q_3 = root_["matrix_q_3"].asDouble();
        if (root_.isMember("reverse_matrix_q")) {
            uint16_t matrix_size = root_["reverse_matrix_q"].size();
            for (size_t i = 0; i < matrix_size; ++i) {
                lat_based_lqr_controller_conf_.reverse_matrix_q.push_back(
                    root_["reverse_matrix_q"][static_cast<uint8_t>(i)].asDouble());
            }
        }
        // lat_based_lqr_controller_conf_.reverse_matrix_q_0 = 
        //     root_["reverse_matrix_q_0"].asDouble();
        // lat_based_lqr_controller_conf_.reverse_matrix_q_1 = 
        //     root_["reverse_matrix_q_1"].asDouble();
        // lat_based_lqr_controller_conf_.reverse_matrix_q_2 = 
        //     root_["reverse_matrix_q_2"].asDouble();
        // lat_based_lqr_controller_conf_.reverse_matrix_q_3 = 
        //     root_["reverse_matrix_q_3"].asDouble();
        lat_based_lqr_controller_conf_.cutoff_freq = root_["cutoff_freq"].asInt();
        lat_based_lqr_controller_conf_.max_iteration = root_["max_iteration"].asInt();
        lat_based_lqr_controller_conf_.mean_filter_window_size = 
            root_["mean_filter_window_size"].asInt();
        lat_based_lqr_controller_conf_.max_lateral_acceleration = 
            root_["max_lateral_acceleration"].asDouble();
        lat_based_lqr_controller_conf_.enable_reverse_leadlag_compensation = 
            root_["enable_reverse_leadlag_compensation"].asBool();
        lat_based_lqr_controller_conf_.enable_steer_mrac_control = 
            root_["enable_steer_mrac_control"].asBool();
        lat_based_lqr_controller_conf_.enable_look_ahead_back_control = 
            root_["enable_look_ahead_back_control"].asBool();
        lat_based_lqr_controller_conf_.lookahead_station = 
            root_["lookahead_station"].asDouble();
        lat_based_lqr_controller_conf_.lookback_station = 
            root_["lookback_station"].asDouble();
        lat_based_lqr_controller_conf_.lookahead_station_high_speed = 
            root_["lookahead_station_high_speed"].asDouble();
        lat_based_lqr_controller_conf_.lookback_station_high_speed = 
            root_["lookback_sation_high_speed"].asDouble();
        lat_based_lqr_controller_conf_.switch_speed = 
            root_["switch_speed"].asDouble();
        lat_based_lqr_controller_conf_.switch_speed_window = 
            root_["switch_speed_window"].asDouble();
        lat_based_lqr_controller_conf_.query_relative_time = 
            root_["query_relative_time"].asDouble();
        lat_based_lqr_controller_conf_.lock_steer_speed = 
            root_["lock_steer_speed"].asDouble();
        lat_based_lqr_controller_conf_.reverse_feedback_ratio = 
            root_["reverse_feedback_ratio"].asDouble();
        lat_based_lqr_controller_conf_.minimum_speed_protection = 
            root_["minimum_speed_protection"].asDouble();
        lat_based_lqr_controller_conf_.enable_navigation_mode_position_update = 
            root_["enable_navigation_mode_position_update"].asBool();
        lat_based_lqr_controller_conf_.trajectory_transform_to_com_reverse = 
            root_["trajectory_transform_to_com_reverse"].asBool();
        lat_based_lqr_controller_conf_.trajectory_transform_to_com_drive = 
            root_["trajectory_transform_to_com_drive"].asBool();
        lat_based_lqr_controller_conf_.enable_feedback_augment_on_high_speed = 
            root_["enable_feedback_augment_on_high_speed"].asBool();
        lat_based_lqr_controller_conf_.enable_maximun_steer_rate_limit = 
            root_["enable_maximun_steer_rate_limit"].asBool();
        lat_based_lqr_controller_conf_.query_time_nearest_point_only = 
            root_["query_time_nearest_point_only"].asBool();
        lat_based_lqr_controller_conf_.enable_navigation_mode_error_filter = 
            root_["enable_navigation_mode_error_filter"].asBool();
        lat_based_lqr_controller_conf_.reverse_use_dynamic_model = 
            root_["reverse_use_dynamic_model"].asBool();
        if (root_.isMember("lat_err_gain_scheduler")) {
            const Json::Value& scheduler = root_["lat_err_gain_scheduler"];
            Json::Value::Members memberNames = scheduler.getMemberNames();
            lat_based_lqr_controller_conf_.lat_error_gain_scheduler.scheduler.resize(memberNames.size());
            for (size_t i = 0; i < memberNames.size(); ++i) {
                const std::string& memberName = memberNames[i];
                lat_based_lqr_controller_conf_.lat_error_gain_scheduler.scheduler[i].speed = 
                  scheduler[memberName]["speed"].asDouble();
                lat_based_lqr_controller_conf_.lat_error_gain_scheduler.scheduler[i].ratio = 
                  scheduler[memberName]["ratio"].asDouble();
            }
        }
        if (root_.isMember("heading_err_gain_scheduler")) {
            const Json::Value& scheduler = root_["heading_err_gain_scheduler"];
            Json::Value::Members memberNames = scheduler.getMemberNames();
            lat_based_lqr_controller_conf_.heading_error_gain_scheduler.scheduler.resize(memberNames.size());
            for (size_t i = 0; i < memberNames.size(); ++i) {
                const std::string& memberName = memberNames[i];
                lat_based_lqr_controller_conf_.heading_error_gain_scheduler.scheduler[i].speed = 
                    scheduler[memberName]["speed"].asDouble();
                lat_based_lqr_controller_conf_.heading_error_gain_scheduler.scheduler[i].ratio = 
                    scheduler[memberName]["ratio"].asDouble();
            }
        }
        if (root_.isMember("reverse_leadlag_conf")) {
            lat_based_lqr_controller_conf_.reverse_leadlag_conf.innerstate_saturation_level = 
                root_["reverse_leadlag_conf"]["innerstate_saturation_level"].asDouble();
            lat_based_lqr_controller_conf_.reverse_leadlag_conf.alpha = 
                root_["reverse_leadlag_conf"]["alpha"].asDouble();
            lat_based_lqr_controller_conf_.reverse_leadlag_conf.beta = 
                root_["reverse_leadlag_conf"]["beta"].asDouble();
            lat_based_lqr_controller_conf_.reverse_leadlag_conf.tau = 
                root_["reverse_leadlag_conf"]["tau"].asDouble();
        }
        if (root_.isMember("steer_mrac_conf")) {
            lat_based_lqr_controller_conf_.steer_mrac_conf.mrac_model_order = 
                root_["steer_mrac_conf"]["mrac_model_order"].asInt();
            lat_based_lqr_controller_conf_.steer_mrac_conf.reference_time_constant = 
                root_["steer_mrac_conf"]["reference_time_constant"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.reference_natural_frequency = 
                root_["steer_mrac_conf"]["reference_natural_frequency"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.reference_damping_ratio = 
                root_["steer_mrac_conf"]["reference_damping_ratio"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.adaption_state_gain = 
                root_["steer_mrac_conf"]["adaption_state_gain"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.adaption_desired_gain = 
                root_["steer_mrac_conf"]["adaption_desired_gain"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.adaption_nonlinear_gain = 
                root_["steer_mrac_conf"]["adaption_nonlinear_gain"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.adaption_matrix_p = 
                root_["steer_mrac_conf"]["adaption_matrix_p"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.mrac_saturation_level = 
                root_["steer_mrac_conf"]["mrac_saturation_level"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.anti_windup_compensation_gain = 
                root_["steer_mrac_conf"]["anti_windup_compemsation_gain"].asDouble();
            lat_based_lqr_controller_conf_.steer_mrac_conf.clamping_time_constant = 
                root_["steer_mrac_conf"]["clamping_time_constant"].asDouble();
        }
        return true;  
    }

    bool LatControllerTest::LoadControlConf() {
        vehicle_param_ = common::VehicleConfigHelper::GetConfig();
        ts_ = lat_based_lqr_controller_conf_.ts;
        if (ts_ <= 0.0) {
            std::cerr << "[LatController] Invalid control parameter: ts <= 0." << std::endl;
            return false;
        }
        // 车辆参数
        wheelbase_ = vehicle_param_.vehicle_param.wheel_base;
        std::cerr << "[LatController] wheelbase: " << wheelbase_ << std::endl;
        steer_ratio_ = vehicle_param_.vehicle_param.steer_ratio;
        steer_signal_direction_max_degree_ = 
        vehicle_param_.vehicle_param.max_steer_angle / M_PI * 180;
        // LQR控制器配置参数
        cf_ = lat_based_lqr_controller_conf_.cf;
        cr_ = lat_based_lqr_controller_conf_.cr;
        preview_window_ = lat_based_lqr_controller_conf_.preview_window;
        lookahead_station_low_speed_ = 
            lat_based_lqr_controller_conf_.lookahead_station;
        lookback_station_low_speed_ = 
            lat_based_lqr_controller_conf_.lookback_station;
        lookahead_station_high_speed_ = 
            lat_based_lqr_controller_conf_.lookahead_station_high_speed;
        lookback_station_high_speed_ = 
            lat_based_lqr_controller_conf_.lookback_station_high_speed;
        max_lat_acc_ = lat_based_lqr_controller_conf_.max_lateral_acceleration;
        low_speed_bound_ = lat_based_lqr_controller_conf_.switch_speed;
        low_speed_window_ = lat_based_lqr_controller_conf_.switch_speed_window;
        
        const double mass_fl = lat_based_lqr_controller_conf_.mass_fl;
        const double mass_fr = lat_based_lqr_controller_conf_.mass_fr;
        const double mass_rl = lat_based_lqr_controller_conf_.mass_rl;
        const double mass_rr = lat_based_lqr_controller_conf_.mass_rr;
        const double mass_front = mass_fl + mass_fr;
        const double mass_rear = mass_rl + mass_rr;
        mass_ = mass_front + mass_rear;  // 整车质量
    
        lf_ = wheelbase_ * (1.0 - mass_front / mass_);  // 前轮轴距
        lr_ = wheelbase_ * (1.0 - mass_rear / mass_);   // 后轮轴距
    
        iz_ = mass_front * lf_ *lf_ + mass_rear * lr_ *lr_;  // 转动惯量 I_z = m*l^2
    
        lqr_eps_ = lat_based_lqr_controller_conf_.eps;
        lqr_max_iteration_ = lat_based_lqr_controller_conf_.max_iteration;
    
        query_relative_time_ = lat_based_lqr_controller_conf_.query_relative_time;
    
        minimum_speed_protection_ = 
            lat_based_lqr_controller_conf_.minimum_speed_protection;
        return true;
    }

    void LatControllerTest::ProcessLogs(
        const common_msg::SimpleLateralDebug *debug, 
        const common_msg::Chassis *chassis) {
        const std::string log_line = std::to_string(debug->lateral_error) + ","
            + std::to_string(debug->ref_heading) + "," + std::to_string(debug->heading) + ","
            + std::to_string(debug->heading_error) + "," + std::to_string(debug->heading_error_rate) + ","
            + std::to_string(debug->lateral_error_rate) + "," + std::to_string(debug->curvature) + ","
            + std::to_string(debug->steer_angle) + "," + std::to_string(debug->steer_angle_feedforward) + ","
            + std::to_string(debug->steer_angle_lateral_contribution) + ","
            + std::to_string(debug->steer_angle_lateral_rate_contribution) + ","
            + std::to_string(debug->steer_angle_heading_contribution) + ","
            + std::to_string(debug->steer_angle_heading_rate_contribution) + ","
            + std::to_string(debug->steer_angle_feedback) + ","
            + std::to_string(chassis->steering_percentage) + "," 
            + std::to_string(injector_->vehicle_state()->linear_velocity());
        if (FLAGS_enable_csv_debug) {
            steer_log_file_ << log_line << std::endl;
        }
        std::cerr << "Steer_Control_Detial: " << log_line << std::endl;
    }

    void LatControllerTest::ComputeLateralErrors(const double x, 
                                const double y, const double theta,
                                const double linear_v, const double angular_v,
                                const double linear_a,
                                TrajectoryAnalyzer &trajectory_analyzer,
                                common_msg::SimpleLateralDebug *debug,
                                const common_msg::Chassis *chassis)
    {
        common_msg::TrajectoryPoint target_point;  // 规划轨迹点的信息
        // 计算目标点（根据相对时间或距离）
        if (lat_based_lqr_controller_conf_.query_time_nearest_point_only) {
            target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
                // 这里的当前时间应该与规划轨迹的时间基准一直（ros、cyber）
                GetSystemTimeSeconds() + query_relative_time_);  
        } else {
            if (FLAGS_use_navigation_mode && 
                !lat_based_lqr_controller_conf_.enable_navigation_mode_position_update) {
                target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
                GetSystemTimeSeconds() + query_relative_time_);
            } else {
                target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
            } 
        }
        // 全局坐标位置
        const double dx = x - target_point.path_point.x;
        const double dy = y - target_point.path_point.y;
        debug->current_target_point.path_point.x = target_point.path_point.x;
        debug->current_target_point.path_point.y = target_point.path_point.y;
        std::cerr << "Vehicle current localization: " 
                  << "[x]:" << x 
                  << ", "
                  << "[y]:" << y 
                  << std::endl;
        // 目标点处的heading角的正、余弦值
        const double cos_target_heading = std::cos(target_point.path_point.theta);
        const double sin_target_heading = std::sin(target_point.path_point.theta);
        // 计算横向误差
        double lateral_error = -sin_target_heading * dx + cos_target_heading * dy;
        // 导航模式误差滤波器
        if (lat_based_lqr_controller_conf_.enable_navigation_mode_error_filter) {
            lateral_error = lateral_error_filter_.Update(lateral_error);
        }
        debug->lateral_error = lateral_error;
        debug->ref_heading = target_point.path_point.theta;
        // 归一化heading误差到[-pi, pi]
        double heading_error = common::NormalizeAngle(theta -  debug->ref_heading);
        // 导航模式滤波器打开对航向偏差进行滤波，默认关闭
        if (lat_based_lqr_controller_conf_.enable_navigation_mode_error_filter) {
            heading_error = heading_error_filter_.Update(heading_error);
        }
        // 航向误差存入debug指针
        debug->heading_error = heading_error;
        // 根据车速动态调整预瞄距离
        double lookahead_station = 0.0;  // 前瞄距离
        double lookback_station = 0.0;  // 后瞄距离
        if (std::abs(linear_v) >= low_speed_bound_) {
            lookahead_station = lookahead_station_high_speed_;
            lookback_station = lookback_station_high_speed_;
            // 车辆速度小于低速边界速度-低速窗口，使用低速预瞄距离（R与非R档）
        } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
            lookahead_station = lookahead_station_low_speed_;
            lookback_station = lookback_station_low_speed_;
        } else {
            // 车速绝对值小于低速边界速度且大于（低速边界速度-低速窗口速度），使用lerp()函数进行插值计算预瞄距离
            lookahead_station = common::lerp(
                lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
                lookahead_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
            lookback_station = common::lerp(
                lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
                lookback_station_high_speed_, low_speed_bound_, std::fabs(linear_v));
        }
        // 预瞄航向控制
        double heading_error_feedback;
        // 倒档，heading_error_feedback = heading_error
        if (injector_->vehicle_state()->gear() == common_msg::Chassis::GEAR_REVERSE) {
            heading_error_feedback = heading_error;
        } else {
            // 前进档，目标点相对时间+预瞄时间（预瞄距离/纵向速度）作为总的相对时间查找轨迹中对应的目标点
            auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
                target_point.relative_time +
                lookahead_station /
                    (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)));
            // heading_error = 当前航向-参考点航向
            // heading_error_feedback = heading_error + 参考点航向 - 预瞄点航向 = 当前点航向-预瞄点航向
            heading_error_feedback = common::NormalizeAngle(
                heading_error + target_point.path_point.theta -
                lookahead_point.path_point.theta);
        }
        // 考虑预瞄距离的航向误差存入debug指针
        debug->heading_error_feedback = heading_error_feedback;
        double lateral_error_feedback;  // 考虑预瞄距离的横向误差计算
        // 倒车档，lateral_error_feedback = lateral_error - 预瞄
        if (injector_->vehicle_state()->gear() == common_msg::Chassis::GEAR_REVERSE) {
            lateral_error_feedback =
                lateral_error - lookback_station * std::sin(heading_error);
        } else {
            // 前进档，lateral_error_feedback = lateral_error + 预瞄,这里的加减与车辆运动学特性相关
            lateral_error_feedback =
                lateral_error + lookahead_station * std::sin(heading_error);
        }
        // 考虑预瞄距离的横向误差存入debug指针
        debug->lateral_error_feedback = lateral_error_feedback;
        auto lateral_error_dot = linear_v * std::sin(heading_error);     // 横向误差率     = 纵向速度*sing(heading_error)
        auto lateral_error_dot_dot = linear_a * std::sin(heading_error); // 横向误差加速度 = 纵向加速度*sin(heading_error)
        // 倒车航向控制，默认false
        if (FLAGS_reverse_heading_control) {
            if (injector_->vehicle_state()->gear() == common_msg::Chassis::GEAR_REVERSE) {
            lateral_error_dot = -lateral_error_dot;
            lateral_error_dot_dot = -lateral_error_dot_dot;
            }
        }
        // 向心加速度a = v^2/R，v为车辆纵向速度， 阿克曼转向几何关系，tan(theta) = L/R, R, L为车辆转弯半径，轴距
        auto centripetal_acceleration =
            linear_v * linear_v / wheelbase_ *
            std::tan(chassis->steering_percentage / 100 *
                    vehicle_param_.vehicle_param.max_steer_angle / steer_ratio_);
        // 横向误差率、横向误差加速度、向心加速度、横向加速度变化率存入debug指针
        debug->lateral_error_rate = lateral_error_dot;
        debug->lateral_acceleration = lateral_error_dot_dot;
        debug->lateral_centripetal_acceleration = centripetal_acceleration;
        debug->lateral_jerk = 
            (debug->lateral_acceleration - previous_lateral_acceleration_) / ts_;
        previous_lateral_acceleration_ = debug->lateral_acceleration;  // 更新上一时刻的横向加速度
        // 计算航向相关误差
        if (injector_->vehicle_state()->gear() == common_msg::Chassis::GEAR_REVERSE) {
        debug->heading_rate = -angular_v;  // 倒车档，航向率为车辆负航向角速度（来自车辆状态信息）
        } else {
            debug->heading_rate = angular_v;   // 前进档，航向率为车辆正航向角速度（来自车辆状态信息）
        }
        // 参考点的航向角变化率=目标点纵向速度/目标点转弯半径，绕Z轴w=v/R，下面的kappa就是曲率 结果放入debug指针
        debug->ref_heading_rate = target_point.path_point.kappa *
                                    target_point.v;
        // 航向误差率 = 车辆当前航向率 - 参考点航向率
        debug->heading_error_rate = (debug->heading_rate -
                                        debug->ref_heading_rate);
        // 航向加速度 = （航向率 - 上一次航向率）/ T， T 控制周期                              
        debug->heading_acceleration = 
            ((debug->heading_rate - previous_heading_rate_) / ts_);
        // 参考点航向加速度 = （参考点航向率 - 上一次参考点航向率）/ T
        debug->ref_heading_acceleration = (
            (debug->ref_heading_rate - previous_ref_heading_rate_) / ts_);
        // 航向误差加速度 = 航向加速度 - 参考点航向加速度
        debug->heading_error_acceleration = (debug->heading_acceleration -
                                                debug->ref_heading_acceleration);
        previous_heading_rate_ = debug->heading_rate;         // 更新上一时刻的航向率
        previous_ref_heading_rate_ = debug->ref_heading_rate; // 更新上一时刻的参考航向率
        // 航向加速度率 = （航向加速度 - 上一次航向加速度）/ T
        debug->heading_jerk = (
            (debug->heading_acceleration - previous_heading_acceleration_) / ts_);
        // 参考航向加速度 = （参考航向加速度 - 上一次参考航向加速度）/ T
        debug->ref_heading_jerk = (
            (debug->ref_heading_acceleration - previous_ref_heading_acceleration_) /
            ts_);
        // 航向加速度率误差 = 航向加速度率 - 参考航向加速度率
        debug->heading_error_jerk = (debug->heading_jerk -
                                        debug->ref_heading_jerk);
        // 上一次航向加速度
        previous_heading_acceleration_ = debug->heading_acceleration;
        // 上一次参考航向加速度
        previous_ref_heading_acceleration_ = debug->ref_heading_acceleration;
        // 目标点曲率存入debug指针
        debug->curvature = (target_point.path_point.kappa);
}

void LatControllerTest::UpdateState(
    common_msg::SimpleLateralDebug *debug, 
    const common_msg::Chassis *chassis) {
    // 更新车辆信息
    auto vehicle_state = injector_->vehicle_state();
    if (FLAGS_use_navigation_mode) {
        ComputeLateralErrors(0.0, 0.0, driving_orientation_, 
            vehicle_state->linear_velocity(),
            vehicle_state->angular_velocity(),
            vehicle_state->linear_acceleration(),
            trajectory_analyzer_, debug, chassis);
    } else { 
        // 车辆坐标转换
        const auto &com = vehicle_state->ComputeCOMPosition(lr_);
        ComputeLateralErrors(com.x(), com.y(), driving_orientation_,
            vehicle_state->linear_velocity(),
            vehicle_state->angular_velocity(),
            vehicle_state->linear_acceleration(),
            trajectory_analyzer_, debug, chassis);
    }
    // 状态矩阵更新
    /**
     * 是否启用前后距离反馈控制（默认开启）
     * lateral_error_feedback = later_error + 参考点到前瞻点的横向误差
     * heading_error_feedback = heading_error + 参考点到后顾点的航向误差
     */
    if (enable_look_ahead_back_control_) {
        matrix_state_(0, 0) = debug->lateral_error_feedback;
        matrix_state_(2, 0) = debug->heading_error_feedback;
    } else { 
        /**
         * e1 横向误差，lateral_error = 当前点到参考点的横向误差
         * e2 航向误差，heading_error = 当前点到参考点的航向误差
         */
        matrix_state_(0, 0) = debug->lateral_error; 
        matrix_state_(2, 0) = debug->heading_error; 
    }
    matrix_state_(1, 0) = debug->lateral_error_rate; // e1_dot 横向误差变化率
    matrix_state_(3, 0) = debug->heading_error_rate; // e2_dot 航向误差变化率
  /**
   * 更新系统状态矩阵X，横向控制中为了提升控制精度一般不考虑使用预瞄点，preview_window = 0
     在横向控制中，最关键的信息是车辆与参考轨迹之间的横向偏差，
     可以通过增加横向预瞄位置误差来提升横向控制的精度（适用于高速、急转弯、精密泊车等）
   */
    for (size_t i = 0; i < preview_window_; ++i) {
        const double preview_time = ts_ * (i + 1);
        const auto preview_point = 
          trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);
        const auto match_point = 
          trajectory_analyzer_.QueryNearestPointByPosition(
            preview_point.path_point.x,
            preview_point.path_point.y);
        const double dx = preview_point.path_point.x - match_point.path_point.x;
        const double dy = preview_point.path_point.y - match_point.path_point.y;
        const double cos_matched_theta = 
          std::cos(match_point.path_point.theta);
        const double sin_matched_theta = 
          std::sin(match_point.path_point.theta);
        const double preview_d_error = 
          cos_matched_theta * dy - sin_matched_theta * dx;
        matrix_state_(basic_state_size_ + i, 0);
    }
}

void LatControllerTest::UpdateMatrix() {
    double v;  // 车辆线速度
    // R档，用相应的运动学模型替换横向平移运动动力学模型
    if (injector_->vehicle_state()->gear() == 
      common_msg::Chassis::GEAR_REVERSE && 
      !lat_based_lqr_controller_conf_.reverse_use_dynamic_model) {
        v = std::min(injector_->vehicle_state()->linear_velocity(),
         -minimum_speed_protection_);
      matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;  // R档时A第1行第3列可不为零
    } else {
    // v为车辆纵向速度和最小保护速度中的最大值
      v = std::max(injector_->vehicle_state()->linear_velocity(),
         minimum_speed_protection_);
         matrix_a_(0, 2) = 0.0;
    }
    // 更新A矩阵非常数项
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
    // 4 x 4 单位矩阵
    Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
    // 更新离散化矩阵matrix_ad_,双线性 A_d = (I - ts/2 * A).inverse() * (I + ts/2 * A)
    matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() * 
                 (matrix_i + ts_ * 0.5 * matrix_a_);
}


/**
 * @brief 更新预瞄窗口的扩展矩阵matrix_adc_,matrix_bdc_,
          默认不使用预瞄窗口,即preview_window_ = 0
 */
void LatControllerTest::UpdateMatrixCompound() { 
    // 初始化预瞄矩阵子块，block(i, j, rows, cols)
    matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
    matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
    /**
     * matrix_adc_ = [Ad    0    0  ...  0]
              [0     0    1  ...  0]
              [0     0    0  ...  1]
              [0     0    0  ...  0]

       matrix_bdc_ = [Bd]
                    [0 ]
                    [0 ]
                    [1 ]
     */
    if (preview_window_ > 0) {
        // 更新矩阵matrix_bdc_
        matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
        // 更新矩阵matrix_adc_
        for (int i = 0; i < preview_window_ - 1; ++i) {
            matrix_adc_(basic_state_size_ + i, basic_state_size_ + i + 1) = 1;
        }
    }
}


/**
 * @brief 计算横向控制的前馈控制量，用于消除车辆的横向稳态误差，D，R档计算方式不同，转化为百分比
          R档：ø = L * (1/R)，基本阿克曼转向几何关系
          D档：delta_ff = L * (1/R) + K_v*v^2/R - k_3(l_r/R - l_f*m*v^2)/R/L/c_r/2
 * @param[in] ref_curvature 参考曲率
 * @return steer_angle_feedforward 横向控制前馈控制量
 */
double LatControllerTest::ComputeFeedForward(double ref_curvature) const {
    // 不足转向梯度系数 kv
    const double kv = 
        (lr_ * mass_ / 2 / cf_ / wheelbase_) - (lf_ * mass_ / 2 / cr_ / wheelbase_);
    const double v = injector_->vehicle_state()->linear_velocity();
    double steer_angle_feedforwardterm;
    // R档
    if (injector_->vehicle_state()->gear() == common_msg::Chassis::GEAR_REVERSE && 
    !lat_based_lqr_controller_conf_.reverse_use_dynamic_model) {
      steer_angle_feedforwardterm = 
        lat_based_lqr_controller_conf_.reverse_feedback_ratio * 
        wheelbase_ * ref_curvature * 180 / M_PI * steer_ratio_ / 
        steer_signal_direction_max_degree_ * 100; 
    } else { 
    // wheelbase-轴距，ref_curvature-参考曲率,v-纵向速度，k_3-航向误差系数，
    // mass_-整车质量，steer_ratio-转向传动比
        steer_angle_feedforwardterm = 
          (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
            matrix_k_(0, 2) * 
            (lr_ * ref_curvature - lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_))
            * 180 / M_PI * steer_ratio_ / 
            steer_signal_direction_max_degree_ * 100;
    }
    return steer_angle_feedforwardterm;
}

/**
 * @brief 倒车模式更新车辆行驶方向，更新matrix_bd_矩阵
 */
void LatControllerTest::UpdateDrivingOrientation() {
    auto vehicle_state = injector_->vehicle_state();
    driving_orientation_ = vehicle_state->heading();
    if (FLAGS_reverse_heading_control) {
        if (vehicle_state->gear() == common_msg::Chassis::GEAR_REVERSE) {
            driving_orientation_ = common::NormalizeAngle(driving_orientation_ + M_PI);
            matrix_bd_ = -matrix_b_ * ts_;
            std::cerr << "[UpdateDrivingOrientation]: matrix_b_ changed due to reverse driving" 
                      << std::endl;
        }
    }
}

/**
 * @brief 横向控制指令
 */
bool LatControllerTest::ComputeControlCommand(
    const common_msg::LocalizationEstimate *localization,
    const common_msg::Chassis *chassis, 
    const common_msg::ADCTrajectory *planning_published_trajectory,
    common_msg::ControlCommand *cmd) {
    // 车辆状态、规划轨迹、前一次纵向控制
    auto vehicle_state = injector_->vehicle_state();
    vehicle_state->Update(*localization, *chassis);
    auto target_tracking_trajectory = *planning_published_trajectory;
    auto previous_lon_debug = injector_->Get_pervious_lon_debug_info();
    /**
     * 导航模式（默认关闭）下对车辆位置进行更新，从地图坐标系转换到车辆坐标系
    */
  if (FLAGS_use_navigation_mode &&
      lat_based_lqr_controller_conf_.enable_navigation_mode_position_update) 
  {
    auto time_stamp_diff =
        planning_published_trajectory->header.timestamp_sec -
        current_trajectory_timestamp_;

    auto curr_vehicle_x = localization->pose.position.x;
    auto curr_vehicle_y = localization->pose.position.y;

    double curr_vehicle_heading = 0.0;
    const auto &orientation = localization->pose.orientation;
    if (localization->pose.heading > kDoubleEpsilon) {
      curr_vehicle_heading = localization->pose.heading;
    } else {
      curr_vehicle_heading =
          common::QuaternionToHeading(orientation.qw, orientation.qx,
                                            orientation.qy, orientation.qz);
    }
    // 新的规划轨迹
    if (time_stamp_diff > kDoubleEpsilon) {
      init_vehicle_x_ = curr_vehicle_x;
      init_vehicle_y_ = curr_vehicle_y;
      init_vehicle_heading_ = curr_vehicle_heading; 

      current_trajectory_timestamp_ =
          planning_published_trajectory->header.timestamp_sec;
    } else {
      auto x_diff_map = curr_vehicle_x - init_vehicle_x_;
      auto y_diff_map = curr_vehicle_y - init_vehicle_y_;
      auto theta_diff = curr_vehicle_heading - init_vehicle_heading_;

      auto cos_map_veh = std::cos(init_vehicle_heading_);
      auto sin_map_veh = std::sin(init_vehicle_heading_);

      auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
      auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

      auto cos_theta_diff = std::cos(-theta_diff);
      auto sin_theta_diff = std::sin(-theta_diff);

      auto tx = -(cos_theta_diff * x_diff_veh - sin_theta_diff * y_diff_veh);
      auto ty = -(sin_theta_diff * x_diff_veh + cos_theta_diff * y_diff_veh);

      auto ptr_trajectory_points =
          target_tracking_trajectory.trajectory_point;
      std::for_each(
          ptr_trajectory_points.begin(), ptr_trajectory_points.end(),
          [&cos_theta_diff, &sin_theta_diff, &tx, &ty,
           &theta_diff](common_msg::TrajectoryPoint &p) {
            auto x = p.path_point.x;
            auto y = p.path_point.y;
            auto theta = p.path_point.theta;

            auto x_new = cos_theta_diff * x - sin_theta_diff * y + tx;
            auto y_new = sin_theta_diff * x + cos_theta_diff * y + ty;
            auto theta_new = common::NormalizeAngle(theta - theta_diff);

            p.path_point.x = x_new;
            p.path_point.y = y_new;
            p.path_point.theta = theta_new;  // 默认是false关闭的这一段忽略
          });
    }
  }
  // 轨迹分析
  trajectory_analyzer_ = 
    std::move(TrajectoryAnalyzer(&target_tracking_trajectory));
  // 轨迹点转换，后轴中心->车辆质心
  if (((lat_based_lqr_controller_conf_.trajectory_transform_to_com_reverse && 
        vehicle_state->gear() == common_msg::Chassis::GEAR_REVERSE) || 
        (lat_based_lqr_controller_conf_.trajectory_transform_to_com_drive && 
        vehicle_state->gear() == common_msg::Chassis::GEAR_DRIVE)) && 
        enable_look_ahead_back_control_) {
        trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
    }
    // 横向控制-R档重建车辆动力学模型
    if (vehicle_state->gear() == common_msg::Chassis::GEAR_REVERSE) {
        cf_ = - lat_based_lqr_controller_conf_.cf;   // 前轮侧偏刚度
        cr_ = - lat_based_lqr_controller_conf_.cr;   // 后轮侧偏刚度
        matrix_a_(0, 1) = 0.0;
        matrix_a_(0, 2) = 1.0;
    } else {
        cf_ = lat_based_lqr_controller_conf_.cf;
        cr_ = lat_based_lqr_controller_conf_.cr;
        matrix_a_(0, 1) = 1.0;
        matrix_a_(0, 2) = 0.0;
    }
    // 更新矩阵A
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(3, 2) = (lf_ * cf_ + lr_ * cr_) / iz_;
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (cr_ * lr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (cr_ * lr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (cr_ *lr_ * lr_ + lf_ * lf_ * cf_) / iz_;
    // 更新矩阵B
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;  
    // 倒车逻辑：更新车辆航向与矩阵matrix_b_
    UpdateDrivingOrientation();
    // 初始化debug横向调试变量
    common_msg::SimpleLateralDebug *debug = &cmd->debug.simple_lat_debug;
    // debug->clear();
    memset(debug, 0, sizeof(common_msg::SimpleLateralDebug));
    // 计算横向误差并更新矩阵matrix_state_
    UpdateState(debug, chassis);
    // 更新矩阵matrix_a_中的非常数项
    UpdateMatrix();
    // 预瞄控制(defaule = false)，更新矩阵matrix_adc_,matrix_bdc_
    UpdateMatrixCompound();
    // 更新Q、R权重矩阵
    int q_param_size = 
        lat_based_lqr_controller_conf_.matrix_q.size();
    int reverse_q_param_size = 
        lat_based_lqr_controller_conf_.reverse_matrix_q.size();
    if (injector_->vehicle_state()->gear() == 
        common_msg::Chassis::GEAR_REVERSE) {
        for (size_t i = 0; i < reverse_q_param_size; ++i) {
            matrix_q_(i, i) = 
                lat_based_lqr_controller_conf_.reverse_matrix_q[i];
        }
    } else {
        for (size_t i = 0; i < q_param_size; ++i) {
            matrix_q_(i, i) = 
                lat_based_lqr_controller_conf_.matrix_q[i];
        }
    }
    // LQR解算
    uint num_iteration;  // 迭代次数
    double result_diff;  // 误差阈值
    std::cerr << "LQR Q matrix: " << matrix_q_ << std::endl;
    std::cerr << "linear_velocity: " << vehicle_state->linear_velocity() << std::endl;
    if (FLAGS_enable_gain_scheduler) {
        // 插值(B样条)计算Q矩阵值
        matrix_q_updated_(0, 0) = matrix_q_(0, 0) * 
            lat_err_interpolation_->Interpolate(std::fabs(vehicle_state->linear_velocity()));
        matrix_q_updated_(2, 2) = matrix_q_(2, 2) *
            heading_err_interpolation_->Interpolate(std::fabs(vehicle_state->linear_velocity()));
        std::cerr << "LQR Q updated matrix:\n" << matrix_q_updated_ << std::endl;
        // std::cerr << "\nLQR matrix_adc_:\n" << matrix_adc_
        //           << "\nmatrix_bdc_:\n" << matrix_bdc_ 
        //           << "\nmatrix_r_:\n"   << matrix_r_ 
        //           << std::endl;
        common::SolveLQRProblem(
            matrix_adc_, matrix_bdc_, matrix_q_updated_,
            matrix_r_, lqr_eps_, lqr_max_iteration_, 
            &matrix_k_, &num_iteration, &result_diff);
    } else {
        common::SolveLQRProblem(
            matrix_adc_, matrix_bdc_, matrix_q_,
            matrix_r_, lqr_eps_, lqr_max_iteration_, 
            &matrix_k_, &num_iteration, &result_diff);
    }
    std::cerr << "LQR iterations is " << num_iteration << ", max iteration threshold is " 
              << lqr_max_iteration_ << ", reslut_diff is " << result_diff 
              << ", matrix_k_ is "<< matrix_k_ << std::endl;
    // 方向盘反馈控制量：u = -kx
    const double steer_angle_feedback = 
        -(matrix_k_ * matrix_state_)(0, 0) 
        * 180 / M_PI 
        * steer_ratio_ 
        / steer_signal_direction_max_degree_ 
        * 100;
    // 方向盘前馈控制量
    const double steer_angle_feedforward = ComputeFeedForward(debug->curvature);
    //
    double steer_angle = 0.0;
    double steer_angle_feedback_augment = 0.0;   // 反馈控制补偿
    // 超前/滞后补偿控制LeadlagControl false
    if (enable_leadlag_) {
        if (lat_based_lqr_controller_conf_.
            enable_feedback_augment_on_high_speed || 
            std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_ ) {
                steer_angle_feedback_augment = 
                    leadlag_controller_.Control(-matrix_state_(0, 0), ts_) 
                    * 180 / M_PI
                    * steer_ratio_ 
                    / steer_signal_direction_max_degree_ 
                    * 100;
            // 速度过度窗口，采用线性插值计算补偿量平滑速度 
            if (std::fabs(vehicle_state->linear_velocity()) > 
              low_speed_bound_ - low_speed_window_) {
                steer_angle_feedback_augment = common::lerp(steer_angle_feedback_augment, 
                    low_speed_bound_ - low_speed_window_, 0.0, low_speed_bound_,
                    std::fabs(vehicle_state->linear_velocity()));
            }
        }
    }
    // 控制量 = 反馈控制量 + 前馈控制量 + 滞后补偿控制量
    steer_angle = steer_angle_feedback 
                + steer_angle_feedforward
                + steer_angle_feedback_augment;
    std::cerr << "steer_angle: " << steer_angle << std::endl;
    // 方向盘最大转角限制
    const double steer_limit = FLAGS_set_steer_limit ? std::atan(
        max_lat_acc_ * wheelbase_ 
        / (vehicle_state->linear_velocity() * vehicle_state->linear_velocity())) 
        * steer_ratio_ 
        * 180 / M_PI 
        / steer_signal_direction_max_degree_ 
        * 100 : 100.0;
    // 利用方向盘最大转角速率限制转角变化值
    const double steer_diff_with_max_rate = 
      lat_based_lqr_controller_conf_.enable_maximun_steer_rate_limit ? 
      vehicle_param_.vehicle_param.max_steer_angle_rate * ts_
      * 180 / M_PI 
      / steer_signal_direction_max_degree_ 
      * 100 : 100.0;
    // std::cerr << "steer_limit: " << steer_limit << std::endl;
    // std::cerr << "steer_diff_with_max_rate: " << steer_diff_with_max_rate << std::endl;
    // 车辆方向盘当前实际转角
    const double steering_position = chassis->steering_percentage;
    /**
     * MRAC控制模型取消
     */
     pre_steering_position_ = steering_position;  // 前一次方向盘转角
    double steer_angle_limited = common::Clamp(steer_angle, 
        -steer_limit, steer_limit);
    steer_angle = steer_angle_limited;
    debug->steer_angle_limited = steer_angle_limited;  // debug写入
    // 对方向盘控制量进行滤波处理并取值限制[-100.0, 100.0]
    steer_angle = digital_filter_.Filter(steer_angle);
    steer_angle = common::Clamp(steer_angle, -100.0, 100.0);
    // 车辆为D档，车速小于方向盘限定车速并处于自动驾驶模式下，方向盘控制指令不变
    if (injector_->vehicle_state()->gear() != common_msg::Chassis::GEAR_REVERSE) {
        if (std::abs(vehicle_state->linear_velocity() < 
        lat_based_lqr_controller_conf_.lock_steer_speed || 
        previous_lon_debug->path_remain <= 0) && vehicle_state->gear() 
        == common_msg::Chassis::GEAR_DRIVE && chassis->driving_mode 
        == common_msg::Chassis::COMPLETE_AUTO_DRIVE) {
            std::cerr << "Into lock steer, path_remain is " << 
            previous_lon_debug->path_remain << "linear_velocity is " 
            << vehicle_state->linear_velocity() << std::endl;
            steer_angle = pre_steer_angle_;
        }
    }
    // 输出转角指令，最大转角速率进行限幅，范围=上次的转角指令+/-最大转角速率 * Ts
    cmd->steering_target = common::Clamp(steer_angle, 
        pre_steer_angle_ - steer_diff_with_max_rate, 
        pre_steer_angle_ + steer_diff_with_max_rate);
    cmd->steering_rate = FLAGS_steer_angle_rate;
    pre_steer_angle_ = cmd->steering_target;
    // 横向误差控制量百分比 -k1 *e1
    const double steer_angle_lateral_contribution = 
        -matrix_k_(0, 0) * matrix_state_(0, 0) 
        * 180 / M_PI 
        * steer_ratio_ 
        / steer_signal_direction_max_degree_ 
        * 100;
    // 横向误差率控制量百分比 -k2 *e1'
    const double steer_angle_lateral_rate_contribution = 
        -matrix_k_(0, 1) * matrix_state_(1, 0) 
        * 180 / M_PI 
        * steer_ratio_ 
        / steer_signal_direction_max_degree_ 
        * 100;
    // 航向误差控制量百分比 -k3 *e2
    const double steer_angle_heading_contribution = 
        -matrix_k_(0, 2) * matrix_state_(2, 0) 
        * 180 / M_PI 
        * steer_ratio_ 
        / steer_signal_direction_max_degree_ 
        * 100;
    // 航向误差率控制量百分比 -k4 *e2'
    const double steer_angle_heading_rate_contribution = 
        -matrix_k_(0, 3) * matrix_state_(3, 0) 
        * 180 / M_PI 
        * steer_ratio_ 
        / steer_signal_direction_max_degree_ 
        * 100;
    // debug 写入
    debug->heading = driving_orientation_;                    // 车辆当前航向角
    debug->steer_angle = steer_angle;                         // 输出方向盘指令
    debug->steer_angle_feedback = steer_angle_feedback;       // 反馈控制量
    debug->steer_angle_feedback_augment =                     // 滞后补偿控制量
        steer_angle_feedback_augment;
    debug->steer_angle_feedforward = steer_angle_feedforward; // 前馈控制量
    debug->steering_position = steering_position;             // 方向盘当前转角
    debug->ref_speed = vehicle_state->linear_velocity();      // 车辆当前线速度
    debug->steer_angle_lateral_contribution =                 // 横向误差控制量贡献
        steer_angle_lateral_contribution;
    debug->steer_angle_lateral_rate_contribution =            // 横向误差率控制量贡献 
        steer_angle_lateral_rate_contribution;
    debug->steer_angle_heading_contribution =                 // 航向误差控制量贡献
        steer_angle_heading_contribution;
    debug->steer_angle_heading_rate_contribution =            // 航向误差率控制量贡献
        steer_angle_heading_rate_contribution;
    // 日志录入
    ProcessLogs(debug, chassis);
    return true;
}

} // namespace control