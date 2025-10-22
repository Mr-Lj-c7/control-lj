#include "lat_controller_test.h"


namespace control{
using Matrix = Eigen::MatrixXd;
namespace {
    std::string GetLogFileName() {
        time_t raw_time;
        char name_buffer[80];
        std::time(&raw_time);  // 系统时间
        std::tm time_tm;       // 时间结构体
        #ifdef _WIN32
            localtime_s(&time_tm, &raw_time);
        #else
            localtime_r(&raw_time, &time_tm);
        #endif
        // 格式化时间
        strftime(name_buffer, 80, 
            "controller/lat_based_lqr_controller/csv/steer_log_simple_optimmal_%F_%H%M%S.csv", 
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
        std::cerr << "Using" << name_ << std::endl;
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
        }
        for (const auto& scheduler : heading_err_gain_scheduler.scheduler) {
            xy2.push_back(std::pair<double, double>(scheduler.speed, scheduler.ratio));
        }
        // 清空并初始化插值器
        lat_err_interpolation_.reset(new common::Interpolation1D);
        assert(lat_err_interpolation_->Init(xy1) && 
            "Failed to initialize lateral error gain scheduler.");
        heading_err_interpolation_.reset(new common::Interpolation1D);
        assert(heading_err_interpolation_->Init(xy2) && 
            "Failed to initialize heading error gain scheduler.");
    };

    bool LatControllerTest::LoadLatBasedLqrControllerConf(
        const std::string &lat_based_lqr_controller_config_file) {
        LatBaseLqrControllerConf lat_lqr_param;
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
        lat_lqr_param.ts = root_["ts"].asDouble();
        lat_lqr_param.preview_window = root_["preview_windows"].asInt();
        lat_lqr_param.cf = root_["cf"].asDouble();
        lat_lqr_param.cr = root_["cr"].asDouble();
        lat_lqr_param.mass_fl = root_["mass_fl"].asInt();
        lat_lqr_param.mass_fr = root_["mass_fr"].asInt();
        lat_lqr_param.mass_rl = root_["mass_rl"].asInt();
        lat_lqr_param.mass_rr = root_["mass_rr"].asInt();
        lat_lqr_param.eps = root_["esp"].asDouble();
        if (root_.isMember("matrix_q")) {
            uint16_t matrix_size = root_["matrix_q"].size();
            for (size_t i = 0; i < matrix_size; ++i) {
                lat_lqr_param.matrix_q.push_back(
                    root_["matrix_q"][static_cast<uint8_t>(i)].asDouble());
            }
        }
        // lat_lqr_param.matrix_q_0 = root_["mtrix_q_0"].asDouble();
        // lat_lqr_param.matrix_q_1 = root_["matrix_q_1"].asDouble();
        // lat_lqr_param.matrix_q_2 = root_["matrix_q_2"].asDouble();
        // lat_lqr_param.matrix_q_3 = root_["matrix_q_3"].asDouble();
        if (root_.isMember("reverse_matrix_q")) {
            uint16_t matrix_size = root_["reverse_matrix_q"].size();
            for (size_t i = 0; i < matrix_size; ++i) {
                lat_lqr_param.matrix_q.push_back(
                    root_["reverse_matrix_q"][static_cast<uint8_t>(i)].asDouble());
            }
        }
        // lat_lqr_param.reverse_matrix_q_0 = 
        //     root_["reverse_matrix_q_0"].asDouble();
        // lat_lqr_param.reverse_matrix_q_1 = 
        //     root_["reverse_matrix_q_1"].asDouble();
        // lat_lqr_param.reverse_matrix_q_2 = 
        //     root_["reverse_matrix_q_2"].asDouble();
        // lat_lqr_param.reverse_matrix_q_3 = 
        //     root_["reverse_matrix_q_3"].asDouble();
        lat_lqr_param.cutoff_freq = root_["cutoff_freq"].asInt();
        lat_lqr_param.max_iteration = root_["max_iteration"].asInt();
        lat_lqr_param.mean_filter_window_size = 
            root_["mean_filter_window_size"].asInt();
        lat_lqr_param.max_lateral_acceleration = 
            root_["max_lateral_acceleration"].asDouble();
        lat_lqr_param.enable_reverse_leadlag_compensation = 
            root_["enable_reverse_leadlag_compensation"].asBool();
        lat_lqr_param.enable_steer_mrac_control = 
            root_["enable_steer_mrac_control"].asBool();
        lat_lqr_param.enable_look_ahead_back_control = 
            root_["enable_look_ahead_back_control"].asBool();
        lat_lqr_param.lookahead_station = 
            root_["lookahead_station"].asDouble();
        lat_lqr_param.lookback_station = 
            root_["lookback_station"].asDouble();
        lat_lqr_param.lookahead_station_high_speed = 
            root_["lookahead_station_high_speed"].asDouble();
        lat_lqr_param.lookback_station_high_speed = 
            root_["lookback_sation_high_speed"].asDouble();
        lat_lqr_param.switch_speed = 
            root_["switch_speed"].asDouble();
        lat_lqr_param.switch_speed_window = 
            root_["switch_speed_window"].asDouble();
        lat_lqr_param.query_relative_time = 
            root_["query_relative_time"].asDouble();
        lat_lqr_param.lock_steer_speed = 
            root_["lock_steer_speed"].asDouble();
        lat_lqr_param.reverse_feedback_ratio = 
            root_["reverse_feedback_ratio"].asDouble();
        lat_lqr_param.minimum_speed_protection = 
            root_["minimum_speed_protection"].asDouble();
        lat_lqr_param.enable_navigation_mode_position_update = 
            root_["enable_navigation_mode_position_update"].asBool();
        lat_lqr_param.trajectory_transform_to_com_reverse = 
            root_["trajectory_transform_to_com_reverse"].asBool();
        lat_lqr_param.trajectory_transform_to_com_drive = 
            root_["trajectory_transform_to_com_drive"].asBool();
        lat_lqr_param.enable_feedback_augment_on_high_speed = 
            root_["enable_feedback_augment_on_high_speed"].asBool();
        lat_lqr_param.enable_maximun_steer_rate_limit = 
            root_["enable_maximun_steer_rate_limit"].asBool();
        lat_lqr_param.query_time_nearest_point_only = 
            root_["query_time_nearest_point_only"].asBool();
        lat_lqr_param.enable_navigation_mode_error_filter = 
            root_["enable_navigation_mode_error_filter"].asBool();
        lat_lqr_param.reverse_use_dynamic_model = 
            root_["reverse_use_dynamic_model"].asBool();
        if (root_.isMember("lat_err_gain_scheduler")) {
            uint32_t xy1 = root_["lat_err_gain_scheduler"].size();
            lat_lqr_param.lat_error_gain_scheduler.scheduler.resize(xy1);
            for (size_t i = 0; i < xy1; ++i) {
                lat_lqr_param.lat_error_gain_scheduler.scheduler[i].speed = 
                    root_["lat_err_gain_scheduler"][static_cast<uint16_t>(i)]["speed"].asDouble();
                lat_lqr_param.lat_error_gain_scheduler.scheduler[i].ratio = 
                    root_["lat_err_gain_scheduler"][static_cast<uint16_t>(i)]["ratio"].asDouble();
            }
        }
        if (root_.isMember("heading_err_gain_scheduler")) {
            uint32_t xy2 = root_["heading_err_gain_scheduler"].size();
            lat_lqr_param.heading_error_gain_scheduler.scheduler.resize(xy2);
            for (size_t i = 0; i < xy2; ++i) {
                lat_lqr_param.heading_error_gain_scheduler.scheduler[i].speed = 
                    root_["heading_err_gain_scheduler"][static_cast<uint16_t>(i)]["speed"].asDouble();
                lat_lqr_param.heading_error_gain_scheduler.scheduler[i].ratio = 
                    root_["heading_err_gain_scheduler"][static_cast<uint16_t>(i)]["ratio"].asDouble();
            }
        }
        if (root_.isMember("reverse_leadlag_conf")) {
            lat_lqr_param.reverse_leadlag_conf.innerstate_saturation_level = 
                root_["reverse_leadlag_conf"]["innerstate_saturation_level"].asDouble();
            lat_lqr_param.reverse_leadlag_conf.alpha = 
                root_["reverse_leadlag_conf"]["alpha"].asDouble();
            lat_lqr_param.reverse_leadlag_conf.beta = 
                root_["reverse_leadlag_conf"]["beta"].asDouble();
            lat_lqr_param.reverse_leadlag_conf.tau = 
                root_["reverse_leadlag_conf"]["tau"].asDouble();
        }
        if (root_.isMember("steer_mrac_conf")) {
            lat_lqr_param.steer_mrac_conf.mrac_model_order = 
                root_["steer_mrac_conf"]["mrac_model_order"].asInt();
            lat_lqr_param.steer_mrac_conf.reference_time_constant = 
                root_["steer_mrac_conf"]["reference_time_constant"].asDouble();
            lat_lqr_param.steer_mrac_conf.reference_natural_frequency = 
                root_["steer_mrac_conf"]["reference_natural_frequency"].asDouble();
            lat_lqr_param.steer_mrac_conf.reference_damping_ratio = 
                root_["steer_mrac_conf"]["reference_damping_ratio"].asDouble();
            lat_lqr_param.steer_mrac_conf.adaption_state_gain = 
                root_["steer_mrac_conf"]["adaption_state_gain"].asDouble();
            lat_lqr_param.steer_mrac_conf.adaption_desired_gain = 
                root_["steer_mrac_conf"]["adaption_desired_gain"].asDouble();
            lat_lqr_param.steer_mrac_conf.adaption_nonlinear_gain = 
                root_["steer_mrac_conf"]["adaption_nonlinear_gain"].asDouble();
            lat_lqr_param.steer_mrac_conf.adaption_matrix_p = 
                root_["steer_mrac_conf"]["adaption_matrix_p"].asDouble();
            lat_lqr_param.steer_mrac_conf.mrac_saturation_level = 
                root_["steer_mrac_conf"]["mrac_saturation_level"].asDouble();
            lat_lqr_param.steer_mrac_conf.anti_windup_compensation_gain = 
                root_["steer_mrac_conf"]["anti_windup_compemsation_gain"].asDouble();
            lat_lqr_param.steer_mrac_conf.clamping_time_constant = 
                root_["steer_mrac_conf"]["clamping_time_constant"].asDouble();
        }   
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
}