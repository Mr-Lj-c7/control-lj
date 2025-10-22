#ifndef LAT_CONTROLLER_TEST_H
#define LAT_CONTROLLER_TEST_H
// control
#include "control_component/controller_task_base/common/trajectory_analyzer.h"
#include "control_component/controller_task_base/common/dependency_injector.h"
#include "controller/lat_based_lqr_controller/config/lat_base_lqr_controller_conf.h"
#include "control_component/controller_task_base/common/leadlag_controller.h"
#include "common/configs/vehicle_config_helper.h"
#include "common_msg/chassis_msgs/chassis_msg.h"
#include "common_msg/control_msgs/control_cmd_msg.h"
#include "common_msg/config_msgs/vehicle_config_msg.h"
#include "common/filters/mean_filter.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/filters/math.h"
#include "common/math/interpolation_1d.h"
#include "common/math/linear_quadratic_regulator.h"
// C++ 17
#include <chrono>
#include <memory>
#include <fstream>
#include <iomanip>
// Eigen3.3.7, OpenCV4.2.0
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;

namespace control{
class LatControllerTest {
public:
    LatControllerTest();
    ~LatControllerTest();

    double GetSystemTimeSeconds() const;

    void SetInjector(std::shared_ptr<DependencyInjector> injector) {
        injector_ = injector;
    }

    void Stop();

    std::string Name() const;

    // 横向控制器初始化
    bool Init(std::shared_ptr<DependencyInjector> injector);


    // 控制指令
    bool ComputeControlCommand(
        const common_msg::LocalizationEstimate *localization,
        const common_msg::Chassis *chassis, 
        const common_msg::ADCTrajectory *trajectory,
        common_msg::ControlCommand *cmd);
    
protected:
    // 日志记录器
    void CloseLogFile();
    void LogInitParameters();
    void ProcessLogs(
        const common_msg::SimpleLateralDebug *debug, 
        const common_msg::Chassis *chassis);

    // 参数初始化
    bool LoadControlConf();
    bool LoadLatBasedLqrControllerConf(const std::string 
        &lat_based_lqr_controller_config_file);
    

    void InitialzeFilters();      // IIR滤波器
    void LoadLatGainScheduler();  // 增益控制器

    // 更新车辆状态方程中的车辆状态矩阵X=[e1, e1', e2, e2'],e1, e2为横向误差和航向误差
    void UpdateState(common_msg::SimpleLateralDebug *debug, 
            const common_msg::Chassis *chassis);

    // 更新车辆当前朝向
    void UpdateDrivingOrientation();
    // 更新系统状态矩阵matrix_a_, matrix_ad_
    void UpdateMatrix();
    // 前瞻距离控制,更新matrix_ad_
    void UpdateMatrixCompound();

    // 前馈控制量计算
    double ComputeFeedForward(double ref_curvature) const;

    // 计算横向相关误差
    void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            TrajectoryAnalyzer &trajectory_analyzer,
                            common_msg::SimpleLateralDebug *debug,
                            const common_msg::Chassis *chassis);
protected:
    // 日志
    bool FLAGS_enable_csv_debug = true;
    std::ofstream steer_log_file_;
    const std::string name_;

    // 规划轨迹
    TrajectoryAnalyzer trajectory_analyzer_;

    // vehicle           横向控制器消息参数
    LatBaseLqrControllerConf lat_based_lqr_controller_conf_;
    common_msg::VehicleConfig vehicle_param_;

    // 车辆动力学模型参数
    double ts_ = 0.0;         // T
    double cf_ = 0.0;         // 前、后轮侧向刚度
    double cr_ = 0.0;
    double wheelbase_ = 0.0;  // 轴距
    double mass_ = 0.0;       // 整车质量
    double lf_ = 0.0;         // 前、后悬长度
    double lr_ = 0.0;  
    double iz_ = 0.0;         // 转动惯量
    double steer_ratio_ = 0.0;  //  传动比
    // 单侧最大转向
    double steer_signal_direction_max_degree_ = 0.0;  
    double max_lat_acc_ = 0.0; // 最大横向加速度
    double preview_window_ = 0.0;  // 预瞄窗口

    // 如果启用，实现将车辆当前时间向前加0.8s在规划轨迹上的对应点作目标点，默认关闭
    double query_relative_time_;
    bool FLAGS_use_navigation_mode = false;
    bool FLAGS_reverse_heading_control = false;

    // 数字滤波器
    common::DigitalFilter digital_filter_;
    // 均值滤波器
    common::MeanFilter lateral_error_filter_;
    common::MeanFilter heading_error_filter_;

    // B样条插值器
    std::unique_ptr<common::Interpolation1D> lat_err_interpolation_;
    std::unique_ptr<common::Interpolation1D> heading_err_interpolation_;

    // LeadlagController超前/滞后控制器
    bool enable_leadlag_ = false;
    common::LeadLagController leadlag_controller_;

    // MracController 自适应控制器，适应不同转向机（暂不考虑）
    bool enable_mrac_ = false;

    // 前向/后向距离控制
    bool enable_look_ahead_back_control_ = false;

    // 预瞄控制相关参数
    double lookahead_station_low_speed_ = 0.0;    // 低速前向行驶-预瞄距离（非R档）
    double lookback_station_low_speed_ = 0.0;     // 低速倒车行驶-预瞄距离（R档）
    double lookahead_station_high_speed_ = 0.0;   // 高速前向行驶-预瞄距离（非R档）
    double lookback_station_high_speed_ = 0.0;    // 高速倒车行驶-预瞄距离（R档）

    // LQR控制算法求解器参数，最大迭代次数
    int lqr_max_iteration_ = 0;
    // LQR控制算法求解器参数，求解精度
    double lqr_eps_ = 0.0;

    
    // 车辆系统状态方程（车辆方向盘动力学模型）X'=A*X(t)+B*U(t)+B1*ø'(t), ø'(t)为期望的航向角变化率
    // 车辆状态矩阵X，[e1, e1', e2, e2']
    const int basic_state_size_ = 4;
    // 车辆状态矩阵[e1, e1', e2, e2']
    Eigen::MatrixXd matrix_state_;
    // 矩阵A 4x4 B 4x1，连续状态方程
    Eigen::MatrixXd matrix_a_;
    Eigen::MatrixXd matrix_b_;
    // 矩阵A 4x4 B 4x1，离散状态方程（双线性变换）
    Eigen::MatrixXd matrix_ad_;
    Eigen::MatrixXd matrix_bd_;
    // 矩阵A B，扩展状态方程
    Eigen::MatrixXd matrix_adc_;
    Eigen::MatrixXd matrix_bdc_;
    // 状态反馈矩阵K u = -kx LQR求解出最优的K K=[k0, k1, k2, k3] 1x4
    Eigen::MatrixXd matrix_k_;
    // LQR 控制量权重系数矩阵R 控制量为前轮转角，R 1x1
    Eigen::MatrixXd matrix_r_;
    // LQR 系统状态量权重系数矩阵Q 状态反馈量[e1, e1', e2, e2'] 4x4
    Eigen::MatrixXd matrix_q_;
    Eigen::MatrixXd matrix_q_updated_;  // 增益控制系数
    // 车辆状态方程系数矩阵A中与车速v相关的时变项系数
    Eigen::MatrixXd matrix_a_coeff_;

   // 低速切换窗口，在窗口范围内对控制参数进行线性插值，防止低高速控制参数切换时过于生硬
   double low_speed_bound_ = 0.0;
   // 低速切换窗口，在窗口范围内对控制参数进行线性插值，防止低高速控制参数切换时过于生硬
   double low_speed_window_ = 0.0;
   // 最小保护速度，车辆状态方程系数矩阵A中有好几项分母中含有变量v，
   // 当v为0或者过小会引发冲击或者错误，因此需要设置一个v保护值，当v小于保护值，将v设置为该值
   double minimum_speed_protection_ = 0.1;

   // 上一时刻横向加速度，用于计算横向加加速度
   double previous_lateral_acceleration_ = 0.0;
   // 上一时刻航向角变化率
   double previous_heading_rate_ = 0.0;
   // 上一时刻的参考点航向角变化率
   double previous_ref_heading_rate_ = 0.0;
   // 上一时刻航向角加速度
   double previous_heading_acceleration_ = 0.0;
   // 上一时刻的参考点航向角加速度
   double previous_ref_heading_acceleration_ = 0.0;

   // 获取车辆状态信息指针对象
   std::shared_ptr<DependencyInjector> injector_;
   
private:
   const std::string lat_based_lqr_controller_conf_path 
       = "controller/lat_based_lqr_controller/conf/controller_conf.json";
};
} // control


#endif