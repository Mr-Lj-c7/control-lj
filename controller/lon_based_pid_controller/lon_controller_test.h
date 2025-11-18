#ifndef LON_CONTROLLER_TEST_H
#define LON_CONTROLLER_TEST_H


#include <chrono>
#include <memory>
#include "control_component/controller_task_base/common/trajectory_analyzer.h"
#include "control_component/controller_task_base/common/dependency_injector.h"
#include "controller/lon_based_pid_controller/conf/lon_based_pid_controller_conf.h"
#include "control_component/controller_task_base/common/leadlag_controller.h"
#include "control_component/controller_task_base/common/pid_controller.h"
#include "control_component/config/leadlag_conf.h"
#include "control_component/config/calibration_table.h"
#include "common_msg/chassis_msgs/chassis_msg.h"
#include "common_msg/control_msgs/control_cmd_msg.h"
#include "common_msg/config_msgs/vehicle_config_msg.h"
#include "common_msg/planning_msgs/planning_msg.h"
#include "common/filters/mean_filter.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/filters/math.h"
#include "common/math/interpolation_1d.h"
#include "common/configs/config_gflags.h"
#include "common/math/interpolation_2d.h"
#include "controller/lon_based_pid_controller/util/check_pit.h"
#include <iomanip>

namespace control {
class LonControllerTest {
public:
    LonControllerTest();
    ~LonControllerTest();

    double GetSystemTimeSeconds() const;

    // 关联各模块消息
    void SetInjector(std::shared_ptr<DependencyInjector> injector) {
        injector_ = injector;
    }

    void Stop() const;  

    // 系统名称
    std::string Name() const;

    // 初始化纵向控制系统
    bool Init(std::shared_ptr<DependencyInjector> injector);

    /**
     * @brief 计算纵向控制指令
     * @param[in] localization 定位消息
     * @param[in] chassis 线控消息
     * @param[in] planning_published_trajectory  规划消息
     * @param[out] cmd 控制指令
     * @return    void
     */
    bool ComputeControlCommand(const common_msg::LocalizationEstimate *localization,
                               const common_msg::Chassis *chassis,
                               const common_msg::ADCTrajectory *planning_published_trajectory,
                               common_msg::ControlCommand *cmd);

    LonBasedPidControllerConf lon_based_pidcontroller_conf_;  // 纵向控制器配置

protected:
     /**
     * @brief 计算纵向误差
     * @param[in] trajectory    规划轨迹
     * @param[in] preview_time  预瞄时间
     * @param[in] ts            采样时间
     * @param[in] debug         调试变量
     * @return    void
     */
    void ComputeLongitudinalErrors(TrajectoryAnalyzer *trajectory,
                                 const double preview_time, const double ts,
                                 common_msg::SimpleLongitudinalDebug *debug);
    
    /**
     * @brief 计算路径剩余距离 
     * @param[in] debug
     * @return null
     */                                
    void GetPathRemain(common_msg::SimpleLongitudinalDebug *debug);

    // 纵向PID控制器配置参数
    bool LoadLonBasedPIDControllerConf(
        const std::string &lon_based_pidcontroller_config_file);
    
    // 加载标定表
    bool LoadCalibrationTable(const std::string& calibration_table_path_);

    // 日志器
    void ProcessLogs(const common_msg::SimpleLongitudinalDebug *debug);

    std::shared_ptr<DependencyInjector> injector_; 

private:
// 坡度角滤波
    void SetDigitalFilterPitchAngle();

    // 初始化车辆标定表
    void InitControlCalibrationTable();

    /**
     * @brief 设置滤波器
     * @param[in] ts 采样时间
     * @param[in] cutoff_freq 截止频率
     * @param[in] digital_filter 低通滤波器
    */
    void SetDigitalFilter(double ts, double cutoff_freq, 
                          common::DigitalFilter *digital_filter);
   /**
   * @brief 终点停车
   * @param[in] debug  
   * @return ture:destination stop ,
   *         false:not destination stop
   */   
    bool IsStopByDestination(common_msg::SimpleLongitudinalDebug *debug);

    /**
     * @brief 行人相关因素长时间停车
     * @param[in] debug 
    */
    bool IsPedestrianStopLongTerm(common_msg::SimpleLongitudinalDebug *debug);

    // 长期完全停车，用于EPB控制
    bool IsFullStopLongTerm(common_msg::SimpleLongitudinalDebug *debug);

    /**
     * @brief EPB（电子驻车制动）控制，状态翻转策略
     * @param[in]  conf 纵向PID控制器参数
     * @param[out] control_command 控车命令
    */
    void SetParkingBrake(const LonBasedPidControllerConf *conf,
                         common_msg::ControlCommand *control_command);

    void CloseLogFile();

    // 其他模块消息
    const common_msg::LocalizationEstimate *localization_ = nullptr;  // 定位消息
    const common_msg::Chassis *chassis_ = nullptr;  // 线控消息
    const common_msg::ADCTrajectory *trajectory_message_ = nullptr;   // 规划消息
    std::unique_ptr<common::Interpolation2D> control_interpolation_;  // 刹车/油门量插值
    std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_; // 轨迹处理

    // 控制器
    common::PIDController speed_pid_controller_;            // 速度PID控制器
    common::PIDController station_pid_controller_;          // 位置PID控制器
    common::LeadlagController speed_leadlag_controller_;    // 速度补偿控制器
    common::LeadlagController station_leadlag_controller_;  // 位置补偿控制器

    common::DigitalFilter digital_filter_pitch_angle_;  // 坡度角滤波

    // 配置参数
    // common_msg::VehicleParam vehicle_param_;       // 车辆参数
    common_msg::VehicleConfig vehicle_param_;       // 车辆参数
    common_msg::calibration_table calibration_table_;  // 加载车辆标定表

    std::string name_;  // 系统名
    bool controller_initialized_ = false;

    double previous_acceleration_reference_ = 0.0;  // 前一次加速度参考
    double previous_acceleration_ = 0.0;            // 前一次加速度
    double reference_spd_cmd = 0.0;                 // 参考速度指令
    double reference_spd_ = 0.0;                    // 参考速度

    // 停车参数
    // 行人因素
    bool is_stop_by_pedestrian_ = false;
    bool is_stop_by_pedestrian_previous_ = false;
    double start_time_ = 0.0;   
    double wait_time_diff_ = 0.0;  // 等待时间
    // 停车状态
    bool is_full_stop_previous_ = false;
    double is_full_stop_start_time_ = 0.0;     // 停车开始时间
    double is_full_stop_wait_time_diff_ = 0.0; // 停车时长
    // EPB
    bool epb_on_change_switch_ = true;  // epb开启
    bool epb_off_change_switch_ = true; // epb关闭
    int epb_change_count_ = 0;          // epb切换次数
    int smode_num_ = 0;

    double standstill_narmal_acceleration_ = 0.0; // 驻车时加速度
    double stop_gain_acceleration_ = 0.0;         // 停车增加速度
    double max_path_remain_when_stopped_ = 0.0;   // 停车最大距离
    bool parking_release_ = false;                // 驻车释放

    // 日志
    std::ofstream speed_log_file_;

    // LonControllerConf path
    const std::string lon_based_pidcontroller_conf_path_ = 
        "../../controller/lon_based_pid_controller/conf/controller_conf.json";
    const std::string calibration_table_path_ = 
        "../../control_component/config/conf/calibration_table.json";
};

} // namespace control

#endif