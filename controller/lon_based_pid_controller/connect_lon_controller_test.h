#include "lon_controller_test.h"
#include <fstream>
#include <json.h>

using LocalizationPb = control::common_msg::LocalizationEstimate;
using ChassisPb = control::common_msg::Chassis;  
using PlanningTrajectoryPb = control::common_msg::ADCTrajectory;
using control::common::VehicleStateProvider;
#define STATUS nullptr
#define relativate_time_ 0
#define LOG_DEBUG

namespace control{
class ConnectLonControllerTest : public LonControllerTest {
public:
    ConnectLonControllerTest() {
        now_timestamp_ = GetSystemTimeSeconds();
    }
    ~ConnectLonControllerTest() = default;
    // 加载数据
    LocalizationPb LoadLocalizationPb(const std::string& filename) const;

    ChassisPb LoadChassisPb(const std::string& filename) const;

    PlanningTrajectoryPb LoadPlanningTrajectoryPb(const std::string& filename) const;

    // 检核结果
    bool CustomExpectedResultWithMessage(
        const std::string& description,
        const double actual_value,
        const double expected_value,
        const double tolerance) const;

    char* Run();   

protected:
    // 参考点
    double station_reference_expected = 0.16716666937000002;   // 参考路径点的纵向距离
    double speed_reference_expected = 1.70833337307;           // 参考速度
    double station_error_expected = -0.0128094011029;          // 纵向位置误差
    double speed_error_expected = 1.70833337307;               // 速度误差

    // 预瞄点
    double preview_station_error_expected = 
        0.91472222328000008 - station_reference_expected;      // 预瞄路径点的纵向距离
    double preview_speed_reference_expected = 2.00277781487;   // 预瞄速度
    double preview_speed_error_expected = 2.00277781487;       // 预瞄速度误差
    double preview_acceleration_reference_expected = 0.405916936513; // 预瞄加速度参考

    double tolerance_1 = 1e-3;
    double tolerance_2 = 2e-2;
    double now_timestamp_ = 0.0;

    // std::shared_ptr<DependencyInjector> injector_;

    const char data_path[100] = "../../controller/lon_based_pid_controller/longitudinal_controller_test/";
};
}  // namespace control