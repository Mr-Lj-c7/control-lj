#ifndef CONNECT_LAT_CONTROLLER_TEST_H
#define CONNECT_LAT_CONTROLLER_TEST_H

#include "lat_controller_test.h"
#include <fstream>
#include <json.h>

using LocalizationPb = control::common_msg::LocalizationEstimate;
using ChassisPb = control::common_msg::Chassis;  
using PlanningTrajectoryPb = control::common_msg::ADCTrajectory;
using control::common::VehicleStateProvider;

#define STATUS nullptr
#define relativate_time_ 0

namespace control{
class ConnectLatControllerTest : public LatControllerTest {
public:
ConnectLatControllerTest() {
    timestamp_ = GetSystemTimeSeconds();
}
~ConnectLatControllerTest() = default;

// 定位
LocalizationPb LoadLocalizationPb(const std::string& filename) const;
// 规划
PlanningTrajectoryPb LoadPlanningTrajectoryPb(const std::string& filename) const;
// 底盘
ChassisPb LoadChassisPb(const std::string& filename) const;

bool CustomExpectedResultWithMessage(const std::string& description,
                                    const double actual_value,
                                    const double expected_value,
                                    const double tolerance) const;
char* Run(); 


protected:
    // 期望值
    double theta_error_expected = -0.03549;                 // 航向误差
    double theta_error_dot_expected = 0.0044552856731;      // 航向误差率
    double d_error_expected = 1.30917375441;                // 横向误差
    double d_error_dot_expected = 0.0;                      // 横向误差率
    double matched_theta_expected = -1.81266;               // 匹配的航向角
    double matched_kappa_expected = -0.00237307;            // 匹配的曲率

    double tolerance = 1e-3;
    double timestamp_ = 0.0;

    std::shared_ptr<DependencyInjector> injector_;
    const char data_path_[100] = "../../controller/lat_based_lqr_controller/lateral_controller_test/";
};


}  // namespace control

#endif