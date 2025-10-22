#ifndef LON_CONTROLLER_TEST_H
#define LON_CONTROLLER_TEST_H

#include <chrono>
#include <memory>
#include "control_component/controller_task_base/common/trajectory_analyzer.h"
#include "control_component/controller_task_base/common/dependency_injector.h"
#include "controller/lon_based_pid_controller/conf/lon_based_pid_controller_conf.h"
#include "common_msg/control_msgs/control_cmd_msg.h"

namespace control {
class LonControllerTest {
public:
    LonControllerTest();
    ~LonControllerTest();

    double GetSystemTimeSeconds() const;

    void SetInjector(std::shared_ptr<DependencyInjector> injector) {
        injector_ = injector;
    }

    void ComputeLongitudinalErrors(TrajectoryAnalyzer *trajectory,
                                 const double preview_time, const double ts,
                                 common_msg::SimpleLongitudinalDebug *debug);

    LonBasedPidControllerConf lon_based_pidcontroller_conf_;  // 纵向控制器配置

protected:
    std::shared_ptr<DependencyInjector> injector_;  

    double previous_acceleration_reference_ = 0.0;  // 前一次加速度参考
    double previous_acceleration_ = 0.0;            // 前一次加速度
    double reference_spd_ = 0.0;                    // 参考速度

};

}

#endif