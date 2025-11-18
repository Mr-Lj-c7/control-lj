#ifndef CHECK_PIT_H
#define CHECK_PIT_H 

#include "common_msg/control_msgs/control_cmd_msg.h"
#include "common/configs/vehicle_config_helper.h"
#include "controller/lon_based_pid_controller/conf/lon_based_pid_controller_conf.h"
#include <utility>
#include <chrono>

namespace control { 

// 洼地区域判断
/**
 * @brief 洼地区域判断
 * @param[in] debug
 * @param[in] conf
 * @param[in] speed 当前车速
 * @param[in] replan 重规划标记
 * @return true - 车在洼地
 */
class CheckPit { 
public:
    static bool CheckInPit(common_msg::SimpleLongitudinalDebug *debug,
                            const LonBasedPidControllerConf* conf,
                            double speed,
                            bool replan);
protected:
    // 系统时间
    static double GetSystemTimeSeconds();
private:
};

}  // control
#endif