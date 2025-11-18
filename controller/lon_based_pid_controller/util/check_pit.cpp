#include "check_pit.h"

namespace control { 

// 洼地区域判断
bool CheckPit::CheckInPit(common_msg::SimpleLongitudinalDebug *debug,
                        const LonBasedPidControllerConf* conf,
                        double speed,
                        bool replan) {
  static std::pair<double, int> replan_count(0, 0);  // 重规划计数    
  double now = GetSystemTimeSeconds();
  if (now - replan_count.first > conf->pit_replan_check_time) {
    replan_count.first = 0.0;
    replan_count.second = 0;
  }
  if (replan) {
    // 初次计数
    if (now - replan_count.first > conf->pit_replan_check_time) {
        replan_count.first = now;
        replan_count.second = 1;
    } else {
        replan_count.first = now;
        replan_count.second++;
    }
  }
  // 重规划频率+车速判断车辆是否处于洼地
  const auto& vehicle_param = 
    common::VehicleConfigHelper::GetConfig().vehicle_param;
  if (replan_count.second >= conf->pit_replan_check_count &&
    abs(speed) < vehicle_param.max_abs_speed_when_stopped) {
        return true;
    } else {
        return false;
    }
}

// 系统时间
double CheckPit::GetSystemTimeSeconds() {
    return std::chrono::duration<double>
            (std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // control