#ifndef GAIN_SCHEDULER_CONF_H
#define GAIN_SCHEDULER_CONF_H 

#include <string>
#include <cmath>
#include <vector>

namespace control {
namespace common_msg {

typedef struct GAIN_SCHEDULER_INFO { 
    double speed;
    double ratio;
} GainSchedulerInfo;


typedef struct GAIN_SCHEDULER_CONF { 
    double pitch;
    double ratio;
} GainSlopeSchedulerInfo;

// 增益调度器
typedef struct GAIN_SCHEDULER { 
    std::vector<GainSchedulerInfo> scheduler;
    GainSlopeSchedulerInfo pitch_scheduler;

} GainScheduler;


}  // namespace common_msg
}  // namespace control


#endif