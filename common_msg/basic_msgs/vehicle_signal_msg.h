#ifndef VEHICLE_SIGNAL_MSG_H
#define VEHICLE_SIGNAL_MSG_H

#include <string>
#include <cmath>
#include <vector>

namespace control {
namespace common_msg {

typedef struct VEHICLE_SIGNAL { 
    enum TurnSignal {
        TURN_NONE = 0,
        TURN_LEFT = 1,
        TURN_RIGHT = 2,
        TURN_HAZARD_WARNING = 3
    };
    TurnSignal turn_signal;
    // 灯光启动指令
    bool high_beam;
    bool low_beam;
    bool horn;
    bool emergency_light;  // 应急灯
} VehicleSignal;

}  // namespace common_msg
}  // namespace control

#endif