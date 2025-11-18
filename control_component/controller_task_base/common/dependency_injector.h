#ifndef DEPENDENCY_INJECTOR_H
#define DEPENDENCY_INJECTOR_H 

#include "common/vehicle_state/vehicle_state_provider.h"
#include "common_msg/control_msgs/control_cmd_msg.h"

namespace control {

class DependencyInjector {
public:
    DependencyInjector() = default;
    ~DependencyInjector() = default;

    control::common::VehicleStateProvider* vehicle_state() {
        return &vehicle_state_;  // 车辆状态
    }
    const common_msg::SimpleLongitudinalDebug* Get_pervious_lon_debug_info() const {
        return &lon_debug_;  // 前一帧的纵向控制信息
    }

    void Set_pervious_control_command(common_msg::ControlCommand* control_command) {
        std::cerr << "[DependencyInjector]: Get the new control command: " <<
            control_command->debug.simple_lon_debug.current_speed << std::endl;
        lon_debug_ = control_command->debug.simple_lon_debug;
    }

protected:
private:
    control::common::VehicleStateProvider vehicle_state_;
    common_msg::SimpleLongitudinalDebug lon_debug_;
};

} // control


#endif // DEPENDENCY_INJECTOR_H