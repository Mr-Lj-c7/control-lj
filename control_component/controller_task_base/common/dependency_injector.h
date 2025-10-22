#ifndef DEPENDENCY_INJECTOR_H
#define DEPENDENCY_INJECTOR_H 

#include "common/vehicle_state/vehicle_state_provider.h"

namespace control {

class DependencyInjector {
public:
    DependencyInjector() = default;
    ~DependencyInjector() = default;

    control::common::VehicleStateProvider* vehicle_state() {
        return &vehicle_state_;
    }
private:
    control::common::VehicleStateProvider vehicle_state_;
};

} // control


#endif // DEPENDENCY_INJECTOR_H