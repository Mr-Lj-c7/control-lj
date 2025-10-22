# include "common_msg/config_msgs/vehicle_config_msg.h"
# include <iostream>
#include <fstream>
# include "json.h"

namespace control {
namespace common {

class VehicleConfigHelper
{
private:
    static bool is_init_;
    static common_msg::VehicleConfig vehicle_config_;  
    static const std::string FLAGS_vehicle_config_path;

public:
    VehicleConfigHelper() = default;
    ~VehicleConfigHelper() = default;

    // 静态成员函数只能通过类名或作用域调用
    static void GetVehicleParams(const common_msg::VehicleConfig &vehicle_config_);
    static void Init(const std::string &config_file_);
    static void Init();
    static const common_msg::VehicleConfig &GetConfig();

    double MinSafeTurnRadius() const;
};
// inline const std::string VehicleConfigHelper::FLAGS_vehicle_config_path = "common/data/vehicle_param.json";
}  // control    
}  // common