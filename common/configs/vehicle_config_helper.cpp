#include "vehicle_config_helper.h"

namespace control {
namespace common {

common_msg::VehicleConfig VehicleConfigHelper::vehicle_config_;
bool VehicleConfigHelper::is_init_ = false;
// const std::string VehicleConfigHelper::FLAGS_vehicle_config_path = "common/data/vehicle_param.json";

void VehicleConfigHelper::Init() { Init(FLAGS_vehicle_config_path); }

// void VehicleConfigHelper::Init(const std::string &config_file) {
//     common_msg::VehicleConfig params;
//     /* json 内容 */
//     GetVehicleParams(params);
//     is_init_ = true;
// }

void VehicleConfigHelper::Init(const std::string &config_file) {
    common_msg::VehicleConfig params;
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "[VehicleConfigHelper]:Failed to open file: " 
          << config_file << std::endl;
        return;
    }
    Json::Value root_;
    Json::Reader reader_;
    if (!reader_.parse(file, root_)) {
        std::cerr << "[VehicleConfigHelper]:Failed to parse JSON file: " 
        << config_file << std::endl;
        return;
    }
    if (root_.isMember("vehicle_param")) {
        std::string brand_ = root_["vehicle_param"]["brand"].asString();
        if (brand_ == "LINCOLN_MKZ"){
            params.vehicle_param.brand = common_msg::LINCOLN_MKZ;
        }
        else if (brand_ == "GEM"){
            params.vehicle_param.brand = common_msg::GEM;
        }
        else if (brand_ == "LEXUS"){
            params.vehicle_param.brand = common_msg::LEXUS;
        }
        else if (brand_ == "TRANSIT"){
            params.vehicle_param.brand = common_msg::TRANSIT;
        }
        else if (brand_ == "GE3"){
            params.vehicle_param.brand = common_msg::GE3;
        }
        else if (brand_ == "WEY"){
            params.vehicle_param.brand = common_msg::WEY;
        }
        else if (brand_ == "ZHONGYUN"){
            params.vehicle_param.brand = common_msg::ZHONGYUN;
        }
        else if (brand_ == "CH"){
            params.vehicle_param.brand = common_msg::CH;
        }
        else if (brand_ == "DKIT"){
            params.vehicle_param.brand = common_msg::DKIT;
        }
        else if (brand_ == "NEOLIX"){
            params.vehicle_param.brand = common_msg::NEOLIX;
        }
        else{
            params.vehicle_param.brand = common_msg::OTHER;
        }
        params.vehicle_param.vehicle_id.other_unique_id = 
            root_["vehicle_param"]["vehicle_id"]["other_unique_id"].asString();
        params.vehicle_param.front_edge_to_center = 
            root_["vehicle_param"]["front_edge_to_center"].asDouble();
        params.vehicle_param.back_edge_to_center = 
            root_["vehicle_param"]["back_edge_to_center"].asDouble();
        params.vehicle_param.left_edge_to_center = 
            root_["vehicle_param"]["left_edge_to_center"].asDouble();
        params.vehicle_param.right_edge_to_center = 
            root_["vehicle_param"]["right_edge_to_center"].asDouble();
        params.vehicle_param.length = root_["vehicle_param"]["length"].asDouble();
        params.vehicle_param.width = root_["vehicle_param"]["width"].asDouble();
        params.vehicle_param.height = root_["vehicle_param"]["height"].asDouble();
        params.vehicle_param.min_turn_radius = 
            root_["vehicle_param"]["min_turn_radius"].asDouble();
        params.vehicle_param.max_acceleration = 
            root_["vehicle_param"]["max_acceleration"].asDouble();
        params.vehicle_param.max_deceleration = 
            root_["vehicle_param"]["max_deceleration"].asDouble();
        params.vehicle_param.max_steer_angle = 
            root_["vehicle_param"]["max_steer_angle"].asDouble();
        params.vehicle_param.max_steer_angle_rate = 
            root_["vehicle_param"]["max_steer_angle_rate"].asDouble();
        params.vehicle_param.steer_ratio = 
            root_["vehicle_param"]["steer_ratio"].asDouble();
        params.vehicle_param.wheel_base = 
            root_["vehicle_param"]["wheel_base"].asDouble();
        params.vehicle_param.wheel_rolling_radius = 
            root_["vehicle_param"]["wheel_rolling_radius"].asDouble();
        params.vehicle_param.max_abs_speed_when_stopped = 
            root_["vehicle_param"]["max_abs_speed_when_stopped"].asDouble();
        params.vehicle_param.brake_deadzone = 
            root_["vehicle_param"]["brake_deadzone"].asDouble();
        params.vehicle_param.throttle_deadzone = 
            root_["vehicle_param"]["throttle_deadzone"].asDouble();
    }
    GetVehicleParams(params);
    is_init_ = true;
    return;
}

void VehicleConfigHelper::GetVehicleParams(const common_msg::VehicleConfig &vehicle_config) { 
    
    if (vehicle_config.vehicle_param.length != 0.0 && 
        vehicle_config.vehicle_param.width  != 0.0 && 
        vehicle_config.vehicle_param.height != 0.0) {
            vehicle_config_ = vehicle_config;
            std::cerr << "[VehicleConfigHelper]:Vehicle Params Init success!" 
              << std::endl;
            return;
    } else {
        std::cerr << "[VehicleConfigHelper]:Vehicle Params Init failed!" 
          << std::endl;
        return;
    }
}

const common_msg::VehicleConfig &VehicleConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}

// 计算最小安全转弯半径
double VehicleConfigHelper::MinSafeTurnRadius() const { 
    const auto &param_ = vehicle_config_;
    double lat_edge_to_center = std::max(param_.vehicle_param.left_edge_to_center, 
        param_.vehicle_param.right_edge_to_center);
    double lon_edge_to_center = std::max(param_.vehicle_param.front_edge_to_center, 
        param_.vehicle_param.back_edge_to_center);
    return std::sqrt((lat_edge_to_center + param_.vehicle_param.min_turn_radius) * 
        (lat_edge_to_center + param_.vehicle_param.min_turn_radius)
        + (lon_edge_to_center * lon_edge_to_center));
}

}  // cotrol
}  // common