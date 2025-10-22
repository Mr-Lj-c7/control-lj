#include "connect_lat_controller_test.h"
// #include "lat_controller_test.h"
// #include <fstream>
// #include <json.h>

// using LocalizationPb = control::common_msg::LocalizationEstimate;
// using ChassisPb = control::common_msg::Chassis;  
// using PlanningTrajectoryPb = control::common_msg::ADCTrajectory;
// using control::common::VehicleStateProvider;


    // // 定位
    // LocalizationPb LoadLocalizationPb(const std::string& filename) {
    //     LocalizationPb localization_pb;
    //     std::ifstream file(filename);
    //     if (!file.is_open()) {
    //         std::cerr << "Failed to open file: " << filename << std::endl;
    //         return localization_pb;
    //     }
    //     Json::Value root_1;
    //     Json::Reader reader_1;
    //     if (!(reader_1.parse(file, root_1))) {
    //         std::cerr << "Failed to parse JSON data in 1_localization file." << std::endl;
    //         return localization_pb;
    //     }
    //     if (root_1.isMember("header")) {
    //         localization_pb.header.timestamp_sec = root_1["header"]["timestamp_sec"].asDouble();
    //         localization_pb.header.module_name = root_1["header"]["module_name"].asString();
    //         localization_pb.header.sequence_num = root_1["header"]["sequence_num"].asUInt();
    //     }
    //     if (root_1.isMember("pose"))
    //     {
    //         Json::Value pose_ = root_1["pose"];
    //         Json::Value position_ = pose_["position"];
    //         localization_pb.pose.position.x = position_["x"].asDouble();
    //         localization_pb.pose.position.y = position_["y"].asDouble();
    //         localization_pb.pose.position.z = position_["z"].asDouble();  
    //         Json::Value orientation_ = pose_["orientation"];
    //         localization_pb.pose.orientation.qx = orientation_["qx"].asDouble();
    //         localization_pb.pose.orientation.qy = orientation_["qy"].asDouble();  
    //         localization_pb.pose.orientation.qz = orientation_["qz"].asDouble();
    //         localization_pb.pose.orientation.qw = orientation_["qw"].asDouble();
    //         Json::Value linear_velocity_ = pose_["linear_velocity"];
    //         localization_pb.pose.linear_velocity.x = linear_velocity_["x"].asDouble();
    //         localization_pb.pose.linear_velocity.y = linear_velocity_["y"].asDouble();
    //         localization_pb.pose.linear_velocity.z = linear_velocity_["z"].asDouble();  
    //         Json::Value linear_acceleration_ = pose_["linear_acceleration"];
    //         localization_pb.pose.linear_acceleration.x = linear_acceleration_["x"].asDouble();
    //         localization_pb.pose.linear_acceleration.y = linear_acceleration_["y"].asDouble();
    //         localization_pb.pose.linear_acceleration.z = linear_acceleration_["z"].asDouble();
    //         Json::Value angular_velocity_ = pose_["angular_velocity"];
    //         localization_pb.pose.angular_velocity.x = angular_velocity_["x"].asDouble();
    //         localization_pb.pose.angular_velocity.y = angular_velocity_["y"].asDouble();
    //         localization_pb.pose.angular_velocity.z = angular_velocity_["z"].asDouble();
    //     }
    //     return localization_pb;
    // }

    // // 底盘
    // ChassisPb LoadChassisPb(const std::string& filename) {
    //     ChassisPb chassis_pb;
    //     std::ifstream file(filename);
    //     if (!file.is_open()) {
    //         std::cerr << "Failed to open file: " << filename << std::endl;
    //         return chassis_pb;
    //     }
    //     Json::Value root_2;
    //     Json::Reader reader_2;
    //     if (!reader_2.parse(file, root_2)) {
    //         std::cerr << "Failed to parse JSON data in 1_chassis file." << std::endl;
    //         return chassis_pb;
    //     }
    //     // 车辆状态
    //     chassis_pb.engine_started = root_2["engine_started"].asBool();
    //     chassis_pb.engine_rpm = root_2["engine_rpm"].asFloat();
    //     chassis_pb.speed_mps = root_2["speed_mps"].asFloat();
    //     chassis_pb.odometer_m = root_2["odometer_m"].asFloat();
    //     chassis_pb.fule_range_m = root_2["fule_range_m"].asFloat();
    //     // 控制器状态
    //     chassis_pb.throttle_percentage = root_2["throttle_percentage"].asFloat();
    //     chassis_pb.brake_percentage = root_2["brake_percentage"].asFloat();
    //     chassis_pb.steering_percentage = root_2["steering_percentage"].asFloat();
    //     chassis_pb.steering_torque_nm = root_2["steering_torque_nm"].asFloat();
    //     chassis_pb.parking_brake = root_2["parking_brake"].asBool();
    //     // driving_mode
    //     std::string driving_mode_str = root_2["driving_mode"].asString();
    //     if (driving_mode_str == "COMPLETE_MANUAL") {
    //         chassis_pb.driving_mode = ChassisPb::COMPLETE_MANUAL;
    //     } else if (driving_mode_str == "COMPLETE_AUTO_DRIVE") {
    //         chassis_pb.driving_mode = ChassisPb::COMPLETE_AUTO_DRIVE;
    //     } else if (driving_mode_str == "AUTO_STEER_ONLY") {
    //         chassis_pb.driving_mode = ChassisPb::AUTO_STEER_ONLY;
    //     } else if (driving_mode_str == "AUTO_SPEED_ONLY") {
    //         chassis_pb.driving_mode = ChassisPb::AUTO_SPEED_ONLY;
    //     } else if (driving_mode_str == "EMERGENCY_MODE") {
    //         chassis_pb.driving_mode = ChassisPb::EMERGENCY_MODE;
    //     } else {
    //         chassis_pb.driving_mode = ChassisPb::COMPLETE_MANUAL;
    //     }
    //     // 错误码
    //     std::string error_code_str = root_2["error_code"].asString();
    //     if (error_code_str == "NO_ERROR") {
    //         chassis_pb.error_code = ChassisPb::NO_ERROR;
    //     } else if (error_code_str == "CMD_NOT_IN_PERIOD") {
    //         chassis_pb.error_code = ChassisPb::CMD_NOT_IN_PERIOD;
    //     } else if (error_code_str == "CHASSIS_ERROR") {
    //         chassis_pb.error_code = ChassisPb::CHASSIS_ERROR;
    //     } else if (error_code_str == "CHASSIS_ERROR_ON_STEER") {
    //         chassis_pb.error_code = ChassisPb::CHASSIS_ERROR_ON_STEER;
    //     } else if (error_code_str == "CHASSIS_ERROR_ON_BRAKE") {
    //         chassis_pb.error_code = ChassisPb::CHASSIS_ERROR_ON_BRAKE;
    //     } else if (error_code_str == "CHASSIS_ERROR_ON_THROTTLE") {
    //         chassis_pb.error_code = ChassisPb::CHASSIS_ERROR_ON_THROTTLE;
    //     } else if (error_code_str == "CHASSIS_ERROR_ON_GEAR") {
    //         chassis_pb.error_code = ChassisPb::CHASSIS_ERROR_ON_GEAR;
    //     } else if (error_code_str == "CHASSIS_CAN_LOST") {
    //         chassis_pb.error_code = ChassisPb::CHASSIS_CAN_LOST;
    //     } else if (error_code_str == "MANUAL_INTERVENTION") {
    //         chassis_pb.error_code = ChassisPb::MANUAL_INTERVENTION;
    //     } else if (error_code_str == "CHASSIS_CAN_NOT_IN_PERIOD") {
    //         chassis_pb.error_code = ChassisPb::CHASSIS_CAN_NOT_IN_PERIOD;
    //     } else if (error_code_str == "UNKNOWN_ERROR") {
    //         chassis_pb.error_code = ChassisPb::UNKNOWN_ERROR;
    //     } else {
    //         chassis_pb.error_code = ChassisPb::NO_ERROR;
    //     }
    //     // 档位状态
    //     std::string gear_location_str = root_2["gear_location"].asString();
    //     if (gear_location_str == "GEAR_NEUTRAL") {
    //         chassis_pb.gear_location = ChassisPb::GEAR_NEUTRAL;
    //     } else if (gear_location_str == "GEAR_DRIVE") {
    //         chassis_pb.gear_location = ChassisPb::GEAR_DRIVE;
    //     } else if (gear_location_str == "GEAR_REVERSE") {
    //         chassis_pb.gear_location = ChassisPb::GEAR_REVERSE;
    //     } else if (gear_location_str == "GEAR_PARKING") {
    //         chassis_pb.gear_location = ChassisPb::GEAR_PARKING;
    //     } else if (gear_location_str == "GEAR_LOW") {
    //         chassis_pb.gear_location = ChassisPb::GEAR_LOW;
    //     } else if (gear_location_str == "GEAR_INVALID") {
    //         chassis_pb.gear_location = ChassisPb::GEAR_INVALID;
    //     } else if (gear_location_str == "GEAR_NONE") {
    //         chassis_pb.gear_location = ChassisPb::GEAR_NONE;
    //     } else {
    //         chassis_pb.gear_location = ChassisPb::GEAR_NEUTRAL; 
    //     }
    //     chassis_pb.steering_timestamp = root_2["steering_timestamp"].asDouble();
    //     // header
    //     if (root_2.isMember("header")) {
    //         chassis_pb.header.timestamp_sec = root_2["header"]["timestamp_sec"].asDouble();
    //         chassis_pb.header.module_name = root_2["header"]["module_name"].asString();
    //         chassis_pb.header.sequence_num = root_2["header"]["sequence_num"].asUInt();
    //     }
    //     // 灯光
    //     if (root_2.isMember("signal")) {
    //         // 转向灯光
    //         std::string turn_signal_str = root_2["signal"]["turn_signal"].asString();
    //         if (turn_signal_str == "TURN_LEFT") {
    //             chassis_pb.signal.turn_signal = control::common_msg::VehicleSignal::TURN_LEFT;
    //         } else if (turn_signal_str == "TURN_RIGHT") {
    //             chassis_pb.signal.turn_signal = control::common_msg::VehicleSignal::TURN_RIGHT;
    //         } else if (turn_signal_str == "TURN_HAZARD_WARNING") {
    //             chassis_pb.signal.turn_signal = control::common_msg::VehicleSignal::TURN_HAZARD_WARNING;
    //         } else {
    //             chassis_pb.signal.turn_signal = control::common_msg::VehicleSignal::TURN_NONE;
    //         }
    //         chassis_pb.signal.high_beam = root_2["signal"]["high_beam"].asBool();
    //         chassis_pb.signal.low_beam = root_2["signal"]["low_beam"].asBool();
    //         chassis_pb.signal.horn = root_2["signal"]["horn"].asBool();
    //         chassis_pb.signal.emergency_light = root_2["signal"]["emergency_light"].asBool();
    //     }
    //     return chassis_pb;
    // }

    // // 规划
    // PlanningTrajectoryPb LoadPlanningTrajectoryPb(const std::string& filename) {
    //     PlanningTrajectoryPb planning_trajectory_pb;
    //     std::ifstream file(filename);
    //     if (!file.is_open()) {
    //         std::cout << "Failed to open file: " << filename << std::endl;
    //         return planning_trajectory_pb;
    //     }
    //     Json::Value root_3;
    //     Json::Reader reader_3;
    //     if (!reader_3.parse(file, root_3)) {
    //         std::cerr << "Failed to parse JSON data in 1_planning file." << std::endl;
    //         return planning_trajectory_pb;
    //     }
    //     if (root_3.isMember("header")) {
    //         planning_trajectory_pb.header.timestamp_sec = root_3["header"]["timestamp_sec"].asDouble();
    //         planning_trajectory_pb.header.module_name = root_3["header"]["module_name"].asString();
    //         planning_trajectory_pb.header.sequence_num = root_3["header"]["sequence_num"].asUInt();
    //     }
    //     planning_trajectory_pb.total_path_length = root_3["total_path_length"].asDouble();
    //     planning_trajectory_pb.total_path_time = root_3["total_path_time"].asDouble();
    //     if (root_3.isMember("estop")) {
    //         planning_trajectory_pb.estop.is_estop = root_3["estop"]["is_estop"].asBool();
    //     }
    //     std::string gear_str = root_3["gear"].asString();
    //     if (gear_str == "GEAR_NEUTRAL") {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_NEUTRAL;
    //     } else if (gear_str == "GEAR_DRIVE") {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_DRIVE;
    //     } else if (gear_str == "GEAR_REVERSE") {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_REVERSE;
    //     } else if (gear_str == "GEAR_PARKING") {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_PARKING;
    //     } else if (gear_str == "GEAR_LOW") {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_LOW;
    //     } else if (gear_str == "GEAR_INVALID") {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_INVALID;
    //     } else if (gear_str == "GEAR_NONE") {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_NONE;
    //     } else {
    //         planning_trajectory_pb.gear = ChassisPb::GEAR_NEUTRAL; 
    //     }
    //     if (root_3.isMember("trajectory_point")) {
    //         uint32_t points_ = root_3["trajectory_point"].size();
    //         // std::cout << "Number of trajectory points: " << points_ << std::endl;
    //         planning_trajectory_pb.trajectory_point.resize(points_);  // 分配内存
    //         for (size_t i = 0; i < points_; ++i) {
    //             planning_trajectory_pb.trajectory_point[i].v = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["v"].asDouble(); 
    //             planning_trajectory_pb.trajectory_point[i].a = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["a"].asDouble();  
    //             planning_trajectory_pb.trajectory_point[i].relative_time = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["relative_time"].asDouble();
    //             planning_trajectory_pb.trajectory_point[i].path_point.x = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["path_point"]["x"].asDouble();
    //             planning_trajectory_pb.trajectory_point[i].path_point.y = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["path_point"]["y"].asDouble();
    //             planning_trajectory_pb.trajectory_point[i].path_point.z = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["path_point"]["z"].asDouble();
    //             planning_trajectory_pb.trajectory_point[i].path_point.theta = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["path_point"]["theta"].asDouble();
    //             planning_trajectory_pb.trajectory_point[i].path_point.kappa = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["path_point"]["kappa"].asDouble();
    //             planning_trajectory_pb.trajectory_point[i].path_point.s = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["path_point"]["s"].asDouble();
    //             planning_trajectory_pb.trajectory_point[i].path_point.dkappa = 
    //               root_3["trajectory_point"][static_cast<int>(i)]["path_point"]["dkappa"].asDouble();
    //         }  
    //     }
    //     std::cout<<"[planning_trajectory_pb.trajectory_point.size]:" 
    //              << planning_trajectory_pb.trajectory_point.size() << std::endl;
    //     return planning_trajectory_pb;
    // }

    // // 检核结果
    // bool CustomExpectedResultWithMessage(
    //     const std::string& description,
    //     const double actual_value,
    //     const double expected_value,
    //     const double tolerance) {
    //     double abs_diff = std::abs(actual_value) - std::abs(expected_value);
    //     bool result = abs_diff <= tolerance;
    //     if (!result) {
    //         std::cerr << "FAIL: "         << description
    //                   << " - Expected: "  << expected_value
    //                   << ", Actual: "     << actual_value
    //                   << ", Difference: " << abs_diff
    //                   << ", Tolerance: "  << tolerance << std::endl;
    //     } else {
    //         std::cout << " PASS: " << description << std::endl;
    //     }
    //     return result;
    // }

int main(char** argv, char** argc) {
    // control::LatControllerTest lat_controller_test;
    // // 车辆状态信息指针对象
    // double timestamp_ = 0.0; 
    // std::shared_ptr<control::DependencyInjector> 
    //    injector_(new control::DependencyInjector());

    // // 读取测试数据
    // auto localization_pb = LoadLocalizationPb(
    //   "../../controller/lat_based_lqr_controller/lateral_controller_test/1_localization.json");

    // auto chassis_pb = LoadChassisPb(
    //   "../../controller/lat_based_lqr_controller/lateral_controller_test/1_chassis.json");

    // auto vehicle_state = injector_->vehicle_state();
    // vehicle_state->Update(localization_pb, chassis_pb);

    // lat_controller_test.SetInjector(injector_);
    
    // auto planning_trajectory_pb = LoadPlanningTrajectoryPb(
    //   "../../controller/lat_based_lqr_controller/lateral_controller_test/1_planning.json");
    // control::TrajectoryAnalyzer trajectory_analyzer(
    //     &planning_trajectory_pb);
    // control::common_msg::SimpleLateralDebug debug;
    // lat_controller_test.ComputeLateralErrors(
    //     vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
    //     vehicle_state->linear_velocity(), vehicle_state->angular_velocity(),
    //     vehicle_state->linear_acceleration(), trajectory_analyzer,
    //     &debug, &chassis_pb);
    // // 期望值
    // double theta_error_expected = -0.03549;                 // 航向误差
    // double theta_error_dot_expected = 0.0044552856731;      // 航向误差率
    // double d_error_expected = 1.30917375441;                // 横向误差
    // double d_error_dot_expected = 0.0;                      // 横向误差率
    // double matched_theta_expected = -1.81266;               // 匹配的航向角
    // double matched_kappa_expected = -0.00237307;            // 匹配的曲率

    // double tolerance = 1e-3;

    // // 检核结果
    // bool T_1 = CustomExpectedResultWithMessage("HeadingError", debug.heading_error,
    //                                            theta_error_expected, tolerance);
    // bool T_2 = CustomExpectedResultWithMessage("HeadingErrorRate", debug.heading_error_rate,
    //                                            theta_error_dot_expected, tolerance);
    // bool T_3 = CustomExpectedResultWithMessage("LateralError", debug.lateral_error,
    //                                            d_error_expected, tolerance);
    // bool T_4 = CustomExpectedResultWithMessage("LateralErrorRate", debug.lateral_error_rate,
    //                                            d_error_dot_expected, tolerance);
    // bool T_5 = CustomExpectedResultWithMessage("RefError", debug.ref_heading,
    //                                            matched_theta_expected, tolerance);
    // bool T_6 = CustomExpectedResultWithMessage("Curvature", debug.curvature,
    //                                            matched_kappa_expected, tolerance);


    // // 输出结果
    // bool all_pass = T_1 && T_2 && T_3 && T_4 && T_5 && T_6;
    // std::cout << "All tests passed: " << (all_pass ? "YES" : "NO") << std::endl;

    control::ConnectLatControllerTest lat_controller_test;
    lat_controller_test.Run();
    return 0;
}