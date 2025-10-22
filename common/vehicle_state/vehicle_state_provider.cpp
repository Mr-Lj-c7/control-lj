#include "vehicle_state_provider.h"
#include <cmath>

namespace control {
namespace common {

    void VehicleStateProvider::Update(
        const common_msg::LocalizationEstimate& localization,
        const common_msg::Chassis& chassis){
        original_localization_ = localization;
        if (!ConstructExceptLinearVelocity(localization)) {
            std::cerr << "Fail to update because ConstructExpectLinearVelocity error." << std::endl;
            return;
        } 
        // 时间戳  
        if (localization.measurement_time != 0.0) {
            vehicle_state_.timestamp = localization.measurement_time;
        } else if (localization.header.timestamp_sec != 0.0) {
            vehicle_state_.timestamp = localization.header.timestamp_sec;
        } else if (chassis.header.timestamp_sec != 0.0) {
            std::cerr << "Unable to use location timestamp for vehicle state, \
                      Use chassis time instead." << std::endl;
            vehicle_state_.timestamp = chassis.header.timestamp_sec;
        }
        // 车辆档位
        if (chassis.gear_location != common_msg::Chassis::GEAR_NONE) {
            vehicle_state_.gear = chassis.gear_location;
        } else {
            std::cerr << "gear location is GEAR_NONE." << std::endl;
        }

        // 车辆速度
        if (chassis.speed_mps != 0.0) {
            vehicle_state_.linear_velocity = chassis.speed_mps;
            if (!FLAGS_reverse_heading_vehicle_state &&
                vehicle_state_.gear == common_msg::Chassis::GEAR_REVERSE) {
            vehicle_state_.linear_velocity = -vehicle_state_.linear_velocity;
            }
        }

        // 方向盘转角
        if (chassis.steering_percentage != 0.0) {
            vehicle_state_.steering_percentage = chassis.steering_percentage;
        }

        // 车辆当前道路曲率
        static constexpr double kEpsilon = 0.1;
        if (std::abs(vehicle_state_.linear_velocity) < kEpsilon) {
            vehicle_state_.kappa = 0.0;
        } else {
            vehicle_state_.kappa = (vehicle_state_.angular_velocity /
                                    vehicle_state_.linear_velocity);
        }

        // 驾驶模式
        vehicle_state_.driving_mode = chassis.driving_mode;
        std::cout << "Update vehicle state successfully." << std::endl;
        return;
    }

    bool VehicleStateProvider::ConstructExceptLinearVelocity(
        const common_msg::LocalizationEstimate &localization) {
        if (localization.pose.position.x == 0.0 &&
            localization.pose.position.y == 0.0 &&
            localization.pose.position.z == 0.0) {
            std::cerr << "Invalid localization input." << std::endl;
            return false;
        }

        // skip localization update when it is in use_navigation_mode.
        if (FLAGS_use_navigation_mode) {
            std::cerr << "Skip localization update when it is in use_navigation_mode." 
                      << std::endl;
            return true;
        }
        // 车辆定位位姿
        vehicle_state_.pose = localization.pose;
        if (localization.pose.position.x != 0.0 &&
            localization.pose.position.y != 0.0 &&
            localization.pose.position.z != 0.0) {
            vehicle_state_.x = localization.pose.position.x;
            vehicle_state_.y = localization.pose.position.y;
            vehicle_state_.z = localization.pose.position.z;
        }

        const auto &orientation = localization.pose.orientation;
        // 车辆航向角
        if (localization.pose.heading != 0.0) {
            vehicle_state_.heading = localization.pose.heading;
        } else {
            vehicle_state_.heading = (
                common::QuaternionToHeading(orientation.qw, orientation.qx,
                                            orientation.qy, orientation.qz));
       }
        // 地图参考系统一致化
        if (FLAGS_enable_map_reference_unify) {
            if (localization.pose.angular_velocity_vrf.x == 0.0 &&
                localization.pose.angular_velocity_vrf.y == 0.0 &&
                localization.pose.angular_velocity_vrf.z == 0.0) {
            std::cerr << "localization.pose().has_angular_velocity_vrf() must be true "
                        "when FLAGS_enable_map_reference_unify is true." 
                      << std::endl;
            return false;
            }
            vehicle_state_.angular_velocity = (  // 航向角速度
                localization.pose.angular_velocity_vrf.z);

            if (localization.pose.linear_acceleration_vrf.x == 0.0 &&
                localization.pose.linear_acceleration_vrf.y == 0.0 &&
                localization.pose.linear_acceleration_vrf.z == 0.0) {
            std::cerr << "localization.pose().has_linear_acceleration_vrf() must be "
                        "true when FLAGS_enable_map_reference_unify is true." 
                      << std::endl;
            return false;
            }
            vehicle_state_.linear_acceleration = (  // 纵向加速度
                localization.pose.linear_acceleration_vrf.y);
        } else {
            if (localization.pose.angular_velocity.x == 0.0 &&
                localization.pose.angular_velocity.y == 0.0 &&
                localization.pose.angular_velocity.z == 0.0) {
            std::cerr << "localization.pose() has no angular velocity." 
                      << std::endl;
            return false;
            }
            vehicle_state_.angular_velocity = (  // 航向角速度
                localization.pose.angular_velocity.z);

            if (localization.pose.linear_acceleration.x == 0.0 &&
                localization.pose.linear_acceleration.y == 0.0 &&
                localization.pose.linear_acceleration.z == 0.0) {
            std::cerr << "localization.pose() has no linear acceleration." 
                      << std::endl;
            return false;
            }
            vehicle_state_.linear_acceleration = (  // 纵向加速度
                localization.pose.linear_acceleration.y);
        }
        // 车辆欧拉角
        if (localization.pose.euler_angles.x != 0.0 &&
            localization.pose.euler_angles.y != 0.0 && 
            localization.pose.euler_angles.z != 0.0) {
            vehicle_state_.roll  = localization.pose.euler_angles.y;
            vehicle_state_.pitch = localization.pose.euler_angles.x;
            vehicle_state_.yaw   = localization.pose.euler_angles.z;
        } else {
            common::EulerAnglesZXYd euler_angle(orientation.qw, orientation.qx,
                                                orientation.qy, orientation.qz);
            vehicle_state_.roll  = euler_angle.roll();
            vehicle_state_.pitch = euler_angle.pitch();
            vehicle_state_.yaw   = euler_angle.yaw();
        }
        return true;
    }

    // position: x, y, z
    double VehicleStateProvider::x() const {
        return vehicle_state_.x;
    }   

    double VehicleStateProvider::y() const {
        return vehicle_state_.y;
    }

    double VehicleStateProvider::z() const {
        return vehicle_state_.z;
    }

    // 欧拉角: roll, pitch, yaw
    double VehicleStateProvider::roll() const {
        return vehicle_state_.roll;
    }

    double VehicleStateProvider::pitch() const {
        return vehicle_state_.pitch;
    }

    double VehicleStateProvider::yaw() const {
        return vehicle_state_.yaw;
    }

    // 航向角
    double VehicleStateProvider::heading() const {
        return vehicle_state_.heading;
    }

    // 当前路径曲率
    double VehicleStateProvider::kappa() const {
        return vehicle_state_.kappa;
    }   

    // 角速度
    double VehicleStateProvider::angular_velocity() const {
        return vehicle_state_.angular_velocity;
    }

    // 线速度
    double VehicleStateProvider::linear_velocity() const {  
        return vehicle_state_.linear_velocity;
    }

    // 纵向加速度
    double VehicleStateProvider::linear_acceleration() const {
        return vehicle_state_.linear_acceleration;
    }

    // 车辆档位
    double VehicleStateProvider::gear() const {
        return vehicle_state_.gear;
    }

    // 方向盘转角
    double VehicleStateProvider::steering_percentage() const {
        return vehicle_state_.steering_percentage;
    }

    // 更新后的定位位姿
    const common_msg::Pose& VehicleStateProvider::pose() const {
        return vehicle_state_.pose;
    }

    // 原始定位位姿
    const common_msg::Pose& VehicleStateProvider::original_pose() const {
        return original_localization_.pose;
    }

    // 速度
    void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
        vehicle_state_.linear_velocity = linear_velocity;
    }

    // 完整车辆状态
    const common_msg::VehicleState& VehicleStateProvider::vehicle_state() const {
        return vehicle_state_;
    }

    // 时间戳
    double VehicleStateProvider::timestamp() const {
        return vehicle_state_.timestamp;
    }
}
}