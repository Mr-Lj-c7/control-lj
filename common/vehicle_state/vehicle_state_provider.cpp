#include "vehicle_state_provider.h"
#include <cmath>

namespace control {
namespace common {
namespace { constexpr double kDoubleEpsilon = 1e-6; }
    void VehicleStateProvider::Update(
        const common_msg::LocalizationEstimate& localization,
        const common_msg::Chassis& chassis){
        original_localization_ = localization;
        if (!ConstructExceptLinearVelocity(localization)) {
            std::cerr << "[VehicleStateProvider]:Fail to update because ConstructExpectLinearVelocity error." 
              << std::endl;
            return;
        } 
        // 时间戳  
        if (localization.measurement_time > kDoubleEpsilon) {
            vehicle_state_.timestamp = localization.measurement_time;
        } else if (localization.header.timestamp_sec > kDoubleEpsilon) {
            vehicle_state_.timestamp = localization.header.timestamp_sec;
        } else if (chassis.header.timestamp_sec > kDoubleEpsilon) {
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
        if (fabs(chassis.speed_mps) > kDoubleEpsilon) {
            vehicle_state_.linear_velocity = chassis.speed_mps;
            if (!FLAGS_reverse_heading_vehicle_state &&
                vehicle_state_.gear == common_msg::Chassis::GEAR_REVERSE) {
            vehicle_state_.linear_velocity = -vehicle_state_.linear_velocity;
            }
        }

        // 方向盘转角
        if (fabs(chassis.steering_percentage) > kDoubleEpsilon) {
            vehicle_state_.steering_percentage = chassis.steering_percentage;
        }

        // 车辆当前道路曲率
        static constexpr double kEpsilon = 0.1;
        if (std::fabs(vehicle_state_.linear_velocity) < kEpsilon) {
            vehicle_state_.kappa = 0.0;
        } else {
            vehicle_state_.kappa = (vehicle_state_.angular_velocity /
                                    vehicle_state_.linear_velocity);
        }

        // 驾驶模式
        vehicle_state_.driving_mode = chassis.driving_mode;
        std::cout << "[VehicleStateProvider]:Update vehicle state successfully." 
          << std::endl;
        return;
    }

    bool VehicleStateProvider::ConstructExceptLinearVelocity(
        const common_msg::LocalizationEstimate &localization) {
        if (fabs(localization.pose.position.x) < kDoubleEpsilon &&
            fabs(localization.pose.position.y) < kDoubleEpsilon &&
            fabs(localization.pose.position.z) < kDoubleEpsilon) {
            std::cerr << "[VehicleStateProvider]:Invalid localization input." 
              << std::endl;
            return false;
        }

        // skip localization update when it is in use_navigation_mode.
        if (FLAGS_use_navigation_mode) {
            std::cerr << "[VehicleStateProvider]:Skip localization update when it is in use_navigation_mode." 
                      << std::endl;
            return true;
        }
        // 车辆定位位姿
        vehicle_state_.pose = localization.pose;
        if (fabs(localization.pose.position.x) > kDoubleEpsilon ||
            fabs(localization.pose.position.y) > kDoubleEpsilon ||
            fabs(localization.pose.position.z) > kDoubleEpsilon) {
            vehicle_state_.x = localization.pose.position.x;
            vehicle_state_.y = localization.pose.position.y;
            vehicle_state_.z = localization.pose.position.z;
        }

        const auto &orientation = localization.pose.orientation;
        // 车辆航向角
        if (fabs(localization.pose.heading) > kDoubleEpsilon) {
            vehicle_state_.heading = localization.pose.heading;
        } else {
            vehicle_state_.heading = (
                common::QuaternionToHeading(orientation.qw, orientation.qx,
                                            orientation.qy, orientation.qz));
       }
        // 地图参考系统一致化
        if (FLAGS_enable_map_reference_unify) {
           if (fabs(localization.pose.angular_velocity_vrf.x) < kDoubleEpsilon &&
               fabs(localization.pose.angular_velocity_vrf.y) < kDoubleEpsilon &&
               fabs(localization.pose.angular_velocity_vrf.z) < kDoubleEpsilon) {
            std::cerr << "[VehicleStateProvider]:localization.pose().has_angular_velocity_vrf() must be true "
                        "when FLAGS_enable_map_reference_unify is true." 
                      << std::endl;
            return false;
            }
            vehicle_state_.angular_velocity = (  // 航向角速度
                localization.pose.angular_velocity_vrf.z);

            if (fabs(localization.pose.linear_acceleration_vrf.x) < kDoubleEpsilon &&
                fabs(localization.pose.linear_acceleration_vrf.y) < kDoubleEpsilon &&
                fabs(localization.pose.linear_acceleration_vrf.z) < kDoubleEpsilon) {
            std::cerr << "[VehicleStateProvider]:localization.pose().has_linear_acceleration_vrf() must be "
                        "true when FLAGS_enable_map_reference_unify is true." 
                      << std::endl;
            return false;
            }
            vehicle_state_.linear_acceleration = (  // 纵向加速度
                localization.pose.linear_acceleration_vrf.y);
        } else {
            if (fabs(localization.pose.angular_velocity.x) < kDoubleEpsilon &&
                fabs(localization.pose.angular_velocity.y) < kDoubleEpsilon &&
                fabs(localization.pose.angular_velocity.z) < kDoubleEpsilon) {
            std::cerr << "[VehicleStateProvider]:localization.pose() has no angular velocity." 
                      << std::endl;
            return false;
            }
            vehicle_state_.angular_velocity = (  // 航向角速度
                localization.pose.angular_velocity.z);

            if (fabs(localization.pose.linear_acceleration.x) < kDoubleEpsilon &&
                fabs(localization.pose.linear_acceleration.y) < kDoubleEpsilon &&
                fabs(localization.pose.linear_acceleration.z) < kDoubleEpsilon) {
            std::cerr << "[VehicleStateProvider]:localization.pose() has no linear acceleration." 
                      << std::endl;
            return false;
            }
            vehicle_state_.linear_acceleration = (  // 纵向加速度
                localization.pose.linear_acceleration.y);
        }
        // 车辆欧拉角
        if (fabs(localization.pose.euler_angles.x) > kDoubleEpsilon ||
            fabs(localization.pose.euler_angles.y) > kDoubleEpsilon || 
            fabs(localization.pose.euler_angles.z) > kDoubleEpsilon) {
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

    common::Vec2d VehicleStateProvider::ComputeCOMPosition(
      const double rear_to_com_distance) const {
        // set length as distance between rear wheel and center of mass.
        Eigen::Vector3d v;
        if ((FLAGS_state_transform_to_com_reverse &&
            vehicle_state_.gear == common_msg::Chassis::GEAR_REVERSE) ||
            (FLAGS_state_transform_to_com_drive &&
            vehicle_state_.gear == common_msg::Chassis::GEAR_DRIVE)) {
            v << 0.0, rear_to_com_distance, 0.0;
        } else {
            v << 0.0, 0.0, 0.0;
        }
        Eigen::Vector3d pos_vec(vehicle_state_.x, vehicle_state_.y,
                                vehicle_state_.z);
        // Initialize the COM position without rotation
        Eigen::Vector3d com_pos_3d = v + pos_vec;

        // If we have rotation information, take it into consideration.
        if (fabs(vehicle_state_.pose.orientation.qw) > kDoubleEpsilon ||
            fabs(vehicle_state_.pose.orientation.qx) > kDoubleEpsilon ||
            fabs(vehicle_state_.pose.orientation.qy) > kDoubleEpsilon ||
            fabs(vehicle_state_.pose.orientation.qz) > kDoubleEpsilon) {
            const auto &orientation = vehicle_state_.pose.orientation;
            Eigen::Quaternion<double> quaternion(orientation.qw, orientation.qx,
                                                orientation.qy, orientation.qz);
            // Update the COM position with rotation
            com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;
        }
        return common::Vec2d(com_pos_3d[0], com_pos_3d[1]);
    }

}
}