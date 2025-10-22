#ifndef VEHICLE_STATE_PROVIDER_H
#define VEHICLE_STATE_PROVIDER_H

#include <iostream>
#include "common/vehicle_state/msg/vehicle_state.h"
#include "common_msg/chassis_msgs/chassis_msg.h"
#include "common_msg/localization_msgs/localization_msg.h"
#include "common/math/euler_angles_zxy.h"

namespace control {
namespace common {

class VehicleStateProvider {
public:
    // 通过定位和底盘信息更新车辆状态
    void Update(const common_msg::LocalizationEstimate& localization,
                    const common_msg::Chassis& chassis);

    double timestamp() const;

    // 车辆定位位姿
    const common_msg::Pose& pose() const;
    const common_msg::Pose& original_pose() const;

    VehicleStateProvider() = default;
    virtual ~VehicleStateProvider() = default;

    // 车辆状态查询接口
    double x() const;
    double y() const;
    double z() const;
    double roll() const;
    double pitch() const;
    double yaw() const;
    double heading() const;
    double kappa() const;
    double linear_velocity() const;
    double angular_velocity() const;
    double linear_acceleration() const;
    double gear() const;
    double steering_percentage() const;

    // 设置线速度
    void set_linear_velocity(const double linear_velocity);

    // 获取完整车辆状态
    const common_msg::VehicleState& vehicle_state() const;

private:
    // 构建除线速度以外的车辆状态
    bool ConstructExceptLinearVelocity(
      const common_msg::LocalizationEstimate& localization);

    // 车辆状态,定位数据
    common_msg::VehicleState vehicle_state_;
    common_msg::LocalizationEstimate original_localization_;

    // 是否启用导航模式
    bool FLAGS_use_navigation_mode = false;
    bool FLAGS_enable_map_reference_unify = false;    // 地图参考系统一致化
    bool FLAGS_reverse_heading_vehicle_state = false; // 反向航向角（倒车）

};

}  // common
}  // control
#endif