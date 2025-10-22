#ifndef POSE_MSG_H
#define POSE_MSG_H

#include "common_msg/basic_msgs/geometry_msg.h"

namespace control {
namespace common_msg {

typedef struct POSE {
    PointENU position;

    Quaternion orientation;

    Point3D linear_velocity;

    Point3D linear_acceleration;

    Point3D angular_velocity;

    double heading;

    Point3D linear_acceleration_vrf;

    Point3D angular_velocity_vrf;
    
    Point3D euler_angles;
} Pose;

}  // namespace common_msg
}  // namespace control
#endif