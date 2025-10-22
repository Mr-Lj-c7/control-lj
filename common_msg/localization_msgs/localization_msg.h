#ifndef LOCALIZATION_MSG_H
#define LOCALIZATION_MSG_H

#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/basic_msgs/header_msg.h"
#include "common_msg/basic_msgs/pnc_point_msg.h"
#include "pose_msg.h"
#include "localization_status_msg.h"

namespace control {
namespace common_msg {

typedef struct UNCERTAINITY {
  // Standard deviation of position, east/north/up in meters.
  Point3D position_std_dev;

  // Standard deviation of quaternion qx/qy/qz, unitless.
  Point3D orientation_std_dev;

  // Standard deviation of linear velocity, east/north/up in meters per second.
  Point3D linear_velocity_std_dev;

  // Standard deviation of linear acceleration, right/forward/up in meters per
  // square second.
  Point3D linear_acceleration_std_dev;

  // Standard deviation of angular velocity, right/forward/up in radians per
  // second.
  Point3D angular_velocity_std_dev;

  // TODO: Define covariance items when needed.
} Uncertainty;

typedef struct LOCALIZATION_ESTIMATE {
   Header header;
   Pose pose;
   Uncertainty uncertainty;

  // The time of pose measurement, seconds since 1970-1-1 (UNIX time).
  double measurement_time;  // In seconds.

  // Future trajectory actually driven by the drivers
  TrajectoryPoint trajectory_point;

  // msf status
   MsfStatus msf_status;
  // msf quality
   MsfSensorMsgStatus sensor_status;
} LocalizationEstimate;


enum MeasureState {
  OK = 0,
  WARNNING = 1,
  ERROR = 2,
  CRITICAL_ERROR = 3,
  FATAL_ERROR = 4
};

typedef struct LOCALIZATION_STATUS {
   Header header;
   MeasureState fusion_status;
   MeasureState gnss_status;
   MeasureState lidar_status;
  // The time of pose measurement, seconds since 1970-1-1 (UNIX time).
   double measurement_time;  // In seconds.
   std::string state_message;
} LocalizationStatus;

}  // namespace common_msg
}  // namespace control
#endif