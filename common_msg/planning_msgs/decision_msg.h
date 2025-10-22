#ifndef DECISION_MSG_H
#define DECISION_MSG_H 

#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/basic_msgs/vehicle_signal_msg.h"
#include "common_msg/routing_msgs/geometry_msg.h"
#include <string>

namespace control {
namespace common_msg {

// dodge the obstacle in lateral direction when driving
typedef struct OBJECT_NUDGE {
  enum Type {
    LEFT_NUDGE = 1,   // drive from the left side to nudge a static obstacle
    RIGHT_NUDGE = 2,  // drive from the right side to nudge a static obstacle
    DYNAMIC_LEFT_NUDGE = 3,   // drive from the left side to nudge a dynamic obstacle
    DYNAMIC_RIGHT_NUDGE = 4  // drive from the right side to nudge a dynamic obstacle
  };
   Type type;
  // minimum lateral distance in meters. positive if type = LEFT_NUDGE
  // negative if type = RIGHT_NUDGE
   double distance_l;
} ObjectNudge;

typedef struct OBJECT_YIELD {
   double distance_s;  // minimum longitudinal distance in meters
   PointENU fence_point;
   double fence_heading;
   double time_buffer;  // minimum time buffer required after the
                                    // obstacle reaches the intersect point.
} ObjectYield;

typedef struct OBJECT_FOLLOW {
   double distance_s;  // minimum longitudinal distance in meters
   PointENU fence_point;
   double  fence_heading;
} ObjectFollow;

typedef struct OBJECT_OVER_TAKE  {
   double distance_s;  // minimum longitudinal distance in meters
   PointENU fence_point;
   double fence_heading;
   double time_buffer;  // minimum time buffer required before the
                                    // obstacle reaches the intersect point.
} ObjectOvertake;

typedef struct OBJECT_SIDE_PASS {
  enum Type {
    LEFT = 1,
    RIGHT = 2
  };
   Type type;
} ObjectSidePass;

// unified object decision while estop
typedef struct OBJECT_AVOID {} ObjectAvoid;


typedef struct OBJECT_STATIC {} ObjectStatic;

typedef struct OBJECT_DYNAMIC {} ObjectDynamic;


typedef struct OBJECT_MOTION_TYPE {
  typedef struct MOTION_TAG {
    ObjectStatic static_;
    ObjectDynamic dynamic;
  } motion_tag;
} ObjectMotionType;

typedef struct OBJECT_IGNORE {} ObjectIgnore;

enum StopReasonCode {
  STOP_REASON_HEAD_VEHICLE = 1,
  STOP_REASON_DESTINATION = 2,
  STOP_REASON_PEDESTRIAN = 3,
  STOP_REASON_OBSTACLE = 4,
  STOP_REASON_PREPARKING = 5,
  STOP_REASON_SIGNAL = 100,  // only for red signal
  STOP_REASON_STOP_SIGN = 101,
  STOP_REASON_YIELD_SIGN = 102,
  STOP_REASON_CLEAR_ZONE = 103,
  STOP_REASON_CROSSWALK = 104,
  STOP_REASON_CREEPER = 105,
  STOP_REASON_REFERENCE_END = 106,  // end of the reference_line
  STOP_REASON_YELLOW_SIGNAL = 107,  // yellow signal
  STOP_REASON_PULL_OVER = 108,      // pull over
  STOP_REASON_SIDEPASS_SAFETY = 109,
  STOP_REASON_PRE_OPEN_SPACE_STOP = 200,
  STOP_REASON_LANE_CHANGE_URGENCY = 201,
  STOP_REASON_EMERGENCY = 202
};

typedef struct OBJECT_STOP {
   StopReasonCode reason_code;
   double distance_s;  // in meters
  // When stopped, the front center of vehicle should be at this point.
   PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
   double stop_heading = 4;
  std::string wait_for_obstacle;
} ObjectStop;

typedef struct OBJECT_DECISION_TYPE {
  typedef struct OBJECT_TAG {
    ObjectIgnore ignore;
    ObjectStop stop;
    ObjectFollow follow;
    ObjectYield yield;
    ObjectOvertake overtake;
    ObjectNudge nudge;
    ObjectAvoid avoid;
    ObjectSidePass side_pass;
  } object_tag;
} ObjectDecisionType;

typedef struct OBJECT_STATUS {
   ObjectMotionType motion_type;
   ObjectDecisionType decision_type;
} ObjectStatus;







typedef struct Objec_tDecision {
   std::string id;
   int32_t perception_id;
   ObjectDecisionType object_decision;
} ObjectDecision;

typedef struct OBJECT_DECISIONS {
   ObjectDecision decision;
} ObjectDecisions;

typedef struct MAIN_CRUISE {
  // cruise current lane
  ChangeLaneType change_lane_type;
} MainCruise;

typedef struct MAIN_STOP {
   StopReasonCode reason_code;
   std::string reason;
  // When stopped, the front center of vehicle should be at this point.
   PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
   double stop_heading;
   ChangeLaneType change_lane_type;
} MainStop;

typedef struct EMERGENCY_STOP_HARDBRAKE {} EmergencyStopHardBrake;

typedef struct EMERGENCY_STOPCRUISE_TO_STOP {} EmergencyStopCruiseToStop;

typedef struct MAIN_EMERGENCY_STOP {
  // Unexpected event happened, human driver is required to take over
  enum ReasonCode {
    ESTOP_REASON_INTERNAL_ERR = 1,
    ESTOP_REASON_COLLISION = 2,
    ESTOP_REASON_ST_FIND_PATH = 3,
    ESTOP_REASON_ST_MAKE_DECISION = 4,
    ESTOP_REASON_SENSOR_ERROR = 5
  };
   ReasonCode reason_code;
   std::string reason;
  typedef struct TASK {
    EmergencyStopHardBrake hard_brake;         // hard brake
  } task;
} MainEmergencyStop;

typedef struct MAIN_MISSION_COMPLETE {
  // arrived at routing destination
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading;
} MainMissionComplete;

typedef struct MainNotReady {
  // decision system is not ready.
  // e.g. wait for routing data.
  std::string reason;
} MainNotReady;

typedef struct MainParking {
  enum ParkingStatus {
    // TODO(QiL): implement and expand to more enums
    IN_PARKING = 1
  };
   ParkingStatus status;
} MainParking;

typedef struct MAIN_DECISION {
  typedef struct TASK {
    MainCruise cruise;
    MainStop stop;
    MainEmergencyStop estop;
    MainMissionComplete mission_complete;
    MainNotReady not_ready;
    MainParking parking;
  } task;
} MainDecision;

typedef struct DECISION_RESULT {
   MainDecision main_decision;
   ObjectDecisions object_decision;
   VehicleSignal vehicle_signal;
} DecisionResult;

} // common_msg
} // control

#endif // DECISION_MSG_H