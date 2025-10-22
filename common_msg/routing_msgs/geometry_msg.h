#ifndef ROUTING_GEOMETRY_MSG_H
#define ROUTING_GEOMETRY_MSG_H

#include <string>
#include "common_msg/basic_msgs/geometry_msg.h"



namespace control {
namespace common_msg {

typedef struct LANE_WAY_POINT {
   std::string id;
   double s;
   PointENU pose;
  // When the developer selects a point on the dreamview route editing
  // the direction can be specified by dragging the mouse
  // dreamview calculates the heading based on this to support construct lane way point with heading
   double heading = 4;
} LaneWaypoint;

typedef struct LANE_SEGMENT {
   std::string id;
   double start_s;
   double end_s;
} LaneSegment;


enum DeadEndRoutingType {
  ROUTING_OTHER = 0,
  ROUTING_IN = 1,
  ROUTING_OUT = 2
};

typedef struct MEASUREMENT {
   double distance;
} Measurement;

enum ChangeLaneType {
  FORWARD = 0,
  LEFT = 1,
  RIGHT = 2
};

typedef struct PASSAGE {
   LaneSegment segment;
   bool can_exit;
   ChangeLaneType change_lane_type = ChangeLaneType::FORWARD;
} Passage;

typedef struct ROAD_SEGMENT {
   std::string id;
   Passage passage;
} RoadSegment;

} // common_msg
} // control

#endif