#ifndef ROUTING_MSG_H
#define ROUTING_MSG_H 
#include <string>
#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/basic_msgs/header_msg.h"
#include "common_msg/routing_msgs/poi_msg.h"
#include "common_msg/routing_msgs/geometry_msg.h"


namespace control {
namespace common_msg {

typedef struct ROUTING_REQUEST {
   Header header;
  // at least two points. The first is start point, the end is final point.
  // The routing must go through each point in waypoint.
   LaneWaypoint waypoint;
   LaneSegment blacklisted_lane;
   std::string blacklisted_road;
   bool broadcast = true;
   ParkingInfo parking_info;
  // If the start pose is set as the first point of "way_point".
   bool is_start_pose_set = false;
}RoutingRequest;

typedef struct ROUTING_RESPONSE {
   Header header;
   RoadSegment road;
   Measurement measurement;
   RoutingRequest routing_request;

  // the map version which is used to build road graph
   double map_version;
//    StatusPb status;
} RoutingResponse;

} // common_msg
} // control

#endif// ROUTING_MSG_H