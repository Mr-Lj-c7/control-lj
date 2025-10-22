#ifndef POI_MSG_H
#define POI_MSG_H

#include <string>
#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/routing_msgs/geometry_msg.h"


namespace control {
namespace common_msg {

enum ParkingSpaceType {
  VERTICAL_PLOT = 0,
  PARALLEL_PARKING = 1
};

typedef struct PARKING_INFO {
   std::string parking_space_id;
   PointENU parking_point;
   ParkingSpaceType parking_space_type;
  // The four corner points are in order.
   Polygon corner_point;
} ParkingInfo;

typedef struct LAND_MARK {
   std::string name;
   LaneWaypoint waypoint;
   std::string parking_space_id;
   ParkingInfo parking_info;
   int32_t cycle_number;
} Landmark;

typedef struct POI {
   Landmark landmark;
} POI;


}
}

#endif // POI_MSG_H