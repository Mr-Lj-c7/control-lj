#ifndef NAVIGATION_MSG_H
#define NAVIGATION_MSG_H

#include "common_msg/basic_msgs/header_msg.h"
#include "common_msg/basic_msgs/pnc_point_msg.h"
#include "common_msg/localization_msgs/localization_msg.h"
// #include "common_msg/map_msgs/map_msg.h"

namespace control {
namespace common_msg {

typedef struct NAVIGATION_PATH {
   Path path;
  // highest = 0 which can directly reach destination; change lane indicator
   uint32_t path_priority;
} NavigationPath;

typedef struct NavigationInfo {
   Header header;
   NavigationPath navigation_path;
} NavigationInfo;

// The map message in transmission format.
typedef struct MAP_MSG {
   Header header;

  // Coordination: FLU
  // x: Forward
  // y: Left
  // z: Up
//    apollo.hdmap.Map hdmap = 2;

  // key: type string; the lane id in Map
  // value: Navigation path; the reference line of the lane
//   map<string, NavigationPath> navigation_path;

  // lane marker info from perception
//    apollo.perception.LaneMarkers lane_marker = 4;

  // localization
   LocalizationEstimate localization;
} MapMsg;


} // common_msg
} // control
#endif // NAVIGATION_MSG_H