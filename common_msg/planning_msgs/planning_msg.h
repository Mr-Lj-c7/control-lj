#ifndef PLANNING_MSG_H
#define PLANNING_MSG_H 

#include "common_msg/basic_msgs/header_msg.h"
#include "common_msg/basic_msgs/pnc_point_msg.h"
#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/basic_msgs/drive_state_msg.h"
#include "common_msg/chassis_msgs/chassis_msg.h"
#include "common_msg/map_msgs/map_id_msg.h"
#include "common_msg/planning_msgs/decision_msg.h"
#include "common_msg/planning_msgs/planning_internal_msg.h"

namespace control {
namespace common_msg {

typedef struct ESTOP {
  // is_estop == true when emergency stop is required
   bool is_estop;
   std::string reason;
} EStop;

typedef struct TASK_STATS {
   std::string name;
   double time_ms;
} TaskStats;

typedef struct LATENCY_STATS {
   double total_time_ms;
   TaskStats task_stats;
   double init_frame_time_ms;
} PlanningLatencyStats;

typedef struct RSS_INFO {
   bool is_rss_safe;
   double cur_dist_lon;
   double rss_safe_dist_lon;
   double acc_lon_range_minimum;
   double acc_lon_range_maximum;
   double acc_lat_left_range_minimum;
   double acc_lat_left_range_maximum;
   double acc_lat_right_range_minimum;
   double acc_lat_right_range_maximum;
} RSSInfo;

typedef struct POINT {
   double x;
   double y; 
} Point;

typedef struct LOCATION_POSE {
  //vehice location pose
   Point vehice_location;

  // left width point of vehice location
   Point left_lane_boundary_point;

  // right width SL point of vehice location
   Point right_lane_boundary_point;
} LocationPose;

// next id: 24
typedef struct ADCTrajectory {
   Header header;

   double total_path_length;  // in meters

   double total_path_time;    // in seconds

   EStop estop;

   PlanningDebug debug;

  // is_replan == true mean replan triggered
   bool is_replan = false;

  // Specify trajectory gear
   Chassis::GearPosition gear;

  // path data + speed data
  std::vector<TrajectoryPoint> trajectory_point;

  // path point without speed info
  PathPoint path_point;

   DecisionResult decision;

   PlanningLatencyStats latency_stats;

  // the routing used for current planning result
   Header routing_header;

  enum RightOfWayStatus {
    UNPROTECTED = 0,
    PROTECTED = 1
  };

  RightOfWayStatus right_of_way_status;

  // lane id along current reference line
   Id lane_id;

  // set the engage advice for based on current planning result.
   EngageAdvice engage_advice;

  // the region where planning cares most
  typedef struct CRITICAL_REGION {
    Polygon region;
  } CriticalRegion;
  // critical region will be empty when planning is NOT sure which region is
  // critical
  // critical regions may or may not overlap
   CriticalRegion critical_region;

  enum TrajectoryType {
    UNKNOWN = 0,
    NORMAL = 1,
    PATH_FALLBACK = 2,
    SPEED_FALLBACK = 3,
    PATH_REUSED = 4,
    OPEN_SPACE = 5,
    EDGE_FOLLOW = 6,
    RELATIVE_CONTROL = 7
  };

   TrajectoryType trajectory_type = UNKNOWN;

   std::string replan_reason;

  // lane id along target reference line
   Id target_lane_id;

  // complete dead end flag
   bool car_in_dead_end;

  // vehicle location pose
   LocationPose location_pose;

  // in RELATIVE_CONTROL trajectory type, if trajectory is collisioned with obstable
   bool is_collision = false;

  // output related to RSS
   RSSInfo rss_info;

} ADCTrajectory;


} // common_msg
} // control

#endif // PLANNING_MSG_H