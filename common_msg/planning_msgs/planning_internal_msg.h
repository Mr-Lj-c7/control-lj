#ifndef PLANNING_INTERNAL_MSG_H
#define PLANNING_INTERNAL_MSG_H

#include "common_msg/basic_msgs/header_msg.h"
#include "common_msg/basic_msgs/pnc_point_msg.h"
#include "common_msg/basic_msgs/geometry_msg.h"
#include "common_msg/chassis_msgs/chassis_msg.h"
#include "common_msg/localization_msgs/localization_msg.h"
// #include "common_msg/chassis_msgs/chassis_msg.h"
#include "common_msg/routing_msgs/geometry_msg.h"
#include "common_msg/routing_msgs/routing_msg.h"
#include "common_msg/routing_msgs/geometry_msg.h"
#include "common_msg/planning_msgs/decision_msg.h"
#include "common_msg/planning_msgs/sl_boundary_msg.h"
#include "common_msg/planning_msgs/navigation_msg.h"


namespace control {
namespace common_msg {

typedef struct SPEED_PLAN {
   std::string name ;
   SpeedPoint speed_point;
} SpeedPlan; 

typedef struct REFERENCE_LINE_DEBUG {
   std::string id ;
   double length ;
   double cost ;
   bool is_change_lane_path ;
   bool is_drivable ;
   bool is_protected ;
   bool is_offroad ;
   double minimum_boundary ;
   double kappa_rms ;
   double dkappa_rms ;
   double kappa_max_abs ;
   double dkappa_max_abs ;
   double average_offset ;
} ReferenceLineDebug;


// next ID: 30
typedef struct PLANNING_DATA {
  // input
   LocalizationEstimate adc_position;
   Chassis chassis;
   RoutingResponse routing;
   TrajectoryPoint init_point;

   Path path;

   SpeedPlan speed_plan;
//    STGraphDebug st_graph;
//    SLFrameDebug sl_frame;

   Header prediction_header;
//    SignalLightDebug signal_light;

//    ObstacleDebug obstacle;
   ReferenceLineDebug reference_line;
//    DpPolyGraphDebug dp_poly_graph;
//    LatticeStTraining lattice_st_image;
//    apollo.relative_map.MapMsg relative_map;
//    AutoTuningTrainingData auto_tuning_training_data ;
   double front_clear_distance;
//    apollo.dreamview.Chart chart;
//    ScenarioDebug scenario;
//    OpenSpaceDebug open_space;
//    SmootherDebug smoother;
//    PullOverDebug pull_over;
//    HybridModelDebug hybrid_model;
} PlanningData;

typedef struct PLANNING_DEBUG {
   PlanningData planning_data;
} PlanningDebug;

} // common_msg
} // control
#endif // PLANNING_INTERNAL_MSG_H