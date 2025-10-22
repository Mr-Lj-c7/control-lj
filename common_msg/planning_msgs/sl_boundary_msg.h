#ifndef SL_BOUNDARY_MSG_H
#define SL_BOUNDARY_MSG_H

#include "common_msg/basic_msgs/pnc_point_msg.h"

namespace control{
namespace common_msg {

typedef struct SLBoundary {
   double start_s;
   double end_s;
   double start_l;
   double end_l ;
   SLPoint boundary_point;
} SLBoundary;


    
} // namespace control
} // namespace common_msg


#endif // SL_BOUNDARY_MSG_H