#ifndef PID_CONF_H
#define PID_CONF_H 
#include <string>
#include <cmath>
#include <vector>

namespace control {
namespace common_msg {

// 超前滞后控制器参数
typedef struct PID_CONF { 
   bool integrator_enable;
   double integrator_saturation_level;
   double kp;
   double ki;
   double kd;
   double kaw = 0.0;
   double output_saturation_level;
} PidConf;

}  // common_msg
}  // control

#endif