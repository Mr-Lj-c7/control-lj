#ifndef LEADLAG_CONF_H
#define LEADLAG_CONF_H 
#include <string>
#include <cmath>
#include <vector>

namespace control {
namespace common_msg {

// 超前滞后控制器参数
typedef struct LEADLAG_CONF { 
    double innerstate_saturation_level;
    double alpha;
    double beta;
    double tau;
    void reset(){
        innerstate_saturation_level = 300;
        alpha = 0.1;
        beta = 1.0;
        tau = 0.0;
    }
} LeadlagConf;

}  // common_msg
}  // control

#endif