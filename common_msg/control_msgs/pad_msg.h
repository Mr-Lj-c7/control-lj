#ifndef PAD_MSG_H
#define PAD_MSG_H

#include "common_msg/basic_msgs/header_msg.h"

namespace control {
namespace common_msg {

    enum DrivingAction { 
        START = 1,   // 启动自动驾驶
        RESET = 2,   // 重置控制系统
        VIN_REQ = 3  // 请求车辆识别
    };

    typedef struct PADMESSAGE {
        Header header;
        DrivingAction action;
    } PadMessage;

} //control
} // common_msg

#endif