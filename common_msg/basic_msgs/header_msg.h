#ifndef HEADER_MSG_H
#define HEADER_MSG_H


#include <vector>
#include <string>
#include <cmath>

namespace control {
namespace common_msg { 
    
    // 消息定义
    typedef struct HEADER { 
        double timestamp_sec;
        std::string module_name;
        uint32_t sequence_num; 

        uint64_t camera_timestamp;
        uint64_t lidar_timestamp;
        uint64_t radar_timestamp;

        uint32_t version; 
        std::string frame_id; 
    } Header;

}  // common_msg
}  // control

#endif