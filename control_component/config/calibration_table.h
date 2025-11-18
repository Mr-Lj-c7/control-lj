#ifndef CALIBRATION_TABLE_H
#define CALIBRATION_TABLE_H

#include <string>
#include <vector>

namespace control {
namespace common_msg {

    typedef struct CONTROL_CALIBRATION_INFO
    {
        double speed;
        double acceleration;
        double command;
    } ControlCalibrationInfo;


    typedef struct CALIBRATION_TABLE { 
        std::vector<ControlCalibrationInfo> calibration;
    } calibration_table;
} // control
} // common_msg


#endif