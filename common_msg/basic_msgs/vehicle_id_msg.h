#ifndef VEHICLE_ID_MSG_H
#define VEHICLE_ID_MSG_H

#include <string>
#include <cmath>
#include <vector>

namespace control {
namespace common_msg {

typedef struct VEHICLE_ID
{
  std::string vin;
  std::string plate;
  std::string other_unique_id;
} VehicleID;

}  // namespace common_msg
}  // namespace control

#endif