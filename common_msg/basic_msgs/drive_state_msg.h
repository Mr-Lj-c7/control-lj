#ifndef DRIVE_STATE_MSG_H
#define DRIVE_STATE_MSG_H 

#include <string>
namespace control {
namespace common_msg {

// This is the engage advice that published by critical runtime modules.
typedef struct ENGAGE_ADVICE {
  enum Advice {
    UNKNOWN = 0,
    DISALLOW_ENGAGE = 1,
    READY_TO_ENGAGE = 2,
    KEEP_ENGAGED = 3,
    PREPARE_DISENGAGE = 4
  };

   Advice advice = DISALLOW_ENGAGE;
   std::string reason;
} EngageAdvice;

}  // namespace common_msg
}  // namespace control
#endif