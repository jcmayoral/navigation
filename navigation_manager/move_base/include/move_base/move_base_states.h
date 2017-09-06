#ifndef NAV_MOVE_BASE_STATE_H_
#define NAV_MOVE_BASE_STATE_H_

#include <vector>
#include <string>

#include <ros/ros.h>

namespace move_base {

  class MoveBaseState {
    public:

      enum States {
        PLANNING,
        CONTROLLING,
        CLEARING,
        RECOVERING
      };

      enum RecoveryTrigger {
        PLANNING_R,
        CONTROLLING_R,
        OSCILLATION_R
      };

    States state_;
    RecoveryTrigger recovery_trigger_;
  };
};

#endif
