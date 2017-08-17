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

    States state_;
  };
};

#endif
