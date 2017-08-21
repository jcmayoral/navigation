#ifndef FAULT_TOPOLOGY_H
#define FAULT_TOPOLOGY_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

namespace fault_core {

  class FaultTopology {
    public:

      enum FaultType {
        MINOR_COLLISION,
        MAJOR_COLLISION,
        SENSORFAULT,
        UNKNOWN_TYPE
      };

      enum FaultCause {
        MISLOCALIZATION,
        DYNAMIC_OBSTACLE,
        MAP_UNACCURACY,
        UNKNOWN
      };

    FaultType type_;
    FaultCause cause_;
    geometry_msgs::Quaternion orientation_;
    double strength;
  };
};

#endif
