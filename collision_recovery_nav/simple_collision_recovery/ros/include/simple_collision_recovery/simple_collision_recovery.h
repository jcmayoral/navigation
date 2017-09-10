#ifndef SIMPLE_COLLISION_RECOVERY_H
#define SIMPLE_COLLISION_RECOVERY_H

#include <ros/ros.h>
#include <fault_core/fault_recovery_behavior.h>
#include <base_local_planner/costmap_model.h>

namespace simple_collision_recovery
{

  class SimpleCollisionRecovery : public fault_core::FaultRecoveryBehavior
  {
    public:

      /**
       * @brief Constructor
       */
      SimpleCollisionRecovery();

      /**
       * @brief Destructor
       */
      ~SimpleCollisionRecovery();

      /**
       * @brief Initializes plugin
       */
      void initialize();

      /**
       * @brief Executes the car maneuver recovery behavior
       */
      void runFaultBehavior();

  };

}  // namespace simple_collision_recovery

#endif  // SIMPLE_COLLISION_RECOVERY_H
