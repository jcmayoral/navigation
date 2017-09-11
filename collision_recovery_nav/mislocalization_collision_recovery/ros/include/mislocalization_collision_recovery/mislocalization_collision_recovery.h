#ifndef SIMPLE_COLLISION_RECOVERY_H
#define SIMPLE_COLLISION_RECOVERY_H

#include <ros/ros.h>
#include <fault_core/fault_recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <std_srvs/Empty.h>

namespace mislocalization_collision_recovery
{

  class MisLocalizationCollisionRecovery : public fault_core::FaultRecoveryBehavior
  {
    public:

      /**
       * @brief Constructor
       */
      MisLocalizationCollisionRecovery();

      /**
       * @brief Destructor
       */
      ~MisLocalizationCollisionRecovery();

      /**
       * @brief Initializes plugin
       */
      void initialize(costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief Executes the car maneuver recovery behavior
       */
      bool runFaultBehavior();

    private:
      ros::ServiceClient client_;

  };

}  // namespace mislocalization_collision_recovery

#endif  // MISLOCALIZATION_COLLISION_RECOVERY_H
