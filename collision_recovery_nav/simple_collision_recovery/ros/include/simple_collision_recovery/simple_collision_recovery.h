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
       * @param name: name of instance
       * @param tfListener: ptr to the tf transform listener of the node
       * @param globalCostmapROS: ptr to the global costmap of the node
       * @param localCostmapROS: ptr to the local costmap of the node
       */
      void initialize(std::string name, tf::TransformListener* tfListener,
        costmap_2d::Costmap2DROS* globalCostmapROS,
        costmap_2d::Costmap2DROS* localCostmapROS);

      /**
       * @brief Executes the car maneuver recovery behavior
       */
      void runFaultBehavior();

  };

}  // namespace simple_collision_recovery

#endif  // SIMPLE_COLLISION_RECOVERY_H
