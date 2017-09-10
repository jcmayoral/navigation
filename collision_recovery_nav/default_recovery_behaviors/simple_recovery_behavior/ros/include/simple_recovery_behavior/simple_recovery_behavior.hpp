#ifndef SIMPLE_RECOVERY_BEHAVIOR_H
#define SIMPLE_RECOVERY_BEHAVIOR_H

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>

namespace simple_recovery_behavior
{

  class SimpleRecoveryBehavior : public nav_core::RecoveryBehavior
  {
    public:

      /**
       * @brief Constructor
       */
      SimpleRecoveryBehavior();

      /**
       * @brief Destructor
       */
      ~SimpleRecoveryBehavior();

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
       * @brief Executes the recovery behavior
       */
      void runBehavior();

  };

}  // namespace simple_recovery_behavior

#endif  // SIMPLE_RECOVERY_BEHAVIOR_H
