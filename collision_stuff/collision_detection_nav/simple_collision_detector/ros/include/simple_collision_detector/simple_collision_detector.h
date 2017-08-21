#ifndef SIMPLE_COLLISION_DETECTOR_H
#define SIMPLE_COLLISION_DETECTOR_H

#include <ros/ros.h>
#include <fault_core/fault_detector.h>
#include <base_local_planner/costmap_model.h>

namespace simple_collision_detector
{

  class SimpleCollisionDetector : public fault_core::FaultDetector
  {
    public:

      /**
       * @brief Constructor
       */
      SimpleCollisionDetector();

      /**
       * @brief Destructor
       */
      ~SimpleCollisionDetector();

      /**
       * @brief Initializes plugin
       * @param name: name of instance
       * @param tfListener: ptr to the tf transform listener of the node
       * @param globalCostmapROS: ptr to the global costmap of the node
       * @param localCostmapROS: ptr to the local costmap of the node
       */
      void initialize(std::string name);

      /**
       * @brief Executes the car maneuver recovery behavior
       */
      bool detectFault();

  };

}  // namespace simple_collision_detector

#endif  // SIMPLE_COLLISION_DETECTOR_H
