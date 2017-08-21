#ifndef SIMPLE_COLLISION_DETECTOR_H
#define SIMPLE_COLLISION_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
#include <base_local_planner/costmap_model.h>
#include <std_msgs/Float32.h>

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
       * @param number of sensors
       */
      void initialize(int sensor_number);

      /**
       * @brief Executes the detection of teh fault
       */
      bool detectFault();

      /**
       * @brief   diagnose Fault
       */
      void diagnoseFault();

      void mainCallBack(std_msgs::Float32 msg);

    private:
      std::vector<ros::Subscriber> array_subcribers_;
  };

}  // namespace simple_collision_detector

#endif  // SIMPLE_COLLISION_DETECTOR_H
