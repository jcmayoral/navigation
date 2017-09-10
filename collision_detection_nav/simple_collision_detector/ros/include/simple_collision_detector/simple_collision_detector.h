#ifndef SIMPLE_COLLISION_DETECTOR_H
#define SIMPLE_COLLISION_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
#include <base_local_planner/costmap_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/AccelStamped.h>
#include <fusion_msgs/sensorFusionMsg.h>

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
      
      /**
       * @brief   detect Fault
       */
      fault_core::FaultTopology getFault();

      void mainCallBack(const fusion_msgs::sensorFusionMsg msg);
      //void secondCallBack(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr&  msg2);
      //void thirdCallBack(const geometry_msgs::AccelStamped::ConstPtr& msg);

    private:
      std::vector<ros::Subscriber> array_subcribers_;
      bool isCollisionDetected;
  };

}  // namespace simple_collision_detector

#endif  // SIMPLE_COLLISION_DETECTOR_H
