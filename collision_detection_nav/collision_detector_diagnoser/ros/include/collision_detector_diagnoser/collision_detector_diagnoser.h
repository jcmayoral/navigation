#ifndef COLLISION_DETECTOR_DIAGNOSER_H
#define COLLISION_DETECTOR_DIAGNOSER_H

#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/AccelStamped.h>
#include <fusion_msgs/sensorFusionMsg.h>
#include <kinetic_energy_monitor/KineticEnergyMonitorMsg.h>
#include <footprint_checker/CollisionCheckerMsg.h>
#include <dynamic_reconfigure/server.h>
#include <collision_detector_diagnoser/dynamic_reconfigureConfig.h>


namespace collision_detector_diagnoser
{

  class CollisionDetectorDiagnoser : public fault_core::FaultDetector
  {
    public:

      /**
       * @brief Constructor
       */
      CollisionDetectorDiagnoser();
      //Node
      CollisionDetectorDiagnoser(int sensor_number);


      /**
       * @brief Destructor
       */
      ~CollisionDetectorDiagnoser();

      /**
       * @brief Initializes plugin
       * @param number of sensors
       */
      void initialize(int sensor_number);

      /**
       * @brief Executes the detection of the fault
       */
      bool detectFault();

      /**
       * @brief Executes the isolation of the fault
       */
      void isolateFault();

      /**
       * @brief   diagnose Fault
       */
      void diagnoseFault();

      /**
       * @brief   detect Fault
       */
      fault_core::FaultTopology getFault();

      //void mainCallBack(const fusion_msgs::sensorFusionMsg msg);
      void mainCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1, const fusion_msgs::sensorFusionMsgConstPtr& detector_2);
      void simpleCallBack(const fusion_msgs::sensorFusionMsg msg);

      //void secondCallBack(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr&  msg2);
      //void thirdCallBack(const geometry_msgs::AccelStamped::ConstPtr& msg);
      void dyn_reconfigureCB(collision_detector_diagnoser::dynamic_reconfigureConfig &config, uint32_t level);

    private:
      std::vector<ros::Subscriber> array_subcribers_;
      bool isCollisionDetected;
      std_msgs::Header time_of_collision_;
      message_filters::Subscriber<fusion_msgs::sensorFusionMsg> *sub_0_;
      message_filters::Subscriber<fusion_msgs::sensorFusionMsg> *sub_1_;
      typedef message_filters::sync_policies::ApproximateTime<fusion_msgs::sensorFusionMsg, fusion_msgs::sensorFusionMsg> MySyncPolicy;
      message_filters::Synchronizer<MySyncPolicy>*sync_;

      ros::ServiceClient strength_srv_client_;
      ros::ServiceClient orientations_srv_client_;

      //Dynamic Reconfigure
      dynamic_reconfigure::Server<collision_detector_diagnoser::dynamic_reconfigureConfig> dyn_server;
      dynamic_reconfigure::Server<collision_detector_diagnoser::dynamic_reconfigureConfig>::CallbackType dyn_server_cb;
      int mode_;
      int sensor_number_;
  };

}  // namespace collision_detector_diagnoser

#endif  // COLLISION_DETECTOR_DIAGNOSER_H
