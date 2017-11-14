#ifndef COLLISION_DETECTOR_DIAGNOSER_H
#define COLLISION_DETECTOR_DIAGNOSER_H

#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/AccelStamped.h>
#include <fusion_msgs/sensorFusionMsg.h>
#include <kinetic_energy_monitor/KineticEnergyMonitorMsg.h>
#include <footprint_checker/CollisionCheckerMsg.h>
#include <dynamic_reconfigure/server.h>
#include <collision_detector_diagnoser/dynamic_reconfigureConfig.h>
#include <collision_detector_diagnoser/sync_policies.h>
#include <collision_detector_diagnoser/sensor_fusion_methods.hpp>
#include <list>

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
      void twoSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                              const fusion_msgs::sensorFusionMsgConstPtr& detector_2);

      void threeSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_3);

      void simpleCallBack(const fusion_msgs::sensorFusionMsg msg);

      //void secondCallBack(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr&  msg2);
      //void thirdCallBack(const geometry_msgs::AccelStamped::ConstPtr& msg);
      void dyn_reconfigureCB(collision_detector_diagnoser::dynamic_reconfigureConfig &config, uint32_t level);

      void selectMode();

    private:
      std::vector<ros::Subscriber> array_subcribers_;
      bool isCollisionDetected;
      std_msgs::Header time_of_collision_;
      std::vector<message_filters::Subscriber<fusion_msgs::sensorFusionMsg>*> filtered_subscribers_;

      message_filters::Synchronizer<MySyncPolicy2>*sync_;

      ros::ServiceClient strength_srv_client_;
      ros::ServiceClient orientations_srv_client_;

      //Dynamic Reconfigure
      dynamic_reconfigure::Server<collision_detector_diagnoser::dynamic_reconfigureConfig> dyn_server;
      dynamic_reconfigure::Server<collision_detector_diagnoser::dynamic_reconfigureConfig>::CallbackType dyn_server_cb;

      int mode_;
      bool filter_;
      int sensor_number_;

      ConsensusApproach consensus_approach_;
      SensorFusionApproach default_approach_;
      SensorFusionApproach* fusion_approach_;
  };

}  // namespace collision_detector_diagnoser

#endif  // COLLISION_DETECTOR_DIAGNOSER_H
