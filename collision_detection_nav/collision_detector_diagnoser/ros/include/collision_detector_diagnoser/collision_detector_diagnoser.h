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
#include <collision_detector_diagnoser/diagnoserConfig.h>
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

      void fourSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_3,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_4);

      void fiveSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_3,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_4,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_5
                              );

      void simpleCallBack(const fusion_msgs::sensorFusionMsg msg);

      //void secondCallBack(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr&  msg2);
      //void thirdCallBack(const geometry_msgs::AccelStamped::ConstPtr& msg);
      void dyn_reconfigureCB(collision_detector_diagnoser::diagnoserConfig &config, uint32_t level);

      void selectMode();


      void plotOrientation(list<fusion_msgs::sensorFusionMsg> v);

    private:

      void unregisterCallbackForSyncronizers();
      void registerCallbackForSyncronizers(int sensor_number);
      void setUnfilteredPublishers(int sensor_number, ros::NodeHandle nh);
      void setFilteredPublishers(int sensor_number, ros::NodeHandle nh);
      void resetFilteredPublishers();
      void resetUnFilteredPublishers();

      std::vector<ros::Subscriber> array_subcribers_;
      bool isCollisionDetected;
      std_msgs::Header time_of_collision_;

      ros::Publisher speak_pub_;

      std::vector<message_filters::Subscriber<fusion_msgs::sensorFusionMsg>*> filtered_subscribers_;

      message_filters::Synchronizer<MySyncPolicy2>*syncronizer_for_two_;
      message_filters::Synchronizer<MySyncPolicy3>*syncronizer_for_three_;
      message_filters::Synchronizer<MySyncPolicy4>*syncronizer_for_four_;
      message_filters::Synchronizer<MySyncPolicy5>*syncronizer_for_five_;

      message_filters::Connection main_connection;

      ros::ServiceClient strength_srv_client_;
      ros::ServiceClient orientations_srv_client_;

      //Dynamic Reconfigure
      dynamic_reconfigure::Server<collision_detector_diagnoser::diagnoserConfig>* dyn_server;
      dynamic_reconfigure::Server<collision_detector_diagnoser::diagnoserConfig>::CallbackType dyn_server_cb;

      int mode_;
      bool filter_;
      int sensor_number_;
      double percentage_threshold_;

      SensorFusionApproach default_approach_;
      ConsensusApproach consensus_approach_;
      WeightedApproach weighted_approach_;
      KalmanFilterApproach kalman_approach_;

      SensorFusionApproach* fusion_approach_;

      //Collision Orientations from sensors
      ros::Publisher orientation_pub_;

  };

}  // namespace collision_detector_diagnoser

#endif  // COLLISION_DETECTOR_DIAGNOSER_H
