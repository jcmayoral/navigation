#include "collision_detector_diagnoser/collision_detector_diagnoser.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(collision_detector_diagnoser, CollisionDetectorDiagnoser,
                        collision_detector_diagnoser::CollisionDetectorDiagnoser,
                        fault_core::FaultDetector)

using namespace fault_core;
using namespace message_filters;
namespace collision_detector_diagnoser
{

  CollisionDetectorDiagnoser::CollisionDetectorDiagnoser(): isCollisionDetected(false), time_of_collision_()
  {
    ros::NodeHandle private_n;
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    strength_srv_client_ = private_n.serviceClient<kinetic_energy_monitor::KineticEnergyMonitorMsg>("kinetic_energy_drop");
    orientations_srv_client_ = private_n.serviceClient<footprint_checker::CollisionCheckerMsg>("collision_checker");
    ROS_INFO("Constructor CollisionDetectorDiagnoser");
  }

  CollisionDetectorDiagnoser::CollisionDetectorDiagnoser(int sensor_number): isCollisionDetected(false), time_of_collision_()
  {
    ros::NodeHandle private_n;
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    strength_srv_client_ = private_n.serviceClient<kinetic_energy_monitor::KineticEnergyMonitorMsg>("kinetic_energy_drop");
    orientations_srv_client_ = private_n.serviceClient<footprint_checker::CollisionCheckerMsg>("collision_checker");
    initialize(sensor_number);
    ROS_INFO("Constructor CollisionDetectorDiagnoser");
  }


  CollisionDetectorDiagnoser::~CollisionDetectorDiagnoser()
  {

  }

  fault_core::FaultTopology CollisionDetectorDiagnoser::getFault()
  {
     return fault_;
  }

  void CollisionDetectorDiagnoser::mainCallBack(const fusion_msgs::sensorFusionMsg msg){
    ROS_DEBUG_STREAM("Message received " << msg.window_size);
    if (msg.msg == fusion_msgs::sensorFusionMsg::ERROR){
      time_of_collision_ = msg.header;
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }

  void CollisionDetectorDiagnoser::initialize(int sensor_number)
  {
    ros::NodeHandle nh;
    ROS_INFO_STREAM("initializing " << sensor_number << " sensors");
    for (int i = 0; i< sensor_number;i++){
      ros::Subscriber sub = nh.subscribe("collisions_"+std::to_string(i), 10, &CollisionDetectorDiagnoser::mainCallBack, this);
      array_subcribers_.push_back(sub);
    }

  }

  bool CollisionDetectorDiagnoser::detectFault()
  {
    ROS_DEBUG("SimpleCollisionDetector Detect Fault");
    //if (isCollisionDetected){
    // isolateFault();
    //}
    return isCollisionDetected;
  }

  void CollisionDetectorDiagnoser::isolateFault(){

    footprint_checker::CollisionCheckerMsg srv;

    if(orientations_srv_client_.call(srv)){
      ROS_INFO("Orientations Computed Correctly");
    }
    else{
      ROS_WARN("Error in orientations Server");
    }

    fault_.type_ = FaultTopology::COLLISION;
    ROS_INFO("Isolating Platform Collision");
    diagnoseFault();
  }

  void CollisionDetectorDiagnoser::diagnoseFault(){
    //Force
    kinetic_energy_monitor::KineticEnergyMonitorMsg srv;
    srv.request.collision_time = time_of_collision_;

    if(strength_srv_client_.call(srv)){
      ROS_INFO_STREAM("Strength: " << srv.response.energy_lost);
    }

    fault_.cause_ = FaultTopology::MISLOCALIZATION;
    ROS_ERROR_ONCE("Collision FOUND");
  }
}  // namespace collision_detector_diagnoser
