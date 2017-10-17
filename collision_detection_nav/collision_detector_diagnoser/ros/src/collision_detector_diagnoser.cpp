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

  CollisionDetectorDiagnoser::CollisionDetectorDiagnoser(): isCollisionDetected(false)
  {
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
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
    if (isCollisionDetected){
      isolateFault();
    }
    return isCollisionDetected;
  }

  void CollisionDetectorDiagnoser::isolateFault(){
    fault_.type_ = FaultTopology::COLLISION;
    ROS_INFO("Isolating Platform Collision");
    diagnoseFault();
  }

  void CollisionDetectorDiagnoser::diagnoseFault(){
    fault_.cause_ = FaultTopology::MISLOCALIZATION;
    ROS_ERROR_ONCE("Collision FOUND");
  }
}  // namespace collision_detector_diagnoser
