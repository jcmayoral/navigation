#include "simple_collision_detector/simple_collision_detector.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(simple_collision_detector, SimpleCollisionDetector,
                        simple_collision_detector::SimpleCollisionDetector,
                        fault_core::FaultDetector)

using namespace fault_core;

namespace simple_collision_detector
{

  SimpleCollisionDetector::SimpleCollisionDetector()
  {
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    ROS_INFO("Constructor SimpleCollisionDetector");
  }


  SimpleCollisionDetector::~SimpleCollisionDetector()
  {

  }

  void SimpleCollisionDetector::mainCallBack(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Message received " << msg);
  }

  void SimpleCollisionDetector::initialize(int sensor_number)
  {
    ros::NodeHandle nh;
    ROS_INFO_STREAM("initializing " << sensor_number << " sensors");
    for (int i = 0; i< sensor_number;i++){
      ros::Subscriber sub = nh.subscribe("collisions_"+std::to_string(i), 10, &SimpleCollisionDetector::mainCallBack, this);
      array_subcribers_.push_back(sub);
    }
  }

  bool SimpleCollisionDetector::detectFault()
  {
    ROS_DEBUG("SimpleCollisionDetector Detect Fault");
    diagnoseFault();
    return false;
  }

  void SimpleCollisionDetector::diagnoseFault(){
    fault_.cause_ = FaultTopology::UNKNOWN;
    fault_.type_ = FaultTopology::COLLISION;
  }
}  // namespace simple_collision_detector
