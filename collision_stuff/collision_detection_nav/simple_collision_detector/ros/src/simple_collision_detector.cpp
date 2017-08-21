#include "simple_collision_detector/simple_collision_detector.h"
#include <pluginlib/class_list_macros.h>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(simple_collision_detector, SimpleCollisionDetector,
                        simple_collision_detector::SimpleCollisionDetector,
                        fault_core::FaultDetector)

namespace simple_collision_detector
{

  SimpleCollisionDetector::SimpleCollisionDetector()
  {
    ROS_INFO("Constructor SimpleCollisionDetector");
  }


  SimpleCollisionDetector::~SimpleCollisionDetector()
  {

  }


  void SimpleCollisionDetector::initialize(std::string name)
  {

  }

  bool SimpleCollisionDetector::detectFault()
  {
    ROS_DEBUG("SimpleCollisionDetector Detect Fault");
    return false;
  }

}  // namespace simple_collision_detector
