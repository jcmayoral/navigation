#include "simple_collision_detector/simple_collision_detector.h"
#include <pluginlib/class_list_macros.h>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(simple_collision_detector, SimpleCollisionDetector,
                        simple_collision_detector::SimpleCollisionDetector,
                        fault_core::FaultDetector)

using namespace fault_core;

namespace simple_collision_detector
{

  SimpleCollisionDetector::SimpleCollisionDetector()
  {
    fault_.type_ =  fault_core::FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = fault_core::FaultTopology::UNKNOWN;
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
    diagnoseFault();
    return false;
  }

  void SimpleCollisionDetector::diagnoseFault(){
    fault_.cause_ = FaultTopology::UNKNOWN;
    fault_.type_ = FaultTopology::COLLISION;
  }
}  // namespace simple_collision_detector
