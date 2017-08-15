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


  void SimpleCollisionDetector::initialize(std::string name,
    tf::TransformListener* tfListener,
    costmap_2d::Costmap2DROS* globalCostmapROS,
    costmap_2d::Costmap2DROS* localCostmapROS)
  {

  }

  void SimpleCollisionDetector::detectFault()
  {
  }

}  // namespace simple_collision_detector
